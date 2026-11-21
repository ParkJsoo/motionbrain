"""저장된 어깨 왕복의 OFF 전후 관측을 표시용 MuJoCo 모델과 대조한다."""
import argparse
import json
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/motionbrain_ros_bridge"))
from motionbrain_ros_bridge.motionbrain_kinematics import (  # noqa: E402
    ArmModel, JointAngles, forward_kinematics, inverse_kinematics,
)
sys.path.insert(0, str(ROOT / "tools/visualization"))
from update_pose import make_pose  # noqa: E402


def read_json(path):
    return json.loads(path.read_text())


def recorded_motion(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    first = rows[0]
    if first["path"] != "/action_outputs":
        raise ValueError("initial output history required")
    boot = first["response"]["bootId"]
    cursor = max(entry["sequence"] for entry in first["response"]["entries"])
    writes = {}
    samples = []
    for row in rows[1:]:
        data = row.get("response") or {}
        if row["path"].startswith("/action_outputs") and data:
            if data["bootId"] != boot:
                raise ValueError("clock domain changed")
            for entry in data["entries"]:
                if entry["sequence"] > cursor:
                    writes[entry["sequence"]] = entry
        elif row["path"] == "/status" and data:
            shoulder = data["shoulderAngle"]
            if shoulder["available"] and shoulder["sensorFresh"] and shoulder["sensorReady"]:
                # The firmware serializes uptime and I2C age separately.
                # Subtraction estimates the sampling time on the driver clock.
                sample = (data["uptimeMs"] - shoulder["ageMs"], shoulder["angleDeg"])
                if samples and sample == samples[-1]:
                    continue
                if samples and sample[0] <= samples[-1][0]:
                    raise ValueError("nonmonotonic or conflicting shoulder samples")
                samples.append(sample)
    segments = []
    start = None
    peak = 0
    for seq, entry in sorted(writes.items()):
        if seq != cursor + 1:
            raise ValueError("missing driver output records")
        cursor = seq
        if entry["motor"] != 4:
            if entry["pwm"]:
                raise ValueError("multi-axis episode")
            continue
        pwm, when = entry["pwm"], entry["atMs"]
        if pwm:
            if start is None:
                start = when
            if peak * pwm < 0:
                raise ValueError("sign reversal without OFF")
            if abs(pwm) > abs(peak):
                peak = pwm
        elif start is not None:
            segments.append((start, when, peak))
            start, peak = None, 0
    if start is not None:
        raise ValueError("missing final OFF")
    return samples, segments


def compare_stop(segment, next_start, samples, seed, anchor, model_path, geometry):
    start, stop, peak = segment
    pre = [s for s in samples if start <= s[0] < stop]
    first = max(s for s in samples if s[0] < start)
    post = min(s for s in samples if stop <= s[0] < next_start)
    prev, last = pre[-2:]
    # This uses the in-motion readings only in a retrospective stop diagnostic.
    # The command-before-image comparison never receives these quantities.
    velocity = 1000 * (last[1] - prev[1]) / (last[0] - prev[0])
    off_deg = last[1] + velocity * (stop - last[0]) / 1000
    pose = make_pose(seed, anchor, off_deg, "extrapolated_at_driver_OFF")
    shoulder = "shoulder_pitch_joint"
    q = pose["joints"]
    model = mujoco.MjModel.from_xml_path(str(model_path))
    joint = model.joint(shoulder)
    result = dict(
        startMs=start, offMs=stop, durationMs=stop-start, peakActualPwm=peak,
        beforeStart=first, lastTwoInMotion=[prev,last], firstAfterOff=post,
        observedDeltaSensorDeg=post[1]-first[1], coastWindowMs=post[0]-stop,
        extrapolatedOffSensorDeg=off_deg, estimatedVelocityDegSec=velocity,
        extrapolatedOffDisplayJointDeg=math.degrees(q[shoulder]),
        modelShoulderRangeDeg=[math.degrees(x) for x in joint.range],
    )
    if not joint.range[0] <= q[shoulder] <= joint.range[1]:
        result["status"] = "extrapolated_OFF_outside_model_joint_range"
        return result

    data = mujoco.MjData(model)
    for name, angle in q.items():
        data.joint(name).qpos[0] = angle
        data.actuator(name).ctrl[0] = angle
    data.joint("gripper_opposing_joint").qpos[0] = -q["gripper_joint"]
    sign = anchor["m4Reference"]["directionSign"]
    data.joint(shoulder).qvel[0] = sign * math.radians(velocity)
    act = model.actuator(shoulder)
    act.gainprm[:] = 0
    act.biasprm[:] = 0
    mujoco.mj_forward(model, data)
    fk = forward_kinematics(JointAngles.from_positions(q), geometry)
    tip = (fk.x_m, fk.y_m, fk.z_m)
    ik = inverse_kinematics(*tip, model=geometry, target_tool_pitch_rad=fk.pitch_rad)
    result.update(
        assumedOtherJointsRad={key: q[key] for key in q if key != shoulder},
        candidateTipM=tip, fkMujocoTipGapM=math.dist(tip, data.body("tool0").xpos),
        inverseBranch=ik.reason,
    )
    for _ in range(round(result["coastWindowMs"] / (1000 * model.opt.timestep))):
        mujoco.mj_step(model, data)
    forecast = off_deg + sign * math.degrees(data.joint(shoulder).qpos[0] - q[shoulder])
    result.update(
        status="compared", simulatedTimeSec=data.time, forecastSensorDeg=forecast,
        absoluteErrorDeg=abs(forecast-post[1]), holdOffEstimateErrorDeg=abs(off_deg-post[1]),
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--anchor", type=Path, required=True)
    args = parser.parse_args()
    samples, spans = recorded_motion(args.episode / "controller.jsonl")
    geometry = ArmModel.from_profile(args.model_dir / "arm_geometry.json")
    seed = read_json(ROOT / "tools/visualization/display-pose.json")
    anchor = read_json(args.anchor)
    model_path = args.model_dir / "model.xml"
    urdf = ET.parse(ROOT / "ros2_ws/src/motionbrain_description/urdf/motionbrain.urdf")
    axes = {name: urdf.find(f"joint[@name='{name}']/axis").get("xyz") for name in seed["joints"]}
    rows = [
        compare_stop(span, spans[idx+1][0] if idx+1 < len(spans) else math.inf,
                     samples, seed, anchor, model_path, geometry)
        for idx, span in enumerate(spans)
    ]
    print(json.dumps(dict(
        episode=args.episode.name, assumedModel="unpowered shoulder after OFF, existing display damping and gravity compensation",
        measuredClock="sample time approximated by status uptimeMs minus I2C ageMs; driver time in controller milliseconds; angles in degrees",
        scope="retrospective OFF comparison; velocities estimated during motion; other joint angles assumed",
        mujocoVersion=mujoco.__version__, rvizUrdfJointAxes=axes, stops=rows,
    ), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
