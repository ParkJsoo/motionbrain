#!/usr/bin/env python3
"""저장된 M4 실측값을 표시 전용 근사 자세에 반영한다. 실물 통신 없음."""
import argparse
import copy
import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
JOINTS = ('base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint', 'wrist_pitch_joint', 'gripper_joint')


def make_pose(seed, anchor, sensor_deg, evidence):
    if type(sensor_deg) not in (int, float) or not math.isfinite(sensor_deg) or not 0 <= sensor_deg <= 360:
        raise ValueError('finite_sensor_angle_required')
    result = copy.deepcopy(seed)
    if set(result['joints']) != set(JOINTS):
        raise ValueError('five_display_joints_required')
    ref = anchor['m4Reference']
    sign = ref['directionSign']
    if sign not in (-1, 1):
        raise ValueError('anchor_direction_required')
    joint_deg = ref['jointReferenceDeg'] + sign*(sensor_deg-ref['sensorReferenceDeg'])
    result['joints']['shoulder_pitch_joint'] = math.radians(joint_deg)
    result.update(description='마지막 M4 실측값을 단일 근사 기준점으로 환산한 정적 표시. 다른 관절은 가정이며 실시간 추종·절대 영점 보정·동역학 보정이 아님.',
        method='last_measured_M4_single_anchor_display_only', controlEligible=False,
        feedbackCalibrated=False, dynamicsCalibrated=False, liveHardwareFollowing=False,
        unmeasured=[name for name in JOINTS if name != 'shoulder_pitch_joint'],
        assumedJoints=[name for name in JOINTS if name != 'shoulder_pitch_joint'],
        lastMeasuredM4=dict(sensorDeg=sensor_deg, displayJointDeg=joint_deg,
            anchorSensorDeg=ref['sensorReferenceDeg'], anchorJointDeg=ref['jointReferenceDeg'],
            directionSign=sign, evidence=evidence, updatedAt=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            absoluteCalibrationVerified=False))
    return result


def docker(*args):
    return subprocess.run(['docker', *args], check=True, capture_output=True, text=True, timeout=20)


def apply_viewers(path, pose):
    """고정된 두 표시 전용 컨테이너만 사용한다. 실물 ROS 노드를 시작하지 않는다."""
    rviz = 'motionbrain-visual-rviz'
    gazebo = 'motionbrain-visual-gazebo'
    for name in (rviz, gazebo):
        if docker('inspect', '-f', '{{.State.Running}}', name).stdout.strip() != 'true':
            raise ValueError('existing_visual_container_not_running:'+name)
    installed = subprocess.run(['docker', 'exec', rviz, 'test', '-f', '/opt/motionbrain/runtime-pose.json'],
        check=False, capture_output=True, timeout=10).returncode == 0
    docker('cp', str(path), rviz+':/opt/motionbrain/runtime-pose.json')
    docker('cp', str(ROOT/'tools/visualization/display_pose_publisher.py'), rviz+':/opt/motionbrain/display_pose_publisher.py')
    docker('cp', str(ROOT/'tools/visualization/desktop.sh'), rviz+':/usr/local/bin/motionbrain-visual-desktop')
    if not installed:
        docker('restart', rviz)
    for name, value in pose['joints'].items():
        topic = f'/model/MotionBrain_uncalibrated/joint/{name}/0/cmd_pos'
        command = 'source /opt/ros/jazzy/setup.bash; export GZ_PARTITION=motionbrain_offline; '
        command += f'gz topic -t {topic} -m gz.msgs.Double -p "data: {value}"'
        docker('exec', gazebo, 'bash', '-lc', command)
    measured = pose['lastMeasuredM4']
    text = f'LAST M4 {measured["sensorDeg"]:.2f} deg / display approx {measured["displayJointDeg"]:.2f} deg\nOther joints assumed\nNo live feedback; physics uncalibrated'
    marker = 'action: ADD_MODIFY, ns: "motionbrain_display", id: 1, type: TEXT, pose: {position: {x: 0.03, y: 0, z: 0.30}}, scale: {x: 0.012, y: 0.012, z: 0.012}, material: {ambient: {r: 0.05, g: 0.05, b: 0.05, a: 1}}, text: '+json.dumps(text)
    docker('exec', gazebo, 'bash', '-lc', 'source /opt/ros/jazzy/setup.bash; export GZ_PARTITION=motionbrain_offline; gz service -s /marker --reqtype gz.msgs.Marker --reptype gz.msgs.Boolean --timeout 3000 --req '+"'"+marker+"'")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor-deg', type=float, required=True)
    parser.add_argument('--anchor', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--evidence', required=True)
    parser.add_argument('--apply-viewers', action='store_true')
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'.local'):
        parser.error('표시용 실측 자세는 .local 아래에만 저장합니다')
    seed = json.loads((ROOT/'tools/visualization/display-pose.json').read_text())
    anchor_raw = args.anchor.read_bytes()
    pose = make_pose(seed, json.loads(anchor_raw), args.sensor_deg, args.evidence)
    pose['anchorSha256'] = hashlib.sha256(anchor_raw).hexdigest()
    evidence_path = (ROOT/args.evidence).resolve(strict=True)
    if not evidence_path.is_relative_to(ROOT/'.local'):
        parser.error('실측 근거는 .local의 기존 파일이어야 합니다')
    pose['lastMeasuredM4']['evidenceSha256'] = hashlib.sha256(evidence_path.read_bytes()).hexdigest()
    output.parent.mkdir(parents=True, exist_ok=True)
    pending = output.with_name(output.name+'.pending')
    pending.write_text(json.dumps(pose, ensure_ascii=False, indent=2)+'\n')
    pending.replace(output)
    if args.apply_viewers:
        apply_viewers(output, pose)
    print(json.dumps(dict(output=str(output), lastMeasuredM4=pose['lastMeasuredM4'], controlEligible=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
