"""정지 구간의 높이·사진·M4 센서 보고를 묶는 근사 읽기 전용 기준점.

센서/모터/네트워크에 접근하지 않는다. 단일 기준점은 정밀 영점 검증이나
센서 없는 다른 관절의 영점 보정, 실물 제어 허용을 뜻하지 않는다.
"""
import hashlib
import math
from pathlib import Path
import re

from learning.arm_photo_reference import infer_reference
from learning.artifacts import _decode_json
from motionbrain_ros_bridge.arm_geometry import load_geometry_profile, geometry_values, profile_digest


_FIELDS = {"schema", "geometryProfilePath", "geometryProfileSha256", "heightMeasurementPath",
           "heightMeasurementSha256", "beforeSnapshotPath", "beforeSnapshotSha256",
           "afterSnapshotPath", "afterSnapshotSha256", "image", "directionSign", "directionEvidence"}


def _check(condition, reason):
    if not condition:
        raise ValueError(reason)


def _number(value):
    return type(value) in (int, float) and -1e9 <= value <= 1e9 and math.isfinite(value)


def _file(root, name, digest):
    _check(isinstance(name, str) and bool(name), "joint_reference_path_required")
    path = Path(name)
    path = (path if path.is_absolute() else root / path).resolve(strict=True)
    _check(path.is_relative_to(root), "joint_reference_source_outside_root")
    raw = path.read_bytes()
    _check(hashlib.sha256(raw).hexdigest() == digest, "joint_reference_source_hash_mismatch")
    return path, raw


def _snapshot(value):
    _check(type(value) is dict and value.get("schema") == "motionbrain.arm_anchor_snapshot.v1",
           "invalid_anchor_snapshot_schema")
    _check(isinstance(value.get("hostClockId"), str) and bool(value["hostClockId"]), "anchor_host_clock_required")
    _check(all(_number(value.get(k)) for k in ("requestStartedAt", "receivedAt"))
           and 0 <= value["requestStartedAt"] <= value["receivedAt"], "invalid_anchor_snapshot_time")
    status, actions, outputs, sequence = (value.get(k) for k in ("status", "actions", "action_outputs", "sequence"))
    _check(all(type(v) is dict for v in (status, actions, outputs, sequence)), "anchor_get_responses_required")
    boot = actions.get("bootId")
    _check(isinstance(boot, str) and re.fullmatch("[0-9a-f]{32}", boot) is not None
           and outputs.get("bootId") == boot and actions.get("active") is False, "anchor_boot_or_action_invalid")
    _check(status.get("state") == "IDLE" and status.get("motorEnabled") is False, "anchor_not_idle_off")
    motors = status.get("motors")
    _check(type(motors) is dict and set(motors) == {f"M{i}" for i in range(1, 6)}, "anchor_five_motors_required")
    _check(all(type(m) is dict for m in motors.values()), "anchor_motor_status_invalid")
    _check(all(type(m.get("speed")) is int and m["speed"] == 0 and m.get("enabled") is False
               for m in motors.values()), "anchor_motor_not_off")
    queue = sequence.get("sequence", sequence)
    _check(type(queue) is dict and queue.get("state") == "IDLE"
           and type(queue.get("totalCount")) is int and queue["totalCount"] == 0, "anchor_queue_not_empty")
    _check(outputs.get("evidence") == "driver_write" and outputs.get("clockDomain") == "controller_boot_millis"
           and outputs.get("gap", False) is False and outputs.get("hasMore", False) is False,
           "anchor_output_evidence_incomplete")
    entries = outputs.get("entries")
    _check(type(entries) is list and bool(entries), "anchor_output_entries_required")
    latest, previous = {}, None
    for entry in entries:
        _check(type(entry) is dict and all(type(entry.get(k)) is int for k in ("sequence", "atMs", "motor", "pwm")),
               "anchor_output_entry_invalid")
        _check(entry["sequence"] > 0 and 0 <= entry["atMs"] < 2**32 and 1 <= entry["motor"] <= 5
               and -255 <= entry["pwm"] <= 255, "anchor_output_entry_invalid")
        _check(previous is None or entry["sequence"] == previous + 1, "anchor_output_sequence_gap")
        latest[entry["motor"]] = entry["pwm"]
        previous = entry["sequence"]
    _check(latest == {i: 0 for i in range(1, 6)}, "anchor_driver_output_not_off")
    shoulder = status.get("shoulderAngle")
    _check(type(shoulder) is dict and all(shoulder.get(k) is True for k in
        ("available", "sensorConnected", "sensorFresh", "sensorReady", "magnetDetected"))
        and all(shoulder.get(k) is False for k in
        ("magnetTooWeak", "magnetTooStrong", "active", "correctionActive", "manualCalibrationUnlock")),
        "anchor_shoulder_sensor_not_ready")
    _check(all(_number(shoulder.get(k)) for k in ("angleDeg", "rawDeg", "mountOffsetDeg", "ageMs"))
           and 0 <= shoulder["angleDeg"] < 360 and 0 <= shoulder["rawDeg"] < 360
           and 0 <= shoulder["ageMs"] <= 150, "anchor_shoulder_sensor_invalid")
    corrected = (shoulder["rawDeg"] + shoulder["mountOffsetDeg"]) % 360
    difference = (corrected - shoulder["angleDeg"] + 180) % 360 - 180
    _check(abs(difference) <= .02, "anchor_sensor_space_mismatch")
    uptime = status.get("uptimeMs")
    _check(type(uptime) is int and 0 <= uptime < 2**32, "anchor_controller_uptime_required")
    return boot, shoulder, entries, uptime


def infer_joint_reference(manifest, root) -> dict:
    """근사 M4 anchor와 별개의 사진 정적 관측을 반환한다. 적용 동작은 없다."""
    _check(type(manifest) is dict and set(manifest) == _FIELDS
           and manifest["schema"] == "motionbrain.arm_joint_reference_input.v1", "invalid_joint_reference_input")
    root = Path(root).resolve(strict=True)
    # Geometry SHA uses the existing canonical profile digest; other SHA fields bind file bytes.
    _check(isinstance(manifest["geometryProfilePath"], str), "joint_reference_path_required")
    geometry_path = Path(manifest["geometryProfilePath"])
    geometry_path = (geometry_path if geometry_path.is_absolute() else root / geometry_path).resolve(strict=True)
    _check(geometry_path.is_relative_to(root), "joint_reference_source_outside_root")
    geometry_raw = geometry_path.read_bytes()
    profile = load_geometry_profile(geometry_path)
    _check(geometry_path.read_bytes() == geometry_raw and profile_digest(profile) == manifest["geometryProfileSha256"],
           "joint_reference_geometry_hash_mismatch")
    geometry = geometry_values(profile)
    height_path, height_raw = _file(root, manifest["heightMeasurementPath"], manifest["heightMeasurementSha256"])
    height = _decode_json(height_raw)
    _check(type(height) is dict and height.get("schema") == "motionbrain.elbow_height_measurement.v1"
           and height.get("unit") == "mm" and height.get("uncertaintyQuantified") is False
           and isinstance(height.get("method"), str) and bool(height["method"].strip()), "invalid_elbow_height_measurement")
    bounds = height.get("elbowHeightRangeMm")
    _check(type(bounds) is list and len(bounds) == 2 and all(_number(v) for v in bounds)
           and 0 <= bounds[0] <= bounds[1] <= 1000, "invalid_elbow_height_range")
    snapshots, source_hashes = [], {}
    for label in ("before", "after"):
        path, raw = _file(root, manifest[label + "SnapshotPath"], manifest[label + "SnapshotSha256"])
        snapshots.append(_decode_json(raw))
        source_hashes[label + "SnapshotSha256"] = hashlib.sha256(raw).hexdigest()
    before, after = snapshots
    bboot, bsensor, bentries, buptime = _snapshot(before)
    aboot, asensor, aentries, auptime = _snapshot(after)
    image = manifest["image"]
    _check(type(image) is dict and before["hostClockId"] == after["hostClockId"] == image.get("hostClockId"),
           "anchor_host_clock_changed")
    _check(bboot == aboot and 0 <= auptime - buptime <= 30000, "anchor_controller_boot_or_time_changed")
    _check(bentries == aentries and bsensor["mountOffsetDeg"] == asensor["mountOffsetDeg"], "anchor_output_or_mount_changed")
    _check(all(_number(image.get(k)) for k in ("sourceEarliest", "sourceLatest"))
           and before["receivedAt"] <= image["sourceEarliest"] <= image["sourceLatest"] <= after["requestStartedAt"]
           and 0 <= after["receivedAt"] - before["requestStartedAt"] <= 30,
           "image_outside_static_snapshot_bracket")
    sensor_delta = asensor["angleDeg"] - bsensor["angleDeg"]
    _check(abs(sensor_delta) <= .25, "anchor_shoulder_moved_or_wrapped")
    photo_input = {key: image[key] for key in ("imagePath", "imageSha256", "imageWidth", "imageHeight", "points",
                                              "forwardImageSign", "horizontalRollDeg")}
    photo_input.update(geometryProfilePath=str(geometry_path), geometryProfileSha256=manifest["geometryProfileSha256"])
    photo = infer_reference(photo_input, root)
    upper_photo = photo["jointCandidateDeg"]["shoulder_pitch"]
    _check(0 < upper_photo < 90, "photo_must_show_elbow_forward_and_above_shoulder")
    base_height, length = geometry["base_height_m"] * 1000, geometry["upper_arm_m"] * 1000
    ratios = [(h - base_height) / length for h in bounds]
    _check(all(0 < r <= 1 for r in ratios), "elbow_height_outside_forward_upper_asin_domain")
    angle_bounds = [math.degrees(math.asin(r)) for r in ratios]
    chosen_height = (bounds[0] + bounds[1]) / 2
    ratio = (chosen_height - base_height) / length
    angle = math.asin(ratio)
    direction = manifest["directionEvidence"]
    _check(type(direction) is dict and direction.get("status") == "provisional_relative_correspondence",
           "direction_must_remain_provisional")
    direction_path, direction_raw = _file(root, direction["path"], direction["sha256"])
    direction_data = _decode_json(direction_raw)
    _check(type(direction_data) is dict, "invalid_provisional_direction_evidence")
    relative = direction_data
    if "regions" in direction_data:
        regions = direction_data["regions"]
        _check(type(regions) is dict and type(regions.get("upper")) is dict,
               "invalid_provisional_direction_evidence")
        relative = regions["upper"].get("exploratoryWithinRunObservation")
        _check(type(relative) is dict and relative.get("valid") is True,
               "invalid_provisional_direction_evidence")
    sd, vd = direction_data.get("sensorDeltaDeg"), relative.get("projectedDeltaDeg")
    _check(all(_number(v) and abs(v) >= .25 for v in (sd, vd)), "provisional_direction_change_insufficient")
    sign = manifest["directionSign"]
    _check(type(sign) is int and sign in (-1, 1) and (1 if sd * vd > 0 else -1) == sign,
           "provisional_direction_sign_mismatch")
    sensor_reference = (bsensor["angleDeg"] + asensor["angleDeg"]) / 2
    source_hashes.update(imageSha256=image["imageSha256"], geometryProfileSha256=profile_digest(profile),
        geometryFileSha256=hashlib.sha256(geometry_raw).hexdigest(), heightMeasurementSha256=hashlib.sha256(height_raw).hexdigest(),
        directionEvidenceSha256=hashlib.sha256(direction_raw).hexdigest())
    return {
        "schema": "motionbrain.arm_joint_reference.v1", "status": "single_pose_coarse_read_only_anchor",
        "evidence": source_hashes, "controllerBootId": bboot, "hostClockId": before["hostClockId"],
        "staticBracket": {"requestStartedAt": before["requestStartedAt"], "receivedAt": after["receivedAt"],
            "lastOutputSequence": bentries[-1]["sequence"], "outputEntriesUnchanged": True,
            "fiveMotorsOff": True, "emptyQueue": True, "reportedShoulderDeltaDeg": sensor_delta,
            "maxAllowedReportedShoulderDeltaDeg": .25, "otherJointMotionMeasured": False,
            "opticalSensorSynchronizationVerified": False},
        "heightReference": {"unit": "mm", "elbowHeightRangeMm": bounds, "selectedNominalHeightMm": chosen_height,
            "baseHeightMm": base_height, "upperArmMm": length, "branch": "elbow_forward_and_above_shoulder",
            "formula": "asin((elbow_height-base_height)/upper_arm)", "angleDeg": math.degrees(angle),
            "heightMinusPhotoShoulderDeg": math.degrees(angle) - upper_photo,
            "heightInputAngleRangeDeg": angle_bounds,
            "rangeMeaning": "input_height_range_only_not_confidence_interval_geometry_uncertainty_unquantified",
            "equalHeightBoundsMeanZeroError": False, "uncertaintyQuantified": False},
        "m4Reference": {"sensorCoordinate": "firmware_angleDeg_mount_offset_already_applied",
            "sensorReferenceDeg": sensor_reference, "sensorReferenceMethod": "midpoint_of_two_static_bracket_reports",
            "rawReportedRangeDeg": [bsensor["rawDeg"], asensor["rawDeg"]], "mountOffsetDeg": bsensor["mountOffsetDeg"],
            "jointReferenceRad": angle, "jointReferenceDeg": math.degrees(angle), "directionSign": sign,
            "directionStatus": "provisional_relative_correspondence",
            "rosMappingCandidate": {"sensor_zero_deg": sensor_reference, "direction_sign": sign,
                                    "ros_joint_zero_rad": angle},
            "parametersRepresentAnchorPair": True, "enabled": False,
            "unobservedDerivedHorizontalSensorAngleDeg": sensor_reference - sign * math.degrees(angle),
            "derivedHorizontalValueWasMeasured": False},
        "staticPhotoReference": {"status": "single_photo_projected_pose_only", "jointCandidateDeg": photo["jointCandidateDeg"],
            "notCombinedWithSensorAnchor": True, "m2M3SensorZerosEstablished": False},
        "unmeasuredJoints": ["base_yaw", "gripper"], "singlePoseOnly": True,
        "precisionZeroCalibrated": False, "absoluteDirectionValidated": False,
        "feedbackCalibrated": False, "cameraExtrinsicsCalibrated": False,
        "calibrationApplied": False, "controlEligible": False,
    }
