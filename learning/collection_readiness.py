"""Offline evidence joins for a future physical collector; never an admission gate.

Reads saved bytes only. A consistent packet records claims, not optical truth or
permission to operate. It cannot promote demos, commissioning captures or fixtures
into a training dataset. Existing transition validation remains independent.
"""
from __future__ import annotations

import math
from learning.artifacts import (StorageError, describe_artifact, read_json_artifact,
                                verify_artifacts)
from learning.hardware_demo import audit_demo
from learning.device_capture import audit_device_capture
from learning.provenance import synthetic_reasons, KNOWN_SYNTHETIC_IMAGE_HASHES
from learning.schema import validate_transition
from learning.features import transition_features

SCHEMA = "motionbrain.collection_scene.v1"
POLICY = dict(maxSourceAgeMs=100, maxM4AgeMs=20, maxAnchorAgeMs=15000,
              maxPersistenceMs=250, minimumSettlingMs=50)


def _object(value):
    return value if isinstance(value, dict) else {}


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _text(value):
    return isinstance(value, str) and bool(value.strip())


def scene_template(transition, source_ref):
    """Unknown fields stay unknown; copying an old view is never a default."""
    return dict(schemaVersion=SCHEMA, transition=source_ref,
                transitionId=transition.get("transitionId"), sessionId=transition.get("sessionId"),
                viewId=None, objectInstanceId=None, targetLabel="lipbalm", policy=dict(POLICY),
                opticalCalibration=None, anchor=None, before=None, after=None,
                settling=[], m4ToleranceDegrees=None, visualReview=None, finalOutputLog=None,
                trainingEligible=False, controlEligible=False)


def load_transition(root, relative):
    ref = describe_artifact(root, relative)
    document = read_json_artifact(root, ref)
    transition = _object(document).get("transition", document)
    if not isinstance(transition, dict):
        raise StorageError("transition object required")
    return transition, ref


def review_transition(root, relative, scene=None):
    """Validate joins, conservative time bounds and original persistence receipts.

All descriptors resolve within root without following symlinks. Optical bounds
must be explicitly supplied by scoped optical calibration, never derived here
from PTS, callback, HTTP receive time or offline file-copy duration.
"""
    transition, source_ref = load_transition(root, relative)
    validation = validate_transition(transition)
    scene = _object(scene)
    issues = []
    evidence_documents = []

    def require(condition, reason):
        if not condition and reason not in issues:
            issues.append(reason)
        return bool(condition)

    def artifact(ref, name, json=False):
        faults = verify_artifacts(root, [ref])
        if not require(not faults, name + ":artifact_missing_or_changed"):
            return {}
        if json:
            try:
                value = read_json_artifact(root, ref)
                evidence_documents.append(value)
                require(isinstance(value, dict), name + ":object_required")
                return _object(value)
            except (StorageError, ValueError):
                require(False, name + ":invalid_json")
        return {}

    def interval(value, name):
        value = _object(value)
        lo, hi = value.get("earliest"), value.get("latest")
        if not require(value.get("clockId") == clock and _finite(lo) and _finite(hi)
                       and 0 <= lo <= hi, name + ":clock_or_bounds_invalid"):
            return None
        return lo, hi

    require(scene.get("schemaVersion") == SCHEMA, "scene:missing_or_unsupported")
    require(scene.get("transition") == source_ref, "scene:transition_bytes_mismatch")
    for key in ("transitionId", "sessionId"):
        require(_text(scene.get(key)) and scene.get(key) == transition.get(key), "scene:" + key + "_mismatch")
    for key in ("viewId", "objectInstanceId"):
        require(_text(scene.get(key)), "scene:" + key + "_required")
    require(scene.get("targetLabel") == "lipbalm", "scene:lipbalm_label_required")
    require(scene.get("policy") == POLICY, "scene:fixed_policy_required")
    require(scene.get("trainingEligible") is False and scene.get("controlEligible") is False,
            "scene:must_remain_ineligible")
    require(validation["valid"], "transition:schema_invalid")
    try:
        transition_features(transition)
    except (ValueError, TypeError, KeyError, AttributeError, OverflowError):
        require(False, "transition:visual_dynamics_features_unavailable")
    # Even a valid historical schema may have relaxed its caller-supplied policy.
    p = _object(transition.get("qualityPolicy"))
    require(p.get("maxSourceAgeMs") == 100 and p.get("maxM4AgeMs") == 20
            and _finite(p.get("minimumSettlingMs")) and p["minimumSettlingMs"] >= 50,
            "transition:fixed_quality_limits_required")
    for reason in validation["exclusionReasons"]:
        require(False, "transition:" + reason)

    action = _object(transition.get("action"))
    clock = _object(action.get("startHostBounds")).get("clockId")
    require(_text(clock), "action:host_clock_required")
    start = interval(action.get("startHostBounds"), "action:start")
    off = interval(action.get("outputOffHostBounds"), "action:off")
    calibration = artifact(scene.get("opticalCalibration"), "optical", json=True)
    for key in ("sessionId", "viewId"):
        require(_text(scene.get(key)) and calibration.get(key) == scene.get(key), "optical:" + key + "_mismatch")
    require(calibration.get("method") == "optical_exposure_bounds"
            and calibration.get("opticalExposureVerified") is True,
            "optical:exposure_evidence_required")
    require(_text(calibration.get("reviewer")), "optical:reviewer_required")
    artifact(calibration.get("measurement"), "optical:measurement")
    validity = interval(calibration.get("validHostBounds"), "optical:validity")
    require(_text(calibration.get("cameraBootId")), "optical:camera_boot_required")

    frames = {}
    m4_samples = {}

    def frame(value, name, observation=None):
        value = _object(value)
        for key in ("sessionId", "viewId", "objectInstanceId", "targetLabel"):
            require(_text(scene.get(key)) and value.get(key) == scene.get(key), name + ":" + key + "_mismatch")
        require(_text(value.get("sourceFrameId")), name + ":frame_id_required")
        image = _object(value.get("image"))
        artifact(image, name + ":image")
        require(isinstance(image.get("sha256"), str) and image["sha256"] not in KNOWN_SYNTHETIC_IMAGE_HASHES,
                name + ":synthetic_or_invalid_image_hash")
        require(value.get("cameraBootId") == calibration.get("cameraBootId")
                and _text(value.get("cameraBootId")), name + ":camera_boot_mismatch")
        captured = value.get("capturedUs")
        require(type(captured) is int and captured > 0 and
                value.get("sourceFrameId") == f'{value.get("cameraBootId")}:{captured}',
                name + ":source_identity_mismatch")
        bounds = interval(value.get("exposureHostBounds"), name + ":exposure")
        require(value.get("opticalCalibration") == scene.get("opticalCalibration")
                and isinstance(value.get("opticalCalibration"), dict), name + ":optical_binding_required")
        if bounds and validity:
            require(validity[0] <= bounds[0] <= bounds[1] <= validity[1], name + ":calibration_expired")
        if observation is not None:
            observation = _object(observation)
            require(observation.get("label") == scene.get("targetLabel")
                    and observation.get("width") == value.get("imageWidth")
                    and observation.get("height") == value.get("imageHeight")
                    and observation.get("targetBox") == value.get("box"),
                    name + ":transition_target_mismatch")
            for key in ("sourceFrameId", "cameraBootId", "capturedUs"):
                require(value.get(key) == observation.get(key), name + ":transition_frame_mismatch")
            require(image.get("sha256") == observation.get("imageSha256")
                    and image.get("artifactRef") == _object(observation.get("image")).get("artifactRef"),
                    name + ":transition_image_mismatch")
            # A scene sidecar cannot replace the collector's clock interval.
            original = interval(observation.get("captureHostBounds"), name + ":collector_capture")
            uncertainty = observation.get("captureHostBoundsUncertaintySeconds")
            if bounds and original and _finite(uncertainty) and uncertainty >= 0:
                require(bounds[0] <= original[0] - uncertainty and
                        bounds[1] >= original[1] + uncertainty, name + ":optical_bounds_narrower_than_collector")
        if name != "anchor":
            sample = artifact(value.get("m4"), name + ":m4", json=True)
            sensor_bounds = interval(sample.get("sampledHostBounds"), name + ":m4_sample")
            angle = sample.get("angleDegrees")
            require(sample.get("motor") == 4 and sample.get("sensor") == "AS5600"
                    and sample.get("measured") is True and _finite(angle) and 0 <= angle < 360,
                    name + ":m4_measurement_required")
            consumed = value.get("consumedMonotonic")
            if sensor_bounds:
                require(_finite(consumed) and sensor_bounds[1] <= consumed
                        and (consumed-sensor_bounds[0])*1000 <= 20 + 1e-6, name + ":m4_stale")
                if name == "before" and start:
                    require(sensor_bounds[1] < start[0], name + ":m4_after_action")
                    require((start[1]-sensor_bounds[0])*1000 <= 20 + 1e-6,
                            name + ":m4_expired_at_dispatch")
                elif off:
                    require(sensor_bounds[0] > off[1], name + ":m4_before_off")
            if observation is not None:
                require(sample == _object(observation).get("m4"), name + ":m4_transition_mismatch")
            m4_samples[name] = (sensor_bounds, angle)
        used = value.get("consumedMonotonic")
        if bounds:
            require(_finite(used) and bounds[1] <= used and (used-bounds[0])*1000 <= 100 + 1e-6,
                    name + ":source_age_exceeds_100ms_or_unknown")
        # Original capture transaction, NOT this offline review's storage time.
        persisted = _object(value.get("persistence"))
        begin, end = persisted.get("startedMonotonic"), persisted.get("completedMonotonic")
        require(persisted.get("basis") == "original_capture_transaction"
                and persisted.get("clockId") == clock and _finite(begin) and _finite(end)
                and 0 <= begin <= end and (end-begin)*1000 <= 250 + 1e-6
                and bounds is not None and begin >= bounds[1] and _finite(used) and end <= used,
                name + ":original_persistence_unverified_or_over_250ms")
        receipt = artifact(persisted.get("receipt"), name + ":persistence_receipt", json=True)
        require(receipt == {k: v for k, v in persisted.items() if k != "receipt"}
                and receipt.get("image") == image
                and receipt.get("sourceFrameId") == value.get("sourceFrameId")
                and all(receipt.get(k) is True for k in ("fileFsynced", "directoryFsynced", "readbackVerified")),
                name + ":persistence_receipt_mismatch")
        box = _object(value.get("box"))
        coords = [box.get(k) for k in ("x", "y", "width", "height")]
        width, height = value.get("imageWidth"), value.get("imageHeight")
        require(type(width) is int and type(height) is int and width > 0 and height > 0
                and all(_finite(c) for c in coords) and coords[0] >= 0 and coords[1] >= 0
                and coords[2] > 0 and coords[3] > 0 and coords[0]+coords[2] <= width
                and coords[1]+coords[3] <= height, name + ":target_box_invalid")
        require(value.get("annotationMethod") == "human_semantic_review" and _text(value.get("reviewer")),
                name + ":semantic_review_required")
        frames[name] = value
        return bounds

    anchor = frame(scene.get("anchor"), "anchor")
    before = frame(scene.get("before"), "before", transition.get("before"))
    after = frame(scene.get("after"), "after", transition.get("after"))
    if before and start:
        require(before[1] < start[0], "before:not_before_action")
        require((start[1]-before[0])*1000 <= 100 + 1e-6, "before:expired_at_dispatch")
        used = _object(scene.get("before")).get("consumedMonotonic")
        require(_finite(used) and used <= start[0], "before:consumed_after_dispatch")
    if after and off:
        require(after[0] > off[1] + .05, "after:not_after_off_and_settling")

    settling = scene.get("settling")
    if not isinstance(settling, list):
        settling = []
    require(3 <= len(settling) <= 6, "settling:requires_3_to_6_frames")
    windows = [frame(value, f"settling[{i}]") for i, value in enumerate(settling[:6])]
    if windows and all(windows) and off and after:
        require(windows[0][0] > off[1] and windows[-1][1] < after[0]
                and all(a[1] < b[0] for a, b in zip(windows, windows[1:])), "settling:order_invalid")
        require((windows[-1][0]-windows[0][1])*1000 >= 50 - 1e-6,
                "settling:span_below_50ms")
    tolerance = scene.get("m4ToleranceDegrees")
    require(_finite(tolerance) and 0 <= tolerance <= 1, "settling:explicit_m4_tolerance_required")
    samples = [m4_samples[f"settling[{i}]"] for i in range(min(len(settling), 6))]
    if samples and all(b for b, _ in samples):
        require(all(a[0][1] < b[0][0] for a, b in zip(samples, samples[1:]))
                and (samples[-1][0][0]-samples[0][0][1])*1000 >= 50 - 1e-6,
                "settling:m4_window_invalid")
    angles = [a for _, a in samples] + [m4_samples["after"][1]]
    if _finite(tolerance) and all(_finite(a) for a in angles):
        require(all(abs((a-b+180) % 360-180) <= tolerance for a in angles for b in angles),
                "settling:m4_moved")
    ids = [f.get("sourceFrameId") for name, f in frames.items() if name != "anchor"]
    require(all(isinstance(i, str) for i in ids) and len(set(i for i in ids if isinstance(i, str))) == len(ids),
            "frames:reused_or_missing")
    ordered_frames = [frames["anchor"], frames["before"],
                      *(frames[f"settling[{i}]"] for i in range(min(len(settling), 6))), frames["after"]]
    source_stamps = [f.get("capturedUs") for f in ordered_frames]
    require(all(type(t) is int and t > 0 for t in source_stamps)
            and all(a < b for a, b in zip(source_stamps, source_stamps[1:])),
            "frames:source_time_not_advancing")
    # Anchor must causally precede every use, including the final dispatch bound.
    if anchor:
        uses = [f.get("consumedMonotonic") for name, f in frames.items() if name != "anchor"]
        if start:
            uses.append(start[1])
        require(all(_finite(t) and t >= anchor[1] and (t-anchor[0])*1000 <= 15000 + 1e-6 for t in uses),
                "anchor:future_or_expired_at_use")
        for name, value in frames.items():
            if name != "anchor":
                b = interval(value.get("exposureHostBounds"), name + ":exposure")
                if b:
                    require(anchor[1] <= b[0], name + ":anchor_from_future_frame")

    review = artifact(scene.get("visualReview"), "visual", json=True)
    require(review.get("transition") == source_ref and review.get("sessionId") == scene.get("sessionId")
            and review.get("viewId") == scene.get("viewId") and review.get("objectInstanceId") == scene.get("objectInstanceId"),
            "visual:identity_mismatch")
    require(_text(review.get("reviewer")) and review.get("method") == "human_sequence_review",
            "visual:reviewer_required")
    require(review.get("frames") == {name: f.get("image") for name, f in frames.items()}, "visual:frame_binding_mismatch")
    require(review.get("wholeArmStationary") is True and review.get("objectStationary") is True,
            "visual:settling_unverified")
    require(review.get("taskOutcome") in ("success", "failure", "unknown"), "visual:outcome_required")
    if _object(transition.get("outcome")).get("taskSuccess") is not None:
        expected = True if review.get("taskOutcome") == "success" else False if review.get("taskOutcome") == "failure" else None
        require(expected is not None and transition["outcome"]["taskSuccess"] is expected,
                "visual:task_outcome_mismatch")

    # Preserve all driver writes after terminal OFF through the after observation.
    tail = artifact(scene.get("finalOutputLog"), "outputs:final", json=True)
    record = _object(action.get("record"))
    require(tail.get("bootId") == record.get("bootId") and tail.get("evidence") == "driver_write"
            and tail.get("clockDomain") == "controller_boot_millis"
            and type(tail.get("dropped")) is int and tail["dropped"] == 0, "outputs:scope_or_dropped")
    received = interval(tail.get("readHostBounds"), "outputs:read")
    if received and after:
        used = frames["after"].get("consumedMonotonic")
        require(_finite(used) and received[0] >= used, "outputs:read_precedes_after")
    baseline = _object(action.get("outputSequenceBounds")).get("last")
    entries = tail.get("entries")
    if not isinstance(entries, list):
        entries = []
    previous = previous_ms = None
    found = False
    originals = _object(action.get("outputLog")).get("entries", [])
    terminal_entries = [e for e in originals if isinstance(e, dict) and e.get("sequence") == baseline] if isinstance(originals, list) else []
    for e in entries:
        e = _object(e)
        at = e.get("atMs")
        require(all(type(e.get(k)) is int and lo <= e[k] <= hi for k, lo, hi in (
                    ("atMs", 0, 2**32-1), ("actionOrdinal", 0, 2**32-1),
                    ("motor", 1, 5), ("pwm", -255, 255)))
                and e.get("reason") in ("driver_write", "forced_off")
                and (e.get("reason") != "forced_off" or e.get("pwm") == 0),
                "outputs:invalid_entry")
        if type(at) is int and 0 <= at < 2**32:
            if previous_ms is not None:
                require((at-previous_ms) % 2**32 < 2**31, "outputs:time_regressed")
            previous_ms = at
        seq = e.get("sequence")
        if not require(type(seq) is int and 1 <= seq < 2**32 and (previous is None or seq == previous+1), "outputs:sequence_gap"):
            continue
        previous = seq
        if seq == baseline:
            found = True
            require(len(terminal_entries) == 1 and e == terminal_entries[0], "outputs:terminal_mismatch")
        if type(baseline) is int and seq > baseline:
            require(type(e.get("pwm")) is int and e["pwm"] == 0, "outputs:reactivated_after_off")
    require(found, "outputs:terminal_missing")
    require(not synthetic_reasons(transition, scene, *evidence_documents), "evidence:synthetic_provenance")
    return dict(schemaVersion="motionbrain.collection_readiness.v1", readOnly=True,
                transition=source_ref, transitionValidation=validation,
                recordedEvidenceComplete=not issues, blockers=issues,
                trainingEligible=False, controlEligible=False, executionAllowed=False,
                physicalVerification="not_established_by_offline_review",
                nextGate="physical_collector_and_dataset_provenance_validation",
                policy=dict(POLICY))


def review_demo(root):
    """Keep every archived manual attempt in the denominator; never convert it."""
    errors = audit_demo(root)
    ref = describe_artifact(root, "demo.json")
    demo = read_json_artifact(root, ref)
    blockers = ["manual_lease_not_bounded_action", "synchronized_before_frame_missing",
                "optical_exposure_unverified", "semantic_anchor_unverified",
                "original_persistence_unverified", "settling_sequence_unverified"]
    attempts = []
    source_attempts = demo.get("attempts")
    for attempt in source_attempts if isinstance(source_attempts, list) else []:
        attempt = _object(attempt)
        assets = _object(attempt.get("artifacts"))
        attempts.append(dict(step=attempt.get("step"), executionRecordPresent="execution/result.json" in assets,
                             iphoneImagePresent=bool({"after.jpg", "after.png"} & set(assets)),
                             camImagePresent="camera.jpg" in assets,
                             reportedFailure=attempt.get("reportedFailure"), blockers=list(blockers),
                             trainingEligible=False))
    return dict(schemaVersion="motionbrain.collection_readiness.v1", source=ref, readOnly=True,
                archiveIntegrityPassed=not errors, integrityErrors=errors, attempts=attempts,
                recordedEvidenceComplete=False, trainingEligible=False, controlEligible=False,
                executionAllowed=False, eligibleTransitions=0, policy=dict(POLICY))


def review_device_capture(root):
    """Audit commissioning bytes without inventing a cross-host clock mapping."""
    audit = audit_device_capture(root)
    return dict(schemaVersion="motionbrain.collection_readiness.v1", readOnly=True,
                commissioningAudit=audit, recordedEvidenceComplete=False,
                trainingEligible=False, controlEligible=False, executionAllowed=False,
                blockers=["commissioning_capture_not_training_transition",
                          "optical_cross_host_clock_mapping_required",
                          "m4_and_image_intervals_must_be_joined_without_relabeling",
                          "lipbalm_semantic_anchor_required", "whole_arm_visual_settling_required",
                          "physical_collector_and_dataset_provenance_validation_required"],
                policy=dict(POLICY))
