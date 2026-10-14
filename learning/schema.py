"""Conservative, standard-library-only validation of action transition evidence.

This validates recorded claims, not physical truth, JPEG bytes, or electrical output.
Callers retain raw records, including rejected records; this function never mutates
its input. Clock bounds use host monotonic seconds and explicit provenance. MCU
millis and camera microseconds are never directly compared. Existing policy episode
logs are deliberately not accepted as action transitions.
"""

import math
import re

SCHEMA_VERSION = "motionbrain.action_transition.v1"
UINT32 = 2**32


def validate_transition(transition):
    """Return schema validity and dynamics-training eligibility separately.

    A valid canceled/failed action can teach dynamics. taskSuccess is a separate
    externally evidenced label; neither outputOff nor targetReached implies it.
    Freshness limits are explicit data collection policy, not silent defaults.
    """
    errors, excluded = [], []

    def error(path, message):
        errors.append(f"{path}: {message}")

    def exclude(reason):
        if reason not in excluded:
            excluded.append(reason)

    def obj(value, path):
        if not isinstance(value, dict):
            error(path, "required object")
            return {}
        return value

    def string(data, key, path):
        value = data.get(key)
        if not isinstance(value, str) or not value.strip():
            error(f"{path}.{key}", "required nonempty string")
            return None
        return value

    def number(data, key, path, low=None, high=None, integer=False):
        value = data.get(key)
        valid = type(value) in (int, float) and (not integer or type(value) is int)
        try:
            valid = valid and math.isfinite(value)
        except OverflowError:
            valid = False
        if not valid or (low is not None and value < low) or (high is not None and value > high):
            error(f"{path}.{key}", "required finite number within range")
            return None
        return value

    def boolean(data, key, path):
        value = data.get(key)
        if type(value) is not bool:
            error(f"{path}.{key}", "required boolean")
            return None
        return value

    def bounds(value, path, provenance=False):
        value = obj(value, path)
        clock = string(value, "clockId", path)
        earliest = number(value, "earliest", path, 0)
        latest = number(value, "latest", path, 0)
        if provenance:
            string(value, "method", path)
            string(value, "evidenceRef", path)
        if earliest is not None and latest is not None and earliest > latest:
            error(path, "inverted interval")
        return clock, earliest, latest

    def all_finite(value, path):
        if isinstance(value, dict):
            for key, item in value.items():
                if not isinstance(key, str):
                    error(path, "JSON object keys must be strings")
                all_finite(item, f"{path}.{key}")
        elif isinstance(value, list):
            for index, item in enumerate(value):
                all_finite(item, f"{path}[{index}]")
        elif type(value) in (int, float):
            try:
                finite = math.isfinite(value)
            except OverflowError:
                finite = False
            if not finite:
                error(path, "nonfinite numeric data")
        elif value is not None and type(value) not in (str, bool):
            error(path, "not JSON data")

    t = obj(transition, "transition")
    all_finite(t, "transition")
    if t.get("schemaVersion") != SCHEMA_VERSION:
        error("schemaVersion", f"must be {SCHEMA_VERSION}")
    for key in ("transitionId", "sessionId", "episodeId"):
        string(t, key, "transition")
    metadata = obj(t.get("metadata"), "metadata")
    for key in ("robotId", "configHash", "calibrationHash", "firmwareVersion", "hostVersion", "detectorVersion"):
        string(metadata, key, "metadata")
    for key in ("configHash", "calibrationHash"):
        if not isinstance(metadata.get(key), str) or re.fullmatch(r"[0-9a-f]{64}", metadata[key]) is None:
            error(f"metadata.{key}", "must be lowercase SHA-256 hex")
    policy = obj(t.get("qualityPolicy"), "qualityPolicy")
    max_age = number(policy, "maxSourceAgeMs", "qualityPolicy", 0)
    max_m4_age = number(policy, "maxM4AgeMs", "qualityPolicy", 0)
    settle_ms = number(policy, "minimumSettlingMs", "qualityPolicy", 0)

    def observation(value, path):
        o = obj(value, path)
        for key in ("observationId", "sourceFrameId", "cameraBootId", "sourceClockDomain", "sourceTimestampSemantics", "hostClockId"):
            string(o, key, path)
        digest = string(o, "imageSha256", path)
        if digest is not None and re.fullmatch(r"[0-9a-f]{64}", digest) is None:
            error(f"{path}.imageSha256", "must be lowercase SHA-256 hex")
        image = obj(o.get("image"), f"{path}.image")
        string(image, "artifactRef", f"{path}.image")
        if string(image, "sha256", f"{path}.image") != digest:
            exclude(f"{path}:image_hash_mismatch")
        if o.get("sourceClockDomain") != "camera_boot_us" or o.get("sourceTimestampSemantics") != "first_dma_buffer":
            exclude(f"{path}:unsupported_source_clock")
        known = boolean(o, "sourceTimeKnown", path)
        metadata_valid = boolean(o, "sourceMetadataValid", path)
        fresh = boolean(o, "sourceFresh", path)
        held = boolean(o, "held", path)
        if known is not True or metadata_valid is not True:
            exclude(f"{path}:source_time_unknown")
        if fresh is not True or held is not False:
            exclude(f"{path}:stale_or_held")
        captured = number(o, "capturedUs", path, 1, integer=True)
        boot = o.get("cameraBootId")
        if not isinstance(boot, str) or re.fullmatch(r"[0-9a-f]{32}", boot) is None:
            error(f"{path}.cameraBootId", "must be camera boot hex ID")
        if captured is not None and o.get("sourceFrameId") != f"{boot}:{captured}":
            exclude(f"{path}:source_identity_mismatch")
        age = number(o, "sourceAgeMs", path, 0)
        if age is not None and max_age is not None and age > max_age:
            exclude(f"{path}:source_age_exceeded")
        detection_id = string(o, "detectionSourceFrameId", path)
        if detection_id != o.get("sourceFrameId"):
            exclude(f"{path}:detection_frame_mismatch")
        received = number(o, "receivedMonotonic", path, 0)
        inferred = number(o, "inferenceCompletedMonotonic", path, 0)
        if received is not None and inferred is not None and inferred < received:
            exclude(f"{path}:inference_precedes_receive")
        capture_bounds = bounds(o.get("captureHostBounds"), f"{path}.captureHostBounds")
        if received is not None and capture_bounds[2] is not None and capture_bounds[2] > received:
            exclude(f"{path}:capture_after_receive")
        string(o, "captureHostBoundsBasis", path)
        calibrated = boolean(o, "captureHostBoundsCalibrated", path)
        if calibrated is not True:
            exclude(f"{path}:source_clock_bounds_unverified")
        else:
            uncertainty = number(o, "captureHostBoundsUncertaintySeconds", path, 0)
            string(o, "captureHostBoundsEvidenceRef", path)
            if capture_bounds[0] == o.get("hostClockId") and capture_bounds[1] is not None:
                # Snapshot age must include acquisition and inference latency. The
                # interval already includes request/response uncertainty; explicit
                # clock calibration uncertainty widens the freshness bound further.
                if inferred is not None and age is not None and age + 1e-6 < (inferred - capture_bounds[1]) * 1000:
                    exclude(f"{path}:source_age_understated")
                if age is not None and uncertainty is not None and max_age is not None and age + uncertainty * 1000 > max_age + 1e-6:
                    exclude(f"{path}:source_age_exceeded")
            if uncertainty is not None and None not in capture_bounds:
                capture_bounds = (capture_bounds[0], capture_bounds[1] - uncertainty,
                                  capture_bounds[2] + uncertainty)
        if capture_bounds[0] != o.get("hostClockId"):
            exclude(f"{path}:clock_mismatch")
        m4 = obj(o.get("m4"), f"{path}.m4")
        if m4.get("motor") != 4 or m4.get("sensor") != "AS5600" or m4.get("measured") is not True:
            error(f"{path}.m4", "requires measured M4 AS5600 angle")
        number(m4, "angleDegrees", f"{path}.m4")
        sample_age = number(m4, "sampleAgeMs", f"{path}.m4", 0)
        string(m4, "evidenceRef", f"{path}.m4")
        m4_bounds = bounds(m4.get("sampledHostBounds"), f"{path}.m4.sampledHostBounds", True)
        if m4_bounds[0] != o.get("hostClockId"):
            exclude(f"{path}:m4_clock_mismatch")
        if sample_age is not None and max_m4_age is not None and sample_age > max_m4_age:
            exclude(f"{path}:m4_stale")
        if received is not None and m4_bounds[1] is not None and sample_age is not None:
            conservative_age = (received - m4_bounds[1]) * 1000
            if sample_age + 1e-6 < conservative_age:
                exclude(f"{path}:m4_sample_age_understated")
        # Sample must be from the same observation interval, not a future sensor read.
        if received is not None and m4_bounds[2] is not None and m4_bounds[2] > received:
            exclude(f"{path}:m4_sample_after_receive")
        return o, capture_bounds, captured

    before, before_bounds, before_us = observation(t.get("before"), "before")
    after, after_bounds, after_us = observation(t.get("after"), "after")
    if before.get("cameraBootId") != after.get("cameraBootId"):
        exclude("camera_reboot_between_observations")
    if before.get("sourceFrameId") == after.get("sourceFrameId") or before.get("observationId") == after.get("observationId"):
        exclude("reused_observation")
    if before_us is not None and after_us is not None and after_us <= before_us:
        exclude("source_capture_not_advancing")

    action = obj(t.get("action"), "action")
    request = obj(action.get("request"), "action.request")
    record = obj(action.get("record"), "action.record")
    for key in ("bootId", "commandId", "kind"):
        string(record, key, "action.record")
        string(request, key, "action.request")
    for key, low, high in (("motor", 1, 5), ("requestedPwm", -255, 255), ("durationMs", 1, 10000), ("percent", 0, 100)):
        number(record, key, "action.record", low, high, True)
    number(record, "targetDegrees", "action.record")
    for key, low, high in (("motor", 1, 5), ("pwm", -255, 255), ("durationMs", 1, 10000), ("percent", 0, 100)):
        number(request, key, "action.request", low, high, True)
    number(request, "targetDegrees", "action.request")
    for key in ("bootId", "commandId", "kind", "motor", "durationMs", "targetDegrees", "percent"):
        if key not in request or request.get(key) != record.get(key):
            error(f"action.request.{key}", "must match accepted record")
    if "pwm" not in request or request.get("pwm") != record.get("requestedPwm"):
        error("action.request.pwm", "must match accepted record")
    if record.get("kind") not in ("pulse", "shoulder"):
        error("action.record.kind", "unsupported action kind")
    if record.get("kind") == "pulse" and (record.get("motor") == 4 or record.get("requestedPwm") == 0 or record.get("targetDegrees") != 0 or record.get("percent") != 0):
        error("action.record", "invalid bounded pulse request")
    if record.get("kind") == "shoulder" and (record.get("motor") != 4 or record.get("requestedPwm") != 0 or record.get("percent") == 0):
        error("action.record", "invalid shoulder request")
    ordinal = number(record, "ordinal", "action.record", 1, UINT32 - 1, True)
    start_ms = number(record, "startedAtMs", "action.record", 0, UINT32 - 1, True)
    end_ms = number(record, "endedAtMs", "action.record", 0, UINT32 - 1, True)
    elapsed = number(record, "elapsedMs", "action.record", 0, UINT32 - 1, True)
    overrun = number(record, "overrunMs", "action.record", 0, UINT32 - 1, True)
    target_reached = boolean(record, "targetReached", "action.record")
    if boolean(record, "outputOff", "action.record") is not True:
        exclude("output_off_unconfirmed")
    state = string(record, "state", "action.record")
    if state not in ("running", "completed", "timed_out", "cancelled", "failed", "outcome_unknown"):
        error("action.record.state", "unsupported action state")
    if state not in ("completed", "timed_out", "cancelled", "failed"):
        exclude("action_not_known_terminal")
    reason = string(record, "reason", "action.record")
    if None not in (start_ms, end_ms, elapsed) and (end_ms - start_ms) % UINT32 != elapsed:
        exclude("action_elapsed_mismatch")
    duration = record.get("durationMs")
    if elapsed is not None and overrun is not None and type(duration) is int:
        if overrun != max(0, elapsed - duration):
            exclude("action_overrun_mismatch")
        if state == "completed" and overrun > 0:
            exclude("completed_action_overran_deadline")
        # Match the production Executor terminal transitions. In particular, a
        # pulse cannot finish successfully before its deadline. A shoulder may
        # reach its target before the deadline yet time out while stopping; that
        # legitimate timed_out + targetReached result must remain recordable.
        kind = record.get("kind")
        if state == "completed":
            if kind == "pulse" and (reason != "duration_elapsed" or elapsed != duration or target_reached is not False):
                exclude("action_terminal_state_inconsistent")
            if kind == "shoulder" and (reason != "target_reached" or target_reached is not True):
                exclude("action_terminal_state_inconsistent")
        elif state == "timed_out":
            if (reason != "deadline_exceeded" or elapsed < duration
                    or (kind == "pulse" and (elapsed == duration or target_reached is not False))
                    or (target_reached is True and (kind != "shoulder" or overrun == 0))):
                exclude("action_terminal_state_inconsistent")
        elif state == "failed":
            if (reason not in ("start_failed", "safety_blocked", "controller_failed")
                    or (reason == "controller_failed" and kind != "shoulder")
                    or target_reached is not False):
                exclude("action_terminal_state_inconsistent")
        elif state == "cancelled":
            if reason not in ("cancelled", "operator_override", "state_changed") or target_reached is not False:
                exclude("action_terminal_state_inconsistent")
    start_bounds = bounds(action.get("startHostBounds"), "action.startHostBounds", True)
    off_bounds = bounds(action.get("outputOffHostBounds"), "action.outputOffHostBounds", True)
    # Mapping evidence is a recorded claim, not verified clock calibration.
    # Do not compare MCU elapsed milliseconds to host intervals without an
    # explicit bound on their clock-rate relationship.
    if len({before_bounds[0], after_bounds[0], start_bounds[0], off_bounds[0]}) != 1:
        exclude("host_clock_mismatch")
    if None not in (before_bounds[2], start_bounds[1]) and before_bounds[2] >= start_bounds[1]:
        exclude("before_not_proven_before_action")
    if None not in (start_bounds[1], off_bounds[2]) and start_bounds[1] > off_bounds[2]:
        exclude("action_host_order_invalid")
    if None not in (after_bounds[1], off_bounds[2], settle_ms) and after_bounds[1] <= off_bounds[2] + settle_ms / 1000:
        exclude("after_not_proven_after_output_off_and_settling")

    for name, observation_data, bound, edge in (("before", before, start_bounds, "latest"), ("after", after, off_bounds, "earliest")):
        m4 = observation_data.get("m4")
        sample_bounds = m4.get("sampledHostBounds") if isinstance(m4, dict) else None
        sample_time = sample_bounds.get(edge) if isinstance(sample_bounds, dict) else None
        if type(sample_time) in (int, float):
            if name == "before" and bound[1] is not None and sample_time >= bound[1]:
                exclude("before:m4_not_proven_before_action")
            if name == "after" and bound[2] is not None and sample_time <= bound[2]:
                exclude("after:m4_not_proven_after_output_off")

    log = obj(action.get("outputLog"), "action.outputLog")
    if log.get("bootId") != record.get("bootId"):
        exclude("output_boot_mismatch")
    if log.get("evidence") != "driver_write" or log.get("clockDomain") != "controller_boot_millis":
        error("action.outputLog", "requires MCU driver-write evidence and clock domain")
    dropped = number(log, "dropped", "action.outputLog", 0, integer=True)
    if dropped != 0:
        exclude("output_history_dropped")
    coverage = obj(action.get("outputSequenceBounds"), "action.outputSequenceBounds")
    first = number(coverage, "first", "action.outputSequenceBounds", 1, UINT32 - 1, True)
    last = number(coverage, "last", "action.outputSequenceBounds", 1, UINT32 - 1, True)
    string(coverage, "evidenceRef", "action.outputSequenceBounds")
    entries = log.get("entries")
    if not isinstance(entries, list) or not entries:
        error("action.outputLog.entries", "requires nonempty output trace")
        entries = []
    sequences, offsets, selected = [], [], []
    for index, value in enumerate(entries):
        path = f"action.outputLog.entries[{index}]"
        entry = obj(value, path)
        sequence = number(entry, "sequence", path, 1, UINT32 - 1, True)
        at = number(entry, "atMs", path, 0, UINT32 - 1, True)
        number(entry, "actionOrdinal", path, 1, UINT32 - 1, True)
        number(entry, "motor", path, 1, 5, True)
        pwm = number(entry, "pwm", path, -255, 255, True)
        if entry.get("reason") not in ("driver_write", "forced_off"):
            error(path, "unsupported output reason")
        if entry.get("reason") == "forced_off" and pwm != 0:
            exclude("nonzero_forced_off")
        if entry.get("actionOrdinal") != ordinal:
            exclude("output_action_mismatch")
        if entry.get("motor") != record.get("motor") and pwm != 0:
            exclude("other_motor_active")
        if sequence is not None:
            sequences.append(sequence)
        if at is not None and start_ms is not None:
            offset = (at - start_ms) % UINT32
            offsets.append(offset)
            if elapsed is not None and offset > elapsed:
                exclude("output_outside_action_interval")
        if entry.get("motor") == record.get("motor"):
            selected.append(entry)
    if sequences:
        if sequences[0] != first or sequences[-1] != last or any(b != a + 1 for a, b in zip(sequences, sequences[1:])):
            exclude("output_sequence_gap")
    if offsets != sorted(offsets):
        exclude("output_time_order_invalid")
    if not selected or selected[-1].get("pwm") != 0:
        exclude("terminal_zero_write_missing")

    outcome = obj(t.get("outcome"), "outcome")
    if "taskSuccess" not in outcome or (outcome.get("taskSuccess") is not None and type(outcome.get("taskSuccess")) is not bool):
        error("outcome.taskSuccess", "requires boolean or explicit null")
    if outcome.get("taskSuccess") is not None:
        source = string(outcome, "labelSource", "outcome")
        string(outcome, "evidenceRef", "outcome")
        if source in ("outputOff", "targetReached", "driver_write"):
            error("outcome.labelSource", "controller state does not establish task success")
    if errors:
        exclude("schema_invalid")
    return {"valid": not errors, "schemaErrors": errors,
            "trainingEligible": not errors and not excluded, "exclusionReasons": excluded}
