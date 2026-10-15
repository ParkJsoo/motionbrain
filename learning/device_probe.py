"""Durable non-driving collection probe using the existing artifact store.

The distinct schema cannot be consumed as an action-transition dataset. No
action intent, zero-action training example, or physical calibration is invented.
"""
from __future__ import annotations

import math
import time
import uuid

from learning.artifacts import StorageError, read_events, read_json_artifact, verify_artifacts
from learning.device_adapter import DeviceReadAdapter, DeviceReadError
from learning.runner import DEFAULT_POLICY


def _idle(status):
    motors = status.get("motors", {})
    return (status.get("state") == "IDLE" and status.get("motorEnabled") is False
            and set(motors) == {"M1", "M2", "M3", "M4", "M5"}
            and all(m.get("speed") == 0 and m.get("enabled") is False for m in motors.values()))


def observation_blockers(observation, age_upper_bound=None):
    """Report collection-policy blockers without treating display freshness as approval."""
    reasons = []
    for field in ("sourceTimeKnown", "sourceMetadataValid", "sourceFresh", "controlEligible"):
        if observation.get(field) is not True:
            reasons.append(field + "_required")
    if observation.get("held") is not False:
        reasons.append("held_observation")
    if observation.get("detectionSourceFrameId") != observation.get("sourceFrameId"):
        reasons.append("detection_source_mismatch")
    age = observation.get("sourceAgeMs") if age_upper_bound is None else age_upper_bound
    if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= DEFAULT_POLICY["maxSourceAgeMs"]:
        reasons.append("source_age_exceeds_collection_policy")
    if observation.get("captureHostBoundsCalibrated") is not True:
        reasons.append("source_clock_unverified")
    else:
        uncertainty = observation.get("captureHostBoundsUncertaintySeconds")
        if type(uncertainty) not in (int, float) or not math.isfinite(uncertainty) or uncertainty < 0:
            reasons.append("source_clock_uncertainty_invalid")
        elif (type(age) in (int, float) and math.isfinite(age)
              and age + uncertainty * 1000 > DEFAULT_POLICY["maxSourceAgeMs"]):
            reasons.append("source_age_with_clock_uncertainty_exceeds_collection_policy")
    # Even a calibrated producer has not established a common action/observation
    # host clock or simultaneous M4 sample in this read-only probe.
    return reasons


def run_device_probe(store, transport, *, samples=12, interval=0.5, sleep=time.sleep):
    if type(samples) is not int or not 2 <= samples <= 100:
        raise ValueError("samples must be between 2 and 100")
    if type(interval) not in (int, float) or not math.isfinite(interval) or not 0 <= interval <= 10:
        raise ValueError("interval must be between 0 and 10 seconds")
    adapter = DeviceReadAdapter(transport)
    session_id = uuid.uuid4().hex
    artifacts, observed, errors, blockers = [], [], [], set()
    previous_recorder = transport.recorder

    def save_json(path, value):
        ref = store.write_json(path, value)
        artifacts.append(ref)
        return ref

    def save_image(index, phase, body, digest):
        ref = store.save_image(str(index), phase, body, digest)
        artifacts.append(ref)
        return ref

    receipts = 0

    def record(receipt):
        nonlocal receipts
        receipts += 1
        ref = save_json(f"raw/http-{receipts:04d}.json", receipt)
        store.append("device_read", session_id, {"receipt": ref})
        if previous_recorder:
            previous_recorder(receipt)

    session = {"schemaVersion": "motionbrain.device_probe_session.v1", "sessionId": session_id,
               "mode": "device_read_only", "evidence": "device_observation",
               "trainingEligible": False, "automaticRetry": False, "resumeSupported": False,
               "hostClockId": adapter.host_clock_id, "endpoints": transport.bases,
               "qualityPolicy": DEFAULT_POLICY, "requestedSamples": samples}
    save_json("session.json", session)
    store.append("probe_started", session_id, {})
    transport.recorder = record
    state, start, end, initial_outputs, final_outputs = "completed", None, None, None, None
    lookup = None
    try:
        start = adapter.capability()
        initial_outputs = adapter.outputs()
        if initial_outputs["dropped"]:
            blockers.add("initial_output_log_incomplete")
        # Random ID is looked up only; it is never submitted or journal-reserved.
        lookup = adapter.lookup_action({"bootId": start["bootId"], "commandId": "probe-" + uuid.uuid4().hex})
        if lookup.get("lookupStatus") != "not_found":
            raise DeviceReadError("missing_action_lookup_not_verified")
        if not start["enabled"]:
            blockers.add("bounded_actions_disabled")
        if start["active"]:
            raise DeviceReadError("controller_action_active")
        for index in range(samples):
            status = adapter.status()
            if not _idle(status):
                raise DeviceReadError("controller_not_idle_or_outputs_nonzero")
            sensor, angle = status.get("sensor", {}), status.get("shoulderAngle", {})
            if (sensor.get("connected") is not True or sensor.get("imuOk") is not True
                    or sensor.get("simulated") is not False or sensor.get("faultLatched") is not False):
                blockers.add("stm32_sensor_not_ready")
            if not all(angle.get(k) is True for k in ("sensorConnected", "sensorFresh", "sensorReady")):
                blockers.add("m4_sensor_not_ready")
            m4_age = angle.get("ageMs")
            if (type(m4_age) not in (int, float) or not math.isfinite(m4_age)
                    or not 0 <= m4_age <= DEFAULT_POLICY["maxM4AgeMs"]):
                blockers.add("m4_status_age_exceeds_collection_policy")
            reading = adapter.observe(lambda body, digest: save_image(index, "perception", body, digest))
            save_json(f"observations/{index:04d}.json", reading)
            observation = reading["observation"]
            blockers.update(observation_blockers(observation, reading["sourceAgeUpperBoundAtRetrievalMs"]))
            camera = adapter.sample_camera_clock(lambda body, digest: save_image(index, "camera", body, digest))
            save_json(f"clock/{index:04d}.json", camera)
            if observation["cameraBootId"] != camera["cameraBootId"]:
                raise DeviceReadError("perception_camera_boot_mismatch")
            observed.append({"sourceFrameId": observation["sourceFrameId"],
                             "sourceAgeMs": observation.get("sourceAgeMs"),
                             "sourceAgeUpperBoundAtRetrievalMs": reading["sourceAgeUpperBoundAtRetrievalMs"],
                             "detectorLatencyMs": observation.get("detector", {}).get("latencyMs"),
                             "m4AngleDegrees": angle.get("angleDeg"),
                             "m4StatusAgeMs": angle.get("ageMs"), "cameraRttMs": camera["rttMs"]})
            store.append("probe_sample", session_id, {"index": index})
            if index + 1 < samples:
                sleep(interval)
        final_outputs = adapter.outputs()
        end = adapter.capability()
        # Historical nonzero writes are retained. Only an unchanged window proves
        # that no new driver writes occurred while this probe was watching.
        if final_outputs != initial_outputs or end != start:
            raise DeviceReadError("controller_changed_during_probe")
        if not _idle(adapter.status()):
            raise DeviceReadError("controller_not_idle_at_end")
    except StorageError:
        # Durable prefix remains auditable; do not pretend a complete report.
        raise
    except KeyboardInterrupt:
        state = "interrupted"
        errors.append({"type": "KeyboardInterrupt", "reason": "operator_interrupt"})
    except Exception as exc:
        state = "aborted"
        errors.append({"type": type(exc).__name__, "reason": str(exc) if isinstance(exc, DeviceReadError) else "device_read_failed"})
    finally:
        transport.recorder = previous_recorder
    clock = adapter.clock_report()
    if not clock["constantOffsetConsistent"]:
        blockers.add("clock_probe_insufficient_or_inconsistent")
    blockers.update(("no_physical_action", "no_action_clock_or_m4_alignment", "physical_clock_calibration_pending"))
    issues = verify_artifacts(store.root, artifacts)
    if issues:
        state = "aborted"
        errors.append({"type": "artifact_integrity", "reason": ";".join(issues)})
    report = {"schemaVersion": "motionbrain.device_probe.v1", "sessionId": session_id,
              "mode": "device_read_only", "evidence": "device_observation", "status": state,
              "trainingEligible": False, "readyForPhysicalCollection": False,
              "executionState": "not_sent", "automaticRetry": False, "motorCommandsSent": 0,
              "getRequests": receipts, "completedSamples": len(observed), "samples": observed,
              "clock": clock, "capabilityBefore": start, "capabilityAfter": end,
              "outputLogUnchanged": initial_outputs is not None and final_outputs == initial_outputs,
              "missingActionLookup": lookup, "blockers": sorted(blockers), "errors": errors,
              "artifacts": artifacts}
    ref = store.write_json("run.json", report)
    store.append("probe_finished", session_id, {"summary": ref})
    return report


def audit_device_probe(run_dir):
    """Read-only audit of artifacts and terminal event; no adapter construction."""
    events, issues = read_events(run_dir)
    if not events or events[0].get("kind") != "probe_started":
        issues.append("probe_start_missing")
    if not events or events[-1].get("kind") != "probe_finished":
        issues.append("probe_terminal_missing")
        return {"ok": False, "issues": issues, "trainingEligible": False}
    payload = events[-1].get("payload")
    ref = payload.get("summary", {}) if isinstance(payload, dict) else {}
    issues.extend(verify_artifacts(run_dir, [ref]))
    try:
        report = read_json_artifact(run_dir, ref)
        if not isinstance(report, dict):
            raise StorageError("probe report must be an object")
        if (report.get("schemaVersion") != "motionbrain.device_probe.v1"
                or report.get("mode") != "device_read_only" or report.get("trainingEligible") is not False
                or report.get("motorCommandsSent") != 0):
            issues.append("probe_contract_invalid")
        sid = report.get("sessionId")
        if not sid or any(event.get("transitionId") != sid for event in events):
            issues.append("probe_session_mismatch")
        issues.extend(verify_artifacts(run_dir, report.get("artifacts", [])))
    except (StorageError, KeyError, TypeError):
        issues.append("probe_report_unreadable")
    return {"ok": not issues, "issues": issues, "trainingEligible": False}
