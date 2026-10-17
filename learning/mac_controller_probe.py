"""Finite camera + controller observation session. No command transport exists.

This preserves separate CoreMedia, Python-monotonic and MCU clock domains.
Process overlap is not exposure/sensor synchronization or an action transition.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from learning.artifacts import (ArtifactStore, StorageError, _decode_json,
                                describe_artifact, read_json_artifact, verify_artifacts)
from learning.mac_camera_probe import create_plan, evaluate_capture
from tools.diagnose_controller_http import idle_error, validate_target

SCHEMA = "motionbrain.mac_controller_probe.v1"
POLICY = dict(statusSamples=6, maxRequests=14, requestTimeoutSeconds=.8,
              sampleIntervalSeconds=.025, sessionSeconds=50, cameraProcessSeconds=47,
              cleanupGraceSeconds=2, maxPersistenceMs=250, maxSourceAgeMs=100,
              maxM4AgeMs=20, maxAnchorAgeMs=15000, retries=0, motorRequests=0)
ENDPOINTS = ("status", "sequence", "actions", "action_outputs")
ROOT = Path(__file__).resolve().parents[1]


class ProbeRejected(RuntimeError):
    pass


def read_controller(url, endpoint, *, clock=time):
    """A bounded curl process, not an unbounded per-socket read timeout."""
    url = validate_target(url)
    if endpoint not in ENDPOINTS:
        raise ValueError("read_only_endpoint_required")
    started = clock.monotonic()
    result = subprocess.run([
        "curl", "--disable", "--globoff", "--noproxy", "*", "--proto", "=http",
        "--connect-timeout", ".8", "--max-time", ".8", "--max-filesize", "65536",
        "--silent", "--show-error", "--write-out", "\n%{http_code}", url+"/"+endpoint,
    ], capture_output=True, timeout=1.2, check=False)
    ended = clock.monotonic()
    body, _, status = result.stdout.rpartition(b"\n")
    receipt = dict(endpoint=endpoint, method="GET", clockDomain="python_monotonic",
                   requestStartedMonotonic=started, receivedMonotonic=ended,
                   elapsedMs=(ended-started)*1000, curlExit=result.returncode,
                   httpStatus=status.decode("ascii", errors="replace"),
                   responseBody=body.decode("utf-8", errors="replace"),
                   bodySha256=hashlib.sha256(body).hexdigest())
    try:
        receipt["response"] = _decode_json(body)
    except (ValueError, UnicodeError):
        receipt["response"] = None
    return receipt


def _check(receipt, endpoint):
    completed = isinstance(receipt, dict) and (
        (receipt.get("transport", "curl") == "curl" and receipt.get("curlExit") == 0)
        or (receipt.get("transport") == "python_http_client" and receipt.get("transportCompleted") is True))
    if (not isinstance(receipt, dict) or receipt.get("endpoint") != endpoint or receipt.get("method") != "GET"
            or not completed or receipt.get("httpStatus") != "200"
            or type(receipt.get("elapsedMs")) not in (int, float)
            or not 0 <= receipt["elapsedMs"] <= 800):
        raise ProbeRejected("controller_request_failed_or_exceeded_800ms")
    start, end = receipt.get("requestStartedMonotonic"), receipt.get("receivedMonotonic")
    if (not all(_number(x) for x in (start, end)) or end < start
            or receipt.get("clockDomain") != "python_monotonic"
            or abs((end-start)*1000-receipt["elapsedMs"]) > .001):
        raise ProbeRejected("controller_receipt_clock_invalid")
    body = receipt.get("responseBody")
    try:
        if (not isinstance(body, str) or hashlib.sha256(body.encode()).hexdigest() != receipt.get("bodySha256")
                or _decode_json(body.encode()) != receipt.get("response")):
            raise ValueError()
    except (ValueError, UnicodeError):
        raise ProbeRejected("controller_body_binding_invalid") from None
    value = receipt.get("response")
    if endpoint == "actions":
        # Both feature configurations are observable; neither enables writes.
        if (not isinstance(value, dict) or value.get("active") is not False
                or type(value.get("enabled")) is not bool
                or not isinstance(value.get("bootId"), str) or not value["bootId"]):
            raise ProbeRejected("controller_action_active_or_unknown")
    else:
        problem = idle_error(endpoint, value)
        if problem:
            raise ProbeRejected(problem)
    if endpoint == "status":
        sensor, m4 = value.get("sensor", {}), value.get("shoulderAngle", {})
        if (not isinstance(sensor, dict) or not isinstance(m4, dict)
                or any(sensor.get(k) is not True for k in ("connected", "imuOk"))
                or any(sensor.get(k) is not False for k in ("simulated", "faultLatched", "blocked"))
                or any(m4.get(k) is not True for k in ("available", "sensorConnected", "sensorFresh", "sensorReady"))
                or any(m4.get(k) is not False for k in ("active", "correctionActive"))):
            raise ProbeRejected("sensor_not_ready_or_motion_active")
        if not _number(m4.get("ageMs")) or not _number(m4.get("angleDeg")):
            raise ProbeRejected("m4_measurement_invalid")
    return value


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value) and value >= 0
    except OverflowError:
        return False


def _sample(row, ordinal):
    m4 = row["response"]["shoulderAngle"]
    upper = m4["ageMs"] + row["elapsedMs"]
    return dict(requestOrdinal=ordinal, angleDegrees=m4["angleDeg"], reportedAgeMs=m4["ageMs"],
                transportInclusiveAgeUpperMs=upper, within20ms=upper <= 20)


def launch_camera(helper, root, device_id, view_id):
    stdout = (root/"camera-wrapper.stdout").open("xb")
    try:
        stderr = (root/"camera-wrapper.stderr").open("xb")
        try:
            return subprocess.Popen([
                sys.executable, str(ROOT/"tools/probe_mac_camera.py"), "capture",
                "--helper", str(Path(helper).resolve(strict=True)), "--device-id", device_id,
                "--view-id", view_id, "--output", str(root/"camera"),
            ], stdout=stdout, stderr=stderr, start_new_session=True)
        finally:
            stderr.close()
    finally:
        stdout.close()


def stop_camera(process):
    # The wrapper and native child share a process group. Do not leave the
    # camera helper alive when the wrapper exits first or the parent aborts.
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=POLICY["cleanupGraceSeconds"])
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=POLICY["cleanupGraceSeconds"])


def run_probe(output, *, controller_url, helper, device_id, view_id,
              read=read_controller, launch=launch_camera, stop=stop_camera, clock=time, clock_probe=None):
    """One new run; refuse on state change, failed storage, timeout or early exit."""
    controller_url = validate_target(controller_url)
    create_plan(device_id, view_id)  # Reject malformed identifiers before device IO.
    helper = Path(helper).resolve(strict=True)
    if not helper.is_file() or not os.access(helper, os.X_OK):
        raise ValueError("executable_camera_helper_required")
    store = ArtifactStore(Path(output))
    refs, receipts, samples = [], [], []
    camera = None
    clock_id = uuid.uuid4().hex
    start = clock.monotonic()
    def budget():
        if clock.monotonic()-start > POLICY["sessionSeconds"]:
            raise ProbeRejected("session_deadline")
    def save(path, value):
        begun = clock.monotonic()
        ref = store.write_json(path, value)
        refs.append(ref)
        if (clock.monotonic()-begun)*1000 > POLICY["maxPersistenceMs"]:
            raise StorageError("probe_persistence_exceeds_250ms")
        return ref
    def get(endpoint, phase):
        budget()
        if len(receipts) >= POLICY["maxRequests"]:
            raise ProbeRejected("request_budget_exhausted")
        try:
            row = read(controller_url, endpoint, clock=clock)
        except (OSError, subprocess.TimeoutExpired) as exc:
            row = dict(endpoint=endpoint, method="GET", error=type(exc).__name__)
        row["phase"] = phase
        row["hostClockId"] = clock_id
        receipts.append(row)
        save(f"controller/{len(receipts):02d}-{endpoint}.json", row)
        return _check(row, endpoint)

    save("plan.json", dict(schemaVersion=SCHEMA, policy=POLICY, controllerUrl=controller_url,
                           viewId=view_id, deviceId=device_id, trainingEligible=False,
                           controlEligible=False, clockDomainsMerged=False, hostClockId=clock_id,
                           clockBridgeRequested=clock_probe is not None))
    result = dict(schemaVersion=SCHEMA, status="rejected", failure=None, samples=samples,
                  trainingEligible=False, controlEligible=False, motorRequests=0,
                  clockDomainsMerged=False, framesPairedWithSensors=False, automaticRetry=False,
                  observationStartedMonotonic=start)
    try:
        if clock_probe is not None:
            bridge_before=clock_probe.sample(clock_id)
            save('clock/before.json',bridge_before)
        baseline = {e:get(e,"baseline") for e in ENDPOINTS}
        if baseline["actions"]["bootId"] != baseline["action_outputs"]["bootId"]:
            raise ProbeRejected("baseline_boot_mismatch")
        camera_started = clock.monotonic()
        camera = launch(helper, store.root, device_id, view_id)
        result["cameraProcessStartedMonotonic"] = camera_started
        while not (store.root/"camera/frame-0000.json").is_file():
            budget()
            if camera.poll() is not None:
                raise ProbeRejected("camera_exited_before_first_frame")
            if clock.monotonic()-camera_started > POLICY["cameraProcessSeconds"]:
                raise ProbeRejected("camera_deadline")
            clock.sleep(.02)
        result["firstFrameFileObservedMonotonic"] = clock.monotonic()
        for i in range(POLICY["statusSamples"]):
            if camera.poll() is not None:
                raise ProbeRejected("camera_process_ended_before_status_samples")
            get("status", "during_camera_process")
            samples.append(_sample(receipts[-1], len(receipts)))
            if i+1 < POLICY["statusSamples"]:
                clock.sleep(POLICY["sampleIntervalSeconds"])
        remaining = min(POLICY["sessionSeconds"]-(clock.monotonic()-start),
                        POLICY["cameraProcessSeconds"]-(clock.monotonic()-camera_started))
        if remaining <= 0:
            raise ProbeRejected("camera_deadline")
        camera.wait(timeout=remaining)
        result["cameraProcessEndedMonotonic"] = clock.monotonic()
        camera_report = evaluate_capture(store.root/"camera")
        result["camera"] = camera_report
        if camera.returncode != 0 or not camera_report["captureProcessingPassed"]:
            raise ProbeRejected("camera_processing_not_passed")
        final = {e:get(e,"final") for e in ENDPOINTS}
        if final["actions"]["bootId"] != baseline["actions"]["bootId"]:
            raise ProbeRejected("controller_boot_changed")
        if final["action_outputs"] != baseline["action_outputs"]:
            raise ProbeRejected("output_history_changed")
        if clock_probe is not None:
            from learning.mac_clock_bridge import link_saved_frames
            bridge_after=clock_probe.sample(clock_id)
            save('clock/after.json',bridge_after)
            linked=link_saved_frames(store.root,bridge_before,bridge_after,receipts[4:10],clock_id,
                                    [camera_started,result['cameraProcessEndedMonotonic']])
            result['clockLink']=save('clock/link.json',linked)
        budget()
        result.update(status="recorded", outputHistoryUnchanged=True,
                      m4Within20ms=sum(x["within20ms"] for x in samples))
    except StorageError:
        raise
    except (ProbeRejected, OSError, ValueError, subprocess.SubprocessError) as exc:
        result["status"] = "rejected"
        result["failure"] = str(exc) if isinstance(exc, ProbeRejected) else type(exc).__name__
    finally:
        # Observation budget excludes process cleanup and final offline sealing.
        result["observationEndedMonotonic"] = clock.monotonic()
        result["observationElapsedSeconds"] = result["observationEndedMonotonic"]-start
        if camera is not None:
            stop(camera)
    result["requestCount"] = len(receipts)
    result["elapsedSeconds"] = clock.monotonic()-start
    # Store references for all original native camera files, including failures.
    camera_root = store.root/"camera"
    if camera_root.is_dir():
        for p in sorted(camera_root.iterdir()):
            refs.append(describe_artifact(store.root, "camera/"+p.name))
    for name in ("camera-wrapper.stdout", "camera-wrapper.stderr"):
        if (store.root/name).exists():
            refs.append(describe_artifact(store.root, name))
    result["artifacts"] = list(refs)
    summary = save("result.json", result)
    store.append("probe_sealed", "observation", {"summary":summary})
    return result


def audit_probe(root):
    """No device access. Recheck immutable bytes and independently audit camera."""
    from learning.artifacts import read_events
    root = Path(root)
    events, issues = read_events(root)
    try:
        if len(events) != 1 or events[0].get("kind") != "probe_sealed":
            raise StorageError("probe_seal_missing")
        result = read_json_artifact(root, events[0]["payload"]["summary"])
        issues += verify_artifacts(root, result["artifacts"])
        descriptors = {ref["artifactRef"]:ref for ref in result["artifacts"]}
        if len(descriptors) != len(result["artifacts"]):
            raise ValueError("duplicate_artifacts")
        plan = read_json_artifact(root, descriptors["plan.json"])
        if (plan.get("schemaVersion") != SCHEMA or plan.get("policy") != POLICY
                or not isinstance(plan.get("hostClockId"), str) or not plan["hostClockId"]):
            issues.append("invalid_probe_plan")
        if (result.get("schemaVersion") != SCHEMA or result.get("trainingEligible") is not False
                or result.get("controlEligible") is not False or result.get("motorRequests") != 0
                or result.get("clockDomainsMerged") is not False or result.get("framesPairedWithSensors") is not False
                or result.get("automaticRetry") is not False or result.get("status") not in ("recorded", "rejected")):
            issues.append("invalid_probe_contract")
        camera = evaluate_capture(root/"camera") if (root/"camera").is_dir() else None
        if result.get("status") == "recorded":
            required_camera = {"camera/"+name for name in ("plan.json", "execution.json", "session.json", "terminal.json")}
            required_camera |= {f"camera/frame-{i:04d}.{ext}" for i in range(21) for ext in ("json", "bgra")}
            if not required_camera.issubset(descriptors):
                issues.append("camera_artifact_bindings_missing")
            if not camera or not camera["captureProcessingPassed"]:
                issues.append("camera_audit_failed")
            camera_plan = read_json_artifact(root, descriptors["camera/plan.json"])
            if (camera_plan.get("viewConfigurationId") != plan.get("viewId")
                    or camera_plan.get("deviceUniqueId") != plan.get("deviceId")):
                issues.append("camera_identity_mismatch")
            expected = [(e, "baseline") for e in ENDPOINTS] + [("status", "during_camera_process")]*6 + [(e, "final") for e in ENDPOINTS]
            paths = [f"controller/{i:02d}-{e}.json" for i, (e, _) in enumerate(expected, 1)]
            if sorted(p for p in descriptors if p.startswith("controller/")) != sorted(paths):
                raise ValueError("controller_receipt_set_invalid")
            rows = [read_json_artifact(root, descriptors[p]) for p in paths]
            previous = -1
            for row, (endpoint, phase) in zip(rows, expected):
                _check(row, endpoint)
                if (row.get("phase") != phase or row.get("hostClockId") != plan["hostClockId"]
                        or row["requestStartedMonotonic"] < previous):
                    raise ValueError("controller_receipt_order_invalid")
                previous = row["receivedMonotonic"]
            if (rows[2]["response"]["bootId"] != rows[3]["response"]["bootId"]
                    or rows[2]["response"]["bootId"] != rows[12]["response"]["bootId"]
                    or rows[3]["response"] != rows[13]["response"]):
                issues.append("controller_boot_or_output_changed")
            samples = [_sample(row, i) for i, row in enumerate(rows[4:10], 5)]
            if (result.get("samples") != samples or result.get("requestCount") != 14
                    or result.get("m4Within20ms") != sum(s["within20ms"] for s in samples)
                    or result.get("outputHistoryUnchanged") is not True or result.get("failure") is not None):
                issues.append("cached_controller_summary_mismatch")
            timing = [result.get(k) for k in ("cameraProcessStartedMonotonic", "firstFrameFileObservedMonotonic", "cameraProcessEndedMonotonic")]
            if (not all(_number(x) for x in timing) or timing != sorted(timing)
                    or not rows[3]["receivedMonotonic"] <= timing[0] <= timing[1] <= rows[4]["requestStartedMonotonic"]
                    or not rows[9]["receivedMonotonic"] <= timing[2] <= rows[10]["requestStartedMonotonic"]
                    or timing[2]-timing[0] > POLICY["cameraProcessSeconds"]):
                issues.append("camera_process_overlap_invalid")
            started, ended = result.get("observationStartedMonotonic"), result.get("observationEndedMonotonic")
            elapsed, total = result.get("observationElapsedSeconds"), result.get("elapsedSeconds")
            if (not all(_number(x) for x in (started, ended, elapsed, total))
                    or not started <= rows[0]["requestStartedMonotonic"] <= rows[-1]["receivedMonotonic"] <= ended
                    or abs(ended-started-elapsed) > .000001 or elapsed > POLICY["sessionSeconds"]
                    or total < elapsed):
                issues.append("observation_deadline_or_timing_invalid")
            if plan.get('clockBridgeRequested') is True:
                from learning.mac_clock_bridge import link_saved_frames
                before=read_json_artifact(root,descriptors['clock/before.json'])
                after=read_json_artifact(root,descriptors['clock/after.json'])
                linked=read_json_artifact(root,descriptors['clock/link.json'])
                if (result.get('clockLink')!=descriptors['clock/link.json'] or linked!=link_saved_frames(
                        root,before,after,rows[4:10],plan['hostClockId'],[timing[0],timing[2]])):
                    issues.append('clock_link_recomputation_mismatch')
            elif result.get('clockLink') is not None:
                issues.append('unexpected_clock_link')
        return dict(ok=not issues, issues=issues, status=result["status"], camera=camera,
                    trainingEligible=False, controlEligible=False, framesPairedWithSensors=False)
    except (StorageError, ProbeRejected, OSError, ValueError, KeyError, TypeError, AttributeError) as exc:
        return dict(ok=False, issues=issues+[type(exc).__name__], trainingEligible=False,
                    controlEligible=False, framesPairedWithSensors=False)
