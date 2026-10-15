"""Read-only shadow measurements with original JPEGs and durable failure records."""
import hashlib
from pathlib import Path
import time

import cv2
import numpy as np

from learning.artifacts import verify_artifacts
from learning.device_adapter import DeviceReadAdapter, DeviceReadError
from learning.device_probe import _idle
from learning.fast_observation import AnchoredRoiMeasurement
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def run_fast_probe(store, transport, *, samples=8, interval=0.1, sleep=time.sleep):
    if type(samples) is not int or not 1 <= samples <= 100:
        raise ValueError("samples must be 1..100")
    if type(interval) not in (int, float) or not 0 <= interval <= 1:
        raise ValueError("interval must be 0..1 seconds")
    artifacts, results = [], []
    adapter = DeviceReadAdapter(transport)
    original_recorder, receipts = transport.recorder, 0

    def save_json(path, value):
        ref = store.write_json(path, value)
        artifacts.append(ref)
        return ref

    def save_image(index, phase, body, digest):
        ref = store.save_image(str(index), phase, body, digest)
        artifacts.append(ref)
        return ref

    def record(receipt):
        nonlocal receipts
        receipts += 1
        save_json(f"raw/{receipts:04d}.json", receipt)
        if original_recorder:
            original_recorder(receipt)

    source_root = Path(__file__).resolve().parents[1]
    sources = ["learning/fast_observation.py", "learning/fast_probe.py", "learning/device_adapter.py",
               "learning/artifacts.py", "tools/probe_fast_observation.py",
               "ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/observation.py"]
    save_json("session.json", {"schemaVersion": "motionbrain.fast_probe_session.v1",
                              "policy": AnchoredRoiMeasurement.POLICY, "requestedSamples": samples,
                              "interval": interval, "hostClockId": transport.host_clock_id,
                              "opencvVersion": cv2.__version__, "numpyVersion": np.__version__,
                              "opencvThreads": cv2.getNumThreads(),
                              "sourceSha256": {p: hashlib.sha256((source_root / p).read_bytes()).hexdigest()
                                               for p in sources},
                              "trainingEligible": False, "controlEligible": False})
    store.append("started", "probe", {})
    transport.recorder = record
    state, error, before, after = "completed", None, None, None
    try:
        adapter.capability()
        before = adapter.outputs()
        if not _idle(adapter.status()):
            raise DeviceReadError("controller_not_idle")
        anchor_bytes = []

        def save_anchor(body, digest):
            anchor_bytes.append(body)
            return save_image("anchor", "source", body, digest)

        anchor = adapter.observe(save_anchor)
        save_json("anchor.json", anchor)
        tracker = AnchoredRoiMeasurement(anchor["observation"], anchor_bytes[0], clock=transport.clock)
        for index in range(samples):
            if not _idle(adapter.status()):
                raise DeviceReadError("controller_not_idle")
            receipt, body = transport.get("camera", "/capture")
            if receipt.get("status") != 200:
                raise DeviceReadError("camera_capture_failed")
            ref = save_image(index, "source", body, receipt["sha256"])
            source = {**parse_capture_headers(receipt["headers"]), "imageSha256": receipt["sha256"]}
            result = tracker.measure(body, source, request_started=receipt["requestStartedMonotonic"],
                                     received=receipt["receivedMonotonic"], host_clock_id=transport.host_clock_id)
            result.update(image=ref, source=source)
            result_ref = save_json(f"measurements/{index:04d}.json", result)
            store.append("measured", str(index), {"measurement": result_ref})
            results.append(result)
            if index + 1 < samples:
                sleep(interval)
    except KeyboardInterrupt:
        state, error = "interrupted", "KeyboardInterrupt"
    except Exception as exc:
        state, error = "failed", f"{type(exc).__name__}: {exc}"
    finally:
        try:
            adapter.capability()  # Detect controller reboot even if output log happens to match.
            after = adapter.outputs()
            if not _idle(adapter.status()):
                raise DeviceReadError("controller_not_idle_at_end")
        except Exception as exc:
            state, error = "failed", f"{type(exc).__name__}: {exc}"
        finally:
            transport.recorder = original_recorder
    unchanged = before is not None and before == after
    if not unchanged and state == "completed":
        state, error = "failed", "controller_output_log_changed"
    integrity_errors = verify_artifacts(store.root, artifacts)
    if integrity_errors:
        state, error = "failed", "artifact_integrity_error"
    summary = {"schemaVersion": "motionbrain.fast_probe.v1", "status": state, "error": error,
               "requestedSamples": samples, "completedSamples": len(results),
               "acceptedMeasurements": sum(r["accepted"] for r in results),
               "results": results, "artifacts": artifacts, "integrityErrors": integrity_errors,
               "outputLogUnchanged": unchanged, "motorCommandsSent": 0,
               "trainingEligible": False, "controlEligible": False}
    ref = store.write_json("run.json", summary)
    store.append("finished", "probe", {"report": ref, "status": state})
    return summary
