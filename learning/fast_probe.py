"""Read-only shadow measurements with original JPEGs and durable failure records."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
from pathlib import Path
import time

import cv2
import numpy as np

from learning.artifacts import verify_artifacts
from learning.capture_timing import capture_timing as analyze_capture_timing, summarize_capture_timing
from learning.device_adapter import DeviceReadAdapter, DeviceReadError, _object
from learning.device_probe import _idle
from learning.fast_observation import AnchoredRoiMeasurement
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def _persist_capture(store, index, receipt, body, pending, record, artifacts, *, clock, parallel):
    # One current JPEG only. Recorder callbacks stay on the calling thread.
    # Always join and collect the image even if receipt recording fails.
    started = clock.monotonic()
    valid_image = receipt is not None and receipt.get("status") == 200 and body is not None
    image_results = []
    def write_image():
        began = clock.monotonic()
        ref = store.save_image(str(index), "source", body, receipt["sha256"])
        saved = (ref, (clock.monotonic()-began)*1000)
        image_results.append(saved)
        return saved
    def write_receipts():
        began = clock.monotonic()
        for item in pending:
            record(item)
        return (clock.monotonic()-began)*1000
    ref, image_ms = None, None
    if parallel and valid_image:
        try:
            with ThreadPoolExecutor(max_workers=1, thread_name_prefix="capture-image") as pool:
                future = pool.submit(write_image)
                pending_error = None
                try:
                    receipt_ms = write_receipts()
                except BaseException as exc:
                    pending_error = exc
                # Defer caller interruption until the current durable write is
                # finished. Also covers a signal after a receipt write failure.
                while True:
                    try:
                        ref, image_ms = future.result()
                        break
                    except KeyboardInterrupt as exc:
                        pending_error = pending_error or exc
                        if future.done():  # worker failure or completed concurrently
                            break
                    except BaseException as exc:
                        pending_error = pending_error or exc
                        break
                if pending_error is not None:
                    raise pending_error
        finally:
            # Context exit joins the worker even when the main thread's wait
            # was interrupted. Collect from the worker-owned completion slot,
            # not from an assignment that a signal may have skipped.
            if image_results:
                artifacts.append(image_results[0][0])
    else:
        receipt_ms = write_receipts()
        if valid_image:
            ref, image_ms = write_image()
            artifacts.append(ref)
    return ref, {"receiptWriteMs": receipt_ms, "imageWriteMs": image_ms,
                 "receiptImageWallMs": (clock.monotonic()-started)*1000}


def run_fast_probe(store, transport, *, samples=8, interval=0.1, sleep=time.sleep, capture_timing=False,
                   match_workers=3, parallel_persistence=True):
    if type(samples) is not int or not 1 <= samples <= 100:
        raise ValueError("samples must be 1..100")
    if type(interval) not in (int, float) or not 0 <= interval <= 1:
        raise ValueError("interval must be 0..1 seconds")
    if type(capture_timing) is not bool:
        raise ValueError("capture_timing must be boolean")
    if type(match_workers) is not int or not 1 <= match_workers <= 3:
        raise ValueError("match_workers must be 1..3")
    if type(parallel_persistence) is not bool:
        raise ValueError("parallel_persistence must be boolean")
    artifacts, results = [], []
    timing_samples = []
    adapter = DeviceReadAdapter(transport)
    original_recorder, receipts = transport.recorder, 0
    camera_receipts = None

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
        if camera_receipts is not None:
            # Only the single synchronous camera GET is deferred. No background
            # queue, dropping, or next capture before its evidence is persisted.
            camera_receipts.append(receipt)
            return
        receipts += 1
        save_json(f"raw/{receipts:04d}.json", receipt)
        if original_recorder:
            original_recorder(receipt)

    source_root = Path(__file__).resolve().parents[1]
    sources = ["learning/fast_observation.py", "learning/fast_probe.py", "learning/device_adapter.py",
               "learning/capture_timing.py", "learning/http_timing.py", "learning/source_age.py",
               "learning/artifacts.py", "tools/probe_fast_observation.py",
               "ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/observation.py"]
    save_json("session.json", {"schemaVersion": "motionbrain.fast_probe_session.v1",
                              "policy": AnchoredRoiMeasurement.POLICY, "requestedSamples": samples,
                              "interval": interval, "hostClockId": transport.host_clock_id,
                              "capturePersistence": "single_frame_after_measurement",
                              "parallelPersistence": parallel_persistence,
                              "captureTimingEnabled": capture_timing,
                              "connectionTimingEnabled": getattr(transport, "connection_timing", False),
                              "opencvVersion": cv2.__version__, "numpyVersion": np.__version__,
                              "opencvThreads": cv2.getNumThreads(),
                              "matchWorkers": match_workers,
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
        tracker = AnchoredRoiMeasurement(anchor["observation"], anchor_bytes[0], clock=transport.clock,
                                         match_workers=match_workers)
        for index in range(samples):
            if not _idle(adapter.status()):
                raise DeviceReadError("controller_not_idle")
            before_camera = _object(transport.get("camera", "/status")[0]) if capture_timing else None
            prepare_started = transport.clock.monotonic()
            if parallel_persistence:
                store.prepare_parent(f"images/{index}/source.jpg")
                store.prepare_parent(f"measurements/{index:04d}.json")
            prepare_ms = (transport.clock.monotonic()-prepare_started)*1000
            store.append("capture_requested", str(index), {})
            receipt, body = None, None
            camera_receipts = []
            try:
                receipt, body = transport.get("camera", "/capture")
                if receipt.get("status") != 200:
                    raise DeviceReadError("camera_capture_failed")
                source = {**parse_capture_headers(receipt["headers"]), "imageSha256": receipt["sha256"]}
                result = tracker.measure(body, source, request_started=receipt["requestStartedMonotonic"],
                                         received=receipt["receivedMonotonic"], host_clock_id=transport.host_clock_id,
                                         capture_receipt=receipt)
            finally:
                pending, camera_receipts = camera_receipts, None
                ref, storage_timing = _persist_capture(store, index, receipt, body, pending, record, artifacts,
                                                     clock=transport.clock, parallel=parallel_persistence)
            result.update(image=ref, source=source)
            measurement_write_started = transport.clock.monotonic()
            result_ref = save_json(f"measurements/{index:04d}.json", result)
            persisted = transport.clock.monotonic()
            storage_timing.update(prepareBeforeCaptureMs=prepare_ms,
                                 measurementWriteMs=(persisted-measurement_write_started)*1000,
                                 measurementToPersistenceMs=(persisted-result["measurementCompletedMonotonic"])*1000
                                 if "measurementCompletedMonotonic" in result else None)
            reported_age = source.get("reportedSourceAgeMs")
            bound_start = result.get("sourceAgeBoundStartedMonotonic")
            age_at_persistence = (reported_age + (persisted-bound_start)*1000
                                  if reported_age is not None and bound_start is not None
                                  and persisted >= result.get("measurementCompletedMonotonic", float("inf")) else None)
            # Acceptance refers to calculation time, never to later consumption.
            # Retain a separate conservative age after durable image/result I/O.
            persistence = {"measurement": result_ref, "persistedMonotonic": persisted,
                           "storageTiming": storage_timing,
                           "hostClockId": transport.host_clock_id,
                           "reportedAgeAtPersistenceMs": age_at_persistence,
                           "sourceAgeBoundBasis": result.get("sourceAgeBoundBasis"),
                           "sourceAgeBoundStartedMonotonic": bound_start,
                           "fullRequestAgeAtPersistenceMs": (reported_age + (persisted-receipt["requestStartedMonotonic"])*1000
                                                             if reported_age is not None else None),
                           "withinAgePolicyAtPersistence": age_at_persistence is not None
                           and 0 <= age_at_persistence <= tracker.POLICY["maxSourceAgeMs"],
                           "controlEligible": False}
            save_json(f"persistence/{index:04d}.json", persistence)
            store.append("measured", str(index), persistence)
            results.append(result)
            if capture_timing:
                after_camera = _object(transport.get("camera", "/status")[0])
                timing = analyze_capture_timing(receipt, before_camera, after_camera)
                save_json(f"timing/{index:04d}.json", timing)
                timing_samples.append(timing)
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
    if capture_timing:
        summary["captureTiming"] = summarize_capture_timing(timing_samples)
    ref = store.write_json("run.json", summary)
    store.append("finished", "probe", {"report": ref, "status": state})
    return summary
