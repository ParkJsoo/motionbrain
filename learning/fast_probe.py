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
from learning.fresh_capture import validate_fresh_capture
from learning.fast_observation import AnchoredRoiMeasurement
from learning.anchor_age import causal_anchor_basis
from learning.source_age import source_age_basis, source_age_upper_ms, validate_fresh_age_mode
from learning.observation_consumer import ShadowObservationConsumer
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
                   match_workers=3, parallel_persistence=True, fresh_capture=False,
                   shadow_consumption=False, max_persistence_ms=250, fresh_age_mode="clock_margin", causal_anchor=False):
    if type(causal_anchor) is not bool or (causal_anchor and fresh_age_mode != "send_start"):
        raise ValueError("causal anchor requires send_start mode")
    validate_fresh_age_mode(fresh_age_mode)
    if fresh_age_mode == "send_start" and (fresh_capture is not True or
            getattr(transport, "connection_timing", False) is not True):
        raise ValueError("send_start requires fresh capture and connection timing")
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
    if type(fresh_capture) is not bool:
        raise ValueError("fresh_capture must be boolean")
    if type(shadow_consumption) is not bool:
        raise ValueError("shadow_consumption must be boolean")
    consumer = ShadowObservationConsumer(transport.clock, transport.host_clock_id,
                                         max_persistence_ms=max_persistence_ms, fresh_age_mode=fresh_age_mode) if shadow_consumption else None
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
               "learning/capture_timing.py", "learning/http_timing.py", "learning/source_age.py", "learning/fresh_capture.py",
               "learning/anchor_age.py", "learning/artifacts.py", "learning/observation_consumer.py", "tools/probe_fast_observation.py",
               "ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/observation.py"]
    save_json("session.json", {"schemaVersion": "motionbrain.fast_probe_session.v1",
                              "policy": AnchoredRoiMeasurement.POLICY, "requestedSamples": samples,
                              "interval": interval, "hostClockId": transport.host_clock_id,
                              "capturePersistence": "single_frame_after_measurement",
                              "parallelPersistence": parallel_persistence,
                              "freshCaptureRequested": fresh_capture,
                              "freshAgeMode": fresh_age_mode,
                              "causalAnchorEnabled": causal_anchor,
                              "anchorWaitMaxPolls": 8 if causal_anchor else None,
                              "anchorWaitIntervalSeconds": 1 if causal_anchor else None,
                              "shadowConsumptionEnabled": shadow_consumption,
                              "maxPersistenceMs": max_persistence_ms if consumer else None,
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
        if fresh_capture:
            capability = _object(transport.get("camera", "/status")[0])
            if capability.get("freshCaptureSupported") is not True:
                raise DeviceReadError("fresh_capture_unsupported")
        checkpoint_receipt, checkpoint_source = None, None
        if causal_anchor:
            checkpoint_receipt, checkpoint_body = transport.get("camera", "/capture", {"fresh": "1"})
            if checkpoint_receipt.get("status") != 200:
                raise DeviceReadError("anchor_checkpoint_capture_failed")
            checkpoint_source = {**parse_capture_headers(checkpoint_receipt["headers"]),
                                 "imageSha256": checkpoint_receipt["sha256"]}
            checkpoint_image = save_image("checkpoint", "source", checkpoint_body, checkpoint_receipt["sha256"])
            if not checkpoint_image["hashMatches"]:
                raise DeviceReadError("anchor_checkpoint_hash_mismatch")
            save_json("anchor_checkpoint.json", {"receipt": checkpoint_receipt,
                "source": checkpoint_source, "image": checkpoint_image})
            # Wait for a later detector frame, without another camera capture.
            # Transport/identity failures abort; only an older valid frame is
            # polled again, with a fixed finite budget. Never copy an old box
            # onto the checkpoint image.
            source_age_basis(checkpoint_receipt, checkpoint_source,
                request_started=checkpoint_receipt["requestStartedMonotonic"],
                received=checkpoint_receipt["receivedMonotonic"],
                host_clock_id=transport.host_clock_id, fresh_age_mode="send_start")
            for attempt in range(8):
                candidate = _object(transport.get("perception", "/api/detection")[0])
                captured = candidate.get("capturedUs")
                if (candidate.get("cameraBootId") != checkpoint_source["cameraBootId"]
                        or type(captured) is not int or not 0 < captured < 10**19
                        or candidate.get("sourceFrameId") != f"{checkpoint_source['cameraBootId']}:{captured}"):
                    raise DeviceReadError("anchor_wait_identity_mismatch")
                store.append("anchor_wait_polled", str(attempt),
                             {"sourceFrameId": candidate["sourceFrameId"],
                              "afterCheckpoint": captured > checkpoint_source["capturedUs"]})
                if captured > checkpoint_source["capturedUs"]:
                    break
                if attempt == 7:
                    raise DeviceReadError("anchor_wait_exhausted")
                sleep(1)
        anchor_bytes = []

        def save_anchor(body, digest):
            anchor_bytes.append(body)
            return save_image("anchor", "source", body, digest)

        anchor = adapter.observe(save_anchor)
        save_json("anchor.json", anchor)
        anchor_evidence = None
        if causal_anchor:
            anchor_evidence = {"hostClockId": transport.host_clock_id,
                "checkpointReceipt": checkpoint_receipt, "checkpointSource": checkpoint_source,
                "anchorObservation": anchor["observation"],
                "anchorRetrievedMonotonic": anchor["retrievedMonotonic"]}
            save_json("anchor_evidence.json", anchor_evidence)
            causal_anchor_basis(anchor_evidence, transport.host_clock_id)
            if consumer:
                consumer = ShadowObservationConsumer(transport.clock, transport.host_clock_id,
                    max_persistence_ms=max_persistence_ms, fresh_age_mode=fresh_age_mode,
                    anchor_evidence=anchor_evidence)
        tracker = AnchoredRoiMeasurement(anchor["observation"], anchor_bytes[0], clock=transport.clock,
                                         match_workers=match_workers, fresh_age_mode=fresh_age_mode, anchor_evidence=anchor_evidence)
        for index in range(samples):
            if consumer:
                consumer.check_ready()
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
                receipt, body = transport.get("camera", "/capture", {"fresh": "1"}) if fresh_capture else transport.get("camera", "/capture")
                if receipt.get("status") != 200:
                    raise DeviceReadError("camera_capture_failed")
                source = {**parse_capture_headers(receipt["headers"]), "imageSha256": receipt["sha256"]}
                if fresh_capture:
                    if receipt.get("query") != {"fresh": "1"}:
                        raise DeviceReadError("fresh_capture_query_mismatch")
                    validate_fresh_capture(receipt, source)
                result = tracker.measure(body, source, request_started=receipt["requestStartedMonotonic"],
                                         received=receipt["receivedMonotonic"], host_clock_id=transport.host_clock_id,
                                         capture_receipt=receipt)
                consumption = consumer.consume(result, source, receipt, body) if consumer else None
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
            age_at_persistence = (source_age_upper_ms(result, reported_age, persisted)
                                  if reported_age is not None and bound_start is not None
                                  and persisted >= result.get("measurementCompletedMonotonic", float("inf")) else None)
            # Acceptance refers to calculation time, never to later consumption.
            # Retain a separate conservative age after durable image/result I/O.
            persistence = {"measurement": result_ref, "persistedMonotonic": persisted,
                           "storageTiming": storage_timing,
                           "hostClockId": transport.host_clock_id,
                           "reportedAgeAtPersistenceMs": age_at_persistence,
                           "sourceAgeBoundBasis": result.get("sourceAgeBoundBasis"),
                           "freshAgeMode": fresh_age_mode,
                           "sourceAgeBoundStartedMonotonic": bound_start,
                           **{key: result[key] for key in (
                               "sourceCaptureLowerBoundMonotonic", "sourceAgeBoundReductionMs",
                               "freshCaptureDelayMs", "freshCaptureDelayLowerMs", "freshClockRateErrorPpm",
                               "freshTimestampErrorMs", "freshClockErrorCalibrated",
                               "sourceAgeClockRateAssumptionUsed", "sourceAgeTimestampMarginUsed") if key in result},
                           "fullRequestAgeAtPersistenceMs": (reported_age + (persisted-receipt["requestStartedMonotonic"])*1000
                                                             if reported_age is not None else None),
                           "withinAgePolicyAtPersistence": age_at_persistence is not None
                           and 0 <= age_at_persistence <= tracker.POLICY["maxSourceAgeMs"],
                           "controlEligible": False}
            save_json(f"persistence/{index:04d}.json", persistence)
            if consumer:
                save_json(f"consumption/{index:04d}.json", consumption)
            store.append("measured", str(index), persistence)
            results.append(result)
            if consumer:
                consumer.finish_persistence(source["sourceFrameId"], transport.host_clock_id)
            if capture_timing:
                after_camera = _object(transport.get("camera", "/status")[0])
                timing = analyze_capture_timing(receipt, before_camera, after_camera, fresh_age_mode=fresh_age_mode)
                save_json(f"timing/{index:04d}.json", timing)
                timing_samples.append(timing)
            if index + 1 < samples:
                sleep(interval)
    except KeyboardInterrupt:
        state, error = "interrupted", "KeyboardInterrupt"
        if consumer:
            consumer.stop(error)
    except Exception as exc:
        state, error = "failed", f"{type(exc).__name__}: {exc}"
        if consumer:
            consumer.stop(error)
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
    if consumer and state != "completed":
        consumer.stop(error)
    summary = {"schemaVersion": "motionbrain.fast_probe.v1", "status": state, "error": error,
               "requestedSamples": samples, "completedSamples": len(results),
               "acceptedMeasurements": sum(r["accepted"] for r in results),
               "results": results, "artifacts": artifacts, "integrityErrors": integrity_errors,
               "outputLogUnchanged": unchanged, "motorCommandsSent": 0,
               "trainingEligible": False, "controlEligible": False}
    if capture_timing:
        summary["captureTiming"] = summarize_capture_timing(timing_samples)
    if consumer:
        summary.update(consumptions=consumer.snapshot(), consumptionHalted=consumer.halted,
                       consumedObservations=sum(row["consumed"] for row in consumer.records))
    ref = store.write_json("run.json", summary)
    store.append("finished", "probe", {"report": ref, "status": state})
    return summary
