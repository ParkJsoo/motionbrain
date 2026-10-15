"""Single confirmed commissioning capture; raw device evidence stays ineligible.

Camera/perception and independent M4 samples retain their original clock domains.
M4 stability plus software output-off is not proof of whole-arm physical settling.
There is no CLI, retry, resume, calibration override or training-schema conversion.
"""
from __future__ import annotations

import copy
import hashlib
import threading

from learning.action_execution import canonical_request, finite
from learning.artifacts import StorageError, verify_artifacts, read_json_artifact, read_events, describe_artifact
from learning.device_adapter import DeviceReadAdapter, _object
from learning.device_probe import observation_blockers

POLICY = dict(maxSourceAgeMs=100, maxM4AgeMs=20, maxAnchorAgeMs=15000,
              maxPersistenceMs=250, minimumStationaryMs=50,
              maxStationarySamples=6, stationaryBudgetMs=500, sampleIntervalMs=25)
SCHEMA = "motionbrain.device_capture.v1"


class CaptureRejected(RuntimeError):
    pass


def zero_outputs(status):
    motors = status.get("motors", {})
    return (status.get("state") in ("IDLE", "ARMED") and isinstance(motors, dict)
            and set(motors) == {f"M{i}" for i in range(1, 6)}
            and all(isinstance(m, dict) and type(m.get("speed")) is int
                    and m["speed"] == 0 and m.get("enabled") is False for m in motors.values()))


class DeviceCaptureSession:
    """Own a new artifact directory and one executor/transport for one attempt.

    The supplied executor remains responsible for exact confirmation, durable IDs,
    default-disabled writes, and unknown-outcome latching. A tolerance must come
    from the caller's commissioning profile; it is not a physical safety default.
    """
    def __init__(self, store, executor, *, m4_tolerance_degrees):
        if not finite(m4_tolerance_degrees) or not 0 <= m4_tolerance_degrees <= 1:
            raise ValueError("explicit_m4_tolerance_in_0_to_1_degree_required")
        self.store, self.executor = store, executor
        self.reader = DeviceReadAdapter(executor.port.transport)
        self.transport = self.reader.transport
        if self.transport.clock is not executor.clock or self.transport.host_clock_id != executor.port.host_clock_id:
            raise ValueError("shared_local_transport_clock_required")
        self.clock = executor.clock
        self.tolerance = m4_tolerance_degrees
        self.used = False
        self.storage_failed = False
        self.lock = threading.Lock()
        self.pending, self.refs, self.receipt_index = [], [], 0
        self.last = self.clock.monotonic()
        self.before = None

    def _now(self):
        now = self.clock.monotonic()
        if not finite(now) or not finite(self.last) or now < self.last or now < 0:
            raise CaptureRejected("local_clock_invalid_or_regressed")
        self.last = now
        return now

    def _write(self, path, value):
        if self.storage_failed:
            raise StorageError("capture_storage_already_failed")
        try:
            ref = self.store.write_json(path, value)
        except StorageError:
            self.storage_failed = True
            raise
        self.refs.append(ref)
        return ref

    def _flush(self):
        # All raw receipts are saved, including rejected HTTP responses. No
        # network callback performs persistence after the final dispatch gate.
        if self.storage_failed:
            raise StorageError("capture_storage_already_failed")
        while self.pending:
            receipt = self.pending[0]
            next_index = self.receipt_index + 1
            self._write(f"raw/http-{next_index:04d}.json", receipt)
            self.receipt_index = next_index
            self.pending.pop(0)

    def _barrier(self, started):
        ended = self._now()
        if (ended-started)*1000 > POLICY["maxPersistenceMs"] + 1e-6:
            raise CaptureRejected("persistence_exceeds_250ms")
        return ended

    def _sample(self, boot):
        cap = self.reader.capability()
        if cap["bootId"] != boot or cap["active"] is not False:
            raise CaptureRejected("controller_boot_or_active_changed")
        receipt, _ = self.transport.get("controller", "/status")
        status = _object(receipt)
        m4, sensor = status.get("shoulderAngle", {}), status.get("sensor", {})
        start, end = receipt["requestStartedMonotonic"], receipt["receivedMonotonic"]
        now = self._now()
        if (receipt.get("hostClockId") != self.transport.host_clock_id
                or not all(finite(v) for v in (start, end)) or not 0 <= start <= end <= now
                or not zero_outputs(status)
                or any(sensor.get(k) is not True for k in ("connected", "imuOk"))
                or any(sensor.get(k) is not False for k in ("simulated", "faultLatched", "blocked"))
                or not isinstance(m4, dict)
                or any(m4.get(k) is not True for k in ("available", "sensorConnected", "sensorFresh", "sensorReady"))
                or any(m4.get(k) is not False for k in ("active", "correctionActive"))
                or not finite(m4.get("ageMs")) or not 0 <= m4["ageMs"] <= POLICY["maxM4AgeMs"]
                or not finite(m4.get("angleDeg")) or not 0 <= m4["angleDeg"] < 360):
            raise CaptureRejected("m4_or_zero_output_unverified")
        lower = start - m4["ageMs"]/1000
        if lower < 0 or (now-lower)*1000 > POLICY["maxM4AgeMs"] + 1e-6:
            raise CaptureRejected("m4_age_including_transport_exceeded")
        return dict(status=status, receipt=receipt, controllerBootId=boot,
                    angleDegrees=m4["angleDeg"], sampleLowerMonotonic=lower,
                    sampleUpperMonotonic=end, hostClockId=self.transport.host_clock_id,
                    basis="status_age_plus_local_round_trip", clockRateCalibrated=False)

    def _observe(self, phase, boot):
        image_bytes = []
        def buffer(body, digest):
            image_bytes.append((body, digest))
            actual = hashlib.sha256(body).hexdigest()
            return dict(sha256=actual, hashMatches=actual == digest, bytes=len(body))
        try:
            observed = self.reader.observe(buffer)
            sample = self._sample(boot)
        except StorageError:
            raise
        except Exception:
            # Preserve even rejected bytes (bad hash/JPEG or a later M4 fault).
            self._flush()
            for index, (body, digest) in enumerate(image_bytes):
                ref = self.store.save_image("capture", f"{phase}-rejected-{index}", body, digest)
                self.refs.append(ref)
            raise
        snapshot = dict(observed, m4Sample=sample)
        started = self._now()
        self._flush()
        image = self.store.save_image("capture", phase, image_bytes[0][0], observed["observation"]["imageSha256"])
        self.refs.append(image)
        snapshot["image"] = image
        self._write(f"observations/{phase}.json", snapshot)
        self.store.append("observation_persisted", "capture", {"phase": phase, "image": image})
        snapshot["persistenceCompletedMonotonic"] = self._barrier(started)
        return snapshot

    def _check(self, snapshot):
        now = self._now()
        o = snapshot["observation"]
        stamp = snapshot["retrievedMonotonic"]
        age = snapshot["sourceAgeUpperBoundAtRetrievalMs"]
        if (snapshot["retrievalHostClockId"] != self.transport.host_clock_id
                or not finite(age) or not finite(stamp) or not 0 <= stamp <= now):
            raise CaptureRejected("retrieval_age_or_clock_unverified")
        age += (now-stamp)*1000
        reasons = observation_blockers(o, age)
        bounds = o.get("captureHostBounds", {})
        times = [bounds.get("earliest"), bounds.get("latest"),
                 o.get("receivedMonotonic"), o.get("inferenceCompletedMonotonic")]
        if (not isinstance(o.get("captureHostBoundsEvidenceRef"), str) or not o["captureHostBoundsEvidenceRef"].strip()
                or bounds.get("clockId") != o.get("hostClockId")
                or not all(finite(t) and t >= 0 for t in times) or times != sorted(times)
                or not finite(o.get("sourceAgeMs"))
                or (all(finite(t) for t in times) and o["sourceAgeMs"] + 1e-6 < (times[-1]-times[0])*1000)):
            reasons.append("producer_clock_evidence_invalid")
        # Shadow ROI inheritance is never promoted into an exact live detection.
        if o.get("detected") is not True or o.get("liveDetected") is not True or o.get("synthetic") is True:
            reasons.append("exact_live_device_detection_required")
        if o.get("sourceClockDomain") != "camera_boot_us" or o.get("sourceTimestampSemantics") != "first_dma_buffer":
            reasons.append("source_timestamp_semantics_unverified")
        if (now-snapshot["m4Sample"]["sampleLowerMonotonic"])*1000 > POLICY["maxM4AgeMs"] + 1e-6:
            reasons.append("m4_expired")
        # Exact-frame detections need no semantic anchor. Anchored/ROI evidence
        # belongs to the existing shadow path, with its unchanged 15 s limit.
        if any(k in o for k in ("anchorSourceFrameId", "anchorAgeMs", "anchorObservation")):
            reasons.append("anchored_shadow_observation_not_supported")
        if reasons:
            raise CaptureRejected(";".join(reasons))
        return now - (age + o["captureHostBoundsUncertaintySeconds"]*1000)/1000

    def _guard(self, request):
        started = self._now()
        self._flush()  # executor preflight receipts, before final freshness check
        self._barrier(started)
        self._check(self.before)

    def _quiet_outputs(self, baseline):
        outputs = self.reader.outputs()
        if outputs["dropped"] != 0:
            raise CaptureRejected("settling_output_history_dropped")
        previous, previous_new = 0, baseline
        for entry in outputs["entries"]:
            sequence = entry["sequence"]
            if type(sequence) is not int or sequence <= previous:
                raise CaptureRejected("settling_output_sequence_invalid")
            previous = sequence
            if sequence > baseline and (sequence != previous_new + 1 or entry["pwm"] != 0):
                raise CaptureRejected("output_reactivated_or_trace_gap_during_settling")
            if sequence > baseline:
                previous_new = sequence
        if previous < baseline:
            raise CaptureRejected("settling_output_history_missing")
        return outputs

    def _stationary(self, action):
        boot = action["request"]["bootId"]
        off = action["outputOffHostBounds"]
        if off["clockId"] != self.transport.host_clock_id or not finite(off["latest"]) or off["latest"] > self._now():
            raise CaptureRejected("output_off_clock_unverified")
        started, samples = self._now(), []
        baseline = action["outputSequenceBounds"]["last"]
        for index in range(POLICY["maxStationarySamples"]):
            self.clock.sleep(POLICY["sampleIntervalMs"]/1000)
            if (self._now()-started)*1000 > POLICY["stationaryBudgetMs"]:
                raise CaptureRejected("stationary_budget_exceeded")
            sample = self._sample(boot)
            outputs = self._quiet_outputs(baseline)
            if sample["sampleLowerMonotonic"] <= off["latest"]:
                raise CaptureRejected("sample_not_proven_after_output_off")
            if samples and sample["sampleLowerMonotonic"] <= samples[-1]["sampleUpperMonotonic"]:
                raise CaptureRejected("m4_sample_intervals_overlap")
            samples.append(sample)
            angles = [s["angleDegrees"] for s in samples]
            # Pairwise circular spread handles the AS5600 wrap at 360 degrees.
            if any(abs((a-b+180) % 360-180) > self.tolerance for a in angles for b in angles):
                raise CaptureRejected("m4_not_stationary")
            persisted = self._now()
            self._flush()
            self._write(f"stationary/{index:02d}.json", dict(sample=sample, outputLog=outputs))
            self._barrier(persisted)
            if (self._now()-started)*1000 > POLICY["stationaryBudgetMs"]:
                raise CaptureRejected("stationary_budget_exceeded")
            span = (sample["sampleLowerMonotonic"]-samples[0]["sampleUpperMonotonic"])*1000
            if len(samples) >= 3 and span >= POLICY["minimumStationaryMs"]:
                return dict(m4Stationary=True, softwareOutputsOff=True, physicalSettledVerified=False,
                            toleranceDegrees=self.tolerance, anglesDegrees=angles, spanLowerMs=span, samples=len(samples),
                            completedMonotonic=self._now(), hostClockId=self.transport.host_clock_id)
        raise CaptureRejected("stationary_sample_budget_exhausted")

    def capture(self, request, *, confirmed_request):
        with self.lock, self.executor.lock:
            if self.used:
                raise RuntimeError("single_attempt_session_no_retry_or_resume")
            self.used = True
            r = canonical_request(request)
            result = dict(schemaVersion=SCHEMA, mode="device_commissioning", request=r,
                          status="rejected", executionState="not_sent", trainingEligible=False,
                          automaticRetry=False, physicalSettledVerified=False)
            previous_recorder = self.transport.recorder
            def record(receipt):
                self.pending.append(copy.deepcopy(receipt))
                if previous_recorder:
                    previous_recorder(receipt)
            self.transport.recorder = record
            try:
                self._write("session.json", dict(schemaVersion=SCHEMA, request=r, policy=POLICY,
                    toleranceDegrees=self.tolerance, trainingEligible=False, mode="device_commissioning"))
                if canonical_request(confirmed_request) != r:
                    raise CaptureRejected("exact_confirmation_required")
                if self.executor.journal.get(r["commandId"]) is not None:
                    raise CaptureRejected("command_id_already_reserved_no_new_capture")
                self.before = self._observe("before", r["bootId"])
                result["before"] = self.before
                self._check(self.before)
                self.store.append("action_intent", "capture", {"request": r, "automaticRetry": False})
                result["executionState"] = "outcome_unknown"
                self.executor.propose(r)
                executed = self.executor.confirm(r["commandId"], confirmed_request=confirmed_request, dispatch_guard=self._guard)
                result["executorResult"] = executed
                result["executionState"] = executed["executionState"]
                started = self._now()
                self._flush()
                self._write("action.json", executed)
                self._barrier(started)
                if executed["executionState"] != "known_terminal":
                    raise CaptureRejected("action_not_known_terminal")
                action = executed["action"]
                result["stationary"] = self._stationary(action)
                after = self._observe("after", r["bootId"])
                result["after"] = after
                final_outputs = self._quiet_outputs(action["outputSequenceBounds"]["last"])
                persisted = self._now()
                self._flush()
                self._write("after-output-log.json", final_outputs)
                self._barrier(persisted)
                lower = self._check(after)
                angle = after["m4Sample"]["angleDegrees"]
                if any(abs((angle-a+180) % 360-180) > self.tolerance for a in result["stationary"]["anglesDegrees"]):
                    raise CaptureRejected("m4_moved_after_stationary_window")
                before_o, after_o = self.before["observation"], after["observation"]
                if (after_o["cameraBootId"] != before_o["cameraBootId"]
                        or after_o["capturedUs"] <= before_o["capturedUs"]
                        or lower <= result["stationary"]["completedMonotonic"]):
                    raise CaptureRejected("after_not_proven_after_stationary_window")
                result["status"] = "recorded"
            except StorageError:
                # No success terminal or storage retry after a failed barrier.
                raise
            except Exception as exc:
                result["reason"] = str(exc) if isinstance(exc, CaptureRejected) else type(exc).__name__
            except BaseException:
                # Missing terminal remains unknown on audit; never resume.
                raise
            finally:
                self.transport.recorder = previous_recorder
            try:
                started = self._now()
                self._flush()
                issues = verify_artifacts(self.store.root, self.refs)
                if issues:
                    raise StorageError("capture_artifact_integrity_failed")
                self._barrier(started)
                result["artifacts"] = copy.deepcopy(self.refs)
                summary = self._write("capture.json", result)
                # A missing final marker cannot prove an intact/complete capture.
                self.store.append("capture_finished", "capture", {"summary": summary})
                self.store.write_json("run.json", dict(schemaVersion=SCHEMA, summary=summary, trainingEligible=False))
                self._barrier(started)
                # This terminal seal is written only after the full barrier
                # passed. Its absence is unconfirmed, never a successful run.
                self.store.append("capture_sealed", "capture", {"summary": summary})
                return result
            except (StorageError, OSError, CaptureRejected) as exc:
                raise StorageError("capture_recording_incomplete; retain directory and inspect journal") from exc


def audit_device_capture(root):
    """Read-only integrity report; never clears executor unknown or promotes data."""
    try:
        terminal = read_json_artifact(root, describe_artifact(root, "run.json"))
        if not isinstance(terminal, dict) or terminal.get("schemaVersion") != SCHEMA or terminal.get("trainingEligible") is not False:
            raise ValueError("terminal_contract_invalid")
        ref = terminal["summary"]
        issues = verify_artifacts(root, [ref])
        summary = read_json_artifact(root, ref)
        if not isinstance(summary, dict):
            raise ValueError("invalid_summary")
        issues += verify_artifacts(root, summary["artifacts"])
        events, event_issues = read_events(root)
        issues += event_issues
        finished = [e for e in events if e.get("kind") == "capture_finished"]
        if (len(finished) != 1 or finished[0].get("payload", {}).get("summary") != ref
                or not events or events[-1].get("kind") != "capture_sealed"
                or events[-1].get("payload", {}).get("summary") != ref):
            issues.append("terminal_event_missing_or_mismatched")
        if summary.get("schemaVersion") != SCHEMA or summary.get("trainingEligible") is not False:
            issues.append("summary_contract_invalid")
        return dict(ok=not issues, issues=issues, trainingEligible=False, automaticRetry=False,
                    executionState=summary.get("executionState") if not issues else "outcome_unknown")
    except (StorageError, ValueError, KeyError, TypeError, AttributeError):
        return dict(ok=False, issues=["capture_incomplete_or_invalid"], executionState="outcome_unknown",
                    trainingEligible=False, automaticRetry=False)
