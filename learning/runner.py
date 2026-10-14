"""One-step, device-free collection with durable intent and partial evidence.

Only synthetic adapters are supported. There is deliberately no HTTP/serial
transport, retry, resume, or physical execution switch in this module.
"""
from __future__ import annotations

import copy
import math
import uuid
from pathlib import Path

from learning.artifacts import ArtifactStore, StorageError, read_events, read_json_artifact, verify_artifacts
from learning.schema import SCHEMA_VERSION, validate_transition

DEFAULT_POLICY = {"maxSourceAgeMs": 100, "maxM4AgeMs": 20, "minimumSettlingMs": 50}
KNOWN_TERMINAL = {"completed", "timed_out", "cancelled", "failed"}


class CaptureFault(Exception):
    pass


class CapturePersistenceError(RuntimeError):
    """Recording failed; persisted intent must be audited, never replayed."""


def _action_matches(action, expected):
    if not isinstance(action, dict) or not isinstance(expected, dict) or action.get("request") != expected:
        return False
    record = action.get("record")
    return (isinstance(record, dict)
            and all(key in expected and record.get(key) == expected[key] for key in
                    ("bootId", "commandId", "kind", "motor", "durationMs", "targetDegrees", "percent"))
            and "pwm" in expected and record.get("requestedPwm") == expected["pwm"])


def _known_terminal(action, expected):
    record = action.get("record", {}) if isinstance(action, dict) else {}
    return (_action_matches(action, expected) and isinstance(record, dict) and record.get("state") in KNOWN_TERMINAL
            and record.get("outputOff") is True)


class CaptureRunner:
    def __init__(self, store: ArtifactStore, adapter, *, config=None, calibration=None,
                 quality_policy=None, checkpoint=None):
        if getattr(adapter, "mode", None) != "dry_run":
            raise ValueError("only dry_run adapters are supported")
        self.store, self.adapter = store, adapter
        self.checkpoint = checkpoint or (lambda stage: None)
        self.policy = copy.deepcopy(quality_policy if quality_policy is not None else DEFAULT_POLICY)
        for key in DEFAULT_POLICY:
            value = self.policy.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f"invalid quality policy: {key}")
        self.session_id = uuid.uuid4().hex
        self.halted = False
        self.closed = False
        self.attempt_ids = set()
        self.finished_attempts = []
        config_ref = store.write_json("snapshots/config.json", config if config is not None else {
            "mode": "dry_run", "adapter": type(adapter).__name__, "qualityPolicy": self.policy})
        calibration_ref = store.write_json("snapshots/calibration.json", calibration if calibration is not None else {
            "evidence": "synthetic", "physicalCalibration": False})
        self.metadata = {
            "robotId": "synthetic-motionbrain", "firmwareVersion": "synthetic-bounded-v1",
            "hostVersion": "dry-run-collector-v1", "detectorVersion": "synthetic-v1",
            "configHash": config_ref["sha256"], "calibrationHash": calibration_ref["sha256"],
        }
        session = {"schemaVersion": "motionbrain.capture_session.v1", "sessionId": self.session_id,
                   "mode": "dry_run", "evidence": "synthetic", "qualityPolicy": self.policy,
                   "metadata": self.metadata, "snapshots": [config_ref, calibration_ref],
                   "resumeSupported": False}
        session_ref = store.write_json("session.json", session)
        self.session_artifacts = [config_ref, calibration_ref, session_ref]
        store.append("session_started", self.session_id, {"session": session_ref})

    def _capture_observation(self, phase, tid, candidate, artifacts):
        observation = copy.deepcopy(self.adapter.observe(phase))
        # Persist exactly what the producer supplied, before attaching local paths.
        raw = self.store.write_json(f"raw/{tid}/{phase}.json", observation)
        artifacts.append(raw)
        candidate[phase] = copy.deepcopy(observation)
        self.store.append("observation_received", tid, {"phase": phase, "raw": raw})
        self.checkpoint(f"{phase}_observation")
        if not isinstance(observation, dict) or not isinstance(observation.get("sourceFrameId"), str):
            raise CaptureFault(f"{phase}:source_identity_missing")
        # Never substitute a latest-image request when the exact ID is missing.
        frame = self.adapter.fetch_frame(observation["sourceFrameId"])
        image = self.store.save_image(tid, phase, frame, observation.get("imageSha256"))
        artifacts.append(image)
        candidate[phase]["image"] = {"artifactRef": image["artifactRef"], "sha256": image["sha256"]}
        self.store.append("image_saved", tid, {"phase": phase, "image": image})
        self.checkpoint(f"{phase}_image")
        if not image["hashMatches"]:
            raise CaptureFault(f"{phase}:image_hash_mismatch")

    def _check_before(self, observation):
        """Dispatch gate; full transition validation remains the schema's job."""
        for key in ("sourceTimeKnown", "sourceMetadataValid", "sourceFresh", "controlEligible"):
            if observation.get(key) is not True:
                raise CaptureFault(f"before:{key}_required")
        if observation.get("held") is not False:
            raise CaptureFault("before:held")
        boot, captured = observation.get("cameraBootId"), observation.get("capturedUs")
        if (not isinstance(boot, str) or len(boot) != 32 or any(c not in "0123456789abcdef" for c in boot)
                or type(captured) is not int or captured <= 0
                or observation.get("sourceFrameId") != f"{boot}:{captured}"
                or observation.get("detectionSourceFrameId") != observation.get("sourceFrameId")):
            raise CaptureFault("before:source_identity_mismatch")
        if observation.get("hostClockId") != self.adapter.host_clock_id:
            raise CaptureFault("before:host_clock_mismatch")
        now = self.adapter.clock.monotonic()
        for age, stamp, limit, name in (
            (observation.get("sourceAgeMs"), observation.get("inferenceCompletedMonotonic"),
             self.policy["maxSourceAgeMs"], "source"),
            (observation.get("m4", {}).get("sampleAgeMs"), observation.get("receivedMonotonic"),
             self.policy["maxM4AgeMs"], "m4"),
        ):
            if (type(age) not in (int, float) or type(stamp) not in (int, float)
                    or not all(math.isfinite(v) for v in (age, stamp, now))
                    or age < 0 or now < stamp or age + (now - stamp) * 1000 > limit + 1e-6):
                raise CaptureFault(f"before:{name}_expired")
        m4 = observation.get("m4", {})
        if m4.get("measured") is not True or m4.get("motor") != 4 or m4.get("sensor") != "AS5600":
            raise CaptureFault("before:m4_measurement_missing")

    def capture(self, *, transition_id=None, episode_id="episode-1"):
        if self.halted or self.closed:
            raise RuntimeError("collector halted; inspect evidence before starting another run")
        tid = transition_id or uuid.uuid4().hex
        if not isinstance(tid, str) or not tid or len(tid) > 64 or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in tid):
            raise ValueError("unsafe transition ID")
        if tid in self.attempt_ids:
            raise ValueError("transition ID already reserved; no replay")
        self.attempt_ids.add(tid)
        request = copy.deepcopy(self.adapter.request(f"{self.session_id}:{tid}"))
        candidate = {"schemaVersion": SCHEMA_VERSION, "sessionId": self.session_id,
                     "episodeId": episode_id, "transitionId": tid, "metadata": self.metadata,
                     "qualityPolicy": self.policy, "before": None, "after": None,
                     "action": {"request": request}, "outcome": {"taskSuccess": None},
                     "collection": {"mode": "dry_run", "evidence": "synthetic"}}
        artifacts = copy.deepcopy(self.session_artifacts)
        errors, status, execution, stage = [], "completed", "not_sent", "attempt_started"
        try:
            self.store.append("attempt_started", tid, {"request": request, "episodeId": episode_id,
                                                      "mode": "dry_run", "evidence": "synthetic"})
        except StorageError as exc:
            self.halted = True
            raise CapturePersistenceError("could not reserve attempt; action was not called") from exc
        try:
            self.checkpoint("attempt_started")
            stage = "before"
            self._capture_observation("before", tid, candidate, artifacts)
            self.checkpoint("before_action")
            self._check_before(candidate["before"])
            issues = verify_artifacts(self.store.root, artifacts)
            if issues:
                raise CaptureFault("pre_action_artifact_verification:" + ";".join(issues))
            stage = "action_intent"
            self.store.append("action_intent", tid, {"request": request, "automaticRetry": False})
            # From this durable boundary onward, absence of a result proves
            # neither execution nor non-execution. No retry or resume exists.
            execution = "outcome_unknown"
            self.checkpoint("action_intent")
            self._check_before(candidate["before"])
            stage = "action"
            action = copy.deepcopy(self.adapter.execute(request))
            candidate["action"] = action
            raw = self.store.write_json(f"raw/{tid}/action.json", action)
            artifacts.append(raw)
            self.store.append("action_received", tid, {"raw": raw})
            if not _known_terminal(action, request):
                raise CaptureFault("action_outcome_unknown_or_output_off_unconfirmed")
            execution = "known_terminal"
            self.checkpoint("action_received")
            stage = "settling"
            self.adapter.settle(self.policy["minimumSettlingMs"])
            self.checkpoint("settled")
            stage = "after"
            self._capture_observation("after", tid, candidate, artifacts)
        except KeyboardInterrupt as exc:
            status = "interrupted"
            errors.append({"stage": stage, "type": "KeyboardInterrupt", "message": str(exc) or "operator_interrupt"})
        except Exception as exc:
            status = "aborted"
            errors.append({"stage": stage, "type": type(exc).__name__, "message": str(exc)})
        except BaseException:
            self.halted = True
            raise
        if status != "completed":
            self.halted = True
        try:
            verification = verify_artifacts(self.store.root, artifacts)
            if verification:
                errors.extend({"stage": "verification", "type": "artifact_integrity", "message": issue} for issue in verification)
                status, self.halted = "aborted", True
            missing = []
            for phase in ("before", "after"):
                value = candidate.get(phase)
                if not isinstance(value, dict):
                    missing.append(f"{phase}.observation")
                if not isinstance(value, dict) or "image" not in value:
                    missing.append(f"{phase}.image")
            for key in ("record", "outputLog", "startHostBounds", "outputOffHostBounds", "outputSequenceBounds"):
                if not isinstance(candidate.get("action"), dict) or key not in candidate["action"]:
                    missing.append(f"action.{key}")
            validation = validate_transition(candidate)
            if status == "completed" and (not validation["valid"] or
                    any(reason != "synthetic_dry_run" for reason in validation["exclusionReasons"])):
                status, self.halted = "excluded", True
            summary = {"schemaVersion": "motionbrain.capture_attempt.v1", "mode": "dry_run",
                       "evidence": "synthetic", "transitionId": tid, "status": status,
                       "executionState": execution, "automaticRetry": False,
                       "transition": candidate, "validation": validation, "trainingEligible": False,
                       "artifacts": artifacts, "missing": missing, "errors": errors}
            result = self.store.write_json(f"attempts/{tid}.json", summary)
            self.store.append("attempt_finished", tid, {"summary": result, "status": status})
            self.finished_attempts.append(result)
            return summary
        except (StorageError, OSError) as exc:
            self.halted = True
            raise CapturePersistenceError(f"recording incomplete for {tid}; audit {self.store.root}; do not retry") from exc

    def close(self):
        """Durable session terminal marker, including intentionally stopped runs."""
        if self.closed:
            raise RuntimeError("session already closed; no rewrite")
        self.closed = True
        try:
            terminal = self.store.write_json("run.json", {
                "sessionId": self.session_id, "mode": "dry_run", "evidence": "synthetic",
                "status": "stopped" if self.halted else "completed",
                "attemptIds": sorted(self.attempt_ids), "attempts": self.finished_attempts,
                "trainingEligible": False, "automaticRetry": False})
            self.store.append("session_finished", self.session_id, {"summary": terminal})
        except StorageError as exc:
            self.halted = True
            raise CapturePersistenceError(f"session close incomplete; audit {self.store.root}") from exc


def audit_run(run_dir):
    """Read-only recovery report. Never constructs an adapter or replays intent."""
    root = Path(run_dir)
    events, issues = read_events(root)
    journal_corrupt = bool(issues)
    if not events or events[0].get("kind") != "session_started":
        issues.append("session_event_missing")
    attempts = {}
    terminal = None
    for event in events:
        if event.get("kind") in ("session_started", "session_finished"):
            payload = event.get("payload")
            reference = payload.get("session" if event["kind"] == "session_started" else "summary") if isinstance(payload, dict) else None
            try:
                record = read_json_artifact(root, reference)
                if not isinstance(record, dict):
                    raise StorageError("session record must be an object")
                if record.get("mode") != "dry_run" or record.get("sessionId") != event.get("transitionId"):
                    raise StorageError("session identity or mode mismatch")
                if event["kind"] == "session_started":
                    issues.extend(verify_artifacts(root, record.get("snapshots")))
                else:
                    terminal = record
                    issues.extend(verify_artifacts(root, record.get("attempts")))
            except StorageError as exc:
                issues.append(str(exc))
            continue
        tid = event.get("transitionId")
        if not tid:
            continue
        attempts.setdefault(tid, []).append(event)
    if terminal is None or events[-1].get("kind") != "session_finished":
        issues.append("session_terminal_missing")
    elif terminal.get("attemptIds") != sorted(attempts):
        issues.append("session_attempt_index_mismatch")
    reports = []
    for tid, journal in attempts.items():
        kinds = [event.get("kind") for event in journal]
        faults = []
        if kinds[0] != "attempt_started" or kinds.count("attempt_started") != 1:
            faults.append("attempt_start_missing_or_duplicate")
        execution = "outcome_unknown"
        summary = None
        intents = [event.get("payload") for event in journal if event.get("kind") == "action_intent"]
        expected = intents[0].get("request") if len(intents) == 1 and isinstance(intents[0], dict) else None
        started_payload = journal[0].get("payload")
        if expected is not None and (not isinstance(started_payload, dict) or expected != started_payload.get("request")):
            faults.append("intent_request_mismatch")
            expected = None
        for event in journal:
            payload = event.get("payload", {})
            if not isinstance(payload, dict):
                faults.append("event_payload_not_object")
                continue
            if event.get("kind") == "image_saved":
                faults.extend(verify_artifacts(root, [payload.get("image")]))
                continue
            reference = payload.get("summary") if event.get("kind") == "attempt_finished" else payload.get("raw")
            if reference is None:
                continue
            failed = verify_artifacts(root, [reference])
            faults.extend(failed)
            if failed:
                continue
            try:
                record = read_json_artifact(root, reference)
                if not isinstance(record, dict):
                    raise ValueError("record must be an object")
                if event.get("kind") == "action_received":
                    if not _action_matches(record, expected):
                        faults.append("action_identity_mismatch_or_intent_missing")
                    elif _known_terminal(record, expected):
                        execution = "known_terminal"
                if event.get("kind") == "attempt_finished":
                    summary = record
                    faults.extend(verify_artifacts(root, record.get("artifacts", [])))
            except (StorageError, OSError, ValueError, TypeError, AttributeError) as exc:
                faults.append(f"record_unreadable:{type(exc).__name__}")
        if summary is None:
            faults.append("attempt_incomplete")
        else:
            # Do not trust a cached eligibility decision, including after tamper.
            validation = validate_transition(summary.get("transition"))
            if summary.get("transitionId") != tid or summary.get("mode") != "dry_run":
                faults.append("summary_identity_or_mode_mismatch")
            if validation["trainingEligible"]:
                faults.append("synthetic_provenance_missing")
            if not faults and not issues and "action_intent" not in kinds and summary.get("executionState") == "not_sent":
                execution = "not_sent"
        if issues or faults:
            # Missing/corrupt journal entries may hide dispatch or result records.
            # Keep proven terminal results only when their evidence is intact.
            if execution != "known_terminal" or journal_corrupt:
                execution = "outcome_unknown"
        reports.append({"transitionId": tid, "status": summary.get("status") if summary else "incomplete",
                        "executionState": execution, "trainingEligible": False, "automaticRetry": False,
                        "issues": faults, "missing": summary.get("missing", []) if summary else ["attempt.summary"],
                        "errors": summary.get("errors", []) if summary else [],
                        "exclusionReasons": validation["exclusionReasons"] if summary else []})
    return {"mode": "dry_run", "readOnly": True, "trainingEligible": False,
            "executionState": "outcome_unknown" if issues else "see_attempts",
            "ok": not issues and all(not item["issues"] for item in reports),
            "issues": issues, "attempts": reports}
