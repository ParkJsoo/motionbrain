"""Append-only visual/action recording, local replay and read-only audit.

No camera, motor port, network or training is opened here. Host receipt windows
are diagnostic segments, never optical exposure bounds or physical settling.
The recorder is single-owner; execution and camera producers are connected later.
"""
from __future__ import annotations

import copy
from fractions import Fraction
import hashlib
from pathlib import Path
import re

from learning.action_execution import canonical_request, checked_record, finite, selected_outputs
from learning.artifacts import (ArtifactStore, StorageError, _decode_json, _read,
                                read_events, read_json_artifact)
from learning.runner import KNOWN_TERMINAL

SESSION_SCHEMA = "motionbrain.visual_action_session.v1"
SUMMARY_SCHEMA = "motionbrain.visual_action_summary.v1"
MAX_FRAME_BYTES = 32 * 1024 * 1024
ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}\Z")


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def identifier(value):
    require(isinstance(value, str) and ID.fullmatch(value), "invalid_identifier")
    return value


def integer(value, low, high):
    return type(value) is int and low <= value <= high


def validate_session(session):
    fields = {"schemaVersion", "sessionId", "viewId", "streamId", "hostClockId", "sourceClockId",
              "mode", "cameraViewFixed", "width", "height", "rowBytes", "videoMirrored",
              "videoRotationAngle", "maxAttempts", "maxFrames", "maxDurationSeconds"}
    require(isinstance(session, dict) and set(session) == fields, "session_fields_required")
    require(session["schemaVersion"] == SESSION_SCHEMA, "session_schema_mismatch")
    for key in ("sessionId", "viewId", "streamId", "hostClockId", "sourceClockId"):
        identifier(session[key])
    require(session["mode"] in ("synthetic", "offline_import"), "unsupported_collection_mode")
    require(type(session["cameraViewFixed"]) is bool and type(session["videoMirrored"]) is bool,
            "camera_flags_required")
    require(integer(session["width"], 1, 4096) and integer(session["height"], 1, 4096)
            and integer(session["rowBytes"], session["width"] * 4, session["width"] * 4 + 4096)
            and session["rowBytes"] * session["height"] <= MAX_FRAME_BYTES, "invalid_frame_shape")
    require(type(session["videoRotationAngle"]) is int
            and session["videoRotationAngle"] in (0, 90, 180, 270), "invalid_rotation")
    require(integer(session["maxAttempts"], 1, 100) and integer(session["maxFrames"], 1, 10000)
            and finite(session["maxDurationSeconds"]) and 0 < session["maxDurationSeconds"] <= 3600,
            "invalid_recording_budget")
    return copy.deepcopy(session)


class _Timeline:
    """Deterministic reducer shared by recording and audit; never sends commands."""
    def __init__(self, session):
        self.session = validate_session(session)
        self.attempts = []
        self.active = None
        self.ids, self.frame_ids = set(), set()
        self.first_at = self.last_at = self.last_pts = None
        self.halted = False
        self.stop_after_attempt = False
        self.controller_records = {}
        self.errors = []

    def apply(self, event, pixels=None, image=None):
        # Invalid input remains in the durable raw log, but never updates a valid
        # timeline partially. The public recorder commits this copy only on success.
        require(not self.halted, "session_halted")
        require(isinstance(event, dict), "event_object_required")
        common = {"type", "at", "hostClockId", "actionId"}
        extra = {
            "begin": {"episodeId", "startPoseId", "request"},
            "frame": {"frameId", "viewId", "streamId", "sourceClockId", "pts", "timestampSemantics",
                      "pixelFormat", "sha256", "features"},
            "dispatch": set(),
            "result": {"executionState", "record", "baselineOutputs", "outputLog"},
            "end": {"motionObservation", "settlingObservation"},
            "failure": {"reason"},
        }
        kind = event.get("type")
        require(isinstance(kind, str) and kind in extra, "unsupported_event")
        require(set(event) == common | extra[kind], "event_fields_required")
        require(event["hostClockId"] == self.session["hostClockId"], "host_clock_mismatch")
        at = event["at"]
        require(finite(at) and at >= 0, "invalid_event_time")
        require(self.last_at is None or at >= self.last_at, "host_time_regressed")
        require(self.first_at is None or at - self.first_at <= self.session["maxDurationSeconds"],
                "recording_time_budget_exceeded")
        action_id = identifier(event["actionId"])
        if kind == "begin":
            require(self.active is None, "action_already_active")
            require(action_id not in self.ids, "duplicate_action_id")
            require(len(self.attempts) < self.session["maxAttempts"], "attempt_budget_exceeded")
            request = canonical_request(event["request"])
            require(request["commandId"] == action_id, "command_action_id_mismatch")
            episode, pose = identifier(event["episodeId"]), identifier(event["startPoseId"])
            self.active = {"actionId": action_id, "episodeId": episode, "startPoseId": pose,
                           "leakageGroup": self.session["sessionId"], "request": request,
                           "beganAt": at, "dispatchAt": None, "terminalReceivedAt": None,
                           "executionState": "not_sent", "record": None, "outputTrace": None,
                           "windows": {"before": [], "during": [], "after": [], "unresolved": []},
                           "motionObservation": "unknown", "settlingObservation": "unverified",
                           "ended": False, "issues": []}
            self.ids.add(action_id)
            self.attempts.append(self.active)
        else:
            require(self.active is not None and self.active["actionId"] == action_id,
                    "active_action_mismatch")
            a = self.active
            if kind == "frame":
                self._frame(event, pixels, image)
            elif kind == "dispatch":
                require(not self.stop_after_attempt, "dispatch_blocked_after_failure")
                require(a["dispatchAt"] is None and a["terminalReceivedAt"] is None,
                        "duplicate_or_late_dispatch")
                require(bool(a["windows"]["before"]), "before_frame_required")
                require(a["windows"]["before"][-1]["receivedAt"] < at, "ambiguous_before_dispatch_order")
                a["dispatchAt"] = at
                a["executionState"] = "outcome_unknown"
            elif kind == "result":
                require(a["terminalReceivedAt"] is None, "duplicate_result")
                state = event["executionState"]
                require(state in ("not_sent", "outcome_unknown", "known_terminal"), "invalid_execution_state")
                require((state == "not_sent") == (a["dispatchAt"] is None), "dispatch_result_mismatch")
                if state == "known_terminal":
                    record = checked_record(event["record"], a["request"])
                    require(record["state"] in KNOWN_TERMINAL, "terminal_record_required")
                    previous = self.controller_records.get(record["bootId"])
                    if previous is not None:
                        require(record["ordinal"] > previous["ordinal"], "controller_ordinal_reused_or_regressed")
                        require((record["startedAtMs"] - previous["endedAtMs"]) % 2**32 < 2**31,
                                "controller_time_regressed")
                    a["record"] = record
                    # Missing output coverage does not discard an authentic terminal
                    # receipt, but stops the run and is reported as incomplete evidence.
                    try:
                        a["outputTrace"] = selected_outputs(event["outputLog"], record, event["baselineOutputs"])
                    except (ValueError, KeyError, TypeError):
                        a["issues"].append("output_coverage_unverified")
                        self.stop_after_attempt = True
                    if a["outputTrace"] is not None:
                        if previous is not None:
                            require(a["outputTrace"]["entries"][0]["sequence"] > previous["lastSequence"],
                                    "controller_output_sequence_reused_or_regressed")
                        self.controller_records[record["bootId"]] = {"ordinal": record["ordinal"],
                            "endedAtMs": record["endedAtMs"], "lastSequence": a["outputTrace"]["entries"][-1]["sequence"]}
                else:
                    require(all(event[k] is None for k in ("record", "baselineOutputs", "outputLog")),
                            "nonterminal_evidence_must_be_null")
                    a["issues"].append(state)
                    self.stop_after_attempt = True
                a["terminalReceivedAt"] = at
                a["executionState"] = state
                if a["record"] is not None and a["record"]["state"] != "completed":
                    self.stop_after_attempt = True
            elif kind == "end":
                require(a["terminalReceivedAt"] is not None or self.stop_after_attempt, "result_required_before_end")
                if a["terminalReceivedAt"] is None:
                    a["issues"].append("result_missing")
                require(event["motionObservation"] in ("observed", "no_visible_motion", "unknown"),
                        "invalid_motion_observation")
                require(event["settlingObservation"] in ("observed_still", "unverified"),
                        "invalid_settling_observation")
                a.update(motionObservation=event["motionObservation"],
                         settlingObservation=event["settlingObservation"], ended=True)
                for phase in ("before", "during", "after"):
                    if not a["windows"][phase]:
                        a["issues"].append("missing_" + phase + "_frames")
                if a["issues"] or self.stop_after_attempt:
                    self.halted = True
                self.active = None
            else:
                identifier(event["reason"])
                a["issues"].append(event["reason"])
                self.stop_after_attempt = True
        self.first_at = at if self.first_at is None else self.first_at
        self.last_at = at

    def _frame(self, event, pixels, image):
        s, a = self.session, self.active
        require(len(self.frame_ids) < s["maxFrames"], "frame_budget_exceeded")
        frame_id = identifier(event["frameId"])
        require(frame_id not in self.frame_ids, "duplicate_frame_id")
        for key in ("viewId", "streamId", "sourceClockId"):
            require(event[key] == s[key], "frame_" + key + "_mismatch")
        require(event["timestampSemantics"] == "presentation_time" and event["pixelFormat"] == "BGRA8",
                "unsupported_frame_format_or_time")
        pts = event["pts"]
        require(isinstance(pts, dict) and set(pts) == {"value", "timescale", "epoch"}
                and integer(pts["value"], 0, 2**63 - 1) and integer(pts["timescale"], 1, 2**31 - 1)
                and integer(pts["epoch"], 0, 2**63 - 1), "invalid_pts")
        current = (pts["epoch"], Fraction(pts["value"], pts["timescale"]))
        require(self.last_pts is None or (current[0] == self.last_pts[0] and current[1] > self.last_pts[1]),
                "pts_reset_repeated_or_regressed")
        require(isinstance(pixels, bytes) and len(pixels) == s["rowBytes"] * s["height"], "frame_bytes_missing_or_shape_mismatch")
        require(hashlib.sha256(pixels).hexdigest() == event["sha256"], "frame_hash_mismatch")
        require(isinstance(event["features"], list) and len(event["features"]) <= 128, "invalid_features")
        names = set()
        for f in event["features"]:
            require(isinstance(f, dict) and set(f) == {"id", "x", "y", "visible", "confidence"}, "feature_fields_required")
            name = identifier(f["id"])
            require(name not in names and type(f["visible"]) is bool, "duplicate_or_invalid_feature")
            names.add(name)
            require(finite(f["confidence"]) and 0 <= f["confidence"] <= 1, "invalid_feature_confidence")
            require((f["visible"] and finite(f["x"]) and finite(f["y"])
                     and 0 <= f["x"] < s["width"] and 0 <= f["y"] < s["height"])
                    or (not f["visible"] and f["x"] is None and f["y"] is None), "invalid_feature_coordinates")
        phase = "before" if a["dispatchAt"] is None else "during"
        if a["terminalReceivedAt"] is not None:
            require(event["at"] > a["terminalReceivedAt"], "ambiguous_after_result_order")
            phase = "after" if a["executionState"] == "known_terminal" else "unresolved"
        a["windows"][phase].append({"frameId": frame_id, "receivedAt": event["at"],
                                    "pts": copy.deepcopy(pts), "image": image,
                                    "features": copy.deepcopy(event["features"])})
        self.frame_ids.add(frame_id)
        self.last_pts = current

    def summary(self):
        attempts = copy.deepcopy(self.attempts)
        for a in attempts:
            if not a["ended"]:
                a["issues"].append("attempt_not_finished")
            if a["executionState"] == "outcome_unknown":
                a["status"] = "outcome_unknown"
            elif a["record"] is not None and a["record"]["state"] != "completed":
                a["status"] = "failed"
            elif a["issues"]:
                a["status"] = "incomplete"
            else:
                a["status"] = "recorded"
            a["trainingEligible"] = False
        return {"schemaVersion": SUMMARY_SCHEMA, "sessionId": self.session["sessionId"],
                "mode": self.session["mode"], "attemptCount": len(attempts), "frameCount": len(self.frame_ids),
                "attempts": attempts, "errors": copy.deepcopy(self.errors), "halted": self.halted,
                "newActionsBlocked": self.halted or self.stop_after_attempt,
                "windowBasis": "host_receipt_order_not_optical_exposure",
                "trainingEligible": False, "controlEligible": False, "physicalLearningEligibleRows": 0,
                "automaticRetry": False, "resumeSupported": False,
                "recordingComplete": bool(attempts) and not self.errors and all(a["status"] == "recorded" for a in attempts)}


class VisualActionCollector:
    """Single-owner event sink. Persist dispatch intent BEFORE calling any port.

    This object does not call that port, cancel it, or establish dispatch freshness.
    Only validated producer events are linked; invalid bytes/events are retained.
    After any storage exception, discard this object and audit the saved directory.
    """
    def __init__(self, output_dir, session):
        self.timeline = _Timeline(session)
        self.store = ArtifactStore(Path(output_dir))
        self.closed = self.storage_failed = False
        self.count = 0
        ref = self.store.write_json("session.json", self.timeline.session)
        self.store.append("session_started", "session", {"session": ref})

    @property
    def halted(self):
        return self.timeline.halted or self.storage_failed

    def record(self, event, pixels=None):
        if self.closed or self.halted:
            raise RuntimeError("collector_closed_or_halted")
        self.count += 1
        try:
            raw = self.store.write_json(f"raw/{self.count:08d}.json", event)
            image = None
            if pixels is not None:
                require(isinstance(pixels, bytes) and 0 < len(pixels) <= MAX_FRAME_BYTES, "invalid_raw_frame_size")
                image = self.store._write(f"frames/{self.count:08d}.bgra", pixels)
            self.store.append("input", "session", {"raw": raw, "image": image})
            candidate = copy.deepcopy(self.timeline)
            try:
                candidate.apply(event, pixels, image)
            except (ValueError, KeyError, TypeError) as exc:
                # Only internal validation codes are persisted, not arbitrary
                # transport exception strings, URLs or authentication headers.
                reason = str(exc) if isinstance(exc, ValueError) else "malformed_event"
                self.timeline.errors.append({"inputIndex": self.count, "reason": reason})
                self.timeline.halted = True
                return False
            self.timeline = candidate
            return not self.halted
        except BaseException:
            self.storage_failed = True
            raise

    def close(self):
        if self.closed or self.storage_failed:
            raise RuntimeError("collector_closed_or_storage_failed")
        try:
            result = self.timeline.summary()
            ref = self.store.write_json("summary.json", result)
            self.store.append("session_closed", "session", {"summary": ref, "inputCount": self.count})
            self.closed = True
            return result
        except BaseException:
            self.storage_failed = True
            raise


def audit_collection(directory):
    """Rebuild summary from immutable events and original bytes; never resume."""
    result = {"integrityOk": False, "sessionClosed": False, "recordingComplete": False,
              "trainingEligible": False, "physicalLearningEligibleRows": 0, "issues": [], "summary": None}
    try:
        events, errors = read_events(directory)
        result["issues"].extend(errors)
        require(bool(events) and events[0]["kind"] == "session_started", "session_start_missing")
        timeline = _Timeline(read_json_artifact(directory, events[0]["payload"]["session"]))
        count = 0
        for index, entry in enumerate(events[1:], 1):
            kind, payload = entry["kind"], entry["payload"]
            if kind == "session_closed":
                require(index == len(events) - 1, "events_after_close")
                require(payload["inputCount"] == count, "input_count_mismatch")
                require(read_json_artifact(directory, payload["summary"]) == timeline.summary(), "summary_mismatch")
                result["sessionClosed"] = True
            else:
                require(kind == "input" and not timeline.halted, "unexpected_event_after_halt")
                count += 1
                event = read_json_artifact(directory, payload["raw"])
                image = payload["image"]
                pixels = None
                if image is not None:
                    pixels = _read(directory, image["artifactRef"])
                    require(type(image["bytes"]) is int and len(pixels) == image["bytes"]
                            and hashlib.sha256(pixels).hexdigest() == image["sha256"], "frame_artifact_integrity_mismatch")
                candidate = copy.deepcopy(timeline)
                try:
                    candidate.apply(event, pixels, image)
                except (ValueError, KeyError, TypeError) as exc:
                    reason = str(exc) if isinstance(exc, ValueError) else "malformed_event"
                    timeline.errors.append({"inputIndex": count, "reason": reason})
                    timeline.halted = True
                else:
                    timeline = candidate
        result["summary"] = timeline.summary()
        result["integrityOk"] = not result["issues"]
        if not result["sessionClosed"]:
            result["issues"].append("session_close_missing")
        result["recordingComplete"] = result["integrityOk"] and result["sessionClosed"] and result["summary"]["recordingComplete"]
    except (StorageError, OSError, ValueError, KeyError, TypeError, UnicodeError) as exc:
        result["issues"].append(str(exc))
    return result


def replay_bundle(bundle_path, output_dir):
    """Import a local BGRA/event bundle. The bundle never contains executable code."""
    path = Path(bundle_path)
    bundle = _decode_json(_read(path.parent, path.name))
    require(isinstance(bundle, dict) and set(bundle) == {"session", "events"}, "invalid_bundle")
    require(isinstance(bundle["events"], list) and len(bundle["events"]) <= 30000, "invalid_bundle_events")
    collector = VisualActionCollector(output_dir, bundle["session"])
    try:
        for item in bundle["events"]:
            require(isinstance(item, dict) and set(item) == {"event", "imagePath"}, "invalid_bundle_item")
            pixels = None
            if item["imagePath"] is not None:
                try:
                    pixels = _read(path.parent, item["imagePath"])
                except StorageError:
                    # Persist the frame metadata and report missing bytes; do not
                    # replace it with a nearby or latest frame.
                    pixels = None
            if not collector.record(item["event"], pixels):
                break
    except KeyboardInterrupt:
        # A signal during persistence makes the store state uncertain. Preserve
        # the prefix and the original interrupt rather than masking it in close().
        if not collector.storage_failed and not collector.closed:
            try:
                collector.close()
            except (StorageError, OSError):
                pass
        raise
    return collector.close()
