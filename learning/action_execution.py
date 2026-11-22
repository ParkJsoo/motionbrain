"""One confirmed bounded action per durable ID; no automatic replay or re-arm.

The port is injected. This core neither enables firmware nor establishes physical
calibration. A terminal controller record proves software output-off only. Use a
separate journal from the legacy M4 executor, one owner per controller in deployment.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import re
import struct
import threading
import time
from contextlib import ExitStack
from pathlib import Path

from learning.runner import _action_matches, KNOWN_TERMINAL
from learning.action_journal import BoundedActionJournal

SCHEMA = "motionbrain.host_bounded_action.v1"
FIELDS = {"bootId", "commandId", "kind", "motor", "pwm", "durationMs", "targetDegrees", "percent"}


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def canonical_request(request):
    if not isinstance(request, dict) or set(request) != FIELDS:
        raise ValueError("canonical_action_fields_required")
    r = copy.deepcopy(request)
    if (not isinstance(r["bootId"], str) or not re.fullmatch(r"[0-9a-f]{32}", r["bootId"])
            or not isinstance(r["commandId"], str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,128}", r["commandId"])):
        raise ValueError("invalid_action_identity")
    for key, low, high in (("motor", 1, 5), ("pwm", -255, 255), ("durationMs", 1, 10000), ("percent", 0, 100)):
        if type(r[key]) is not int or not low <= r[key] <= high:
            raise ValueError("invalid_action_" + key)
    if not finite(r["targetDegrees"]) or not -360 <= r["targetDegrees"] <= 360:
        raise ValueError("invalid_target")
    # MCU stores float32 and reports six decimals. Reject values whose accepted
    # echo would differ; never silently execute a rounded target.
    echoed = round(struct.unpack("f", struct.pack("f", r["targetDegrees"]))[0], 6)
    if echoed != r["targetDegrees"]:
        raise ValueError("target_not_exactly_echoable")
    if r["kind"] == "pulse":
        if r["motor"] == 4 or r["pwm"] == 0 or r["targetDegrees"] != 0 or r["percent"] != 0:
            raise ValueError("invalid_pulse")
    elif r["kind"] == "shoulder":
        if r["motor"] != 4 or r["pwm"] != 0 or not 75 <= r["percent"] <= 100:
            raise ValueError("invalid_shoulder")
    else:
        raise ValueError("unsupported_action_kind")
    return r


class ActionLimits:
    """Explicit commissioning envelope, not a claim that these values are safe."""
    def __init__(self, *, motor, max_duration_ms, max_pwm, max_percent, min_degrees, max_degrees):
        for v, lo, hi in ((motor, 1, 5), (max_duration_ms, 1, 10000), (max_pwm, 0, 255), (max_percent, 0, 100)):
            if type(v) is not int or not lo <= v <= hi:
                raise ValueError("invalid_action_limits")
        if not all(finite(v) for v in (min_degrees, max_degrees)) or not -360 <= min_degrees <= max_degrees <= 360:
            raise ValueError("invalid_angle_limits")
        self.values = dict(motor=motor, max_duration_ms=max_duration_ms, max_pwm=max_pwm,
                           max_percent=max_percent, min_degrees=min_degrees, max_degrees=max_degrees)

    def validate(self, request):
        r, v = canonical_request(request), self.values
        if (r["motor"] != v["motor"] or r["durationMs"] > v["max_duration_ms"]
                or abs(r["pwm"]) > v["max_pwm"] or r["percent"] > v["max_percent"]
                or not v["min_degrees"] <= r["targetDegrees"] <= v["max_degrees"]):
            raise ValueError("outside_commissioning_limits")
        return r


def checked_record(record, request):
    if not _action_matches({"request": request, "record": record}, request):
        raise ValueError("action_record_mismatch")
    canonical_request({**{k: record[k] for k in FIELDS - {"pwm"}}, "pwm": record.get("requestedPwm")})
    for key in ("ordinal", "startedAtMs", "endedAtMs", "elapsedMs", "overrunMs"):
        v = record.get(key)
        if type(v) is not int or not (1 if key == "ordinal" else 0) <= v < 2**32:
            raise ValueError("invalid_action_record")
    for key in ("outputOff", "targetReached"):
        if type(record.get(key)) is not bool:
            raise ValueError("invalid_action_record")
    if record.get("state") not in KNOWN_TERMINAL | {"running", "outcome_unknown"}:
        raise ValueError("invalid_action_state")
    if not isinstance(record.get("reason"), str):
        raise ValueError("invalid_action_reason")
    if record["state"] in KNOWN_TERMINAL:
        if (record["outputOff"] is not True
                or (record["endedAtMs"]-record["startedAtMs"]) % 2**32 != record["elapsedMs"]
                or record["overrunMs"] != max(0, record["elapsedMs"]-request["durationMs"])):
            raise ValueError("invalid_action_terminal")
        state, reason, elapsed, reached = (record[k] for k in ("state", "reason", "elapsedMs", "targetReached"))
        if state == "completed":
            valid = record["overrunMs"] == 0 and (
                (request["kind"] == "pulse" and reason == "duration_elapsed" and elapsed == request["durationMs"] and not reached)
                or (request["kind"] == "shoulder" and reason == "target_reached" and reached))
        elif state == "timed_out":
            valid = reason == "deadline_exceeded" and elapsed >= request["durationMs"] and (
                request["kind"] == "shoulder" or (record["overrunMs"] > 0 and not reached)) and (
                not reached or record["overrunMs"] > 0)
        elif state == "cancelled":
            valid = reason in ("cancelled", "operator_override", "state_changed") and not reached
        else:
            valid = reason in ("start_failed", "safety_blocked", "controller_failed") and not reached and (
                reason != "controller_failed" or request["kind"] == "shoulder")
        if not valid:
            raise ValueError("inconsistent_action_terminal")
    keys = (FIELDS - {"pwm"}) | {"requestedPwm", "ordinal", "startedAtMs", "endedAtMs", "elapsedMs",
                               "overrunMs", "state", "reason", "outputOff", "targetReached"}
    return {k: record[k] for k in keys}


def output_snapshot_cursor(log):
    """Validate the retained interval of a full controller output snapshot."""
    if not isinstance(log, dict):
        raise ValueError("output_snapshot_interval_unverified")
    dropped, entries = log.get("dropped"), log.get("entries")
    if (type(dropped) is not int or not 0 <= dropped < 2**32
            or not isinstance(entries, list) or (dropped and not entries)
            or dropped + len(entries) >= 2**32
            or any(not isinstance(e, dict) or type(e.get("sequence")) is not int or e["sequence"] != seq
                   for seq, e in enumerate(entries, dropped + 1))):
        raise ValueError("output_snapshot_interval_unverified")
    return dropped + len(entries)


def selected_outputs(log, record, baseline, *, allow_rolled_history=False):
    if (not isinstance(log, dict) or log.get("bootId") != record["bootId"]
            or log.get("clockDomain") != "controller_boot_millis" or log.get("evidence") != "driver_write"
            or type(log.get("dropped")) is not int or log["dropped"] < 0 or not isinstance(log.get("entries"), list)):
        raise ValueError("output_log_unverified")
    if (not isinstance(baseline, dict) or baseline.get("bootId") != record["bootId"]
            or type(baseline.get("dropped")) is not int or baseline["dropped"] < 0
            or not isinstance(baseline.get("entries"), list)):
        raise ValueError("baseline_outputs_unverified")
    if allow_rolled_history:
        baseline_last = output_snapshot_cursor(baseline)
        off_cursor = baseline.get("verifiedOffSequence")
        if type(off_cursor) is not int or off_cursor != baseline_last:
            raise ValueError("output_window_unverified")
        return selected_output_window(log, record, off_cursor, baseline["dropped"])
    elif baseline["dropped"] or log["dropped"]:
        raise ValueError("output_log_unverified")
    else:
        baseline_last = baseline["entries"][-1]["sequence"] if baseline["entries"] else 0
    return _select_action_outputs(log, record, baseline_last)


def selected_output_window(log, record, off_cursor, previous_dropped):
    if (not isinstance(log, dict) or log.get("bootId") != record["bootId"]
            or log.get("clockDomain") != "controller_boot_millis"
            or log.get("evidence") != "driver_write" or type(off_cursor) is not int
            or type(previous_dropped) is not int or type(log.get("dropped")) is not int
            or not 0 <= previous_dropped <= log["dropped"] <= off_cursor):
        raise ValueError("output_window_unverified")
    return _select_action_outputs(log, record, off_cursor, terminal_cursor=output_snapshot_cursor(log))


def _select_action_outputs(log, record, baseline_last, *, terminal_cursor=None):
    entries, previous, previous_offset = [], 0, 0
    for entry in log["entries"]:
        if (not isinstance(entry, dict) or type(entry.get("sequence")) is not int
                or not previous < entry["sequence"] < 2**32
                or type(entry.get("actionOrdinal")) is not int or not 0 <= entry["actionOrdinal"] < 2**32):
            raise ValueError("output_sequence_invalid")
        previous = entry["sequence"]
        if entry.get("actionOrdinal") == record["ordinal"]:
            for key, low, high in (("atMs", 0, 2**32-1), ("motor", 1, 5), ("pwm", -255, 255)):
                if type(entry.get(key)) is not int or not low <= entry[key] <= high:
                    raise ValueError("output_entry_invalid")
            offset = (entry["atMs"]-record["startedAtMs"]) % 2**32
            if (entry.get("reason") not in ("driver_write", "forced_off")
                    or (entry["reason"] == "forced_off" and entry["pwm"] != 0)
                    or (entry["motor"] != record["motor"] and entry["pwm"] != 0)
                    or not previous_offset <= offset <= record["elapsedMs"]):
                raise ValueError("output_action_mismatch")
            previous_offset = offset
            entries.append(copy.deepcopy(entry))
    if (not entries or entries[0]["sequence"] != baseline_last+1 or any(b["sequence"] != a["sequence"]+1 for a,b in zip(entries,entries[1:]))
            or (terminal_cursor is not None and entries[-1]["sequence"] != terminal_cursor)
            or not any(e["motor"] == record["motor"] and e["pwm"] == 0 for e in entries)
            or next(e for e in reversed(entries) if e["motor"] == record["motor"])["pwm"] != 0):
        raise ValueError("output_coverage_unverified")
    window = {**{k: log[k] for k in ("bootId", "clockDomain", "evidence")}, "dropped": 0, "entries": entries}
    if log["dropped"]:
        window["historicalDropped"] = log["dropped"]
    return window


class BoundedActionExecutor:
    def __init__(self, journal_path, port, limits, *, clock=time, ttl_seconds=20, max_polls=20, poll_interval=.05):
        if not Path(journal_path).is_absolute() or str(journal_path) == ":memory:":
            raise ValueError("persistent_absolute_journal_required")
        if (not finite(ttl_seconds) or not 0 < ttl_seconds <= 20
                or type(max_polls) is not int or not 1 <= max_polls <= 200
                or not finite(poll_interval) or not 0 < poll_interval <= 1):
            raise ValueError("invalid_executor_budget")
        self.port, self.limits, self.clock = port, limits, clock
        self.ttl, self.max_polls, self.poll_interval = ttl_seconds, max_polls, poll_interval
        self.lock, self.failed = threading.RLock(), False
        self.stop_requested = threading.Event()
        self.journal = BoundedActionJournal(str(journal_path), 10000)
        try:
            for row in self.journal.records():
                if row["request"].get("executorSchema") != SCHEMA:
                    raise ValueError("journal_protocol_mismatch")
            for row in self.journal.interrupted():
                dispatched = row["phase"] == "dispatching"
                evidence = ({"outputBaseline": row["result"]["outputBaseline"]}
                            if dispatched and "outputBaseline" in (row.get("result") or {}) else {})
                self._save(row["commandId"], "outcome_unknown" if dispatched else "not_sent",
                           self._result(row["commandId"], "outcome_unknown" if dispatched else "not_sent",
                                        "executor_restarted", dispatched, **evidence))
        except BaseException:
            self.journal.close()
            raise

    def _now(self):
        now = self.clock.monotonic()
        if not finite(now) or now < 0:
            raise ValueError("invalid_clock")
        return now

    @staticmethod
    def _result(cid, state, reason, attempted=False, **extra):
        return dict(schemaVersion=SCHEMA, commandId=cid, executionState=state, reason=reason,
                    dispatchAttempted=attempted, automaticRetry=False, physicalArrivalVerified=False,
                    trainingEligible=False, **extra)

    def _save(self, cid, phase, result):
        try:
            self.journal.save(cid, phase, result)
        except BaseException:
            self.failed = True
            raise
        return copy.deepcopy(result)

    def propose(self, request):
        with self.lock:
            if self.failed:
                raise RuntimeError("journal_unavailable")
            r = self.limits.validate(request)
            fingerprint = hashlib.sha256(json.dumps(r, sort_keys=True, allow_nan=False).encode()).hexdigest()
            now = self._now()
            return self.journal.insert(r["commandId"], fingerprint,
                {"executorSchema": SCHEMA, "action": r, "createdMonotonic": now}, now+self.ttl,
                self._result(r["commandId"], "not_sent", "awaiting_confirmation"))

    def result(self, cid):
        with self.lock:
            try:
                row = self.journal.get(cid)
                return copy.deepcopy(row["result"]) if row else self._result(cid, "outcome_unknown", "not_found")
            except Exception:
                self.failed = True
                return self._result(cid, "outcome_unknown", "journal_unavailable")

    def confirm(self, cid, *, confirmed_request, dispatch_guard, motion_feedback=None, lookup_worker=None):
        """Caller must confirm the exact displayed request and supply a fresh gate.

        The guard runs after journal reservation and all preflight I/O, immediately
        before the submit callback. It must revalidate observation/M4 freshness and
        persistence, raising on failure. The finite pilot CLI supplies this guard.
        An optional prestarted read-only worker grants an exclusive per-action lease.
        While that GET is pending the owner continues checking motion feedback;
        only this owner may submit, cancel or update the command journal.
        """
        with self.lock, ExitStack() as leases:
            if self.failed:
                return self._result(cid, "outcome_unknown", "journal_unavailable")
            try:
                row, error = self.journal.reserve(cid)
                if error:
                    if row and error == "proposal_already_consumed":
                        return {**copy.deepcopy(row["result"]), "replayed": True}
                    if row and error in ("unresolved_outcome_unknown", "executor_busy"):
                        return self._save(cid,"not_sent",self._result(cid,"not_sent",error))
                    return self._result(cid, "not_sent", error)
                if motion_feedback is not None and not callable(motion_feedback):
                    raise ValueError('motion_feedback_callback_required')
                if lookup_worker is not None and motion_feedback is None:
                    raise ValueError('asynchronous_lookup_requires_active_monitor')
                r = self.limits.validate(row["request"]["action"])
                if canonical_request(confirmed_request) != r or not callable(dispatch_guard):
                    raise ValueError("explicit_matching_confirmation_required")
                def fresh():
                    if self.stop_requested.is_set():
                        raise InterruptedError("operator_stop_requested")
                    if not row["request"]["createdMonotonic"] <= self._now() < row["expiresAt"]:
                        raise ValueError("confirmation_expired_or_clock_regressed")
                fresh()
                lookup = None
                if lookup_worker is not None:
                    origin = self.port.transport.bases['controller']
                    lookup = leases.enter_context(lookup_worker.reserve(r,origin))
                baseline = self.port.preflight(r)
                rolled_history = getattr(self.port, "allow_rolled_history", False)
                baseline_evidence = ({"outputBaseline": {
                    "verifiedOffSequence": baseline["verifiedOffSequence"], "dropped": baseline["dropped"]}}
                    if rolled_history else {})
                fresh()
            except Exception as exc:
                if 'row' not in locals() or row is None:
                    self.failed = True
                    return self._result(cid, "outcome_unknown", "journal_unavailable")
                return self._save(cid, "not_sent", self._result(cid, "not_sent", type(exc).__name__))
            started = None
            visual_cancel = False
            cancel_applied = False
            try:
                # Crash after this write is conservatively unknown, even if the
                # final observation gate later stops the call before transport.
                self._save(cid, "dispatching", self._result(cid, "outcome_unknown", "dispatch_reserved", True,
                                                           **baseline_evidence))
                fresh()
                dispatch_guard(copy.deepcopy(r))
                if lookup is not None:
                    lookup.check_ready(self.port.transport.bases['controller'])
                fresh()
                started = self._now()
                ack = self.port.submit(r)
                received = self._now()
                if received < started:
                    raise ValueError("clock_regressed")
                if not isinstance(ack, dict) or ack.get("accepted") is not True or ack.get("code") not in ("accepted", "duplicate"):
                    raise ValueError("acceptance_unverified")
                # Duplicate at the first host send means an action may predate
                # these bounds. Preserve unknown rather than fabricate timing.
                if ack["code"] != "accepted":
                    raise ValueError("unexpected_controller_duplicate")
                record = checked_record(ack.get("record"), r)
                identity = (record["ordinal"], record["startedAtMs"])
                start_bounds = self._bounds(started, received, "submit_round_trip")
                deadline = started + r["durationMs"]/1000 + 1.0
                pending_lookup = None
                lookup_count = 0
                while True:
                    if self.stop_requested.is_set():
                        raise InterruptedError("operator_stop_requested")
                    if record["state"] in KNOWN_TERMINAL:
                        outputs = selected_outputs(self.port.outputs(), record, baseline,
                            allow_rolled_history=rolled_history)
                        action = {"request": r, "record": record, "outputLog": outputs,
                            "startHostBounds": start_bounds,
                            "outputOffHostBounds": self._bounds(started, received, "terminal_round_trip"),
                            "outputSequenceBounds": {"first": outputs["entries"][0]["sequence"],
                                "last": outputs["entries"][-1]["sequence"], "evidenceRef": "host-journal:"+cid}}
                        result = self._result(cid, "known_terminal", "controller_output_off", True, action=action)
                        if visual_cancel:
                            result.update(cancelAttempted=True, cancelConfirmed=cancel_applied and record['state'] == 'cancelled'
                                          and record['reason'] == 'cancelled')
                        return self._save(cid, "known_terminal", result)
                    if (record["state"] != "running" or self._now() >= deadline
                            or (pending_lookup is None and lookup_count >= self.max_polls)):
                        raise TimeoutError("terminal_not_confirmed")
                    if motion_feedback is not None:
                        decision = motion_feedback(copy.deepcopy(r), start_bounds['latest'])
                        if decision not in ('continue', 'cancel'):
                            raise ValueError('invalid_motion_feedback')
                        if decision == 'cancel':
                            # Reserve this write before transport. A lost response must not
                            # cause a second cancel from the generic exception handler.
                            visual_cancel = True
                            self._save(cid, 'outcome_unknown', self._result(
                                cid, 'outcome_unknown', 'visual_cancel_reserved', True,
                                cancelAttempted=True, cancelConfirmed=False, **baseline_evidence))
                            cancel_reply = self.port.cancel(r)
                            cancel_applied = cancel_reply.get('cancelApplied') is True
                            record = checked_record(cancel_reply, r)
                            received = self._now()
                            if ((record['ordinal'], record['startedAtMs']) != identity
                                    or record['state'] not in KNOWN_TERMINAL):
                                raise ValueError('visual_cancel_unverified')
                            continue
                    self.clock.sleep(self.poll_interval)
                    previous = received
                    if lookup is None:
                        lookup_count += 1
                        observed = self.port.lookup(r)
                    else:
                        if pending_lookup is None:
                            lookup_count += 1
                            pending_lookup = lookup.begin()
                        if not pending_lookup.done():
                            continue
                        observed = pending_lookup.result(timeout=0)
                        pending_lookup = None
                    record = checked_record(observed, r)
                    if (record["ordinal"], record["startedAtMs"]) != identity:
                        raise ValueError("action_identity_changed")
                    received = self._now()
                    if received < previous:
                        raise ValueError("clock_regressed")
            except BaseException as exc:
                if self.failed:
                    raise
                attempted = started is not None
                result = self._save(cid, "outcome_unknown" if attempted else "not_sent",
                    self._result(cid, "outcome_unknown" if attempted else "not_sent", type(exc).__name__, attempted,
                                 **({"cancelAttempted": True, "cancelConfirmed": False} if visual_cancel else {}),
                                 **(baseline_evidence if attempted else {})))
                if attempted:
                    # One best-effort targeted cancel, durably reserved first.
                    # Even a cancel acknowledgement never changes this unknown.
                    result = self.cancel(cid)
                if not isinstance(exc, Exception):
                    raise
                return result

    def _bounds(self, start, end, method):
        return dict(clockId=self.port.host_clock_id, earliest=start, latest=end,
                    method=method, evidenceRef="host-journal:causal-http")

    def request_stop(self):
        """Signal cancellation without waiting for the executor lock or blocking I/O."""
        self.stop_requested.set()

    def cancel(self, cid):
        with self.lock:
            if self.failed:
                return self._result(cid, "outcome_unknown", "journal_unavailable")
            row = self.journal.get(cid)
            if row is None or row["phase"] != "outcome_unknown" or row["result"].get("cancelAttempted"):
                return self.result(cid)
            result = {**row["result"], "cancelAttempted": True, "cancelConfirmed": False}
            self._save(cid, "outcome_unknown", result)
            try:
                record = checked_record(self.port.cancel(row["request"]["action"]), row["request"]["action"])
                result.update(cancelConfirmed=record["state"] in KNOWN_TERMINAL and record["outputOff"], cancelRecord=record)
            except Exception as exc:
                result["cancelError"] = type(exc).__name__
            return self._save(cid, "outcome_unknown", result)

    def inspect(self, cid):
        """Read controller evidence without clearing unknown, retrying, or re-arming."""
        with self.lock:
            row = self.journal.get(cid)
            if row is None:
                return self._result(cid, "outcome_unknown", "not_found")
            try:
                record = checked_record(self.port.lookup(row["request"]["action"]), row["request"]["action"])
                return {**self.result(cid), "inspectionRecord": record, "journalChanged": False}
            except Exception as exc:
                return {**self.result(cid), "inspectionError": type(exc).__name__, "journalChanged": False}

    def close(self):
        with self.lock:
            self.journal.close()
