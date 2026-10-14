from __future__ import annotations

import json
import math
import re
import time
import urllib.parse
from typing import Any, Callable

from motionbrain_ros_bridge.m4_command_journal import M4CommandJournal
from motionbrain_ros_bridge.m4_write_contract import M4ContractError, validate_m4_request

FetchStatus = Callable[[], dict[str, Any]]
PostTarget = Callable[[str], dict[str, Any]]
_COMMAND_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}\Z")


class M4WriteExecutorCore:
    """At most one POST attempt per recorded ID, not physical exactly-once execution.

    `forwarded` and `dispatchAttempted` mean the POST callback was entered (or
    may have been entered when recovering a reserved dispatch after a crash). A true
    `success` means controller acceptance, never target attainment.
    `acknowledged` means a valid controller response, including a rejection.
    When journal reads fail, an unobserved ID has outcome_unknown even though
    dispatchAttempted is false: absent local evidence does not prove not_sent.
    In-memory journals only protect the lifetime of this object. Durable journals
    support restart recovery and exclusive ownership on macOS/Linux (flock).
    """

    def __init__(self, ttl_seconds: float = 20.0, *, journal_path: str = ":memory:",
                 max_records: int = 10000, clock: Callable[[], float] = time.monotonic) -> None:
        if isinstance(ttl_seconds, bool) or not math.isfinite(ttl_seconds) or ttl_seconds <= 0:
            raise ValueError("ttl_seconds must be finite and positive")
        self.ttl_seconds = ttl_seconds
        self.clock = clock
        self.journal = M4CommandJournal(journal_path, max_records)
        self._journal_failed = False
        self._last_results: dict[str, dict[str, Any]] = {}
        try:
            for record in self.journal.interrupted():
                attempted = record["phase"] == "dispatching"
                state = "outcome_unknown" if attempted else "canceled"
                result = self._result(record["commandId"], "executor_restarted", state,
                                      attempted=attempted)
                self.journal.save(record["commandId"], state, result)
                self._last_results[record["commandId"]] = result
        except Exception:
            self.journal.close()
            raise

    def close(self) -> None:
        self.journal.close()

    @staticmethod
    def _command_id(value: Any) -> str:
        if not isinstance(value, str) or not _COMMAND_ID.fullmatch(value):
            raise M4ContractError("invalid_command_id" if value else "command_id_required")
        return value

    @staticmethod
    def _request(proposal: dict[str, Any]) -> dict[str, Any]:
        command_id = M4WriteExecutorCore._command_id(proposal.get("commandId", ""))
        if proposal.get("joint") != "shoulder_pitch_joint":
            raise M4ContractError("unsupported_joint")
        if proposal.get("forwarded") is not False or proposal.get("operatorConfirmationRequired") is not True:
            raise M4ContractError("invalid_proposal_boundary")
        try:
            value = proposal["targetPositionRad"]
            if isinstance(value, bool):
                raise ValueError()
            target = float(value)
        except (KeyError, TypeError, ValueError, OverflowError):
            raise M4ContractError("invalid_target") from None
        if not math.isfinite(target):
            raise M4ContractError("non_finite_target")
        timeout = proposal.get("timeoutMs", 10000)
        if isinstance(timeout, str) and re.fullmatch(r"[0-9]+", timeout):
            try:
                timeout = int(timeout)
            except ValueError:
                raise M4ContractError("invalid_timeout") from None
        if isinstance(timeout, bool) or not isinstance(timeout, int) or not 500 <= timeout <= 10000:
            raise M4ContractError("invalid_timeout")
        request = dict(commandId=command_id, joint="shoulder_pitch_joint",
                       targetPositionRad=target, timeoutMs=timeout, mode="physical")
        # Source clock metadata is audit data, not comparable with this host's
        # monotonic clock. Identical retransmits retain the first metadata.
        for key in ("createdAt", "createdClockDomain", "schemaVersion"):
            if key in proposal:
                request[key] = proposal[key]
        return request

    def accept_proposal(self, proposal: dict[str, Any]) -> dict[str, Any]:
        request = self._request(proposal)
        command_id = request["commandId"]
        fingerprint = json.dumps({key: request[key] for key in
            ("joint", "targetPositionRad", "timeoutMs", "mode")}, sort_keys=True, allow_nan=False)
        result = self._result(command_id, "proposal_pending", "not_sent")
        result["accepted"] = True
        result = self._serialize(result)
        record = self.journal.insert(command_id, fingerprint, request,
                                     self.clock() + self.ttl_seconds, result)
        return dict(record["result"])

    def get_result(self, command_id: str) -> dict[str, Any] | None:
        if self._journal_failed:
            return self._unavailable_result(command_id)
        try:
            with self.journal.lock:
                record = self.journal.get(command_id)
                if record is None:
                    return None
                result = dict(record["result"])
                result["ledgerState"] = record["phase"]
                result["proposalMetadata"] = {key: record["request"][key] for key in
                    ("createdAt", "createdClockDomain", "schemaVersion") if key in record["request"]}
                result = self._serialize(result)
                self._last_results[command_id] = result
                return result
        except Exception:
            self._journal_failed = True
            return self._unavailable_result(command_id)

    def confirm(self, command_id: str, fetch_status: FetchStatus,
                post_target: PostTarget) -> dict[str, Any]:
        if self._journal_failed:
            return self._unavailable_result(command_id)
        try:
            record, rejection = self.journal.reserve(command_id)
        except Exception:
            self._journal_failed = True
            return self._unavailable_result(command_id)
        if rejection:
            result = self._result(command_id, rejection, "not_sent")
            if record is not None and rejection == "proposal_already_consumed":
                result = dict(record["result"])
                result.update(reason=rejection, message=rejection, replayed=True,
                              originalResult=record["result"])
            return self._serialize(result)
        if self.clock() >= record["expiresAt"]:
            return self._finish(command_id, "not_sent",
                                self._result(command_id, "proposal_expired", "not_sent"))
        try:
            request = dict(record["request"], confirmId="explicit-command-confirmation")
            validated = validate_m4_request(request, fetch_status())
        except M4ContractError as exc:
            return self._finish(command_id, "not_sent",
                self._result(command_id, exc.reason, "not_sent", detail=exc.detail))
        except Exception as exc:
            return self._finish(command_id, "not_sent", self._result(
                command_id, "status_fetch_failed", "not_sent", detail={"error": str(exc)}))
        # A slow status request must not extend the confirmation window.
        if self.clock() >= record["expiresAt"]:
            return self._finish(command_id, "not_sent",
                                self._result(command_id, "proposal_expired", "not_sent"))
        path = "/shoulder?" + urllib.parse.urlencode({
            "action": "angle", "degrees": f"{validated['requestedSensorDeg']:.6f}",
            "percent": "75"})
        # The legacy MCU endpoint does not support command IDs or timeoutMs.
        # Persist ambiguity BEFORE entering transport, including the crash gap.
        reserved = self._result(command_id, "dispatch_reserved", "outcome_unknown", attempted=True)
        try:
            with self.journal.lock:
                if self._journal_failed:
                    # A concurrent result query may have failed while status
                    # validation was in flight. This command has not been sent.
                    self._last_results[command_id] = self._result(
                        command_id, "journal_unavailable", "not_sent")
                    return self._unavailable_result(command_id)
                self.journal.save(command_id, "dispatching", reserved)
                self._last_results[command_id] = reserved
        except Exception:
            # This attempt has certainly not entered POST, regardless of whether
            # SQLite committed its reservation before reporting a write error.
            self._last_results[command_id] = self._result(
                command_id, "journal_unavailable", "not_sent")
            self._journal_failed = True
            return self._unavailable_result(command_id)
        try:
            response = post_target(path)
            if not isinstance(response, dict) or type(response.get("success")) is not bool:
                raise ValueError("invalid_controller_response")
            success = response["success"]
            result = self._result(command_id, "forwarded" if success else "controller_rejected",
                "accepted" if success else "failed", attempted=True, acknowledged=True,
                success=success, detail={"controllerResponse": response})
            result["message"] = str(response.get("message", response.get("error", result["reason"])))
            result = self._serialize(result)
        except Exception as exc:
            result = self._result(command_id, "transport_outcome_unknown", "outcome_unknown",
                                  attempted=True, detail={"error": str(exc)})
        return self._finish(command_id, result["executionState"], result)

    def _finish(self, command_id: str, phase: str, result: dict) -> dict:
        try:
            with self.journal.lock:
                self.journal.save(command_id, phase, result)
                self._last_results[command_id] = result
            return result
        except Exception:
            self._last_results[command_id] = self._result(command_id, "journal_unavailable",
                "outcome_unknown" if result["dispatchAttempted"] else "not_sent",
                attempted=result["dispatchAttempted"])
            self._journal_failed = True
            return self._unavailable_result(command_id)

    def _unavailable_result(self, command_id: str) -> dict:
        with self.journal.lock:
            previous = self._last_results.get(command_id)
            result = dict(previous) if previous is not None else self._result(
                command_id, "journal_unavailable", "outcome_unknown")
            result.update(reason="journal_unavailable", message="journal_unavailable",
                          journalAvailable=False)
            return self._serialize(result)

    @staticmethod
    def _serialize(result: dict) -> dict:
        result["rawJson"] = json.dumps({key: value for key, value in result.items() if key != "rawJson"},
                                       separators=(",", ":"), sort_keys=True, allow_nan=False)
        return result

    @staticmethod
    def _result(command_id: str, reason: str, state: str, *, attempted: bool = False,
                acknowledged: bool = False, success: bool = False, detail: dict | None = None) -> dict:
        payload = dict(success=success, forwarded=attempted, commandId=command_id,
                       reason=reason, message=reason, dispatchAttempted=attempted,
                       acknowledged=acknowledged, executionState=state)
        if detail:
            payload["detail"] = detail
        return M4WriteExecutorCore._serialize(payload)
