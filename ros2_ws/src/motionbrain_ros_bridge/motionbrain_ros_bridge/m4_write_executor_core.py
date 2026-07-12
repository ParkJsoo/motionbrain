from __future__ import annotations

import json
import time
import urllib.parse
from dataclasses import dataclass
from typing import Any, Callable

from motionbrain_ros_bridge.m4_write_contract import M4ConfirmationStore
from motionbrain_ros_bridge.m4_write_contract import M4ContractError
from motionbrain_ros_bridge.m4_write_contract import validate_m4_request


FetchStatus = Callable[[], dict[str, Any]]
PostTarget = Callable[[str], dict[str, Any]]


@dataclass
class PendingProposal:
    request: dict[str, Any]
    expires_at: float
    consumed: bool = False


class M4WriteExecutorCore:
    def __init__(self, ttl_seconds: float = 20.0) -> None:
        self.ttl_seconds = ttl_seconds
        self.pending: dict[str, PendingProposal] = {}

    def accept_proposal(self, proposal: dict[str, Any]) -> dict[str, Any]:
        command_id = str(proposal.get("commandId", "")).strip()
        if not command_id:
            raise M4ContractError("command_id_required")
        if proposal.get("joint") != "shoulder_pitch_joint":
            raise M4ContractError("unsupported_joint")
        if proposal.get("forwarded") is not False or proposal.get("operatorConfirmationRequired") is not True:
            raise M4ContractError("invalid_proposal_boundary")
        request = {
            "commandId": command_id,
            "joint": "shoulder_pitch_joint",
            "targetPositionRad": float(proposal["targetPositionRad"]),
            "timeoutMs": int(proposal.get("timeoutMs", 10000)),
            "mode": "physical",
        }
        self.pending[command_id] = PendingProposal(request, time.time() + self.ttl_seconds)
        return {"accepted": True, "commandId": command_id, "forwarded": False}

    def confirm(
        self,
        command_id: str,
        fetch_status: FetchStatus,
        post_target: PostTarget,
    ) -> dict[str, Any]:
        item = self.pending.get(command_id)
        if item is None:
            return self._rejection(command_id, "proposal_not_found")
        if item.consumed:
            return self._rejection(command_id, "proposal_already_consumed")
        item.consumed = True
        if time.time() > item.expires_at:
            return self._rejection(command_id, "proposal_expired")
        try:
            store = M4ConfirmationStore()
            confirmation = store.issue(item.request)
            request = dict(item.request, confirmId=confirmation["confirmId"])
            store.consume(confirmation["confirmId"], request)
            validated = validate_m4_request(request, fetch_status())
            path = "/shoulder?" + urllib.parse.urlencode(
                {
                    "action": "angle",
                    "degrees": f"{validated['requestedSensorDeg']:.6f}",
                    "percent": "75",
                }
            )
            response = post_target(path)
            success = bool(response.get("success", False))
            return {
                "success": success,
                "forwarded": True,
                "commandId": command_id,
                "reason": "forwarded" if success else "controller_rejected",
                "message": str(response.get("message", response.get("error", ""))),
                "rawJson": json.dumps(response, separators=(",", ":"), sort_keys=True),
            }
        except M4ContractError as exc:
            return self._rejection(command_id, exc.reason, exc.detail)
        except Exception as exc:
            return self._rejection(command_id, "transport_error", {"error": str(exc)})

    @staticmethod
    def _rejection(command_id: str, reason: str, detail: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = {
            "success": False,
            "forwarded": False,
            "commandId": command_id,
            "reason": reason,
            "message": reason,
        }
        if detail:
            payload["detail"] = detail
        payload["rawJson"] = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        return payload
