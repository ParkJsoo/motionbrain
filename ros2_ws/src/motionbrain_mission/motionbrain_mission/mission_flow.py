import json
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional


class MissionState(str, Enum):
    IDLE = "IDLE"
    WAIT_DETECTION = "WAIT_DETECTION"
    ALIGN = "ALIGN"
    WAIT_CONFIRM = "WAIT_CONFIRM"
    COMPLETE = "COMPLETE"
    BLOCKED = "BLOCKED"
    CANCELED = "CANCELED"


@dataclass
class DetectionSnapshot:
    available: bool = False
    detected: bool = False
    alignment: str = "LOST"
    command_suggestion: str = "none"
    area_ratio: float = 0.0


@dataclass
class GuardSnapshot:
    ready: bool = False
    reason: str = "missing_guard"
    suggested_action: str = "none"
    status_fresh: bool = False
    detection_fresh: bool = False


@dataclass
class MissionConfig:
    require_center_alignment: bool = True
    require_guard_ready: bool = True
    act_action: str = "toggle"


@dataclass
class MissionDecision:
    state: MissionState
    reason: str
    next_step: str
    suggested_action: str
    act_request: Optional[str] = None


class MissionFlow:
    def __init__(self, config: Optional[MissionConfig] = None) -> None:
        self.config = config or MissionConfig()
        self.state = MissionState.IDLE
        self.reason = "idle"
        self.detection = DetectionSnapshot()
        self.guard = GuardSnapshot()

    def update_detection(
        self,
        *,
        available: bool,
        detected: bool,
        alignment: str,
        command_suggestion: str,
        area_ratio: float,
    ) -> MissionDecision:
        self.detection = DetectionSnapshot(
            available=available,
            detected=detected,
            alignment=alignment or "LOST",
            command_suggestion=command_suggestion or "none",
            area_ratio=area_ratio,
        )
        return self.evaluate()

    def update_guard_json(self, data: str) -> MissionDecision:
        try:
            payload: Dict[str, Any] = json.loads(data)
        except json.JSONDecodeError:
            self.guard = GuardSnapshot(reason="invalid_guard_json")
            return self.evaluate()

        return self.update_guard(
            ready=bool(payload.get("ready", False)),
            reason=str(payload.get("reason", "unknown")),
            suggested_action=str(payload.get("suggestedAction", "none")),
            status_fresh=bool(payload.get("statusFresh", False)),
            detection_fresh=bool(payload.get("detectionFresh", False)),
        )

    def update_guard(
        self,
        *,
        ready: bool,
        reason: str,
        suggested_action: str,
        status_fresh: bool,
        detection_fresh: bool,
    ) -> MissionDecision:
        self.guard = GuardSnapshot(
            ready=ready,
            reason=reason or "unknown",
            suggested_action=suggested_action or "none",
            status_fresh=status_fresh,
            detection_fresh=detection_fresh,
        )
        return self.evaluate()

    def handle_command(self, command: str) -> MissionDecision:
        normalized = parse_command(command)
        if normalized == "start":
            self.state = MissionState.WAIT_DETECTION
            self.reason = "started"
            return self.evaluate()
        if normalized == "cancel":
            self.state = MissionState.CANCELED
            self.reason = "operator_cancel"
            return self.decision("none")
        if normalized == "reset":
            self.state = MissionState.IDLE
            self.reason = "reset"
            return self.decision("none")
        if normalized == "confirm":
            if self.state != MissionState.WAIT_CONFIRM:
                self.reason = "confirm_ignored"
                return self.evaluate()
            if self.config.require_guard_ready and not self.guard.ready:
                self.state = MissionState.BLOCKED
                self.reason = f"guard_{self.guard.reason}"
                return self.decision("operator_check")
            self.state = MissionState.COMPLETE
            self.reason = "operator_confirmed"
            return self.decision("complete", act_request=self.config.act_action)

        self.reason = "unknown_command"
        return self.evaluate()

    def evaluate(self) -> MissionDecision:
        if self.state in {MissionState.IDLE, MissionState.CANCELED, MissionState.COMPLETE}:
            return self.decision("none")

        if self.config.require_guard_ready and not self.guard.ready:
            self.state = MissionState.BLOCKED
            self.reason = f"guard_{self.guard.reason}"
            return self.decision("operator_check")

        if self.state == MissionState.BLOCKED:
            if self.config.require_guard_ready and not self.guard.ready:
                return self.decision("operator_check")
            self.state = MissionState.WAIT_DETECTION
            self.reason = "guard_recovered"

        if not self.detection.available:
            self.state = MissionState.WAIT_DETECTION
            self.reason = "camera_unavailable"
            return self.decision("wait_for_detection")
        if not self.detection.detected:
            self.state = MissionState.WAIT_DETECTION
            self.reason = "target_not_detected"
            return self.decision("wait_for_detection")

        if self.config.require_center_alignment and self.detection.alignment != "CENTER":
            self.state = MissionState.ALIGN
            self.reason = "alignment_required"
            return self.decision(self.detection.command_suggestion or self.guard.suggested_action)

        self.state = MissionState.WAIT_CONFIRM
        self.reason = "operator_confirm_required"
        return self.decision("operator_confirm")

    def decision(self, next_step: str, act_request: Optional[str] = None) -> MissionDecision:
        suggestion = self.detection.command_suggestion or self.guard.suggested_action or "none"
        return MissionDecision(
            state=self.state,
            reason=self.reason,
            next_step=next_step,
            suggested_action=suggestion,
            act_request=act_request,
        )

    def to_dict(self, decision: Optional[MissionDecision] = None) -> dict[str, Any]:
        current = decision or self.decision("none")
        return {
            "state": current.state.value,
            "reason": current.reason,
            "nextStep": current.next_step,
            "suggestedAction": current.suggested_action,
            "guardReady": self.guard.ready,
            "guardReason": self.guard.reason,
            "statusFresh": self.guard.status_fresh,
            "detectionFresh": self.guard.detection_fresh,
            "targetDetected": self.detection.detected,
            "alignment": self.detection.alignment,
            "areaRatio": round(self.detection.area_ratio, 6),
        }

    def to_json(self, decision: Optional[MissionDecision] = None) -> str:
        return json.dumps(
            self.to_dict(decision),
            separators=(",", ":"),
        )


def parse_command(data: str) -> str:
    stripped = (data or "").strip()
    if not stripped:
        return ""
    if stripped.startswith("{"):
        try:
            payload = json.loads(stripped)
        except json.JSONDecodeError:
            return ""
        return str(payload.get("command", payload.get("action", ""))).strip().lower()
    return stripped.lower()
