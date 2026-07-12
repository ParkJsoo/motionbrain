import json
from dataclasses import dataclass
from typing import Any


SAFE_HOLD_ACTION = "hold"
ASK_OPERATOR_ACTION = "ask_operator"
LIGHT_TOGGLE_ACTION = "light_toggle"
CUP_GRASP_PLAN_ACTION = "cup_grasp_plan"
ALIGN_LEFT_ACTION = "align_left"
ALIGN_RIGHT_ACTION = "align_right"


@dataclass
class PolicyStatusSnapshot:
    available: bool = False
    state: str = "UNKNOWN"
    moving: bool = False
    faulted: bool = False
    base_active: bool = False
    safety_blocked: bool = False
    fault_latched: bool = False


@dataclass
class PolicyDetectionSnapshot:
    available: bool = False
    detected: bool = False
    fresh: bool = False
    held: bool = False
    alignment: str = "LOST"
    command_suggestion: str = "none"
    label: str = ""
    color: str = ""
    confidence: float | None = None
    area_ratio: float = 0.0


@dataclass
class PolicyGuardSnapshot:
    ready: bool = False
    reason: str = "missing_guard"
    status_fresh: bool = False
    detection_fresh: bool = False


@dataclass
class PolicyConfig:
    target_label: str = "cup"
    min_confidence: float = 0.50
    allow_motion_candidates: bool = True


@dataclass
class PolicyProposal:
    instruction: str
    action: str
    confidence: float
    reason: str
    requires_operator_confirm: bool
    physical_motion_candidate: bool
    preconditions: list[str]

    def to_dict(self) -> dict[str, Any]:
        return {
            "instruction": self.instruction,
            "action": self.action,
            "confidence": round(self.confidence, 6),
            "reason": self.reason,
            "requiresOperatorConfirm": self.requires_operator_confirm,
            "physicalMotionCandidate": self.physical_motion_candidate,
            "preconditions": list(self.preconditions),
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), separators=(",", ":"), sort_keys=True)


def propose_policy_action(
    *,
    instruction: str = "",
    status: PolicyStatusSnapshot | None = None,
    detection: PolicyDetectionSnapshot | None = None,
    guard: PolicyGuardSnapshot | None = None,
    config: PolicyConfig | None = None,
) -> PolicyProposal:
    status = status or PolicyStatusSnapshot()
    detection = detection or PolicyDetectionSnapshot()
    guard = guard or PolicyGuardSnapshot()
    config = config or PolicyConfig()
    instruction = instruction.strip()

    if not detection.detected:
        target_confidence = 0.0
    else:
        target_confidence = normalized_confidence(detection.confidence)

    normalized_instruction = instruction.lower()
    target_label = normalized_label(config.target_label)
    detected_label = normalized_label(detection.label or detection.color)

    if wants_light(normalized_instruction):
        blockers = policy_blockers(
            status,
            detection,
            guard,
            require_armed=False,
            require_guard=False,
            require_detection=False,
            require_base_idle=False,
        )
        if blockers:
            return proposal(
                instruction,
                SAFE_HOLD_ACTION,
                1.0,
                blockers[0],
                requires_operator_confirm=False,
                physical_motion_candidate=False,
                preconditions=blockers,
            )
        return proposal(
            instruction,
            LIGHT_TOGGLE_ACTION,
            1.0,
            "operator_confirmed_non_motion_candidate",
            requires_operator_confirm=True,
            physical_motion_candidate=False,
            preconditions=["status_fresh", "operator_confirm"],
        )

    blockers = policy_blockers(
        status,
        detection,
        guard,
        require_armed=True,
        require_guard=True,
        require_detection=True,
        require_base_idle=True,
    )
    if blockers:
        return proposal(
            instruction,
            SAFE_HOLD_ACTION,
            1.0,
            blockers[0],
            requires_operator_confirm=False,
            physical_motion_candidate=False,
            preconditions=blockers,
        )

    if not detection.detected:
        return proposal(
            instruction,
            ASK_OPERATOR_ACTION,
            0.0,
            "target_not_detected",
            requires_operator_confirm=False,
            physical_motion_candidate=False,
            preconditions=["fresh_detection", "target_detected"],
        )

    if wants_cup_plan(normalized_instruction, target_label):
        cup_blockers = cup_plan_blockers(detection, detected_label, target_label, config)
        if cup_blockers:
            return proposal(
                instruction,
                ASK_OPERATOR_ACTION,
                target_confidence,
                cup_blockers[0],
                requires_operator_confirm=False,
                physical_motion_candidate=False,
                preconditions=cup_blockers,
            )
        return proposal(
            instruction,
            CUP_GRASP_PLAN_ACTION,
            target_confidence,
            "cup_centered_plan_ready",
            requires_operator_confirm=True,
            physical_motion_candidate=False,
            preconditions=[
                "guard_ready",
                "fresh_detection",
                "target_cup",
                "center_alignment",
                "operator_confirm",
            ],
        )

    alignment = detection.alignment.upper()
    if alignment in {"LEFT", "RIGHT"}:
        action = ALIGN_LEFT_ACTION if alignment == "LEFT" else ALIGN_RIGHT_ACTION
        return proposal(
            instruction,
            action if config.allow_motion_candidates else ASK_OPERATOR_ACTION,
            target_confidence,
            "alignment_nudge_candidate" if config.allow_motion_candidates else "motion_candidates_disabled",
            requires_operator_confirm=config.allow_motion_candidates,
            physical_motion_candidate=config.allow_motion_candidates,
            preconditions=[
                "guard_ready",
                "state_armed",
                "fresh_detection",
                "base_idle",
                "operator_confirm",
            ],
        )

    if alignment == "CENTER":
        return proposal(
            instruction,
            SAFE_HOLD_ACTION,
            target_confidence,
            "target_centered",
            requires_operator_confirm=False,
            physical_motion_candidate=False,
            preconditions=["guard_ready", "fresh_detection"],
        )

    return proposal(
        instruction,
        ASK_OPERATOR_ACTION,
        target_confidence,
        "alignment_lost",
        requires_operator_confirm=False,
        physical_motion_candidate=False,
        preconditions=["valid_alignment"],
    )


def policy_blockers(
    status: PolicyStatusSnapshot,
    detection: PolicyDetectionSnapshot,
    guard: PolicyGuardSnapshot,
    *,
    require_armed: bool,
    require_guard: bool,
    require_detection: bool,
    require_base_idle: bool,
) -> list[str]:
    blockers: list[str] = []
    if not guard.status_fresh:
        blockers.append("status_stale")
    if not status.available:
        blockers.append("status_unavailable")
    if status.faulted or status.fault_latched:
        blockers.append("faulted")
    if status.safety_blocked:
        blockers.append("safety_blocked")
    if status.moving:
        blockers.append("already_moving")
    if require_base_idle and status.base_active:
        blockers.append("base_busy")
    if require_armed and status.state != "ARMED":
        blockers.append("state_not_armed")
    if require_guard and not guard.ready:
        blockers.append(f"guard_{guard.reason or 'not_ready'}")
    if require_detection and (not guard.detection_fresh or not detection.fresh):
        blockers.append("detection_stale")
    if require_detection and not detection.available:
        blockers.append("detection_unavailable")
    if require_detection and detection.held:
        blockers.append("held_detection")
    return blockers


def cup_plan_blockers(
    detection: PolicyDetectionSnapshot,
    detected_label: str,
    target_label: str,
    config: PolicyConfig,
) -> list[str]:
    blockers: list[str] = []
    if detected_label != target_label:
        blockers.append("target_mismatch")
    confidence = normalized_confidence(detection.confidence)
    if confidence < config.min_confidence:
        blockers.append("confidence_below_threshold")
    if detection.alignment.upper() != "CENTER":
        blockers.append("alignment_not_center")
    return blockers


def proposal(
    instruction: str,
    action: str,
    confidence: float,
    reason: str,
    *,
    requires_operator_confirm: bool,
    physical_motion_candidate: bool,
    preconditions: list[str],
) -> PolicyProposal:
    return PolicyProposal(
        instruction=instruction,
        action=action,
        confidence=max(0.0, min(float(confidence), 1.0)),
        reason=reason,
        requires_operator_confirm=requires_operator_confirm,
        physical_motion_candidate=physical_motion_candidate,
        preconditions=preconditions,
    )


def normalized_label(value: str) -> str:
    return " ".join(value.strip().lower().replace("_", " ").split())


def normalized_confidence(value: float | None) -> float:
    if value is None:
        return 0.0
    return max(0.0, min(float(value), 1.0))


def wants_light(instruction: str) -> bool:
    return any(token in instruction for token in ("light", "lamp", "search light", "라이트", "조명"))


def wants_cup_plan(instruction: str, target_label: str) -> bool:
    if target_label and target_label in instruction:
        return True
    return any(token in instruction for token in ("cup", "grasp", "pick", "컵", "집", "잡"))
