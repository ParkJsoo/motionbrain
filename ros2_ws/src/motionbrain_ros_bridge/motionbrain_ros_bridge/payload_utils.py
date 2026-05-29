import json
from typing import Any


ALIGN_DEADBAND = 0.15


def parse_light_action(payload: str) -> str | None:
    text = payload.strip().lower()
    if text in {"on", "off", "toggle"}:
        return text

    try:
        data = json.loads(payload)
    except json.JSONDecodeError:
        return None

    if not isinstance(data, dict):
        return None

    action = str(data.get("action", "")).strip().lower()
    if action in {"on", "off", "toggle"}:
        return action
    return None


def compact_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "on", "armed", "active"}:
            return True
        if text in {"0", "false", "no", "off", "idle", "stopped"}:
            return False
    return default


def as_float(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def as_uint(value: Any, default: int = 0) -> int:
    if value is None:
        return default
    try:
        return max(int(value), 0)
    except (TypeError, ValueError):
        return default


def as_str(value: Any, default: str = "") -> str:
    if value is None:
        return default
    return str(value)


def classify_alignment(offset_x: float | None, deadband: float = ALIGN_DEADBAND) -> str:
    if offset_x is None:
        return "LOST"
    if offset_x < -deadband:
        return "LEFT"
    if offset_x > deadband:
        return "RIGHT"
    return "CENTER"


def command_suggestion_for_alignment(alignment: str) -> str:
    if alignment == "LEFT":
        return "base_left"
    if alignment == "RIGHT":
        return "base_right"
    if alignment == "CENTER":
        return "hold"
    return "none"

