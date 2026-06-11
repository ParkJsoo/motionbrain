import json
import shlex
import urllib.parse
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


def normalize_routine_action(value: Any) -> str:
    text = str(value or "").strip().lower().replace("-", "_")
    aliases = {
        "dryrun": "dry_run",
        "dry_run": "dry_run",
        "status": "status",
        "list": "status",
        "abort": "abort",
        "run": "run",
        "execute": "run",
    }
    return aliases.get(text, text)


def parse_routine_command(payload: str) -> dict[str, str] | None:
    text = payload.strip()
    if not text:
        return None

    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        data = None

    if isinstance(data, dict):
        action = normalize_routine_action(data.get("action") or data.get("routineAction"))
        routine_name = as_str(data.get("name") or data.get("routineName")).strip()
        confirm_code = as_str(
            data.get("confirm")
            or data.get("confirmCode")
            or data.get("confirmationCode")
        ).strip()
        if not action:
            return None
        return {
            "action": action,
            "routine_name": routine_name,
            "confirm_code": confirm_code,
        }

    if data is not None:
        return None

    try:
        tokens = shlex.split(text)
    except ValueError:
        tokens = text.split()
    if not tokens:
        return None

    if tokens[0].lower() == "routine" and len(tokens) > 1:
        tokens = tokens[1:]

    action = normalize_routine_action(tokens[0])
    routine_name = ""
    confirm_code = ""
    for token in tokens[1:]:
        key, separator, value = token.partition("=")
        if separator and key.strip().lower() in {"confirm", "confirm_code", "confirmcode"}:
            confirm_code = value.strip()
        elif not routine_name:
            routine_name = token.strip()

    if not action:
        return None
    return {
        "action": action,
        "routine_name": routine_name,
        "confirm_code": confirm_code,
    }


def compact_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def perception_detection_url(perception_url: str) -> str:
    text = perception_url.strip().rstrip("/")
    if not text:
        return ""

    parsed = urllib.parse.urlparse(text)
    path = parsed.path.rstrip("/")
    if path.endswith("/api/detection"):
        return text
    if path.endswith("/api"):
        path = f"{path}/detection"
    else:
        path = f"{path}/api/detection"
    return urllib.parse.urlunparse(parsed._replace(path=path))


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
