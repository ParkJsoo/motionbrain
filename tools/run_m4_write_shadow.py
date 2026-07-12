#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from datetime import UTC
from datetime import datetime
from pathlib import Path
from typing import Any
from typing import Callable


ROOT = Path(__file__).resolve().parents[1]
BRIDGE_SRC = ROOT / "ros2_ws" / "src" / "motionbrain_ros_bridge"
if str(BRIDGE_SRC) not in sys.path:
    sys.path.insert(0, str(BRIDGE_SRC))

from motionbrain_ros_bridge.m4_write_contract import M4ConfirmationStore  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import M4ContractError  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import M4WriteConfig  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import ros_rad_from_sensor_deg  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import validate_m4_request  # noqa: E402


FetchJson = Callable[[str, float], dict[str, Any]]
SCHEMA_VERSION = "motionbrain.m4_write_shadow.v1"


def fetch_json(url: str, timeout: float) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("status_not_object")
    return payload


def status_evidence(status: dict[str, Any]) -> dict[str, Any]:
    sensor = status.get("sensor") if isinstance(status.get("sensor"), dict) else {}
    shoulder = status.get("shoulderAngle") if isinstance(status.get("shoulderAngle"), dict) else {}
    teleop = status.get("teleop") if isinstance(status.get("teleop"), dict) else {}
    motors = status.get("motors") if isinstance(status.get("motors"), dict) else {}
    return {
        "state": status.get("state", "UNKNOWN"),
        "motorEnabled": bool(status.get("motorEnabled", False)),
        "sensorBlocked": bool(sensor.get("blocked", False)),
        "faultLatched": bool(sensor.get("faultLatched", False)),
        "teleopDeadman": bool(teleop.get("deadman", False)),
        "teleopControlActive": bool(teleop.get("controlActive", False)),
        "shoulder": {
            "available": bool(shoulder.get("available", False)),
            "connected": bool(shoulder.get("sensorConnected", False)),
            "fresh": bool(shoulder.get("sensorFresh", False)),
            "ready": bool(shoulder.get("sensorReady", False)),
            "active": bool(shoulder.get("active", False)),
            "angleDeg": shoulder.get("angleDeg"),
            "ageMs": shoulder.get("ageMs"),
        },
        "motors": {
            key: {
                "enabled": bool(value.get("enabled", False)),
                "speed": int(value.get("speed", 0) or 0),
            }
            for key, value in motors.items()
            if isinstance(value, dict)
        },
    }


def evaluate_shadow_request(
    *,
    status_url: str,
    target_sensor_deg: float,
    command_id: str,
    timeout_ms: int = 5000,
    http_timeout: float = 6.0,
    fetch_json_func: FetchJson = fetch_json,
) -> dict[str, Any]:
    config = M4WriteConfig()
    request = {
        "commandId": command_id,
        "joint": "shoulder_pitch_joint",
        "targetPositionRad": ros_rad_from_sensor_deg(target_sensor_deg, config),
        "timeoutMs": timeout_ms,
        "mode": "shadow",
    }
    evidence: dict[str, Any] = {
        "schemaVersion": SCHEMA_VERSION,
        "createdAt": datetime.now(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        "statusUrl": status_url,
        "request": dict(request),
        "transport": {"method": "GET_STATUS_ONLY", "postAttempted": False, "forwarded": False},
        "accepted": False,
        "wouldExecute": False,
    }
    try:
        confirmation_store = M4ConfirmationStore()
        confirmation = confirmation_store.issue(request)
        request["confirmId"] = confirmation["confirmId"]
        confirmation_store.consume(confirmation["confirmId"], request)
        status = fetch_json_func(status_url, http_timeout)
        evidence["status"] = status_evidence(status)
        validated = validate_m4_request(request, status, config)
        evidence.update(
            {
                "accepted": True,
                "wouldExecute": True,
                "reason": "shadow_request_valid",
                "confirmation": confirmation,
                "validatedRequest": validated,
            }
        )
    except M4ContractError as exc:
        evidence.update({"reason": exc.reason, "detail": exc.detail})
    except Exception as exc:
        evidence.update({"reason": "status_unavailable", "detail": {"error": str(exc)}})
    return evidence


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate an M4 write against live status without POSTing.")
    parser.add_argument("--status-url", required=True)
    parser.add_argument("--target-deg", type=float, required=True)
    parser.add_argument("--command-id", default="")
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--http-timeout", type=float, default=6.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    command_id = args.command_id.strip() or f"shadow-{int(time.time() * 1000)}"
    evidence = evaluate_shadow_request(
        status_url=args.status_url,
        target_sensor_deg=args.target_deg,
        command_id=command_id,
        timeout_ms=args.timeout_ms,
        http_timeout=args.http_timeout,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        f"M4 shadow: accepted={evidence['accepted']} "
        f"would_execute={evidence['wouldExecute']} reason={evidence.get('reason')}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
