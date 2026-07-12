from __future__ import annotations

import math
import secrets
import threading
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class M4WriteConfig:
    sensor_zero_deg: float = 222.80
    direction_sign: int = 1
    ros_joint_zero_rad: float = 0.0
    min_timeout_ms: int = 500
    max_timeout_ms: int = 10000


class M4ContractError(ValueError):
    def __init__(self, reason: str, **detail: Any) -> None:
        super().__init__(reason)
        self.reason = reason
        self.detail = detail


def confirmation_fingerprint(
    request: dict[str, Any], config: M4WriteConfig | None = None
) -> tuple[str, float, int, str]:
    config = config or M4WriteConfig()
    if str(request.get("joint", "")) != "shoulder_pitch_joint":
        raise M4ContractError("unsupported_joint")
    mode = str(request.get("mode", ""))
    if mode not in {"shadow", "physical"}:
        raise M4ContractError("invalid_mode")
    try:
        target_rad = float(request["targetPositionRad"])
        timeout_ms = int(request.get("timeoutMs", 5000))
    except (KeyError, TypeError, ValueError):
        raise M4ContractError("invalid_confirmation_request") from None
    if not config.min_timeout_ms <= timeout_ms <= config.max_timeout_ms:
        raise M4ContractError("invalid_timeout")
    target_deg = sensor_deg_from_ros_rad(target_rad, config)
    return ("shoulder_pitch_joint", round(target_deg, 6), timeout_ms, mode)


class M4ConfirmationStore:
    def __init__(self, ttl_seconds: float = 20.0) -> None:
        self.ttl_seconds = ttl_seconds
        self.lock = threading.Lock()
        self.pending: dict[str, dict[str, Any]] = {}

    def issue(self, request: dict[str, Any]) -> dict[str, Any]:
        fingerprint = confirmation_fingerprint(request)
        now = time.time()
        confirm_id = secrets.token_urlsafe(18)
        with self.lock:
            self.pending[confirm_id] = {
                "fingerprint": fingerprint,
                "expiresAt": now + self.ttl_seconds,
                "consumed": False,
            }
        return {
            "confirmId": confirm_id,
            "expiresAt": now + self.ttl_seconds,
            "fingerprint": {
                "joint": fingerprint[0],
                "targetSensorDeg": fingerprint[1],
                "timeoutMs": fingerprint[2],
                "mode": fingerprint[3],
            },
        }

    def consume(self, confirm_id: str, request: dict[str, Any]) -> None:
        fingerprint = confirmation_fingerprint(request)
        now = time.time()
        with self.lock:
            item = self.pending.get(confirm_id)
            if item is None:
                raise M4ContractError("confirmation_not_found")
            if item["consumed"]:
                raise M4ContractError("confirmation_already_consumed")
            if now > item["expiresAt"]:
                item["consumed"] = True
                raise M4ContractError("confirmation_expired")
            item["consumed"] = True
            if item["fingerprint"] != fingerprint:
                raise M4ContractError("confirmation_command_mismatch")


def sensor_deg_from_ros_rad(position_rad: float, config: M4WriteConfig) -> float:
    if not math.isfinite(position_rad):
        raise M4ContractError("non_finite_target")
    if config.direction_sign not in {-1, 1}:
        raise M4ContractError("calibration_unavailable", field="direction_sign")
    return config.sensor_zero_deg + config.direction_sign * math.degrees(
        position_rad - config.ros_joint_zero_rad
    )


def ros_rad_from_sensor_deg(sensor_deg: float, config: M4WriteConfig | None = None) -> float:
    config = config or M4WriteConfig()
    if not math.isfinite(sensor_deg):
        raise M4ContractError("non_finite_sensor_position")
    if config.direction_sign not in {-1, 1}:
        raise M4ContractError("calibration_unavailable", field="direction_sign")
    return config.ros_joint_zero_rad + config.direction_sign * math.radians(
        sensor_deg - config.sensor_zero_deg
    )


def validate_m4_request(
    request: dict[str, Any],
    status: dict[str, Any],
    config: M4WriteConfig | None = None,
) -> dict[str, Any]:
    config = config or M4WriteConfig()
    command_id = str(request.get("commandId", "")).strip()
    if not command_id:
        raise M4ContractError("command_id_required")
    if str(request.get("joint", "")) != "shoulder_pitch_joint":
        raise M4ContractError("unsupported_joint")
    if str(request.get("mode", "")) not in {"shadow", "physical"}:
        raise M4ContractError("invalid_mode")
    if not str(request.get("confirmId", "")).strip():
        raise M4ContractError("operator_confirmation_required")
    try:
        target_rad = float(request["targetPositionRad"])
    except (KeyError, TypeError, ValueError):
        raise M4ContractError("invalid_target") from None
    try:
        timeout_ms = int(request.get("timeoutMs", 5000))
    except (TypeError, ValueError):
        raise M4ContractError("invalid_timeout") from None
    if not config.min_timeout_ms <= timeout_ms <= config.max_timeout_ms:
        raise M4ContractError("invalid_timeout")
    target_deg = sensor_deg_from_ros_rad(target_rad, config)
    sensor = status.get("sensor") if isinstance(status.get("sensor"), dict) else {}
    shoulder = status.get("shoulderAngle") if isinstance(status.get("shoulderAngle"), dict) else {}
    motors = status.get("motors") if isinstance(status.get("motors"), dict) else {}
    teleop = status.get("teleop") if isinstance(status.get("teleop"), dict) else {}
    if status.get("state") != "ARMED":
        raise M4ContractError("state_not_armed")
    if sensor.get("faultLatched"):
        raise M4ContractError("fault_latched")
    if sensor.get("blocked"):
        raise M4ContractError("safety_blocked", blockReason=sensor.get("blockReason", "UNKNOWN"))
    if not shoulder.get("available") or not shoulder.get("sensorConnected"):
        raise M4ContractError("sensor_unavailable")
    if not shoulder.get("sensorFresh"):
        raise M4ContractError("sensor_stale")
    if not shoulder.get("sensorReady"):
        raise M4ContractError("sensor_not_ready")
    try:
        allowed_min_deg = float(shoulder["softMinDeg"])
        allowed_max_deg = float(shoulder["softMaxDeg"])
    except (KeyError, TypeError, ValueError):
        raise M4ContractError("soft_limits_unavailable") from None
    if (
        not math.isfinite(allowed_min_deg)
        or not math.isfinite(allowed_max_deg)
        or allowed_min_deg >= allowed_max_deg
    ):
        raise M4ContractError("soft_limits_invalid")
    if target_deg < allowed_min_deg or target_deg > allowed_max_deg:
        raise M4ContractError(
            "target_out_of_range",
            requestedSensorDeg=target_deg,
            allowedMinDeg=allowed_min_deg,
            allowedMaxDeg=allowed_max_deg,
        )
    if shoulder.get("active"):
        raise M4ContractError("already_moving")
    if teleop.get("controlActive") or teleop.get("deadman"):
        raise M4ContractError("teleop_active")
    for motor_id, motor in motors.items():
        if motor_id == "M4" or not isinstance(motor, dict):
            continue
        if motor.get("enabled") or int(motor.get("speed", 0) or 0) != 0:
            raise M4ContractError("other_motor_active", motor=motor_id)
    return {
        "commandId": command_id,
        "joint": "shoulder_pitch_joint",
        "mode": request["mode"],
        "confirmId": str(request["confirmId"]),
        "requestedPositionRad": target_rad,
        "requestedSensorDeg": target_deg,
        "timeoutMs": timeout_ms,
        "allowedMinDeg": allowed_min_deg,
        "allowedMaxDeg": allowed_max_deg,
    }


def rejection_payload(error: M4ContractError, command_id: str = "") -> dict[str, Any]:
    return {
        "accepted": False,
        "executed": False,
        "forwarded": False,
        "simulated": True,
        "commandId": command_id,
        "reason": error.reason,
        **error.detail,
    }
