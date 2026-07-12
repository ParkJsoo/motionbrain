from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class M4WriteConfig:
    sensor_zero_deg: float = 222.80
    direction_sign: int = 1
    ros_joint_zero_rad: float = 0.0
    min_sensor_deg: float = 230.0
    max_sensor_deg: float = 245.0
    min_timeout_ms: int = 500
    max_timeout_ms: int = 10000


class M4ContractError(ValueError):
    def __init__(self, reason: str, **detail: Any) -> None:
        super().__init__(reason)
        self.reason = reason
        self.detail = detail


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
    if target_deg < config.min_sensor_deg or target_deg > config.max_sensor_deg:
        raise M4ContractError(
            "target_out_of_range",
            requestedSensorDeg=target_deg,
            allowedMinDeg=config.min_sensor_deg,
            allowedMaxDeg=config.max_sensor_deg,
        )
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
        "allowedMinDeg": config.min_sensor_deg,
        "allowedMaxDeg": config.max_sensor_deg,
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
