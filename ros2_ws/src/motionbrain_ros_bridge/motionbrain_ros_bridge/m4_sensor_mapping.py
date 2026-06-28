import math
from dataclasses import dataclass


def _finite_float(value: float, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def _direction_sign(value: int) -> int:
    if isinstance(value, bool):
        raise ValueError("direction_sign must be -1 or 1")
    if value not in (-1, 1):
        raise ValueError("direction_sign must be -1 or 1")
    return int(value)


@dataclass(frozen=True)
class SensorJointCalibration:
    sensor_zero_deg: float
    direction_sign: int
    ros_joint_zero_rad: float

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "sensor_zero_deg",
            _finite_float(self.sensor_zero_deg, "sensor_zero_deg"),
        )
        object.__setattr__(
            self,
            "direction_sign",
            _direction_sign(self.direction_sign),
        )
        object.__setattr__(
            self,
            "ros_joint_zero_rad",
            _finite_float(self.ros_joint_zero_rad, "ros_joint_zero_rad"),
        )


@dataclass(frozen=True)
class JointLimit:
    lower: float
    upper: float

    def __post_init__(self) -> None:
        lower = _finite_float(self.lower, "lower")
        upper = _finite_float(self.upper, "upper")
        if lower > upper:
            raise ValueError("lower must be less than or equal to upper")
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)


def sensor_degrees_to_ros_radians(
    sensor_angle_deg: float,
    calibration: SensorJointCalibration,
) -> float:
    sensor_angle = _finite_float(sensor_angle_deg, "sensor_angle_deg")
    return calibration.ros_joint_zero_rad + math.radians(
        calibration.direction_sign * (sensor_angle - calibration.sensor_zero_deg)
    )


def ros_radians_to_sensor_degrees(
    ros_joint_rad: float,
    calibration: SensorJointCalibration,
) -> float:
    ros_joint = _finite_float(ros_joint_rad, "ros_joint_rad")
    return calibration.sensor_zero_deg + math.degrees(
        (ros_joint - calibration.ros_joint_zero_rad) / calibration.direction_sign
    )


def sensor_soft_limits_to_ros_joint_limits(
    sensor_soft_min_deg: float,
    sensor_soft_max_deg: float,
    calibration: SensorJointCalibration,
) -> JointLimit:
    lower_sensor = _finite_float(sensor_soft_min_deg, "sensor_soft_min_deg")
    upper_sensor = _finite_float(sensor_soft_max_deg, "sensor_soft_max_deg")
    if lower_sensor > upper_sensor:
        raise ValueError(
            "sensor_soft_min_deg must be less than or equal to sensor_soft_max_deg"
        )

    mapped = (
        sensor_degrees_to_ros_radians(lower_sensor, calibration),
        sensor_degrees_to_ros_radians(upper_sensor, calibration),
    )
    return JointLimit(lower=min(mapped), upper=max(mapped))


def shoulder_feedback_to_ros_joint_position(
    *,
    calibration_enabled: bool,
    feedback_available: bool,
    sensor_connected: bool,
    sensor_fresh: bool,
    sensor_ready: bool,
    shoulder_angle_deg: float,
    legacy_tilt_angle_deg: float,
    calibration: SensorJointCalibration | None = None,
) -> float:
    if not feedback_available:
        return math.radians(float(legacy_tilt_angle_deg))

    if not calibration_enabled:
        return math.nan

    if not (sensor_connected and sensor_fresh and sensor_ready):
        return math.nan

    if calibration is None:
        raise ValueError("calibration is required when shoulder feedback calibration is enabled")

    try:
        return sensor_degrees_to_ros_radians(shoulder_angle_deg, calibration)
    except ValueError:
        return math.nan


def shoulder_feedback_to_measured_ros_joint_position(
    *,
    calibration_enabled: bool,
    feedback_available: bool,
    sensor_connected: bool,
    sensor_fresh: bool,
    sensor_ready: bool,
    shoulder_angle_deg: float,
    calibration: SensorJointCalibration | None = None,
) -> float:
    if not feedback_available:
        return math.nan

    if not calibration_enabled:
        return math.nan

    if not (sensor_connected and sensor_fresh and sensor_ready):
        return math.nan

    if calibration is None:
        raise ValueError("calibration is required when shoulder feedback calibration is enabled")

    try:
        return sensor_degrees_to_ros_radians(shoulder_angle_deg, calibration)
    except ValueError:
        return math.nan
