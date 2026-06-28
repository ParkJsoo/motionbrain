import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "motionbrain_ros_bridge"))

from motionbrain_ros_bridge.m4_sensor_mapping import JointLimit  # noqa: E402
from motionbrain_ros_bridge.m4_sensor_mapping import SensorJointCalibration  # noqa: E402
from motionbrain_ros_bridge.m4_sensor_mapping import ros_radians_to_sensor_degrees  # noqa: E402
from motionbrain_ros_bridge.m4_sensor_mapping import sensor_degrees_to_ros_radians  # noqa: E402
from motionbrain_ros_bridge.m4_sensor_mapping import sensor_soft_limits_to_ros_joint_limits  # noqa: E402
from motionbrain_ros_bridge.m4_sensor_mapping import shoulder_feedback_to_measured_ros_joint_position  # noqa: E402
from motionbrain_ros_bridge.m4_sensor_mapping import shoulder_feedback_to_ros_joint_position  # noqa: E402


def shoulder_calibration(direction_sign: int = 1) -> SensorJointCalibration:
    return SensorJointCalibration(
        sensor_zero_deg=234.0,
        direction_sign=direction_sign,
        ros_joint_zero_rad=0.0,
    )


class M4SensorMappingTest(unittest.TestCase):
    def test_sensor_zero_234_degrees_maps_to_joint_zero_radians(self) -> None:
        calibration = shoulder_calibration()

        self.assertEqual(
            sensor_degrees_to_ros_radians(234.0, calibration),
            0.0,
        )
        self.assertAlmostEqual(
            sensor_degrees_to_ros_radians(244.0, calibration),
            math.radians(10.0),
        )
        self.assertAlmostEqual(
            sensor_degrees_to_ros_radians(230.0, calibration),
            math.radians(-4.0),
        )

    def test_direction_sign_reverses_sensor_to_ros_joint_direction(self) -> None:
        calibration = shoulder_calibration(direction_sign=-1)

        self.assertAlmostEqual(
            sensor_degrees_to_ros_radians(244.0, calibration),
            math.radians(-10.0),
        )
        self.assertAlmostEqual(
            sensor_degrees_to_ros_radians(230.0, calibration),
            math.radians(4.0),
        )

    def test_sensor_soft_limits_map_to_ordered_ros_joint_limits(self) -> None:
        calibration = shoulder_calibration(direction_sign=-1)

        limits = sensor_soft_limits_to_ros_joint_limits(230.0, 245.0, calibration)

        self.assertEqual(type(limits), JointLimit)
        self.assertLessEqual(limits.lower, limits.upper)
        self.assertAlmostEqual(limits.lower, math.radians(-11.0))
        self.assertAlmostEqual(limits.upper, math.radians(4.0))

    def test_calibration_requires_explicit_inputs(self) -> None:
        with self.assertRaises(TypeError):
            SensorJointCalibration()  # type: ignore[call-arg]

    def test_rejects_invalid_sign_and_nonfinite_values(self) -> None:
        for invalid_sign in (0, 2, -2, True):
            with self.subTest(invalid_sign=invalid_sign):
                with self.assertRaises(ValueError):
                    SensorJointCalibration(
                        sensor_zero_deg=234.0,
                        direction_sign=invalid_sign,
                        ros_joint_zero_rad=0.0,
                    )

        calibration = shoulder_calibration()
        invalid_values = (math.nan, math.inf, -math.inf)
        for invalid_value in invalid_values:
            with self.subTest(invalid_value=invalid_value):
                with self.assertRaises(ValueError):
                    sensor_degrees_to_ros_radians(invalid_value, calibration)
                with self.assertRaises(ValueError):
                    ros_radians_to_sensor_degrees(invalid_value, calibration)
                with self.assertRaises(ValueError):
                    sensor_soft_limits_to_ros_joint_limits(
                        230.0,
                        invalid_value,
                        calibration,
                    )

    def test_sensor_to_ros_round_trip_preserves_calibrated_sensor_degrees(self) -> None:
        calibration = SensorJointCalibration(
            sensor_zero_deg=234.0,
            direction_sign=-1,
            ros_joint_zero_rad=math.radians(5.0),
        )

        for sensor_angle_deg in (230.0, 234.0, 241.5, 245.0):
            with self.subTest(sensor_angle_deg=sensor_angle_deg):
                ros_joint_rad = sensor_degrees_to_ros_radians(
                    sensor_angle_deg,
                    calibration,
                )
                self.assertAlmostEqual(
                    ros_radians_to_sensor_degrees(ros_joint_rad, calibration),
                    sensor_angle_deg,
                )

    def test_ready_shoulder_feedback_uses_calibrated_sensor_angle(self) -> None:
        position = shoulder_feedback_to_ros_joint_position(
            calibration_enabled=True,
            feedback_available=True,
            sensor_connected=True,
            sensor_fresh=True,
            sensor_ready=True,
            shoulder_angle_deg=244.0,
            legacy_tilt_angle_deg=30.0,
            calibration=shoulder_calibration(direction_sign=-1),
        )

        self.assertAlmostEqual(position, math.radians(-10.0))

    def test_ready_measured_shoulder_feedback_uses_calibrated_sensor_angle(self) -> None:
        position = shoulder_feedback_to_measured_ros_joint_position(
            calibration_enabled=True,
            feedback_available=True,
            sensor_connected=True,
            sensor_fresh=True,
            sensor_ready=True,
            shoulder_angle_deg=244.0,
            calibration=shoulder_calibration(direction_sign=-1),
        )

        self.assertAlmostEqual(position, math.radians(-10.0))

    def test_disabled_calibration_publishes_nan_for_available_feedback(self) -> None:
        position = shoulder_feedback_to_ros_joint_position(
            calibration_enabled=False,
            feedback_available=True,
            sensor_connected=True,
            sensor_fresh=True,
            sensor_ready=True,
            shoulder_angle_deg=244.0,
            legacy_tilt_angle_deg=30.0,
            calibration=None,
        )

        self.assertTrue(math.isnan(position))

    def test_measured_shoulder_feedback_never_falls_back_to_estimated_tilt(self) -> None:
        unavailable_position = shoulder_feedback_to_measured_ros_joint_position(
            calibration_enabled=True,
            feedback_available=False,
            sensor_connected=True,
            sensor_fresh=True,
            sensor_ready=True,
            shoulder_angle_deg=244.0,
            calibration=shoulder_calibration(),
        )
        disabled_position = shoulder_feedback_to_measured_ros_joint_position(
            calibration_enabled=False,
            feedback_available=True,
            sensor_connected=True,
            sensor_fresh=True,
            sensor_ready=True,
            shoulder_angle_deg=244.0,
            calibration=None,
        )

        self.assertTrue(math.isnan(unavailable_position))
        self.assertTrue(math.isnan(disabled_position))

    def test_stale_or_not_ready_shoulder_feedback_publishes_nan(self) -> None:
        states = [
            {"sensor_connected": False, "sensor_fresh": True, "sensor_ready": True},
            {"sensor_connected": True, "sensor_fresh": False, "sensor_ready": True},
            {"sensor_connected": True, "sensor_fresh": True, "sensor_ready": False},
        ]
        for state in states:
            with self.subTest(state=state):
                position = shoulder_feedback_to_ros_joint_position(
                    calibration_enabled=True,
                    feedback_available=True,
                    shoulder_angle_deg=244.0,
                    legacy_tilt_angle_deg=30.0,
                    calibration=shoulder_calibration(),
                    **state,
                )

                self.assertTrue(math.isnan(position))

    def test_stale_or_not_ready_measured_feedback_publishes_nan(self) -> None:
        states = [
            {"sensor_connected": False, "sensor_fresh": True, "sensor_ready": True},
            {"sensor_connected": True, "sensor_fresh": False, "sensor_ready": True},
            {"sensor_connected": True, "sensor_fresh": True, "sensor_ready": False},
        ]
        for state in states:
            with self.subTest(state=state):
                position = shoulder_feedback_to_measured_ros_joint_position(
                    calibration_enabled=True,
                    feedback_available=True,
                    shoulder_angle_deg=244.0,
                    calibration=shoulder_calibration(),
                    **state,
                )

                self.assertTrue(math.isnan(position))

    def test_nonfinite_ready_feedback_publishes_nan(self) -> None:
        for invalid_angle in (math.nan, math.inf, -math.inf):
            with self.subTest(invalid_angle=invalid_angle):
                position = shoulder_feedback_to_ros_joint_position(
                    calibration_enabled=True,
                    feedback_available=True,
                    sensor_connected=True,
                    sensor_fresh=True,
                    sensor_ready=True,
                    shoulder_angle_deg=invalid_angle,
                    legacy_tilt_angle_deg=30.0,
                    calibration=shoulder_calibration(),
                )

                self.assertTrue(math.isnan(position))

    def test_nonfinite_ready_measured_feedback_publishes_nan(self) -> None:
        for invalid_angle in (math.nan, math.inf, -math.inf):
            with self.subTest(invalid_angle=invalid_angle):
                position = shoulder_feedback_to_measured_ros_joint_position(
                    calibration_enabled=True,
                    feedback_available=True,
                    sensor_connected=True,
                    sensor_fresh=True,
                    sensor_ready=True,
                    shoulder_angle_deg=invalid_angle,
                    calibration=shoulder_calibration(),
                )

                self.assertTrue(math.isnan(position))

    def test_legacy_tilt_fallback_only_when_shoulder_feedback_unavailable(self) -> None:
        position = shoulder_feedback_to_ros_joint_position(
            calibration_enabled=False,
            feedback_available=False,
            sensor_connected=False,
            sensor_fresh=False,
            sensor_ready=False,
            shoulder_angle_deg=244.0,
            legacy_tilt_angle_deg=30.0,
            calibration=None,
        )

        self.assertAlmostEqual(position, math.radians(30.0))

    def test_enabled_ready_feedback_requires_calibration(self) -> None:
        with self.assertRaises(ValueError):
            shoulder_feedback_to_ros_joint_position(
                calibration_enabled=True,
                feedback_available=True,
                sensor_connected=True,
                sensor_fresh=True,
                sensor_ready=True,
                shoulder_angle_deg=244.0,
                legacy_tilt_angle_deg=30.0,
                calibration=None,
            )


if __name__ == "__main__":
    unittest.main()
