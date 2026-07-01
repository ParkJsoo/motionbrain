import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "motionbrain_ros_bridge"))

from motionbrain_ros_bridge.motionbrain_kinematics import ArmModel  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import JointAngles  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import forward_kinematics  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import inverse_kinematics  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import joint_limit_violations  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import joint_positions_from_message  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import (  # noqa: E402
    validate_complete_finite_joint_positions,
)


class MotionBrainKinematicsTest(unittest.TestCase):
    def test_forward_kinematics_zero_pose(self) -> None:
        model = ArmModel()
        pose = forward_kinematics(JointAngles(), model)

        expected_radius = (
            model.shoulder_offset_m
            + model.upper_arm_m
            + model.forearm_m
            + model.wrist_m
            + model.tool_m
        )
        self.assertAlmostEqual(pose.x_m, expected_radius)
        self.assertAlmostEqual(pose.y_m, 0.0)
        self.assertAlmostEqual(pose.z_m, model.base_height_m)
        self.assertTrue(pose.within_joint_limits)

    def test_forward_kinematics_yaw_rotation(self) -> None:
        pose = forward_kinematics(JointAngles(base_yaw=math.pi / 2.0))

        self.assertAlmostEqual(pose.x_m, 0.0, places=7)
        self.assertGreater(pose.y_m, 0.7)

    def test_joint_limit_violations(self) -> None:
        violations = joint_limit_violations(
            JointAngles(shoulder_pitch=math.radians(100.0), gripper=math.radians(60.0)),
        )

        self.assertEqual(violations, ("shoulder_pitch_joint", "gripper_joint"))

    def test_inverse_kinematics_reachable_target(self) -> None:
        solution = inverse_kinematics(0.70, 0.0, 0.09)

        self.assertTrue(solution.reachable)
        self.assertEqual(solution.reason, "ok")
        pose = forward_kinematics(solution.joint_angles)
        self.assertAlmostEqual(pose.x_m, 0.70, places=2)
        self.assertAlmostEqual(pose.z_m, 0.09, places=2)

    def test_inverse_kinematics_outside_workspace(self) -> None:
        solution = inverse_kinematics(2.0, 0.0, 0.09)

        self.assertFalse(solution.reachable)
        self.assertEqual(solution.reason, "outside_workspace")

    def test_joint_state_message_mapping(self) -> None:
        positions = joint_positions_from_message(
            ["elbow_pitch_joint", "base_yaw_joint"],
            [0.2, 0.1],
        )
        angles = JointAngles.from_positions(positions)

        self.assertEqual(angles.base_yaw, 0.1)
        self.assertEqual(angles.elbow_pitch, 0.2)
        self.assertEqual(angles.shoulder_pitch, 0.0)

    def test_complete_finite_joint_state_validation(self) -> None:
        positions, errors = validate_complete_finite_joint_positions(
            [
                "base_yaw_joint",
                "shoulder_pitch_joint",
                "elbow_pitch_joint",
                "wrist_pitch_joint",
                "gripper_joint",
            ],
            [0.0, 0.1, -0.1, 0.0, 0.0],
        )

        self.assertEqual((), errors)
        self.assertEqual(0.1, positions["shoulder_pitch_joint"])

    def test_joint_state_validation_rejects_missing_and_nonfinite_inputs(self) -> None:
        _positions, errors = validate_complete_finite_joint_positions(
            ["base_yaw_joint", "shoulder_pitch_joint"],
            [0.0, math.nan],
        )

        self.assertIn("nonfinite:shoulder_pitch_joint", errors)
        self.assertIn("missing:elbow_pitch_joint", errors)
        self.assertIn("missing:wrist_pitch_joint", errors)
        self.assertIn("missing:gripper_joint", errors)


if __name__ == "__main__":
    unittest.main()
