import math
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "motionbrain_ros_bridge"))

from motionbrain_ros_bridge.motionbrain_kinematics import ArmModel  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import JointAngles  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import forward_kinematics  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import inverse_kinematics  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import joint_limit_violations  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import joint_positions_from_message  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import tool_orientation_quaternion
from motionbrain_ros_bridge.motionbrain_kinematics import (  # noqa: E402
    validate_complete_finite_joint_positions,
)


FULL_ARM_URDFS = [
    ROOT / "ros2_ws/src/motionbrain_description/urdf/motionbrain.urdf",
    ROOT / "ros2_ws/src/motionbrain_ros2_control_mock/urdf/motionbrain_mock_control.urdf",
    ROOT / "ros2_ws/src/motionbrain_hardware_interface/urdf/motionbrain_hardware_interface.urdf",
]


def xml_transform(path, positions, endpoint="tool0"):
    """Independent URDF parent-chain composition, using axis-angle rotations."""
    root = ET.parse(path).getroot()
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    chain = []
    while endpoint in by_child:
        joint = by_child[endpoint]
        chain.append(joint)
        endpoint = joint.find("parent").get("link")
    result = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find("origin")
        translation = np.eye(4)
        if origin is not None:
            translation[:3, 3] = list(map(float, origin.get("xyz", "0 0 0").split()))
            # These kinematic origins are unrotated; fail if the fixtures change.
            if any(map(float, origin.get("rpy", "0 0 0").split())):
                raise AssertionError("rotated joint origin needs independent RPY support")
        result = result @ translation
        if joint.get("type") == "revolute":
            axis = np.array(list(map(float, joint.find("axis").get("xyz").split())))
            axis /= np.linalg.norm(axis)
            x, y, z = axis
            skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            angle = positions.get(joint.get("name"), 0)
            rotation = np.eye(4)
            rotation[:3, :3] = np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew@skew)
            result = result @ rotation
    return result


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


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

    def test_fk_and_ros_orientation_match_all_full_arm_urdfs(self):
        rng = np.random.default_rng(42)
        poses = [[0, math.pi/6, 0, 0, 0], [0, 0, 0, 0, .5]]
        poses += rng.uniform(-.7, .7, (30, 5)).tolist()
        for path in FULL_ARM_URDFS:
            for values in poses:
                with self.subTest(path=path.name, joints=values):
                    angles = JointAngles(*values)
                    fk = forward_kinematics(angles)
                    transform = xml_transform(path, dict(zip([
                        "base_yaw_joint", "shoulder_pitch_joint", "elbow_pitch_joint",
                        "wrist_pitch_joint", "gripper_joint"], values)))
                    np.testing.assert_allclose(transform[:3, 3], [fk.x_m, fk.y_m, fk.z_m], atol=1e-10)
                    np.testing.assert_allclose(transform[:3, :3],
                        quaternion_matrix(tool_orientation_quaternion(fk.yaw_rad, fk.pitch_rad)), atol=1e-10)

    def test_ik_targets_match_independent_urdf_chain(self):
        for target, pitch in [((.70, 0, .09), 0), ((.73, .12, .18), .2), ((.70, -.1, .04), -.1)]:
            solution = inverse_kinematics(*target, target_tool_pitch_rad=pitch)
            self.assertTrue(solution.reachable, solution.reason)
            positions = dict(zip(["base_yaw_joint", "shoulder_pitch_joint", "elbow_pitch_joint",
                                  "wrist_pitch_joint", "gripper_joint"], solution.joint_angles.as_list()))
            for path in FULL_ARM_URDFS:
                np.testing.assert_allclose(xml_transform(path, positions)[:3, 3], target, atol=1e-10)

    def test_gripper_opening_does_not_move_tool_reference(self):
        for path in FULL_ARM_URDFS:
            a = xml_transform(path, {"shoulder_pitch_joint": .3, "gripper_joint": -.5})
            b = xml_transform(path, {"shoulder_pitch_joint": .3, "gripper_joint": .5})
            np.testing.assert_allclose(a, b, atol=1e-10)
            self.assertFalse(np.allclose(
                xml_transform(path, {"gripper_joint": -.5}, "gripper_jaw_link"),
                xml_transform(path, {"gripper_joint": .5}, "gripper_jaw_link")))

    def test_m4_only_models_use_same_elevation_direction(self):
        for name in ["motionbrain_m4_measured.urdf", "motionbrain_m4_proposal.urdf"]:
            path = ROOT / "ros2_ws/src/motionbrain_hardware_interface/urdf" / name
            actual = xml_transform(path, {"shoulder_pitch_joint": .4}, "upper_arm_link")
            expected = quaternion_matrix(tool_orientation_quaternion(0, .4))
            np.testing.assert_allclose(actual[:3, :3], expected, atol=1e-10)

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
