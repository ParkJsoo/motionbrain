import math
import copy
import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "motionbrain_ros_bridge"))

from motionbrain_ros_bridge.motionbrain_kinematics import ArmModel  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import JointAngles  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import JOINT_LIMITS  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import forward_kinematics  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import inverse_kinematics  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import joint_limit_violations  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import joint_positions_from_message  # noqa: E402
from motionbrain_ros_bridge.motionbrain_kinematics import tool_orientation_quaternion
from motionbrain_ros_bridge.motionbrain_kinematics import (  # noqa: E402
    validate_complete_finite_joint_positions,
)
from motionbrain_ros_bridge.arm_geometry import load_geometry_profile
from motionbrain_ros_bridge.arm_geometry_urdf import URDF_PATHS, render_urdf, geometry_mismatches
from motionbrain_ros_bridge.rviz_appearance import rviz_description


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
        self.assertAlmostEqual(pose.y_m, .31)

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
        for target, pitch in [((.28, 0, .07), 0), ((.27, .04, .10), .2), ((.28, -.02, .04), -.1)]:
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
        solution = inverse_kinematics(0.28, 0.0, 0.07)

        self.assertTrue(solution.reachable)
        self.assertEqual(solution.reason, "ok")
        pose = forward_kinematics(solution.joint_angles)
        self.assertAlmostEqual(pose.x_m, 0.28, places=2)
        self.assertAlmostEqual(pose.z_m, 0.07, places=2)

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


class GeometryDerivativesTest(unittest.TestCase):
    def test_folded_elbow_display_range_does_not_expand_control_limits(self):
        display = ET.parse(ROOT / 'ros2_ws/src/motionbrain_description/urdf/motionbrain.urdf')
        limit = display.find("joint[@name='elbow_pitch_joint']/limit")
        self.assertLess(float(limit.get('lower')), math.radians(-109))
        self.assertAlmostEqual(float(limit.get('lower')), -math.radians(150), places=10)
        self.assertAlmostEqual(float(limit.get('upper')), math.radians(150), places=10)
        for relative in URDF_PATHS[1:3]:
            control = ET.parse(ROOT / 'ros2_ws/src' / relative)
            limit = control.find("joint[@name='elbow_pitch_joint']/limit")
            self.assertAlmostEqual(float(limit.get('lower')), -1.5708)
            self.assertAlmostEqual(float(limit.get('upper')), 1.5708)
        self.assertAlmostEqual(JOINT_LIMITS['elbow_pitch_joint'].lower, -math.pi / 2)

    def test_repository_geometry_matches_one_packaged_profile(self):
        profile = load_geometry_profile()
        for relative in URDF_PATHS:
            path = ROOT / 'ros2_ws/src' / relative
            self.assertEqual([], geometry_mismatches(path.read_text(), profile), relative)

    def test_different_geometry_matches_fk_ik_all_urdfs_and_preserves_control(self):
        profile = copy.deepcopy(load_geometry_profile())
        # Synthetic software fixture, explicitly not physical measurements.
        values = dict(base_height_m=.065, shoulder_offset_m=0., upper_arm_m=.12,
                      forearm_m=.10, wrist_m=.06, tool_m=.035)
        for key, value in values.items():
            profile['parameters'][key]['value_m'] = value
            profile['parameters'][key].update(status='nominal', source='synthetic_software_fixture',
                method='not_measured', uncertainty_m=None, evidence_sha256=None, physical_revision=None)
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            source = folder / 'profile.json'
            source.write_text(json.dumps(profile))
            model = ArmModel.from_profile(source)
            full_paths = []
            for relative in URDF_PATHS:
                path = ROOT / 'ros2_ws/src' / relative
                original = path.read_text()
                rendered = render_urdf(original, profile)
                if '<ros2_control' in original:
                    self.assertEqual(original[original.index('<ros2_control'):],
                                     rendered[rendered.index('<ros2_control'):])
                candidate = folder / path.name
                candidate.write_text(rendered)
                self.assertEqual([], geometry_mismatches(rendered, profile))
                if 'base_yaw_joint' in rendered:
                    full_paths.append(candidate)
                else:
                    actual = xml_transform(candidate, {'shoulder_pitch_joint': .4}, 'upper_arm_link')
                    np.testing.assert_allclose(actual[:3, 3], [0, 0, .065], atol=1e-12)
            for values in np.random.default_rng(18).uniform(-.6, .6, (12, 5)):
                angles = JointAngles(*values)
                pose = forward_kinematics(angles, model)
                positions = dict(zip(['base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint',
                                      'wrist_pitch_joint', 'gripper_joint'], values))
                for path in full_paths:
                    np.testing.assert_allclose(xml_transform(path, positions)[:3, 3],
                                               [pose.x_m, pose.y_m, pose.z_m], atol=1e-11)
            target = (.26, .025, .08)
            solution = inverse_kinematics(*target, model=model, target_tool_pitch_rad=.1)
            self.assertTrue(solution.reachable)
            pose = forward_kinematics(solution.joint_angles, model)
            np.testing.assert_allclose([pose.x_m, pose.y_m, pose.z_m], target, atol=1e-11)
            # Detect a stale derivative and a stale schematic visual separately.
            display = ET.fromstring(FULL_ARM_URDFS[0].read_text())
            display.find("link[@name='upper_arm_link']/visual/geometry/box").set('size', '.10 .006 .015')
            for stale in [original.replace('0 0 0.07', '0 0 0.08'),
                          ET.tostring(display, encoding='unicode')]:
                self.assertTrue(geometry_mismatches(stale, load_geometry_profile()))

    def test_unmodelled_transforms_and_axes_are_not_silently_discarded(self):
        source = FULL_ARM_URDFS[0].read_text()
        changed_templates = [source.replace('axis xyz="0 -1 0"', 'axis xyz="0 1 0"')]
        for attr, value in [('xyz', '0 .01 0'), ('rpy', '0 0 .1')]:
            tree = ET.fromstring(source)
            tree.find("joint[@name='shoulder_pitch_joint']/origin").set(attr, value)
            changed_templates.append(ET.tostring(tree, encoding='unicode'))
        for changed in changed_templates:
            with self.assertRaises(ValueError):
                render_urdf(changed, load_geometry_profile())

    def test_visual_updates_preserve_preceding_inertia_and_collision(self):
        source = FULL_ARM_URDFS[0].read_text()
        physical = '''<inertial><origin xyz="0.01 0.02 0.03"/><mass value="0.2"/>
        <inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial>
        <collision><origin xyz="0.03 0.02 0.01"/>
        <geometry><box size="0.23 .04 .04"/></geometry></collision>'''
        source = source.replace('<link name="upper_arm_link">', '<link name="upper_arm_link">' + physical)
        profile = load_geometry_profile()
        profile['parameters']['upper_arm_m']['value_m'] = .12
        rendered = render_urdf(source, profile)
        self.assertIn(physical, rendered)
        visual = ET.fromstring(rendered).find("link[@name='upper_arm_link']/visual")
        self.assertEqual(float(visual.find('origin').get('xyz').split()[0]), .06)
        self.assertEqual(float(visual.find('geometry/box').get('size').split()[0]), .12)

    def test_missing_extra_recolored_or_rotated_visuals_are_detected(self):
        source = FULL_ARM_URDFS[0].read_text()
        for kind in ('missing', 'extra', 'color', 'rotation'):
            tree = ET.fromstring(source)
            link = tree.find("link[@name='upper_arm_link']")
            visual = link.find('visual')
            if kind == 'missing':
                link.remove(visual)
            elif kind == 'extra':
                extra = copy.deepcopy(visual);extra.set('name', 'unexpected_extra')
                link.append(extra)
            elif kind == 'color':
                visual.find('material/color').set('rgba', '0 0 1 1')
            else:
                visual.find('origin').set('rpy', '0 .2 0')
            self.assertTrue(geometry_mismatches(ET.tostring(tree, encoding='unicode'), load_geometry_profile()), kind)

    def test_display_jaws_mirror_and_tool_frame_stays_fixed(self):
        source = FULL_ARM_URDFS[0]
        tree = ET.parse(source).getroot()
        follower = tree.find("joint[@name='gripper_opposing_joint']/mimic")
        self.assertEqual(follower.attrib, dict(joint='gripper_joint', multiplier='-1', offset='0'))
        for q in (0, .15, .3):
            positions = {'gripper_joint': q, 'gripper_opposing_joint': -q}
            left = xml_transform(source, positions, 'gripper_jaw_link')
            right = xml_transform(source, positions, 'gripper_opposing_jaw_link')
            # Mirror symmetry in the local gripper plane, not aperture calibration.
            a = left @ np.array([.055, 0, 0, 1]);b = right @ np.array([.055, 0, 0, 1])
            np.testing.assert_allclose(a[[0,2]], b[[0,2]], atol=1e-12)
            self.assertAlmostEqual(a[1], -b[1])
            np.testing.assert_allclose(xml_transform(source, positions), xml_transform(source, {}), atol=1e-12)

    def test_opposing_jaw_limits_reverse_asymmetric_master_limits(self):
        source = ET.parse(FULL_ARM_URDFS[0]).getroot()
        source.find("joint[@name='gripper_joint']/limit").set('lower','0')
        source.find("joint[@name='gripper_joint']/limit").set('upper','.5')
        rendered = ET.fromstring(render_urdf(ET.tostring(source,encoding='unicode'),load_geometry_profile()))
        limit = rendered.find("joint[@name='gripper_opposing_joint']/limit")
        self.assertEqual(float(limit.get('lower')), -.5)
        self.assertEqual(float(limit.get('upper')), 0.)

    def test_rviz_color_groups_preserve_original_frames_and_every_visual(self):
        source = FULL_ARM_URDFS[0]
        original = ET.parse(source).getroot()
        converted = rviz_description(source.read_text())
        tree = ET.fromstring(converted)
        self.assertEqual(len(original.findall('.//visual')), len(tree.findall('.//visual')))
        def signature(node):
            return node.tag, node.attrib, [signature(child) for child in node]
        for joint in original.findall('joint'):
            self.assertEqual(signature(joint), signature(tree.find(f"joint[@name='{joint.get('name')}']")))
        for link in tree.findall('link'):
            colors = {v.find('material/color').get('rgba') for v in link.findall('visual')}
            self.assertLessEqual(len(colors), 1)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'rviz.urdf';path.write_text(converted)
            positions=dict(shoulder_pitch_joint=.4,elbow_pitch_joint=-.3,wrist_pitch_joint=.2,
                           gripper_joint=.15,gripper_opposing_joint=-.15)
            np.testing.assert_allclose(xml_transform(path,positions),xml_transform(source,positions),atol=1e-12)
            for joint in tree.findall('joint'):
                if not joint.get('name').startswith('rviz_visual_'):
                    continue
                parent=joint.find('parent').get('link');child=joint.find('child').get('link')
                np.testing.assert_allclose(xml_transform(path,positions,child),xml_transform(source,positions,parent),atol=1e-12)

    def test_self_closing_empty_camera_does_not_consume_opposing_jaw(self):
        source = ET.parse(FULL_ARM_URDFS[0]).getroot()
        source.find("link[@name='camera_link']").text = None
        # Standard XML serializers compact empty links into self-closing tags.
        compact = ET.tostring(source,encoding='unicode',short_empty_elements=True)
        self.assertIn('<link name="camera_link" />',compact)
        rendered = render_urdf(compact,load_geometry_profile())
        tree = ET.fromstring(rendered)
        self.assertEqual(len(tree.findall('.//visual')),119)
        self.assertEqual(len(tree.findall("link[@name='gripper_opposing_jaw_link']/visual")),10)
        self.assertEqual(geometry_mismatches(rendered,load_geometry_profile()),[])


if __name__ == "__main__":
    unittest.main()
