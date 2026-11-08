"""외형은 FK와 물리 보정에서 분리되며 두 턱은 같은 local +X를 쓴다."""
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/motionbrain_ros_bridge"))
from motionbrain_ros_bridge.arm_appearance import build_visuals  # noqa: E402


class ArmAppearanceTest(unittest.TestCase):
    def setUp(self):
        self.values = dict(base_height_m=.07, shoulder_offset_m=0, upper_arm_m=.09,
                           forearm_m=.11, wrist_m=.045, tool_m=.065)
        self.visuals = build_visuals(self.values)

    def test_visuals_are_deterministic_named_colored_and_physics_free(self):
        other = build_visuals(self.values)
        self.assertEqual(list(self.visuals), list(other))
        names = []
        for link, elements in self.visuals.items():
            self.assertEqual([ET.tostring(v) for v in elements], [ET.tostring(v) for v in other[link]])
            for visual in elements:
                self.assertEqual(visual.tag, "visual")
                names.append(visual.get("name"))
                self.assertTrue(visual.get("name").startswith("owi_"))
                self.assertIsNotNone(visual.find("material/color"))
                self.assertIsNone(visual.find("inertial"))
                self.assertIsNone(visual.find("collision"))
                self.assertIsNone(visual.find("joint"))
                shape = list(visual.find("geometry"))[0]
                self.assertIn(shape.tag, ["box", "cylinder", "sphere"])
                for raw in shape.attrib.values():
                    self.assertTrue(all(math.isfinite(float(v)) and float(v) > 0 for v in raw.split()))
        self.assertEqual(len(names), len(set(names)))
        self.assertGreater(len(names), 100)

    def test_legacy_camera_has_no_floating_visual_and_zero_offset_has_no_sliver(self):
        self.assertEqual(self.visuals["camera_link"], [])
        self.assertFalse(any("shoulder_connector" in v.get("name") for v in self.visuals["shoulder_link"]))
        self.assertTrue(any(v.get("name") == "owi_camera_board" for v in self.visuals["gripper_link"]))

    def test_both_finger_tips_reach_tool_reference_in_joint_local_x(self):
        for name in ["gripper_jaw_link", "gripper_opposing_jaw_link"]:
            finger = next(v for v in self.visuals[name] if v.get("name").endswith("_finger"))
            center = float(finger.find("origin").get("xyz").split()[0])
            length = float(finger.find("geometry/box").get("size").split()[0])
            self.assertAlmostEqual(center - length / 2, 0)
            self.assertAlmostEqual(.01 + center + length / 2, self.values["tool_m"])
        left = next(v for v in self.visuals["gripper_jaw_link"] if v.get("name").endswith("_inner_pad"))
        right = next(v for v in self.visuals["gripper_opposing_jaw_link"] if v.get("name").endswith("_inner_pad"))
        self.assertLess(float(left.find("origin").get("xyz").split()[1]), 0)
        self.assertGreater(float(right.find("origin").get("xyz").split()[1]), 0)

    def test_m2_motor_case_is_behind_external_wrist_hinge(self):
        cap = next(v for v in self.visuals["forearm_link"] if v.get("name") == "owi_m2_motor_near_cap")
        x = float(cap.find("origin").get("xyz").split()[0])
        self.assertAlmostEqual(x, .07)
        self.assertLess(x, self.values["forearm_m"])

    def test_link_visual_lengths_follow_skeleton_without_mutating_input(self):
        changed = dict(self.values, upper_arm_m=.13, shoulder_offset_m=.02)
        visuals = build_visuals(changed)
        rail = next(v for v in visuals["upper_arm_link"] if v.get("name") == "owi_upper_rail_near")
        self.assertAlmostEqual(float(rail.find("geometry/box").get("size").split()[0]), .13)
        self.assertTrue(any("shoulder_connector" in v.get("name") for v in visuals["shoulder_link"]))
        self.assertEqual(self.values["upper_arm_m"], .09)

    def test_bad_dimensions_fail_and_degenerate_fixed_lengths_do_not_make_zero_shapes(self):
        for bad in [True, -1, float("nan"), float("inf"), 10**500]:
            with self.subTest(value=bad), self.assertRaises(ValueError):
                build_visuals(dict(self.values, forearm_m=bad))
        visuals = build_visuals(dict(self.values, wrist_m=0, tool_m=0, base_height_m=0))
        self.assertEqual(visuals["gripper_jaw_link"], [])
        self.assertEqual(visuals["gripper_opposing_jaw_link"], [])
        for elements in visuals.values():
            for visual in elements:
                for value in list(visual.find("geometry"))[0].attrib.values():
                    self.assertTrue(all(float(v) > 0 for v in value.split()))


if __name__ == "__main__":
    unittest.main()
