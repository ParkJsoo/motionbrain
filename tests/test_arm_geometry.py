"""기하 프로필의 실패 경계와 설치 후 resource 사용을 검증한다."""

import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ROOT / "ros2_ws/src/motionbrain_ros_bridge"
sys.path.insert(0, str(PACKAGE_ROOT))

from motionbrain_ros_bridge.arm_geometry import (  # noqa: E402
    GEOMETRY_FIELDS, geometry_values, load_geometry_profile, profile_digest,
)


class ArmGeometryTest(unittest.TestCase):
    def setUp(self):
        self.profile = load_geometry_profile()

    def test_default_separates_reported_measurement_from_remaining_legacy_values(self):
        self.assertEqual(geometry_values(self.profile), dict(zip(
            GEOMETRY_FIELDS, [.07, 0, .09, .11, .045, .065])))
        self.assertEqual(self.profile["status"], "nominal")
        for name, field in self.profile["parameters"].items():
            if name in ('base_height_m', 'upper_arm_m', 'forearm_m'):
                self.assertEqual(field['status'], 'measured')
                self.assertIsNone(field['uncertainty_m'])
                self.assertEqual(len(field['evidence_sha256']), 64)
                continue
            if name == 'shoulder_offset_m':
                self.assertEqual(field['status'], 'nominal')
                self.assertEqual(field['method'], 'unscaled_assembly_diagram_intersection_assumption')
                self.assertIsNone(field['uncertainty_m'])
                continue
            self.assertEqual(field["status"], "nominal")
            self.assertIn('nominal', field['method'])
            self.assertIsNone(field["uncertainty_m"])
            self.assertEqual(len(field['evidence_sha256']), 64)

    def test_confirmed_distal_total_does_not_double_count_intermediate_segment(self):
        values = geometry_values(self.profile)
        self.assertAlmostEqual(values['wrist_m'] + values['tool_m'], .110)
        self.assertEqual(self.profile['parameters']['wrist_m']['status'], 'nominal')
        self.assertEqual(self.profile['parameters']['tool_m']['status'], 'nominal')

    def test_zero_radial_shoulder_offset_is_supported(self):
        self.profile["parameters"]["shoulder_offset_m"]["value_m"] = 0
        self.assertEqual(geometry_values(self.profile)["shoulder_offset_m"], 0)

    def test_invalid_values_are_rejected_by_all_consumers(self):
        for value in [True, False, "0.1", None, -0.001, 11, float("nan"),
                      float("inf"), float("-inf"), 10**500]:
            for consumer in [geometry_values, profile_digest]:
                with self.subTest(value=repr(value), consumer=consumer.__name__):
                    p = copy.deepcopy(self.profile)
                    p["parameters"]["upper_arm_m"]["value_m"] = value
                    with self.assertRaises(ValueError):
                        consumer(p)
        self.profile["parameters"]["upper_arm_m"]["value_m"] = 0
        with self.assertRaises(ValueError):
            geometry_values(self.profile)

    def test_unknown_schema_units_fields_and_transforms_are_rejected(self):
        cases = [
            lambda p: p.update(schema="future"),
            lambda p: p.update(units="millimetres"),
            lambda p: p.update(model="general_xyz"),
            lambda p: p.update(extra=1),
            lambda p: p["parameters"].pop("tool_m"),
            lambda p: p["parameters"]["wrist_m"].update(extra=1),
            lambda p: p["convention"].update(pitch_axis=[0, 1, 0]),
            lambda p: p["convention"].update(joint_origin_rpy=[0, .01, 0]),
            lambda p: p["convention"].update(yaw_axis=[False, 0, 1]),
            lambda p: p["convention"].update(jaw_motion_affects_tool=0),
            lambda p: p["convention"].update(tool_reference="moving_jaw"),
        ]
        for index, mutate in enumerate(cases):
            with self.subTest(case=index):
                p = copy.deepcopy(self.profile)
                mutate(p)
                with self.assertRaises(ValueError):
                    geometry_values(p)

    def measured(self, name, revision="test_fixture_revision"):
        self.profile["parameters"][name].update(
            status="measured", source="synthetic_test_evidence", method="axis_center_ruler",
            uncertainty_m=.001, evidence_sha256="a" * 64, physical_revision=revision)

    def fixture_revision(self):
        for name in GEOMETRY_FIELDS:
            if self.profile['parameters'][name]['status'] == 'measured':
                self.measured(name)

    def test_partial_measurement_keeps_profile_nominal(self):
        self.fixture_revision()
        self.measured("upper_arm_m")
        self.measured("forearm_m")
        self.assertEqual(geometry_values(self.profile)["upper_arm_m"], .09)
        self.assertEqual(self.profile["parameters"]["wrist_m"]["status"], "nominal")
        self.assertEqual(self.profile["parameters"]["tool_m"]["status"], "nominal")
        self.profile["status"] = "measured"
        with self.assertRaisesRegex(ValueError, "geometry_profile_status_mismatch"):
            geometry_values(self.profile)

    def test_measured_requires_evidence_method_and_revision_but_allows_unknown_uncertainty(self):
        self.fixture_revision()
        self.measured("upper_arm_m")
        self.measured("forearm_m")
        valid = copy.deepcopy(self.profile)
        self.profile['parameters']['upper_arm_m']['uncertainty_m'] = None
        geometry_values(self.profile)
        for key, value in [("uncertainty_m", True),
                           ("uncertainty_m", -.001), ("evidence_sha256", None),
                           ("evidence_sha256", "not-a-hash"), ("physical_revision", None),
                           ("physical_revision", ""), ("method", "not_measured"),
                           ("source", "unknown")]:
            with self.subTest(key=key, value=value):
                self.profile = copy.deepcopy(valid)
                self.profile["parameters"]["upper_arm_m"][key] = value
                with self.assertRaises(ValueError):
                    geometry_values(self.profile)

    def test_measured_fields_must_share_one_physical_revision(self):
        self.measured("upper_arm_m", "revision_a")
        self.measured("forearm_m", "revision_b")
        with self.assertRaisesRegex(ValueError, "geometry_physical_revision_mismatch"):
            geometry_values(self.profile)

    def test_all_measured_is_supported_but_validated_is_not(self):
        for name in GEOMETRY_FIELDS:
            self.measured(name)
        self.profile["status"] = "measured"
        geometry_values(self.profile)
        self.profile["parameters"]["upper_arm_m"]["status"] = "validated"
        with self.assertRaisesRegex(ValueError, "unsupported_geometry_status"):
            geometry_values(self.profile)

    def test_digest_is_order_independent_and_includes_evidence_and_status(self):
        original = copy.deepcopy(self.profile)
        digest = profile_digest(original)
        reordered = json.loads(json.dumps(original, sort_keys=True))
        self.assertEqual(digest, profile_digest(reordered))
        self.fixture_revision()
        self.measured("upper_arm_m")
        self.measured("forearm_m")
        self.assertNotEqual(digest, profile_digest(self.profile))
        canonical = json.dumps(original, sort_keys=True, separators=(",", ":"),
                               ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.assertEqual(digest, hashlib.sha256(canonical).hexdigest())

    def test_explicit_file_never_falls_back_for_missing_or_invalid_data(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.json"
            with self.assertRaises(FileNotFoundError):
                load_geometry_profile(path)
            for raw in ['{"schema":1,"schema":2}', '{"value":NaN}', '[]', '{']:
                path.write_text(raw)
                with self.subTest(raw=raw), self.assertRaises(ValueError):
                    load_geometry_profile(path)
            path.write_text(json.dumps(self.profile))
            self.assertEqual(load_geometry_profile(path), self.profile)

    @unittest.skipUnless(importlib.util.find_spec("setuptools"), "setuptools build runtime not installed")
    def test_built_package_resource_works_without_source_checkout(self):
        with tempfile.TemporaryDirectory() as tmp:
            site = Path(tmp) / "site"
            built = subprocess.run(
                [sys.executable, "setup.py", "build_py", "--build-lib", str(site)],
                cwd=PACKAGE_ROOT, capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
            self.assertTrue((site / "motionbrain_ros_bridge/arm_geometry.json").is_file())
            env = dict(os.environ, PYTHONPATH=str(site))
            loaded = subprocess.run(
                [sys.executable, "-c", "from motionbrain_ros_bridge.arm_geometry import "
                 "load_geometry_profile,geometry_values; "
                 "assert geometry_values(load_geometry_profile())['upper_arm_m']==.09; "
                 "from motionbrain_ros_bridge.motionbrain_kinematics import ArmModel; "
                 "assert ArmModel().upper_arm_m==.09"],
                cwd=tmp, env=env, capture_output=True, text=True)
            self.assertEqual(loaded.returncode, 0, loaded.stdout + loaded.stderr)


if __name__ == "__main__":
    unittest.main()
