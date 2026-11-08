"""후보 파일·실측 근거 해시의 정합성과 기존 출력 보호를 검사한다."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('calibrate_arm_geometry_cli', ROOT / 'tools/calibrate_arm_geometry.py')
cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cli)


class CalibrationBundleTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()
        original_root = cli.ROOT
        self.addCleanup(setattr, cli, 'ROOT', original_root)
        cli.ROOT = self.root
        for relative in cli.URDF_PATHS:
            source = ROOT / 'ros2_ws/src' / relative
            target = self.root / 'ros2_ws/src' / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read_bytes())

    def call(self, args):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return cli.main(args)

    def test_all_derivatives_and_profile_are_hash_bound_without_control_eligibility(self):
        output = self.root / '.local/candidate'
        self.assertEqual(self.call(['--output', str(output)]), 0)
        manifest = json.loads((output / 'geometry-manifest.json').read_text())
        self.assertEqual(len(manifest['generatedSha256']), 8)
        for name, digest in manifest['generatedSha256'].items():
            self.assertEqual(hashlib.sha256((output / name).read_bytes()).hexdigest(), digest)
        for key in ['physicallyCalibrated', 'feedbackCalibrated', 'dynamicsCalibrated',
                    'contactCalibrated', 'controlEligible']:
            self.assertIs(manifest[key], False)
        self.assertEqual(manifest['geometryStatus'], 'nominal')
        self.assertEqual(self.call(['--check']), 0)

    def test_measured_source_must_match_actual_evidence_file_before_output(self):
        from motionbrain_ros_bridge.arm_geometry import load_geometry_profile
        profile = load_geometry_profile()
        raw = self.root / 'raw.json'
        raw.write_text('{"fixture_only": true, "upper_arm_mm": 120}')
        profile['parameters']['upper_arm_m'].update(value_m=.12, status='measured',
            source='synthetic_fixture', method='ruler', uncertainty_m=.002,
            evidence_sha256=hashlib.sha256(raw.read_bytes()).hexdigest(), physical_revision='test')
        # Keep this fixture independent of measurements in the shipped profile.
        for name, field in profile['parameters'].items():
            if name != 'upper_arm_m':
                field.update(status='nominal', source='synthetic_unmeasured', method='not_measured',
                             uncertainty_m=None, evidence_sha256=None, physical_revision=None)
        path = self.root / 'profile.json'
        path.write_text(json.dumps(profile))
        output = self.root / '.local/candidate'
        args = ['--profile', str(path), '--output', str(output)]
        with self.assertRaises(SystemExit):
            self.call(args)
        self.assertFalse(output.exists())
        raw.write_text('changed')
        with self.assertRaises(SystemExit):
            self.call(args + ['--evidence', str(raw)])
        self.assertFalse(output.exists())
        raw.write_text('{"fixture_only": true, "upper_arm_mm": 120}')
        self.assertEqual(self.call(args + ['--evidence', str(raw)]), 0)
        # Partial measurements must not silently promote untouched defaults.
        manifest = json.loads((output / 'geometry-manifest.json').read_text())
        self.assertEqual(manifest['geometryStatus'], 'nominal')

    def test_existing_or_public_output_is_not_overwritten(self):
        output = self.root / '.local/candidate'
        output.mkdir(parents=True)
        (output / 'sentinel').write_text('keep')
        with self.assertRaises(FileExistsError):
            self.call(['--output', str(output)])
        self.assertEqual((output / 'sentinel').read_text(), 'keep')
        with self.assertRaises(SystemExit):
            self.call(['--output', str(self.root / 'public')])
        self.assertFalse((self.root / 'public').exists())
