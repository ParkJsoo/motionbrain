import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from learning import artifacts
from learning.artifacts import StorageError
from learning.hardware_demo import archive_demo, audit_demo, POLICY


class HardwareDemoTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.source = self.root / 'source'
        self.step = self.source / 'step-01'
        (self.step / 'execution').mkdir(parents=True)
        (self.step / 'plan.json').write_text('{"joint":"gripper"}')
        self.result = self.step / 'execution/result.json'
        self.result.write_text('{"failure":null,"finalIdleVerified":true,"trainingEligible":true}')
        (self.step / 'after.png').write_bytes(b'\x89PNG\r\n\x1a\nfixture')
        (self.source / '.control-token').write_text('must-never-be-copied')
        (self.step / 'run_once.py').write_text('must-never-be-copied')
        self.dest = self.root / 'archive'

    def test_raw_exact_bytes_remain_ineligible_and_credentials_excluded(self):
        manifest = archive_demo(self.source, self.dest, observation='placed')
        self.assertEqual([], audit_demo(self.dest))
        self.assertFalse(manifest['trainingEligible'])
        self.assertFalse(manifest['attempts'][0]['trainingEligible'])
        self.assertEqual(POLICY, manifest['qualityPolicy'])
        self.assertEqual(self.result.read_bytes(), (self.dest / 'step-01/execution/result.json').read_bytes())
        self.assertFalse((self.dest / '.control-token').exists())
        self.assertFalse((self.dest / 'step-01/run_once.py').exists())
        self.assertEqual('placed', manifest['observation']['label'])
        with self.assertRaises(StorageError):
            archive_demo(self.source, self.dest)

    def test_failure_and_missing_tail_are_preserved_without_success_inference(self):
        self.result.write_text('{"failure":"timeout","finalIdleVerified":false}')
        (self.source / 'step-03').mkdir()
        manifest = archive_demo(self.source, self.dest)
        self.assertEqual(['step_number_gap'], manifest['sourceIssues'])
        self.assertEqual('timeout', manifest['attempts'][0]['reportedFailure'])
        self.assertFalse(manifest['attempts'][1]['executionRecordPresent'])
        self.assertIn('execution/result.json', manifest['attempts'][1]['missing'])
        self.assertEqual('unknown', manifest['observation']['label'])

    def test_symlink_source_rejected_without_terminal_manifest(self):
        self.result.unlink()
        self.result.symlink_to(self.source / '.control-token')
        with self.assertRaises(StorageError):
            archive_demo(self.source, self.dest)
        self.assertFalse((self.dest / 'demo.json').exists())

    def test_credentials_and_duplicate_keys_rejected(self):
        for text in ('{"headers":{"X-MotionBrain-Token":"secret"}}', '{"x":1,"x":2}'):
            with self.subTest(text=text):
                self.result.write_text(text)
                with self.assertRaises((StorageError, ValueError)):
                    archive_demo(self.source, self.root / str(len(text)))

    def test_hash_tampering_and_eligibility_tampering_detected(self):
        archive_demo(self.source, self.dest)
        (self.dest / 'step-01/after.png').write_bytes(b'changed')
        self.assertTrue(audit_demo(self.dest))
        path = self.dest / 'demo.json'
        data = json.loads(path.read_text())
        data['trainingEligible'] = True
        data['qualityPolicy']['maxSourceAgeMs'] = 101
        path.write_text(json.dumps(data))
        errors = audit_demo(self.dest)
        self.assertIn('invalid_trainingEligible', errors)
        self.assertIn('invalid_qualityPolicy', errors)

    def test_original_v1_png_inventory_remains_valid(self):
        archive_demo(self.source, self.dest)
        path = self.dest / 'demo.json'; data = json.loads(path.read_text())
        data['attempts'][0]['missing'].remove('after.jpg')
        path.write_text(json.dumps(data))
        self.assertEqual([], audit_demo(self.dest))

    def test_jpeg_observation_supported_without_transcoding(self):
        (self.step / 'after.png').unlink()
        raw = b'\xff\xd8fixture\xff\xd9'
        (self.step / 'after.jpg').write_bytes(raw)
        manifest = archive_demo(self.source, self.dest)
        self.assertTrue(manifest['sourceRecordsComplete'])
        self.assertEqual([], audit_demo(self.dest))
        self.assertEqual(raw, (self.dest / 'step-01/after.jpg').read_bytes())
        self.assertEqual(['step-01/after.jpg'], manifest['observation']['evidenceRefs'])

    def test_mislabeled_jpeg_rejected(self):
        (self.step / 'after.png').write_bytes(b'\xff\xd8fixture\xff\xd9')
        with self.assertRaises(StorageError):
            archive_demo(self.source, self.dest)
        self.assertFalse((self.dest / 'demo.json').exists())

    def test_final_readback_failure_removes_terminal(self):
        original = artifacts._read
        def read(root, relative):
            if relative == 'demo.json':
                raise StorageError('injected terminal readback failure')
            return original(root, relative)
        with patch('learning.artifacts._read', side_effect=read):
            with self.assertRaises(StorageError):
                archive_demo(self.source, self.dest)
        self.assertFalse((self.dest / 'demo.json').exists())

    def test_timing_and_completeness_claims_cannot_be_promoted(self):
        archive_demo(self.source, self.dest)
        path = self.dest / 'demo.json'; data = json.loads(path.read_text())
        data['timing']['sourceExposureTimeVerified'] = True
        data['qualityChecks']['sourceCapturePersistence'] = 'passed'
        data['sourceRecordsComplete'] = False
        path.write_text(json.dumps(data))
        errors = audit_demo(self.dest)
        for field in ('timing', 'qualityChecks', 'sourceRecordsComplete'):
            self.assertIn('invalid_' + field, errors)

    def test_removed_asset_descriptor_detected(self):
        archive_demo(self.source, self.dest)
        path = self.dest / 'demo.json'; data = json.loads(path.read_text())
        del data['attempts'][0]['artifacts']['after.png']
        path.write_text(json.dumps(data))
        self.assertIn('invalid_file_inventory', audit_demo(self.dest))

    def test_empty_source_and_bad_annotation_are_rejected(self):
        empty = self.root / 'empty'; empty.mkdir()
        with self.assertRaises(StorageError):
            archive_demo(empty, self.dest)
        with self.assertRaises(ValueError):
            archive_demo(self.source, self.dest, observation='training_success')


if __name__ == '__main__':
    unittest.main()
