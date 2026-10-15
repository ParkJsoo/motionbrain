"""Offline fixture tests for the exact harness used on Pi; no device/network calls."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tests'))
sys.path.insert(0, str(ROOT))
import test_fast_observation as fixtures
import test_fresh_source_age as fresh_fixtures
from learning.artifacts import ArtifactStore

spec = importlib.util.spec_from_file_location('fault_probe', Path(__file__).with_name('fault_probe.py'))
fault = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fault)


class FaultHarnessTest(unittest.TestCase):
    setUp = fixtures.FastProbeTest.setUp

    def case(self, mode):
        fresh_fixtures.FreshSourceAgeProbeTest.configure(self)
        report, audit = fault.run_case(self.root, mode, self.transport, sleep=self.transport.clock.advance)
        summary = fault.verify_case(self.root, report, audit)
        self.assertEqual(self.capture_count, 1)
        self.assertEqual(summary['captureAttemptsAfterInjection'], 0)
        self.assertTrue(summary['consumed'])
        return report, audit, summary

    def test_delay_stops_and_preserves_original(self):
        _, _, s = self.case('image_delay')
        self.assertEqual(s['storageStatus'], 'late')
        self.assertGreater(s['persistenceElapsedMs'], 250)
        self.assertTrue(s['imagePreserved'] and s['receiptPreserved'])

    def test_image_failure_is_unknown_with_missing_image(self):
        _, _, s = self.case('image_failure')
        self.assertEqual(s['storageStatus'], 'unconfirmed')
        self.assertFalse(s['imagePreserved'])
        self.assertTrue(s['receiptPreserved'])

    def test_receipt_failure_joins_and_preserves_image(self):
        _, _, s = self.case('receipt_failure')
        self.assertEqual(s['storageStatus'], 'unconfirmed')
        self.assertTrue(s['imagePreserved'])
        self.assertFalse(s['receiptPreserved'])

    def test_unrelated_store_and_anchor_are_not_injected(self):
        store = fault.FaultStore(self.root, 'image_failure')
        digest = __import__('hashlib').sha256(self.body).hexdigest()
        store.save_image('anchor', 'source', self.body, digest)
        store.write_json('run.json', {'status':'fixture'})
        other = ArtifactStore(self.root.parent/'other')
        other.save_image('0', 'source', self.body, digest)
        self.assertEqual(store.injections, [])

    def test_extra_capture_or_missing_fault_never_verifies(self):
        report, audit, _ = self.case('image_delay')
        changed = copy.deepcopy(audit)
        changed['calls'].append(copy.deepcopy(next(c for c in audit['calls'] if c['path']=='/capture')))
        with self.assertRaises(AssertionError): fault.verify_case(self.root, report, changed)
        changed = {**audit, 'injections': []}
        with self.assertRaises(AssertionError): fault.verify_case(self.root, report, changed)

    def test_corrupted_preserved_image_is_rejected(self):
        report, audit, _ = self.case('receipt_failure')
        (self.root/'images/0/source.jpg').write_bytes(b'corrupt')
        with self.assertRaises(AssertionError): fault.verify_case(self.root, report, audit)

    def test_unexpected_outcome_stops_the_scenario_sequence(self):
        state = {'unchanged': True}
        with patch.object(fault, 'snapshot', return_value=state), patch.object(fault, 'make_http'), \
             patch.object(fault, 'run_case', return_value=({}, {})) as run, \
             patch.object(fault, 'verify_case', side_effect=AssertionError('unexpected')):
            with self.assertRaises(AssertionError): fault.main(self.root.parent)
        self.assertEqual(run.call_count, 1)
        self.assertFalse((self.root.parent/'fault-evidence/summary.json').exists())


if __name__ == '__main__':
    unittest.main()
