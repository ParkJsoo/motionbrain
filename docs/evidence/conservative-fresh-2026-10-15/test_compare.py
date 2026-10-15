"""Orchestrator tests: no network/device calls; fake subprocess results only."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('live_compare', Path(__file__).with_name('compare.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class CompareTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.state = {'camera': {'freshCaptureSupported': True}, 'outputs': {},
                      'controller': {'lastCommand': dict(id=9, type='disarm', source='web', executedAtMs=1)}}
        self.report = dict(runDir='/unused', status='completed', error=None, completedSamples=12,
                           acceptedMeasurements=4, consumedObservations=4, consumptionHalted=False,
                           outputLogUnchanged=True, motorCommandsSent=0)

    def invoke(self, process):
        with patch.object(runner, 'snapshot', return_value=self.state), patch.object(runner.time, 'sleep'), \
             patch.object(runner.subprocess, 'run', process):
            return runner.compare(self.root, object())

    def test_completed_sequence_and_fixed_gates(self):
        with patch.object(runner.subprocess, 'run') as process:
            process.return_value = subprocess.CompletedProcess([], 0, json.dumps(self.report), '')
            self.assertTrue(self.invoke(process))
            self.assertEqual(process.call_count, 4)
            for index, call in enumerate(process.call_args_list):
                cmd = call.args[0]
                self.assertIn('--shadow-consumption', cmd)
                self.assertEqual(cmd[cmd.index('--max-persistence-ms')+1], '250')
                self.assertIn('--fresh-capture',cmd)
                self.assertEqual(cmd[cmd.index('--fresh-age-mode')+1],runner.PLAN[index])

    def test_failed_or_halted_run_never_starts_another(self):
        for status, halted, code in [('failed', True, 2), ('completed', True, 0), ('completed', False, 2)]:
            with self.subTest(status=status, halted=halted, code=code), patch.object(runner.subprocess, 'run') as process:
                report = {**self.report, 'status': status, 'consumptionHalted': halted,
                          'error': 'persistence_deadline_or_consumption_evidence'}
                process.return_value = subprocess.CompletedProcess([], code, json.dumps(report), '')
                self.assertFalse(self.invoke(process))
                self.assertEqual(process.call_count, 1)
                self.assertEqual(json.loads((self.root/'comparison-stop.json').read_text())['remainingRunsSkipped'], 3)

    def test_missing_terminal_output_never_retries(self):
        with patch.object(runner.subprocess, 'run') as process:
            process.return_value = subprocess.CompletedProcess([], 2, '', 'disk error')
            with self.assertRaises(json.JSONDecodeError): self.invoke(process)
            self.assertEqual(process.call_count, 1)
            self.assertTrue((self.root/'comparison-after.json').exists())

    def test_timeout_never_retries(self):
        with patch.object(runner.subprocess, 'run') as process:
            process.side_effect = subprocess.TimeoutExpired(['probe'], 180)
            with self.assertRaises(subprocess.TimeoutExpired): self.invoke(process)
            self.assertEqual(process.call_count, 1)
            self.assertTrue((self.root/'comparison-after.json').exists())


if __name__ == '__main__':
    unittest.main()
