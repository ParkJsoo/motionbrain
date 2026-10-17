"""Curl interruption evidence using mocked processes only; no network."""
import base64
import hashlib
import json
import subprocess
import unittest
from unittest.mock import Mock, patch

from learning.mac_controller_probe import read_controller, _check, ProbeRejected, audit_probe
from tests import test_mac_controller_probe as fixture


class CurlCancellationTest(unittest.TestCase):
    def process(self, outcomes, returncode=-15):
        process = Mock(returncode=returncode)
        process.communicate.side_effect = outcomes
        return process

    def read(self, process):
        with patch('learning.mac_controller_probe.subprocess.Popen', return_value=process) as spawn:
            row = read_controller('http://192.0.2.1', 'status')
        spawn.assert_called_once()
        return row

    def test_interrupt_retains_cleanup_observed_stdout_and_stderr(self):
        out, err = b'{"partial":', b'curl interrupted\xff'
        process = self.process([KeyboardInterrupt(), (out, err)])
        row = self.read(process)
        self.assertTrue(row['cancelled'])
        self.assertEqual(row['error'], 'cancelled')
        self.assertEqual(base64.b64decode(row['stdoutBase64']), out)
        self.assertEqual(base64.b64decode(row['stderrBase64']), err)
        self.assertEqual(row['bodySha256'], hashlib.sha256(out).hexdigest())
        self.assertEqual(row['stderrSha256'], hashlib.sha256(err).hexdigest())
        self.assertEqual(row['httpStatus'], '')
        process.terminate.assert_called_once()
        process.kill.assert_not_called()
        self.assertEqual([c.kwargs['timeout'] for c in process.communicate.call_args_list], [1.2, .2])
        process.stdout.close.assert_called_once()
        process.stderr.close.assert_called_once()

    def test_terminate_timeout_escalates_once_and_keeps_cumulative_output(self):
        partial = subprocess.TimeoutExpired('curl', .2, output=b'part', stderr=b'warning')
        process = self.process([KeyboardInterrupt(), partial, (b'partial', b'warning\n')], -9)
        row = self.read(process)
        self.assertEqual(base64.b64decode(row['stdoutBase64']), b'partial')
        self.assertEqual(base64.b64decode(row['stderrBase64']), b'warning\n')
        self.assertEqual(len(row['outputObservations']), 2)
        self.assertEqual(row['cleanupErrors'], ['TimeoutExpired'])
        self.assertEqual(process.communicate.call_count, 3)
        process.kill.assert_called_once()

    def test_initial_deadline_also_returns_a_failed_attempt_with_original_bytes(self):
        failure = subprocess.TimeoutExpired('curl', 1.2, output=b'prefix', stderr=b'failed')
        process = self.process([failure, (b'prefix-end', b'failed-end')])
        row = self.read(process)
        self.assertEqual(row['error'], 'TimeoutExpired')
        self.assertNotIn('cancelled', row)
        self.assertEqual(base64.b64decode(row['stdoutBase64']), b'prefix-end')
        with self.assertRaises(ProbeRejected): _check(row, 'status')

    def test_failed_final_cleanup_keeps_longest_observed_bytes_and_marks_incomplete(self):
        first = subprocess.TimeoutExpired('curl', 1.2, output=b'first', stderr=b'warn')
        second = subprocess.TimeoutExpired('curl', .2, output=b'first-more', stderr=b'warn-more')
        last = subprocess.TimeoutExpired('curl', .2, output=None, stderr=None)
        process = self.process([first, second, last], None)
        row = self.read(process)
        self.assertTrue(row['cleanupIncomplete'])
        self.assertEqual(base64.b64decode(row['stdoutBase64']), b'first-more')
        self.assertEqual(base64.b64decode(row['stderrBase64']), b'warn-more')
        self.assertEqual([c.kwargs['timeout'] for c in process.communicate.call_args_list], [1.2, .2, .2])

    def test_repeated_interrupts_during_cleanup_are_finite(self):
        process = self.process([KeyboardInterrupt(), KeyboardInterrupt(), KeyboardInterrupt()], None)
        row = self.read(process)
        self.assertTrue(row['cancelled'])
        self.assertTrue(row['cleanupIncomplete'])
        self.assertEqual(process.communicate.call_count, 3)
        process.terminate.assert_called_once()
        process.kill.assert_called_once()

    def test_cancelled_complete_body_cannot_masquerade_as_success(self):
        process = self.process([KeyboardInterrupt(), (b'{}\n200', b'')], 0)
        row = self.read(process)
        self.assertEqual(row['httpStatus'], '')
        self.assertEqual(row['responseBody'], '{}\n200')
        with self.assertRaises(ProbeRejected): _check(row, 'status')

    def test_spawn_error_is_an_attempt_without_new_process_retry(self):
        with patch('learning.mac_controller_probe.subprocess.Popen', side_effect=OSError('unavailable')) as spawn:
            row = read_controller('http://192.0.2.1', 'status')
        self.assertEqual(row['error'], 'OSError')
        self.assertIsNone(row['curlExit'])
        self.assertFalse(row['cleanupIncomplete'])
        spawn.assert_called_once()

    def test_collector_preserves_cancelled_attempt_denominator_and_seals(self):
        f = fixture.ProbeTest(); f.setUp(); self.addCleanup(f.doCleanups)
        process = self.process([KeyboardInterrupt(), (b'{"incomplete', b'cancelled')])
        with patch('learning.mac_controller_probe.subprocess.Popen', return_value=process) as spawn:
            result = f.run_probe(read=read_controller)
        self.assertEqual(result['failure'], 'cancelled')
        self.assertEqual(result['requestCount'], 1)
        row = json.loads((f.root / 'controller/01-status.json').read_text())
        self.assertTrue(row['cancelled'])
        self.assertEqual(base64.b64decode(row['stdoutBase64']), b'{"incomplete')
        self.assertTrue(audit_probe(f.root)['ok'])
        f.launch.assert_not_called()
        spawn.assert_called_once()


if __name__ == '__main__':
    unittest.main()
