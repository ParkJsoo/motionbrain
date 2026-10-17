"""Offline HTTP failure fixtures; no device or camera access."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import shutil
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import unittest
from unittest.mock import patch

TOOL = Path(__file__).resolve().parents[1] / 'tools' / 'diagnose_controller_http.py'
spec = importlib.util.spec_from_file_location('controller_http_diagnostic', TOOL)
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


def payloads():
    return [
        {'state': 'IDLE', 'motorEnabled': False, 'motors': {
            f'M{i}': {'speed': 0, 'enabled': False} for i in range(1, 6)}},
        {'sequence': {'state': 'IDLE', 'totalCount': 0}},
        {'enabled': False, 'active': False, 'bootId': 'boot-1'},
        {'bootId': 'boot-1', 'entries': []},
    ]


class DiagnosticTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.out = Path(self.tmp.name) / 'run'
        self.bodies = [json.dumps(p).encode() for p in payloads()]
        self.calls = []
        self.metrics = b'200\n0.01\n0.02\n0.03\n100'
        self.exit_code = 0

    def runner(self, cmd, stdout, stderr, timeout, check):
        self.calls.append(cmd)
        body = self.bodies[len(self.calls)-1]
        Path(cmd[cmd.index('--output')+1]).write_bytes(body)
        stdout.write(self.metrics)
        stderr.write(b'fixture stderr')
        return subprocess.CompletedProcess(cmd, self.exit_code)

    def run_fixture(self):
        with patch.object(diag.subprocess, 'run', side_effect=self.runner):
            return diag.collect('http://controller.example:80', self.out)

    def test_four_allowlisted_gets_and_raw_evidence(self):
        result = self.run_fixture()
        self.assertTrue(result['complete'])
        self.assertFalse(result['motionQualified'])
        self.assertEqual([c[-1].rsplit('/',1)[1] for c in self.calls], list(diag.ENDPOINTS))
        for cmd in self.calls:
            self.assertEqual(cmd[:3], ['curl', '--disable', '--globoff'])
            self.assertNotIn('--location', cmd)
            self.assertNotIn('--retry', cmd)
            self.assertNotIn('--request', cmd)
        self.assertEqual((self.out/'01-status/response.body').read_bytes(), self.bodies[0])

    def test_timeout_http_000_retains_all_raw_bytes_and_stops(self):
        self.metrics = b'000\n0\n0\n0.8\n0'
        self.exit_code = 28
        self.bodies[0] = b'{partial'
        result = self.run_fixture()
        self.assertEqual(result['failure'], 'transport_error')
        self.assertEqual(result['requests'][0]['metrics']['httpCode'], 0)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual((self.out/'01-status/response.body').read_bytes(), b'{partial')
        self.assertEqual((self.out/'01-status/curl.stderr').read_bytes(), b'fixture stderr')
        self.assertEqual((self.out/'01-status/curl.stdout').read_bytes(), self.metrics)

    def test_http_error_not_confused_with_json_error(self):
        self.metrics = b'503\n0.01\n0.02\n0.03\n100'
        self.bodies[0] = b'<html>unavailable</html>'
        self.assertEqual(self.run_fixture()['failure'], 'http_error')
        self.assertEqual(len(self.calls), 1)

    def test_success_http_with_invalid_json(self):
        self.bodies[0] = b'bad json'
        self.assertEqual(self.run_fixture()['failure'], 'payload_invalid')
        self.assertEqual((self.out/'01-status/response.body').read_bytes(), b'bad json')

    def test_latency_threshold_is_not_collection_timeout(self):
        self.metrics = b'200\n0.01\n0.7\n0.801\n100'
        self.assertEqual(self.run_fixture()['failure'], 'diagnostic_latency_exceeded')
        self.assertEqual(len(self.calls), 1)

    def test_nonidle_or_missing_motor_stops_before_next_endpoint(self):
        self.bodies[0] = b'{"state":"IDLE","motorEnabled":false,"motors":{}}'
        self.assertEqual(self.run_fixture()['failure'], 'controller_not_confirmed_idle')
        self.assertEqual(len(self.calls), 1)

    def test_nonempty_queue_stops(self):
        self.bodies[1] = b'{"sequence":{"state":"IDLE","totalCount":1}}'
        self.assertEqual(self.run_fixture()['failure'], 'sequence_not_confirmed_empty')
        self.assertEqual(len(self.calls), 2)

    def test_active_action_stops(self):
        self.bodies[2] = b'{"enabled":true,"active":true,"bootId":"boot-1"}'
        self.assertEqual(self.run_fixture()['failure'], 'actions_not_confirmed_inactive')
        self.assertEqual(len(self.calls), 3)

    def test_boot_change_stops_and_cannot_claim_complete(self):
        self.bodies[3] = b'{"bootId":"boot-2","entries":[]}'
        result = self.run_fixture()
        self.assertEqual(result['failure'], 'controller_boot_changed')
        self.assertFalse(result['complete'])

    def test_response_size_guard(self):
        self.bodies[0] = b' ' * (diag.POLICY['maxResponseBytes'] + 1)
        self.assertEqual(self.run_fixture()['failure'], 'payload_invalid')

    def test_invalid_metrics_still_retained(self):
        self.metrics = b'not metrics'
        self.assertEqual(self.run_fixture()['failure'], 'metrics_invalid')
        self.assertEqual((self.out/'01-status/curl.stdout').read_bytes(), self.metrics)

    def test_missing_curl_is_reported_without_retry(self):
        with patch.object(diag.subprocess, 'run', side_effect=FileNotFoundError('curl')) as run:
            result = diag.collect('http://controller.example', self.out)
        self.assertEqual(result['failure'], 'curl_unavailable')
        self.assertEqual(run.call_count, 1)

    def test_session_deadline_preserves_partial_evidence(self):
        def timeout(cmd, stdout, stderr, **kwargs):
            stdout.write(b'partial metrics')
            raise subprocess.TimeoutExpired(cmd, kwargs['timeout'])
        with patch.object(diag.subprocess, 'run', side_effect=timeout):
            result = diag.collect('http://controller.example', self.out)
        self.assertEqual(result['failure'], 'session_deadline')
        self.assertEqual((self.out/'01-status/curl.stdout').read_bytes(), b'partial metrics')

    def test_interrupt_has_terminal_report(self):
        with patch.object(diag.subprocess, 'run', side_effect=KeyboardInterrupt):
            result = diag.collect('http://controller.example', self.out)
        self.assertEqual(result['failure'], 'interrupted')
        attempt = json.loads((self.out/'01-status/result.json').read_text())
        self.assertEqual(attempt['failure'], 'interrupted')
        self.assertGreaterEqual(attempt['finishedMonotonic'], attempt['startedMonotonic'])
        self.assertEqual(len(result['requests']), 1)
        self.assertEqual(json.loads((self.out/'result.json').read_text())['failure'], 'interrupted')

    def test_interrupt_after_http_before_result_preserves_request_result(self):
        with patch.object(diag.subprocess, 'run', side_effect=self.runner), patch.object(diag, 'parse_metrics', side_effect=KeyboardInterrupt):
            result = diag.collect('http://controller.example', self.out)
        self.assertEqual(result['failure'], 'interrupted')
        attempt = json.loads((self.out/'01-status/result.json').read_text())
        self.assertEqual(attempt['failure'], 'interrupted')
        self.assertIn('finishedMonotonic', attempt)
        self.assertEqual(len(self.calls), 1)
        self.assertEqual((self.out/'01-status/response.body').read_bytes(), self.bodies[0])

    def test_expired_budget_sends_no_request(self):
        with patch.object(diag.time, 'monotonic', side_effect=[0, 13, 13]), patch.object(diag.subprocess, 'run') as run:
            result = diag.collect('http://controller.example', self.out)
        self.assertEqual(result['failure'], 'session_budget_exhausted')
        run.assert_not_called()

    def test_slow_attempt_write_cannot_start_request_after_deadline(self):
        now = [0]
        original_write = diag.write_json
        def slow_write(path, value):
            original_write(path, value)
            if path.name == 'attempt.json':
                now[0] = 13
        with patch.object(diag.time, 'monotonic', side_effect=lambda: now[0]), patch.object(diag, 'write_json', side_effect=slow_write), patch.object(diag.subprocess, 'run') as run:
            result = diag.collect('http://controller.example', self.out)
        self.assertEqual(result['failure'], 'session_budget_exhausted')
        run.assert_not_called()

    def test_interrupt_preserves_partial_stdout_and_stderr(self):
        def interrupt(cmd, stdout, stderr, **kwargs):
            stdout.write(b'partial metrics')
            stderr.write(b'partial error')
            raise KeyboardInterrupt
        with patch.object(diag.subprocess, 'run', side_effect=interrupt) as run:
            result = diag.collect('http://controller.example', self.out)
        self.assertEqual(result['failure'], 'interrupted')
        self.assertEqual(run.call_count, 1)
        self.assertEqual((self.out/'01-status/curl.stdout').read_bytes(), b'partial metrics')
        self.assertEqual((self.out/'01-status/curl.stderr').read_bytes(), b'partial error')
        self.assertIn('finishedMonotonic', json.loads((self.out/'01-status/result.json').read_text()))

    @unittest.skipUnless(shutil.which('curl'), 'curl unavailable')
    def test_real_curl_uses_only_fixed_local_fixture_gets(self):
        observed = []
        fixtures = dict(zip(diag.ENDPOINTS, self.bodies))
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                observed.append(('GET', self.path))
                body = fixtures[self.path.lstrip('/')]
                self.send_response(200)
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            def log_message(self, *args):
                pass
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            result = diag.collect(f'http://127.0.0.1:{server.server_port}', self.out)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
        self.assertTrue(result['complete'], result)
        self.assertEqual(observed, [('GET', '/' + ep) for ep in diag.ENDPOINTS])

    def test_closed_directory_cannot_be_reused(self):
        self.run_fixture()
        before = (self.out/'result.json').read_bytes()
        with patch.object(diag.subprocess, 'run') as run:
            with self.assertRaises(FileExistsError):
                diag.collect('http://controller.example', self.out)
            run.assert_not_called()
        self.assertEqual((self.out/'result.json').read_bytes(), before)

    def test_invalid_targets_rejected_before_output_or_request(self):
        for target in ('https://controller', 'http://user:pw@controller',
                       'http://controller/capture', 'http://controller?command=run',
                       'http://controller/#x', 'http://controller:99999',
                       'http://a{b,c}', 'http://a b', 'file:///tmp/x'):
            with self.subTest(target=target), patch.object(diag.subprocess, 'run') as run:
                with self.assertRaises(ValueError):
                    diag.collect(target, self.out)
                run.assert_not_called()
                self.assertFalse(self.out.exists())


if __name__ == '__main__':
    unittest.main()
