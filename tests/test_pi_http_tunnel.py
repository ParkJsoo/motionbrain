"""실제 SSH·장치 없이 로컬 모의 자식으로 터널 준비·정리를 검증한다."""
import copy
import shlex
import socket
import subprocess
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from learning.pi_http_tunnel import PiHttpTunnel, PiHttpTunnelError

REAL_POPEN = subprocess.Popen
FORWARDS = [dict(localPort=18321, targetHost='192.168.219.179'),
            dict(localPort=18322, targetHost='192.168.219.168')]


class Store:
    def __init__(self): self.rows = {}
    def write_json(self, name, value):
        if name in self.rows: raise AssertionError('diagnostic overwritten')
        self.rows[name] = copy.deepcopy(value)


class PiHttpTunnelTests(unittest.TestCase):
    def tunnel(self, **kwargs):
        return PiHttpTunnel('motionbrain-pi', FORWARDS, **kwargs)

    def mock_child(self, tunnel, *, delay=0., ready=True, before='', after='', exit_code=None, ignore_term=False):
        self.args = []
        script = ('import signal,sys,time; ' +
            ('signal.signal(signal.SIGTERM, signal.SIG_IGN); ' if ignore_term else '') +
            f'print({before!r}, file=sys.stderr, flush=True); time.sleep({delay}); ' +
            (f'print({tunnel._ready_line!r}, flush=True); ' if ready else '') +
            (f'print({after!r}, file=sys.stderr, flush=True); ' if after else '') +
            (f'sys.exit({exit_code})' if exit_code is not None else 'time.sleep(20)'))
        def factory(command, **kwargs):
            self.args.append((copy.deepcopy(command), kwargs))
            return REAL_POPEN([sys.executable, '-u', '-c', script], **kwargs)
        return patch('learning.pi_http_tunnel.subprocess.Popen', side_effect=factory)

    def test_constructor_never_starts_process_or_opens_socket(self):
        with patch('learning.pi_http_tunnel.subprocess.Popen') as spawn, patch.object(socket, 'create_connection') as connect:
            tunnel = self.tunnel()
            self.assertFalse(spawn.called); self.assertFalse(connect.called)
            self.assertIsNone(tunnel.process)

    def test_delayed_remote_ready_confirms_connection_without_tcp_probe(self):
        tunnel = self.tunnel(startup_seconds=1)
        started = time.monotonic()
        with self.mock_child(tunnel, delay=.07), \
                patch.object(socket.socket, 'connect', side_effect=AssertionError('TCP readiness probe')), \
                patch.object(socket.socket, 'connect_ex', side_effect=AssertionError('TCP readiness probe')), \
                patch.object(socket, 'create_connection', side_effect=AssertionError('TCP readiness probe')):
            with tunnel as active:
                self.assertIs(active, tunnel)
                self.assertGreaterEqual(time.monotonic()-started, .06)
                tunnel.check()
                self.assertIsNone(tunnel.process.poll())
        self.assertTrue(tunnel.summary['closed'])
        self.assertIsNotNone(tunnel.process.returncode)
        self.assertEqual(tunnel.summary['readinessTcpConnections'], 0)

    def test_safe_ssh_options_and_only_loopback_private_http_forwards(self):
        tunnel = self.tunnel(hold_seconds=180)
        with self.mock_child(tunnel):
            with tunnel:
                command, options = self.args[0]
                for flag in ['-T', '-oBatchMode=yes', '-oConnectTimeout=3', '-oExitOnForwardFailure=yes',
                             '-oControlMaster=no', '-oControlPath=none']:
                    self.assertIn(flag, command)
                forwards = [command[i+1] for i,v in enumerate(command) if v == '-L']
                self.assertEqual(forwards, ['127.0.0.1:18321:192.168.219.179:80',
                                            '127.0.0.1:18322:192.168.219.168:80'])
                self.assertEqual(command[-2], 'motionbrain-pi')
                remote = shlex.split(command[-1])
                self.assertEqual(remote[:3], ['python3', '-u', '-c'])
                self.assertIn('time.sleep(180)', remote[3])
                self.assertEqual(options['stdin'], subprocess.DEVNULL)
                self.assertTrue(options['start_new_session'])
                self.assertNotIn('shell', options)

    def test_exit_or_forward_bind_failure_is_not_ready_and_has_bounded_evidence(self):
        store = Store(); tunnel = self.tunnel(store=store)
        with self.mock_child(tunnel, ready=False, before='bind: Address already in use', exit_code=255):
            with self.assertRaisesRegex(PiHttpTunnelError, 'ssh_exited_before_tunnel_ready'): tunnel.start()
        self.assertFalse(tunnel.summary['ready'])
        self.assertEqual(tunnel.process.returncode, 255)
        row = store.rows['pi-http-tunnel-diagnostic.json']
        self.assertIn('Address already in use', row['stderr'])
        self.assertEqual(row['deviceRequests'], 0)
        self.assertFalse(row['automaticRetry'])

    def test_ready_timeout_cleans_own_child_and_does_not_retry(self):
        tunnel = self.tunnel(startup_seconds=.12)
        with self.mock_child(tunnel, ready=False):
            with self.assertRaisesRegex(PiHttpTunnelError, 'tunnel_ready_timeout'): tunnel.start()
        self.assertEqual(len(self.args), 1)
        self.assertIsNotNone(tunnel.process.returncode)
        with self.assertRaisesRegex(PiHttpTunnelError, 'single_use'): tunnel.start()

    def test_cleanup_kills_unresponsive_own_child_and_waits(self):
        tunnel = self.tunnel()
        other = REAL_POPEN([sys.executable, '-c', 'import time;time.sleep(20)'])
        try:
            with self.mock_child(tunnel, ignore_term=True):
                with tunnel: pass
            self.assertEqual(tunnel.process.returncode, -9)
            self.assertIsNone(other.poll())
            tunnel.close()
        finally:
            other.terminate(); other.wait(timeout=2)

    def test_context_exception_still_reaps_child(self):
        tunnel = self.tunnel()
        with self.mock_child(tunnel):
            with self.assertRaisesRegex(ValueError, 'body_failed'):
                with tunnel: raise ValueError('body_failed')
        self.assertIsNotNone(tunnel.process.returncode)

    def test_output_overflow_is_rejected_and_log_stays_bounded(self):
        store = Store(); tunnel = self.tunnel(store=store, max_log_bytes=1024)
        with self.mock_child(tunnel, ready=False, before='x'*5000):
            with self.assertRaisesRegex(PiHttpTunnelError, 'output_limit'): tunnel.start()
        self.assertIsNotNone(tunnel.process.returncode)
        row = store.rows['pi-http-tunnel-diagnostic.json']
        self.assertLessEqual(len(row['stderr'].encode()), 1024)

    def test_process_exit_after_ready_is_not_healthy(self):
        tunnel = self.tunnel()
        with self.mock_child(tunnel):
            tunnel.start()
            tunnel.process.terminate(); tunnel.process.wait(timeout=2)
            with self.assertRaisesRegex(PiHttpTunnelError, 'tunnel_ssh_exited'): tunnel.check()
        self.assertTrue(tunnel.summary['closed'])

    def test_hold_budget_expiry_cleans_child_without_device_request(self):
        offset = [0.]
        clock = SimpleNamespace(monotonic=lambda:time.monotonic()+offset[0])
        tunnel = self.tunnel(hold_seconds=1, clock=clock)
        with self.mock_child(tunnel):
            tunnel.start(); offset[0] = 2.
            with self.assertRaisesRegex(PiHttpTunnelError, 'hold_expired'): tunnel.check()
        self.assertIsNotNone(tunnel.process.returncode)
        self.assertEqual(tunnel.summary['deviceRequests'], 0)

    def test_spawn_error_is_recorded_without_retry_and_close_is_idempotent(self):
        store = Store(); tunnel = self.tunnel(store=store)
        with patch('learning.pi_http_tunnel.subprocess.Popen', side_effect=OSError('unavailable')) as spawn:
            with self.assertRaises(OSError): tunnel.start()
            self.assertEqual(spawn.call_count, 1)
        tunnel.close()
        self.assertTrue(tunnel.summary['closed'])
        self.assertEqual(store.rows['pi-http-tunnel-diagnostic.json']['failure'], 'OSError')

    def test_bad_alias_target_port_budget_and_mutation_rejected(self):
        for alias in ['-oProxyCommand=bad', 'host;echo bad', 'user@host', 'host\nother']:
            with self.subTest(alias=alias), self.assertRaises(ValueError): PiHttpTunnel(alias, FORWARDS)
        for host in ['8.8.8.8', '127.0.0.1', '169.254.1.1', '::1', '192.168.1.1:80', 123]:
            with self.subTest(host=host), self.assertRaises(ValueError):
                PiHttpTunnel('pi', [dict(localPort=18321, targetHost=host)])
        for port in [True, 80, 65536, '18321']:
            with self.subTest(port=port), self.assertRaises(ValueError):
                PiHttpTunnel('pi', [dict(localPort=port, targetHost='10.0.0.1')])
        for kwargs in [dict(startup_seconds=7), dict(startup_seconds=True), dict(hold_seconds=181),
                       dict(hold_seconds=1.5), dict(max_log_bytes=100000), dict(log_name='../log.json')]:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError): self.tunnel(**kwargs)
        tunnel = self.tunnel(); tunnel.forwards[0]['targetHost'] = '8.8.8.8'
        with patch('learning.pi_http_tunnel.subprocess.Popen') as spawn:
            with self.assertRaisesRegex(PiHttpTunnelError, 'declaration_changed'): tunnel.start()
            self.assertFalse(spawn.called)


if __name__ == '__main__': unittest.main()
