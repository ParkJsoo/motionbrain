import base64
from copy import deepcopy
import hashlib
import http.client
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.ov2640_register_probe import (
    AEC_REGISTERS, DSP_REGISTERS, SENSOR_REGISTERS, SCHEMA, audit, collect,
    direct_get, validate_snapshot)
from tools.probe_ov2640_registers import main

HASH = 'a' * 64
NONCE = '1' * 16


class Clock:
    def __init__(self):
        self.now = 100.
    def monotonic(self):
        return self.now
    def sleep(self, duration):
        self.now += duration


def snapshot(nonce=NONCE, ordinal=1, bank=0):
    operations = [('read', -1, 255, bank), ('write_bank', 1, 255, 1), ('read', 1, 255, 1)]
    operations += [('read', 1, r, 38 if r == 10 else 0) for r in SENSOR_REGISTERS]
    operations += [('write_bank', 0, 255, 0), ('read', 0, 255, 0)]
    operations += [('read', 0, r, 0) for r in DSP_REGISTERS]
    operations += [('write_bank', bank, 255, bank), ('read', bank, 255, bank)]
    tx = [dict(op=op, bank=b, register=r, value=v, status=0,
               addressStatus=0 if op == 'read' else None, startedUs=i, endedUs=i + 1)
          for i, (op, b, r, v) in enumerate(operations)]
    return dict(schemaVersion=SCHEMA, nonce=nonce, cameraBootId='b' * 32, appElfSha256=HASH,
                attempt=ordinal, ok=True, failure='', startedUs=0, endedUs=len(tx),
                xclkRequestedHz=20000000, xclkConfiguredHz=20000000,
                sensorPidReported=38, sensorAddress=48, frameSize='qvga', frameWidth=320,
                frameHeight=240, jpegQuality=15, internalAcquisition=True, imageExports=0, imageRequests=0,
                trialClosed=ordinal == 3, cameraDeinitStatus=0 if ordinal == 3 else None,
                exposureUpperBoundUs=None, opticalExposureVerified=False,
                exposureUpperBoundVerified=False, atomicSnapshot=False, bankRestored=True,
                trainingEligible=False, controlEligible=False, transactions=tx)


def response(value, start=100.):
    raw = json.dumps(value).encode()
    return dict(method='GET', route='/diagnostics/ov2640?nonce=' + value['nonce'], nonce=value['nonce'],
                transportCompleted=True, httpStatus=200, responseBodyBase64=base64.b64encode(raw).decode(),
                bodySha256=hashlib.sha256(raw).hexdigest(), requestStartedMonotonic=start,
                receivedMonotonic=start + .1)


class ProbeTests(unittest.TestCase):
    def request(self, edit=None):
        calls = []
        def fake(url, nonce, *, clock, timeout):
            calls.append(nonce)
            value = snapshot(nonce, len(calls))
            if edit:
                edit(value)
            row = response(value, clock.monotonic())
            clock.sleep(.1)
            return row
        return calls, fake

    def test_three_requests_and_independent_audit(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            calls, request = self.request()
            result = collect(path, 'http://localhost:1234', HASH, request=request, clock=Clock())
            self.assertTrue(result['successfulRun'])
            self.assertEqual(len(calls), 3)
            self.assertEqual(len(set(calls)), 3)
            checked = audit(path)
            self.assertTrue(checked['ok'], checked)
            self.assertTrue(checked['successfulRun'])
            self.assertFalse(checked['opticalExposureVerified'])
            self.assertIsNone(checked['exposureUpperBoundUs'])

    def test_intent_is_durable_before_request(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            calls, fake = self.request()
            def request(*args, **kwargs):
                ordinal = len(calls) + 1
                self.assertTrue((path / f'attempts/{ordinal:02d}-intent.json').is_file())
                self.assertTrue((path / 'plan.json').is_file())
                return fake(*args, **kwargs)
            self.assertTrue(collect(path, 'http://localhost', HASH, request=request, clock=Clock())['successfulRun'])

    def test_first_failure_stops_with_full_denominator(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            calls, request = self.request(lambda s: s.update(ok=False, failure='sccb_read'))
            result = collect(path, 'http://localhost', HASH, request=request, clock=Clock())
            self.assertEqual(len(calls), 1)
            self.assertEqual(result['plannedCount'], 3)
            self.assertFalse(result['successfulRun'])
            self.assertTrue(audit(path)['ok'])
            self.assertFalse(audit(path)['successfulRun'])

    def test_boot_change_stops_at_second_response(self):
        def edit(s):
            if s['attempt'] == 2:
                s['cameraBootId'] = 'c' * 32
        with tempfile.TemporaryDirectory() as folder:
            calls, request = self.request(edit)
            result = collect(Path(folder) / 'run', 'http://localhost', HASH, request=request, clock=Clock())
            self.assertEqual(len(calls), 2)
            self.assertEqual(result['failure'], 'boot_firmware_or_configuration_changed')

    def test_runtime_changes_and_bank_errors_rejected(self):
        base = snapshot()
        for change in ('pid', 'bank', 'missing', 'write', 'static', 'upper', 'atomic', 'identity'):
            with self.subTest(change=change):
                s = snapshot()
                if change == 'pid': s['transactions'][3]['value'] = 39
                if change == 'bank': s['transactions'][-1]['value'] = 1
                if change == 'missing': s['transactions'].pop(3)
                if change == 'write': s['transactions'][3]['op'] = 'write_bank'
                if change == 'static': s['transactions'][4]['value'] = 1
                if change == 'upper': s['exposureUpperBoundUs'] = 30000
                if change == 'atomic': s['atomicSnapshot'] = True
                if change == 'identity': s['appElfSha256'] = 'c' * 64
                with self.assertRaises(ValueError):
                    validate_snapshot(response(s), NONCE, HASH, 1, base)

    def test_cleanup_image_requests_xclk_and_snapshot_budget(self):
        changes = [('trialClosed', True, 1), ('trialClosed', False, 3),
                   ('cameraDeinitStatus', 0, 1), ('cameraDeinitStatus', None, 3),
                   ('cameraDeinitStatus', -1, 3), ('cameraDeinitStatus', False, 3),
                   ('imageRequests', 1, 1), ('imageRequests', False, 1),
                   ('xclkConfiguredHz', 19999999, 1), ('endedUs', 750001, 1)]
        for key, value, ordinal in changes:
            with self.subTest(key=key, value=value, ordinal=ordinal):
                s = snapshot(ordinal=ordinal)
                s[key] = value
                with self.assertRaises(ValueError):
                    validate_snapshot(response(s), NONCE, HASH, ordinal)
        s = snapshot()
        s['endedUs'] = 750000
        self.assertTrue(validate_snapshot(response(s), NONCE, HASH, 1)['ok'])

    def test_aec_may_change_but_is_never_atomic(self):
        s = snapshot()
        for tx in s['transactions']:
            if tx['bank'] == 1 and tx['register'] in AEC_REGISTERS:
                tx['value'] = {0x04: 3, 0x10: 254, 0x45: 63}[tx['register']]
        value = validate_snapshot(response(s), NONCE, HASH, 1, snapshot())
        self.assertFalse(value['atomicSnapshot'])

    def test_initial_bank_one_restored(self):
        self.assertTrue(validate_snapshot(response(snapshot(bank=1)), NONCE, HASH, 1)['bankRestored'])

    def test_wrong_nonce_attempt_or_failed_bus(self):
        for key, value in [('nonce', 'f' * 16), ('attempt', 2), ('bankRestored', False)]:
            with self.subTest(key=key):
                s = snapshot()
                s[key] = value
                with self.assertRaises(ValueError): validate_snapshot(response(s), NONCE, HASH, 1)
        s = snapshot()
        s['transactions'][3].update(addressStatus=5, status=-32768, value=None)
        with self.assertRaises(ValueError): validate_snapshot(response(s), NONCE, HASH, 1)

    def test_partial_cancelled_body_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            def request(url, nonce, *, clock, timeout):
                row = response(snapshot(nonce))
                raw = b'{"partial":'
                row.update(cancelled=True, error='cancelled', transportCompleted=False,
                           responseBodyBase64=base64.b64encode(raw).decode(),
                           bodySha256=hashlib.sha256(raw).hexdigest())
                clock.sleep(.1)
                return row
            result = collect(path, 'http://localhost', HASH, request=request, clock=Clock())
            self.assertEqual(result['failure'], 'cancelled')
            self.assertEqual(result['responseCount'], 1)
            raw = json.loads((path / 'attempts/01-raw.json').read_text())
            self.assertEqual(base64.b64decode(raw['responseBodyBase64']), b'{"partial":')
            self.assertTrue(audit(path)['ok'])

    def test_existing_directory_refused_no_requests(self):
        with tempfile.TemporaryDirectory() as folder:
            calls, request = self.request()
            with self.assertRaises(Exception): collect(folder, 'http://localhost', HASH, request=request)
            self.assertEqual(calls, [])

    def test_changed_raw_or_missing_terminal_fails_audit(self):
        for mode in ('raw', 'terminal'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'run'
                _, request = self.request()
                collect(path, 'http://localhost', HASH, request=request, clock=Clock())
                if mode == 'raw': (path / 'attempts/01-raw.json').write_text('{}')
                else: (path / 'events/00000001.json').unlink()
                self.assertFalse(audit(path)['ok'])

    def test_cli_requires_explicit_execution(self):
        with patch('tools.probe_ov2640_registers.collect') as call:
            with self.assertRaises(SystemExit): main(['--url', 'http://localhost'])
            call.assert_not_called()

    def test_unsafe_url_or_nonce_no_connection(self):
        with patch('learning.ov2640_register_probe.http.client.HTTPConnection') as connection:
            for url, nonce in [('http://localhost/capture', NONCE), ('http://localhost', '../capture'), ('https://localhost', NONCE)]:
                with self.assertRaises(ValueError): direct_get(url, nonce)
            connection.assert_not_called()

    def test_direct_transport_partial_exception_and_cancel(self):
        for exception in (KeyboardInterrupt(), http.client.IncompleteRead(b'partial')):
            with self.subTest(exception=type(exception).__name__):
                class Response:
                    status = 200
                    length = 10
                    chunks = 0
                    def read1(self, count):
                        self.chunks += 1
                        if self.chunks == 1: return b'prefix'
                        raise exception
                with patch('learning.ov2640_register_probe.http.client.HTTPConnection') as constructor:
                    conn = constructor.return_value
                    conn.getresponse.return_value = Response()
                    result = direct_get('http://localhost', NONCE)
                    self.assertFalse(result['transportCompleted'])
                    expected = b'prefix' if isinstance(exception, KeyboardInterrupt) else b'prefixpartial'
                    self.assertEqual(base64.b64decode(result['responseBodyBase64']), expected)
                    conn.close.assert_called_once()
                    self.assertEqual(conn.request.call_args.args[:2], ('GET', '/diagnostics/ov2640?nonce=' + NONCE))

    def test_non_exposure_bits_in_aec_bytes_are_static(self):
        for register, bit in ((0x04, 4), (0x45, 64)):
            s = snapshot()
            next(t for t in s['transactions'] if t['bank'] == 1 and t['register'] == register)['value'] = bit
            with self.assertRaisesRegex(ValueError, 'configuration_changed'):
                validate_snapshot(response(s), NONCE, HASH, 1, snapshot())

    def test_signal_during_storage_preserves_raw_and_stops(self):
        import signal
        from learning.artifacts import ArtifactStore
        original = ArtifactStore.write_json
        for phase in ('intent', 'raw'):
            with self.subTest(phase=phase), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'run'
                calls, request = self.request()
                def write(store, relative, value):
                    if relative == f'attempts/01-{phase}.json':
                        signal.raise_signal(signal.SIGINT)
                    return original(store, relative, value)
                with patch.object(ArtifactStore, 'write_json', write):
                    result = collect(path, 'http://localhost', HASH, request=request, clock=Clock())
                self.assertEqual(result['failure'], 'cancelled')
                self.assertEqual(len(calls), 1 if phase == 'raw' else 0)
                self.assertEqual(result['responseCount'], len(calls))
                self.assertTrue(audit(path)['ok'])
                self.assertFalse(audit(path)['successfulRun'])

    def test_late_success_is_rejected_without_next_request(self):
        with tempfile.TemporaryDirectory() as folder:
            calls, fake = self.request()
            def request(*args, **kwargs):
                row = fake(*args, **kwargs)
                row['receivedMonotonic'] = row['requestStartedMonotonic'] + 2.01
                kwargs['clock'].sleep(2)
                return row
            result = collect(Path(folder) / 'run', 'http://localhost', HASH, request=request, clock=Clock())
            self.assertEqual(len(calls), 1)
            self.assertEqual(result['failure'], 'request_timing')

    def test_session_deadline_prevents_late_second_attempt(self):
        class SlowClock(Clock):
            def sleep(self, duration):
                self.now += 29 if duration == 1 else duration
        with tempfile.TemporaryDirectory() as folder:
            calls, request = self.request()
            result = collect(Path(folder) / 'run', 'http://localhost', HASH, request=request, clock=SlowClock())
            self.assertEqual(len(calls), 1)
            self.assertEqual(result['plannedCount'], 3)
            self.assertEqual(result['failure'], 'session_budget_exhausted')

    def test_cancel_before_response_keeps_intent(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            def request(*args, **kwargs): raise KeyboardInterrupt()
            result = collect(path, 'http://localhost', HASH, request=request, clock=Clock())
            self.assertEqual(result['intentCount'], 1)
            self.assertEqual(result['responseCount'], 0)
            self.assertEqual(result['failure'], 'cancelled')
            self.assertTrue(audit(path)['ok'])
            self.assertFalse(audit(path)['successfulRun'])

    def test_unreported_attempt_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            _, request = self.request()
            collect(path, 'http://localhost', HASH, request=request, clock=Clock())
            (path / 'attempts/04-raw.json').write_text('{}')
            self.assertIn('unreported_or_missing_attempt_artifacts', audit(path)['errors'])

    def test_invalid_expected_firmware_refused_before_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            calls, request = self.request()
            with self.assertRaises(ValueError): collect(path, 'http://localhost', 'bad', request=request)
            self.assertEqual(calls, [])
            self.assertFalse(path.exists())

    def test_redirect_and_duplicate_json_never_qualify(self):
        row = response(snapshot())
        row['httpStatus'] = 302
        with self.assertRaises(ValueError): validate_snapshot(row, NONCE, HASH, 1)
        raw = b'{"nonce":"a","nonce":"b"}'
        row.update(httpStatus=200, responseBodyBase64=base64.b64encode(raw).decode(),
                   bodySha256=hashlib.sha256(raw).hexdigest())
        with self.assertRaises(ValueError): validate_snapshot(row, NONCE, HASH, 1)

    def test_hard_deadline_interrupts_drip_response(self):
        import time
        class Response:
            status = 200
            length = None
            def read1(self, count):
                time.sleep(.03)
                return b'x'
        with patch('learning.ov2640_register_probe.http.client.HTTPConnection') as constructor:
            constructor.return_value.getresponse.return_value = Response()
            result = direct_get('http://localhost', NONCE, timeout=.01)
            self.assertFalse(result['transportCompleted'])
            self.assertEqual(result['error'], 'request_deadline')
            constructor.return_value.close.assert_called_once()

    def test_oversized_body_never_completes(self):
        class Response:
            status = 200
            length = None
            def read1(self, count): return b'x' * count
        with patch('learning.ov2640_register_probe.http.client.HTTPConnection') as constructor:
            constructor.return_value.getresponse.return_value = Response()
            result = direct_get('http://localhost', NONCE)
            self.assertFalse(result['transportCompleted'])
            self.assertEqual(len(base64.b64decode(result['responseBodyBase64'])), 32769)
            self.assertEqual(result['error'], 'response_too_large')


if __name__ == '__main__':
    unittest.main()
