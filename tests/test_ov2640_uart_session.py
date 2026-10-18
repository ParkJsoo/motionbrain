"""Synthetic UART/HTTP only. Optional private raw replay never opens a device."""
import base64
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from learning.ov2640_uart_session import collect, audit, audit_pre_pulse, _identity, _serial_factory
from tests.test_ov2640_register_probe import snapshot, response, HASH
from tools.probe_ov2640_uart import main

PORT = '/dev/mock-camera'
IDENTITY = dict(serial_number=None, vid=1234, pid=5678, location='test-port')
BOOT = 'b' * 32
READY = (b'boot: (SPI_FAST_FLASH_BOOT)\nMotionBrain ESP32-CAM boot\nRegister diagnostic ready: boot='
         + BOOT.encode() + b' appElfSha256=' + HASH.encode() + b' IP=192.168.90.2;\n')


class Clock:
    def __init__(self): self.now = 100.
    def monotonic(self): return self.now
    def sleep(self, seconds):
        if seconds < 0: raise ValueError('negative_sleep')
        self.now += seconds
        time.sleep(.001)


class FakeSerial:
    def __init__(self, raw, event):
        self.event, self.raw = event, raw
        self.is_open = False
        self.opens = self.closes = 0
        self.chunks = queue.Queue()
        self.rts = self.dtr = False
        self.close_error = False
    def open(self):
        self.opens += 1
        self.is_open = True
        self.event('dtr_driver_call', value=False)
        self.event('dtr_driver_return', value=False)
        self.event('rts_driver_call', value=False)
        self.event('rts_driver_return', value=False)
        self.event('suppressed_initial_input_flush', implementation='synthetic')
        for chunk in self.raw: self.chunks.put(chunk)
    def read(self, size):
        try: value = self.chunks.get(timeout=.002)
        except queue.Empty: return b''
        if isinstance(value, Exception): raise value
        return value
    def close(self):
        self.closes += 1
        self.is_open = False
        if self.close_error: raise OSError('synthetic_close_failure')


class Harness:
    def __init__(self, raw=None, edit=None):
        self.raw = [READY] if raw is None else raw
        self.edit = edit
        self.serial = None
        self.calls = 0
        self.enumerations = 0
        self.clock = Clock()
    def ports(self):
        self.enumerations += 1
        return [] if self.enumerations == 1 else [SimpleNamespace(device=PORT, **IDENTITY)]
    def factory(self, port, event):
        self.serial = FakeSerial(self.raw, event)
        event('initial_line_values', dtr=False, rts=False, physicalLevelsMeasured=False)
        return self.serial
    def request(self, url, nonce, *, clock, timeout):
        self.calls += 1
        assert self.serial.is_open
        row = response(snapshot(nonce, self.calls), clock.monotonic())
        if self.edit: self.edit(self, row)
        self.clock.sleep(.1)
        time.sleep(.005)
        return row
    def run(self, path, **kwargs):
        return collect(path, PORT, IDENTITY, HASH, serial_factory=self.factory,
                       ports=self.ports, request=self.request, clock=self.clock, **kwargs)


def replace_artifact(root, descriptor, value):
    raw = (json.dumps(value, sort_keys=True) + '\n').encode()
    (root / descriptor['artifactRef']).write_bytes(raw)
    return dict(artifactRef=descriptor['artifactRef'], bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())


def reseal_result(root, result):
    event_path = root / 'events/00000001.json'
    event = json.loads(event_path.read_text())
    event['payload']['result'] = replace_artifact(root, event['payload']['result'], result)
    event_path.write_text(json.dumps(event))


class UartTests(unittest.TestCase):
    def test_single_retained_handle_three_http_and_audit(self):
        with tempfile.TemporaryDirectory() as folder:
            h = Harness()
            path = Path(folder) / 'run'
            result = h.run(path)
            self.assertTrue(result['successfulRun'], result)
            self.assertEqual((h.serial.opens, h.serial.closes, h.calls), (1, 1, 3))
            self.assertEqual(result['explicitPulseAttempts'], 0)
            self.assertTrue(result['readerStopped'])
            checked = audit(path)
            self.assertTrue(checked['ok'], checked)
            self.assertTrue(checked['successfulRun'])
            self.assertFalse(result['physicalBootCountGuaranteed'])
            self.assertFalse(result['allIntervalLossless'])

    def test_present_at_arming_never_opens(self):
        with tempfile.TemporaryDirectory() as folder:
            h = Harness()
            h.enumerations = 1
            result = h.run(Path(folder) / 'run')
            self.assertEqual(result['failure'], 'target_present_at_arming')
            self.assertIsNone(h.serial)
            self.assertEqual(h.calls, 0)

    def test_identity_mismatch_and_explicit_null(self):
        actual = SimpleNamespace(device=PORT, **IDENTITY)
        _identity(PORT, IDENTITY, [actual])
        for changed in ({k:v for k,v in IDENTITY.items() if k != 'serial_number'},
                        dict(IDENTITY, serial_number='invented'), dict(IDENTITY, location='wrong')):
            with self.assertRaises(ValueError): _identity(PORT, changed, [actual])
        with tempfile.TemporaryDirectory() as folder:
            h = Harness()
            def ports():
                h.enumerations += 1
                return [] if h.enumerations == 1 else [SimpleNamespace(device=PORT, **dict(IDENTITY, pid=1))]
            h.ports = ports
            result = h.run(Path(folder) / 'run')
            self.assertEqual(result['failure'], 'usb_identity_mismatch')
            self.assertIsNone(h.serial)

    def test_appearance_timeout_no_open(self):
        with tempfile.TemporaryDirectory() as folder:
            h = Harness()
            h.ports = lambda: []
            result = h.run(Path(folder) / 'run')
            self.assertEqual(result['failure'], 'target_appearance_timeout')
            self.assertIsNone(h.serial)
            self.assertEqual(h.calls, 0)

    def test_slow_enumeration_cannot_open_after_appearance_deadline(self):
        with tempfile.TemporaryDirectory() as folder:
            h = Harness()
            def ports():
                h.enumerations += 1
                if h.enumerations == 1:
                    return []
                h.clock.now += 31
                return [SimpleNamespace(device=PORT, **IDENTITY)]
            h.ports = ports
            result = h.run(Path(folder) / 'run')
            self.assertEqual(result['failure'], 'target_appearance_timeout')
            self.assertIsNone(h.serial)
            self.assertEqual(h.calls, 0)

    def test_delayed_open_intent_cannot_open_after_appearance_deadline(self):
        from learning.artifacts import ArtifactStore
        original = ArtifactStore.write_json
        with tempfile.TemporaryDirectory() as folder:
            h = Harness()
            def write(store, path, value):
                if path.startswith('lifecycle/') and value.get('kind') == 'open_intent':
                    h.clock.now += 31
                return original(store, path, value)
            with patch.object(ArtifactStore, 'write_json', write):
                result = h.run(Path(folder) / 'run')
            self.assertEqual(result['failure'], 'target_appearance_timeout')
            self.assertEqual(h.serial.opens, 0)
            self.assertEqual(h.calls, 0)

    def test_fragmented_uart_sequence_succeeds(self):
        with tempfile.TemporaryDirectory() as folder:
            h = Harness([READY[:18], READY[18:88], READY[88:]])
            result = h.run(Path(folder) / 'run')
            self.assertTrue(result['successfulRun'], result)
            self.assertEqual(result['rawBytes'], len(READY))
            self.assertEqual(len(result['chunks']), 3)

    def test_missing_or_unordered_boot_and_wrong_firmware_rejected(self):
        for raw in (READY[READY.index(b'Register'):], READY.replace(b'SPI_FAST_FLASH_BOOT', b'unknown'),
                    READY.replace(HASH.encode(), b'c' * 64), READY + READY):
            with self.subTest(raw=raw[:18]), tempfile.TemporaryDirectory() as folder:
                h = Harness([raw])
                result = h.run(Path(folder) / 'run')
                self.assertFalse(result['successfulRun'])
                self.assertEqual(h.calls, 0)
                self.assertEqual(result['explicitPulseAttempts'], 0)

    def test_synthetic_prior_rom_only_failure_is_not_reclassified(self):
        raw = b'ets synthetic\nboot: (SPI_FAST_FLASH_BOOT)\nload:0x0000\n'
        self.assertFalse(audit_pre_pulse(raw)['allowedToPulse'])
        with tempfile.TemporaryDirectory() as folder:
            h = Harness([raw])
            path = Path(folder) / 'run'
            result = h.run(path)
            self.assertEqual(result['failure'], 'ready_timeout')
            self.assertEqual(result['rawBytes'], len(raw))
            self.assertEqual(result['rawSha256'], hashlib.sha256(raw).hexdigest())
            self.assertEqual((h.calls, result['explicitPulseAttempts']), (0, 0))
            self.assertTrue(audit(path)['ok'])
            self.assertFalse(audit(path)['successfulRun'])

    @unittest.skipUnless(os.getenv('MOTIONBRAIN_UART_REGRESSION_RAW'), 'optional private raw replay')
    def test_private_failed_raw_through_actual_runner(self):
        raw = Path(os.environ['MOTIONBRAIN_UART_REGRESSION_RAW']).read_bytes()
        self.assertFalse(audit_pre_pulse(raw)['allowedToPulse'])
        with tempfile.TemporaryDirectory() as folder:
            h = Harness([raw])
            path = Path(folder) / 'run'
            result = h.run(path)
            self.assertEqual(result['failure'], 'ready_timeout')
            self.assertEqual(result['rawSha256'], hashlib.sha256(raw).hexdigest())
            self.assertEqual(result['rawBytes'], len(raw))
            self.assertEqual((h.calls, result['explicitPulseAttempts']), (0, 0))
            self.assertTrue(audit(path)['ok'])
            self.assertFalse(audit(path)['successfulRun'])

    def test_reboot_during_http_stops_and_keeps_original(self):
        def reboot(h, row): h.serial.chunks.put(b'SPI_FAST_FLASH_BOOT\n')
        with tempfile.TemporaryDirectory() as folder:
            h = Harness(edit=reboot)
            path = Path(folder) / 'run'
            result = h.run(path)
            self.assertFalse(result['successfulRun'])
            self.assertEqual(h.calls, 1)
            self.assertEqual(len(result['httpResponses']), 1)
            self.assertTrue(audit(path)['ok'], audit(path))
            self.assertFalse(audit(path)['successfulRun'])

    def test_reader_error_and_overflow_stop_next_http(self):
        for extra in (OSError('synthetic_read_error'), b'x' * 66000):
            with self.subTest(error=type(extra).__name__), tempfile.TemporaryDirectory() as folder:
                def inject(h, row): h.serial.chunks.put(extra)
                h = Harness(edit=inject)
                result = h.run(Path(folder) / 'run')
                self.assertFalse(result['successfulRun'])
                self.assertEqual(h.calls, 1)
                self.assertTrue(result['readerStopped'])

    def test_http_boot_mismatch_keeps_original(self):
        def edit(h, row):
            data = snapshot(row['nonce'], h.calls)
            data['cameraBootId'] = 'c' * 32
            row.update(response(data, row['requestStartedMonotonic']))
        with tempfile.TemporaryDirectory() as folder:
            h = Harness(edit=edit)
            path = Path(folder) / 'run'
            result = h.run(path)
            self.assertFalse(result['successfulRun'])
            self.assertEqual(h.calls, 1)
            self.assertEqual(len(result['httpResponses']), 1)
            self.assertTrue(audit(path)['ok'])

    def test_partial_http_cancel_remains_failure(self):
        def edit(h, row):
            raw = b'{"partial":'
            row.update(cancelled=True, transportCompleted=False, error='cancelled',
                       responseBodyBase64=base64.b64encode(raw).decode(), bodySha256=hashlib.sha256(raw).hexdigest())
        with tempfile.TemporaryDirectory() as folder:
            h = Harness(edit=edit)
            path = Path(folder) / 'run'
            result = h.run(path)
            self.assertFalse(result['successfulRun'])
            self.assertEqual(h.calls, 1)
            self.assertEqual(len(result['httpResponses']), 1)
            self.assertTrue(audit(path)['ok'])

    def test_early_uart_close_during_http_rejected(self):
        def edit(h, row): h.serial.is_open = False
        with tempfile.TemporaryDirectory() as folder:
            h = Harness(edit=edit)
            result = h.run(Path(folder) / 'run')
            self.assertFalse(result['successfulRun'])
            self.assertEqual(h.calls, 1)

    def test_late_fault_after_last_http_overrides_register_success(self):
        def edit(h, row):
            if h.calls == 3: h.serial.chunks.put(b'Guru Meditation\n')
        with tempfile.TemporaryDirectory() as folder:
            h = Harness(edit=edit)
            result = h.run(Path(folder) / 'run')
            self.assertFalse(result['successfulRun'])
            self.assertEqual(h.calls, 3)

    def test_close_failure_kept(self):
        def edit(h, row): h.serial.close_error = True
        with tempfile.TemporaryDirectory() as folder:
            h = Harness(edit=edit)
            result = h.run(Path(folder) / 'run')
            self.assertFalse(result['successfulRun'])
            self.assertIn('uart_close_failed', result['failure'])

    def test_raw_tamper_or_missing_terminal_rejected(self):
        for mode in ('raw', 'terminal'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                path = Path(folder) / 'run'
                Harness().run(path)
                if mode == 'raw': (path / 'uart/0000.raw').write_bytes(b'changed')
                else: (path / 'events/00000001.json').unlink()
                self.assertFalse(audit(path)['ok'])

    def test_cli_is_nonexecuting_without_execute(self):
        with patch('tools.probe_ov2640_uart.collect') as call:
            with self.assertRaises(SystemExit): main(['--port', PORT])
            call.assert_not_called()

    def test_cleanup_evidence_failure_or_cancel_cannot_skip_close(self):
        from learning.artifacts import ArtifactStore, StorageError
        original = ArtifactStore.write_json
        for kind in ('reader_stop_intent', 'close_intent'):
            for error in (StorageError('synthetic_storage_failure'), KeyboardInterrupt()):
                with self.subTest(kind=kind, error=type(error).__name__), tempfile.TemporaryDirectory() as folder:
                    h = Harness()
                    def write(store, path, value):
                        if path.startswith('lifecycle/') and value.get('kind') == kind:
                            raise error
                        return original(store, path, value)
                    with patch.object(ArtifactStore, 'write_json', write):
                        result = h.run(Path(folder) / 'run')
                    self.assertFalse(result['successfulRun'])
                    self.assertEqual((h.serial.opens, h.serial.closes), (1, 1))
                    self.assertFalse(h.serial.is_open)

    def test_reader_start_failure_still_closes_once(self):
        with tempfile.TemporaryDirectory() as folder, patch('threading.Thread.start', side_effect=RuntimeError('synthetic_start_failure')):
            h = Harness()
            result = h.run(Path(folder) / 'run')
            self.assertFalse(result['successfulRun'])
            self.assertEqual((h.serial.opens, h.serial.closes, h.calls), (1, 1, 0))

    def test_uart_fault_after_collector_success_rejects_outer_run(self):
        from learning.ov2640_uart_session import collect_registers
        with tempfile.TemporaryDirectory() as folder:
            h = Harness()
            def wrapped(*args, **kwargs):
                result = collect_registers(*args, **kwargs)
                self.assertTrue(result['successfulRun'])
                h.serial.chunks.put(b'SPI_FAST_FLASH_BOOT\n')
                time.sleep(.01)
                return result
            path = Path(folder) / 'run'
            with patch('learning.ov2640_uart_session.collect_registers', wrapped):
                result = h.run(path)
            self.assertFalse(result['successfulRun'])
            self.assertEqual(h.calls, 3)
            self.assertEqual(result['failure'], 'reboot_marker')
            self.assertFalse(audit(path)['successfulRun'])

    def test_consistent_manifest_with_missing_required_event_fails_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            Harness().run(path)
            result = json.loads((path / 'result.json').read_text())
            for index, descriptor in enumerate(result['lifecycle']):
                event = json.loads((path / descriptor['artifactRef']).read_text())
                if event['kind'] == 'ready_verified':
                    event['kind'] = 'removed'
                    result['lifecycle'][index] = replace_artifact(path, descriptor, event)
                    break
            reseal_result(path, result)
            checked = audit(path)
            self.assertFalse(checked['ok'])
            self.assertIn('incomplete_lifecycle', checked['errors'])

    def test_distinct_http_copies_cannot_be_combined(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            Harness().run(path)
            result = json.loads((path / 'result.json').read_text())
            descriptor = result['httpResponses'][0]
            row = json.loads((path / descriptor['artifactRef']).read_text())
            row['receivedMonotonic'] -= .01
            replacement = replace_artifact(path, descriptor, row)
            result['httpResponses'][0] = replacement
            for index, descriptor in enumerate(result['lifecycle']):
                event = json.loads((path / descriptor['artifactRef']).read_text())
                if event['kind'] == 'http_end' and event['nonce'] == row['nonce']:
                    event['raw'] = replacement
                    result['lifecycle'][index] = replace_artifact(path, descriptor, event)
                    break
            reseal_result(path, result)
            self.assertIn('http_original_register_mismatch', audit(path)['errors'])

    def test_ready_scheduler_jump_past_deadline_rejected(self):
        from learning.ov2640_uart_session import _Reader
        original = _Reader.state
        with tempfile.TemporaryDirectory() as folder:
            h = Harness()
            jumped = False
            def state(reader):
                nonlocal jumped
                value = original(reader)
                if value and not jumped:
                    h.clock.now += 16
                    jumped = True
                return value
            with patch.object(_Reader, 'state', state):
                result = h.run(Path(folder) / 'run')
            self.assertEqual(result['failure'], 'ready_timeout')
            self.assertEqual(h.calls, 0)

    def test_unreported_files_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run'
            Harness().run(path)
            (path / 'uart/9999.raw').write_bytes(b'unreported')
            self.assertIn('unreported_uart', audit(path)['errors'])

    def test_adapter_suppresses_only_initial_flush_without_opening_device(self):
        events = []
        port = _serial_factory(PORT, lambda kind, **v: events.append((kind, v)))
        self.assertFalse(port.is_open)
        port.opening = True
        port._reset_input_buffer()
        self.assertEqual(events[-1][0], 'suppressed_initial_input_flush')
        with self.assertRaises(ValueError): port._reset_input_buffer()
        with self.assertRaises(ValueError): port.reset_output_buffer()
        port.opening = False
        with self.assertRaises(ValueError): port.open()


if __name__ == '__main__': unittest.main()
