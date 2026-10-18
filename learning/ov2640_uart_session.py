"""One retained UART handle around a finite diagnostic; never upload firmware.

Observation starts after open returns. Suppressing pyserial's initial flush does
not prove that OS/USB buffering, line changes, or pre-open bytes are lossless.
"""
import base64
import hashlib
import ipaddress
import math
import os
from pathlib import Path
import re
import threading
import time

from learning.artifacts import (ArtifactStore, StorageError, _decode_json, _read,
                                read_json_artifact, read_events, verify_artifacts)
from learning.ov2640_register_probe import (collect as collect_registers, direct_get, audit as audit_registers,
                                            _defer_cancellation)

POLICY = dict(mode='observe_power_on', appearanceSeconds=30, readySeconds=15,
              registerSeconds=30, maxRequests=3, explicitPulseLimit=0,
              serialReadTimeoutSeconds=.02, readerJoinSeconds=.5, readerJoinTotalSeconds=1,
              finalObservationSeconds=.25, maxUartBytes=65536,
              uploads=0, reconnects=0, retries=0, serialWrites=0)
READY = re.compile(rb'Register diagnostic ready: boot=([0-9a-f]{32}) appElfSha256=([0-9a-f]{64}) IP=([0-9.]+);')
BOOT_MARKERS = (b'SPI_FAST_FLASH_BOOT', b'MotionBrain ESP32-CAM boot', b'Register diagnostic ready:')
FAULT_MARKERS = (b'Guru Meditation', b'Brownout detector', b'panic', b'watchdog')


def audit_pre_pulse(raw):
    """Offline regression entry point. Any observed pre-pulse boot stays rejected."""
    if not isinstance(raw, bytes):
        raise TypeError('raw_bytes_required')
    found = [m.decode() for m in BOOT_MARKERS + FAULT_MARKERS if m in raw]
    return dict(allowedToPulse=not found, failure='unexpected_pre_pulse_boot_or_fault' if found else None,
                markers=found, bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
                physicalBootCountGuaranteed=False, opticalExposureVerified=False)


def _serial_factory(port_name, event):
    import serial
    from serial.serialposix import Serial
    if os.name != 'posix' or serial.__version__ != '3.5' or serial.Serial is not Serial:
        raise ValueError('supported_posix_pyserial_3_5_required')
    class RetainedSerial(Serial):
        initial_flush_suppressed = 0
        opening = False
        dtr_applied = False
        rts_applied = False
        def open(self):
            if self.initial_flush_suppressed:
                raise ValueError('reopen_forbidden')
            self.opening = True
            try:
                super().open()
            finally:
                self.opening = False
            if not self.dtr_applied or not self.rts_applied:
                raise ValueError('initial_line_application_failed')
            if self.initial_flush_suppressed != 1:
                raise ValueError('initial_flush_hook_unverified')
        def _reset_input_buffer(self):
            if not self.opening or self.initial_flush_suppressed:
                raise ValueError('input_flush_forbidden')
            self.initial_flush_suppressed += 1
            event('suppressed_initial_input_flush', implementation='pyserial_3.5_posix_private_hook')
        def reset_output_buffer(self):
            raise ValueError('output_flush_forbidden')
        def _update_dtr_state(self):
            event('dtr_driver_call', value=self._dtr_state)
            super()._update_dtr_state()
            self.dtr_applied = True
            event('dtr_driver_return', value=self._dtr_state)
        def _update_rts_state(self):
            event('rts_driver_call', value=self._rts_state)
            super()._update_rts_state()
            self.rts_applied = True
            event('rts_driver_return', value=self._rts_state)
    port = RetainedSerial(port=None, baudrate=115200, timeout=.02, write_timeout=.2, exclusive=True)
    port.dtr = False
    port.rts = False
    event('initial_line_values', dtr=False, rts=False, physicalLevelsMeasured=False)
    port.port = port_name
    return port


def _ports():
    from serial.tools import list_ports
    return list_ports.comports()


def _identity(port, identity, available):
    if (not isinstance(identity, dict) or set(identity) != {'serial_number', 'vid', 'pid', 'location'}
            or (identity['serial_number'] is not None and (not isinstance(identity['serial_number'], str) or not identity['serial_number']))
            or not isinstance(identity['location'], str) or not identity['location']
            or any(type(identity[k]) is not int or not 0 <= identity[k] <= 65535 for k in ('vid', 'pid'))):
        raise ValueError('explicit_complete_usb_identity_required')
    matches = [p for p in available if p.device == port]
    if len(matches) != 1 or any(getattr(matches[0], k, None) != v for k, v in identity.items()):
        raise ValueError('usb_identity_mismatch')


class _Reader:
    def __init__(self, port, store, clock):
        self.port, self.store, self.clock = port, store, clock
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.raw = bytearray()
        self.chunks = []
        self.failure = None
        self.phase = 'ready_wait'
        self.ready = None
        self.finished = False
        self.started = None
        self.ended = None
        self.thread = threading.Thread(target=self.run, name='ov2640-uart-reader', daemon=True)
    def run(self):
        self.started = self.clock.monotonic()
        try:
            while not self.stop.is_set():
                chunk = self.port.read(4096)
                at = self.clock.monotonic()
                if not isinstance(chunk, bytes):
                    raise ValueError('serial_read_not_bytes')
                if chunk:
                    with self.lock:
                        ordinal = len(self.chunks)
                        phase = self.phase
                    descriptor = self.store._write(f'uart/{ordinal:04d}.raw', chunk)
                    receipt = self.store.write_json(f'receipts/{ordinal:04d}.json',
                        dict(raw=descriptor, receivedMonotonic=at, phase=phase))
                    with self.lock:
                        self.raw.extend(chunk)
                        self.chunks.append(receipt)
                        if len(self.raw) > POLICY['maxUartBytes']:
                            self.failure = 'uart_byte_budget'
                        self._detect()
                        if self.failure:
                            break
                if not getattr(self.port, 'is_open', False):
                    raise ValueError('uart_closed_while_observing')
        except Exception as exc:
            with self.lock:
                self.failure = self.failure or 'uart_reader_error:' + str(exc)
        finally:
            self.ended = self.clock.monotonic()
            self.finished = True
    def _detect(self):
        raw = bytes(self.raw)
        post = raw
        if any(post.count(m) > 1 for m in BOOT_MARKERS):
            self.failure = self.failure or 'reboot_marker'
        if any(m in post for m in FAULT_MARKERS):
            self.failure = self.failure or 'uart_fault_marker'
        if self.phase == 'ready_wait' and b'Register diagnostic closed:' in post:
            self.failure = self.failure or 'closed_before_ready'
        found = READY.search(post)
        if found:
            first = post.find(BOOT_MARKERS[0])
            second = post.find(BOOT_MARKERS[1])
            if not 0 <= first < second < found.start():
                self.failure = self.failure or 'incomplete_or_unordered_boot_sequence'
            candidate = dict(bootId=found[1].decode(), appElfSha256=found[2].decode(), ip=found[3].decode())
            if self.ready is not None and candidate != self.ready:
                self.failure = self.failure or 'ready_identity_changed'
            self.ready = candidate
    def state(self):
        with self.lock:
            if self.failure:
                raise ValueError(self.failure)
            if self.finished:
                raise ValueError('uart_reader_stopped_early')
            return dict(self.ready) if self.ready else None
    def phase_to(self, phase):
        with self.lock:
            self.phase = phase
            self._detect()


def collect(output_dir, port, expected_identity, expected_app_elf_sha256, *,
            serial_factory=_serial_factory, ports=_ports, request=direct_get, clock=time):
    """Execute only after explicit approval; injectable dependencies are for offline tests."""
    if not isinstance(expected_app_elf_sha256, str) or not re.fullmatch('[0-9a-f]{64}', expected_app_elf_sha256):
        raise ValueError('expected_firmware_hash_required')
    store = ArtifactStore(Path(output_dir))
    events = []
    link_refs = []
    def event(kind, **fields):
        item = dict(kind=kind, monotonic=clock.monotonic(), **fields)
        with _defer_cancellation():
            ref = store.write_json(f'lifecycle/{len(events):04d}.json', item)
            events.append(ref)
    plan = store.write_json('plan.json', dict(schemaVersion='motionbrain.ov2640_uart_plan.v1',
        policy=POLICY, port=port, usbIdentity=expected_identity, expectedAppElfSha256=expected_app_elf_sha256,
        physicalBootCountGuaranteed=False, allIntervalLossless=False,
        initialOpenMayChangeLines=True, initialPyserialFlushSuppressed=True,
        readyBoundOrigin='open_return', commandedResetPulses=0))
    device = reader = None
    failure = None
    ready = None
    register_result = None
    reader_started = False
    def check():
        if reader is not None:
            reader.state()
    def pause(duration):
        until = clock.monotonic() + duration
        while clock.monotonic() < until:
            check()
            remaining = until - clock.monotonic()
            if remaining > 0:
                clock.sleep(min(.02, remaining))
        check()
    try:
        # A new approved power-on trial must be armed while this target is absent.
        initial_ports = ports()
        if any(p.device == port for p in initial_ports):
            raise ValueError('target_present_at_arming')
        event('target_absent_at_arming')
        appearance_limit = clock.monotonic() + POLICY['appearanceSeconds']
        while clock.monotonic() < appearance_limit:
            available = ports()
            if clock.monotonic() >= appearance_limit:
                raise ValueError('target_appearance_timeout')
            if any(p.device == port for p in available):
                _identity(port, expected_identity, available)
                event('target_appearance_identity_verified', usbIdentity=expected_identity)
                break
            clock.sleep(.05)
        else:
            raise ValueError('target_appearance_timeout')
        event('serial_create_intent')
        if clock.monotonic() >= appearance_limit:
            raise ValueError('target_appearance_timeout')
        device = serial_factory(port, event)
        event('open_intent')
        if clock.monotonic() >= appearance_limit:
            raise ValueError('target_appearance_timeout')
        device.open()
        opened = clock.monotonic()
        event('open_return', isOpen=device.is_open)
        reader = _Reader(device, store, clock)
        event('reader_start_intent')
        reader.thread.start()
        reader_started = True
        until = opened + POLICY['readySeconds']
        while clock.monotonic() < until:
            ready = reader.state()
            if ready:
                break
            pause(.02)
        if ready is None or clock.monotonic() > until:
            raise ValueError('ready_timeout')
        if ready['appElfSha256'] != expected_app_elf_sha256:
            raise ValueError('ready_firmware_mismatch')
        ip = ipaddress.IPv4Address(ready['ip'])
        if not ip.is_private or ip.is_loopback or ip.is_unspecified or ip.is_multicast:
            raise ValueError('invalid_ready_ip')
        ready.update(url='http://' + str(ip), observedMonotonic=clock.monotonic())
        event('ready_verified', **ready)
        reader.phase_to('http')
        class MonitoredClock:
            monotonic = staticmethod(clock.monotonic)
            sleep = staticmethod(pause)
        def linked_request(url, nonce, **kwargs):
            check()
            event('http_start', nonce=nonce)
            row = request(url, nonce, **kwargs)
            # Persist the unmodified transport response before adding linkage or
            # UART-fault rejection, including cancellation and partial bodies.
            with _defer_cancellation():
                raw_ref = store.write_json(f'http-links/{nonce}.json', row)
                link_refs.append(raw_ref)
                event('http_end', nonce=nonce, raw=raw_ref)
            try:
                body = _decode_json(base64.b64decode(row.get('responseBodyBase64', ''), validate=True))
                if isinstance(body, dict) and body.get('cameraBootId') != ready['bootId']:
                    row['error'] = 'uart_http_boot_identity_mismatch'
                check()
            except (ValueError, TypeError, UnicodeError, RecursionError) as exc:
                row['error'] = row.get('error') or str(exc) or type(exc).__name__
            return row
        register_result = collect_registers(store.root / 'registers', ready['url'], expected_app_elf_sha256,
                                            request=linked_request, clock=MonitoredClock)
        if not register_result['successfulRun']:
            raise ValueError('register_collection_failed:' + str(register_result['failure']))
        pause(POLICY['finalObservationSeconds'])
    except KeyboardInterrupt:
        failure = 'cancelled'
    except (OSError, ValueError, StorageError, RuntimeError) as exc:
        failure = str(exc) or type(exc).__name__
    finally:
        def cleanup_event(kind, **fields):
            nonlocal failure
            try:
                event(kind, **fields)
            except KeyboardInterrupt:
                failure = failure or 'cancelled_during_cleanup'
            except Exception as exc:
                failure = failure or 'cleanup_evidence_failed:' + str(exc)
        # Evidence failure must never skip resource cleanup. Stop and close use
        # independent finally clauses; join is legal only after start succeeded.
        try:
            if reader is not None:
                cleanup_event('reader_stop_intent')
                reader.stop.set()
                if reader_started:
                    try:
                        reader.thread.join(POLICY['readerJoinSeconds'])
                    except KeyboardInterrupt:
                        failure = failure or 'cancelled_during_cleanup'
                    if reader.thread.is_alive():
                        failure = failure or 'reader_join_timeout'
                cleanup_event('reader_join_return', stopped=not reader.thread.is_alive(),
                              readerStarted=reader.started, readerEnded=reader.ended)
        finally:
            if device is not None:
                cleanup_event('close_intent')
                try:
                    with _defer_cancellation():
                        device.close()
                    cleanup_event('close_return', isOpen=device.is_open)
                    if device.is_open:
                        failure = failure or 'uart_close_unverified'
                except KeyboardInterrupt:
                    failure = failure or 'cancelled_during_cleanup'
                except Exception as exc:
                    failure = failure or 'uart_close_failed:' + str(exc)
            if reader is not None:
                if reader_started and reader.thread.is_alive():
                    try:
                        reader.thread.join(POLICY['readerJoinSeconds'])
                    except KeyboardInterrupt:
                        failure = failure or 'cancelled_during_cleanup'
                failure = failure or reader.failure
    with reader.lock if reader is not None else threading.Lock():
        chunks = list(reader.chunks) if reader else []
        raw = bytes(reader.raw) if reader else b''
    result = dict(schemaVersion='motionbrain.ov2640_uart_run.v1', plan=plan, lifecycle=events,
        chunks=chunks, httpResponses=link_refs, rawBytes=len(raw), rawSha256=hashlib.sha256(raw).hexdigest(),
        explicitPulseAttempts=0, ready=ready, failure=failure,
        successfulRun=failure is None and bool(register_result and register_result['successfulRun']),
        readerStopped=reader is None or not reader.thread.is_alive(),
        registerRunCreated=(store.root / 'registers').exists(),
        physicalBootCountGuaranteed=False, allIntervalLossless=False,
        opticalExposureVerified=False, trainingEligible=False, controlEligible=False,
        powerOffCameraRequired=True)
    ref = store.write_json('result.json', result)
    store.append('uart_session_sealed', 'observation', {'result': ref})
    return result


def audit(root):
    """Offline raw and lifecycle audit; failures remain failures."""
    root = Path(root)
    errors = []
    result = {}
    try:
        events, problems = read_events(root)
        if problems or len(events) != 1 or events[0]['kind'] != 'uart_session_sealed':
            raise ValueError('missing_uart_terminal')
        result = read_json_artifact(root, events[0]['payload']['result'])
        plan = read_json_artifact(root, result['plan'])
        if plan['policy'] != POLICY:
            raise ValueError('policy_mismatch')
        lifecycle = [read_json_artifact(root, d) for d in result['lifecycle']]
        kinds = [e['kind'] for e in lifecycle]
        times = [e['monotonic'] for e in lifecycle]
        if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in times) or times != sorted(times):
            raise ValueError('lifecycle_time_order')
        for index, descriptor in enumerate(result['lifecycle']):
            if descriptor['artifactRef'] != f'lifecycle/{index:04d}.json':
                raise ValueError('lifecycle_reference')
        originals = [read_json_artifact(root, d) for d in result['httpResponses']]
        for directory, descriptors in (('lifecycle', result['lifecycle']),
                                       ('receipts', result['chunks']), ('http-links', result['httpResponses'])):
            refs = [d['artifactRef'] for d in descriptors]
            actual = {str(p.relative_to(root)) for p in (root / directory).iterdir()} if (root / directory).exists() else set()
            if len(refs) != len(set(refs)) or actual != set(refs):
                raise ValueError('unreported_or_duplicate_' + directory)

        if result['registerRunCreated']:
            checked = audit_registers(root / 'registers')
            if not checked['ok']:
                errors.append('register_integrity_failed')
        if not result['readerStopped']:
            errors.append('reader_not_stopped_evidence_incomplete')
        if kinds.count('open_intent') > 1 or kinds.count('rts_assert_intent') > 0:
            raise ValueError('reopen_or_repeated_pulse')
        if kinds.count('rts_assert_intent') != result['explicitPulseAttempts']:
            raise ValueError('pulse_denominator')
        raw = bytearray()
        pre = bytearray()
        uart_refs = []
        for index, descriptor in enumerate(result['chunks']):
            if descriptor['artifactRef'] != f'receipts/{index:04d}.json':
                raise ValueError('chunk_order')
            receipt = read_json_artifact(root, descriptor)
            if receipt['raw']['artifactRef'] != f'uart/{index:04d}.raw':
                raise ValueError('uart_reference')
            uart_refs.append(receipt['raw']['artifactRef'])
            errors.extend(verify_artifacts(root, [receipt['raw']]))
            chunk = _read(root, receipt['raw']['artifactRef'])
            raw.extend(chunk)
            if receipt['phase'] == 'pre_pulse':
                pre.extend(chunk)
        actual_uart = {str(p.relative_to(root)) for p in (root / 'uart').iterdir()} if (root / 'uart').exists() else set()
        if actual_uart != set(uart_refs):
            raise ValueError('unreported_uart')
        if len(raw) != result['rawBytes'] or hashlib.sha256(raw).hexdigest() != result['rawSha256']:
            raise ValueError('uart_raw_integrity')
        if not audit_pre_pulse(bytes(pre))['allowedToPulse'] and result['explicitPulseAttempts']:
            errors.append('pulse_after_preboot_guard_failure')
        if result['successfulRun']:
            if result['failure'] is not None or not result['readerStopped']:
                errors.append('successful_result_with_failure')
            required = ('target_absent_at_arming', 'target_appearance_identity_verified',
                        'open_return', 'reader_start_intent', 'suppressed_initial_input_flush',
                        'initial_line_values', 'dtr_driver_call', 'dtr_driver_return', 'rts_driver_call', 'rts_driver_return',
                        'ready_verified', 'reader_stop_intent', 'reader_join_return',
                        'close_intent', 'close_return')
            if any(kinds.count(k) != 1 for k in required) or kinds.count('http_start') != 3 or kinds.count('http_end') != 3:
                errors.append('incomplete_lifecycle')
            elif not (kinds.index('open_return') < kinds.index('reader_start_intent')
                      < kinds.index('ready_verified') < kinds.index('http_start')
                      < max(i for i, k in enumerate(kinds) if k == 'http_end')
                      < kinds.index('reader_stop_intent') < kinds.index('close_intent')):
                errors.append('uart_lifetime_gap')
            checked = audit_registers(root / 'registers')
            if not checked['ok'] or not checked['successfulRun']:
                errors.append('register_audit_failed')
            if any(e.get('value') is not False for e in lifecycle if e['kind'] in ('rts_driver_call', 'dtr_driver_call')):
                errors.append('unexpected_control_line_command')
            absent_at = next(e['monotonic'] for e in lifecycle if e['kind'] == 'target_absent_at_arming')
            appearance_at = next(e['monotonic'] for e in lifecycle if e['kind'] == 'target_appearance_identity_verified')
            if not 0 <= appearance_at - absent_at <= POLICY['appearanceSeconds']:
                errors.append('appearance_deadline')
            open_at = next(e['monotonic'] for e in lifecycle if e['kind'] == 'open_return')
            ready_at = next(e['monotonic'] for e in lifecycle if e['kind'] == 'ready_verified')
            if not 0 <= ready_at - open_at <= POLICY['readySeconds']:
                errors.append('ready_deadline')
            http_events = [e for e in lifecycle if e['kind'] in ('http_start', 'http_end')]
            if [e['kind'] for e in http_events] != ['http_start', 'http_end'] * 3:
                errors.append('http_lifecycle_order')
            elif not 0 <= http_events[-1]['monotonic'] - http_events[0]['monotonic'] <= POLICY['registerSeconds']:
                errors.append('http_session_deadline')
            joined = next(e for e in lifecycle if e['kind'] == 'reader_join_return')
            if (joined['stopped'] is not True or joined['readerStarted'] is None or joined['readerEnded'] is None
                    or not joined['readerStarted'] <= http_events[0]['monotonic'] <= http_events[-1]['monotonic'] <= joined['readerEnded']):
                errors.append('reader_http_lifetime_gap')
            register_events, register_errors = read_events(root / 'registers')
            if register_errors or len(register_events) != 1:
                raise ValueError('register_terminal_mismatch')
            register_run = read_json_artifact(root / 'registers', register_events[0]['payload']['result'])
            for index, original in enumerate(originals):
                register_row = read_json_artifact(root / 'registers', register_run['attempts'][index]['raw'])
                original_comparable, register_comparable = dict(original), dict(register_row)
                original_comparable.pop('error', None)
                register_comparable.pop('error', None)
                if original_comparable != register_comparable:
                    errors.append('http_original_register_mismatch')
                start_event, end_event = http_events[index * 2:index * 2 + 2]
                if (start_event['nonce'] != original['nonce'] or end_event['nonce'] != original['nonce']
                        or end_event['raw'] != result['httpResponses'][index]
                        or not start_event['monotonic'] <= original['requestStartedMonotonic']
                        <= original['receivedMonotonic'] <= end_event['monotonic']):
                    errors.append('http_lifecycle_original_mismatch')
                body = _decode_json(base64.b64decode(original['responseBodyBase64'], validate=True))
                if not isinstance(body, dict) or body.get('cameraBootId') != result['ready']['bootId']:
                    errors.append('uart_http_boot_identity_mismatch')
            if len(originals) != 3:
                errors.append('http_original_denominator')
            first, second = bytes(raw).find(BOOT_MARKERS[0]), bytes(raw).find(BOOT_MARKERS[1])
            ready_match = READY.search(bytes(raw))
            if ready_match is None or not 0 <= first < second < ready_match.start():
                errors.append('incomplete_or_unordered_boot_sequence')
            found = READY.findall(bytes(raw))
            if len(found) != 1 or found[0][0].decode() != result['ready']['bootId'] or found[0][1].decode() != plan['expectedAppElfSha256']:
                errors.append('ready_raw_identity')
            if any(bytes(raw).count(m) > 1 for m in BOOT_MARKERS) or any(m in raw for m in FAULT_MARKERS):
                errors.append('raw_reboot_or_fault')
        if any(result.get(k) is not False for k in ('physicalBootCountGuaranteed', 'allIntervalLossless',
                                                   'opticalExposureVerified', 'trainingEligible', 'controlEligible')):
            errors.append('unsupported_qualification')
    except (OSError, ValueError, StorageError, KeyError, TypeError, RecursionError, IndexError, StopIteration) as exc:
        errors.append(str(exc) or type(exc).__name__)
    return dict(ok=not errors, evidenceIntegrityOk=not errors, errors=errors,
                successfulRun=not errors and result.get('successfulRun') is True,
                failure=result.get('failure'), physicalBootCountGuaranteed=False,
                opticalExposureVerified=False, trainingEligible=False, controlEligible=False)
