"""Finite single-axis HTTP sequences with durable evidence and no write retries.

This transport is for attended collection, not position control. The ESP32
sequence timer limits the requested step; actual ramp-down is recorded separately.
"""
from __future__ import annotations

import copy
import json
import math
import time
import urllib.parse
import urllib.request

from learning.controller_latency import deadline
from learning.device_adapter import _NoRedirect


JOINTS = {
    'gripper': (1, {'open': 1, 'close': -1}),
    'wrist': (2, {'up': 1, 'down': -1}),
    'elbow': (3, {'up': -1, 'down': 1}),
    'shoulder': (4, {'up': 1, 'down': -1}),
    'base': (5, {'left': 1, 'right': -1}),
}


def validate_target(value):
    parsed = urllib.parse.urlsplit(value)
    if (parsed.scheme != 'http' or not parsed.hostname or parsed.username is not None
            or parsed.password is not None or parsed.path not in ('', '/')
            or parsed.query or parsed.fragment or any(c.isspace() for c in value)
            or any(c in value for c in ('\\', '{', '}', '[', ']'))):
        raise ValueError('plain_http_host_required')
    parsed.port  # Reject invalid/out-of-range ports before any device access.
    return value.rstrip('/')


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_pulse(pulse):
    if not isinstance(pulse, dict) or set(pulse) != {
            'joint', 'motor', 'direction', 'percent', 'durationMs', 'expectedPwm'}:
        raise ValueError('pulse_fields_invalid')
    joint = pulse['joint']
    if joint not in JOINTS or type(pulse['motor']) is not int:
        raise ValueError('pulse_joint_invalid')
    motor, directions = JOINTS[joint]
    if pulse['motor'] != motor or pulse['direction'] not in directions:
        raise ValueError('pulse_direction_invalid')
    if (type(pulse['percent']) is not int or not 1 <= pulse['percent'] <= 100
            or type(pulse['durationMs']) is not int or not 1 <= pulse['durationMs'] <= 1500
            or type(pulse['expectedPwm']) is not int
            or pulse['expectedPwm'] != directions[pulse['direction']] * (255 * pulse['percent'] // 100)):
        raise ValueError('pulse_output_invalid')
    if motor == 4 and pulse['direction'] == 'up':
        raise ValueError('m4_up_not_enabled')
    return copy.deepcopy(pulse)


def response_json(raw, path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                # The legacy sequence endpoint repeats root state with a
                # sequence meaning. Its nested sequence is checked separately.
                if path == '/sequence' and key == 'state':
                    result.setdefault('legacyStateValues', [result[key]]).append(value)
                else:
                    raise ValueError('duplicate_response_key')
            result[key] = value
        return result
    def invalid_constant(value):
        raise ValueError('nonfinite_response')
    value = json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid_constant)
    if not isinstance(value, dict):
        raise ValueError('response_object_required')
    return value


class SequencePort:
    """One exclusive collector; root owns journal and execution authorization.

    ``may_have_run`` changes immediately before sending RUN, not when receiving
    its response. An exception never authorizes a resend. ``finish`` performs
    bounded cleanup once and caches its result, including a failed result.
    """
    def __init__(self, url, token, boot_id, store, *, clock=time, timeout=.8):
        self.url = validate_target(url)
        if (not isinstance(token, str) or not token
                or not all(33 <= ord(c) <= 126 for c in token)):
            raise ValueError('invalid_runtime_token')
        if not isinstance(boot_id, str) or not boot_id:
            raise ValueError('expected_boot_required')
        if not finite_number(timeout) or not 0 < timeout <= 2:
            raise ValueError('bounded_timeout_required')
        self.token, self.boot_id, self.store = token, boot_id, store
        self.clock, self.timeout = clock, timeout
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
        self.serial = 0
        self.last_sequence = None
        self.last_outputs = {}
        self._known_entries = {}
        self.may_have_run = False
        self.writes_possible = False
        self.off_verified = False
        self.evidence_failed = False
        self._finished = False
        self._finish_result = False
        self._active_pulse = None
        self._trace = []
        self.last_snapshot = None
        self.last_dispatch_before = None
        self.last_dispatch_after = None
        self.last_off_at = None

    def _save(self, path, value, emergency=False):
        try:
            self.store.write_json(path, value)
        except BaseException:
            self.evidence_failed = True
            if not emergency:
                raise

    def _call(self, path, payload=None, *, emergency=False, run=False):
        if path not in ('/status', '/sequence', '/actions', '/action_outputs', '/command', '/motor'):
            raise ValueError('unsupported_endpoint')
        self.serial += 1
        name = f'http-{self.serial:06d}'
        event = {'path': path, 'method': 'GET' if payload is None else 'POST',
                 'payload': payload, 'startedAt': self.clock.monotonic()}
        # No token/header is ever persisted. Intent must be durable before any
        # normal write; emergency cleanup proceeds even when storage is broken.
        self._save(name + '-intent.json', event, emergency)
        body = None if payload is None else urllib.parse.urlencode(payload).encode('ascii')
        headers = {'Connection': 'close'}
        if payload is not None:
            headers.update({'X-MotionBrain': '1', 'X-MotionBrain-Token': self.token,
                            'Content-Type': 'application/x-www-form-urlencoded'})
        request = urllib.request.Request(self.url + path, data=body, headers=headers,
                                         method=event['method'])
        try:
            with deadline(self.timeout):
                if run:
                    self.last_dispatch_before = self.clock.monotonic()
                    self.may_have_run = True
                with self.opener.open(request, timeout=self.timeout) as response:
                    raw = response.read(65537)
                    code = response.status
                if run:
                    self.last_dispatch_after = self.clock.monotonic()
            if len(raw) > 65536 or code != 200:
                raise ValueError('invalid_http_response')
            parsed = response_json(raw, path)
            self._save(name + '-response.json', {'httpStatus': code,
                'receivedAt': self.clock.monotonic(), 'rawUtf8': raw.decode('utf-8'),
                'parsed': parsed}, emergency)
            if payload is not None and parsed.get('success') is not True:
                raise ValueError('command_rejected')
            return parsed
        except BaseException as exc:
            # Error text may originate outside this module; retain type only.
            self._save(name + '-error.json', {'error': type(exc).__name__,
                       'endedAt': self.clock.monotonic()}, True)
            raise

    def _check_status(self, status, expected, pulse=None, baseline_angle=None):
        if status.get('state') != expected:
            raise ValueError('controller_state_changed')
        sensor, teleop = status.get('sensor', {}), status.get('teleop', {})
        if (sensor.get('connected') is not True or sensor.get('imuOk') is not True
                or any(sensor.get(k) is not False for k in ('simulated', 'faultLatched', 'blocked'))
                or not finite_number(sensor.get('lastUpdateMs'))
                or not 0 <= sensor['lastUpdateMs'] <= 150
                or teleop.get('deadman') is not False or teleop.get('controlActive') is not False
                or status.get('baseAngle', {}).get('active') is not False
                or status.get('shoulderAngle', {}).get('active') is not False):
            raise ValueError('controller_safety_not_ready')
        motors = status.get('motors', {})
        if not isinstance(motors, dict) or set(motors) != {f'M{i}' for i in range(1, 6)}:
            raise ValueError('motor_status_invalid')
        for i in range(1, 6):
            motor = motors[f'M{i}'];speed = motor.get('speed')
            if type(speed) is not int:
                raise ValueError('motor_speed_invalid')
            if pulse is not None and i == pulse['motor']:
                if speed * pulse['expectedPwm'] < 0 or abs(speed) > abs(pulse['expectedPwm']):
                    raise ValueError('unexpected_motor_speed')
            elif speed != 0 or motor.get('enabled') is not False:
                raise ValueError('unexpected_motor_output')
        if pulse is None and status.get('motorEnabled') is not False:
            raise ValueError('outputs_not_off')
        if pulse is not None and pulse['motor'] == 4:
            angle = self._shoulder_angle(status)
            if baseline_angle is None or abs(angle - baseline_angle) > 3:
                raise ValueError('m4_observed_change_exceeded')

    @staticmethod
    def _shoulder_angle(status):
        shoulder = status.get('shoulderAngle', {})
        if (any(shoulder.get(k) is not True for k in
                ('available', 'sensorConnected', 'sensorFresh', 'sensorReady'))
                or shoulder.get('manualCalibrationUnlock') is not False
                or shoulder.get('manualGuardBlocked') is not False
                or not finite_number(shoulder.get('ageMs')) or not 0 <= shoulder['ageMs'] <= 150
                or not finite_number(shoulder.get('angleDeg'))
                or not finite_number(shoulder.get('manualDownBoundaryDeg'))
                or not finite_number(shoulder.get('manualUpBoundaryDeg'))
                or not shoulder['manualDownBoundaryDeg'] + 4 < shoulder['angleDeg'] < shoulder['manualUpBoundaryDeg'] - 4):
            raise ValueError('m4_feedback_not_ready')
        return shoulder['angleDeg']

    def _consume(self, payload, *, quiet=False):
        if (payload.get('bootId') != self.boot_id or payload.get('evidence') != 'driver_write'
                or payload.get('clockDomain') != 'controller_boot_millis'):
            raise ValueError('output_identity_changed')
        entries = payload.get('entries')
        if not isinstance(entries, list) or not entries:
            raise ValueError('output_history_missing')
        previous = None
        previous_time = None
        for entry in entries:
            if (not isinstance(entry, dict) or type(entry.get('sequence')) is not int
                    or type(entry.get('motor')) is not int or not 1 <= entry['motor'] <= 5
                    or type(entry.get('pwm')) is not int or not -255 <= entry['pwm'] <= 255
                    or type(entry.get('atMs')) is not int or entry['atMs'] < 0
                    or previous is not None and entry['sequence'] != previous + 1
                    or previous_time is not None and entry['atMs'] < previous_time):
                raise ValueError('output_history_invalid')
            if entry['sequence'] in self._known_entries and self._known_entries[entry['sequence']] != entry:
                raise ValueError('output_history_changed')
            previous = entry['sequence']
            previous_time = entry['atMs']
        if self.last_sequence is not None and entries[-1]['sequence'] < self.last_sequence:
            raise ValueError('output_sequence_regressed')
        new = entries if self.last_sequence is None else [e for e in entries if e['sequence'] > self.last_sequence]
        if self.last_sequence is not None and new and new[0]['sequence'] != self.last_sequence + 1:
            raise ValueError('output_history_gap')
        if quiet and self.last_sequence is not None and any(e['pwm'] != 0 for e in new):
            raise ValueError('unexpected_quiet_output')
        for entry in new:
            if self._active_pulse is not None:
                self._trace.append(copy.deepcopy(entry))
                pulse = self._active_pulse
                if entry['pwm'] != 0 and (entry['motor'] != pulse['motor']
                        or entry['pwm'] * pulse['expectedPwm'] < 0
                        or abs(entry['pwm']) > abs(pulse['expectedPwm'])):
                    raise ValueError('unexpected_driver_output')
            self.last_outputs[entry['motor']] = entry['pwm']
        self.last_sequence = entries[-1]['sequence']
        self._known_entries = {e['sequence']: copy.deepcopy(e) for e in entries}
        return new

    def _snapshot(self, *, emergency=False):
        actions = self._call('/actions', emergency=emergency)
        if actions.get('bootId') != self.boot_id or actions.get('active') is not False:
            raise ValueError('controller_boot_or_action_changed')
        status = self._call('/status', emergency=emergency)
        self._check_status(status, 'IDLE')
        output = self._call('/action_outputs', emergency=emergency)
        self._consume(output, quiet=not emergency and self._active_pulse is None)
        if self.last_outputs != {i: 0 for i in range(1, 6)}:
            raise ValueError('driver_off_not_proven')
        snapshot = {'actions': actions, 'status': status, 'outputs': output}
        self.last_snapshot = snapshot
        self.off_verified = True
        self.last_off_at = self.clock.monotonic()
        return snapshot

    def _queue(self, expected_count, *, emergency=False):
        response = self._call('/sequence', emergency=emergency)
        sequence = response.get('sequence', {})
        if sequence.get('state') != 'IDLE' or sequence.get('totalCount') != expected_count:
            raise ValueError('single_step_queue_changed')
        return response

    def preflight(self):
        self.off_verified = False
        snapshot = self._snapshot()
        try:
            snapshot['sequence'] = self._queue(0)
        except BaseException:
            self.off_verified = False
            raise
        return snapshot

    def check_idle(self):
        """Read-only quiet-window check; caller controls the polling frequency."""
        # A prior OFF proof (including a previous finish) cannot certify a new
        # quiet window. A new failure must permit bounded emergency cleanup.
        self._finished = False
        self.off_verified = False
        status = self._call('/status')
        self._check_status(status, 'IDLE')
        self._consume(self._call('/action_outputs'), quiet=True)
        if self.last_outputs != {i: 0 for i in range(1, 6)}:
            raise ValueError('driver_off_not_proven')
        self.off_verified = True
        return status

    def _stop_and_clear(self, *, restore_default, emergency):
        errors = []
        self.off_verified = False
        # Every operation is bounded and attempted at most once. Even a lost
        # STOP response must not suppress the following physical-state reads.
        for operation in (
                lambda: self._call('/command', {'cmd': 'stop'}, emergency=emergency),
                lambda: self._snapshot(emergency=emergency),
                lambda: self._call('/sequence', {'action': 'clear'}, emergency=emergency)):
            try:
                operation()
            except BaseException as exc:
                errors.append(type(exc).__name__)
        if restore_default:
            try:
                self._call('/motor', {'action': 'default', 'speed': 100}, emergency=emergency)
            except BaseException as exc:
                errors.append(type(exc).__name__)
        try:
            self._queue(0, emergency=emergency)
        except BaseException as exc:
            errors.append(type(exc).__name__)
        return not errors and self.off_verified

    def finish(self):
        if self._finished:
            return self._finish_result
        self._finished = True
        if not self.writes_possible:
            self._finish_result = self.off_verified
        else:
            # Output validation failures must not prevent emergency OFF proof.
            self._active_pulse = None
            self._finish_result = self._stop_and_clear(restore_default=True, emergency=True)
        return self._finish_result

    def execute(self, pulse, tick):
        self.may_have_run = False
        self.last_dispatch_before = None
        self.last_dispatch_after = None
        self._finished = False
        self._finish_result = False
        pulse = validate_pulse(pulse)
        self._active_pulse = None
        self._trace = []
        try:
            tick()
            snapshot = self.preflight()
            baseline_angle = self._shoulder_angle(snapshot['status']) if pulse['motor'] == 4 else None
            self.writes_possible = True
            self._call('/motor', {'action': 'default', 'speed': 255})
            self._call('/sequence', {'action': 'add', 'joint': pulse['joint'],
                'direction': pulse['direction'], 'speed': pulse['percent'], 'duration': pulse['durationMs']})
            self._queue(1)
            tick()
            self.off_verified = False
            self._call('/command', {'cmd': 'arm'})
            armed_status = self._call('/status')
            self._check_status(armed_status, 'ARMED')
            if pulse['motor'] == 4 and abs(self._shoulder_angle(armed_status) - baseline_angle) > 3:
                raise ValueError('m4_observed_change_before_run')
            new = self._consume(self._call('/action_outputs'))
            if any(e['pwm'] != 0 for e in new):
                raise ValueError('unexpected_output_before_run')
            self._active_pulse = pulse
            tick()
            self._call('/sequence', {'action': 'run'}, run=True)
            before = self.last_dispatch_before
            after = self.last_dispatch_after
            # Poll outputs first after RUN so a slow status response cannot
            # needlessly age the 128-entry output ring before its first drain.
            while True:
                tick()
                self._consume(self._call('/action_outputs'))
                self._check_status(self._call('/status'), 'ARMED', pulse, baseline_angle)
                # RUN may spend part of its round trip before the controller
                # starts its timer. Waiting from the response upper bound
                # avoids cutting that timer short because of network latency.
                remaining = after + pulse['durationMs'] / 1000 - self.clock.monotonic()
                if remaining <= 0:
                    break
                self.clock.sleep(min(.05, remaining))
            self._finish_result = self._stop_and_clear(restore_default=True, emergency=True)
            if not self._finish_result or self.evidence_failed:
                # Do not resend a failed cleanup write as an implicit retry.
                self._finished = True
                raise ValueError('pulse_cleanup_failed')
            off = self.last_off_at
            trace = copy.deepcopy(self._trace)
            if not any(e['motor'] == pulse['motor'] and e['pwm'] != 0 for e in trace):
                raise ValueError('pulse_output_not_observed')
            if not any(e['motor'] == pulse['motor'] and e['pwm'] == 0 for e in trace):
                raise ValueError('pulse_off_not_observed')
            self._active_pulse = None
            return {'dispatchBefore': before, 'dispatchAfter': after, 'offAt': off,
                    'outputTrace': trace, 'offVerified': True, 'motor': pulse['motor'],
                    'requestedPulse': pulse}
        except BaseException:
            self.finish()
            raise
