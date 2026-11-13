"""Finite single-axis HTTP sequences with durable evidence and no write retries.

This transport is for attended collection, not position control. The ESP32
sequence timer limits the requested step; actual ramp-down is recorded separately.
"""
from __future__ import annotations

import copy
import http.client
import json
import math
import time
import urllib.parse
import urllib.request

from learning.controller_latency import deadline
from learning.device_adapter import _NoRedirect
from learning.http_phase_trace import HttpPhaseTrace


JOINTS = {
    'gripper': (1, {'open': 1, 'close': -1}),
    'wrist': (2, {'up': 1, 'down': -1}),
    'elbow': (3, {'up': -1, 'down': 1}),
    'shoulder': (4, {'up': 1, 'down': -1}),
    'base': (5, {'left': 1, 'right': -1}),
}

WRIST_DEADLINE_POLICY = 'wrist-deadline-550-v1'
ELBOW_DEADLINE_POLICY = 'elbow-deadline-350-v1'
ELBOW_PLATEAU_POLICY = 'elbow-plateau-650-v1'
ELBOW_FINE_POLICY = 'elbow-fine-450-v1'
WRIST_READ_BUDGET_REVISION = 'shared-command-deadline-v2'
QUIET_IDLE_POLL_SECONDS = .5


def check_quiet_idle(port):
    """비구동 대기에서만 기존 OFF 근거·공유 기한을 사용하는 제한된 GET 확인."""
    return port.check_idle(allow_quiet_disconnect_retry=True, allow_quiet_monitor_timeout=True,
                           allow_quiet_read_recovery=True)


def validate_monitor_policy(pulse, policy):
    if policy is None:
        return
    wrist = (policy == WRIST_DEADLINE_POLICY and pulse['joint'] == 'wrist'
             and pulse['durationMs'] == 550)
    elbow_profiles = {ELBOW_DEADLINE_POLICY: (60, 350), ELBOW_PLATEAU_POLICY: (35, 650),
                      ELBOW_FINE_POLICY: (60, 450)}
    elbow = (isinstance(policy, str) and policy in elbow_profiles and pulse['joint'] == 'elbow'
             and pulse['direction'] in ('up', 'down')
             and type(pulse['percent']) is int and type(pulse['durationMs']) is int
             and (pulse['percent'], pulse['durationMs']) == elbow_profiles[policy])
    if not (wrist or elbow):
        raise ValueError('unsupported_monitor_timing_policy')


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
    class ObjectPairs(list):
        pass

    def normalize(value, *, root=False):
        if not isinstance(value, ObjectPairs):
            if isinstance(value, list):
                return [normalize(item) for item in value]
            return value
        result = {}
        repeated_state = False
        for key, item in value:
            # This field is parser evidence, never controller-provided data.
            if key == 'legacyStateValues':
                raise ValueError('reserved_response_key')
            item = normalize(item)
            if key in result:
                # Firmware command replies repeat the same system state;
                # sequence replies append a separate sequence state. Allow
                # exactly these two root fields, never nested or conflicting
                # command states. Keep both values as audit evidence.
                if (not root or key != 'state' or repeated_state
                        or not isinstance(result[key], str) or not isinstance(item, str)
                        or path not in ('/command', '/sequence')
                        or (path == '/command' and result[key] != item)):
                    raise ValueError('duplicate_response_key')
                result['legacyStateValues'] = [result[key], item]
                repeated_state = True
            result[key] = item
        return result

    def invalid_constant(value):
        raise ValueError('nonfinite_response')
    value = json.loads(raw, object_pairs_hook=ObjectPairs, parse_constant=invalid_constant)
    if not isinstance(value, ObjectPairs):
        raise ValueError('response_object_required')
    return normalize(value, root=True)


class SequencePort:
    """One exclusive collector; root owns journal and execution authorization.

    ``may_have_run`` changes immediately before sending RUN, not when receiving
    its response. An exception never authorizes a resend. ``finish`` performs
    bounded cleanup once and caches its result, including a failed result.
    """
    def __init__(self, url, token, boot_id, store, *, clock=time, timeout=.8, compact_reads=False):
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
        self.compact_reads = compact_reads
        self._last_output_time = None
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
        self._default_opener = self.opener
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
        self._quiet_retry_off_proof = False
        self._quiet_read_recovery_used = False
        self._last_http_failure_serial = None
        self._before_run = None

    def _save(self, path, value, emergency=False):
        try:
            self.store.write_json(path, value)
        except BaseException:
            self.evidence_failed = True
            if not emergency:
                raise

    def _call(self, path, payload=None, *, emergency=False, run=False, quiet_disconnect_retry=False, request_timeout=None, quiet_monitor_timeout=False, initial_output_read=False, read_deadline=None):
        self._last_http_failure_serial = None
        endpoint = urllib.parse.urlsplit(path).path
        if read_deadline is not None and (not finite_number(read_deadline)
                or payload is not None or emergency or run or quiet_disconnect_retry
                or endpoint not in ('/status', '/collection_status', '/action_outputs')):
            raise ValueError('readonly_monitor_deadline_required')
        if endpoint not in ('/status', '/collection_status', '/sequence', '/actions', '/action_outputs', '/command', '/motor'):
            raise ValueError('unsupported_endpoint')
        if '?' in path and (endpoint != '/action_outputs' or payload is not None):
            raise ValueError('unsupported_endpoint')
        if type(quiet_monitor_timeout) is not bool:
            raise ValueError('quiet_monitor_timeout_invalid')
        if type(initial_output_read) is not bool:
            raise ValueError('initial_output_read_invalid')
        if initial_output_read and not (self.compact_reads and endpoint == '/action_outputs'
                and payload is None and request_timeout is not None
                and not emergency and not run and not quiet_disconnect_retry and not quiet_monitor_timeout
                and self._active_pulse is None and not self.may_have_run and not self.writes_possible
                and not self.evidence_failed and not self.off_verified and self.last_snapshot is None):
            raise ValueError('initial_output_read_scope')
        if quiet_monitor_timeout and not (self._quiet_retry_off_proof and self._active_pulse is None
                and not self.evidence_failed and endpoint in ('/status', '/collection_status', '/action_outputs')
                and payload is None and not emergency and not run
                ):
            raise ValueError('quiet_monitor_timeout_scope')
        maximum = max(self.timeout, 1.5) if quiet_monitor_timeout or initial_output_read else self.timeout
        budget = maximum if request_timeout is None else min(maximum, request_timeout)
        if read_deadline is not None:
            budget = min(budget, read_deadline-self.clock.monotonic())
        if budget <= 0:
            raise TimeoutError('request_deadline')
        self.serial += 1
        name = f'http-{self.serial:06d}'
        event = {'path': path, 'method': 'GET' if payload is None else 'POST',
                 'payload': payload, 'startedAt': self.clock.monotonic(), 'timeoutSeconds': budget}
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
        # 최초 비구동 부팅 조회에만 적용한다. 주입한 fake/custom opener와 이후
        # 일반 조회·구동·quiet GET 재연결의 동작은 그대로 둔다.
        phase_trace = None
        request_opener = self.opener
        if (self.serial == 1 and path == '/actions' and payload is None and not run
                and not emergency and self._active_pulse is None and not self.may_have_run
                and not self.writes_possible and self.opener is self._default_opener):
            phase_trace = HttpPhaseTrace(clock=self.clock)
            request_opener = urllib.request.build_opener(
                urllib.request.ProxyHandler({}), _NoRedirect(), phase_trace.handler())
        phase_started = self.clock.monotonic() if phase_trace is not None else None
        call_failed = False
        try:
            # Durable intent writes consume the same absolute read budget.
            # Never open a new socket after the monitor's stop deadline.
            if read_deadline is not None:
                budget = min(budget, read_deadline-self.clock.monotonic())
                if budget <= 0:
                    raise TimeoutError('request_deadline')
            with deadline(budget):
                started = self.clock.monotonic()
                for attempt in range(2):
                    remaining = budget - (self.clock.monotonic() - started)
                    if read_deadline is not None:
                        remaining = min(remaining, read_deadline-self.clock.monotonic())
                    if remaining <= 0:
                        raise TimeoutError('request_deadline')
                    try:
                        if run:
                            # Selection must remain fresh after durable intent
                            # storage. Consume the guard even if it rejects;
                            # neither a failed guard nor an ambiguous RUN may
                            # authorize a second dispatch.
                            before_run, self._before_run = self._before_run, None
                            if before_run is not None:
                                before_run()
                            remaining = budget - (self.clock.monotonic() - started)
                            if remaining <= 0:
                                raise TimeoutError('request_deadline')
                            self.last_dispatch_before = self.clock.monotonic()
                            self.may_have_run = True
                        with request_opener.open(request, timeout=remaining) as response:
                            raw = response.read(65537)
                            code = response.status
                        break
                    except http.client.RemoteDisconnected:
                        # Both permitted read recoveries share this request's
                        # alarm/budget. Quiet recovery is scoped to an explicitly
                        # opted-in check_idle with a prior OFF proof; that proof
                        # never substitutes for validating the fresh responses.
                        queue_read = (path == '/sequence' and not self.may_have_run
                                      and self.off_verified)
                        quiet_read = (quiet_disconnect_retry and self._quiet_retry_off_proof
                                      and endpoint in ('/status', '/collection_status', '/action_outputs')
                                      and self._active_pulse is None)
                        if (attempt or payload is not None or run or emergency
                                or self.evidence_failed or not (queue_read or quiet_read)):
                            raise
                        remaining = budget - (self.clock.monotonic() - started)
                        if remaining <= 0:
                            raise
                        self._save(name + '-attempt-1-error.json', {
                            'error': 'RemoteDisconnected', 'endedAt': self.clock.monotonic()})
                        self._save(name + '-attempt-2-intent.json', {
                            **event, 'startedAt': self.clock.monotonic(),
                            'attempt': 2, 'sharedTimeoutSeconds': budget})
                if run:
                    self.last_dispatch_after = self.clock.monotonic()
            if len(raw) > 65536 or code != 200:
                raise ValueError('invalid_http_response')
            parsed = response_json(raw, path)
            self._save(name + '-response.json', {'httpStatus': code,
                'receivedAt': self.clock.monotonic(), 'rawUtf8': raw.decode('utf-8'),
                'parsed': parsed}, emergency)
            if read_deadline is not None and self.clock.monotonic() >= read_deadline:
                raise TimeoutError('request_deadline')
            if payload is not None and parsed.get('success') is not True:
                raise ValueError('command_rejected')
            return parsed
        except BaseException as exc:
            call_failed = True
            # Error text may originate outside this module; retain type only.
            self._save(name + '-error.json', {'error': type(exc).__name__,
                       'endedAt': self.clock.monotonic()}, True)
            self._last_http_failure_serial = self.serial
            raise
        finally:
            if phase_trace is not None:
                try:
                    self._save(name + '-phase.json', {
                        'phase': phase_trace.phase, 'events': phase_trace.events,
                        'elapsedSeconds': self.clock.monotonic()-phase_started})
                except BaseException:
                    self.evidence_failed = True
                    self._last_http_failure_serial = self.serial
                    # 이미 발생한 기한 초과·중단·전송 실패를 저장 오류로 덮지 않는다.
                    # 성공한 조회의 계측 저장 실패는 호출 실패로 전파해 ARM을 막는다.
                    if not call_failed:
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
        if new and self._last_output_time is not None and new[0]['atMs'] < self._last_output_time:
            raise ValueError('output_time_regressed')
        for entry in new:
            if self._active_pulse is not None:
                self._trace.append(copy.deepcopy(entry))
                pulse = self._active_pulse
                if entry['pwm'] != 0 and (entry['motor'] != pulse['motor']
                        or entry['pwm'] * pulse['expectedPwm'] < 0
                        or abs(entry['pwm']) > abs(pulse['expectedPwm'])):
                    raise ValueError('unexpected_driver_output')
            self.last_outputs[entry['motor']] = entry['pwm']
        self._last_output_time = entries[-1]['atMs']
        self.last_sequence = entries[-1]['sequence']
        self._known_entries = {e['sequence']: copy.deepcopy(e) for e in entries}
        return new

    def _status(self, **kwargs):
        status = self._call('/collection_status' if self.compact_reads else '/status', **kwargs)
        if self.compact_reads and status.get('schemaVersion') != 'collection.v1':
            raise ValueError('collection_status_schema_invalid')
        return status

    def _outputs(self, *, quiet=False, deadline=None, **kwargs):
        if not self.compact_reads:
            payload = self._call('/action_outputs', request_timeout=None if deadline is None else deadline-self.clock.monotonic(), **kwargs)
            return payload, self._consume(payload, quiet=quiet)
        started = self.clock.monotonic()
        bootstrap = self.last_sequence is None
        # Initial retained-ring loading occurs before any collector write. It
        # may need sixteen small responses. Only this read-only bootstrap allows
        # a 1.5s page, within an 8s total. Later drains keep their budget.
        # Explicit quiet monitoring may use the existing 1.5s status budget.
        bootstrap_budget = (bootstrap and self._active_pulse is None
                            and not self.may_have_run and not self.writes_possible
                            and not kwargs.get('emergency') and not self.evidence_failed
                            and not self.off_verified and self.last_snapshot is None)
        extended_quiet = kwargs.get('quiet_monitor_timeout') is True
        drain_budget = 8. if bootstrap_budget else (max(self.timeout, 1.5) if extended_quiet else self.timeout)
        if deadline is not None:
            drain_budget = min(drain_budget, deadline-self.clock.monotonic())
        new = []
        pages = []
        # A full ring fits in sixteen pages. Both elapsed time and page count
        # are bounded, including the read-only initial baseline.
        for _ in range(16):
            cursor = self.last_sequence or 0
            payload = self._call(f'/action_outputs?after={cursor}&limit=8',
                                 request_timeout=drain_budget-(self.clock.monotonic()-started),
                                 initial_output_read=bootstrap_budget, **kwargs)
            if self.clock.monotonic() - started >= drain_budget:
                raise TimeoutError('request_deadline')
            if (payload.get('bootId') != self.boot_id or payload.get('evidence') != 'driver_write'
                    or payload.get('clockDomain') != 'controller_boot_millis'):
                raise ValueError('output_identity_changed')
            oldest, latest = payload.get('oldestSequence'), payload.get('latestSequence')
            entries = payload.get('entries')
            if (type(oldest) is not int or type(latest) is not int or not 1 <= oldest <= latest
                    or payload.get('gap') is not False or type(payload.get('hasMore')) is not bool
                    or not isinstance(entries, list) or len(entries) > 8 or latest < cursor):
                raise ValueError('output_page_invalid_or_gap')
            if entries:
                if not all(isinstance(entry, dict) for entry in entries):
                    raise ValueError('output_history_invalid')
                expected = oldest if cursor == 0 else cursor + 1
                if (entries[0].get('sequence') != expected
                        or type(entries[-1].get('sequence')) is not int
                        or entries[-1]['sequence'] > latest
                        or payload['hasMore'] != (entries[-1]['sequence'] < latest)):
                    raise ValueError('output_page_continuity_invalid')
                new.extend(self._consume(payload, quiet=quiet and not bootstrap))
            elif cursor != latest or payload['hasMore']:
                raise ValueError('output_page_empty_before_end')
            pages.append(payload)
            if not payload['hasMore']:
                return {**payload, 'entries': [e for page in pages for e in page['entries']]}, new
        raise ValueError('output_page_budget_exceeded')

    def _snapshot(self, *, emergency=False):
        actions = self._call('/actions', emergency=emergency)
        if actions.get('bootId') != self.boot_id or actions.get('active') is not False:
            raise ValueError('controller_boot_or_action_changed')
        status = self._status(emergency=emergency)
        self._check_status(status, 'IDLE')
        output, _ = self._outputs(emergency=emergency, quiet=not emergency and self._active_pulse is None)
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

    def check_idle(self, *, allow_quiet_disconnect_retry=False, allow_quiet_monitor_timeout=False,
                   allow_quiet_read_recovery=False):
        """Quiet monitor only: one fresh GET verification after a transient read failure.

        Recovery is limited to one per port/session and 4.5s total per check.
        Prior OFF enables recovery, but never substitutes for current evidence.
        No writes, active pulse, finalization, or malformed evidence may recover.
        """
        if type(allow_quiet_monitor_timeout) is not bool or type(allow_quiet_read_recovery) is not bool:
            raise ValueError('quiet_monitor_timeout_invalid')
        if allow_quiet_monitor_timeout and allow_quiet_disconnect_retry is not True:
            raise ValueError('quiet_monitor_timeout_scope')
        if allow_quiet_read_recovery and not (allow_quiet_monitor_timeout and allow_quiet_disconnect_retry):
            raise ValueError('quiet_read_recovery_scope')
        self._quiet_retry_off_proof = (allow_quiet_disconnect_retry is True
                                      and self.off_verified and self._active_pulse is None
                                      and not self.evidence_failed)
        if self._finish_result:
            self._finished = False
        self.off_verified = False
        try:
            extended = allow_quiet_monitor_timeout and self._quiet_retry_off_proof
            recovery = extended and allow_quiet_read_recovery and not self._quiet_read_recovery_used
            deadline = self.clock.monotonic()+4.5 if recovery else None
            for attempt in range(2 if recovery else 1):
                try:
                    status = self._status(quiet_disconnect_retry=self._quiet_retry_off_proof and not extended,
                        quiet_monitor_timeout=extended,
                        **({'request_timeout':deadline-self.clock.monotonic()} if deadline is not None else {}))
                    self._check_status(status, 'IDLE')
                    self._outputs(quiet=True, quiet_disconnect_retry=self._quiet_retry_off_proof and not extended,
                                  quiet_monitor_timeout=extended, deadline=deadline)
                    if self.last_outputs != {i: 0 for i in range(1, 6)}:
                        raise ValueError('driver_off_not_proven')
                    self.off_verified = True
                    return status
                except (TimeoutError, http.client.RemoteDisconnected, urllib.error.URLError) as exc:
                    transient = isinstance(exc,(TimeoutError,http.client.RemoteDisconnected)) or (
                        isinstance(exc,urllib.error.URLError) and isinstance(exc.reason,TimeoutError))
                    if (not recovery or attempt or not transient or self.evidence_failed or
                            self._last_http_failure_serial != self.serial or
                            self._active_pulse is not None or self.clock.monotonic() >= deadline):
                        raise
                    self._quiet_read_recovery_used = True
                    self._save(f'quiet-monitor-{self.serial:06d}-recovery-intent.json',dict(
                        reason=type(exc).__name__,failedRequestSerial=self.serial,
                        startedAt=self.clock.monotonic(),deadline=deadline,
                        lastVerifiedSequence=self.last_sequence,readOnly=True,
                        offVerified=False,maxRecoveriesPerSession=1))
        finally:
            self._quiet_retry_off_proof = False

    def _stop_and_clear(self, *, restore_default, emergency):
        errors = []
        self.off_verified = False
        # Every operation is bounded and attempted at most once. Even a lost
        # STOP response must not suppress the following physical-state reads.
        try:
            self._call('/command', {'cmd': 'stop'}, emergency=emergency)
        except BaseException as exc:
            errors.append(type(exc).__name__)
        original_timeout = self.timeout
        try:
            # STOP is already attempted at its original deadline. Subsequent
            # state proof and non-motion cleanup get a separate bounded wait,
            # without retrying a lost response or extending active monitoring.
            self.timeout = 2.
            for operation in (
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
        finally:
            self.timeout = original_timeout
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

    def execute(self, pulse, tick, *, before_run=None, after_armed_before_run=None, monitor_policy=None):
        """Execute once; an optional guard may reject just before RUN is sent.

        The guard runs after the RUN intent is durable and inside its request
        deadline. It must raise to reject; no return value grants authority or
        replaces the caller's own authorization and observation checks.

        after_armed_before_run runs once after ARMED/OFF preparation, outside
        the RUN request deadline and before its durable intent. It may acquire
        fresh observations or reject; before_run remains the nonblocking guard.
        """
        prepare = ({} if after_armed_before_run is None else
                   {'after_armed_before_run': after_armed_before_run})
        return self._execute_validated(validate_pulse(pulse), tick, before_run=before_run,
                                       monitor_policy=monitor_policy, **prepare)

    def _monitor_wrist_deadline(self, end, tick):
        verified = False
        while self.clock.monotonic() < end:
            tick()
            remaining = end-self.clock.monotonic()
            if remaining <= 0:
                break
            # After one complete safety check, reserve the tail for local
            # camera checks and STOP rather than another blocking HTTP read.
            if verified and remaining <= .2:
                self.clock.sleep(min(.02, remaining))
                continue
            # Reads share the command's absolute deadline. A separate short
            # timeout can abort a healthy action even though STOP is not due.
            read_end = end
            self._outputs(deadline=read_end, read_deadline=read_end)
            remaining = end-self.clock.monotonic()
            if remaining <= 0:
                break
            if verified and remaining <= .2:
                continue
            read_end = end
            status = self._status(request_timeout=read_end-self.clock.monotonic(),
                                  read_deadline=read_end)
            self._check_status(status, 'ARMED', self._active_pulse, None)
            verified = True
            self.clock.sleep(min(.05, max(0., end-self.clock.monotonic())))
        if not verified:
            raise ValueError('active_safety_status_not_verified')
        if self.clock.monotonic()-end > .02:
            raise TimeoutError('monitor_deadline_overrun')

    def _execute_validated(self, pulse, tick, *, before_run=None, after_armed_before_run=None,
                           stop_when=None, monitor_policy=None):
        """Shared finite body for internally validated, narrowly scoped adapters.

        General callers must use execute(). The commissioning adapter validates
        an exact, consumed scope before entering here; this is not a live API.
        """
        if before_run is not None and not callable(before_run):
            raise ValueError('before_run_callback_required')
        if after_armed_before_run is not None and not callable(after_armed_before_run):
            raise ValueError('after_armed_before_run_callback_required')
        if stop_when is not None and not callable(stop_when):
            raise ValueError('stop_when_callback_required')
        validate_monitor_policy(pulse, monitor_policy)
        if monitor_policy is not None and stop_when is not None:
            raise ValueError('monitor_policy_does_not_support_return_goal')
        self.may_have_run = False
        self.last_dispatch_before = None
        self.last_dispatch_after = None
        self._finished = False
        self._finish_result = False
        self._active_pulse = None
        self._trace = []
        self._before_run = before_run
        stopped_early = False
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
            armed_status = self._status()
            self._check_status(armed_status, 'ARMED')
            if pulse['motor'] == 4 and abs(self._shoulder_angle(armed_status) - baseline_angle) > 3:
                raise ValueError('m4_observed_change_before_run')
            _, new = self._outputs()
            if any(e['pwm'] != 0 for e in new):
                raise ValueError('unexpected_output_before_run')
            self._active_pulse = pulse
            tick()
            if after_armed_before_run is not None:
                after_armed_before_run()
                tick()
            self._call('/sequence', {'action': 'run'}, run=True)
            before = self.last_dispatch_before
            after = self.last_dispatch_after
            # Poll outputs first after RUN so a slow status response cannot
            # needlessly age the 128-entry output ring before its first drain.
            if monitor_policy is not None:
                self._monitor_wrist_deadline(after+pulse['durationMs']/1000, tick)
            while monitor_policy is None:
                tick()
                self._outputs()
                status = self._status()
                self._check_status(status, 'ARMED', pulse, baseline_angle)
                if stop_when is not None:
                    stop = stop_when(copy.deepcopy(status))
                    if type(stop) is not bool:
                        raise ValueError('stop_when_boolean_required')
                    if stop:
                        stopped_early = self.clock.monotonic() < after + pulse['durationMs'] / 1000
                        break
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
            receipt = {'dispatchBefore': before, 'dispatchAfter': after, 'offAt': off,
                    'outputTrace': trace, 'offVerified': True, 'motor': pulse['motor'],
                    'requestedPulse': pulse}
            if monitor_policy is not None:
                receipt.update(monitorTimingPolicy=monitor_policy, monitorStatusVerified=True,
                               monitorReadBudgetRevision=WRIST_READ_BUDGET_REVISION)
            if stop_when is not None:
                receipt.update(stoppedEarly=stopped_early,
                               stopReason='observed_return_goal' if stopped_early else 'requested_period_complete')
            return receipt
        except BaseException:
            self.finish()
            raise
        finally:
            self._before_run = None
