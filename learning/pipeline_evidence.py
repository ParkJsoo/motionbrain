"""수집 원본의 단계별 재감사. 장치 요청·원본 수정·학습·실행 권한은 없다.

원래 실패는 그대로 보존한다. 명시적 개발 재가공만 동일한 출력/영상/시간
감사를 거쳐 전이 입력을 반환하며, 고정 후 평가 성공으로 승격하지 않는다.
"""
from __future__ import annotations
import copy
import json
from pathlib import Path
import re
import statistics

from learning import manual_collection_dataset as data
from learning.artifacts import StorageError, _decode_json
from learning.supervised_collection import PROFILES
from learning.supervised_measurement import finite, validate_execution
from learning.supervised_transport import SequencePort, response_json, validate_pulse

SCHEMA = 'motionbrain.pipeline_evidence.v1'
ERRORS = (ValueError, KeyError, TypeError, OSError, StorageError, IndexError, AttributeError)


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def _reason(exc):
    return str(exc) if isinstance(exc, (ValueError, StorageError)) else type(exc).__name__


def _processor():
    names = ('pipeline_evidence.py', 'manual_collection_dataset.py', 'action_recorder.py',
             'supervised_measurement.py', 'supervised_transport.py', 'supervised_collection.py',
             'mac_clock_bridge.py', 'lossless_frame.py', 'artifacts.py')
    hashes = {name: data.digest(Path(__file__).with_name(name).read_bytes()) for name in names}
    encoded = json.dumps(dict(schema=SCHEMA, policy=data.POLICY, sources=hashes), sort_keys=True).encode()
    return data.digest(encoded)


def _single_command(ev, session, plan):
    commands = [e for e in session['events'] if e.get('kind') == 'command']
    results = [e for e in session['events'] if e.get('kind') == 'result']
    _require(len(commands) == len(results) == 1, 'single_complete_command_required')
    command = ev.json('command-001-intent.json'); result = ev.json('command-001-result.json')
    _require(command == commands[0] and result == results[0]
             and command['commandId'] == result['commandId'], 'command_artifact_changed')
    _require(len(list(ev.root.glob('command-*'))) == 2, 'command_artifact_count')
    pulse = validate_pulse(command['pulse']); receipt = result['receipt']
    _require(pulse['joint'] == session['joint'] == plan['joint'], 'session_command_joint_mismatch')
    sign, cap, duration = PROFILES[pulse['joint']]['directions'][pulse['direction']]
    _require(pulse['percent'] <= cap and pulse['durationMs'] <= duration
             and pulse['expectedPwm'] * sign > 0, 'profile_limit')
    _require(receipt.get('offVerified') is True and session.get('finalIdleOffVerified') is True,
             'receipt_and_terminal_off_required')
    validate_execution(dict(receipt, pulse=pulse), pulse)
    policy = command.get('monitorTimingPolicy')
    _require(receipt.get('monitorTimingPolicy') == policy, 'monitor_timing_policy_mismatch')
    if policy is not None:
        from learning.supervised_transport import validate_monitor_policy
        validate_monitor_policy(pulse, policy)
        _require(receipt.get('monitorStatusVerified') is True, 'active_monitor_status_required')
    times = [command['at'], receipt['dispatchBefore'], receipt['dispatchAfter'],
             receipt['offAt'], result['at'], session['endedAt']]
    _require(all(finite(t) for t in times) and times == sorted(times), 'command_timing')
    statuses, outputs, boot = data.audit_http(ev, session, command, receipt)
    return command, receipt, statuses, outputs, boot


def _no_run(ev, session):
    """명령 이벤트 부재만으로 미실행을 단정하지 않고 전체 HTTP/출력을 대조한다."""
    _require(not list(ev.root.glob('command-*')), 'command_artifacts_without_events')
    events = [e for e in session['events'] if e.get('kind') == 'http']
    _require(events and not any(e.get('kind') in ('http_error', 'http_retry') for e in session['events']),
             'http_failure_or_retry')
    _require(len(list(ev.root.glob('http-*'))) == len(events) * 2, 'http_evidence_count_or_retry')
    output = {}; boot = None; previous = None; statuses = []; queue = None; initial_latest = None
    for i, event in enumerate(events, 1):
        prefix = f'http-{i:06d}'
        intent = ev.json(prefix + '-intent.json'); response = ev.json(prefix + '-response.json')
        path, payload = intent['path'], intent['payload']
        parsed = response_json(response['rawUtf8'], path)
        _require(response.get('httpStatus') == 200 and parsed == response['parsed'] == event['response']
                 and event['path'] == path and event['payload'] == payload
                 and intent['method'] == ('GET' if payload is None else 'POST'), 'http_response_changed')
        times = [event['requestStartedAt'], intent['startedAt'], response['receivedAt'], event['responseReceivedAt']]
        _require(all(finite(t) for t in times) and times == sorted(times)
                 and (previous is None or previous <= times[0]), 'http_chronology')
        previous = times[-1]
        _require(payload is None or parsed.get('success') is True, 'http_command_rejected')
        _require(payload is None or (path, payload) in [('/command', {'cmd': 'stop'}),
                 ('/sequence', {'action': 'clear'}), ('/motor', {'action': 'default', 'speed': 100})],
                 'no_run_unexpected_write')
        if path == '/actions':
            if boot is None: boot = parsed.get('bootId')
            _require(parsed.get('bootId') == boot and parsed.get('active') is False, 'controller_boot_changed')
        if path in ('/status', '/collection_status'): statuses.append(event)
        if path == '/sequence' and payload is None: queue = parsed.get('sequence')
        if path.split('?')[0] == '/action_outputs':
            _require(parsed.get('bootId') == boot and parsed.get('evidence') == 'driver_write'
                     and parsed.get('clockDomain') == 'controller_boot_millis'
                     and parsed.get('gap', False) is False, 'output_boot_or_gap')
            if initial_latest is None and not parsed.get('hasMore', False):
                _require(bool(parsed['entries']), 'initial_output_snapshot_empty')
                initial_latest = max(e['sequence'] for e in parsed['entries'])
            for entry in parsed['entries']:
                n = entry['sequence']
                _require(n not in output or output[n] == entry, 'output_history_changed')
                output[n] = entry
    baseline = session['outputBaselineSequence']
    _require(type(baseline) is int and baseline > 0 and isinstance(boot, str) and len(boot) == 32,
             'controller_evidence_missing')
    _require(baseline == initial_latest, 'no_run_output_baseline_changed')
    entries = [output[k] for k in sorted(output)]
    _require(entries and all(e['pwm'] == 0 for e in entries if e['sequence'] > baseline), 'output_without_command')
    baseline_outputs = {e['motor']: e['pwm'] for e in entries if e['sequence'] <= baseline}
    _require(baseline_outputs == {i: 0 for i in range(1, 6)}, 'baseline_output_not_zero')
    port = object.__new__(SequencePort)
    port.boot_id = boot; port.last_sequence = None; port._known_entries = {}; port._last_output_time = None
    port.last_outputs = {}; port._active_pulse = None
    port._consume(dict(bootId=boot, evidence='driver_write', clockDomain='controller_boot_millis', entries=entries))
    _require(port.last_outputs == {i: 0 for i in range(1, 6)} and statuses
             and queue and queue.get('state') == 'IDLE' and queue.get('totalCount') == 0
             and session.get('finalIdleOffVerified') is True, 'no_run_final_off_not_proven')
    port._check_status(statuses[-1]['response'], 'IDLE')
    return boot


def ingest_session(root, expected_sha256, *, allow_failed_development=False):
    """불변 원본을 읽어 실행·녹화·영상 창·센서 감사를 독립적으로 반환한다.

    source 해시 불일치는 입구에서 예외로 거절한다. 나머지 실패는 해당 단계의
    rejected/unknown 상태로 남겨 다른 증거 감사를 막지 않는다.
    """
    _require(type(allow_failed_development) is bool, 'explicit_recovery_boolean_required')
    _require(isinstance(expected_sha256, str) and re.fullmatch('[0-9a-f]{64}', expected_sha256),
             'source_sha256_required')
    ev = data.Evidence(Path(root)); raw = ev.raw('session.json')
    _require(data.digest(raw) == expected_sha256, 'session_hash_mismatch')
    session = _decode_json(raw)
    _require(isinstance(session, dict), 'source_session_object_required')
    processor = _processor()
    key = data.digest(json.dumps([expected_sha256, processor, allow_failed_development]).encode())
    report = dict(schemaVersion=SCHEMA, source=dict(path=str(Path(root).absolute()), sha256=expected_sha256),
        processorSha256=processor, stageKey=key,
        originalOutcome=dict(phase=session.get('phase'), error=session.get('error')),
        execution=dict(status='unknown', offVerified=False, episode=None),
        recording=dict(status='rejected'), visual=dict(status='rejected', row=None, reason=None),
        sensor=dict(status='unavailable', before=None, after=None, reason=None),
        sourceHashes=ev.hashes, recoveryDevelopmentOnly=False,
        trainingEligible=False, controlEligible=False)
    try:
        _require(session.get('schemaVersion') == 'motionbrain.manual_collection.v1'
                 and session.get('datasetEligible') is False and session.get('trainingPerformed') is False
                 and session.get('phase') in ('completed', 'failed'),
                 'original_session_contract_changed')
        plan = ev.json('plan.json')
        _require(plan.get('sessionId') == session.get('sessionId') and plan.get('joint') == session.get('joint')
                 and session.get('joint') in PROFILES, 'session_plan_identity')
    except ERRORS as exc:
        for name in ('execution', 'recording', 'visual', 'sensor'): report[name]['reason'] = _reason(exc)
        return report
    frames = identity = None
    try:
        frames, identity = data.audit_camera(ev, plan, session)
        report['recording'] = dict(status='verified', frames=len(frames), cameraIdentity=identity)
    except ERRORS as exc: report['recording']['reason'] = _reason(exc)
    commands = [e for e in session.get('events', []) if e.get('kind') == 'command']
    results = [e for e in session.get('events', []) if e.get('kind') == 'result']
    if not commands and not results:
        try:
            boot = _no_run(ev, session)
            report['execution'] = dict(status='no_run', offVerified=True, episode=None, controllerBootId=boot)
        except ERRORS as exc: report['execution']['reason'] = _reason(exc)
        report['visual']['reason'] = 'no_complete_command_transition'
        report['sensor']['reason'] = 'no_complete_command_transition'
        return report
    try:
        command, receipt, statuses, outputs, boot = _single_command(ev, session, plan)
        report['execution'] = dict(status='verified', offVerified=True,
            episode=[boot, command['commandId']], command={k: command['pulse'][k]
            for k in ('joint', 'direction', 'percent', 'durationMs')})
    except ERRORS as exc:
        report['execution']['reason'] = _reason(exc)
        report['visual']['reason'] = 'execution_not_verified'
        report['sensor']['reason'] = 'execution_not_verified'
        return report
    pre = [command['at'] + x for x in data.POLICY['preSeconds']]
    post = [receipt['offAt'] + x for x in data.POLICY['postSeconds']]
    if session['joint'] != 'shoulder':
        report['sensor'] = dict(status='not_applicable', before=None, after=None, reason=None)
    else:
        for label, window in [('before', pre), ('after', post)]:
            try: report['sensor'][label] = data.sensor_window(statuses, window)
            except ERRORS as exc:
                report['sensor']['reason'] = (report['sensor']['reason'] + ';' if report['sensor']['reason'] else '') + label + ':' + _reason(exc)
        report['sensor']['status'] = 'verified' if report['sensor']['reason'] is None else 'rejected'
    failed = session.get('phase') != 'completed' or session.get('error') is not None
    report['recoveryDevelopmentOnly'] = failed and allow_failed_development
    if failed and not allow_failed_development:
        report['visual'].update(status='deferred', reason='original_failed_source_requires_explicit_development_reprocessing')
        return report
    if frames is None:
        report['visual']['reason'] = 'recording_not_verified'
        return report
    try:
        before = data.select_frames(frames, pre, command['at']); after = data.select_frames(frames, post)
        target = min(after, key=lambda f: abs((f['sourceEarliest'] + f['sourceLatest']) / 2 - statistics.mean(post)))
        row = dict(sessionId=session['sessionId'], status='eligible',
            inputs=dict(image=data.frame_ref(before[-1]), command=copy.deepcopy(report['execution']['command'])),
            targets=dict(image=data.frame_ref(target)), evidence=dict(commandId=command['commandId'],
                commandAt=command['at'], dispatchHostBracket=[receipt['dispatchBefore'], receipt['dispatchAfter']],
                offReceiptAt=receipt['offAt'], controllerBootId=boot, actualOutputTrace=receipt['outputTrace'],
                preFrameIds=[f['frameIndex'] for f in before], postFrameIds=[f['frameIndex'] for f in after],
                auditedFrames=len(frames), cameraIdentity=identity, sourceHashes=ev.hashes,
                cameraHostBounds=[frames[0]['sourceEarliest'], session['endedAt']]),
            sourceHash=expected_sha256, recoveryDevelopmentOnly=bool(failed), originalDatasetEligible=False,
            trainingPerformed=False, controlEligible=False)
        if command.get('monitorTimingPolicy') is not None:
            row['evidence']['executionTimingPolicy'] = command['monitorTimingPolicy']
        report['visual'] = dict(status='verified', row=row, reason=None)
    except ERRORS as exc: report['visual']['reason'] = _reason(exc)
    return report
