"""현재 수동 M4 원본의 읽기 전용 전이 감사.

기존 실행 결과를 변경하지 않는다. 영상 수치는 현재 등록 계약과 원본 픽셀에
묶인 명시적 주석이며 자동 계측 정확도나 접촉 판정을 증명하지 않는다.
장치·네트워크·촬영·학습·파일 쓰기 경로가 없다.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import re

from learning import m4_up_commissioning as core
from learning.m4_lift_data import _native
from learning.manual_collection_dataset import Evidence, audit_http, digest, require
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort, response_json

SCHEMA = 'motionbrain.manual_m4_transition_audit.v1'
REGISTRATION_SCHEMA = 'motionbrain.task_visual_state_registration.v1'
ANNOTATION_SCHEMA = 'motionbrain.task_transition_annotation.v1'
STATE = ('objectRisePx', 'toolRisePx', 'slipDyPx', 'startAngleDeg')
MANUAL_SCHEMAS = (core.SCHEMA, core.CONTACT_MANUAL650_SCHEMA, core.RETURN_SCHEMA)


def _hash(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _http(ev, receipt, pulse):
    """기존 single-RUN 감사 재사용, DOWN의 음수 출력도 부호 보존 검사."""
    events, baseline, readiness_at = [], None, None
    for i, path in enumerate(sorted(ev.root.glob('http-*-intent.json')), 1):
        name = f'http-{i:06d}'
        require(path.name == name+'-intent.json', 'task_http_sequence_gap')
        intent, response = ev.json(name+'-intent.json'), ev.json(name+'-response.json')
        parsed = response_json(response['rawUtf8'], intent['path'])
        events.append(dict(kind='http', path=intent['path'], payload=intent['payload'], response=parsed,
                           requestStartedAt=intent['startedAt'], responseReceivedAt=response['receivedAt']))
        if readiness_at is None and intent['payload'] is not None:
            readiness_at = intent['startedAt']
        if baseline is None and intent['path'].split('?')[0] == '/action_outputs':
            baseline = parsed.get('latestSequence')
    require(bool(events), 'task_http_missing')
    statuses, entries, boot = audit_http(ev, dict(events=events, outputBaselineSequence=baseline),
                                        dict(pulse=pulse, at=readiness_at), receipt)
    require(receipt.get('offVerified') is True and receipt.get('motor') == 4
            and receipt.get('requestedPulse') == pulse, 'task_receipt_changed')
    moving = [e for e in receipt['outputTrace'] if e['pwm'] != 0]
    expected = pulse['expectedPwm']
    require(moving and all(e['motor'] == 4 and 0 < e['pwm']/expected <= 1 for e in moving),
            'task_output_direction_or_ceiling')
    require(all(_number(receipt.get(k)) for k in ('dispatchBefore', 'dispatchAfter', 'offAt'))
            and receipt['dispatchBefore'] <= receipt['dispatchAfter'] < receipt['offAt'],
            'task_receipt_time_order')
    return statuses, boot, moving[0]['sequence']


def _pair(ev, label, scope, receipt, frames, bridge):
    record = ev.json(label+'-observation.json')
    require(record.get('hostClockId') == scope['hostClockId'] and record.get('context') == scope['context']
            and record.get('stable') is True, 'task_observation_context')
    name = record['evidenceFile']
    require(digest(ev.raw(name)) == record['evidenceSha256']
            and ev.json(name) == {k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')},
            'task_observation_review_changed')
    onsite = record['onsite']
    require(all(type(onsite.get(k)) is bool for k in
                ('operatorPresent','clearanceVerified','jawContactVerified','tableSupportVerified'))
            and onsite['operatorPresent'] and onsite['clearanceVerified'], 'task_onsite_review_missing')
    if label == 'before':
        require(onsite['jawContactVerified'] and onsite['tableSupportVerified'] is
                (scope['context']['supportMode'] == 'table_supported'), 'task_initial_support_review_changed')
    require(record['external']['root'] == 'external' and record['gripper']['root'] == 'gripper',
            'task_source_roots')
    external = core._external_frame(ev.root, record['external'], scope)
    gripper = core._gripper_frame(ev.root, record['gripper'], scope, label)
    require(any(f['frameId'] == external['frameId'] for f in frames), 'task_external_not_in_recording')
    # Include all selected files in immutable evidence rechecks. Native audit also
    # binds the complete recording and selected image bytes to ingress receipts.
    ingress = ev.json('external/'+record['external']['ingressFile'])
    for source in ('external/'+ingress['metadataFile'], 'external/'+ingress['imageFile'],
                   f'gripper/esp32-{label}.json', f'gripper/esp32-{label}.jpg'):
        ev.raw(source)
    for frame in (external, gripper):
        times = [frame[k] for k in ('sourceEarliest','sourceLatest','persistedAt')]
        require(all(_number(t) for t in times) and times == sorted(times)
                and bridge['validPythonBounds'][0] <= times[0] <= times[-1] <= bridge['validPythonBounds'][1],
                'task_camera_clock_bounds')
        if label == 'before':
            require(receipt['dispatchBefore']-scope['maxObservationAgeSeconds'] <= times[0]
                    and times[-1] <= receipt['dispatchBefore']-.1, 'task_input_not_available_before_run')
        else:
            require(receipt['offAt']+3 <= times[0] and times[-1] <= receipt['offAt']+6,
                    'task_target_outside_window')
    return dict(external=external, gripper=gripper, onsite=copy.deepcopy(onsite),
                cameraBinding=dict(external={k:external[k] for k in
                    ('configurationSha256','width','height','videoMirrored','videoRotationAngle')},
                    gripper=dict(bootId=scope['context']['gripperCameraBootId'],
                                 **{k:gripper[k] for k in ('width','height')})))


def _angle(statuses, start, end):
    selected = [e for e in statuses if start <= e['requestStartedAt'] <= e['responseReceivedAt'] <= end]
    require(selected, 'task_angle_window_missing')
    return SequencePort._shoulder_angle(selected[-1]['response'])


def audit_manual_m4(root):
    """검증된 현재 수동 실행 증거를 반환한다. 영상 수치는 생성하지 않는다."""
    ev = Evidence(Path(root).resolve())
    scope, result = ev.json('scope.json'), ev.json('result.json')
    require(scope.get('schemaVersion') in MANUAL_SCHEMAS, 'task_manual_m4_schema_required')
    core.validate_recorded_scope(scope)
    require(result.get('schemaVersion') == scope['schemaVersion']
            and result.get('scopeSha256') == plan_digest(scope)
            and result.get('status') == 'completed' and result.get('errors') == []
            and result.get('finalIdleOffVerified') is True and result.get('mayHaveRun') is True
            and result.get('commandOutcome') == 'receipt_recorded'
            and result.get('modelSelected') is False and result.get('trainingEligible') is False,
            'task_completed_manual_raw_required')
    require(result.get('parentModelSelected', False) is False, 'task_model_trial_not_manual_training')
    receipt = result['receipt']
    statuses, boot, first = _http(ev, receipt, scope['pulse'])
    require(boot == scope['controllerBootId'], 'task_controller_boot_changed')
    frames, bridge = _native(ev, scope, result['observerClose'])
    pairs = {label:_pair(ev, label, scope, receipt, frames, bridge) for label in ('before','after')}
    before, after = pairs['before'], pairs['after']
    require(after['external']['frameIndex'] > before['external']['frameIndex']
            and after['gripper']['capturedUs'] > before['gripper']['capturedUs'], 'task_capture_order')
    angles = dict(before=_angle(statuses, receipt['dispatchBefore']-scope['maxObservationAgeSeconds'],
                               receipt['dispatchBefore']),
                  after=_angle(statuses, receipt['offAt']+3, receipt['offAt']+6))
    require(abs(angles['before']-scope['expectedStartAngleDeg']) <= scope['startAngleToleranceDeg'],
            'task_start_angle_scope_changed')
    for event in statuses:
        if event['requestStartedAt'] < receipt['dispatchBefore']:
            continue
        delta = SequencePort._shoulder_angle(event['response'])-angles['before']
        require(abs(delta) <= scope['maxObservedDeltaDeg']
                and (delta <= .25 if scope['pulse']['direction'] == 'down' else delta >= -.25),
                'task_observed_angle_excursion')
    require(all(digest((ev.root/name).read_bytes()) == sha for name,sha in ev.hashes.items()),
            'task_source_changed_during_audit')
    return dict(schemaVersion=SCHEMA, status='raw_verified_annotation_required', sourceRoot=str(ev.root),
                sourceResultSha256=ev.hashes['result.json'], sourceProofs=copy.deepcopy(ev.hashes),
                originalContext=copy.deepcopy(scope['context']),
                command={k:scope['pulse'][k] for k in ('joint','direction','percent','durationMs')},
                supportMode=scope['context']['supportMode'], episode=[boot,str(first)],
                pairs=pairs, anglesDeg=angles, originalOutcomeChanged=False,
                trainingEligible=False, controlEligible=False, devicesAccessed=False)


def build_manual_m4_row(root, registration, annotation):
    """원본 픽셀·측정 계약·명시 주석에 묶인 개발 row. 실행 허가가 아니다."""
    report = audit_manual_m4(root)
    require(isinstance(registration, dict) and set(registration) ==
            {'schemaVersion','context','cameraBinding','acceptedSourceContexts','measurementMethod'}
            and registration['schemaVersion'] == REGISTRATION_SCHEMA
            and isinstance(registration['measurementMethod'], str) and bool(registration['measurementMethod']),
            'task_state_registration_required')
    context = registration['context']
    require(isinstance(context, dict) and set(context) == {'object','physicalView','poseRegime'}
            and all(isinstance(v,str) and bool(v) for v in context.values())
            and all(context[k] == report['originalContext'][k] for k in context),
            'task_registration_scene_changed')
    require(isinstance(registration['acceptedSourceContexts'], list)
            and report['originalContext'] in registration['acceptedSourceContexts'], 'task_source_context_not_registered')
    registration_sha = plan_digest(registration)
    require(isinstance(annotation,dict) and set(annotation) ==
            {'schemaVersion','registrationSha256','sourceResultSha256','measurementMethod','before','after'}
            and annotation['schemaVersion'] == ANNOTATION_SCHEMA
            and annotation['registrationSha256'] == registration_sha
            and annotation['sourceResultSha256'] == report['sourceResultSha256']
            and annotation['measurementMethod'] == registration['measurementMethod'], 'task_annotation_binding')
    states, identities = {}, []
    for side in ('before','after'):
        pair = report['pairs'][side]; supplied = annotation[side]
        require(pair['cameraBinding'] == registration['cameraBinding'], 'task_camera_registration_changed')
        require(isinstance(supplied,dict) and set(supplied) == {'state','external','gripper'}, 'task_annotation_side')
        for camera in ('external','gripper'):
            expected = {k:pair[camera][k] for k in ('frameId','pixelSha256')}
            require(supplied[camera] == expected, 'task_annotation_pixels_changed')
        state = supplied['state']
        require(isinstance(state,dict) and set(state) == set(STATE)
                and all(_number(v) for v in state.values())
                and state['startAngleDeg'] == report['anglesDeg'][side], 'task_annotation_state_or_angle')
        states[side] = copy.deepcopy(state)
        identities.extend(camera+':'+pair[camera]['frameId'] for camera in ('external','gripper'))
    source_hash = plan_digest(dict(sourceProofs=report['sourceProofs'], registration=registration, annotation=annotation))
    return dict(id=report['episode'][0]+':'+report['episode'][1],
                context=dict(copy.deepcopy(context), registrationSha256=registration_sha),
                supportMode=report['supportMode'], command=report['command'],
                before=states['before'], after=states['after'], sourceHash=source_hash,
                episode=report['episode'], frameIds=identities)
