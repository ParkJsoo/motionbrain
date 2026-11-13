"""같은 촬영의 고정 ROI 기준과 supervised 명령·영상·센서 사후 감사.

영상 관측은 영상 평면의 변화이며 관절 영점·실물 각도·동기화 보정이 아니다.
이 모듈은 장치·네트워크·인증정보를 사용하거나 원본을 쓰지 않는다.
"""
from __future__ import annotations

import copy
import hashlib
import math
from pathlib import Path
import re

import numpy as np

from learning.arm_motion_comparison import _native, pinned_frame, nearest_sensor, sensor_delta
from learning.arm_motion_observation import observe_pair, _mask
from learning.artifacts import _decode_json, _read
from learning.manual_collection_dataset import Evidence, audit_camera, audit_http
from learning.supervised_collection import validate_plan, plan_digest
from learning.supervised_measurement import validate_execution
from learning.supervised_transport import SequencePort, response_json
from learning.http_phase_trace import http_inventory_matches

TEMPLATE_SCHEMA = 'motionbrain.arm_reference_template.v1'
REGISTRATION_SCHEMA = 'motionbrain.arm_session_reference.v1'
RESULT_SCHEMA = 'motionbrain.supervised_arm_video_comparison.v1'

def check(value, why):
    if not value:
        raise ValueError(why)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


class PinnedEvidence(Evidence):
    def raw(self, name):
        previous = self.hashes.get(name)
        raw = super().raw(name)
        check(previous is None or self.hashes[name] == previous, 'source_changed_between_reads')
        return raw


def unchanged(ev):
    check(all(sha(_read(ev.root, name)) == digest for name, digest in ev.hashes.items()),
          'source_changed_during_audit')


def replay_port(boot):
    port = object.__new__(SequencePort)
    port.boot_id = boot
    port.last_sequence = None
    port._known_entries = {}
    port._last_output_time = None
    port.last_outputs = {}
    port._active_pulse = None
    return port


def audit_trial_http(ev, report, plan):
    check(report.get('schemaVersion') == 'motionbrain.supervised_collection.v1', 'source_schema_changed')
    check(report.get('status') == 'completed' and report.get('finalIdleOffVerified') is True
          and not report.get('errors'),
          'collection_not_completed')
    check(len(plan['pulses']) == 1 and len(report['trials']) == 1, 'single_trial_required')
    trial = report['trials'][0]
    check(trial.get('id') == plan['sessionId'] + '-001' and trial.get('status') == 'executed'
          and trial.get('offVerified') is True, 'trial_incomplete')
    check(ev.json('trial-001-result.json') == trial, 'trial_receipt_report_changed')
    command = ev.json('trial-001-intent.json')
    pulse = plan['pulses'][0]
    check(command.get('id') == trial['id'] and command.get('pulse') == pulse
          and finite(command.get('at')), 'trial_intent_changed')
    validate_execution(trial, pulse)
    check(trial.get('measurementReady') is True and finite(trial.get('postEnd'))
          and trial.get('quietVerifiedUntil') == trial['postEnd']
          and trial['postEnd'] >= trial['offAt'] + plan['postSeconds'], 'post_window_not_verified')
    intents = sorted(ev.root.glob('http-*-intent.json'))
    expected = {f'http-{i:06d}-{suffix}.json' for i in range(1, len(intents)+1)
                for suffix in ('intent', 'response')}
    check(bool(intents) and http_inventory_matches(ev, expected),
          'http_count_error_or_retry')
    events = []
    baseline = None
    port = replay_port(plan['controllerBootId'])
    last_time = -math.inf
    active = False
    collecting_execution_trace = False
    baseline_angle = None
    for ordinal, path in enumerate(intents, 1):
        check(path.name == f'http-{ordinal:06d}-intent.json', 'http_sequence_changed')
        intent = ev.json(path.name)
        response = ev.json(path.name.replace('-intent', '-response'))
        parsed = response_json(response['rawUtf8'], intent['path'])
        check(response['httpStatus'] == 200 and parsed == response['parsed'], 'http_response_changed')
        check(all(finite(t) for t in (intent['startedAt'], response['receivedAt']))
              and last_time <= intent['startedAt'] <= response['receivedAt'], 'http_time_changed')
        last_time = response['receivedAt']
        route, payload = intent['path'], intent['payload']
        check(intent['method'] == ('GET' if payload is None else 'POST'), 'http_method_changed')
        check(route in ('/actions', '/status', '/sequence', '/motor', '/command', '/action_outputs'),
              'unsupported_http_route_or_pagination')
        event = dict(kind='http', path=route, payload=payload, response=parsed,
                     requestStartedAt=intent['startedAt'], responseReceivedAt=response['receivedAt'])
        events.append(event)
        if payload is not None:
            check(parsed.get('success') is True, 'http_write_rejected')
        if route == '/sequence' and payload == {'action': 'run'}:
            check(not active and baseline is None and port.last_sequence is not None,
                  'repeated_run_or_missing_output_baseline')
            check(port.last_outputs == {i: 0 for i in range(1, 6)}, 'baseline_output_not_zero')
            baseline = port.last_sequence
            active = True
            collecting_execution_trace = True
        if route == '/command' and payload == {'cmd': 'stop'}:
            active = False
        if route == '/sequence' and payload == {'action': 'clear'}:
            collecting_execution_trace = False
        if route == '/actions':
            check(parsed.get('bootId') == plan['controllerBootId'] and parsed.get('active') is False,
                  'controller_boot_changed')
        if route == '/status':
            if baseline_angle is None:
                baseline_angle = port._shoulder_angle(parsed)
            port._check_status(parsed, 'ARMED' if active else parsed.get('state'),
                               pulse if active else None, baseline_angle if active else None)
            check(parsed.get('state') in ('IDLE', 'ARMED'), 'invalid_controller_state')
            port._shoulder_angle(parsed)
        if route == '/action_outputs':
            check(parsed.get('gap', False) is False, 'output_gap')
            port._consume(parsed, quiet=baseline is None or not collecting_execution_trace)
    check(baseline is not None, 'run_missing')
    # This existing audit independently checks exact write sequence, actual RUN/OFF
    # HTTP brackets, complete receipt trace, final output, status and empty queue.
    session = dict(events=events, outputBaselineSequence=baseline)
    statuses, entries, boot = audit_http(ev, session, command, trial)
    check(boot == plan['controllerBootId'], 'plan_boot_changed')
    rows = [dict(at=e['responseReceivedAt'], angleDeg=port._shoulder_angle(e['response']))
            for e in statuses]
    check(command['at'] <= trial['dispatchBefore'], 'command_after_run')
    stop_indices = [i for i, event in enumerate(events)
                    if event['path'] == '/command' and event['payload'] == {'cmd': 'stop'}]
    check(len(stop_indices) == 2 and stop_indices[1] >= 2, 'final_cleanup_stop_missing')
    final_stop = stop_indices[1]
    quiet_snapshot = events[final_stop - 2:final_stop]
    check([event['path'] for event in quiet_snapshot] == ['/status', '/action_outputs']
          and all(event['payload'] is None for event in quiet_snapshot), 'post_quiet_snapshot_missing')
    quiet_through = quiet_snapshot[-1]['responseReceivedAt']
    check(quiet_through <= trial['quietVerifiedUntil'] <= events[final_stop]['requestStartedAt'],
          'quiet_receipt_http_bracket_changed')
    nonzero = [entry for entry in trial['outputTrace'] if entry['pwm'] != 0]
    first_nonzero = nonzero[0]
    first_off = next(entry for entry in trial['outputTrace']
                     if entry['sequence'] > first_nonzero['sequence']
                     and entry['motor'] == pulse['motor'] and entry['pwm'] == 0)
    observed = dict(requestedPulse=copy.deepcopy(pulse),
                    observedNonzeroPwmRange=[min(e['pwm'] for e in nonzero), max(e['pwm'] for e in nonzero)],
                    observedPeakPwm=max((e['pwm'] for e in nonzero), key=abs),
                    firstNonzeroAtMs=first_nonzero['atMs'], firstSubsequentOffAtMs=first_off['atMs'],
                    firstNonzeroToOffMs=(first_off['atMs'] - first_nonzero['atMs']) % 2**32,
                    outputTrace=copy.deepcopy(trial['outputTrace']),
                    semantics='driver_write_observed_interval_not_requested_duration')
    return trial, rows, dict(outputBaselineSequence=baseline, controllerBootId=boot, observedExecution=observed,
                             outputEntryCount=len(entries), httpExchangeCount=len(events),
                             firstWriteStartedAt=min(e['requestStartedAt'] for e in events if e['payload'] is not None),
                             quietOutputsVerifiedThrough=quiet_through,
                             receiptMatchesReport=True, fullTraceVerified=True,
                             runOffHttpBracketsVerified=True, finalIdleOffEmptyQueueVerified=True)



def native_image(ev, camera, frame):
    frame = pinned_frame([frame], frame['frameIndex'], frame['frameId'])
    ingress = f"host-ingress/{frame['frameIndex']:04d}.json"
    source = dict(root='native-camera', ingressFile=ingress,
                  ingressSha256=sha(ev.raw('native-camera/' + ingress)),
                  clockSha256=sha(ev.raw('native-camera/host-ingress/clock-before.json')))
    scope = dict(hostClockId=camera['sessionId'], context=dict(
        externalDeviceId=camera['deviceId'], externalViewId=camera['viewId']))
    image, verified, _ = _native(ev.root, source, scope, expected=frame)
    return image, verified



def select_endpoints(frames, trial, post_seconds, *, verified_quiet_end):
    # This private comparison permits one sample period gap near the 3s target,
    # but never uses a frame beyond the actually verified quiet interval.
    target = trial['offAt'] + post_seconds
    check(finite(verified_quiet_end) and verified_quiet_end <= trial['quietVerifiedUntil'],
          'verified_quiet_bound_required')
    quiet_end = verified_quiet_end
    before = [f for f in frames if f['sourceLatest'] <= trial['dispatchBefore'] - .1]
    after = [f for f in frames if target - .5 <= f['sourceEarliest']
             <= f['sourceLatest'] <= min(quiet_end, target + .5)]
    check(bool(before) and bool(after), 'endpoint_frames_missing_in_verified_window')
    first = pinned_frame(frames, 0, frames[0]['frameId'])
    check(first in before, 'first_reference_frame_not_pre_action')
    last = min(after, key=lambda f: (abs((f['sourceEarliest'] + f['sourceLatest']) / 2 - target),
                                     -f['frameIndex']))
    proof = dict(policy='nearest_3s_target_within_0_5s_and_verified_quiet_window',
                 beforeMarginSeconds=.1, afterTargetSeconds=post_seconds,
                 afterTargetToleranceSeconds=.5, rawQuietOutputsVerifiedThrough=quiet_end,
                 quietReceiptUntil=trial['quietVerifiedUntil'],
                 afterSourceElapsedBoundsSeconds=[last['sourceEarliest'] - trial['offAt'],
                                                  last['sourceLatest'] - trial['offAt']],
                 afterSourceTargetErrorBoundsSeconds=[last['sourceEarliest'] - target,
                                                      last['sourceLatest'] - target])
    return [first, last], proof



def load_template(raw, expected_sha256=None):
    """실행 전 고정하는 원본 JSON 바이트와 픽셀 ROI를 검증한다."""
    check(isinstance(raw, bytes) and 0 < len(raw) <= 131072, 'arm_reference_template_bytes')
    check(expected_sha256 is None or sha(raw) == expected_sha256, 'arm_reference_template_hash_changed')
    template = _decode_json(raw)
    check(type(template) is dict and set(template) == {'schemaVersion', 'imageSize', 'regions'}
          and template['schemaVersion'] == TEMPLATE_SCHEMA, 'arm_reference_template_schema')
    size = template['imageSize']
    check(type(size) is list and len(size) == 2 and all(type(v) is int for v in size)
          and 32 <= size[0] <= 1920 and 32 <= size[1] <= 1080, 'arm_reference_template_dimensions')
    check(type(template['regions']) is dict and set(template['regions']) == {'upper', 'forearm'},
          'arm_reference_template_regions')
    for region in template['regions'].values():
        check(type(region) is dict and set(region) == {'movingPolygon', 'backgroundPolygons', 'forwardImageSign'},
              'arm_reference_region_schema')
        check(type(region['forwardImageSign']) is int and region['forwardImageSign'] in (-1, 1),
              'arm_reference_forward_sign')
        backgrounds = region['backgroundPolygons']
        check(type(backgrounds) is list and 2 <= len(backgrounds) <= 8, 'arm_reference_background_regions')
        occupied = _mask(region['movingPolygon'], (size[1], size[0]))
        for polygon in backgrounds:
            mask = _mask(polygon, (size[1], size[0]))
            check(not np.any((occupied != 0) & (mask != 0)), 'arm_reference_regions_overlap')
            occupied |= mask
    return template


def baseline_quality(image, template):
    check([image.shape[1], image.shape[0]] == template['imageSize'], 'arm_reference_image_size_changed')
    result = {}
    for name, region in template['regions'].items():
        quality = observe_pair(image, image, region['movingPolygon'], region['backgroundPolygons'],
                               region['forwardImageSign'])
        check(quality['valid'], 'arm_reference_baseline_quality:' + name + ':' + quality['reason'])
        result[name] = quality
    return result


def reference_sensor(ev, plan, frame, *, expected=None):
    """초기 기준 가까이의 GET 상태 한 쌍만 핀한다. 재생 시 같은 쌍을 쓴다."""
    selector = PinnedEvidence(ev.root)
    rows, files = [], {}
    port = replay_port(plan['controllerBootId'])
    if expected is None:
        names = [path.name for path in sorted(ev.root.glob('http-*-intent.json'))]
    else:
        check(type(expected) is dict and type(expected.get('intentFile')) is str
              and re.fullmatch('http-[0-9]{6}-intent.json', expected['intentFile']),
              'arm_reference_sensor_pin_invalid')
        names = [expected['intentFile']]
    for name in names:
        intent = selector.json(name)
        if intent.get('path') not in ('/status', '/collection_status'):
            check(expected is None, 'arm_reference_sensor_route_changed')
            continue
        response_name = name.replace('-intent', '-response')
        response = selector.json(response_name)
        parsed = response_json(response['rawUtf8'], intent['path'])
        check(intent.get('method') == 'GET' and intent.get('payload') is None
              and response.get('httpStatus') == 200 and response.get('parsed') == parsed,
              'arm_reference_sensor_response_changed')
        check(finite(intent.get('startedAt')) and finite(response.get('receivedAt'))
              and intent['startedAt'] <= response['receivedAt'], 'arm_reference_sensor_time_changed')
        row = dict(at=response['receivedAt'], angleDeg=port._shoulder_angle(parsed))
        rows.append(row)
        files[row['at']] = (dict(intentFile=name, responseFile=response_name,
                                requestStartedAt=intent['startedAt']), parsed)
    midpoint = (frame['sourceEarliest'] + frame['sourceLatest']) / 2
    sample = nearest_sensor(rows, midpoint)
    selected, status = files[sample['hostObservedAt']]
    port._check_status(status, 'IDLE')
    sample.update(selected)
    check(expected is None or sample == expected, 'arm_reference_sensor_pin_changed')
    for name in (sample['intentFile'], sample['responseFile']):
        check(ev.raw(name) == selector.raw(name), 'arm_reference_sensor_bytes_changed')
    return sample


def build_registration(root, plan, template_raw, frame, *, first_sensor=None):
    """첫 사전 프레임만 등록한다. 호출자는 완료 시각과 불변 저장을 담당한다.

    live에서는 recorder가 이미 저장한 frame을 읽을 뿐 촬영/통신을 호출하지 않는다.
    닫힌 세션의 전체 녹화 감사는 compare_run에서 별도로 수행한다.
    """
    template = load_template(template_raw, plan.get('armReferenceTemplateSha256'))
    check(frame.get('frameIndex') == 0 and frame.get('frameId') == plan['sessionId'] + ':0',
          'arm_reference_first_frame_required')
    ev = PinnedEvidence(root)
    image, verified = native_image(ev, plan['camera'], frame)
    for key, field in (('metadataFile', 'metadataSha256'), ('imageFile', 'imageSha256')):
        check(sha(ev.raw('native-camera/' + frame[key])) == frame[field], 'arm_reference_frame_bytes_changed')
    check([verified['width'], verified['height']] == [plan['camera']['width'], plan['camera']['height']],
          'arm_reference_plan_image_size_changed')
    sensor_sample = reference_sensor(ev, plan, frame, expected=first_sensor)
    quality = baseline_quality(image, template)
    unchanged(ev)
    return dict(schemaVersion=REGISTRATION_SCHEMA, mode='pre_action_same_capture',
                planSha256=plan_digest(plan), templateSha256=sha(template_raw),
                sessionId=plan['sessionId'], controllerBootId=plan['controllerBootId'],
                camera=copy.deepcopy(plan['camera']), frame=copy.deepcopy(frame), nativeFrame=verified,
                baselineQuality=quality, firstFrameSensorSample=sensor_sample, sourceProofs=ev.hashes,
                referencePolicy='first_frame_index_zero_no_fallback_no_roi_reposition',
                registeredAt=None, physicallyCalibrated=False, controlEligible=False)


def _verify_registered_reference(ev, plan, template_raw, frames, trial, http_proof, reference):
    check(ev.json('arm-reference.json') == reference, 'arm_reference_report_changed')
    check(ev.hashes['arm-reference.json'] == ev.json('report.json').get('armReferenceSha256'),
          'arm_reference_artifact_hash_changed')
    check(ev.raw('arm-reference-template.json') == template_raw, 'arm_reference_saved_template_changed')
    check(type(reference) is dict and reference.get('schemaVersion') == REGISTRATION_SCHEMA
          and reference.get('mode') == 'pre_action_same_capture', 'arm_reference_registration_schema')
    first = pinned_frame(frames, 0, plan['sessionId'] + ':0')
    check(reference.get('frame') == first, 'arm_reference_source_frame_changed')
    rebuilt = build_registration(ev.root, plan, template_raw, first,
                                 first_sensor=reference.get('firstFrameSensorSample'))
    for key in ('schemaVersion', 'mode', 'planSha256', 'templateSha256', 'sessionId',
                'controllerBootId', 'camera', 'frame', 'nativeFrame', 'sourceProofs', 'referencePolicy',
                'firstFrameSensorSample',
                'physicallyCalibrated', 'controlEligible'):
        check(reference.get(key) == rebuilt[key], 'arm_reference_binding_changed:' + key)
    check(type(reference.get('baselineQuality')) is dict
          and set(reference['baselineQuality']) == {'upper', 'forearm'}
          and all(q.get('valid') is True for q in reference['baselineQuality'].values()),
          'arm_reference_quality_receipt_changed')
    at = reference.get('registeredAt')
    check(finite(at) and first['stdoutReadAt'] <= at <= http_proof['firstWriteStartedAt']
          <= trial['dispatchBefore'], 'arm_reference_registered_after_write')
    return copy.deepcopy(reference)


def compare_run(root, template_raw, expected_plan_sha256, *, allow_offline_registration=False):
    """전체 원본을 감사한 뒤 같은 촬영 첫 영상→동작 후 영상을 관측한다.

    과거 자료 사후등록은 명시적 옵션이 필요하며 동작 전 등록으로 승격하지 않는다.
    이 결과는 학습 점수·3D 관절각·영점 보정값으로 사용하지 않는다.
    """
    check(type(expected_plan_sha256) is str and re.fullmatch('[0-9a-f]{64}', expected_plan_sha256),
          'expected_plan_sha256_required')
    template = load_template(template_raw)
    ev = PinnedEvidence(root)
    report = ev.json('report.json')
    plan = validate_plan(ev.json('plan.json'))
    check(report.get('plan') == plan and report.get('planSha256') == plan_digest(plan) == expected_plan_sha256,
          'plan_changed')
    trial, rows, http_proof = audit_trial_http(ev, report, plan)
    frames, identity = audit_camera(ev, plan, report)
    selected, endpoint_proof = select_endpoints(frames, trial, plan['postSeconds'],
                                               verified_quiet_end=http_proof['quietOutputsVerifiedThrough'])
    reference = report.get('armReference')
    pre_action = 'armReferenceTemplateSha256' in plan
    if pre_action:
        load_template(template_raw, plan['armReferenceTemplateSha256'])
        check(reference is not None, 'arm_reference_registration_missing')
        reference = _verify_registered_reference(ev, plan, template_raw, frames, trial, http_proof, reference)
    else:
        check(reference is None and 'armReferenceSha256' not in report,
              'arm_reference_without_plan_pin')
        check(allow_offline_registration is True, 'explicit_offline_registration_required')
        reference = build_registration(ev.root, plan, template_raw, selected[0])
        reference.update(mode='offline_reconstruction_not_pre_action', registeredAt=None)
    images, verified = zip(*(native_image(ev, plan['camera'], frame) for frame in selected))
    check(all(verified[0][k] == verified[1][k] for k in (
        'width', 'height', 'videoMirrored', 'videoRotationAngle', 'configurationSha256')), 'endpoint_camera_changed')
    check(verified[0] == reference['nativeFrame'], 'registered_reference_endpoint_changed')
    check(verified[0]['persistedAt'] <= trial['dispatchBefore'], 'reference_not_available_before_dispatch')
    samples = [copy.deepcopy(reference['firstFrameSensorSample']),
               nearest_sensor(rows, (selected[1]['sourceEarliest'] + selected[1]['sourceLatest']) / 2)]
    delta = sensor_delta(*samples)
    regions = {}
    for name, region in template['regions'].items():
        observation = observe_pair(images[0], images[1], region['movingPolygon'],
                                   region['backgroundPolygons'], region['forwardImageSign'])
        regions[name] = dict(declaredRegions=copy.deepcopy(region), observation=observation,
                             projectedMinusSensorDeg=observation['projectedDeltaDeg'] - delta
                             if observation['valid'] else None)
    unchanged(ev)
    # Registration reads independently pinned first-frame files; retain those
    # alongside the complete closed recorder and HTTP evidence audited above.
    check(all(sha(_read(ev.root, name)) == digest
              for name, digest in reference['sourceProofs'].items()), 'reference_source_changed_during_audit')
    result = dict(schemaVersion=RESULT_SCHEMA, sourceSchema=report['schemaVersion'],
                  sourceReportSha256=ev.hashes['report.json'], planSha256=expected_plan_sha256,
                  templateSha256=sha(template_raw), reference=reference,
                  preActionReferenceVerified=pre_action, sourceRawHttpAudited=True,
                  httpAudit=http_proof, endpointSelection=endpoint_proof, nativeFrames=list(verified),
                  cameraIdentity=identity, sensorSamples=samples, sensorDeltaDeg=delta,
                  regions=regions, sourceProofs=ev.hashes,
                  summary=dict(requested=2, valid=sum(r['observation']['valid'] for r in regions.values()),
                               rejected=sum(not r['observation']['valid'] for r in regions.values())),
                  measurement='same_capture_image_plane_rotation_not_3d_joint_angle',
                  opticalSensorSynchronizationVerified=False, physicallyCalibrated=False,
                  jointZeroCalibrated=False, controlEligible=False, trainingPerformed=False,
                  newPhysicalCommands=0)
    return result, images
