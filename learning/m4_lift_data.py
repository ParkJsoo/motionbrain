"""Read-only audit of NEW completed M4 learning trials; no devices or fitting.

Images may repeat byte-for-byte. Reuse identity comes from audited single RUN,
camera frame identities and ingress receipts. Actual output/sensors stay evidence;
only pre-command visual measurements and the planned pulse are model inputs.
"""
from __future__ import annotations

import copy
from fractions import Fraction
from pathlib import Path

import cv2
import numpy as np

from learning import m4_up_commissioning as core
from learning import m4_lift_learning as learning
from learning.action_recorder import ActionRecorder, recording_bridge
from learning.artifacts import _decode_json, _read
from learning.mac_clock_bridge import validate_sample
from learning.manual_collection_dataset import Evidence, audit_http, digest, require, select_frames
from learning.supervised_collection import plan_digest
from learning.supervised_measurement import finite
from learning.supervised_transport import response_json


class _CompareIngress:
    def __init__(self, ev, frames):
        self.ev, self.frames, self.index = ev, frames, 0

    def write_json(self, name, frame):
        require(self.ev.json('external/host-ingress/'+name) == frame
                and self.frames[self.index] == frame, 'lift_ingress_or_summary_changed')
        self.index += 1


def _native(ev, scope, closed):
    require(closed.get('status') == 'completed' and closed.get('errors') == [], 'lift_camera_close_failed')
    require(closed['gripper'].get('status') == 'completed', 'lift_gripper_close_failed')
    native = closed['native']; frames = native['frames']
    require(native['reason'] is None and bool(frames) and native['recordedFrames'] == len(frames),
            'lift_native_recording_failed')
    require(ev.json('external/host-ingress/recording-summary.json') == native, 'lift_recording_summary_changed')
    before = ev.json('external/host-ingress/clock-before.json')
    after = ev.json('external/host-ingress/clock-after.json')
    bridge = recording_bridge(before, after)
    require(before['hostClockId'] == scope['hostClockId'] and native['clockBridge'] == bridge
            and ev.json('external/host-ingress/clock-bridge.json') == bridge, 'lift_closed_clock_required')
    ctx = scope['context']
    plan = dict(deviceId=ctx['externalDeviceId'], viewId=ctx['externalViewId'], sessionId=scope['hostClockId'],
                maxFrames=500, seconds=90, sampleFps=4, maxBytes=4200000000)
    recorder = ActionRecorder(ev.root/'external', 'unused-offline-helper', plan)
    recorder.started = before['pairs'][-1]['after']
    recorder.offset_low, recorder.offset_high = validate_sample(before)
    recorder.store = _CompareIngress(ev, frames)
    lines = [_decode_json(line) for line in ev.raw('external/helper.stdout').splitlines()]
    require(len(lines) == len(frames)+1 and lines[-1].get('event') == 'terminal', 'lift_native_receipt_count')
    for i, (line, frame) in enumerate(zip(lines, frames)):
        require(line.get('event') == 'frame' and frame['frameIndex'] == i, 'lift_native_frame_order')
        recorder._packet(line['receipt'], frame['stdoutReadAt'])
        require(bridge['validPythonBounds'][0] <= frame['sourceEarliest'] <= frame['sourceLatest']
                <= frame['stdoutReadAt'] <= bridge['validPythonBounds'][1], 'lift_native_clock_bounds')
        ev.hashes['external/'+frame['metadataFile']] = frame['metadataSha256']
        ev.hashes['external/'+frame['imageFile']] = frame['imageSha256']
    recorder.closing = True
    recorder._read_terminal(lines[-1])
    terminal = ev.json('external/terminal.json')
    require(terminal.get('droppedFrames') == 0 and native['terminalStatus'] == terminal['status'],
            'lift_camera_dropped_frames')
    return frames, bridge


def _http(ev, receipt, pulse):
    intents = sorted(ev.root.glob('http-*-intent.json'))
    require(bool(intents), 'lift_http_missing')
    events, baseline, readiness_at = [], None, None
    for i, path in enumerate(intents, 1):
        name = f'http-{i:06d}'
        require(path.name == name+'-intent.json', 'lift_http_sequence_gap')
        intent, response = ev.json(name+'-intent.json'), ev.json(name+'-response.json')
        parsed = response_json(response['rawUtf8'], intent['path'])
        events.append(dict(kind='http', path=intent['path'], payload=intent['payload'], response=parsed,
                           requestStartedAt=intent['startedAt'], responseReceivedAt=response['receivedAt']))
        if readiness_at is None and intent['payload'] is not None:
            readiness_at = intent['startedAt']
        if baseline is None and intent['path'].split('?')[0] == '/action_outputs':
            baseline = parsed.get('latestSequence')
    # Legacy auditor's command.at denotes its PRE-ARM readiness checkpoint.
    # Using wire dispatch there would incorrectly require an ARMED status to be
    # IDLE. RUN timing remains bound to the actual receipt, including all windows.
    statuses, entries, boot = audit_http(ev, dict(events=events, outputBaselineSequence=baseline),
                                        dict(pulse=pulse, at=readiness_at), receipt)
    require(receipt.get('offVerified') is True and receipt.get('motor') == 4 and
            receipt.get('requestedPulse') == pulse, 'lift_receipt_pulse_changed')
    moving = [e for e in receipt['outputTrace'] if e['pwm'] != 0]
    require(bool(moving) and all(e['motor'] == 4 and 0 < e['pwm'] <= pulse['expectedPwm'] for e in moving),
            'lift_output_ceiling_or_direction')
    require(all(finite(receipt[k]) for k in ('dispatchBefore', 'dispatchAfter', 'offAt')) and
            receipt['dispatchBefore'] <= receipt['dispatchAfter'] < receipt['offAt'], 'lift_receipt_time_order')
    return statuses, entries, boot, moving[0]['sequence']


def _pair(ev, label, scope, receipt, registration, allowed_frames):
    record = ev.json(label+'-observation.json')
    require(record['hostClockId'] == scope['hostClockId'] and record['context'] == scope['context']
            and record['stable'] is True, 'lift_observation_context_changed')
    require(digest(ev.raw(record['evidenceFile'])) == record['evidenceSha256'] and
            ev.json(record['evidenceFile']) == {k:v for k,v in record.items()
                                              if k not in ('evidenceFile', 'evidenceSha256')}, 'lift_review_changed')
    onsite = record['onsite']
    require(all(type(onsite.get(k)) is bool for k in
                ('operatorPresent','clearanceVerified','jawContactVerified','tableSupportVerified')) and
            onsite['operatorPresent'] and onsite['clearanceVerified'] and
            (label != 'before' or (onsite['jawContactVerified'] and onsite['tableSupportVerified'])),
            'lift_onsite_review_missing')
    require(record['external']['root'] == 'external', 'lift_native_root_changed')
    external = core._external_frame(ev.root, record['external'], scope)
    gripper = core._gripper_frame(ev.root, record['gripper'], scope, label)
    binding = dict(external={k:external[k] for k in ('configurationSha256','width','height','videoMirrored','videoRotationAngle')},
                   gripper={k:gripper[k] for k in ('width','height')})
    require(registration.get('cameraBinding') == binding, 'lift_camera_binding_changed')
    metadata_name = record['gripper']['root']+f'/esp32-{label}.json'
    metadata = ev.json(metadata_name)
    require(ev.hashes[metadata_name] == record['gripper']['metadataSha256'], 'lift_gripper_metadata_changed')
    headers = {k.lower():v for k,v in metadata['headers'].items()}
    gripper['requestedUs'] = int(headers['x-motionbrain-capture-requested-us'])
    require(any(f['frameId'] == external['frameId'] for f in allowed_frames), 'lift_selected_frame_outside_window')
    for frame in (external, gripper):
        times = [frame[k] for k in ('sourceEarliest', 'sourceLatest', 'persistedAt')]
        require(all(finite(t) for t in times) and times == sorted(times), 'lift_dual_times_invalid')
        if label == 'before':
            require(receipt['dispatchBefore']-scope['maxObservationAgeSeconds'] <= times[0]
                    and times[-1] <= receipt['dispatchBefore']-.1, 'lift_input_after_dispatch')
        else:
            require(receipt['offAt']+3 <= times[0] and times[-1] <= receipt['offAt']+6,
                    'lift_target_outside_window')
    raw_source = ev.json(label+'-measurement-source.json')
    name = label+'-external.png'; png = ev.raw(name)
    pixels = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_UNCHANGED)
    require(pixels is not None and pixels.shape == (external['height'], external['width'], 4)
            and digest(pixels.tobytes()) == external['pixelSha256'], 'lift_preview_pixels_changed')
    expected = dict(id=scope['hostClockId']+':'+label, context=scope['context'],
                    external=dict(path=name, sha256=digest(png)),
                    gripper=dict(path=record['gripper']['root']+f'/esp32-{label}.jpg',
                                 sha256=record['gripper']['imageSha256']))
    require(raw_source == expected, 'lift_measurement_source_changed')
    for source in raw_source['external'], raw_source['gripper']:
        require(digest(ev.raw(source['path'])) == source['sha256'], 'lift_measured_bytes_changed')
    features = learning.measure(ev.root, raw_source, registration)
    require(features == ev.json(label+'-measurement.json'), 'lift_saved_measurement_changed')
    return features, external, gripper, [record['external']['ingressSha256'], record['gripper']['metadataSha256']]


def build_row(root, registration):
    """Return one audited row. Never changes raw training flags or source files."""
    ev = Evidence(Path(root).resolve())
    scope, result = ev.json('scope.json'), ev.json('result.json')
    require(scope.get('schemaVersion') in (core.LEARNING_SCHEMA, core.MODEL_SCHEMA), 'lift_new_learning_schema_required')
    require(result.get('schemaVersion') == scope['schemaVersion'] and result.get('status') == 'completed'
            and result.get('errors') == [] and result.get('finalIdleOffVerified') is True
            and result.get('trainingEligible') is False and result.get('mayHaveRun') is True
            and result.get('commandOutcome') == 'receipt_recorded', 'lift_completed_raw_trial_required')
    require(scope['context'] == registration['context'] and
            scope['observationContractSha256'] == learning.contract_sha(registration) and
            result['scopeSha256'] == plan_digest(scope), 'lift_scope_or_contract_changed')
    require(scope['defaultSpeed'] == 255 and scope['maxCommands'] == 1 and
            scope['phase'] in ('training','evaluation','model_selection') and
            (scope['schemaVersion'] == core.MODEL_SCHEMA) == (scope['phase'] == 'model_selection') and
            scope['pulse'] in core.LEARNING_CANDIDATES, 'lift_learning_scope_invalid')
    pulse = result['selectedPulse']
    require(pulse in core.LEARNING_CANDIDATES and
            (scope['phase'] == 'model_selection' or pulse == scope['pulse']) and
            result['modelSelected'] is (scope['phase'] == 'model_selection'), 'lift_selected_pulse_changed')
    receipt = result['receipt']
    statuses, outputs, boot, first = _http(ev, receipt, pulse)
    require(boot == scope['controllerBootId'], 'lift_controller_boot_changed')
    frames, bridge = _native(ev, scope, result['observerClose'])
    windows = ([receipt['dispatchBefore']-3, receipt['dispatchBefore']-.1],
               [receipt['offAt']+3, receipt['offAt']+6])
    selected = [select_frames(frames, window, receipt['dispatchBefore'] if i == 0 else None)
                for i, window in enumerate(windows)]
    pairs = [_pair(ev, label, scope, receipt, registration, selected[i]) for i,label in enumerate(('before','after'))]
    before, after = pairs
    require(after[1]['frameIndex'] > before[1]['frameIndex'] and
            Fraction(*after[1]['pts']) > Fraction(*before[1]['pts']) and
            after[2]['requestedUs'] > before[2]['capturedUs'], 'lift_camera_order_changed')
    require(all(bridge['validPythonBounds'][0] <= pair[2]['sourceEarliest'] <= pair[2]['persistedAt']
                <= bridge['validPythonBounds'][1] for pair in pairs), 'lift_gripper_outside_recording')
    require(result['beforeFeatures'] == before[0] and result['afterFeatures'] == after[0], 'lift_result_features_changed')
    # Re-read everything touched: audit may not silently race source mutation.
    require(all(digest(_read(ev.root, name)) == sha for name,sha in ev.hashes.items()), 'lift_source_changed_during_audit')
    return dict(id=scope['trialId'], phase=scope['phase'], context=copy.deepcopy(scope['context']),
                contractSha256=scope['observationContractSha256'], pulse={k:pulse[k] for k in
                    ('joint','direction','percent','durationMs')}, before=before[0], after=after[0],
                episodeIdentity=[boot, first], sourceEvidenceSha256=before[3]+after[3],
                sourceCaptureIds=[p[1]['frameId'] if j == 0 else p[2]['frameId'] for p in pairs for j in range(2)],
                sourceRoot=str(ev.root), resultSha256=ev.hashes['result.json'], sourceProofs=dict(ev.hashes),
                evidence=dict(outputTrace=copy.deepcopy(receipt['outputTrace']),
                              angleObservations=copy.deepcopy(result['angleObservations']),
                              cameraHostBounds=bridge['validPythonBounds'], windows=windows),
                originalTrainingEligible=False)
