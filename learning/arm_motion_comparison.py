"""저장된 M4 전후 영상과 센서의 사후 대조. 장치 연결·학습은 하지 않는다."""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
import re

import cv2
import numpy as np

from learning.arm_motion_observation import observe_pair
from learning.artifacts import _decode_json, _read, StorageError
from learning.lossless_frame import decode
from learning.m4_up_commissioning import _external_frame, LEARNING_SHORT_SCHEMA
from learning.supervised_collection import plan_digest


SCHEMA = 'motionbrain.arm_motion_comparison.plan.v1'


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def nearest_sensor(rows, at, max_gap=.5):
    """호스트 관측 시각의 근접값만 선택한다. 센서·노출 동기화를 뜻하지 않는다."""
    require(type(rows) is list and bool(rows), 'sensor_rows_required')
    require(type(at) in (int, float) and math.isfinite(at), 'finite_frame_time_required')
    require(type(max_gap) in (int, float) and 0 < max_gap <= 2, 'bounded_sensor_gap_required')
    previous = -math.inf
    for row in rows:
        require(type(row) is dict and all(type(row.get(k)) in (int, float) and
                math.isfinite(row[k]) for k in ('at', 'angleDeg')), 'invalid_sensor_row')
        require(row['at'] > previous and 0 <= row['angleDeg'] < 360, 'sensor_order_or_range_invalid')
        previous = row['at']
    selected = min(rows, key=lambda r: abs(r['at'] - at))
    gap = selected['at'] - at
    require(abs(gap) <= max_gap, 'sensor_frame_time_gap_exceeded')
    return dict(angleDeg=selected['angleDeg'], hostObservedAt=selected['at'],
                frameHostPts=at, hostTimeGapSeconds=gap, opticalSynchronizationVerified=False)


def sensor_delta(before, after):
    delta = after['angleDeg'] - before['angleDeg']
    # The current sensor mapping does not unwrap: refuse a wrap rather than
    # inventing a physical direction from two ambiguous values.
    require(abs(delta) < 180, 'sensor_wrap_ambiguous')
    return delta


def pinned_frame(frames, index, frame_id):
    """최종 결과에 고정된 유일한 프레임만 선택한다."""
    require(type(frames) is list and 1 <= len(frames) <= 1000, 'bounded_native_summary_required')
    require(all(type(f) is dict and type(f.get('frameIndex')) is int and
                isinstance(f.get('frameId'), str) for f in frames), 'invalid_native_summary')
    indices = [f['frameIndex'] for f in frames]
    ids = [f['frameId'] for f in frames]
    require(len(set(indices)) == len(frames) and len(set(ids)) == len(frames) and
            indices == sorted(indices), 'duplicate_or_unordered_native_summary')
    selected = [f for f in frames if f['frameIndex'] == index and f['frameId'] == frame_id]
    require(len(selected) == 1, 'endpoint_not_in_pinned_native_summary')
    return selected[0]


def _native(root, source, scope, *, expected):
    ingress_raw = _read(root, source['root'] + '/' + source['ingressFile'])
    ingress = _decode_json(ingress_raw)
    require(ingress == expected, 'native_summary_frame_changed')
    verified = _external_frame(root, source, scope)
    require(_read(root, source['root'] + '/' + source['ingressFile']) == ingress_raw,
            'native_ingress_changed_during_read')
    require(all(verified[k] == expected[k] for k in
                ('frameId', 'frameIndex', 'pixelSha256', 'width', 'height', 'sourceEarliest', 'sourceLatest')),
            'native_verified_frame_changed')
    stored = _read(root, source['root'] + '/' + ingress['imageFile'])
    pixels = decode(stored, ingress['rawByteLength'], ingress['pixelSha256'])
    image = np.frombuffer(pixels, np.uint8).reshape(verified['height'], verified['width'], 4)[:, :, :3]
    return image, verified, ingress


def audit_case(case, regions, workspace, *, reference_view, series_stride=0):
    require(type(case) is dict and set(case) == {'root', 'resultSha256'}, 'invalid_case_fields')
    workspace = Path(workspace).resolve()
    relative = Path(case['root'])
    require(not relative.is_absolute() and '..' not in relative.parts, 'private_case_path_required')
    root = (workspace / relative).resolve(strict=True)
    require(root.is_relative_to(workspace / '.local'), 'private_case_path_required')
    result_raw = _read(root, 'result.json')
    require(sha(result_raw) == case['resultSha256'], 'result_hash_changed')
    result, scope = _decode_json(result_raw), _decode_json(_read(root, 'scope.json'))
    require(result['schemaVersion'] == scope['schemaVersion'] == LEARNING_SHORT_SCHEMA and
            result['scopeSha256'] == plan_digest(scope), 'unsupported_or_changed_case_scope')
    require(plan_digest(scope['context']) == reference_view['contextSha256'] and
            scope['controllerBootId'] == reference_view['controllerBootId'], 'comparison_context_or_boot_changed')
    require(result['status'] == 'completed' and result['finalIdleOffVerified'] is True and
            result['mayHaveRun'] is True, 'incomplete_case_execution')
    pulse = result['selectedPulse']
    require(pulse['joint'] == 'shoulder' and pulse['motor'] == 4 and
            pulse == scope['pulse'], 'case_not_single_m4_pulse')
    receipt = result['receipt']
    require(receipt['offVerified'] is True and receipt['motor'] == 4 and
            receipt['dispatchBefore'] <= receipt['dispatchAfter'] < receipt['offAt'], 'invalid_case_receipt')
    trace = receipt['outputTrace']
    require(bool(trace) and all(e['pwm'] == 0 or e['motor'] == 4 for e in trace), 'other_motor_output_present')
    moving_outputs = [e for e in trace if e['pwm'] != 0]
    require(bool(moving_outputs) and all(type(e['sequence']) is int for e in moving_outputs),
            'physical_episode_identity_missing')
    frame_data = []
    sources = []
    native_frames = result['observerClose']['native']['frames']
    for label in ('before', 'after'):
        observation_raw = _read(root, label + '-observation.json')
        observation = _decode_json(observation_raw)
        require(observation['hostClockId'] == scope['hostClockId'] and
                observation['context'] == scope['context'], 'case_view_or_clock_changed')
        external = observation['external']
        ingress = _decode_json(_read(root, external['root'] + '/' + external['ingressFile']))
        expected = pinned_frame(native_frames, ingress['frameIndex'], ingress['frameId'])
        image, verified, ingress = _native(root, external, scope, expected=expected)
        png_source = _decode_json(_read(root, label + '-measurement-source.json'))
        require(png_source['context'] == scope['context'], 'png_view_changed')
        png_ref = png_source['external']
        png_raw = _read(root, png_ref['path'])
        require(sha(png_raw) == png_ref['sha256'] == result[label + 'Features']['sourceSha256']['external'],
                'derived_png_hash_changed')
        png = cv2.imdecode(np.frombuffer(png_raw, np.uint8), cv2.IMREAD_COLOR)
        require(png is not None and np.array_equal(png, image), 'derived_png_not_native_pixels')
        frame_data.append((image, verified, ingress))
        sources.append(dict(observationSha256=sha(observation_raw), pngSha256=sha(png_raw),
                            native=verified, ingressSha256=sha(_read(root, observation['external']['root'] + '/' +
                                                                   observation['external']['ingressFile']))))
    before, after = frame_data
    require(before[1]['sourceLatest'] < receipt['dispatchBefore'] and
            after[1]['sourceEarliest'] >= receipt['offAt'] + 3, 'case_frame_phase_invalid')
    require(all(before[1][k] == after[1][k] for k in
                ('width', 'height', 'videoMirrored', 'videoRotationAngle', 'configurationSha256')),
            'camera_configuration_changed')
    require(all(before[1][k] == reference_view[k] for k in
                ('videoMirrored', 'videoRotationAngle', 'configurationSha256')), 'comparison_camera_changed')
    require((before[1]['width'], before[1]['height']) == tuple(regions['imageSize']), 'regions_image_size_changed')
    sensor = [nearest_sensor(result['angleObservations'], f[2]['convertedHostPtsSeconds']) for f in frame_data]
    delta = sensor_delta(*sensor)
    observation = observe_pair(before[0], after[0], regions['movingPolygon'],
                               regions['backgroundPolygons'], regions['forwardImageSign'])
    row = dict(id=scope['trialId'], sourcePhase=scope['phase'], sourceRoot=case['root'],
               resultSha256=sha(result_raw), scopeSha256=plan_digest(scope),
               context=scope['context'], controllerBootId=scope['controllerBootId'],
               episodeIdentity=[scope['controllerBootId'], moving_outputs[0]['sequence']],
               frameSources=sources, selectedPulse=pulse, sensorSamples=sensor,
               sensorDeltaDeg=delta, observation=observation, valid=observation['valid'],
               projectedMinusSensorDeg=observation['projectedDeltaDeg'] - delta if observation['valid'] else None,
               priorModelSelected=result['modelSelected'], newPhysicalCommands=0,
               rawHttpReaudited=False, exactOpticalSensorSynchronizationVerified=False)
    start_pts = before[2]['convertedHostPtsSeconds']
    row['hostInspectionWindow'] = dict(dispatchBeforeMinusStart=receipt['dispatchBefore']-start_pts,
                                      dispatchAfterMinusStart=receipt['dispatchAfter']-start_pts,
                                      offObservedMinusStart=receipt['offAt']-start_pts)
    row['sensorSeries'] = [dict(atMinusBeforePts=s['at']-start_pts, angleDeg=s['angleDeg'],
                               deltaDeg=s['angleDeg']-sensor[0]['angleDeg'])
                           for s in result['angleObservations']
                           if start_pts <= s['at'] <= after[2]['convertedHostPtsSeconds']]
    if series_stride:
        frames = native_frames
        selected = [f for f in frames if before[1]['frameIndex'] <= f['frameIndex'] <= after[1]['frameIndex']]
        require(bool(selected) and len(selected) <= 512, 'bounded_series_required')
        selected = selected[::series_stride]
        if selected[-1]['frameIndex'] != after[1]['frameIndex']:
            selected.append(after[2])
        source_root = _decode_json(_read(root, 'before-observation.json'))['external']
        series = []
        for frame in selected:
            source = dict(source_root, ingressFile=f"host-ingress/{frame['frameIndex']:04d}.json")
            source['ingressSha256'] = sha(_read(root, source['root'] + '/' + source['ingressFile']))
            image, verified, ingress = _native(root, source, scope, expected=frame)
            measurement = observe_pair(before[0], image, regions['movingPolygon'],
                                        regions['backgroundPolygons'], regions['forwardImageSign'])
            sample = None
            sensor_reason = None
            try:
                sample = nearest_sensor(result['angleObservations'], ingress['convertedHostPtsSeconds'])
            except ValueError as exc:
                if str(exc) != 'sensor_frame_time_gap_exceeded':
                    raise
                sensor_reason = str(exc)
            series.append(dict(frameIndex=frame['frameIndex'],
                               atMinusBeforePts=ingress['convertedHostPtsSeconds'] - before[2]['convertedHostPtsSeconds'],
                               sensorDeltaDeg=sensor_delta(sensor[0], sample) if sample else None,
                               sensorHostGap=sample['hostTimeGapSeconds'] if sample else None,
                               sensorMatched=sample is not None, sensorRejectedReason=sensor_reason,
                               visual=measurement, native=verified))
        row['series'] = series
        row['seriesSensorUnmatched'] = sum(not s['sensorMatched'] for s in series)
        row['seriesVisualRejected'] = sum(not s['visual']['valid'] for s in series)
        row['seriesInterpretation'] = 'each_frame_compared_to_same_baseline; no optical_sensor_synchronization'
    return row


def compare_manifest(manifest, workspace, *, series_stride=0):
    require(type(manifest) is dict and set(manifest) == {'schema', 'regions', 'referenceView', 'cases'} and
            manifest['schema'] == SCHEMA, 'invalid_comparison_manifest')
    regions = manifest['regions']
    require(type(regions) is dict and set(regions) == {'imageSize', 'movingPolygon', 'backgroundPolygons',
                                                     'forwardImageSign'}, 'invalid_comparison_regions')
    view = manifest['referenceView']
    require(type(view) is dict and set(view) == {'contextSha256', 'controllerBootId', 'configurationSha256',
                                               'videoMirrored', 'videoRotationAngle'}, 'invalid_reference_view')
    require(type(series_stride) is int and 0 <= series_stride <= 32, 'invalid_series_stride')
    cases = manifest['cases']
    require(type(cases) is list and 1 <= len(cases) <= 12, 'bounded_case_list_required')
    require(all(type(c) is dict and set(c) == {'root', 'resultSha256'} and
                isinstance(c['root'], str) and isinstance(c['resultSha256'], str) and
                re.fullmatch('[0-9a-f]{64}', c['resultSha256']) for c in cases), 'invalid_case_fields')
    canonical_roots = [(Path(workspace).resolve() / c['root']).resolve() for c in cases]
    require(len(set(canonical_roots)) == len(cases) and
            len({c['resultSha256'] for c in cases}) == len(cases), 'duplicate_case')
    rows = []
    trial_ids, episodes = set(), set()
    for case in cases:
        try:
            row = audit_case(case, regions, workspace, reference_view=view, series_stride=series_stride)
        except (ValueError, OSError, KeyError, TypeError, StorageError, cv2.error) as exc:
            row = dict(sourceRoot=case.get('root'), valid=False, rejectedReason=str(exc),
                       projectedMinusSensorDeg=None, newPhysicalCommands=0)
        if 'id' in row:
            episode = tuple(row['episodeIdentity'])
            require(row['id'] not in trial_ids and episode not in episodes, 'duplicate_physical_episode')
            trial_ids.add(row['id']); episodes.add(episode)
        rows.append(row)
    good = [r for r in rows if r['valid']]
    moving = [r for r in good if abs(r['sensorDeltaDeg']) > .2]
    quiet = [r for r in good if abs(r['sensorDeltaDeg']) <= .2]
    return dict(schema='motionbrain.arm_motion_comparison.result.v1', cases=rows,
                declaredRegions=regions, referenceView=view,
                summary=dict(requested=len(rows), accepted=len(good), rejected=len(rows)-len(good),
                             sensorChangeThresholdDeg=.2, movingCases=len(moving), quietCases=len(quiet),
                             movingSameSign=sum(r['sensorDeltaDeg']*r['observation']['projectedDeltaDeg'] > 0 for r in moving),
                             movingMeanAbsoluteDifferenceDeg=float(np.mean([abs(r['projectedMinusSensorDeg']) for r in moving])) if moving else None,
                             quietMaximumAbsProjectedChangeDeg=max((abs(r['observation']['projectedDeltaDeg']) for r in quiet), default=None)),
                interpretation='retrospective_projected_video_vs_host_observed_sensor_change_only',
                newPhysicalCommands=0, modelRefit=False, controlEligible=False,
                m4ZeroCalibrated=False, m4DirectionCalibrated=False,
                cameraExtrinsicsCalibrated=False, rawHttpReaudited=False,
                exactOpticalSensorSynchronizationVerified=False)
