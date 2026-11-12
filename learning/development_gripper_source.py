"""개발 관측기를 기존 fresh JPEG·단발 수집 증거에 연결한다. 실행 권한은 없다.

저장된 전후 쌍은 기존 bind_gripper_pair의 시간·부팅 검사를 재사용한다.
원본 감사 결과나 관측기 admission을 새로 만들지 않으며 학습 전이도 생성하지 않는다.
"""
from __future__ import annotations

import copy
import math
import re

from learning.artifacts import _decode_json, _read
from learning.batch_observation import require
from learning.development_gripper_observer import (
    DevelopmentGripperObserver, LANDMARK_PREFIX, STATE_FIELDS, sha,
)
from learning.gripper_camera_dataset import bind_gripper_pair, read_frame

BINDING_SCHEMA = 'motionbrain.development_gripper_source_binding.v1'


def load_observer(root, binding_path, binding_sha256):
    raw = _read(root, binding_path)
    require(sha(raw) == binding_sha256, 'development_observer_binding_hash')
    spec = _decode_json(raw)
    require(isinstance(spec, dict) and set(spec) ==
            {'schemaVersion', 'context', 'references', 'configurationSha256'} and
            spec['schemaVersion'] == BINDING_SCHEMA, 'development_observer_binding_fields')
    require(isinstance(spec['references'], list) and 1 <= len(spec['references']) <= 8,
            'development_observer_reference_budget')
    references = []
    for reference in spec['references']:
        require(isinstance(reference, dict) and set(reference) ==
                {'id', 'path', 'sha256', 'points', 'rois'}, 'development_source_reference_fields')
        body = _read(root, reference['path'])
        require(sha(body) == reference['sha256'], 'development_source_reference_hash')
        references.append(dict(id=reference['id'], imageBytes=body, imageSha256=reference['sha256'],
                               points=reference['points'], rois=reference['rois']))
    observer = DevelopmentGripperObserver(spec['context'], references)
    require(observer.binding == spec['configurationSha256'], 'development_observer_configuration_binding')
    return observer


def scene_context(observer):
    """등록 문맥의 표현. 실제 현장 확인을 대신하는 함수는 아니다."""
    return dict(physicalView=observer.context['viewId'], object=observer.context['object'],
                poseRegime=observer.context['poseRegime'], landmarks=LANDMARK_PREFIX+observer.binding)


def observe_bound_frame(observer, frame_root, name, *, session_id, camera_boot_id,
                        actual_scene, now, max_age_seconds=5., previous=None):
    require(isinstance(camera_boot_id, str) and re.fullmatch('[0-9a-f]{32}', camera_boot_id),
            'development_camera_boot_required')
    require(actual_scene == scene_context(observer), 'development_actual_scene_changed')
    require(type(now) in (int, float) and math.isfinite(now) and
            type(max_age_seconds) in (int, float) and math.isfinite(max_age_seconds) and
            .1 <= max_age_seconds <= 10, 'development_source_age_policy')
    metadata_raw = _read(frame_root, f'esp32-{name}.json')
    metadata_snapshot = _decode_json(metadata_raw)
    require(isinstance(metadata_snapshot, dict) and isinstance(metadata_snapshot.get('headers'), dict),
            'development_source_metadata_fields')
    frame, metadata, boot = read_frame(frame_root, name, session_id)
    require(metadata == metadata_snapshot, 'development_source_metadata_changed')
    require(metadata.get('status') == 200 and metadata.get('method') == 'GET' and
            metadata.get('path') == '/capture' and metadata.get('query') == {'fresh': '1'},
            'development_fresh_capture_request_required')
    require(boot == camera_boot_id, 'development_camera_boot_changed')
    require(metadata['persistedAt'] <= now and
            0 <= now-metadata['requestStartedAt'] <= max_age_seconds, 'development_source_stale')
    if previous is not None:
        require(previous['cameraBootId'] == boot and previous['hostClockId'] == session_id and
                frame['requestedUs'] > previous['capturedUs'] and
                metadata['requestStartedAt'] >= previous['persistedAt'],
                'development_source_reused_or_regressed')
    body = _read(frame_root, f'esp32-{name}.jpg')
    result = observer.observe(body, frame['frameId'], observer.context, image_sha256=frame['imageSha256'])
    require(_read(frame_root, f'esp32-{name}.json') == metadata_raw and
            sha(_read(frame_root, f'esp32-{name}.jpg')) == frame['imageSha256'],
            'development_source_bytes_changed')
    state = None
    if result['features'] is not None:
        features = result['features']
        state = [*features['objectMinusJawMidpointPx'], features['jawHorizontalSpanPx']]
    result.update(sourceFrame=copy.deepcopy(frame), cameraBootId=boot,
        hostClockId=session_id, sourceHostBracket=[metadata['requestStartedAt'], metadata['receivedAt']],
        persistedAt=metadata['persistedAt'], capturedUs=frame['capturedUs'],
        requestMode='fresh_after_request', sourceMetadataSha256=sha(metadata_raw),
        sourceAgeAtCheckSeconds=now-metadata['requestStartedAt'],
        actualSceneDeclaration=copy.deepcopy(actual_scene), sourceFreshMetadataVerified=True,
        exposureSynchronizationVerified=False, stateFields=STATE_FIELDS.copy(), state=state,
        observationContract=LANDMARK_PREFIX+observer.binding, trainingEligible=False,
        controlEligible=False, liveAdmissionVerified=False, motionAdmissionVerified=False)
    return result


def observe_saved_pair(observer, row, pair_root, *, actual_scene, camera_boot_id):
    """기존 감사 행에 보존된 예정 명령·시간 창을 그대로 요구한다."""
    from learning.relative_action_learning import command_key
    command_key(row['inputs']['command'])
    snapshots = {f'esp32-{side}.{suffix}': _read(pair_root, f'esp32-{side}.{suffix}')
                 for side in ('before', 'after') for suffix in ('json', 'jpg')}
    paired = bind_gripper_pair(row, pair_root)
    require(all(_read(pair_root, name) == raw for name, raw in snapshots.items()),
            'development_pair_bytes_changed')
    for side, key in [('before', 'inputs'), ('after', 'targets')]:
        require(paired['evidence']['gripperFrameEvidence'][side] ==
                _decode_json(snapshots[f'esp32-{side}.json']) and
                paired[key]['image']['imageSha256'] == sha(snapshots[f'esp32-{side}.jpg']),
                'development_paired_source_binding_changed')
    before, pre, _ = read_frame(pair_root, 'before', row['sessionId'])
    after, post, _ = read_frame(pair_root, 'after', row['sessionId'])
    observed = {}
    for side, metadata in [('before', pre), ('after', post)]:
        observed[side] = observe_bound_frame(observer, pair_root, side, session_id=row['sessionId'],
            camera_boot_id=camera_boot_id, actual_scene=actual_scene,
            now=metadata['persistedAt'], max_age_seconds=5.,
            previous=observed.get('before') if side == 'after' else None)
        require(observed[side]['sourceFrame'] == paired['inputs' if side == 'before' else 'targets']['image'] and
                observed[side]['sourceMetadataSha256'] == sha(snapshots[f'esp32-{side}.json']),
                'development_paired_source_frame_changed')
    require(all(_read(pair_root, name) == raw for name, raw in snapshots.items()),
            'development_pair_bytes_changed')
    return dict(schemaVersion='motionbrain.development_gripper_pair.v1',
        sourceSessionId=row['sessionId'], sourceHash=row.get('sourceHash'),
        plannedCommand=copy.deepcopy(row['inputs']['command']),
        commandAt=row['evidence']['commandAt'], offReceiptAt=row['evidence']['offReceiptAt'],
        observationContract=LANDMARK_PREFIX+observer.binding, perFrame=observed,
        sourcePairHashes={name: sha(raw) for name, raw in snapshots.items()},
        status='complete_development' if all(v['state'] is not None for v in observed.values())
               else 'partial_development',
        stateFields=STATE_FIELDS.copy(),
        savedReceiptReplayOnly=True, currentFreshnessVerified=False,
        originalOutcomeChanged=False, trainingEligible=False, controlEligible=False,
        reason='development_observer_motion_admission_and_learning_adapter_required')
