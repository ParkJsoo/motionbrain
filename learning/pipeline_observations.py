"""감사된 원본 전이의 재처리 가능한 관측. 장치 요청·구동·학습 권한 없음.

부분 관측을 세 점 상태로 채우지 않는다. 이 경로의 사후 재처리는 개발자료이며
새 고정 평가 또는 실시간 제어 적격을 만들지 않는다.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from learning.artifacts import StorageError, _decode_json, _read
from learning.gripper_camera_dataset import bind_gripper_pair
from learning.lossless_frame import decode
from learning.manual_collection_dataset import POLICY, require
from learning.relative_action_learning import annotated_transition, registered_landmarks, validate_rows

SCHEMA = 'motionbrain.pipeline_observations.v1'
NAMES = ('object', 'jawLeft', 'jawRight')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def _bytes(root, name, expected):
    raw = _read(root, name)
    require(hashlib.sha256(raw).hexdigest() == expected, 'observation_asset_hash_changed')
    return raw


def _image(root, frame):
    raw = _bytes(root/'native-camera', frame['imageFile'], frame['imageSha256'])
    pixels = decode(raw, frame['rawByteLength'], frame['pixelSha256'])
    image = np.frombuffer(pixels, np.uint8).reshape(frame['height'], frame['width'], 4)
    ok, png = cv2.imencode('.png', image[:, :, :3])
    require(ok, 'observation_png_encode_failed')
    return png.tobytes()


def _times(row):
    at, off = row['evidence']['commandAt'], row['evidence']['offReceiptAt']
    require(all(type(v) in (int, float) and math.isfinite(v) for v in (at, off)) and at < off,
            'observation_command_times_invalid')
    before, after = row['inputs']['image'], row['targets']['image']
    if 'gripperFrameEvidence' in row['evidence']:
        pre, post = (row['evidence']['gripperFrameEvidence'][k] for k in ('before', 'after'))
        require(pre['persistedAt'] <= at-.1 and post['requestStartedAt'] >= off+3.,
                'observation_frame_not_causal')
    else:
        require(at+POLICY['preSeconds'][0] <= before['sourceEarliest'] <= before['sourceLatest'] <= at-.1
                and before['stdoutReadAt']+before['persistenceMs']/1000 < at,
                'observation_input_not_available_before_command')
        require(off+3. <= after['sourceEarliest'] <= after['sourceLatest'] <= off+6.,
                'observation_target_outside_fixed_window')


def _manual(annotation, frame):
    require(annotation.get('frameId') == frame['frameId']
            and annotation.get('pixelSha256') == frame['pixelSha256']
            and annotation.get('inputFrameOnly') is True
            and annotation.get('sourceFrameIds') == [frame['frameId']],
            'manual_annotation_frame_or_source_mismatch')
    require(annotation.get('method') == 'analyst_points' and annotation.get('accepted') is True,
            'explicit_manual_annotation_acceptance_required')
    points = annotation.get('points', {})
    require(isinstance(points, dict) and not set(points)-set(NAMES), 'manual_annotation_points_invalid')
    diagnostics = {}
    for name in NAMES:
        xy = points.get(name)
        valid = (isinstance(xy, list) and len(xy) == 2
                 and all(type(v) in (int, float) and math.isfinite(v) for v in xy)
                 and 0 <= xy[0] < frame['width'] and 0 <= xy[1] < frame['height'])
        diagnostics[name] = dict(status='valid' if valid else 'missing_or_invalid',
                                 point=copy.deepcopy(xy) if valid else None)
    return copy.deepcopy(annotation), diagnostics


def observe_transition(source_root, audited_row, observer_spec, *, artifact_root=None):
    """원본 감사 행과 별도 관측 계약을 연결하고 결함을 명시적으로 반환한다.

    analyst_points.annotations의 각 항목은 sourceFrameIds=[그 프레임 ID]를 선언해야
    한다. 이는 주석 출처 선언이며 물리 정확도 또는 분석가의 행위 증명이 아니다.
    """
    root = Path(source_root).resolve()
    assets = root if artifact_root is None else Path(artifact_root).resolve()
    result = dict(schemaVersion=SCHEMA, status='rejected', transition=None, perFrame={},
                  contractSha256=None, recoveryDevelopmentOnly=True, trainingEligible=False,
                  controlEligible=False, sourceRoot=str(root), sourceHash=None,
                  originalOutcomeChanged=False)
    try:
        source_hash = hashlib.sha256(_read(root, 'session.json')).hexdigest()
        result['sourceHash'] = source_hash
        require(audited_row.get('sourceHash', source_hash) == source_hash, 'audited_source_hash_changed')
        row = copy.deepcopy(audited_row)
        spec = copy.deepcopy(observer_spec)
        if spec.get('kind') == 'development_gripper':
            from learning.development_gripper_source import load_observer, observe_saved_pair, scene_context
            require(set(spec) == {'kind', 'context', 'captureGroup', 'gripperPairRoot',
                                 'observerBindingPath', 'observerBindingSha256', 'cameraBootId'},
                    'development_pipeline_observer_fields')
            require(isinstance(spec['captureGroup'], str) and spec['captureGroup'],
                    'development_capture_group_required')
            observer = load_observer(assets, spec['observerBindingPath'], spec['observerBindingSha256'])
            require(spec['context'] == scene_context(observer), 'development_pipeline_context_changed')
            # 상대 경로 검사는 기존 원본 읽기 정책을 재사용한다.
            _read(assets, spec['gripperPairRoot']+'/esp32-before.json')
            pair = observe_saved_pair(observer, row, assets/spec['gripperPairRoot'],
                actual_scene=spec['context'], camera_boot_id=spec['cameraBootId'])
            result.update(status='development_only' if pair['status'] == 'complete_development'
                          else 'partial_development', contractSha256=observer.binding,
                perFrame=pair['perFrame'], developmentObservation=pair,
                reason=pair['reason'], transition=None, trainingEligible=False)
            return result
        pair_root = spec.get('gripperPairRoot')
        if pair_root is not None:
            # _read's relative-path validation prevents absolute paths and traversal.
            _read(assets, pair_root+'/esp32-before.json')
            row = bind_gripper_pair(row, assets/pair_root)
        _times(row)
        kind = spec['kind']; context = copy.deepcopy(spec['context'])
        require(isinstance(context, dict) and set(context) in
                ({'physicalView', 'object', 'poseRegime'},
                 {'physicalView', 'object', 'poseRegime', 'landmarks'}), 'observation_context_required')
        require(all(isinstance(v, str) and v for v in context.values()), 'observation_context_required')
        tracker = None
        contract = dict(kind=kind, context=context.copy())
        if kind == 'analyst_points':
            require('landmarks' in context and not registered_landmarks(context['landmarks']),
                    'analyst_must_not_impersonate_registered_contract')
            contract['annotationProtocol'] = spec.get('annotationProtocol', 'frame_only_analyst_points.v1')
            require(contract['annotationProtocol'] == 'frame_only_analyst_points.v1',
                    'unsupported_manual_annotation_protocol')
        else:
            from learning.registered_scene_landmarks import RegisteredSceneLandmarks
            from learning.registered_rigid_scene_landmarks import RegisteredRigidSceneLandmarks
            from learning.registered_gripper_landmarks import RegisteredGripperLandmarks
            from learning.registered_gripper_crosssection import RegisteredGripperCrosssection
            cls = {'registered_scene': RegisteredSceneLandmarks,
                   'registered_rigid': RegisteredRigidSceneLandmarks,
                   'registered_gripper': RegisteredGripperLandmarks,
                   'registered_gripper_crosssection': RegisteredGripperCrosssection}.get(kind)
            require(cls is not None, 'unsupported_observer_kind')
            registration = _decode_json(_bytes(assets, spec['registrationPath'], spec['registrationSha256']))
            reference = _bytes(assets, spec['referenceImagePath'], spec['referenceImageSha256'])
            tracker = cls(registration, reference)
            require(context.get('landmarks', tracker.landmark_contract) == tracker.landmark_contract,
                    'registered_context_contract_changed')
            context['landmarks'] = tracker.landmark_contract
            if 'sceneContext' in registration:
                require({k: context[k] for k in ('physicalView', 'object', 'poseRegime')}
                        == registration['sceneContext'], 'registration_scene_context_changed')
            contract.update(context=context, registrationSha256=spec['registrationSha256'],
                            referenceImageSha256=spec['referenceImageSha256'])
        result['contractSha256'] = digest(contract)
        annotations = {}
        for side in ('before', 'after'):
            frame = row['inputs' if side == 'before' else 'targets']['image']
            record = dict(frameId=frame['frameId'], pixelSha256=frame['pixelSha256'],
                          role='input' if side == 'before' else 'target', status='rejected')
            result['perFrame'][side] = record
            try:
                if pair_root:
                    name = 'esp32-'+side+'.jpg'
                    blob = _bytes(assets/pair_root, name, frame['imageSha256'])
                    binding = dict(path=str(assets/pair_root/name), encoding='jpeg')
                else:
                    blob = _image(root, frame)
                    binding = dict(path=str(root/'native-camera'/frame['imageFile']),
                                   encoding='native_bgra_lossless', rawByteLength=frame['rawByteLength'])
                binding.update(frameId=frame['frameId'], imageSha256=frame['imageSha256'],
                    pixelSha256=frame['pixelSha256'], width=frame['width'], height=frame['height'])
                record['imageBinding'] = binding
                if tracker is None:
                    annotation, points = _manual(spec['annotations'][side], frame)
                else:
                    observed = tracker.observe(blob, frame['frameId'])
                    annotation = observed['annotation']
                    points = {name: dict(status='valid', point=copy.deepcopy(annotation['points'][name]))
                              for name in NAMES}
                    record['quality'] = observed.get('quality', {})
                record['perPoint'] = points
                if all(v['status'] == 'valid' for v in points.values()):
                    record.update(status='valid', annotation=annotation)
                    annotations[side] = annotation
                else:
                    record['status'] = 'partial'
            except (ValueError, KeyError, TypeError, StorageError, OSError) as exc:
                record.update(reason=str(exc), perPoint={name: dict(status='unavailable') for name in NAMES})
        if len(annotations) != 2:
            result['status'] = 'partial' if any(v['status'] in ('valid', 'partial')
                                               for v in result['perFrame'].values()) else 'rejected'
            result['reason'] = 'complete_before_and_after_observations_required'
            return result
        transition = annotated_transition(row, dict(sessionId=row['sessionId'], context=context,
            captureGroup=spec['captureGroup'], **annotations))
        transition.update(sourceHash=source_hash, sourceRoot=str(root),
                          observationContractSha256=result['contractSha256'], recoveryDevelopmentOnly=True)
        validate_rows([transition])
        result.update(status='eligible', transition=transition, trainingEligible=True)
    except (ValueError, KeyError, TypeError, StorageError, OSError, cv2.error) as exc:
        result['reason'] = str(exc)
    return result
