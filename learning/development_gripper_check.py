"""개발 관측기의 저장 원본/명시적 단일 읽기전용 촬영 검사. 모터 경로는 없다."""
from __future__ import annotations

import copy
import math
from pathlib import Path
import time

from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.batch_observation import require
from learning.development_gripper_source import load_observer, observe_bound_frame, scene_context
from learning.development_gripper_observer import sha
from learning.grasp_observation import GripperObservationSource
from learning.gripper_camera_dataset import read_frame

SCHEMA = 'motionbrain.development_gripper_source_check.v1'
ROOT = Path(__file__).resolve().parents[1]


def run_check(config_path, output_dir, *, frame_root=None, capture=False, root=ROOT,
              clock=time, source_factory=GripperObservationSource):
    private = (Path(root)/'.local').resolve()
    output = Path(output_dir).resolve()
    require(output.is_relative_to(private) and output != private, 'development_check_private_output')
    require(type(capture) is bool and capture != (frame_root is not None), 'development_check_one_source_mode')
    raw = Path(config_path).read_bytes(); config = _decode_json(raw)
    require(isinstance(config, dict) and set(config) == {'schemaVersion', 'assetRoot', 'observerBindingPath',
        'observerBindingSha256', 'cameraBootId', 'sessionId', 'actualScene', 'maxAgeSeconds', 'cameraUrl'} and
        config['schemaVersion'] == SCHEMA, 'development_check_config')
    assets = Path(config['assetRoot']).resolve()
    require(assets.is_relative_to(private) and assets != private, 'development_check_private_assets')
    observer = load_observer(assets, config['observerBindingPath'], config['observerBindingSha256'])
    require(config['actualScene'] == scene_context(observer), 'development_check_actual_scene_changed')
    # 생성자는 장치에 접근하지 않는다. 모든 정적 원점·부팅·시계 조건을 촬영 전에 검사한다.
    GripperObservationSource(config['cameraUrl'], config['sessionId'], config['cameraBootId'],
        config['actualScene'], None, timeout=6., max_captures=1, clock=clock)
    require(type(config['maxAgeSeconds']) in (int, float) and math.isfinite(config['maxAgeSeconds']) and
            .1 <= config['maxAgeSeconds'] <= 10, 'development_source_age_policy')
    if frame_root is not None:
        frame_root = Path(frame_root).resolve()
        require(frame_root.is_relative_to(private) and frame_root != private, 'development_check_private_source')
    store = ArtifactStore(output)
    store.write_json('check-intent.json', dict(configSha256=sha(raw), capture=capture,
        sourceMode='current_readonly_capture' if capture else 'saved_receipt_replay',
        hardwareMotionCommands=0, trainingPerformed=False))
    result = dict(schemaVersion=SCHEMA, status='failed', configSha256=sha(raw),
        sourceMode='current_readonly_capture' if capture else 'saved_receipt_replay',
        savedReceiptReplayOnly=not capture, newCameraCaptures=0, hardwareMotionCommands=0,
        currentFreshnessVerified=False,
        trainingPerformed=False, trainingEligible=False, controlEligible=False,
        liveAdmissionVerified=False, motionAdmissionVerified=False,
        exposureSynchronizationVerified=False, observation=None)
    source = None
    try:
        if capture:
            source = source_factory(config['cameraUrl'], config['sessionId'], config['cameraBootId'],
                copy.deepcopy(config['actualScene']), store, timeout=6., max_captures=1, clock=clock)
            source.start(deadline=clock.monotonic()+6.)
            record = source.capture(deadline=clock.monotonic()+6.)
            source.verify(record)
            source_folder = source._bindings[record['frame']['frameId']]['folder']
            frame_root = store.root/source_folder
            now = clock.monotonic()
        else:
            _, metadata, _ = read_frame(frame_root, 'frame', config['sessionId'])
            now = metadata['persistedAt']
        observation = observe_bound_frame(observer, frame_root, 'frame', session_id=config['sessionId'],
            camera_boot_id=config['cameraBootId'], actual_scene=config['actualScene'], now=now,
            max_age_seconds=config['maxAgeSeconds'])
        result.update(status='checked_readonly_development', observation=observation,
                      currentFreshnessVerified=capture)
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) if isinstance(exc, ValueError) else type(exc).__name__
    finally:
        if source is not None:
            result['newCameraCaptures'] = source.count
            result['sourceFinalization'] = source.close()
            if result['sourceFinalization']['status'] != 'completed':
                result['status'] = 'failed'
        store.write_json('report.json', result)
    return result
