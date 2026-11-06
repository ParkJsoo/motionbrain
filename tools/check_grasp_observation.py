#!/usr/bin/env python3
"""집게 카메라 원본과 고정 모델의 관측 계약을 무출력으로 검사한다."""
from __future__ import annotations

import argparse
import copy
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.grasp_observation import GripperObservationSource
from learning.gripper_camera_dataset import POLICY, read_frame
from learning.manual_collection_dataset import require
from learning.relative_action_learning import STATE, point_state, predict
from learning.relative_model_bundle import load_frozen_model, sha256

SCHEMA = 'motionbrain.grasp_observation_check.v1'


def read_binding(path, expected_hash, config):
    """Reaudit a saved source binding; never replace original source times."""
    path = Path(path)
    raw = path.read_bytes()
    require(sha256(raw) == expected_hash, 'source_binding_hash_changed')
    binding = _decode_json(raw)
    record = binding['record']
    root = path.parent
    require(binding['metadataSha256'] == sha256(_read(root, 'esp32-frame.json'))
            and binding['imageSha256'] == sha256(_read(root, 'esp32-frame.jpg')), 'source_bytes_changed')
    frame, metadata, boot = read_frame(root, 'frame', config['sessionId'])
    require(boot == config['expectedCameraBootId'], 'camera_boot_binding_changed')
    context = dict(config['sceneContext'], cameraIdentity=dict(camera='gripper', cameraBootId=boot,
        timestampSemantics='first_dma_buffer', observationPolicy=copy.deepcopy(POLICY)),
        imageShape=[frame['height'], frame['width']], stateFields=copy.deepcopy(STATE))
    expected = dict(frame={k: frame[k] for k in ('frameId', 'pixelSha256', 'width', 'height')},
        context=context, hostClockId=config['sessionId'], sourceEarliest=metadata['requestStartedAt'],
        sourceLatest=metadata['receivedAt'], receivedAt=metadata['receivedAt'], persistedAt=metadata['persistedAt'])
    require(record == expected, 'source_binding_content_changed')
    return record


def assess(record, annotation, bindings):
    state, state_reason = None, 'point_review_not_provided'
    if annotation is not None:
        try:
            state = point_state(annotation, record['frame'])
            state_reason = 'accepted_saved_frame_points_only'
        except (ValueError, KeyError, TypeError) as exc:
            state_reason = str(exc) if isinstance(exc, ValueError) else 'invalid_point_review'
    models = []
    for binding in bindings:
        model = load_frozen_model(binding['path'], binding['sha256'])
        changed = sorted(k for k in set(model['context']) | set(record['context'])
                         if model['context'].get(k) != record['context'].get(k))
        entry = dict(id=binding['id'], modelSha256=binding['sha256'],
                     contextChangedFields=changed, commands=[])
        if changed:
            entry['status'] = 'observation_context_changed'
        elif state is None:
            entry['status'] = 'point_review_required'
        else:
            entry['status'] = 'queried'
            for candidate in model['models']:
                command = candidate['command']
                result = dict(command=command)
                try:
                    result.update(status='predicted', predictedState=predict(model, state, command, record['context']))
                except ValueError as exc:
                    result.update(status='rejected', reason=str(exc))
                entry['commands'].append(result)
        models.append(entry)
    return dict(state=state, stateStatus=state_reason, models=models,
                predictedCommands=sum(c['status'] == 'predicted' for m in models for c in m['commands']))


def run(config_path, output_dir, *, capture=False, source_binding=None, source_binding_sha256=None,
        annotation_path=None, annotation_sha256=None, root=ROOT, clock=time):
    require(Path(output_dir).resolve().is_relative_to((Path(root)/'.local').resolve()), 'private_output_required')
    require(type(capture) is bool and (capture != (source_binding is not None)), 'one_explicit_source_mode_required')
    require((source_binding is None) == (source_binding_sha256 is None), 'source_binding_and_hash_required')
    require((annotation_path is None) == (annotation_sha256 is None), 'annotation_and_hash_required')
    config_raw = Path(config_path).read_bytes(); config = _decode_json(config_raw)
    require(set(config) == {'schemaVersion', 'sessionId', 'cameraUrl', 'expectedCameraBootId',
                           'sceneContext', 'modelBindings', 'captureTimeoutSeconds'}
            and config['schemaVersion'] == SCHEMA, 'observation_check_config_fields')
    bindings = config['modelBindings']
    require(isinstance(bindings, list) and all(isinstance(b, dict) and set(b) == {'id', 'path', 'sha256'}
            and isinstance(b['id'], str) and b['id'] for b in bindings)
            and len({b['id'] for b in bindings}) == len(bindings), 'unique_model_bindings_required')
    # Validate every model before any camera GET.
    for binding in bindings:
        load_frozen_model(binding['path'], binding['sha256'])
    store = ArtifactStore(Path(output_dir))
    store.write_json('check-intent.json', dict(configSha256=sha256(config_raw), capture=capture,
        sourceBindingSha256=source_binding_sha256, annotationSha256=annotation_sha256,
        controllerWrites=0, trainingPerformed=False, controlEligible=False))
    source = None
    report = dict(schemaVersion=SCHEMA, status='failed', errors=[], controllerWrites=0,
        trainingPerformed=False, controlEligible=False, graspVerified=False, newCameraCaptures=0,
        liveControlReadinessVerified=False)
    try:
        if capture:
            source = GripperObservationSource(config['cameraUrl'], config['sessionId'],
                config['expectedCameraBootId'], config['sceneContext'], store,
                timeout=config['captureTimeoutSeconds'], max_captures=1, clock=clock)
            end = clock.monotonic() + config['captureTimeoutSeconds'] + 1.
            source.start(deadline=end)
            record = source.capture(deadline=end)
            report['newCameraCaptures'] = source.count
        else:
            record = read_binding(source_binding, source_binding_sha256, config)
        annotation = None
        if annotation_path is not None:
            raw = Path(annotation_path).read_bytes()
            require(sha256(raw) == annotation_sha256, 'annotation_hash_changed')
            annotation = _decode_json(raw)
        report.update(status='checked', observation=record,
            observationUse='saved_frame_diagnostic_not_live_dispatch_input',
            **assess(record, annotation, bindings))
    except (Exception, KeyboardInterrupt) as exc:
        report['errors'].append(dict(type=type(exc).__name__,
            reason=str(exc) if isinstance(exc, ValueError) else 'observation_check_failed'))
    finally:
        if source is not None:
            report['newCameraCaptures'] = source.count
            report['sourceFinalization'] = source.close()
            if report['sourceFinalization']['status'] != 'completed':
                report['status'] = 'failed'
        store.write_json('report.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--capture', action='store_true')
    mode.add_argument('--source-binding', type=Path)
    parser.add_argument('--source-binding-sha256')
    parser.add_argument('--annotation', type=Path)
    parser.add_argument('--annotation-sha256')
    args = parser.parse_args(argv)
    try:
        report = run(args.config, args.output_dir, capture=args.capture,
            source_binding=args.source_binding, source_binding_sha256=args.source_binding_sha256,
            annotation_path=args.annotation, annotation_sha256=args.annotation_sha256)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print(__import__('json').dumps(dict(status=report['status'],
        newCameraCaptures=report['newCameraCaptures'], controllerWrites=0,
        models=[dict(id=m['id'], status=m['status'], contextChangedFields=m['contextChangedFields'])
                for m in report.get('models', [])], controlEligible=False)))
    return 0 if report['status'] == 'checked' else 1


if __name__ == '__main__':
    raise SystemExit(main())
