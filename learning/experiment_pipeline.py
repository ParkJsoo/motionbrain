"""원본 → 독립 감사 → 관측 → 분리 학습/평가. 장치 통신이나 실행 권한 없음."""
from __future__ import annotations

import fcntl
import hashlib
import json
import os
from pathlib import Path
import platform
import uuid

from learning.artifacts import StorageError
from learning.pipeline_evidence import ingest_session
from learning.pipeline_observations import observe_transition
from learning.pipeline_learning import run_learning, canonical_bytes

SCHEMA = 'motionbrain.experiment_pipeline.v1'


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'),
                       allow_nan=False)+'\n').encode()


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def sync_directory(path):
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def atomic(path, raw):
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.pending')
    try:
        with temporary.open('xb') as out:
            out.write(raw); out.flush(); os.fsync(out.fileno())
        os.replace(temporary, path)
        sync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def artifact(root, value):
    raw = encoded(value); key = sha(raw); path = root/(key+'.json')
    if path.exists():
        if path.read_bytes() != raw:
            raise ValueError('existing_pipeline_artifact_corrupt')
    else:
        atomic(path, raw)
    return {'path': str(path.name), 'sha256': key}


def processor_identity():
    import cv2
    import numpy
    names = ('experiment_pipeline.py', 'pipeline_collection.py', 'manual_collection.py',
             'supervised_collection.py', 'pipeline_evidence.py', 'pipeline_observations.py',
             'pipeline_learning.py', 'manual_collection_dataset.py', 'relative_action_learning.py',
             'relative_model_bundle.py', 'registered_scene_landmarks.py', 'registered_rigid_scene_landmarks.py',
             'registered_gripper_landmarks.py', 'registered_gripper_crosssection.py',
             'arm_motion_observation.py', 'gripper_camera_dataset.py',
             'action_recorder.py', 'supervised_transport.py', 'supervised_measurement.py',
             'lossless_frame.py', 'mac_clock_bridge.py', 'artifacts.py', 'object_arc.py')
    sources = {name: sha(Path(__file__).with_name(name).read_bytes()) for name in names}
    return {'sources': sources, 'python': platform.python_version(),
            'opencv': cv2.__version__, 'numpy': numpy.__version__}


def run_pipeline(manifest_path, output):
    """선언된 모든 source를 매번 재감사한다. 재실행은 구동이나 중복 산출물을 만들지 않는다."""
    manifest_path = Path(manifest_path).resolve(); raw = manifest_path.read_bytes()
    manifest = json.loads(raw)
    if manifest.get('schemaVersion') != SCHEMA:
        raise ValueError('experiment_pipeline_manifest_required')
    if not set(manifest) <= {'schemaVersion', 'artifactRoot', 'sources', 'split', 'previousModels'}:
        raise ValueError('unexpected_manifest_fields')
    sources = manifest.get('sources')
    if not isinstance(sources, list) or not sources:
        raise ValueError('explicit_sources_required')
    ids = [s['id'] for s in sources]
    if any(not isinstance(x, str) or not x for x in ids) or len(ids) != len(set(ids)):
        raise ValueError('unique_source_ids_required')
    base = (manifest_path.parent/manifest.get('artifactRoot', '.')).resolve(strict=True)
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=True)
    stages = output/'artifacts'; stages.mkdir(exist_ok=True)
    with (output/'pipeline.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        records, transitions = [], []
        for source in sources:
            record = {'id': source['id'], 'evidence': None, 'observation': None,
                      'status': 'rejected', 'trainingEligible': False}
            try:
                if not set(source) <= {'id', 'path', 'sha256', 'allowFailedDevelopment', 'observer'}:
                    raise ValueError('unexpected_source_fields')
                source_root = (base/source['path']).resolve(strict=True)
                if not source_root.is_relative_to(base):
                    raise ValueError('source_path_outside_artifact_root')
                recovered = source.get('allowFailedDevelopment', False)
                if type(recovered) is not bool:
                    raise ValueError('recovery_option_must_be_boolean')
                evidence = ingest_session(source_root, source['sha256'],
                                          allow_failed_development=recovered)
                record['evidence'] = artifact(stages, evidence)
                visual = evidence['visual']
                if visual['status'] != 'verified':
                    record['reason'] = 'visual_evidence_'+visual['status']
                elif source.get('observer') is None:
                    record.update(status='awaiting_observation', reason='observer_not_configured')
                else:
                    observation = observe_transition(source_root, visual['row'], source['observer'],
                                                     artifact_root=base)
                    record['observation'] = artifact(stages, observation)
                    record['status'] = observation['status']
                    if observation.get('transition') is not None and observation['status'] == 'eligible':
                        transition = observation['transition']
                        # Source membership cannot be supplied by annotations or observer configuration.
                        transition['sourceHash'] = evidence['source']['sha256']
                        transition['recoveryDevelopmentOnly'] = bool(
                            transition.get('recoveryDevelopmentOnly', False)
                            or visual['row'].get('recoveryDevelopmentOnly', False))
                        transitions.append(transition)
                        record['trainingEligible'] = True
                    else:
                        record['reason'] = observation.get('reason', 'observation_not_eligible')
            except (ValueError, OSError, KeyError, TypeError, StorageError) as exc:
                record['reason'] = str(exc)
                record['errorType'] = type(exc).__name__
            records.append(record)
        try:
            learning = run_learning(transitions, manifest['split'],
                                    previous_models=manifest.get('previousModels'))
        except (ValueError, KeyError, TypeError) as exc:
            # A rejected split must leave collection/observation evidence inspectable.
            learning = {'status': 'rejected', 'reason': str(exc), 'partitions': [],
                        'controlEligible': False}
        models = output/'models'; models.mkdir(exist_ok=True)
        frozen = []
        for partition in learning.get('partitions', []):
            if partition.get('model') is None:
                continue
            model_raw = canonical_bytes(partition['model'])
            model_sha = sha(model_raw)
            if model_sha != partition['modelSha256']:
                raise ValueError('frozen_model_digest_disagrees')
            path = models/(model_sha+'.json')
            if path.exists() and path.read_bytes() != model_raw:
                raise ValueError('existing_frozen_model_corrupt')
            if not path.exists():
                atomic(path, model_raw)
            frozen.append({'partitionId': partition['partitionId'], 'sha256': model_sha,
                           'path': str(Path('models')/path.name)})
        processor = processor_identity()
        report = {'schemaVersion': SCHEMA, 'manifestSha256': sha(raw),
                  'processor': processor, 'processorSha256': sha(encoded(processor)), 'sources': records,
                  'counts': {'listed': len(records), 'eligible': len(transitions),
                             'unavailable': len(records)-len(transitions)},
                  'transitions': transitions, 'learning': learning, 'frozenModels': frozen,
                  'hardwareRequests': 0, 'motorCommands': 0, 'controlEligible': False,
                  'physicalGraspVerified': False, 'physicalLiftVerified': False}
        reference = artifact(stages, report)
        atomic(output/'latest.json', encoded(reference))
        return report
