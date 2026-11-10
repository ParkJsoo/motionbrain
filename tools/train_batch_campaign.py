#!/usr/bin/env python3
"""명시한 비공개 묶음 전이를 원본 재감사해 학습·개발 평가한다. 장치 접근 없음."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.artifacts import _decode_json
from learning.batch_learning import fit, evaluate, model_sha256, validate_model
from learning.batch_observation import digest
from learning.pipeline_evidence import ingest_session

SCHEMA = 'motionbrain.batch_training_manifest.v1'


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
                      allow_nan=False).encode()


def private_path(repository, name):
    require(isinstance(name, (str, Path)) and bool(str(name)), 'private_path_required')
    path = (repository / name).resolve()
    require(path.is_relative_to((repository / '.local').resolve()), 'private_path_required')
    return path


def read_hashed(path, expected):
    require(isinstance(expected, str) and len(expected) == 64 and
            all(c in '0123456789abcdef' for c in expected), 'artifact_sha256_required')
    raw = path.read_bytes()
    require(hashlib.sha256(raw).hexdigest() == expected, 'artifact_hash_changed')
    return _decode_json(raw)


def partition(row):
    return digest({k: row[k] for k in ('joint', 'context', 'stateFields')})


def load_transition(repository, entry, split):
    if isinstance(entry, dict) and entry.get('sourceKind') == 'm4_scope':
        from learning.batch_m4 import load_learning_transition
        return load_learning_transition(repository, entry, split)
    require(isinstance(entry, dict) and set(entry) == {
        'transitionPath', 'transitionSha256', 'sourcePath', 'sourceSha256'}, 'transition_manifest_fields')
    transition = read_hashed(private_path(repository, entry['transitionPath']), entry['transitionSha256'])
    require(isinstance(transition, dict) and transition.get('schemaVersion') ==
            'motionbrain.batch_visual_transition.v1', 'batch_transition_schema')
    require(transition.get('phase') == split and split in ('training', 'evaluation'), 'transition_split_changed')
    require(transition.get('sourceSha256') == entry['sourceSha256'], 'transition_source_hash_changed')
    source = private_path(repository, entry['sourcePath'])
    audit = ingest_session(source, entry['sourceSha256'])
    require(audit['execution']['status'] == 'verified' and audit['execution']['offVerified'] is True
            and audit['recording']['status'] == 'verified' and audit['visual']['status'] == 'verified',
            'original_source_not_eligible')
    require(audit.get('recoveryDevelopmentOnly') is False, 'failed_source_not_supported')
    row = transition.get('learningRow')
    require(isinstance(row, dict), 'transition_learning_row_required')
    original = audit['visual']['row']
    joint = transition.get('joint')
    context = transition.get('context')
    require(isinstance(context, dict) and set(context) == {'viewId', 'poseRegime', 'object', 'load'},
            'transition_context_fields')
    require(transition.get('selectionSemantics') ==
            'pinned_pre_window_frame_not_default_audit_input', 'pinned_input_semantics_required')
    session = _decode_json((source/'session.json').read_bytes())
    pinned = _decode_json((source/'batch-before-observation.json').read_bytes())
    require(isinstance(pinned, dict) and isinstance(pinned.get('source'), dict), 'pinned_input_required')
    frame = pinned['source'].get('frame')
    require(isinstance(frame, dict) and type(frame.get('frameIndex')) is int and
            0 <= frame['frameIndex'] < len(session['frames']) and
            session['frames'][frame['frameIndex']] == frame and
            frame['frameIndex'] in original['evidence']['preFrameIds'],
            'pinned_input_not_in_original_window')
    command_at = original['evidence']['commandAt']
    require(command_at-3 <= frame['sourceEarliest'] <= frame['sourceLatest'] <= command_at-.1 and
            frame['stdoutReadAt']+frame['persistenceMs']/1000 < command_at,
            'pinned_input_not_causal')
    require(transition.get('before') == pinned, 'transition_pinned_input_changed')
    states = {}
    observer_hash = None
    for side, ref in [('before', frame), ('after', original['targets']['image'])]:
        evidence = transition.get(side)
        require(isinstance(evidence, dict), 'transition_observation_required')
        stored = evidence.get('source', {})
        require(isinstance(stored.get('folder'), str) and Path(stored['folder']).resolve() ==
                (source / 'native-camera').resolve() and stored.get('frame') == ref,
                'transition_original_frame_changed')
        observed = evidence.get('observation', {})
        metadata = evidence.get('derivedMetadata')
        require(observed.get('status') == 'accepted' and observed.get('frameBinding') == metadata
                and isinstance(metadata, dict) and metadata.get('frameId') == ref['frameId']
                and metadata.get('capturedAt') == ref['sourceEarliest']
                and metadata.get('persistedAt') == ref['stdoutReadAt']+ref['persistenceMs']/1000, 'transition_frame_binding_changed')
        spec_hash = observed.get('specSha256')
        require(isinstance(spec_hash, str) and len(spec_hash) == 64 and
                all(c in '0123456789abcdef' for c in spec_hash), 'transition_observer_hash_required')
        require(observer_hash is None or observer_hash == spec_hash, 'transition_observer_changed')
        observer_hash = spec_hash
        require(joint in observed.get('states', {}), 'transition_joint_state_missing')
        states[side] = observed['states'][joint]
        if not transition.get('gripperPair', {}).get('before'):
            require(states[side].get('state') == transition.get(side+'State') and
                    states[side].get('stateFields') == transition.get('stateFields'),
                    'transition_derived_state_changed')
    expected = dict(sessionId=original['sessionId'], episodeId=':'.join(audit['execution']['episode']),
                    sourceHash=entry['sourceSha256'], frameIds=[frame['frameId'],
                    original['targets']['image']['frameId']], captureGroup=transition.get('captureGroup'),
                    joint=joint, context={**context, 'observerSha256': observer_hash},
                    stateFields=transition['stateFields'], before=transition['beforeState'],
                    after=transition['afterState'], command=audit['execution']['command'])
    if transition.get('gripperPair', {}).get('before'):
        from learning.batch_runtime import audit_pair_transition
        fused = audit_pair_transition(source, original, transition, repository)
        require(all(transition[k] == fused[k] for k in ('beforeState','afterState','stateFields')),
                'transition_fused_state_changed')
        expected.update(stateFields=fused['stateFields'], before=fused['beforeState'], after=fused['afterState'],
                        context={**expected['context'], **fused['extraContext']},
                        pairedFrameIds=fused['pairedFrameIds'], pairedSourceHashes=fused['pairedSourceHashes'])
    if transition.get('supplementalObservations', {}).get('status') == 'recorded':
        from learning.batch_runtime import audit_retention_transition
        supplemental = audit_retention_transition(source, original, transition, repository)
        expected['supplementalFrameIds'] = supplemental
    require(transition.get('command') == expected['command'] and
            transition.get('sessionId') == expected['sessionId'] and
            transition.get('episodeId') == audit['execution']['episode'] and
            transition.get('frameIds') == expected['frameIds'], 'transition_execution_binding_changed')
    # Runtime may attach audited timing, but this offline command never creates
    # prospective claims from an after-state that it has already read.
    for name, value in [('commandAt', original['evidence']['commandAt']),
                        ('beforePersistedAt', frame['stdoutReadAt']+frame['persistenceMs']/1000)]:
        if name in row:
            expected[name] = value
    require(row == expected, 'transition_learning_row_changed')
    return copy.deepcopy(row), dict(transitionSha256=entry['transitionSha256'],
                                   sourceSha256=entry['sourceSha256'], auditStageKey=audit['stageKey'])


def check_identities(training, evaluation):
    seen = {k: set() for k in ('sessionId', 'episodeId', 'sourceHash', 'frameIds')}
    groups = set()
    for split, rows in [('training', training), ('evaluation', evaluation)]:
        current_groups = set()
        for row in rows:
            for key in seen:
                values = set(row[key]+row.get('pairedFrameIds', [])+row.get('supplementalFrameIds', [])) if key == 'frameIds' else {row[key]}
                require(not (seen[key] & values), 'manifest_identity_overlap:'+key)
                seen[key].update(values)
            current_groups.add(row['captureGroup'])
        if split == 'evaluation':
            require(not groups & current_groups, 'manifest_capture_group_overlap')
        groups.update(current_groups)


def run_manifest(manifest_path, output, *, repository=ROOT):
    repository = Path(repository).resolve()
    manifest_path = private_path(repository, manifest_path)
    output = private_path(repository, output)
    manifest_raw = manifest_path.read_bytes()
    manifest = _decode_json(manifest_raw)
    require(isinstance(manifest, dict) and set(manifest) <= {
        'schemaVersion', 'training', 'evaluation', 'frozenModels'} and manifest.get('schemaVersion') == SCHEMA,
        'batch_training_manifest_schema')
    for key in ('training', 'evaluation', 'frozenModels'):
        require(isinstance(manifest.get(key, []), list), 'manifest_list_required:'+key)
    require(bool(manifest.get('training')) != bool(manifest.get('frozenModels')),
            'choose_training_or_frozen_models')
    require(not output.exists(), 'new_output_directory_required')
    require(not (manifest_path.is_relative_to(output) or output == manifest_path.parent),
            'output_must_not_contain_input_manifest')
    loaded, provenance = {}, {}
    for split in ('training', 'evaluation'):
        loaded[split], provenance[split] = [], []
        for entry in manifest.get(split, []):
            row, evidence = load_transition(repository, entry, split)
            loaded[split].append(row); provenance[split].append(evidence)
    check_identities(loaded['training'], loaded['evaluation'])
    grouped = {}
    for row in loaded['training']:
        grouped.setdefault(partition(row), []).append(row)
    models = {key: fit(rows) for key, rows in grouped.items()}
    for entry in manifest.get('frozenModels', []):
        require(isinstance(entry, dict) and set(entry) == {'path', 'sha256'}, 'frozen_model_manifest_fields')
        model = validate_model(read_hashed(private_path(repository, entry['path']), entry['sha256']))
        require(model_sha256(model) == entry['sha256'], 'canonical_model_hash_required')
        key = partition(model)
        require(key not in models, 'duplicate_frozen_model_context')
        models[key] = model
    evaluations = {}
    for row in loaded['evaluation']:
        evaluations.setdefault(partition(row), []).append(row)
    report = dict(schemaVersion='motionbrain.batch_training_report.v1', kind='developmentReplay',
                  manifestSha256=hashlib.sha256(manifest_raw).hexdigest(),
                  trainingRows=len(loaded['training']), evaluationRows=len(loaded['evaluation']),
                  provenance=provenance, models=[], unmatchedEvaluation=[],
                  deviceAccess=False, credentialAccess=False, motorCommands=0,
                  prospectiveEvidenceVerified=False, observationRecomputed=False,
                  evaluationPassed=None, controlEligible=False)
    serialized = {}
    for key, model in models.items():
        sha = model_sha256(model)
        raw = canonical(model)
        require(hashlib.sha256(raw).hexdigest() == sha, 'canonical_model_encoding_changed')
        name = 'models/'+sha+'.json'
        serialized[name] = raw
        rows = evaluations.pop(key, [])
        result = evaluate(model, rows) if rows else None
        report['models'].append(dict(partitionId=key, joint=model['joint'], context=model['context'],
                                     stateFields=model['stateFields'], path=name, sha256=sha,
                                     supportedCommands=len(model['models']), deficits=model['deficits'],
                                     evaluation=result))
    for key, rows in evaluations.items():
        report['unmatchedEvaluation'].append(dict(partitionId=key, listed=len(rows), reason='no_matching_model'))
    # Validate all inputs and run fitting/evaluation before creating any output.
    output.mkdir(parents=True, exist_ok=False)
    for name, raw in serialized.items():
        target = output/name; target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as stream:
            stream.write(raw)
    report_raw = canonical(report)
    report_sha = hashlib.sha256(report_raw).hexdigest()
    with (output/'report.json').open('xb') as stream:
        stream.write(report_raw)
    with (output/'latest.json').open('xb') as stream:
        stream.write(canonical(dict(report='report.json', sha256=report_sha)))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        report = run_manifest(args.manifest, args.output)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print(json.dumps(dict(trainingRows=report['trainingRows'], evaluationRows=report['evaluationRows'],
                          models=len(report['models']), report=str(args.output/'report.json'),
                          kind=report['kind'], controlEligible=False, motorCommands=0), ensure_ascii=False))


if __name__ == '__main__':
    main()
