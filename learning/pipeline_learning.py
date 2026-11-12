"""관측 전이를 구도·관절별로 학습하고 고정된 별도 자료에서 재현 평가한다.

장치 접근이나 제어 권한이 없다. OOD 진단은 기존 predict의 거절을 바꾸지
않으며, 이 모듈만으로 평가 자료의 실제 촬영 시점이나 실물 성능을 증명하지 않는다.
"""
from __future__ import annotations

import copy
import hashlib
import json
import re
from collections import Counter, defaultdict

import numpy as np

from learning import relative_action_learning as relative
from learning.relative_model_bundle import validate_model

SCHEMA = 'motionbrain.pipeline_learning.v1'


def canonical_bytes(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=False, allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _require(ok, reason):
    if not ok:
        raise ValueError(reason)


def partition_id(context, joint):
    return digest(dict(context=context, joint=joint))


def _manifest(manifest):
    _require(isinstance(manifest, dict), 'split_manifest_required')
    groups = []
    for name in ('trainGroups', 'evaluationGroups'):
        value = manifest.get(name)
        _require(isinstance(value, list) and all(isinstance(g, str) and g for g in value)
                 and len(value) == len(set(value)), 'explicit_unique_split_groups_required')
        groups.append(set(value))
    _require(not groups[0].intersection(groups[1]), 'training_evaluation_group_overlap')
    for flag in ('allowOodDiagnostics', 'allowRecoveredDevelopmentTraining', 'allowDevelopmentGripperTraining'):
        _require(type(manifest.get(flag, False)) is bool, 'split_flags_must_be_boolean')
    return groups


def _identity(row, allow_development):
    source = row.get('sourceHash')
    _require(isinstance(source, str) and re.fullmatch('[0-9a-f]{64}', source) is not None,
             'transition_source_hash_required')
    relative.validate_rows([row], allow_development_observations=allow_development)
    _require(type(row.get('recoveryDevelopmentOnly', False)) is bool,
             'recovery_flag_must_be_boolean')
    return dict(source={source}, session={row['sessionId']}, episode={tuple(row['episode'])},
                frame=set(row['frameIds']))


def _check_split_overlap(rows, groups, allow_development):
    identities = [{name: set() for name in ('source', 'session', 'episode', 'frame')}
                  for _ in range(2)]
    for row in rows:
        identity = _identity(row, allow_development)
        for index, group in enumerate(groups):
            if row['captureGroup'] in group:
                for name, values in identity.items():
                    identities[index][name].update(values)
    for name in identities[0]:
        _require(not identities[0][name].intersection(identities[1][name]),
                 'training_evaluation_overlap_' + name)


def _metrics(pairs):
    if not pairs:
        return dict(n=0, maePerStatePx=None, meanStateErrorPx=None)
    residual = np.asarray([np.asarray(prediction) - np.asarray(actual)
                           for prediction, actual in pairs], dtype=float)
    return dict(n=len(pairs), maePerStatePx=np.abs(residual).mean(axis=0).tolist(),
                meanStateErrorPx=float(np.linalg.norm(residual, axis=1).mean()))


def _command_entry(model, command):
    key = relative.command_key(command)
    entries = [m for m in model['models'] if relative.command_key(m['command']) == key]
    return entries[0] if entries else None


def _range_deficit(entry, state, context):
    return [dict(field=field, value=value, minimum=lo, maximum=hi,
                 below=max(0., lo-value), above=max(0., value-hi))
            for field, value, lo, hi in zip(relative.state_fields(context), state, entry['minimum'], entry['maximum'])
            if value < lo or value > hi]


def _offline_prediction(entry, state, *, ridge):
    x = relative.vector(state)
    delta = np.asarray(entry['deltaMean'])
    if ridge:
        delta = delta + ((x-np.asarray(entry['mean'])) / np.asarray(entry['scale'])) @ np.asarray(entry['coefficient'])
    result = (x+delta).tolist()
    relative.vector(result)
    return result


def _evaluate(model, rows, allow_ood):
    predictions, rejected, diagnostics = [], [], []
    all_baselines = dict(commandMean=[], noChange=[])
    common = dict(stateRidge=[], commandMean=[], noChange=[])
    for row in rows:
        x, y, command = row['before'], row['after'], row['command']
        entry = _command_entry(model, command) if model else None
        item = dict(sessionId=row['sessionId'], sourceHash=row['sourceHash'],
                    actualState=copy.deepcopy(y), beforeState=copy.deepcopy(x),
                    recoveryDevelopmentOnly=row.get('recoveryDevelopmentOnly', False),
                    stateRidge=None, commandMean=None, noChange=copy.deepcopy(x))
        all_baselines['noChange'].append((x, y))
        if entry:
            try:
                item['commandMean'] = _offline_prediction(entry, x, ridge=False)
                all_baselines['commandMean'].append((item['commandMean'], y))
            except ValueError as exc:
                item['commandMeanRejected'] = str(exc)
        try:
            _require(model is not None, 'no_supported_training_model')
            item['stateRidge'] = relative.predict(model, x, command, row['context'])
            # The strict ridge and the baselines use exactly the same cases here.
            if all(item[name] is not None for name in common):
                for name in common:
                    common[name].append((item[name], y))
        except ValueError as exc:
            failure = dict(sessionId=row['sessionId'], sourceHash=row['sourceHash'], reason=str(exc))
            if entry:
                failure['rangeDeficits'] = _range_deficit(entry, x, row['context'])
            rejected.append(failure)
            item['strictRejection'] = str(exc)
            if allow_ood and str(exc) == 'state_outside_training_range' and entry:
                diagnostic = dict(sessionId=row['sessionId'], sourceHash=row['sourceHash'],
                                  kind='offline_ood_diagnostic_only', strictRejection=str(exc),
                                  eligibleForEvaluationPass=False, controlEligible=False)
                try:
                    diagnostic['predictedState'] = _offline_prediction(entry, x, ridge=True)
                    diagnostic['absoluteErrorPx'] = np.abs(np.asarray(diagnostic['predictedState'])-y).tolist()
                except ValueError as error:
                    diagnostic['reason'] = str(error)
                diagnostics.append(diagnostic)
        predictions.append(item)
    metrics = {name: _metrics(pairs) for name, pairs in common.items()}
    n = metrics['stateRidge']['n']
    strict_count = sum(item['stateRidge'] is not None for item in predictions)
    return dict(kind='developmentReplay', temporalOrderVerified=False, listed=len(rows),
                posthocEvaluationRows=sum(r.get('recoveryDevelopmentOnly', False) for r in rows),
                inSupportEvaluated=strict_count, commonBaselineEvaluated=n,
                strictRejected=rejected, predictions=predictions,
                inSupportMetrics=metrics,
                baselinesAllRows={name: _metrics(pairs) for name, pairs in all_baselines.items()},
                oodDiagnostics=diagnostics,
                beatsBothBaselinesOnCommonCases=bool(n and all(
                    metrics['stateRidge']['meanStateErrorPx'] < metrics[name]['meanStateErrorPx']
                    for name in ('commandMean', 'noChange'))),
                evaluationPassed=None, controlEligible=False,
                independentGeneralizationProven=False)


def _previous_models(values, allow_development):
    if values is None:
        return []
    if isinstance(values, dict):
        values = list(values.values())
    _require(isinstance(values, list), 'previous_models_must_be_list_or_mapping')
    result = []
    for item in values:
        _require(isinstance(item, dict), 'previous_model_required')
        model = item.get('model', item)
        model = validate_model(model, allow_development_observations=allow_development)
        if 'model' in item and 'modelSha256' in item:
            _require(digest(model) == item['modelSha256'], 'previous_model_canonical_hash_changed')
        result.append(model)
    return result


def _previous_comparison(current, previous, rows):
    """Compare on the common strict-support subset, never extrapolated cases."""
    old_sources = dict(session=set(previous['trainingSessionIds']),
                       group=set(previous['trainingCaptureGroups']),
                       episode={tuple(e) for e in previous['trainingEpisodes']},
                       frame=set(previous['trainingFrameIds']))
    for row in rows:
        overlap = (row['sessionId'] in old_sources['session']
                   or row['captureGroup'] in old_sources['group']
                   or tuple(row['episode']) in old_sources['episode']
                   or bool(set(row['frameIds']).intersection(old_sources['frame'])))
        _require(not overlap, 'previous_training_evaluation_overlap')
    pairs = dict(current=[], previous=[])
    rejected = []
    for row in rows:
        try:
            _require(current is not None, 'no_supported_training_model')
            a = relative.predict(current, row['before'], row['command'], row['context'])
            b = relative.predict(previous, row['before'], row['command'], row['context'])
            pairs['current'].append((a, row['after'])); pairs['previous'].append((b, row['after']))
        except ValueError as exc:
            rejected.append(dict(sessionId=row['sessionId'], reason=str(exc)))
    metrics = {name: _metrics(values) for name, values in pairs.items()}
    status = 'unavailable'
    if metrics['current']['n']:
        a, b = [metrics[name]['meanStateErrorPx'] for name in ('current', 'previous')]
        status = 'improved' if a < b else 'regressed' if a > b else 'equal'
    return dict(previousModelSha256=digest(previous), listed=len(rows),
                commonEvaluated=metrics['current']['n'], rejected=rejected,
                metrics=metrics, status=status, controlEligible=False,
                automaticPromotion=False)


def run_learning(transitions, split_manifest, previous_models=None):
    """Process an explicit immutable split; no random split, transport, or overwrite.

    Rows use relative_action_learning's transition contract plus sourceHash.
    recoveryDevelopmentOnly rows require opt-in for training. Evaluation includes
    them as explicitly posthoc development replay, never prospective fixed proof.
    Model hashes use canonical_bytes, not arbitrary file layout.
    """
    _require(isinstance(transitions, list), 'transitions_must_be_list')
    groups = _manifest(split_manifest)
    allow_development = split_manifest.get('allowDevelopmentGripperTraining', False)
    _check_split_overlap(transitions, groups, allow_development)
    old_models = _previous_models(previous_models, allow_development)
    partitions, excluded = defaultdict(lambda: dict(training=[], evaluation=[])), []
    for row in transitions:
        split = 'training' if row['captureGroup'] in groups[0] else 'evaluation' if row['captureGroup'] in groups[1] else None
        reason = None
        if split is None:
            reason = 'group_not_selected'
        elif row.get('recoveryDevelopmentOnly', False):
            if split == 'training' and not split_manifest.get('allowRecoveredDevelopmentTraining', False):
                reason = 'recovered_training_requires_explicit_opt_in'
        if reason:
            excluded.append(dict(sessionId=row['sessionId'], sourceHash=row['sourceHash'],
                                 captureGroup=row['captureGroup'], reason=reason))
            continue
        key = partition_id(row['context'], row['command']['joint'])
        partitions[key][split].append(copy.deepcopy(row))
    results = []
    for key in sorted(partitions):
        partition = partitions[key]
        training, evaluation = partition['training'], partition['evaluation']
        sample = (training or evaluation)[0]
        context, joint = sample['context'], sample['command']['joint']
        if training:
            relative.validate_rows(training, allow_development_observations=allow_development)
        if evaluation:
            relative.validate_rows(evaluation, allow_development_observations=allow_development)
        counts = Counter(relative.command_key(r['command']) for r in training)
        observed_commands = set(counts) | {relative.command_key(r['command']) for r in evaluation}
        deficits = [dict(kind='command_training_support', command=dict(zip(
            ('joint', 'direction', 'percent', 'durationMs'), command)), count=count,
            minimum=2, missing=max(0, 2-count)) for command in sorted(observed_commands)
                    for count in [counts[command]] if count < 2]
        coverage = []
        for command in sorted(observed_commands):
            selected = [r for r in training if relative.command_key(r['command']) == command]
            states = np.asarray([r['before'] for r in selected], dtype=float)
            coverage.append(dict(command=dict(zip(('joint', 'direction', 'percent', 'durationMs'), command)),
                rows=len(selected), captureGroups=sorted({r['captureGroup'] for r in selected}),
                minimum=states.min(axis=0).tolist() if selected else None,
                maximum=states.max(axis=0).tolist() if selected else None,
                stateVariationRank=int(np.linalg.matrix_rank(states-states.mean(axis=0))) if selected else 0,
                supportIsAccuracyOrGeneralizationProof=False))
        model = validate_model(relative.fit(training, allow_development_observations=allow_development),
                               allow_development_observations=allow_development) if training else None
        if model is not None and not model['models']:
            model = None
        if not training:
            deficits.append(dict(kind='no_training_rows'))
        if not evaluation:
            deficits.append(dict(kind='no_separate_evaluation_rows'))
        report = _evaluate(model, evaluation, split_manifest.get('allowOodDiagnostics', False))
        comparisons = []
        for old in old_models:
            if old['context'] != context or not any(m['command']['joint'] == joint for m in old['models']):
                continue
            try:
                comparisons.append(_previous_comparison(model, old, evaluation))
            except ValueError as exc:
                comparisons.append(dict(previousModelSha256=digest(old), status='unavailable',
                                        reason=str(exc), controlEligible=False, automaticPromotion=False))
        results.append(dict(partitionId=key, context=copy.deepcopy(context), joint=joint,
            status='trained' if model else 'unavailable',
            counts=dict(training=len(training), evaluation=len(evaluation)),
            trainingMembership=[dict(sessionId=r['sessionId'], sourceHash=r['sourceHash'],
                                     captureGroup=r['captureGroup'], episode=r['episode'], frameIds=r['frameIds'])
                                for r in training],
            model=model, modelSha256=digest(model) if model else None,
            modelHashSemantics='canonical_json_sha256', evaluation=report,
            previousComparison=comparisons, deficits=deficits, trainingCoverage=coverage, controlEligible=False))
    result = dict(schemaVersion=SCHEMA, splitManifest=copy.deepcopy(split_manifest),
                  transitionsSha256=digest(transitions), counts=dict(listed=len(transitions),
                    training=sum(p['counts']['training'] for p in results),
                    evaluation=sum(p['counts']['evaluation'] for p in results), excluded=len(excluded)),
                  excluded=excluded, partitions=results, trainingPerformed=any(p['model'] is not None for p in results),
                  controlEligible=False, hardwareExecutionPerformed=False)
    result['reportSha256'] = digest(result)
    return result
