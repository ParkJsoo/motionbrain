"""관절·관측 계약별 임의 차원 국소 전이 학습. 장치·자동 재학습 접근 없음."""
from __future__ import annotations

import copy
import math

import numpy as np

from learning.batch_observation import JOINTS, digest, number, require

SCHEMA = 'motionbrain.batch_state_ridge.v1'


def _context(context):
    require(isinstance(context, dict) and
            {'viewId', 'poseRegime', 'object', 'load', 'observerSha256'} <= set(context) and
            all(isinstance(context[k], str) and context[k] for k in
                ('viewId', 'poseRegime', 'object', 'load')), 'learning_observer_context_required')
    value = context['observerSha256']
    require(isinstance(value, str) and len(value) == 64 and
            all(c in '0123456789abcdef' for c in value), 'learning_observer_hash_required')


def vector(value, size):
    require(isinstance(value, list) and len(value) == size and all(number(x) for x in value),
            'finite_state_dimension_required')
    return np.asarray(value, dtype=float)


def command_key(command):
    require(isinstance(command, dict) and set(command) == {'joint', 'direction', 'percent', 'durationMs'},
            'command_fields')
    require(command['joint'] in JOINTS and isinstance(command['direction'], str) and
            bool(command['direction']) and type(command['percent']) is int and
            1 <= command['percent'] <= 100 and type(command['durationMs']) is int and
            command['durationMs'] > 0, 'command_values')
    # Learning a recorded command does not authorize its execution or widen profiles.
    return digest(command)


def _rows(rows):
    require(isinstance(rows, list) and bool(rows), 'transition_rows_required')
    first = rows[0]
    require(isinstance(first, dict), 'transition_row_required')
    fields = first.get('stateFields')
    require(isinstance(fields, list) and bool(fields) and all(isinstance(v, str) and v for v in fields)
            and len(set(fields)) == len(fields), 'unique_state_fields_required')
    require(first.get('joint') in JOINTS, 'joint_context_required')
    _context(first.get('context'))
    seen = {k:set() for k in ('sessionId', 'episodeId', 'frameIds', 'sourceHash')}
    for row in rows:
        require(isinstance(row, dict) and all(row.get(k) == first[k]
                for k in ('joint', 'context', 'stateFields')), 'learning_context_changed')
        for key in ('sessionId', 'episodeId', 'captureGroup', 'sourceHash'):
            require(isinstance(row.get(key), str) and bool(row[key]), 'transition_identity_required:'+key)
        require(len(row['sourceHash']) == 64 and all(c in '0123456789abcdef' for c in row['sourceHash']),
                'source_hash_required')
        refs = row.get('frameIds')
        require(isinstance(refs, list) and len(refs) == 2 and all(isinstance(v, str) and v for v in refs)
                and len(set(refs)) == 2, 'before_after_frame_ids_required')
        for key in seen:
            identity = set(refs) if key == 'frameIds' else {row[key]}
            require(not seen[key] & identity, 'duplicate_training_identity:'+key)
            seen[key].update(identity)
        vector(row.get('before'), len(fields)); vector(row.get('after'), len(fields))
        command_key(row.get('command'))
        require(row['command']['joint'] == row['joint'], 'command_joint_changed')
    return first, seen


def model_sha256(model):
    validate_model(model)
    return digest(model)


def validate_model(model):
    """Reject malformed frozen arrays before NumPy can broadcast them silently."""
    fields = {'schemaVersion', 'joint', 'context', 'stateFields', 'alpha', 'models',
              'deficits', 'trainingIdentities', 'trainingCaptureGroups', 'controlEligible', 'semantics'}
    require(isinstance(model, dict) and set(model) == fields and model['schemaVersion'] == SCHEMA,
            'frozen_model_schema')
    require(model['joint'] in JOINTS and model['alpha'] == 1. and
            model['controlEligible'] is False, 'frozen_model_contract')
    _context(model['context'])
    names = model['stateFields']
    require(isinstance(names, list) and bool(names) and all(isinstance(n, str) and n for n in names)
            and len(set(names)) == len(names), 'frozen_state_fields')
    size = len(names)
    require(isinstance(model['models'], dict) and isinstance(model['deficits'], dict), 'frozen_model_entries')
    require(not set(model['models']) & set(model['deficits']), 'frozen_model_deficit_overlap')
    for key, entry in model['models'].items():
        require(isinstance(entry, dict) and set(entry) == {'command', 'count', 'mean', 'scale',
                'deltaMean', 'coefficient', 'minimum', 'maximum'}, 'frozen_entry_fields')
        require(command_key(entry['command']) == key and entry['command']['joint'] == model['joint'],
                'frozen_command_binding')
        require(type(entry['count']) is int and entry['count'] >= 2, 'frozen_support_count')
        for name in ('mean', 'scale', 'deltaMean', 'minimum', 'maximum'):
            vector(entry[name], size)
        require(all(v >= 1. for v in entry['scale']), 'frozen_scale_invalid')
        require(all(lo <= mid <= hi for lo, mid, hi in
                    zip(entry['minimum'], entry['mean'], entry['maximum'])), 'frozen_bounds_invalid')
        coefficients = entry['coefficient']
        require(isinstance(coefficients, list) and len(coefficients) == size,
                'frozen_coefficient_dimension')
        for values in coefficients:
            vector(values, size)
    for key, deficit in model['deficits'].items():
        require(isinstance(deficit, dict) and set(deficit) == {'command', 'count'} and
                command_key(deficit['command']) == key and deficit['command']['joint'] == model['joint'] and
                type(deficit['count']) is int and deficit['count'] == 1, 'frozen_deficit_invalid')
    identities = model['trainingIdentities']
    require(isinstance(identities, dict) and
            set(identities) == {'sessionId', 'episodeId', 'frameIds', 'sourceHash'}, 'frozen_training_identities')
    for values in [*identities.values(), model['trainingCaptureGroups']]:
        require(isinstance(values, list) and bool(values) and
                all(isinstance(v, str) and v for v in values) and len(set(values)) == len(values),
                'frozen_training_identity_values')
    require(isinstance(model['semantics'], str) and bool(model['semantics']), 'frozen_semantics_required')
    digest(model)  # Also excludes non-finite values in nested context and metadata.
    return copy.deepcopy(model)


def fit(rows):
    """One frozen joint/context model; unsupported commands remain explicit deficits."""
    first, identities = _rows(rows)
    models, deficits = {}, {}
    for key in sorted({command_key(r['command']) for r in rows}):
        selected = [r for r in rows if command_key(r['command']) == key]
        if len(selected) < 2:
            deficits[key] = dict(command=copy.deepcopy(selected[0]['command']), count=len(selected))
            continue
        x = np.asarray([r['before'] for r in selected], float)
        delta = np.asarray([r['after'] for r in selected], float)-x
        mean, scale = x.mean(axis=0), np.maximum(x.std(axis=0), 1.)
        z, intercept = (x-mean)/scale, delta.mean(axis=0)
        coefficient = np.linalg.solve(z.T@z+np.eye(len(first['stateFields'])), z.T@(delta-intercept))
        models[key] = dict(command=copy.deepcopy(selected[0]['command']), count=len(selected),
            mean=mean.tolist(), scale=scale.tolist(), deltaMean=intercept.tolist(),
            coefficient=coefficient.tolist(), minimum=x.min(axis=0).tolist(), maximum=x.max(axis=0).tolist())
    result = dict(schemaVersion=SCHEMA, joint=first['joint'], context=copy.deepcopy(first['context']),
        stateFields=copy.deepcopy(first['stateFields']), alpha=1., models=models, deficits=deficits,
        trainingIdentities={k:sorted(v) for k,v in identities.items()},
        trainingCaptureGroups=sorted({r['captureGroup'] for r in rows}),
        controlEligible=False, semantics='local_2d_feature_changes_not_pose_contact_or_execution_authorization')
    validate_model(result)
    return result


def _entry(model, before, command, context):
    validate_model(model)
    require(context == model['context'] and command['joint'] == model['joint'], 'prediction_context_changed')
    x = vector(before, len(model['stateFields']))
    entry = model['models'].get(command_key(command))
    require(entry is not None, 'unsupported_command')
    require(all(lo <= value <= hi for value,lo,hi in zip(x, entry['minimum'], entry['maximum'])),
            'state_outside_training_range')
    return x, entry


def predict(model, before, command, context):
    x, entry = _entry(model, before, command, context)
    prediction = x+np.asarray(entry['deltaMean'])+(
        (x-np.asarray(entry['mean']))/np.asarray(entry['scale']))@np.asarray(entry['coefficient'])
    require(np.isfinite(prediction).all(), 'invalid_prediction')
    return prediction.tolist()


def prospective_evidence(model, row, prepared_at):
    """Call with only the before state available, before the actual command."""
    require(number(prepared_at) and number(row.get('beforePersistedAt')) and
            row['beforePersistedAt'] <= prepared_at, 'prospective_input_not_persisted')
    return dict(sessionId=row['sessionId'], modelSha256=model_sha256(model),
        beforeStateSha256=digest(row['before']), commandSha256=digest(row['command']),
        prediction=predict(model, row['before'], row['command'], row['context']),
        beforePersistedAt=row['beforePersistedAt'], preparedAt=prepared_at)


def _metrics(pairs):
    if not pairs:
        return dict(n=0, maePerState=None, meanStateError=None)
    errors = np.asarray([np.asarray(a)-np.asarray(b) for a,b in pairs])
    return dict(n=len(pairs), maePerState=np.abs(errors).mean(axis=0).tolist(),
                meanStateError=float(np.linalg.norm(errors, axis=1).mean()))


def evaluate(model, rows, prospective=None):
    """Same-case baselines; a prospective ledger is verified, never fabricated here.

    Prospective records require the runtime's separately audited commandAt and
    beforePersistedAt on each row. Hash/timing consistency is not camera accuracy.
    """
    validate_model(model)
    first, identities = _rows(rows)
    require(all(first[k] == model[k] for k in ('joint', 'context', 'stateFields')), 'evaluation_context_changed')
    for key, values in identities.items():
        require(not values & set(model['trainingIdentities'][key]), 'training_evaluation_overlap:'+key)
    require(not {r['captureGroup'] for r in rows} & set(model['trainingCaptureGroups']),
            'training_evaluation_group_overlap')
    require(prospective is None or isinstance(prospective, dict), 'prospective_ledger_required')
    pairs = {k:[] for k in ('stateRidge', 'commandMean', 'noChange')}
    predictions, rejected = [], []
    for row in rows:
        try:
            prediction = predict(model, row['before'], row['command'], row['context'])
            if prospective is not None:
                proof = prospective.get(row['sessionId'])
                require(isinstance(proof, dict) and proof.get('sessionId') == row['sessionId'] and
                        proof.get('modelSha256') == model_sha256(model) and
                        proof.get('beforeStateSha256') == digest(row['before']) and
                        proof.get('commandSha256') == digest(row['command']) and
                        proof.get('prediction') == prediction, 'prospective_prediction_binding_changed')
                require(number(row.get('commandAt')) and number(row.get('beforePersistedAt')) and
                        number(proof.get('preparedAt')) and
                        proof.get('beforePersistedAt') == row['beforePersistedAt'] and
                        row['beforePersistedAt'] <= proof['preparedAt'] < row['commandAt'],
                        'prediction_not_before_command')
            entry = model['models'][command_key(row['command'])]
            candidates = dict(stateRidge=prediction,
                commandMean=(np.asarray(row['before'])+np.asarray(entry['deltaMean'])).tolist(),
                noChange=row['before'])
            for key, value in candidates.items():
                pairs[key].append((value, row['after']))
            predictions.append(dict(sessionId=row['sessionId'], **copy.deepcopy(candidates)))
        except (ValueError, KeyError, TypeError) as exc:
            rejected.append(dict(sessionId=row['sessionId'], reason=str(exc)))
    return dict(kind='prospectiveFrozenEvaluation' if prospective is not None else 'developmentReplay',
        listed=len(rows), evaluated=len(predictions), rejected=rejected, predictions=predictions,
        metrics={key:_metrics(value) for key,value in pairs.items()},
        prospectiveEvidenceVerified=prospective is not None and bool(predictions) and not rejected,
        evaluationPassed=None, controlEligible=False, modelSha256=model_sha256(model))
