"""M1 전용 shaft 간격 1차원 전이 모델. 오프라인 대안이며 제어 경로가 없다.

물체와 턱 중심의 상대 XY는 관측 진단으로만 남긴다. 이 모델은 XY, 턱 끝,
접촉, 파지나 다른 관절 동작을 예측하지 않으며 평가 시점의 선후도 증명하지 않는다.
"""
from __future__ import annotations

import copy
import math
import re

import numpy as np

from learning import relative_action_learning as relative
from learning.manual_collection_dataset import require
from learning.pipeline_learning import canonical_bytes, digest

SCHEMA = 'motionbrain.m1_shaft_ridge.v1'
PREFIX = relative.REGISTERED_METHODS['registered_gripper_crosssection_points']
SHAFT_PREFIX = 'registered-gripper-shaft-only-v1:'
TRANSITION_SCHEMA = 'motionbrain.m1_shaft_transition.v1'


def _scope(context):
    landmarks = context.get('landmarks', '') if isinstance(context, dict) else ''
    prefix = SHAFT_PREFIX if str(landmarks).startswith(SHAFT_PREFIX) else PREFIX
    fields = (['shaftSeparationPx'] if prefix == SHAFT_PREFIX else
              relative.state_fields(context if isinstance(context, dict) else {}))
    require(isinstance(context, dict) and set(context) == {
                'physicalView', 'object', 'poseRegime', 'landmarks', 'cameraIdentity', 'imageShape', 'stateFields'}
            and all(isinstance(context[k], str) and context[k] for k in
                    ('physicalView', 'object', 'poseRegime', 'landmarks'))
            and isinstance(context['cameraIdentity'], dict) and bool(context['cameraIdentity'])
            and isinstance(context['imageShape'], list) and len(context['imageShape']) == 2
            and all(type(v) is int and v > 0 for v in context['imageShape'])
            and str(landmarks).startswith(prefix)
            and re.fullmatch('[0-9a-f]{64}', landmarks[len(prefix):]) is not None
            and context.get('stateFields') == fields,
            'registered_gripper_crosssection_context_required')


def _command_scope(context, command):
    require(isinstance(command, dict) and command.get('joint') == 'gripper',
            'shaft_observation_requires_gripper_only')
    relative.command_key(command)


def _scalar(value):
    require(type(value) in (int, float) and math.isfinite(value) and value >= 0,
            'finite_nonnegative_shaft_separation_required')
    return float(value)


def _rows(rows):
    require(isinstance(rows, list) and bool(rows), 'transitions_required')
    typed = rows[0].get('schemaVersion') == TRANSITION_SCHEMA
    require(all((r.get('schemaVersion') == TRANSITION_SCHEMA) == typed for r in rows),
            'transition_schema_changed')
    if typed:
        context = rows[0]['context']
        ids, episodes, frames = set(), set(), set()
        for row in rows:
            require(row['context'] == context, 'observation_context_changed')
            require(not {'before', 'after'}.intersection(row), 'scalar_transition_must_not_supply_3d_state')
            require(isinstance(row.get('sessionId'), str) and row['sessionId']
                    and row['sessionId'] not in ids, 'duplicate_or_missing_session')
            require(isinstance(row.get('captureGroup'), str) and row['captureGroup'], 'capture_group_required')
            episode, refs = row.get('episode'), row.get('frameIds')
            require(isinstance(episode, list) and len(episode) == 2
                    and all(isinstance(v, str) and v for v in episode)
                    and tuple(episode) not in episodes, 'duplicate_or_missing_episode')
            require(isinstance(refs, list) and len(refs) == 2
                    and all(isinstance(v, str) and v for v in refs)
                    and refs[0] != refs[1] and not frames.intersection(refs), 'reused_frame')
            ids.add(row['sessionId']); episodes.add(tuple(episode)); frames.update(refs)
            _scalar(row.get('beforeShaftPx')); _scalar(row.get('afterShaftPx'))
    else:
        context = relative.validate_rows(rows)
    _scope(context)
    require(str(context['landmarks']).startswith(SHAFT_PREFIX) == typed,
            'transition_schema_context_mismatch')
    for row in rows:
        _command_scope(context, row['command'])
    sources = [r.get('sourceHash') for r in rows]
    require((not typed and all(s is None for s in sources)) or all(isinstance(s, str)
            and re.fullmatch('[0-9a-f]{64}', s) is not None for s in sources),
            'consistent_source_hashes_required')
    present = [s for s in sources if s is not None]
    require(len(present) == len(set(present)), 'duplicate_source_hash')
    return context


def _shaft(row, phase):
    if row.get('schemaVersion') == TRANSITION_SCHEMA:
        return row[phase+'ShaftPx']
    return row[phase][2]


def fit(rows):
    """Fit delta-shaft with scalar standardized input and fixed ridge alpha=1."""
    context = _rows(rows)
    entries = []
    for key in sorted({relative.command_key(r['command']) for r in rows}):
        selected = [r for r in rows if relative.command_key(r['command']) == key]
        if len(selected) < 2:
            continue
        x = np.asarray([_shaft(r, 'before') for r in selected], dtype=float)
        delta = np.asarray([_shaft(r, 'after')-_shaft(r, 'before') for r in selected], dtype=float)
        mean, scale, intercept = float(x.mean()), max(float(x.std()), 1.), float(delta.mean())
        z = (x-mean)/scale
        coefficient = float(z@(delta-intercept)/(z@z+1.))
        entries.append(dict(command=copy.deepcopy(selected[0]['command']), count=len(selected),
            mean=mean, scale=scale, coefficient=coefficient, deltaMean=intercept,
            minimum=float(x.min()), maximum=float(x.max())))
    model = dict(schemaVersion=SCHEMA, context=copy.deepcopy(context), models=entries,
        inputStateFields=['shaftSeparationPx'], predictedStateFields=['shaftSeparationPx'],
        excludedStateFields=['objectMinusJawMidXpx', 'objectMinusJawMidYpx'],
        annotationSemantics='registered_shaft_crosssection_2d_not_jaw_tip_contact_grasp_or_other_joint_model',
        alpha=1., controlEligible=False, independentTest=False,
        trainingSessionIds=[r['sessionId'] for r in rows],
        trainingCaptureGroups=sorted({r['captureGroup'] for r in rows}),
        trainingEpisodes=[copy.deepcopy(r['episode']) for r in rows],
        trainingFrameIds=[f for r in rows for f in r['frameIds']],
        trainingSourceHashes=[r['sourceHash'] for r in rows if r.get('sourceHash') is not None])
    return validate_model(model)


def validate_model(model):
    """Validate an immutable artifact; no training or range expansion occurs."""
    require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA
            and type(model.get('alpha')) in (int, float) and model['alpha'] == 1.
            and model.get('controlEligible') is False and model.get('independentTest') is False,
            'offline_m1_shaft_model_required')
    _scope(model.get('context'))
    require(model.get('inputStateFields') == ['shaftSeparationPx']
            and model.get('predictedStateFields') == ['shaftSeparationPx']
            and model.get('excludedStateFields') == ['objectMinusJawMidXpx', 'objectMinusJawMidYpx'],
            'shaft_only_model_required')
    for field in ('trainingSessionIds', 'trainingCaptureGroups', 'trainingFrameIds'):
        values = model.get(field)
        require(isinstance(values, list) and bool(values)
                and all(isinstance(v, str) and v for v in values)
                and len(values) == len(set(values)), 'model_training_provenance_required')
    n = len(model['trainingSessionIds']); episodes = model.get('trainingEpisodes')
    require(isinstance(episodes, list) and len(episodes) == n
            and all(isinstance(e, list) and len(e) == 2
                    and all(isinstance(s, str) and s for s in e) for e in episodes)
            and len({tuple(e) for e in episodes}) == n
            and len(model['trainingFrameIds']) == 2*n, 'model_training_provenance_required')
    sources = model.get('trainingSourceHashes')
    source_counts = (n,) if model['context']['landmarks'].startswith(SHAFT_PREFIX) else (0, n)
    require(isinstance(sources, list) and len(sources) in source_counts
            and all(isinstance(s, str) and re.fullmatch('[0-9a-f]{64}', s) for s in sources)
            and len(sources) == len(set(sources)), 'model_source_hashes_required')
    require(isinstance(model.get('models'), list), 'model_commands_required')
    keys, total = set(), 0
    for entry in model['models']:
        require(isinstance(entry, dict), 'model_command_parameters_required')
        key = relative.command_key(entry.get('command'))
        _command_scope(model['context'], entry['command'])
        require(key not in keys and type(entry.get('count')) is int and entry['count'] >= 2,
                'unique_command_two_transitions_required')
        keys.add(key); total += entry['count']
        require(all(type(entry.get(k)) in (int, float) and math.isfinite(entry[k])
                    for k in ('minimum', 'maximum', 'mean', 'scale', 'deltaMean', 'coefficient'))
                and 0 <= entry['minimum'] <= entry['mean'] <= entry['maximum']
                and entry['scale'] >= 1., 'invalid_scalar_model_parameters')
    require(total <= n, 'model_support_exceeds_training_sources')
    return copy.deepcopy(model)


def predict(model, shaft_separation_px, command, context, *, baseline=False):
    """Return one predicted shaft separation; XY is neither accepted nor gated."""
    validate_model(model)
    require(context == model['context'], 'observation_context_changed')
    _command_scope(context, command)
    key = relative.command_key(command); x = _scalar(shaft_separation_px)
    selected = [e for e in model['models'] if relative.command_key(e['command']) == key]
    require(len(selected) == 1, 'command_without_two_training_transitions')
    entry = selected[0]
    require(entry['minimum'] <= x <= entry['maximum'], 'shaft_outside_training_range')
    delta = entry['deltaMean']
    if not baseline:
        delta += (x-entry['mean'])/entry['scale']*entry['coefficient']
    return _scalar(x+delta)


def evaluate(model, rows):
    """Same held-out cases for ridge and both baselines; rejected cases retained."""
    validate_model(model)
    if rows:
        context = _rows(rows)
        require(context == model['context'], 'observation_context_changed')
    require(not set(model['trainingCaptureGroups']).intersection(r['captureGroup'] for r in rows),
            'training_evaluation_group_overlap')
    require(not set(model['trainingSessionIds']).intersection(r['sessionId'] for r in rows)
            and not {tuple(e) for e in model['trainingEpisodes']}.intersection(tuple(r['episode']) for r in rows)
            and not set(model['trainingFrameIds']).intersection(f for r in rows for f in r['frameIds'])
            and not set(model['trainingSourceHashes']).intersection(r.get('sourceHash') for r in rows),
            'training_evaluation_overlap')
    predictions, rejected = [], []
    for row in rows:
        try:
            x = _shaft(row, 'before')
            predictions.append(dict(sessionId=row['sessionId'], actualShaftSeparationPx=_shaft(row, 'after'),
                shaftRidge=predict(model, x, row['command'], row['context']),
                commandMean=predict(model, x, row['command'], row['context'], baseline=True), noChange=x,
                bodyRelativeXYDiagnostic=(None if row.get('schemaVersion') == TRANSITION_SCHEMA else
                    dict(before=row['before'][:2], after=row['after'][:2]))))
        except ValueError as exc:
            rejected.append(dict(sessionId=row['sessionId'], reason=str(exc)))
    metrics = {}
    for name in ('shaftRidge', 'commandMean', 'noChange'):
        if predictions:
            errors = np.asarray([p[name]-p['actualShaftSeparationPx'] for p in predictions])
            metrics[name] = dict(n=len(predictions), maePx=float(np.abs(errors).mean()),
                                 rmsePx=float(np.sqrt(np.mean(errors**2))))
    return dict(listed=len(rows), evaluated=len(predictions), rejected=rejected,
        predictions=predictions, metrics=metrics, modelSha256=digest(model),
        evaluationKind='developmentReplay', temporalOrderVerified=False,
        evaluationPassed=None, controlEligible=False, independentTest=False,
        predictedStateFields=['shaftSeparationPx'], bodyRelativeXY='observation_diagnostic_only')
