"""Experimental M4 sensor-prior transfer to one registered camera view.

The declared operating envelope is not an empirical training-support range.
Old models are never modified. This core has no transport or execution authority.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

SCHEMA = 'motionbrain.m4_approach_transfer.v1'
COMMANDS = [dict(joint='shoulder', direction='down', percent=p, durationMs=350) for p in (20, 25)]
LIMIT_KEYS = {'sensorBandDeg', 'maxObservedDeltaDeg', 'maxPredictionStepDeg',
              'maxStateStepPx', 'maxEvaluationErrorPx', 'minimumImprovementPx'}


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def vector(value):
    require(isinstance(value, list) and len(value) == 3
            and all(number(x) for x in value) and value[2] >= 0, 'finite_camera_state_required')
    return np.asarray(value, dtype=float)


def hash_value(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def load_prior(path, expected_sha):
    raw = Path(path).read_bytes()
    require(hash_value(expected_sha) and hashlib.sha256(raw).hexdigest() == expected_sha,
            'prior_file_hash_changed')
    return json.loads(raw)


def _limits(limits):
    require(isinstance(limits, dict) and set(limits) == LIMIT_KEYS, 'declared_limits_required')
    band = limits['sensorBandDeg']
    require(isinstance(band, list) and len(band) == 2 and all(number(x) for x in band)
            and 0 <= band[0] < band[1] <= 360, 'sensor_band_invalid')
    require(all(number(limits[k]) and limits[k] > 0 for k in LIMIT_KEYS - {'sensorBandDeg'})
            and limits['maxObservedDeltaDeg'] <= 2.5
            and limits['maxPredictionStepDeg'] <= 2.
            and limits['maxStateStepPx'] <= 40.
            and limits['maxEvaluationErrorPx'] <= 6.
            and limits['minimumImprovementPx'] >= 3., 'engineering_limits_exceeded')


def _rows(rows, context):
    require(isinstance(rows, list) and bool(rows) and isinstance(context, dict) and bool(context),
            'transitions_and_actual_context_required')
    identities = {key: set() for key in ('episodes', 'frames', 'sources', 'groups')}
    for row in rows:
        require(row.get('context') == context and row.get('command') in COMMANDS,
                'context_or_command_changed')
        vector(row['before']); vector(row['after'])
        require(all(number(row.get(k)) for k in ('sensorBefore', 'sensorAfter')),
                'finite_sensor_observations_required')
        episode, frames = row.get('episode'), row.get('frameIds')
        require(isinstance(episode, list) and len(episode) == 2
                and all(isinstance(x, str) and x for x in episode)
                and tuple(episode) not in identities['episodes'], 'duplicate_or_missing_episode')
        require(isinstance(frames, list) and len(frames) == 2 and frames[0] != frames[1]
                and all(isinstance(x, str) and x for x in frames)
                and not identities['frames'].intersection(frames), 'duplicate_or_missing_frames')
        source, group = row.get('sourceHash'), row.get('captureGroup')
        require(hash_value(source) and source not in identities['sources']
                and isinstance(group, str) and bool(group), 'source_or_group_invalid')
        identities['episodes'].add(tuple(episode)); identities['frames'].update(frames)
        identities['sources'].add(source); identities['groups'].add(group)
    return identities


def fit(prior, prior_sha, transitions, context, limits):
    _limits(limits)
    ids = _rows(transitions, context)
    require(hash_value(prior_sha) and prior.get('schemaVersion') == 'motionbrain.manual_m4_precommitted_18.v1'
            and prior.get('controlEligible') is False, 'frozen_sensor_prior_required')
    means = []
    for command in COMMANDS:
        found = [x for x in prior['models']['conditionMeans'] if x.get('command') == command]
        require(len(found) == 1 and found[0].get('count') == 3
                and number(found[0].get('meanDeg')) and found[0]['meanDeg'] < 0,
                'prior_command_support_missing')
        means.append(copy.deepcopy(found[0]))
    require(len(transitions) >= 2 and all(any(r['command'] == c for r in transitions) for c in COMMANDS),
            'two_scoped_calibration_conditions_required')
    angles = np.asarray([r['sensorAfter'] - r['sensorBefore'] for r in transitions])
    changes = np.asarray([vector(r['after']) - vector(r['before']) for r in transitions])
    band = limits['sensorBandDeg']
    require(all(band[0] <= r[k] <= band[1] for r in transitions for k in ('sensorBefore', 'sensorAfter'))
            and np.all(angles <= -.35) and np.all(np.abs(angles) <= limits['maxObservedDeltaDeg'])
            and np.all(np.linalg.norm(changes, axis=1) >= 2.)
            and np.all(np.linalg.norm(changes, axis=1) <= limits['maxStateStepPx']),
            'calibration_effect_not_identifiable')
    jacobian = np.sum(angles[:, None] * changes, axis=0) / float(angles @ angles)
    require(np.linalg.norm(jacobian) > 0, 'calibration_camera_mapping_inconsistent')
    directions = changes / angles[:, None]
    cosines = directions @ jacobian / (np.linalg.norm(directions, axis=1) * np.linalg.norm(jacobian))
    residual = changes - angles[:, None] * jacobian
    require(np.all(cosines >= .8) and np.max(np.abs(residual)) <= limits['maxEvaluationErrorPx'],
            'calibration_camera_mapping_inconsistent')
    prior_deltas = np.asarray([next(x['meanDeg'] for x in means if x['command'] == r['command'])
                               for r in transitions])
    sensor_scale = float(prior_deltas @ angles / (prior_deltas @ prior_deltas))
    require(sensor_scale > 0 and np.max(np.abs(angles - sensor_scale * prior_deltas)) <= .6,
            'calibration_sensor_prior_inconsistent')
    observations = np.asarray([r[k] for r in transitions for k in ('before', 'after')])
    model = dict(schemaVersion=SCHEMA, context=copy.deepcopy(context), limits=copy.deepcopy(limits),
                 priorSha256=prior_sha, priorCanonicalSha256=digest(prior), priorMeans=means,
                 jacobian=jacobian.tolist(), sensorScale=sensor_scale,
                 calibration=copy.deepcopy(transitions),
                 provenance={key: sorted(list(x) for x in values) if key == 'episodes' else sorted(values)
                             for key, values in ids.items()},
                 observedSensorRangeDeg=[min(r[k] for r in transitions for k in ('sensorBefore', 'sensorAfter')),
                                         max(r[k] for r in transitions for k in ('sensorBefore', 'sensorAfter'))],
                 observedStateMinimum=observations.min(axis=0).tolist(),
                 observedStateMaximum=observations.max(axis=0).tolist(),
                 rangeExtensionExperiment=True, operatingLimitsAreTrainingSupport=False,
                 calibratedUncertainty=False, controlEligible=False, depthOrContactVerified=False)
    model['modelSha256'] = digest(model)
    return model


def _model(model):
    require(model.get('schemaVersion') == SCHEMA and model.get('controlEligible') is False
            and model.get('modelSha256') == digest({k: v for k, v in model.items() if k != 'modelSha256'}),
            'transfer_model_changed')
    _limits(model['limits'])


def predict(model, state, sensor_deg, command, context):
    _model(model); x = vector(state); limits = model['limits']
    require(context == model['context'], 'observation_context_changed')
    require(command in COMMANDS, 'unsupported_command')
    require(number(sensor_deg) and limits['sensorBandDeg'][0] <= sensor_deg <= limits['sensorBandDeg'][1],
            'sensor_outside_operating_band')
    prior = next(m['meanDeg'] for m in model['priorMeans'] if m['command'] == command)
    delta_angle = prior * model['sensorScale']; delta = np.asarray(model['jacobian']) * delta_angle
    end = sensor_deg + delta_angle
    require(abs(delta_angle) <= limits['maxPredictionStepDeg']
            and np.linalg.norm(delta) <= limits['maxStateStepPx']
            and limits['sensorBandDeg'][0] <= end <= limits['sensorBandDeg'][1], 'predicted_step_outside_limits')
    prediction = (x + delta).tolist(); vector(prediction)
    same = [r for r in model['calibration'] if r['command'] == command]
    baseline = ((x + np.mean([vector(r['after']) - vector(r['before']) for r in same], axis=0)).tolist()
                if len(same) >= 2 else None)
    return dict(predictedState=prediction, predictedSensorDeltaDeg=delta_angle, predictedSensorAfterDeg=end,
                baselines=dict(noChange=state.copy(), historicalSensorMeanDeltaDeg=prior,
                               unscaledPriorCameraState=(x + np.asarray(model['jacobian']) * prior).tolist(),
                               commandMean=dict(count=len(same), supported=len(same) >= 2,
                                                                      predictedState=baseline)),
                rangeExtensionExperiment=True, controlEligible=False)


def evaluate(model, rows):
    _model(model); identities = _rows(rows, model['context'])
    for key, values in identities.items():
        old = {tuple(x) if key == 'episodes' else x for x in model['provenance'][key]}
        require(not values.intersection(old), 'evaluation_training_overlap_' + key)
    cases = []
    for row in rows:
        try:
            prediction = predict(model, row['before'], row['sensorBefore'], row['command'], row['context'])
            errors = np.abs(np.asarray(prediction['predictedState']) - vector(row['after'])).tolist()
            sensor_error = abs(prediction['predictedSensorDeltaDeg'] - (row['sensorAfter'] - row['sensorBefore']))
            band = model['limits']['sensorBandDeg']; actual_delta = row['sensorAfter'] - row['sensorBefore']
            passed = (max(errors) <= model['limits']['maxEvaluationErrorPx'] and sensor_error <= .6
                      and -model['limits']['maxObservedDeltaDeg'] <= actual_delta <= -.35
                      and band[0] <= row['sensorAfter'] <= band[1]
                      and np.linalg.norm(vector(row['after']) - vector(row['before'])) >= 2.)
            cases.append(dict(prediction=prediction, absoluteErrorPx=errors, sensorErrorDeg=sensor_error,
                              baselineAbsoluteErrorsPx={name: np.abs(np.asarray(value)-vector(row['after'])).tolist()
                                  for name, value in prediction['baselines'].items()
                                  if name in ('noChange', 'unscaledPriorCameraState')},
                              passed=bool(passed)))
        except ValueError as exc:
            cases.append(dict(passed=False, reason=str(exc)))
    report = dict(modelSha256=model['modelSha256'], rows=copy.deepcopy(rows), cases=cases,
                  listed=len(rows), evaluated=len(cases), retrained=False,
                  evaluationPassed=all(x['passed'] for x in cases), independentGeneralizationProven=False)
    report['evaluationSha256'] = digest(report)
    return report


def select(model, state, sensor_deg, commands, target, context, evaluation, expected_evaluation_sha):
    _model(model); x, goal = vector(state), vector(target)
    require(hash_value(expected_evaluation_sha) and evaluation.get('evaluationSha256') == expected_evaluation_sha
            and evaluate(model, evaluation.get('rows')) == evaluation
            and evaluation.get('evaluationPassed') is True, 'passed_frozen_evaluation_required')
    require(context == model['context'] and isinstance(commands, list) and bool(commands)
            and len({digest(c) for c in commands}) == len(commands), 'context_or_candidates_invalid')
    hold = dict(kind='hold', command=None, predictedState=state.copy(), errorPx=float(np.linalg.norm(x - goal)))
    candidates, rejected = [hold], []
    evaluated_commands = [row['command'] for row, case in
                          zip(evaluation['rows'], evaluation['cases']) if case['passed']]
    for command in commands:
        try:
            prediction = predict(model, state, sensor_deg, command, context)
            require(command in evaluated_commands, 'candidate_without_frozen_evaluation')
            error = float(np.linalg.norm(np.asarray(prediction['predictedState']) - goal))
            candidates.append(dict(kind='action', command=copy.deepcopy(command), **prediction,
                                   errorPx=error, improvementPx=hold['errorPx'] - error))
        except ValueError as exc:
            rejected.append(dict(command=copy.deepcopy(command), reason=str(exc)))
    eligible = [hold] + [x for x in candidates[1:] if x['improvementPx'] >= model['limits']['minimumImprovementPx']]
    return dict(selected=copy.deepcopy(min(eligible, key=lambda c: c['errorPx'])), candidates=candidates,
                rejected=rejected, controlEligible=False, rangeExtensionExperiment=True,
                graspVerified=False, depthOrContactVerified=False)
