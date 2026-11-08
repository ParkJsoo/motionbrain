"""Paired observed M4 outcomes in one context; no calibrated future coverage.

The frozen transfer model remains a point-prediction baseline. Empirical outcomes
preserve sensor/camera pairing. This pure core has no execution authority.
"""
from __future__ import annotations

import copy
import numpy as np

from learning import m4_approach_transfer as transfer

SCHEMA = 'motionbrain.m4_empirical_approach.v1'
COMMANDS = transfer.COMMANDS
require, digest, vector = transfer.require, transfer.digest, transfer.vector


def fit(rows, parentTransferModel):
    transfer._model(parentTransferModel)
    context, limits = parentTransferModel['context'], parentTransferModel['limits']
    identities = transfer._rows(rows, context)
    outcomes = []
    for index, row in enumerate(rows):
        delta = row['sensorAfter'] - row['sensorBefore']
        change = vector(row['after']) - vector(row['before'])
        require(-limits['maxObservedDeltaDeg'] <= delta <= -.35
                and 2. <= np.linalg.norm(change) <= limits['maxStateStepPx']
                and all(limits['sensorBandDeg'][0] <= row[k] <= limits['sensorBandDeg'][1]
                        for k in ('sensorBefore', 'sensorAfter')), 'observed_effect_outside_limits')
        outcomes.append(dict(rowIndex=index, command=copy.deepcopy(row['command']),
                             sensorDeltaDeg=delta, stateDeltaPx=change.tolist()))
    # The baseline has also seen its own calibration, even if absent from rows.
    for key, old in parentTransferModel['provenance'].items():
        identities[key].update(tuple(x) if key == 'episodes' else x for x in old)
    model = dict(schemaVersion=SCHEMA, context=copy.deepcopy(context), limits=copy.deepcopy(limits),
                 parentTransferModel=copy.deepcopy(parentTransferModel), training=copy.deepcopy(rows),
                 outcomes=outcomes,
                 commandSupport=[dict(command=copy.deepcopy(c), count=sum(r['command'] == c for r in rows),
                                      supported=sum(r['command'] == c for r in rows) >= 2) for c in COMMANDS],
                 provenance={k: sorted(list(x) for x in v) if k == 'episodes' else sorted(v)
                             for k, v in identities.items()},
                 calibratedUncertainty=False, futureCoverageGuaranteed=False,
                 operatingLimitsAreTrainingSupport=False, rangeExtensionExperiment=True,
                 controlEligible=False, depthOrContactVerified=False)
    model['modelSha256'] = digest(model)
    return model


def _model(model):
    require(model.get('schemaVersion') == SCHEMA and model.get('controlEligible') is False
            and model.get('modelSha256') == digest({k: v for k, v in model.items() if k != 'modelSha256'}),
            'empirical_model_changed')
    transfer._model(model['parentTransferModel']); transfer._limits(model['limits'])


def predict(model, state, sensor_deg, command, context):
    _model(model); x = vector(state); limits = model['limits']; band = limits['sensorBandDeg']
    require(context == model['context'], 'observation_context_changed')
    require(command in COMMANDS, 'unsupported_command')
    require(transfer.number(sensor_deg) and band[0] <= sensor_deg <= band[1], 'sensor_outside_operating_band')
    samples = [r for r in model['outcomes'] if r['command'] == command]
    require(len(samples) >= 2, 'insufficient_empirical_command_support')
    predictions = []
    for sample in samples:
        angle, change = sample['sensorDeltaDeg'], np.asarray(sample['stateDeltaPx'])
        end, predicted = sensor_deg + angle, (x + change).tolist()
        vector(predicted)
        require(abs(angle) <= limits['maxPredictionStepDeg']
                and np.linalg.norm(change) <= limits['maxStateStepPx'] and band[0] <= end <= band[1],
                'empirical_predicted_step_outside_limits')
        predictions.append(dict(sampleIndex=sample['rowIndex'], predictedState=predicted,
            predictedSensorDeltaDeg=angle, predictedSensorAfterDeg=end))
    states = np.asarray([p['predictedState'] for p in predictions])
    angles = [p['predictedSensorDeltaDeg'] for p in predictions]
    try:
        parent = dict(supported=True, **transfer.predict(model['parentTransferModel'], state,
                                                        sensor_deg, command, context))
    except ValueError as exc:
        parent = dict(supported=False, reason=str(exc))
    return dict(predictedState=states.mean(axis=0).tolist(), predictedSensorDeltaDeg=float(np.mean(angles)),
        predictedSensorAfterDeg=sensor_deg + float(np.mean(angles)), observedOutcomes=predictions,
        empiricalEnvelope=dict(stateMinimum=states.min(axis=0).tolist(), stateMaximum=states.max(axis=0).tolist(),
            sensorDeltaMinimumDeg=min(angles), sensorDeltaMaximumDeg=max(angles),
            marginalBoundsAreNotIndependentOutcomes=True),
        baselines=dict(noChange=state.copy(), parentTransferPrediction=parent),
        calibratedUncertainty=False, futureCoverageGuaranteed=False, controlEligible=False)


def _independent(model, rows):
    identities = transfer._rows(rows, model['context'])
    for key, values in identities.items():
        old = {tuple(x) if key == 'episodes' else x for x in model['provenance'][key]}
        require(not values.intersection(old), 'evaluation_training_overlap_' + key)


def _case(model, row):
    try:
        prediction = predict(model, row['before'], row['sensorBefore'], row['command'], row['context'])
        actual, delta = vector(row['after']), row['sensorAfter'] - row['sensorBefore']
        limits = model['limits']; errors = []
        for sample in prediction['observedOutcomes']:
            camera = np.abs(np.asarray(sample['predictedState']) - actual).tolist()
            sensor = abs(sample['predictedSensorDeltaDeg'] - delta)
            errors.append(dict(sampleIndex=sample['sampleIndex'], absoluteErrorPx=camera, sensorErrorDeg=sensor,
                               matched=max(camera) <= limits['maxEvaluationErrorPx'] and sensor <= .6))
        matched = [e['sampleIndex'] for e in errors if e['matched']]
        envelope = prediction['empiricalEnvelope']
        inside = dict(camera=bool(np.all(actual >= envelope['stateMinimum'])
                                  and np.all(actual <= envelope['stateMaximum'])),
                      sensor=envelope['sensorDeltaMinimumDeg'] <= delta <= envelope['sensorDeltaMaximumDeg'])
        effect = (-limits['maxObservedDeltaDeg'] <= delta <= -.35
                  and limits['sensorBandDeg'][0] <= row['sensorAfter'] <= limits['sensorBandDeg'][1]
                  and np.linalg.norm(actual - vector(row['before'])) >= 2.)
        parent = prediction['baselines']['parentTransferPrediction']
        baselines = dict(noChange=np.abs(vector(row['before']) - actual).tolist())
        if parent['supported']:
            baselines['parentTransferPrediction'] = np.abs(np.asarray(parent['predictedState']) - actual).tolist()
        return dict(prediction=prediction, meanAbsoluteErrorPx=np.abs(np.asarray(prediction['predictedState']) - actual).tolist(),
            meanSensorErrorDeg=abs(prediction['predictedSensorDeltaDeg'] - delta),
            baselineAbsoluteErrorsPx=baselines, pairedSampleErrors=errors, matchedPairedSamples=matched,
            insideObservedRange=inside, actualEffectValid=bool(effect), passed=bool(matched and effect),
            calibratedUncertainty=False, futureCoverageGuaranteed=False)
    except ValueError as exc:
        return dict(passed=False, reason=str(exc))


def evaluate(model, rows):
    _model(model); _independent(model, rows)
    cases = [_case(model, row) for row in rows]
    report = dict(modelSha256=model['modelSha256'], rows=copy.deepcopy(rows), cases=cases,
        listed=len(rows), evaluated=len(cases), retrained=False, evaluationPassed=all(c['passed'] for c in cases),
        calibratedUncertainty=False, futureCoverageGuaranteed=False, independentGeneralizationProven=False)
    report['evaluationSha256'] = digest(report)
    return report


def application_check(model, row):
    _model(model); _independent(model, [row])
    report = dict(modelSha256=model['modelSha256'], row=copy.deepcopy(row), **_case(model, row),
                  addedToFrozenEvaluation=False, retrained=False)
    report['applicationCheckSha256'] = digest(report)
    return report


def select(model, state, sensor_deg, commands, target, context, evaluation, expected_evaluation_sha):
    _model(model); x, goal = vector(state), vector(target)
    require(transfer.hash_value(expected_evaluation_sha)
            and evaluation.get('evaluationSha256') == expected_evaluation_sha
            and evaluate(model, evaluation.get('rows')) == evaluation
            and evaluation.get('evaluationPassed') is True, 'passed_frozen_evaluation_required')
    require(context == model['context'] and isinstance(commands, list) and bool(commands)
            and len({digest(c) for c in commands}) == len(commands), 'context_or_candidates_invalid')
    hold = dict(kind='hold', command=None, predictedState=state.copy(), errorPx=float(np.linalg.norm(x - goal)))
    candidates, rejected = [hold], []
    evaluated = [r['command'] for r, c in zip(evaluation['rows'], evaluation['cases']) if c['passed']]
    for command in commands:
        try:
            prediction = predict(model, state, sensor_deg, command, context)
            require(command in evaluated, 'candidate_without_frozen_evaluation')
            errors = [float(np.linalg.norm(np.asarray(p['predictedState']) - goal))
                      for p in prediction['observedOutcomes']]
            worst = max(errors)
            candidates.append(dict(kind='action', command=copy.deepcopy(command), **prediction,
                sampledTargetErrorsPx=errors, errorPx=worst, improvementPx=hold['errorPx'] - worst,
                meanErrorPx=float(np.linalg.norm(np.asarray(prediction['predictedState']) - goal))))
        except ValueError as exc:
            rejected.append(dict(command=copy.deepcopy(command), reason=str(exc)))
    eligible = [hold] + [c for c in candidates[1:] if c['improvementPx'] >= model['limits']['minimumImprovementPx']]
    return dict(selected=copy.deepcopy(min(eligible, key=lambda c: c['errorPx'])), candidates=candidates,
        rejected=rejected, selectionCriterion='worst_observed_outcome_goal_error',
        calibratedUncertainty=False, futureCoverageGuaranteed=False, rangeExtensionExperiment=True,
        controlEligible=False, graspVerified=False, depthOrContactVerified=False)
