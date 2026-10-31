"""Offline, one-step ranking of M2 commands using an already fitted predictor.

No fitting, device access or execution payloads. Bounds describe recorded inputs,
not physical joint limits or confidence intervals. Original audit is a prerequisite.
"""
import copy
import hashlib
import json
import math

import numpy as np

from learning.joint_collection import ANGLE_ENVELOPE
from learning.joint_prediction import (
    ALPHAS, HORIZON_SECONDS, INPUT_NAMES, features, predict, validate_rows,
)

COMMANDS = (('down', 700), ('up', 700), ('down', 900), ('up', 900))


def canonical_sha256(value):
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(encoded.encode()).hexdigest()


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _vector(value):
    return isinstance(value, list) and len(value) == 6 and all(map(_finite, value))


class OfflineActionSelector:
    """Use train-only marginal bounds, then minimize predicted absolute goal error.

    `valid` and `settled` are caller assertions from an observation audit, not new
    camera checks. Use the same projected coordinate system as the fitted data.
    """

    def __init__(self, artifact, dataset):
        if (not isinstance(artifact, dict) or not isinstance(dataset, dict)
                or dataset.get('schemaVersion') != 'motionbrain.audited_projected_joint_dataset.v1'
                or artifact.get('schemaVersion') != 'motionbrain.projected_joint_prediction.v1'
                or artifact.get('inputNames') != list(INPUT_NAMES)
                or artifact.get('horizonSeconds') != HORIZON_SECONDS
                or artifact.get('controlEligible') is not False
                or dataset.get('controlEligible') is not False):
            raise ValueError('offline_prediction_contract_required')
        self.dataset_sha256 = canonical_sha256(dataset)
        if artifact.get('datasetSha256') != self.dataset_sha256:
            raise ValueError('model_dataset_binding_mismatch')
        rows = dataset.get('rows')
        if (not isinstance(rows, list)
                or any(not isinstance(r, dict) or not _vector(r.get('x'))
                       or not _finite(r.get('y')) for r in rows)):
            raise ValueError('finite_numeric_rows_required')
        validate_rows(rows)
        model = artifact.get('selectedModel')
        if (not isinstance(model, dict)
                or any(not _vector(model.get(k)) for k in ('mean', 'scale', 'coefficients'))
                or any(v <= 0 for v in model['scale'])
                or not _finite(model.get('intercept'))
                or not _finite(model.get('alpha')) or model['alpha'] not in ALPHAS
                or type(model.get('fitRows')) is not int or model['fitRows'] != 16):
            raise ValueError('invalid_fitted_model')
        train = [r for r in rows if r['split'] == 'train']
        x = np.asarray([r['x'] for r in train])
        scale = x.std(axis=0)
        scale[scale < 1e-9] = 1
        if (not np.allclose(model['mean'], x.mean(axis=0), rtol=1e-12, atol=1e-12)
                or not np.allclose(model['scale'], scale, rtol=1e-12, atol=1e-12)):
            raise ValueError('train_only_normalization_mismatch')
        self._model = copy.deepcopy(model)
        self.model_sha256 = canonical_sha256(artifact)
        self.velocity_bounds = (float(x[:, 1].min()), float(x[:, 1].max()))
        self.angle_bounds = {}
        for direction, duration in COMMANDS:
            angles = [r['x'][0] for r in train
                      if r['conditionId'].endswith(f'-{direction}-{duration}')]
            self.angle_bounds[(direction, duration)] = (min(angles), max(angles))

    def select(self, observation, goal_deg, *, tolerance_deg=1.0):
        if not isinstance(observation, dict):
            raise ValueError('observation_object_required')
        angle = observation.get('angleDeg')
        velocity = observation.get('velocityDegS')
        if (not all(map(_finite, (angle, velocity, goal_deg, tolerance_deg)))
                or tolerance_deg < 0):
            raise ValueError('finite_state_goal_and_tolerance_required')
        if not ANGLE_ENVELOPE[0] <= goal_deg <= ANGLE_ENVELOPE[1]:
            raise ValueError('goal_outside_projected_envelope')
        result = dict(
            schemaVersion='motionbrain.offline_action_selection.v1',
            modelSha256=self.model_sha256, datasetSha256=self.dataset_sha256,
            angleDeg=angle, velocityDegS=velocity, goalDeg=goal_deg,
            toleranceDeg=tolerance_deg, horizonSeconds=HORIZON_SECONDS,
            noChangeGoalErrorDeg=abs(goal_deg-angle), candidates=[], selected=None,
            status='abstain', reason=None, controlEligible=False,
            physicalJointAngleCalibrated=False,
        )
        if observation.get('valid') is not True or observation.get('settled') is not True:
            result['reason'] = 'valid_settled_observation_required'
            return result
        if not self.velocity_bounds[0] <= velocity <= self.velocity_bounds[1]:
            result['reason'] = 'velocity_outside_training_bounds'
            return result
        eligible = []
        for direction, duration in COMMANDS:
            lo, hi = self.angle_bounds[(direction, duration)]
            candidate = dict(kind='pulse', direction=direction, durationMs=duration,
                             pwmMagnitude=160, angleBoundsDeg=[lo, hi],
                             eligible=False, rejection=None)
            result['candidates'].append(candidate)
            if not lo <= angle <= hi:
                candidate['rejection'] = 'angle_outside_command_training_bounds'
                continue
            with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
                delta = float(predict(self._model, features(angle, velocity, direction, duration)))
            end = angle + delta
            if not math.isfinite(end) or not ANGLE_ENVELOPE[0] <= end <= ANGLE_ENVELOPE[1]:
                candidate['rejection'] = 'prediction_outside_projected_envelope'
                continue
            candidate.update(eligible=True, predictedDeltaDeg=delta, predictedEndDeg=end,
                             predictedGoalErrorDeg=abs(goal_deg-end))
            eligible.append(candidate)
        if not eligible:
            result['reason'] = 'no_supported_candidate'
            return result
        best = min(eligible, key=lambda c: (c['predictedGoalErrorDeg'], c['durationMs'], c['direction']))
        current_error = result['noChangeGoalErrorDeg']
        if current_error <= tolerance_deg or best['predictedGoalErrorDeg'] >= current_error:
            result.update(status='hold', reason=('within_goal_tolerance' if current_error <= tolerance_deg
                                                 else 'no_predicted_improvement'),
                          selected=dict(kind='hold', predictedEndDeg=angle,
                                        predictedGoalErrorDeg=current_error,
                                        assumption='no_change_not_measured_hold_outcome'))
        else:
            result.update(status='selected', reason='minimum_predicted_goal_error',
                          selected=copy.deepcopy(best))
        return result


def replay_saved_states(artifact, dataset, goals, *, split='test', tolerance_deg=1.0):
    """Exploratory replay, not a new held-out policy-success estimate.

    Logged labels are attached only after selection. Counterfactual outcomes and
    real hold outcomes are unknown; matching-action rows are a biased subset.
    """
    if (split not in ('train', 'validation', 'test')
            or not isinstance(goals, (list, tuple)) or not goals):
        raise ValueError('split_and_goals_required')
    selector = OfflineActionSelector(artifact, dataset)
    results = []
    for row in dataset['rows']:
        if row['split'] != split:
            continue
        _, logged_direction, logged_duration = row['conditionId'].split('-')
        for goal in goals:
            decision = selector.select(dict(angleDeg=row['x'][0], velocityDegS=row['x'][1],
                                            valid=True, settled=True), goal,
                                       tolerance_deg=tolerance_deg)
            selected = decision['selected']
            matches = bool(selected and selected['kind'] == 'pulse'
                           and selected['direction'] == logged_direction
                           and selected['durationMs'] == int(logged_duration))
            logged_end = row['x'][0] + row['y']
            decision.update(sessionId=row['sessionId'], conditionId=row['conditionId'],
                            loggedEndDeg=logged_end, loggedGoalErrorDeg=abs(goal-logged_end),
                            selectedMatchesLoggedAction=matches,
                            selectedObservedGoalErrorDeg=abs(goal-logged_end) if matches else None)
            results.append(decision)
    return dict(
        schemaVersion='motionbrain.offline_action_replay.v1', split=split, goalsDeg=list(goals),
        modelSha256=selector.model_sha256, datasetSha256=selector.dataset_sha256,
        support=dict(basis='training_inputs_only_marginal_bounds_not_joint_support',
                     velocityBoundsDegS=list(selector.velocity_bounds)),
        counts={s: sum(r['status'] == s for r in results) for s in ('selected', 'hold', 'abstain')},
        loggedActionMatches=sum(r['selectedMatchesLoggedAction'] for r in results),
        decisions=results, policySuccessRate=None, counterfactualOutcomesKnown=False,
        trainingPerformed=False, controlEligible=False,
    )
