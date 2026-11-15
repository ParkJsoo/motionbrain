"""무하중 관측에서 파지 후 한 번의 들기 후보를 비교하는 명시적 전이 실험.

물체를 든 자료로 학습한 모델이 아니며 하중 전이는 검증되지 않았다. 가상 영상점
상승의 예측은 실제 물체의 탁상 이탈·보유 성공을 뜻하지 않는다.
"""
from __future__ import annotations
import copy
import math
import re

import numpy as np

from learning.near_gripper_tip_observer import digest

SCHEMA = 'motionbrain.experimental_unloaded_to_held_lift_response.v1'
COMMAND = dict(joint='elbow', direction='up', percent=70, durationMs=1000)
SCALES = [180., 180., 15., 20.]


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def _sha(value): return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def _state(value):
    _require(isinstance(value, list) and len(value) == 4
             and all(type(v) in (int, float) and math.isfinite(v) for v in value)
             and 0 <= value[0] < 1920 and 0 <= value[1] < 1080
             and 0 <= value[2] < 360 and -180 <= value[3] <= 180, 'lift_finite_absolute_virtual_state_required')
    return np.array(value, dtype=float)


def _difference(a, b):
    result = np.array(a, dtype=float)-np.array(b, dtype=float)
    result[..., 3] = (result[..., 3]+180.) % 360.-180.
    return result


def _policy():
    return dict(command=copy.deepcopy(COMMAND), scales=SCALES.copy(), maximumNearestStandardDistance=1.,
        maximumM4DifferenceDeg=15., maximumImageAxisDifferenceDeg=20., minimumRawSources=2,
        maximumCommands=1, initialFixedLiftGoalPixels=80.,
        supportMeaning='explicit_unloaded_to_held_development_transfer_hypothesis_not_load_or_safety_validation')


def fit(rows, *, audit_sources):
    _require(isinstance(rows, list) and len(rows) >= 2 and callable(audit_sources), 'lift_two_raw_audited_sources_required')
    _require(audit_sources(copy.deepcopy(rows)) == rows, 'lift_source_audit_changed')
    sessions, sources, contracts = set(), set(), set()
    for row in rows:
        _require(isinstance(row, dict) and isinstance(row.get('sessionId'), str) and bool(row['sessionId'])
                 and row['sessionId'] not in sessions and _sha(row.get('sourceSha256'))
                 and row['sourceSha256'] not in sources, 'lift_unique_original_sources_required')
        _require(row.get('command') == COMMAND and type(row['command'].get('percent')) is int
                 and type(row['command'].get('durationMs')) is int, 'lift_exact_UP_training_command_required')
        _state(row.get('before')); _state(row.get('after'))
        _require(_sha(row.get('observationContractSha256')), 'lift_observation_contract_required')
        evidence = row.get('sourceEvidence')
        _require(isinstance(evidence, dict) and all(_sha(evidence.get(k)) for k in
            ('beforeImageSha256', 'afterImageSha256', 'beforeSensorSha256', 'afterSensorSha256',
             'beforeGripperImageSha256', 'afterGripperImageSha256'))
            and evidence.get('trainingHeldLoadVerified') is False, 'lift_raw_two_camera_sensor_and_load_provenance')
        sessions.add(row['sessionId']); sources.add(row['sourceSha256']); contracts.add(row['observationContractSha256'])
    _require(len(contracts) == 1, 'lift_observation_contract_changed')
    before = np.array([row['before'] for row in rows]); after = np.array([row['after'] for row in rows])
    origin = before[0]; z = _difference(before, origin)/SCALES; standardized_mean = z.mean(axis=0); z -= standardized_mean
    delta = _difference(after, before); delta_mean = delta.mean(axis=0)
    coefficients = np.linalg.solve(z.T@z+np.eye(4), z.T@(delta-delta_mean))
    residual = delta-(delta_mean+z@coefficients)
    return dict(schemaVersion=SCHEMA, supportPolicy=_policy(), observationContractSha256=next(iter(contracts)),
        beforeStates=before.tolist(), origin=origin.tolist(), standardizedMean=standardized_mean.tolist(),
        deltaMean=delta_mean.tolist(), coefficients=coefficients.tolist(), trainingRows=copy.deepcopy(rows),
        trainingVirtualPointRmsePx=float(np.sqrt(np.mean(residual[:, :2]**2))),
        trainingHeldLoadVerified=False, transferToHeldLoadUnverified=True,
        controlEligible=False, developmentOnly=True, independentTest=False, calibratedUncertainty=False,
        uncertaintyInterval=None, graspVerified=False, liftVerified=False, originalModelsChanged=False)


def _current(model, current):
    _require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA and model.get('supportPolicy') == _policy()
             and model.get('trainingHeldLoadVerified') is False and model.get('transferToHeldLoadUnverified') is True
             and model.get('controlEligible') is False and model.get('developmentOnly') is True,
             'lift_explicit_transfer_model_required')
    _require(isinstance(current, dict) and current.get('observationContractSha256') == model['observationContractSha256']
             and _sha(current.get('observationSha256')), 'lift_current_observation_binding')
    return _state(current.get('state'))


def make_goal(model, initial_current):
    state = _current(model, initial_current)
    _require(state[1] >= 80., 'lift_fixed_goal_outside_image')
    return dict(schemaVersion=SCHEMA+'.fixed_goal', modelSha256=digest(model),
        initialObservation=copy.deepcopy(initial_current), initialInputSha256=digest(initial_current),
        targetVirtualPointPx=[float(state[0]), float(state[1]-80.)],
        liftGoalPixels=80., maximumCommands=1,
        meaning='fixed_virtual_point_goal_not_actual_object_height_or_retained_lift')


def predict(model, current):
    state = _current(model, current)
    before = np.array(model['beforeStates'], dtype=float); difference = _difference(before, state)
    distances = np.linalg.norm(difference/SCALES, axis=1); nearest = int(np.argmin(distances))
    m4_difference = abs(float(difference[nearest, 2])); axis_difference = abs(float(difference[nearest, 3]))
    _require(distances[nearest] <= 1. and m4_difference <= 15. and axis_difference <= 20., 'lift_current_outside_local_transfer_support')
    z = _difference(state, model['origin'])/SCALES-np.array(model['standardizedMean'])
    baseline = state+np.array(model['deltaMean']); prediction = baseline+z@np.array(model['coefficients'])
    prediction[3] = (prediction[3]+180.) % 360.-180.; baseline[3] = (baseline[3]+180.) % 360.-180.
    _state(prediction.tolist()); _state(baseline.tolist())
    return dict(command=copy.deepcopy(COMMAND), predictedState=prediction.tolist(), commandMeanState=baseline.tolist(),
        support=dict(nearestTrainingRow=nearest, nearestStandardDistance=float(distances[nearest]),
            M4DifferenceDeg=m4_difference, imageAxisDifferenceDeg=axis_difference, safetyBound=False),
        transferToHeldLoadUnverified=True, calibratedUncertainty=False)


def select(model, current, fixed_goal, *, commands_used):
    state = _current(model, current)
    _require(type(commands_used) is int and commands_used in (0, 1), 'lift_one_command_budget')
    _require(isinstance(fixed_goal, dict)
             and fixed_goal == make_goal(model, fixed_goal.get('initialObservation')),
             'lift_initial_fixed_goal_changed')
    target = fixed_goal['targetVirtualPointPx']; error = float(np.linalg.norm(state[:2]-target))
    stop = dict(kind='stop', command=None, predictedState=state.tolist(), errorPx=error)
    ranked, rejected = [stop], []
    if commands_used == 0:
        try:
            result = predict(model, current)
            ranked.append(dict(kind='action', errorPx=float(np.linalg.norm(np.array(result['predictedState'][:2])-target)),
                commandMeanErrorPx=float(np.linalg.norm(np.array(result['commandMeanState'][:2])-target)), **result))
        except ValueError as exc: rejected.append(dict(command=copy.deepcopy(COMMAND), reason=str(exc)))
    ranked.sort(key=lambda item: item['errorPx'])
    return dict(schemaVersion=SCHEMA+'.decision', modelSha256=digest(model), inputSha256=digest(current),
        fixedGoalSha256=digest(fixed_goal), targetVirtualPointPx=copy.deepcopy(target),
        selected=copy.deepcopy(ranked[0]), candidates=ranked, rejected=rejected,
        selectionReason='one_command_budget_exhausted' if commands_used else
            'minimum_predicted_virtual_goal_error' if ranked[0]['kind'] == 'action' else 'stop_has_minimum_virtual_goal_error',
        commandsUsed=commands_used, maximumCommands=1, transferToHeldLoadUnverified=True,
        currentHeldLoadTrainingSupportVerified=False, actualObjectTableSeparationVerified=False,
        retainedLiftVerified=False, calibratedUncertainty=False, safetyBound=False,
        controlEligible=False, developmentOnly=True)
