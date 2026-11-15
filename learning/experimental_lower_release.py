"""UP 전이 역함수의 미검증 DOWN 가설.

DOWN 자료로 학습한 모델이라고 표현하지 않는다. 실제 탁상 접촉·힘·정지는
외부 영상/장치 actor가 판정한다. 원 UP·M1 모델과 원본은 변경하지 않는다.
"""
from __future__ import annotations
import copy

import numpy as np

from learning import experimental_lift_response as lift
from learning.near_gripper_tip_observer import digest

SCHEMA = 'motionbrain.experimental_inverse_UP_lowering.v1'
DOWN = dict(joint='elbow', direction='down', percent=70, durationMs=1000)


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def _policy():
    return dict(inferenceCommand=copy.deepcopy(DOWN), originalSourceCommand=copy.deepcopy(lift.COMMAND),
        scales=lift.SCALES.copy(), maximumNearestStandardDistance=1., maximumM4DifferenceDeg=15.,
        maximumImageAxisDifferenceDeg=20., maximumCommands=1,
        meaning='inverse_UP_response_development_hypothesis_not_observed_DOWN_dynamics_or_contact_force_bound')


def fit_lowering(rows, *, audit_sources):
    # The forward audit only accepts actual completed UP source commands. No
    # swapped record is passed off as an observed DOWN transition.
    forward = lift.fit(rows, audit_sources=audit_sources)
    before = np.array([row['after'] for row in rows]); after = np.array([row['before'] for row in rows])
    origin = before[0]; z = lift._difference(before, origin)/lift.SCALES
    mean = z.mean(axis=0); z -= mean
    delta = lift._difference(after, before); delta_mean = delta.mean(axis=0)
    coefficients = np.linalg.solve(z.T@z+np.eye(4), z.T@(delta-delta_mean))
    return dict(schemaVersion=SCHEMA, supportPolicy=_policy(), observationContractSha256=forward['observationContractSha256'],
        originalForwardModelSha256=digest(forward), originalUPTrainingRows=copy.deepcopy(rows),
        conditioningBeforeStates=before.tolist(), origin=origin.tolist(), standardizedMean=mean.tolist(),
        deltaMean=delta_mean.tolist(), coefficients=coefficients.tolist(), directDOWNTrainingRows=0,
        inferredInverseUPRows=len(rows), directionAsymmetryVerified=False, heldLoadTransferVerified=False,
        controlEligible=False, developmentOnly=True, calibratedUncertainty=False, uncertaintyInterval=None,
        originalModelsChanged=False, physicalTableSupportVerified=False)


def _current(model, current):
    _require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA and model.get('supportPolicy') == _policy()
             and model.get('directDOWNTrainingRows') == 0 and model.get('directionAsymmetryVerified') is False
             and model.get('heldLoadTransferVerified') is False and model.get('controlEligible') is False,
             'lower_explicit_inverse_hypothesis_required')
    _require(isinstance(current, dict) and current.get('observationContractSha256') == model['observationContractSha256']
             and lift._sha(current.get('observationSha256')), 'lower_current_raw_binding')
    return lift._state(current.get('state'))


def make_lowering_goal(model, pre_lift_observation):
    state = _current(model, pre_lift_observation)
    return dict(schemaVersion=SCHEMA+'.saved_pre_lift_goal', modelSha256=digest(model),
        preLiftObservation=copy.deepcopy(pre_lift_observation), sourceObservationSha256=digest(pre_lift_observation),
        targetVirtualPointPx=state[:2].tolist(), maximumCommands=1,
        meaning='return_to_saved_pre_lift_image_point_not_table_contact_or_force_verification')


def predict_lowering(model, current):
    state = _current(model, current); before = np.array(model['conditioningBeforeStates'])
    difference = lift._difference(before, state); distances = np.linalg.norm(difference/lift.SCALES, axis=1)
    nearest = int(np.argmin(distances)); m4 = abs(float(difference[nearest, 2])); axis = abs(float(difference[nearest, 3]))
    _require(distances[nearest] <= 1. and m4 <= 15. and axis <= 20., 'lower_current_outside_inverse_support')
    z = lift._difference(state, model['origin'])/lift.SCALES-np.array(model['standardizedMean'])
    baseline = state+np.array(model['deltaMean']); predicted = baseline+z@np.array(model['coefficients'])
    predicted[3] = (predicted[3]+180.) % 360.-180.; baseline[3] = (baseline[3]+180.) % 360.-180.
    lift._state(predicted.tolist()); lift._state(baseline.tolist())
    return dict(command=copy.deepcopy(DOWN), predictedState=predicted.tolist(), commandMeanState=baseline.tolist(),
        support=dict(nearestStandardDistance=float(distances[nearest]), M4DifferenceDeg=m4,
            imageAxisDifferenceDeg=axis, safetyBound=False), directDOWNTrainingRows=0,
        inverseUPHypothesis=True, directionAsymmetryVerified=False, heldLoadTransferVerified=False)


def select_lowering(model, current, fixed_goal, *, commands_used):
    state = _current(model, current)
    _require(type(commands_used) is int and commands_used in (0, 1), 'lower_one_command_budget')
    _require(isinstance(fixed_goal, dict) and fixed_goal == make_lowering_goal(model, fixed_goal.get('preLiftObservation')),
             'lower_saved_goal_changed')
    target = fixed_goal['targetVirtualPointPx']; error = float(np.linalg.norm(state[:2]-target))
    stop = dict(kind='stop', command=None, predictedState=state.tolist(), errorPx=error)
    ranked, rejected = [stop], []
    if commands_used == 0:
        try:
            result = predict_lowering(model, current)
            ranked.append(dict(kind='action', errorPx=float(np.linalg.norm(np.array(result['predictedState'][:2])-target)),
                commandMeanErrorPx=float(np.linalg.norm(np.array(result['commandMeanState'][:2])-target)), **result))
        except ValueError as exc: rejected.append(dict(command=copy.deepcopy(DOWN), reason=str(exc)))
    ranked.sort(key=lambda row: row['errorPx'])
    return dict(schemaVersion=SCHEMA+'.decision', modelSha256=digest(model), inputSha256=digest(current),
        fixedGoalSha256=digest(fixed_goal), selected=copy.deepcopy(ranked[0]), candidates=ranked, rejected=rejected,
        targetVirtualPointPx=copy.deepcopy(target), commandsUsed=commands_used, maximumCommands=1,
        directDOWNTrainingRows=0, inverseUPHypothesis=True, directionAsymmetryVerified=False,
        physicalTableSupportVerified=False, releaseAuthorizedByModel=False, controlEligible=False)
