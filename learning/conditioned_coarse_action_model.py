"""현재 영상·동작 전 M4로 후보 반응을 비교하는 별도 국소 개발 모델.

지원 거리는 학습 자료에 대한 개발 정책이며 이동 여유·정지 거리·예측 신뢰구간을
보장하지 않는다. 이전 모델은 변경하거나 승격하지 않고 원본 전이를 다시 감사한다.
모터 출력·OFF·장치 경계는 이 모듈 밖의 유한 actor가 담당한다.
"""
from __future__ import annotations
import copy
import hashlib
import json
import math
import re

import numpy as np

SCHEMA = 'motionbrain.conditioned_coarse_action_ridge.v1'
STATE_FIELDS = ['objectMinusToolProxyXpx', 'objectMinusToolProxyYpx', 'm4SensorDeg', 'toolAxisImageAngleDeg']
SCALES = [180., 180., 10., 20.]
COMMANDS = [dict(joint='elbow', direction=direction, percent=70, durationMs=1000) for direction in ('up', 'down')]
COMMANDS += [dict(joint='shoulder', direction='down', percent=70, durationMs=1500)]
COMMANDS += [dict(joint='shoulder', direction='down', percent=70, durationMs=1000)]
MINIMUM_DISTANCE_PX = 80.
MINIMUM_PREDICTED_DISTANCE_PX = 40.
MAXIMUM_COMMANDS = 3


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _state(value):
    _require(isinstance(value, list) and len(value) == 4
             and all(type(v) in (int, float) and math.isfinite(v) for v in value), 'conditioned_finite_4d_state')
    _require(0 <= value[2] < 360 and -180 <= value[3] <= 180, 'conditioned_state_angle_domain')
    return np.array(value, dtype=float)


def _key(command):
    _require(isinstance(command, dict) and set(command) == {'joint', 'direction', 'percent', 'durationMs'}
             and type(command.get('percent')) is int and type(command.get('durationMs')) is int
             and command in COMMANDS, 'conditioned_explicit_candidate_command')
    return tuple(command[k] for k in ('joint', 'direction', 'percent', 'durationMs'))


def _difference(a, b):
    difference = np.asarray(a, dtype=float)-np.asarray(b, dtype=float)
    difference[..., 3] = (difference[..., 3]+180.) % 360.-180.
    return difference


def _policy():
    return dict(standardScales=SCALES.copy(), maximumNearestStandardDistance=1.,
        maximumM4DifferenceDeg=10., maximumToolAxisDifferenceDeg=20.,
        minimumTransitionsPerCommand=2, maximumCommands=MAXIMUM_COMMANDS,
        minimumCoarseDistancePx=MINIMUM_DISTANCE_PX,
        minimumPredictedDistancePx=MINIMUM_PREDICTED_DISTANCE_PX,
        distanceThresholdMeaning='development_phase_transition_not_collision_clearance_guarantee', localExtrapolation=True,
        safetyBound=False, calibratedUncertainty=False)


def fit(rows, *, audit_sources):
    _require(isinstance(rows, list) and bool(rows) and callable(audit_sources), 'conditioned_raw_audit_required')
    audited = audit_sources(copy.deepcopy(rows))
    _require(audited == rows, 'conditioned_raw_audit_changed')
    seen_sessions, seen_sources, contracts = set(), set(), set()
    keys = []
    for row in rows:
        _require(isinstance(row, dict) and isinstance(row.get('sessionId'), str) and bool(row['sessionId'])
                 and row['sessionId'] not in seen_sessions, 'conditioned_unique_session_required')
        source = row.get('sourceSha256')
        _require(isinstance(source, str) and re.fullmatch('[0-9a-f]{64}', source) is not None
                 and source not in seen_sources, 'conditioned_unique_source_required')
        contract = row.get('observationContractSha256')
        _require(isinstance(contract, str) and re.fullmatch('[0-9a-f]{64}', contract) is not None,
                 'conditioned_observation_contract_required')
        evidence = row.get('sourceEvidence')
        _require(isinstance(evidence, dict) and all(isinstance(evidence.get(k), str)
                 and re.fullmatch('[0-9a-f]{64}', evidence[k]) is not None for k in
                 ('beforeImageSha256', 'afterImageSha256', 'beforeSensorSha256', 'afterSensorSha256')),
                 'conditioned_image_sensor_provenance_required')
        _state(row.get('before')); _state(row.get('after')); keys.append(_key(row.get('command')))
        seen_sessions.add(row['sessionId']); seen_sources.add(source); contracts.add(contract)
    _require(len(contracts) == 1, 'conditioned_observation_contract_changed')
    entries, deficits = [], []
    for key in sorted(set(keys)):
        selected = [row for row, row_key in zip(rows, keys) if row_key == key]
        command = copy.deepcopy(selected[0]['command'])
        if len(selected) < 2:
            deficits.append(dict(command=command, rows=len(selected), minimum=2)); continue
        before = np.array([r['before'] for r in selected], dtype=float)
        after = np.array([r['after'] for r in selected], dtype=float)
        origin = before[0].copy()
        z = _difference(before, origin)/SCALES
        mean = z.mean(axis=0); z -= mean
        delta = _difference(after, before); delta_mean = delta.mean(axis=0)
        coefficients = np.linalg.solve(z.T@z+np.eye(4), z.T@(delta-delta_mean))
        residual = delta-(delta_mean+z@coefficients)
        entries.append(dict(command=command, rows=len(selected), beforeStates=before.tolist(),
            origin=origin.tolist(), standardizedMean=mean.tolist(), deltaMean=delta_mean.tolist(),
            coefficients=coefficients.tolist(), trainingResidualXYRmse=float(np.sqrt(np.mean(residual[:, :2]**2))),
            sourceSha256s=[r['sourceSha256'] for r in selected]))
    return dict(schemaVersion=SCHEMA, stateFields=STATE_FIELDS.copy(), supportPolicy=_policy(),
        models=entries, deficits=deficits, trainingRows=copy.deepcopy(rows),
        observationContractSha256=next(iter(contracts)), controlEligible=False, developmentOnly=True,
        independentTest=False, calibratedUncertainty=False, uncertaintyInterval=None,
        assumptions=['local_response_transfer_unverified', 'M4_and_image_heading_conditioning_not_full_pose_or_load',
                     'nearest_support_policy_not_safety_or_accuracy_bound'],
        newFamilyExplicitSourceReaudit=True, originalModelsChanged=False)


def _validate_model(model):
    _require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA
             and model.get('stateFields') == STATE_FIELDS and model.get('supportPolicy') == _policy()
             and model.get('controlEligible') is False and model.get('developmentOnly') is True
             and model.get('calibratedUncertainty') is False, 'conditioned_development_model_required')
    _require(isinstance(model.get('models'), list) and isinstance(model.get('trainingRows'), list), 'conditioned_model_parameters')
    keys = set()
    for entry in model['models']:
        key = _key(entry.get('command')); _require(key not in keys, 'conditioned_duplicate_model_command'); keys.add(key)
        _require(type(entry.get('rows')) is int and entry['rows'] >= 2
                 and isinstance(entry.get('beforeStates'), list) and len(entry['beforeStates']) == entry['rows'],
                 'conditioned_model_support')
        for before in entry['beforeStates']: _state(before)
        _state(entry.get('origin'))
        for name, shape in [('standardizedMean', (4,)), ('deltaMean', (4,)), ('coefficients', (4, 4))]:
            try: value = np.array(entry.get(name), dtype=float)
            except (TypeError, ValueError): raise ValueError('conditioned_model_parameters') from None
            _require(value.shape == shape and np.isfinite(value).all(), 'conditioned_model_parameters')


def predict(model, current, command):
    _validate_model(model); state = _state(current); key = _key(command)
    entries = [entry for entry in model['models'] if _key(entry['command']) == key]
    _require(len(entries) == 1 and entries[0]['rows'] >= 2, 'conditioned_two_transitions_required')
    entry = entries[0]; before = np.array(entry['beforeStates'], dtype=float)
    difference = _difference(state, before)
    distances = np.linalg.norm(difference/SCALES, axis=1)
    nearest = int(np.argmin(distances)); nearest_difference = difference[nearest]
    support = dict(nearestTrainingRow=nearest, nearestStandardDistance=float(distances[nearest]),
        M4DifferenceDeg=abs(float(nearest_difference[2])), toolAxisDifferenceDeg=abs(float(nearest_difference[3])),
        developmentLocalExtrapolation=True, safetyBound=False)
    _require(support['nearestStandardDistance'] <= 1.
             and support['M4DifferenceDeg'] <= 10. and support['toolAxisDifferenceDeg'] <= 20.,
             'conditioned_state_outside_local_support')
    z = _difference(state, np.array(entry['origin']))/SCALES-np.array(entry['standardizedMean'])
    mean_delta = np.array(entry['deltaMean']); delta = mean_delta+z@np.array(entry['coefficients'])
    prediction, baseline = state+delta, state+mean_delta
    prediction[3] = (prediction[3]+180.) % 360.-180.; baseline[3] = (baseline[3]+180.) % 360.-180.
    _state(prediction.tolist()); _state(baseline.tolist())
    return dict(predictedState=prediction.tolist(), commandMeanState=baseline.tolist(), support=support,
        calibratedUncertainty=False, uncertaintyInterval=None,
        trainingResidualXYRmse=entry['trainingResidualXYRmse'])


def select(model, current, target_xy, candidates, *, commands_used):
    _validate_model(model); state = _state(current)
    _require(isinstance(target_xy, list) and len(target_xy) == 2
             and all(type(v) in (int, float) and math.isfinite(v) for v in target_xy), 'conditioned_target_xy_required')
    _require(type(commands_used) is int and 0 <= commands_used <= MAXIMUM_COMMANDS, 'conditioned_command_budget')
    _require(isinstance(candidates, list) and bool(candidates), 'conditioned_candidates_required')
    keys = [_key(command) for command in candidates]
    _require(len(keys) == len(set(keys)), 'conditioned_duplicate_candidate')
    distance = float(np.linalg.norm(state[:2]-target_xy))
    stop = dict(kind='stop', command=None, predictedState=state.tolist(), errorPx=distance,
                commandMeanErrorPx=distance)
    ranked, rejected = [stop], []
    reason = 'minimum_predicted_xy_error'
    if commands_used >= MAXIMUM_COMMANDS: reason = 'command_budget_exhausted'
    elif distance < MINIMUM_DISTANCE_PX: reason = 'current_near_gripper_phase'
    else:
        for command in candidates:
            try:
                prediction = predict(model, current, command)
                error = float(np.linalg.norm(np.array(prediction['predictedState'][:2])-target_xy))
                baseline_error = float(np.linalg.norm(np.array(prediction['commandMeanState'][:2])-target_xy))
                _require(error >= MINIMUM_PREDICTED_DISTANCE_PX, 'predicted_near_gripper_phase_requires_separate_approach')
                ranked.append(dict(kind='action', command=copy.deepcopy(command), errorPx=error,
                    commandMeanErrorPx=baseline_error,
                    predictedPhase='near_gripper' if error < MINIMUM_DISTANCE_PX else 'coarse', **prediction))
            except ValueError as exc:
                rejected.append(dict(command=copy.deepcopy(command), reason=str(exc)))
    ranked.sort(key=lambda row: row['errorPx'])
    if ranked[0]['kind'] == 'stop' and reason == 'minimum_predicted_xy_error': reason = 'stop_has_minimum_predicted_xy_error'
    return dict(schemaVersion=SCHEMA+'.decision', modelSha256=digest(model),
        observationContractSha256=model['observationContractSha256'], inputState=state.tolist(), targetXY=copy.deepcopy(target_xy),
        selected=copy.deepcopy(ranked[0]), candidates=ranked, rejected=rejected, selectionReason=reason,
        commandsUsed=commands_used, maximumCommands=MAXIMUM_COMMANDS,
        currentDistancePx=distance, minimumCoarseDistancePx=MINIMUM_DISTANCE_PX,
        minimumPredictedDistancePx=MINIMUM_PREDICTED_DISTANCE_PX,
        controlEligible=False, developmentOnly=True, calibratedUncertainty=False,
        safetyBound=False, graspVerified=False, liftVerified=False)
