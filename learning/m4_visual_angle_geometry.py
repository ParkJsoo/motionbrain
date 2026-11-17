"""원본 전이로 학습하는 국소 M4 각도→가상 영상점 기하 모델.

영상의 유효 회전중심은 실물 관절 중심·깊이 교정이 아니다. 계획 목표각은
장치가 달성할 미래 각도의 가설이며, 실행 후 영상·센서로 따로 확인해야 한다.
출력·정지·통신·장치 상태는 별도 유한 actor가 관리한다.
"""
from __future__ import annotations
import copy
import hashlib
import json
import math
import re

import numpy as np

SCHEMA = 'motionbrain.m4_visual_angle_geometry.v1'
TRAINING_COMMAND = dict(joint='shoulder', direction='down', percent=70, durationMs=1000)


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _number(value): return type(value) in (int, float) and math.isfinite(value)


def _point(value):
    _require(isinstance(value, list) and len(value) == 2 and all(_number(v) for v in value)
             and 0 <= value[0] < 1920 and 0 <= value[1] < 1080, 'geometry_image_point_required')
    return np.array(value, dtype=float)


def _observation(value):
    _require(isinstance(value, dict) and _number(value.get('m4Deg')) and 0 <= value['m4Deg'] < 360,
             'geometry_observation_required')
    point = _point(value.get('virtualPointPx')); target = _point(value.get('objectCenterPx'))
    return point, target, float(value['m4Deg'])


def _sha(value): return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def _rotation(degrees):
    theta = np.radians(degrees); c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def _policy():
    return dict(betaRange=[.8, 1.2], betaGridSteps=401, minimumRawTransitions=2,
        maximumPlannedDecreaseDeg=8., sourceAngleExpansionDeg=8., downOnly=True,
        minimumTargetAngleDeg=127.58, maximumTargetAngleDeg=296.12,
        boundaryMeaning='existing_manual_boundaries_plus_four_degree_margin_not_global_collision_envelope',
        maximumCommands=3, supportMeaning='development_local_angle_hypothesis_not_safety_or_accuracy_bound')


def fit(rows, *, audit_sources):
    _require(isinstance(rows, list) and len(rows) >= 2 and callable(audit_sources), 'geometry_two_raw_audited_sources_required')
    _require(audit_sources(copy.deepcopy(rows)) == rows, 'geometry_raw_audit_changed')
    sessions, sources, contracts = set(), set(), set()
    before_points, after_points, changes, angles, object_drifts = [], [], [], [], []
    for row in rows:
        _require(isinstance(row, dict) and isinstance(row.get('sessionId'), str) and bool(row['sessionId'])
                 and row['sessionId'] not in sessions and _sha(row.get('sourceSha256'))
                 and row['sourceSha256'] not in sources, 'geometry_unique_original_sources_required')
        _require(row.get('command') == TRAINING_COMMAND and type(row['command'].get('percent')) is int
                 and type(row['command'].get('durationMs')) is int, 'geometry_completed_down_training_command_required')
        _require(_sha(row.get('observationContractSha256')), 'geometry_observation_contract_required')
        proof = row.get('sourceEvidence')
        _require(isinstance(proof, dict) and all(_sha(proof.get(k)) for k in
            ('beforeImageSha256', 'afterImageSha256', 'beforeSensorSha256', 'afterSensorSha256')),
            'geometry_image_sensor_provenance_required')
        p0, o0, a0 = _observation(row.get('before')); p1, o1, a1 = _observation(row.get('after'))
        delta = a1-a0
        _require(delta <= -.2, 'geometry_observed_down_progress_required')
        before_points.append(p0); after_points.append(p1); changes.append(delta); angles.extend([a0, a1])
        object_drifts.append(float(np.linalg.norm(o1-o0)))
        sessions.add(row['sessionId']); sources.add(row['sourceSha256']); contracts.add(row['observationContractSha256'])
    _require(len(contracts) == 1, 'geometry_observation_contract_changed')
    best = None; objectives = []
    for beta in np.linspace(.8, 1.2, 401):
        rotations = [_rotation(float(beta)*delta) for delta in changes]
        matrix = np.vstack([np.eye(2)-rotation for rotation in rotations])
        labels = np.concatenate([p1-rotation@p0 for p0, p1, rotation in zip(before_points, after_points, rotations)])
        pivot, _, rank, _ = np.linalg.lstsq(matrix, labels, rcond=None)
        _require(rank == 2 and np.isfinite(pivot).all(), 'geometry_pivot_not_identifiable')
        residual = matrix@pivot-labels
        loss = float(residual@residual); objectives.append(loss)
        if best is None or loss < best['loss']:
            best = dict(beta=float(beta), pivotPx=pivot.tolist(), loss=loss)
    baseline = np.mean([(p1-p0)/delta for p0, p1, delta in zip(before_points, after_points, changes)], axis=0)
    residuals = [p1-(_rotation(best['beta']*delta)@(p0-np.array(best['pivotPx']))+best['pivotPx'])
                 for p0, p1, delta in zip(before_points, after_points, changes)]
    return dict(schemaVersion=SCHEMA, supportPolicy=_policy(), beta=best['beta'], pivotPx=best['pivotPx'],
        pivotMeaning='effective_local_image_rotation_center_not_calibrated_physical_joint_center',
        commandMeanDisplacementPerDegreePx=baseline.tolist(),
        sourceAngleRangeDeg=[min(angles), max(angles)], trainingRows=copy.deepcopy(rows),
        observationContractSha256=next(iter(contracts)),
        fitDiagnostics=dict(rows=len(rows), meanPointErrorPx=float(np.mean(np.linalg.norm(residuals, axis=1))),
            maximumPointErrorPx=float(np.max(np.linalg.norm(residuals, axis=1))),
            betaAtSearchBoundary=best['beta'] in (.8, 1.2), betaIdentifiabilityVerified=False,
            betaGridObjectiveRange=[min(objectives), max(objectives)], objectCenterDriftPx=object_drifts),
        controlEligible=False, developmentOnly=True, independentTest=False,
        calibratedUncertainty=False, uncertaintyInterval=None, originalModelsChanged=False,
        assumptions=['local_projection_geometry_unverified', 'target_angle_attainment_requires_after_sensor_check',
            'object_stationarity_unverified_for_future_move', 'source_angle_expansion_is_development_hypothesis',
            'training_residual_is_not_prediction_interval_or_clearance'])


def _validate(model):
    _require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA
             and model.get('supportPolicy') == _policy() and model.get('controlEligible') is False
             and model.get('developmentOnly') is True and model.get('calibratedUncertainty') is False
             and _sha(model.get('observationContractSha256')) and _number(model.get('beta'))
             and .8 <= model['beta'] <= 1.2, 'geometry_development_model_required')
    for name in ('pivotPx', 'commandMeanDisplacementPerDegreePx', 'sourceAngleRangeDeg'):
        value = model.get(name)
        _require(isinstance(value, list) and len(value) == 2 and all(_number(v) for v in value), 'geometry_model_parameters')
    _require(model['sourceAngleRangeDeg'][0] <= model['sourceAngleRangeDeg'][1], 'geometry_source_angle_range')


def _current(model, current):
    _validate(model)
    _require(isinstance(current, dict)
             and current.get('observationContractSha256') == model['observationContractSha256']
             and _sha(current.get('observationSha256')), 'geometry_current_raw_observation_binding')
    point, target, angle = _observation(current)
    policy = model['supportPolicy']; low, high = model['sourceAngleRangeDeg']
    _require(low-8. <= angle <= high+8. and policy['minimumTargetAngleDeg'] < angle < policy['maximumTargetAngleDeg'],
             'geometry_current_angle_outside_local_support')
    return point, target, angle


def _objective(observed_target, objective_point_px):
    if objective_point_px is None:
        return observed_target, {}
    point = _point(objective_point_px)
    binding = dict(schemaVersion=SCHEMA+'.objective', pointPx=point.tolist(),
        coordinateFrame='external_image_1920x1080',
        meaning='explicit_2d_planning_goal_not_observed_object_center_or_physical_grasp_point',
        learnedModelChanged=False)
    return point, dict(objectiveBinding=binding, objectiveBindingSha256=digest(binding))


def predict(model, current, target_angle_deg, *, objective_point_px=None):
    """명시 2D 목표는 비용만 바꾸며 원 관측·학습 모델·지원 한도는 유지한다."""
    point, observed_target, angle = _current(model, current)
    target, objective = _objective(observed_target, objective_point_px); policy = model['supportPolicy']
    _require(_number(target_angle_deg), 'geometry_target_angle_required')
    delta = float(target_angle_deg)-angle
    _require(-8. <= delta < 0., 'geometry_planned_down_zero_to_eight_degrees_only')
    low, high = model['sourceAngleRangeDeg']
    _require(low-8. <= target_angle_deg <= high+8.
             and policy['minimumTargetAngleDeg'] < target_angle_deg < policy['maximumTargetAngleDeg'],
             'geometry_target_angle_outside_local_support')
    pivot = np.array(model['pivotPx'])
    predicted = _rotation(model['beta']*delta)@(point-pivot)+pivot
    baseline = point+delta*np.array(model['commandMeanDisplacementPerDegreePx'])
    _point(predicted.tolist()); _point(baseline.tolist())
    return dict(targetAngleDeg=float(target_angle_deg), deltaAngleDeg=delta,
        predictedVirtualPointPx=predicted.tolist(), errorPx=float(np.linalg.norm(predicted-target)),
        commandMeanPointPx=baseline.tolist(), commandMeanErrorPx=float(np.linalg.norm(baseline-target)),
        support=dict(sourceAngleRangeDeg=model['sourceAngleRangeDeg'], sourceAngleExpansionDeg=8.,
            maximumPlannedDecreaseDeg=8., localHypothesis=True, safetyBound=False),
        calibratedUncertainty=False, uncertaintyInterval=None, **objective)


def select(model, current, goal_candidates, *, commands_used=0, objective_point_px=None):
    """모든 후보·기준선·STOP을 같은 목표로 비교하고 목표 바인딩은 원 입력과 분리한다."""
    point, observed_target, angle = _current(model, current)
    target, objective = _objective(observed_target, objective_point_px)
    _require(type(commands_used) is int and 0 <= commands_used <= 3, 'geometry_command_budget')
    _require(isinstance(goal_candidates, list) and 1 <= len(goal_candidates) <= 32
             and all(_number(v) for v in goal_candidates)
             and len(set(goal_candidates)) == len(goal_candidates), 'geometry_explicit_distinct_goal_candidates')
    distance = float(np.linalg.norm(point-target))
    stop = dict(kind='stop', targetAngleDeg=None, deltaAngleDeg=0., predictedVirtualPointPx=point.tolist(),
                errorPx=distance, commandMeanErrorPx=distance, **copy.deepcopy(objective))
    ranked, rejected = [stop], []
    if commands_used < 3:
        for candidate in goal_candidates:
            if candidate == angle: continue
            try: ranked.append(dict(kind='target_angle', **predict(model, current, candidate,
                                                                   objective_point_px=objective_point_px)))
            except ValueError as exc: rejected.append(dict(targetAngleDeg=candidate, reason=str(exc)))
    ranked.sort(key=lambda row: row['errorPx'])
    reason = ('command_budget_exhausted' if commands_used == 3 else
              'minimum_predicted_xy_error' if ranked[0]['kind'] == 'target_angle' else 'stop_has_minimum_predicted_xy_error')
    return dict(schemaVersion=SCHEMA+'.decision', modelSha256=digest(model), inputSha256=digest(current),
        observationContractSha256=model['observationContractSha256'], selected=copy.deepcopy(ranked[0]),
        candidates=ranked, rejected=rejected, selectionReason=reason, commandsUsed=commands_used,
        maximumCommands=3, currentDistancePx=distance, controlEligible=False, developmentOnly=True,
        calibratedUncertainty=False, safetyBound=False, graspVerified=False, liftVerified=False, **copy.deepcopy(objective))
