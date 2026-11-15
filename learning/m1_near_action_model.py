"""고정 첫 영상 폭을 목표로 쓰는 별도 M1 근접 개발 모델. 접촉·집기 성공 모델이 아니다."""
from __future__ import annotations
import copy
import math
import re

import numpy as np

from learning.near_gripper_tip_observer import validate_observation, digest

SCHEMA = 'motionbrain.m1_near_visible_tip_ridge.v1'
COMMANDS = [dict(joint='gripper', direction=direction, percent=70, durationMs=1000) for direction in ('close', 'open')]


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def _sha(value): return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def _key(command):
    _require(isinstance(command, dict) and set(command) == {'joint', 'direction', 'percent', 'durationMs'}
             and type(command.get('percent')) is int and type(command.get('durationMs')) is int
             and command in COMMANDS, 'near_model_explicit_command')
    return command['direction']


def _policy():
    return dict(minimumRowsPerCommand=2, localXYDifferencePx=30., localGapDifferencePx=60.,
        maximumCommands=6, targetGapFromReferenceWidthPx=-4.,
        minimumGapFromReferenceWidthPx=-12., maximumOpeningAboveOriginalBaselinePx=5.,
        safetyBound=False, referenceWidthChangesDuringSession=False,
        meaning='development_pixel_experiment_bounds_not_contact_force_or_collision_guarantee')


def fit(rows, *, reference, audit_sources):
    _require(isinstance(rows, list) and bool(rows) and callable(audit_sources), 'near_model_raw_two_camera_audit_required')
    _require(audit_sources(copy.deepcopy(rows)) == rows, 'near_model_source_audit_changed')
    reference_state = validate_observation(reference)
    _require(rows[0].get('before') == reference, 'near_model_first_before_reference_required')
    _require(reference['nearClosingVisibilityGate']['passed'] is True, 'near_model_reference_visibility_gate')
    width = reference['objectVisibleWidthPx']; baseline = float(reference_state[2])
    _require(type(width) in (int, float) and math.isfinite(width) and width > 24
             and baseline >= width-4., 'near_model_reference_width_and_opening')
    minimum, maximum = float(width-12.), baseline+5.
    contracts, sessions, sources = set(), set(), set(); keys = []
    for row in rows:
        _require(isinstance(row, dict) and isinstance(row.get('sessionId'), str) and bool(row['sessionId'])
                 and row['sessionId'] not in sessions and _sha(row.get('sourceSha256'))
                 and row['sourceSha256'] not in sources, 'near_model_unique_source_session')
        key = _key(row.get('command')); keys.append(key)
        before = validate_observation(row.get('before')); after = validate_observation(row.get('after'))
        contract = row.get('observationContractSha256')
        _require(contract == row['before']['observationContractSha256'] == row['after']['observationContractSha256']
                 == reference['observationContractSha256'], 'near_model_observation_contract_changed')
        _require(row['before']['nearClosingVisibilityGate']['passed'] is True, 'near_model_before_visibility_gate')
        _require(minimum <= before[2] <= maximum and minimum <= after[2] <= maximum,
                 'near_model_source_gap_outside_experiment')
        evidence = row.get('sourceEvidence')
        _require(isinstance(evidence, dict) and all(_sha(evidence.get(k)) for k in
            ('beforeExternalImageSha256', 'afterExternalImageSha256', 'beforeGripperImageSha256', 'afterGripperImageSha256'))
            and evidence['beforeGripperImageSha256'] == row['before']['imageSha256']
            and evidence['afterGripperImageSha256'] == row['after']['imageSha256'], 'near_model_two_camera_source_bindings')
        contracts.add(contract); sessions.add(row['sessionId']); sources.add(row['sourceSha256'])
    entries, deficits = [], []
    scales = np.array([30., 30., 60.])
    for direction in ('close', 'open'):
        selected = [row for row, key in zip(rows, keys) if key == direction]
        if len(selected) < 2:
            deficits.append(dict(direction=direction, rows=len(selected), minimum=2)); continue
        before = np.array([row['before']['state'] for row in selected], dtype=float)
        after = np.array([row['after']['state'] for row in selected], dtype=float)
        mean = before.mean(axis=0); z = (before-mean)/scales
        delta = after-before; delta_mean = delta.mean(axis=0)
        coefficients = np.linalg.solve(z.T@z+np.eye(3), z.T@(delta-delta_mean))
        entries.append(dict(command=copy.deepcopy(selected[0]['command']), rows=len(selected), beforeStates=before.tolist(),
            meanState=mean.tolist(), deltaMean=delta_mean.tolist(), coefficients=coefficients.tolist(),
            trainingGapRmsePx=float(np.sqrt(np.mean((delta-(delta_mean+z@coefficients))[:, 2]**2))),
            sourceSha256s=[row['sourceSha256'] for row in selected]))
    return dict(schemaVersion=SCHEMA, supportPolicy=_policy(), observationContractSha256=next(iter(contracts)),
        referenceObservation=copy.deepcopy(reference), referenceImageSha256=reference['imageSha256'],
        referenceObjectWidthPx=float(width), originalBaselineGapPx=baseline,
        targetGapPx=float(width-4.), minimumGapPx=minimum, maximumOpeningGapPx=maximum,
        models=entries, deficits=deficits, trainingRows=copy.deepcopy(rows),
        controlEligible=False, developmentOnly=True, independentTest=False,
        calibratedUncertainty=False, uncertaintyInterval=None, contactVerified=False, graspVerified=False,
        originalModelsChanged=False, assumptions=['image_width_is_not_contact_force_or_depth',
            'local_response_transfer_unverified', 'external_alignment_and_retention_verification_required'])


def _validate(model):
    _require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA and model.get('supportPolicy') == _policy()
             and model.get('controlEligible') is False and model.get('developmentOnly') is True
             and model.get('calibratedUncertainty') is False, 'near_model_development_model_required')
    reference = model.get('referenceObservation'); state = validate_observation(reference)
    width = reference['objectVisibleWidthPx']
    _require(model.get('observationContractSha256') == reference['observationContractSha256']
             and model.get('referenceImageSha256') == reference['imageSha256']
             and model.get('referenceObjectWidthPx') == width
             and model.get('originalBaselineGapPx') == state[2]
             and model.get('targetGapPx') == width-4. and model.get('minimumGapPx') == width-12.
             and model.get('maximumOpeningGapPx') == state[2]+5., 'near_model_fixed_reference_changed')


def predict(model, current, command):
    _validate(model); state = validate_observation(current); key = _key(command)
    _require(current['observationContractSha256'] == model['observationContractSha256'], 'near_model_current_contract_changed')
    _require(current['nearClosingVisibilityGate']['passed'] is True, 'near_model_current_visibility_gate')
    _require(model['minimumGapPx'] <= state[2] <= model['maximumOpeningGapPx'], 'near_model_current_gap_outside_experiment')
    entries = [entry for entry in model['models'] if _key(entry['command']) == key]
    _require(len(entries) == 1 and entries[0]['rows'] >= 2, 'near_model_two_rows_per_command_required')
    entry = entries[0]; before = np.array(entry['beforeStates'], dtype=float)
    differences = np.abs(before-state)
    normalized = differences/np.array([30., 30., 60.]); distances = normalized.max(axis=1)
    nearest = int(np.argmin(distances))
    _require(distances[nearest] <= 1., 'near_model_current_outside_local_support')
    z = (state-np.array(entry['meanState']))/np.array([30., 30., 60.])
    baseline = state+np.array(entry['deltaMean'])
    predicted = baseline+z@np.array(entry['coefficients'])
    _require(predicted.shape == (3,) and np.isfinite(predicted).all()
             and model['minimumGapPx'] <= predicted[2] <= model['maximumOpeningGapPx'], 'near_model_predicted_gap_outside_experiment')
    return dict(predictedState=predicted.tolist(), commandMeanState=baseline.tolist(),
        gapErrorPx=abs(float(predicted[2])-model['targetGapPx']),
        commandMeanGapErrorPx=abs(float(baseline[2])-model['targetGapPx']),
        support=dict(nearestTrainingRow=nearest, absoluteStateDifferencePx=differences[nearest].tolist(),
            maximumScaledDifference=float(distances[nearest]), developmentOnly=True, safetyBound=False))


def select(model, current, candidates, *, commands_used):
    _validate(model); state = validate_observation(current)
    _require(type(commands_used) is int and 0 <= commands_used <= 6, 'near_model_command_budget')
    _require(isinstance(candidates, list) and bool(candidates), 'near_model_candidates_required')
    keys = [_key(command) for command in candidates]
    _require(len(keys) == len(set(keys)), 'near_model_duplicate_candidate')
    stop = dict(kind='stop', command=None, predictedState=state.tolist(), gapErrorPx=abs(float(state[2])-model['targetGapPx']))
    ranked, rejected = [stop], []
    reason = 'minimum_predicted_fixed_gap_error'
    if commands_used == 6: reason = 'command_budget_exhausted'
    elif current['nearClosingVisibilityGate']['passed'] is not True: reason = 'current_image_visibility_gate_failed'
    elif not model['minimumGapPx'] <= state[2] <= model['maximumOpeningGapPx']: reason = 'current_gap_outside_experiment'
    else:
        for command in candidates:
            try: ranked.append(dict(kind='action', command=copy.deepcopy(command), **predict(model, current, command)))
            except ValueError as exc: rejected.append(dict(command=copy.deepcopy(command), reason=str(exc)))
    ranked.sort(key=lambda row: row['gapErrorPx'])
    if ranked[0]['kind'] == 'stop' and reason == 'minimum_predicted_fixed_gap_error': reason = 'stop_has_minimum_fixed_gap_error'
    return dict(schemaVersion=SCHEMA+'.decision', modelSha256=digest(model), inputSha256=digest(current),
        selected=copy.deepcopy(ranked[0]), candidates=ranked, rejected=rejected, selectionReason=reason,
        targetGapPx=model['targetGapPx'], referenceObjectWidthPx=model['referenceObjectWidthPx'],
        minimumGapPx=model['minimumGapPx'], maximumOpeningGapPx=model['maximumOpeningGapPx'],
        commandsUsed=commands_used, maximumCommands=6, controlEligible=False, developmentOnly=True,
        calibratedUncertainty=False, contactVerified=False, graspVerified=False, liftVerified=False)


RELEASE_TABLE_SUPPORT_SCHEMA = 'motionbrain.m1_release_table_support_assessment.v1'


def make_release_goal(model, table_support_assessment):
    """실제 내려놓기 관측의 별도 근거에만 결합한 고정 열기 목표. 파일 감사는 actor 몫이다."""
    _validate(model)
    proof = table_support_assessment
    _require(isinstance(proof, dict) and proof.get('schemaVersion') == RELEASE_TABLE_SUPPORT_SCHEMA
             and proof.get('tableSupportVerified') is True
             and isinstance(proof.get('controllerBootId'), str)
             and re.fullmatch('[0-9a-f]{32}', proof['controllerBootId']) is not None
             and isinstance(proof.get('cameraBootId'), str)
             and re.fullmatch('[0-9a-f]{32}', proof['cameraBootId']) is not None
             and type(proof.get('outputSequence')) is int and proof['outputSequence'] > 0
             and type(proof.get('m4Deg')) in (int, float) and math.isfinite(proof['m4Deg'])
             and 0 <= proof['m4Deg'] < 360
             and all(type(proof.get(k)) in (int, float) and math.isfinite(proof[k]) for k in ('observedAt', 'assessedAt'))
             and 0 <= proof['observedAt'] <= proof['assessedAt'], 'release_actual_table_support_assessment_required')
    sources = proof.get('sources')
    _require(isinstance(sources, dict) and set(sources) == {'external', 'gripper', 'observation'},
             'release_actual_table_support_sources_required')
    for ref in sources.values():
        _require(isinstance(ref, dict) and set(ref) == {'path', 'sha256'}
                 and isinstance(ref['path'], str) and ref['path'].startswith('.local/')
                 and all(part not in ('', '.', '..') for part in ref['path'].split('/')[1:])
                 and '\\' not in ref['path'] and _sha(ref['sha256']), 'release_actual_source_binding_required')
    return dict(schemaVersion=SCHEMA+'.fixed_release_goal.v1', modelSha256=digest(model),
        targetOpeningGapPx=model['originalBaselineGapPx'], maximumCommands=1,
        tableSupportAssessment=copy.deepcopy(proof), tableSupportAssessmentSha256=digest(proof),
        actualTableSupportMustBeReauditedByActor=True,
        meaning='fixed_original_opening_goal_after_actual_table_support_not_close_target_or_release_success')


def select_release(model, current, fixed_goal, *, commands_used):
    """같은 학습 반응을 새 열기 목표에 비교한다. 기존 닫기 select의 목표는 바꾸지 않는다."""
    _validate(model); state = validate_observation(current)
    _require(isinstance(fixed_goal, dict) and fixed_goal == make_release_goal(model, fixed_goal.get('tableSupportAssessment')),
             'release_fixed_goal_or_actual_assessment_changed')
    _require(type(commands_used) is int and commands_used in (0, 1), 'release_one_command_budget')
    target = fixed_goal['targetOpeningGapPx']
    stop = dict(kind='stop', command=None, predictedState=state.tolist(), gapErrorPx=abs(float(state[2])-target))
    ranked, rejected = [stop], []
    if commands_used == 0:
        command = next(command for command in COMMANDS if command['direction'] == 'open')
        try:
            response = predict(model, current, command)
            # predict() also reports the old close-goal error. Replace those
            # metrics explicitly; the learned next state itself is unchanged.
            response['gapErrorPx'] = abs(response['predictedState'][2]-target)
            response['commandMeanGapErrorPx'] = abs(response['commandMeanState'][2]-target)
            ranked.append(dict(kind='action', command=copy.deepcopy(command), **response))
        except ValueError as exc:
            rejected.append(dict(command=copy.deepcopy(command), reason=str(exc)))
    ranked.sort(key=lambda candidate: candidate['gapErrorPx'])
    return dict(schemaVersion=SCHEMA+'.release_decision.v1', modelSha256=digest(model), inputSha256=digest(current),
        fixedGoalSha256=digest(fixed_goal), targetOpeningGapPx=target,
        selected=copy.deepcopy(ranked[0]), candidates=ranked, rejected=rejected,
        commandsUsed=commands_used, maximumCommands=1,
        selectionReason='release_one_command_budget_exhausted' if commands_used else
            'minimum_predicted_original_opening_error' if ranked[0]['kind'] == 'action' else 'stop_has_minimum_original_opening_error',
        actualTableSupportMustBeReauditedByActor=True, releaseVerified=False,
        contactVerified=False, graspVerified=False, controlEligible=False, developmentOnly=True)
