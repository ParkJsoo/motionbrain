"""감사된 OPEN 원본의 최근접 관측 반응을 옮기는 별도 국소 개발 모델.

기존 ridge·실험 상한을 바꾸거나 출력을 잘라 맞추지 않는다. 가까운 원본 하나의
관측 Δ는 단일 원본 전이 가설이며 반복성·안전·실제 해제를 보증하지 않는다.
기존 최근접 반경을 유지하고 주변 원본 수를 추가 실행 조건으로 강제하지 않는다.
"""
from __future__ import annotations
import copy
import math

import numpy as np

from learning import m1_near_action_model as frozen
from learning.near_gripper_tip_observer import digest, validate_observation

SCHEMA = 'motionbrain.m1_local_observed_OPEN_response.v1'
COMMAND = dict(joint='gripper', direction='open', percent=70, durationMs=1000)


def _require(ok, reason):
    if not ok: raise ValueError(reason)


def _policy():
    return dict(estimator='one_nearest_observed_delta', normalizedStateScalesPx=[30., 30., 60.],
        distance='maximum_absolute_scaled_difference', maximumDistance=1., minimumTrainingSources=2,
        maximumCommands=1,
        transferMeaning='single_nearest_source_hypothesis_not_independent_validation',
        nearestTiePolicy='source_order', outputClipped=False, originalExperimentBoundsChanged=False,
        calibratedUncertainty=False, safetyBound=False)


def _rows(source):
    return [copy.deepcopy(row) for row in source['trainingRows'] if row['command'] == COMMAND]


def source_binding(source):
    frozen._validate(source)
    rows = _rows(source)
    return dict(sourceFrozenModelSha256=digest(source), sourceReferenceSha256=digest(source['referenceObservation']),
        observationContractSha256=source['observationContractSha256'], sourceRowsSha256=digest(rows),
        sourceSessionIds=[row['sessionId'] for row in rows], sourceSha256s=[row['sourceSha256'] for row in rows])


def _validate_rows(rows, reference, minimum, maximum):
    contract = reference['observationContractSha256']; sessions, sources = set(), set()
    _require(isinstance(rows, list) and len(rows) >= 2, 'local_OPEN_two_audited_sources_required')
    for row in rows:
        _require(isinstance(row, dict) and row.get('command') == COMMAND and
                 isinstance(row.get('sessionId'), str) and row['sessionId'] and row['sessionId'] not in sessions and
                 frozen._sha(row.get('sourceSha256')) and row['sourceSha256'] not in sources,
                 'local_OPEN_unique_original_sources_required')
        before = validate_observation(row.get('before')); after = validate_observation(row.get('after'))
        _require(row.get('observationContractSha256') == row['before']['observationContractSha256'] ==
                 row['after']['observationContractSha256'] == contract, 'local_OPEN_observation_contract_changed')
        _require(row['before']['nearClosingVisibilityGate']['passed'] is True and
                 minimum <= before[2] <= maximum and minimum <= after[2] <= maximum,
                 'local_OPEN_source_outside_original_experiment')
        evidence = row.get('sourceEvidence', {})
        _require(all(frozen._sha(evidence.get(k)) for k in ('beforeExternalImageSha256', 'afterExternalImageSha256',
                 'beforeGripperImageSha256', 'afterGripperImageSha256')) and
                 evidence['beforeGripperImageSha256'] == row['before']['imageSha256'] and
                 evidence['afterGripperImageSha256'] == row['after']['imageSha256'],
                 'local_OPEN_two_camera_original_bindings_required')
        sessions.add(row['sessionId']); sources.add(row['sourceSha256'])


def fit(source_model, *, audit_sources):
    """기존 고정 모델의 OPEN 원본을 별도로 재감사해 메모리 기반 반응 모델로 고정한다."""
    frozen._validate(source_model); rows = _rows(source_model)
    _require(callable(audit_sources) and audit_sources(copy.deepcopy(rows)) == rows, 'local_OPEN_original_reaudit_changed')
    reference = copy.deepcopy(source_model['referenceObservation'])
    _validate_rows(rows, reference, source_model['minimumGapPx'], source_model['maximumOpeningGapPx'])
    binding = source_binding(source_model)
    return dict(schemaVersion=SCHEMA, supportPolicy=_policy(), sourceBinding=binding,
        sourceBindingSha256=digest(binding), referenceObservation=reference,
        observationContractSha256=source_model['observationContractSha256'],
        minimumGapPx=source_model['minimumGapPx'], maximumOpeningGapPx=source_model['maximumOpeningGapPx'],
        targetOpeningGapPx=source_model['originalBaselineGapPx'], trainingRows=copy.deepcopy(rows),
        command=copy.deepcopy(COMMAND), weightsFromOriginalRidgeReused=False, originalModelChanged=False,
        controlEligible=False, developmentOnly=True, independentTest=False, prospectivelyTested=False,
        calibratedUncertainty=False, uncertaintyInterval=None, physicalReleaseVerified=False)


def _validate(model):
    _require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA and model.get('supportPolicy') == _policy()
             and model.get('command') == COMMAND and model.get('controlEligible') is False
             and model.get('developmentOnly') is True and model.get('independentTest') is False
             and model.get('prospectivelyTested') is False and model.get('calibratedUncertainty') is False,
             'local_OPEN_development_model_required')
    reference = model.get('referenceObservation'); state = validate_observation(reference)
    _require(model.get('observationContractSha256') == reference['observationContractSha256'] and
             model.get('minimumGapPx') == reference['objectVisibleWidthPx']-12. and
             model.get('maximumOpeningGapPx') == state[2]+5. and model.get('targetOpeningGapPx') == state[2],
             'local_OPEN_original_reference_bounds_changed')
    rows = model.get('trainingRows'); _validate_rows(rows, reference, model['minimumGapPx'], model['maximumOpeningGapPx'])
    binding = model.get('sourceBinding')
    _require(isinstance(binding, dict) and digest(binding) == model.get('sourceBindingSha256') and
             frozen._sha(binding.get('sourceFrozenModelSha256')) and
             binding.get('sourceReferenceSha256') == digest(reference) and
             binding.get('observationContractSha256') == model['observationContractSha256'] and
             binding.get('sourceRowsSha256') == digest(rows) and
             binding.get('sourceSessionIds') == [r['sessionId'] for r in rows] and
             binding.get('sourceSha256s') == [r['sourceSha256'] for r in rows], 'local_OPEN_source_binding_changed')


def validate_source(model, source_model):
    _validate(model)
    _require(model['sourceBinding'] == source_binding(source_model) and
             model['referenceObservation'] == source_model['referenceObservation'], 'local_OPEN_frozen_source_changed')
    return copy.deepcopy(model['sourceBinding'])


def predict(model, current):
    """원본 이후값은 고정 학습 라벨이다. 현재 입력에 현재 동작 이후 정보는 쓰지 않는다."""
    _validate(model); state = validate_observation(current)
    _require(current['observationContractSha256'] == model['observationContractSha256'], 'local_OPEN_current_contract_changed')
    _require(current['nearClosingVisibilityGate']['passed'] is True, 'local_OPEN_current_visibility_gate')
    _require(model['minimumGapPx'] <= state[2] <= model['maximumOpeningGapPx'], 'local_OPEN_current_gap_outside_experiment')
    rows = model['trainingRows']; scales = np.array(model['supportPolicy']['normalizedStateScalesPx'])
    before = np.array([row['before']['state'] for row in rows]); after = np.array([row['after']['state'] for row in rows])
    distances = np.max(np.abs(before-state)/scales, axis=1); nearest = int(np.argmin(distances))
    _require(distances[nearest] <= 1., 'local_OPEN_current_outside_local_support')
    delta = after-before; predicted = state+delta[nearest]; mean = state+delta.mean(axis=0)
    _require(np.isfinite(predicted).all() and model['minimumGapPx'] <= predicted[2] <= model['maximumOpeningGapPx'],
             'local_OPEN_predicted_gap_outside_original_experiment')
    nearby = [i for i, value in enumerate(distances) if value <= 1.]
    return dict(predictedState=predicted.tolist(), commandMeanState=mean.tolist(),
        observedSourceDeltaState=delta[nearest].tolist(), targetOpeningGapPx=model['targetOpeningGapPx'],
        openingErrorPx=abs(float(predicted[2])-model['targetOpeningGapPx']),
        commandMeanOpeningErrorPx=abs(float(mean[2])-model['targetOpeningGapPx']),
        support=dict(nearestTrainingRow=nearest, nearestSourceSessionId=rows[nearest]['sessionId'],
            nearestSourceSha256=rows[nearest]['sourceSha256'], maximumScaledDifference=float(distances[nearest]),
            sourceScaledDistances=distances.tolist(), nearbySourceCount=len(nearby), nearbyTrainingRows=nearby,
            transferMeaning=model['supportPolicy']['transferMeaning'],
            independentValidation=False, safetyBound=False),
        candidateEligible=True, candidateRejection=None,
        modelSha256=digest(model), inputSha256=digest(current), controlEligible=False,
        calibratedUncertainty=False, uncertaintyInterval=None, prospectivelyTested=False)
