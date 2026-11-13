"""고정 모델의 원학습 행으로 수동 시작 배치 후보를 사전 거절한다.

장치·파일·학습·명령 실행 경로는 없다. 같은 명령의 관측 delta 최소·최대는
과거 자료의 경험적 범위이며, 미래 출력의 보장 범위나 신뢰구간이 아니다.
검사를 통과해도 별도 현장 조건·실행 승인을 대신하지 않는다.

모델의 식별자·평균 통계만으로는 원학습 행 내용을 증명할 수 없다. 호출자는
고정 모델 및 원학습 manifest의 원본 파일 SHA를 먼저 검증하고, 검토·승인 계약에
별도로 고정한 canonical 행 digest를 expected_training_rows_sha256로 전달해야 한다.
검사할 행에서 호출 직전에 새 digest를 만들어 기대값으로 삼아서는 안 된다.
"""
from __future__ import annotations

import copy
import math

import numpy as np

from learning.manual_collection_dataset import require
from learning.relative_action_learning import command_key, select, validate_rows, vector
from learning.relative_model_bundle import validate_model
from learning.supervised_collection import plan_digest


def _training_evidence(model, rows):
    """모델을 다시 학습하지 않고 원학습 행의 식별·문맥·저장 통계를 대조한다."""
    context = validate_rows(rows, allow_development_observations=model.get('developmentObservationOnly') is True)
    require(context == model['context'], 'positioning_training_context_mismatch')
    require([r['sessionId'] for r in rows] == model['trainingSessionIds'] and
            sorted({r['captureGroup'] for r in rows}) == model['trainingCaptureGroups'] and
            [r['episode'] for r in rows] == model['trainingEpisodes'] and
            [f for r in rows for f in r['frameIds']] == model['trainingFrameIds'],
            'positioning_training_provenance_mismatch')
    groups = {}
    for row in rows:
        groups.setdefault(command_key(row['command']), []).append(row)
    require({command_key(entry['command']) for entry in model['models']} ==
            {key for key, group in groups.items() if len(group) >= 2},
            'positioning_training_commands_mismatch')
    for entry in model['models']:
        group = groups[command_key(entry['command'])]
        before = np.asarray([r['before'] for r in group], dtype=float)
        delta = np.asarray([np.asarray(r['after'])-r['before'] for r in group])
        statistics = dict(minimum=before.min(axis=0), maximum=before.max(axis=0),
            mean=before.mean(axis=0), scale=np.maximum(before.std(axis=0), 1.), deltaMean=delta.mean(axis=0))
        require(entry['count'] == len(group) and all(
            np.allclose(entry[name], value, rtol=0., atol=1e-9) for name, value in statistics.items()),
            'positioning_training_statistics_mismatch')
    return groups


def assess_manual_positioning(model, training_rows, state, context, command, fence, *,
                              expected_training_rows_sha256,
                              target=None, tolerance=None, minimum_improvement_px=.25):
    """수동 배치의 경험적 도착 범위와 후속 모델 접근 가능성을 순수하게 평가한다.

    잘못된 문맥·원학습 근거는 ValueError로 거절한다. 유효한 근거의 범위 위반은
    status='rejected'/reasons로 반환하며, 위반이 없어도 status='review_required'다.
    target/tolerance를 함께 주면 관측 delta별 도착점이 목표 밖인지, 그 지점에서
    고정 모델의 지원·개선 후보가 있는지도 보수적으로 대조한다.
    expected_training_rows_sha256는 외부의 검토된 계약에서 받아야 한다. 이 함수는
    행 내용의 일치를 확인하지만 그 외부 계약의 승인 자체를 생성하지 않는다.
    """
    require(isinstance(expected_training_rows_sha256, str) and
            len(expected_training_rows_sha256) == 64 and
            all(c in '0123456789abcdef' for c in expected_training_rows_sha256),
            'positioning_expected_training_rows_digest_required')
    require(plan_digest(training_rows) == expected_training_rows_sha256,
            'positioning_training_rows_content_digest_mismatch')
    fixed = validate_model(model, allow_development_observations=True)
    require(context == fixed['context'], 'positioning_context_mismatch')
    groups = _training_evidence(fixed, training_rows)
    x = vector(state)
    require(isinstance(fence, dict) and set(fence) == {'minimum', 'maximum'}, 'positioning_fence_fields')
    lo, hi = vector(fence['minimum']), vector(fence['maximum'])
    require(np.all(lo < hi), 'positioning_finite_fence_required')
    key = command_key(command)
    require(type(minimum_improvement_px) in (int, float) and math.isfinite(minimum_improvement_px)
            and minimum_improvement_px >= 0, 'positioning_finite_improvement_required')
    require((target is None) == (tolerance is None), 'positioning_goal_and_tolerance_pair_required')
    if target is not None:
        goal, tol = vector(target), vector(tolerance)
        require(np.all(tol > 0), 'positioning_positive_goal_tolerance_required')
    inside = lambda value: bool(np.all(value >= lo) and np.all(value <= hi))
    result = dict(status='rejected', reasons=[], currentState=x.tolist(), command=copy.deepcopy(command),
        trainingRowsSha256=expected_training_rows_sha256,
        currentInsideFence=inside(x), currentCommandInSupport=False, observedDeltaMinimum=None,
        observedDeltaMaximum=None, observedEndpointEnvelope=None, observedSamples=0,
        empiricalEnvelopeNotConfidenceBound=True, futureOutcomeGuaranteed=False,
        modelSelected=False, controlEligible=False, executionAuthorized=False, trainingPerformed=False)
    if not inside(x):
        result['reasons'].append('current_state_outside_fence')
    entries = [entry for entry in fixed['models'] if command_key(entry['command']) == key]
    if len(entries) != 1 or len(groups.get(key, [])) < 2:
        result['reasons'].append('command_without_two_original_training_transitions')
        return result
    group, entry = groups[key], entries[0]
    delta = np.asarray([np.asarray(row['after'])-row['before'] for row in group])
    minimum, maximum = delta.min(axis=0), delta.max(axis=0)
    lower, upper = x+minimum, x+maximum
    # 축별 관측 최소·최대의 상자는 개별 표본보다 넓을 수 있다. 축소하지 않는다.
    result.update(observedSamples=len(group), observedDeltaMinimum=minimum.tolist(),
        observedDeltaMaximum=maximum.tolist(),
        observedEndpointEnvelope=dict(minimum=lower.tolist(), maximum=upper.tolist()),
        currentCommandInSupport=bool(np.all(x >= entry['minimum']) and np.all(x <= entry['maximum'])))
    if not inside(lower) or not inside(upper):
        result['reasons'].append('observed_endpoint_envelope_exceeds_fence')
    if not result['currentCommandInSupport']:
        result['reasons'].append('current_state_outside_command_training_support')
    if target is not None:
        diagnostics = []
        commands = [item['command'] for item in fixed['models']]
        for change in delta:
            endpoint = x+change
            reached = bool(np.all(np.abs(endpoint-goal) <= tol))
            actionable = []
            if inside(endpoint):
                decision = select(fixed, endpoint.tolist(), goal.tolist(), commands, context,
                                  minimum_improvement_px=minimum_improvement_px)
                hold = next(item for item in decision['candidates'] if item['kind'] == 'stop')
                actionable = [copy.deepcopy(item) for item in decision['candidates']
                    if item['kind'] == 'action' and inside(np.asarray(item['predictedState'])) and
                    hold['errorPx']-item['errorPx'] > minimum_improvement_px]
            diagnostics.append(dict(endpointState=endpoint.tolist(), targetAlreadyReached=reached,
                                    improvingSupportedCandidates=actionable))
        result['followupDiagnostics'] = diagnostics
        if any(item['targetAlreadyReached'] for item in diagnostics):
            result['reasons'].append('observed_endpoint_may_already_satisfy_target')
        if any(not item['improvingSupportedCandidates'] for item in diagnostics):
            result['reasons'].append('observed_endpoint_lacks_supported_improving_followup')
    if not result['reasons']:
        result['status'] = 'review_required'
    return result
