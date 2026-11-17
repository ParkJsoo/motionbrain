"""외부 탁상 영상으로 복구 목적을 판정하고 기존 OPEN 반응을 비교한다.

장치 접근·fit·누락 근접 특징 보간 없음. 원본·신선도·실패 명령의 확인된
OFF·현재 출력 epoch는 호출자가 검증한다. 영상 조건은 실제 파지 증명이 아니다.
"""
import copy

from learning import limited_scene_cycle_assessment as image
from learning import m1_near_action_model as response
from learning import m1_local_open_response as local_response
from learning.near_gripper_tip_observer import validate_observation

SCHEMA = 'motionbrain.external_table_gripper_recovery.v1'


def assess_table(baseline, current):
    """같은 등록에서 물체가 저장된 탁상 영상 위치에 남았는지 검사한다."""
    def evaluate():
        base = image._external(baseline); now = image._external(current)
        image._require(baseline['registrationSha256'] == current['registrationSha256'],
                       'recovery_external_contract_changed')
        image._require(baseline['imageSha256'] != current['imageSha256'],
                       'recovery_current_image_reused')
        image._require(base['upright'], 'recovery_baseline_not_upright')
        image._table(base, now)
        return dict(recoverySchemaVersion=SCHEMA, purpose='restore_opening_then_reobserve',
                    purposeOwner='external_table_image_rule', baseline=copy.deepcopy(baseline),
                    current=copy.deepcopy(current), actualEmptyGripperVerified=False,
                    apparentTablePositionVerified=True)
    return image._result('recovery_table_object', evaluate)


def select_reopening(model, near, assessment, *, commands_used, local_open_model=None):
    """정상 근접 입력에서만 고정 OPEN 모델과 STOP의 초기 벌어짐 오차를 비교한다."""
    response._validate(model)
    image._require(type(commands_used) is int and commands_used in (0, 1),
                   'recovery_single_OPEN_budget')
    target = model['originalBaselineGapPx']
    stop = dict(kind='stop', command=None, openingErrorPx=None)
    decision = dict(schemaVersion=SCHEMA+'.decision', selected=stop, candidates=[stop], rejected=[],
                    purpose='restore_opening_then_reobserve', purposeOwner='external_table_image_rule',
                    selectionOwner='admission_guard', modelSelectedAction=False, modelSelectedStop=False,
                    modelPredictionPerformed=False,
                    modelSha256=response.digest(model), targetOpeningGapPx=target,
                    commandsUsed=commands_used, maximumCommands=1, modelsRefitted=False,
                    missingNearStateImputed=False, controlEligible=False,
                    physicalGraspVerified=False, recoveryCompleted=False)
    try:
        image._require(isinstance(assessment, dict) and assessment.get('accepted') is True,
                       'recovery_external_table_not_admitted')
        evidence = assessment['evidence']
        image._require(assessment == assess_table(evidence['baseline'], evidence['current']),
                       'recovery_external_assessment_changed')
        decision['externalAssessmentSha256'] = response.digest(assessment)
        if local_open_model is not None:
            local_response.validate_source(local_open_model, model)
            decision.update(localOpenModelSha256=response.digest(local_open_model),
                            localOpenSourceBindingSha256=local_open_model['sourceBindingSha256'],
                            responseFamily=local_response.SCHEMA, localOpenOptIn=True,
                            localTransferMeaning=local_open_model['supportPolicy']['transferMeaning'])
        state = validate_observation(near)
        stop.update(predictedState=state.tolist(), openingErrorPx=abs(float(state[2])-target))
        if commands_used:
            decision.update(selectionOwner='budget_guard', reason='recovery_single_OPEN_budget_exhausted')
            return copy.deepcopy(decision)
        command = next(c for c in response.COMMANDS if c['direction'] == 'open')
        prediction = (response.predict(model, near, command) if local_open_model is None else
                      local_response.predict(local_open_model, near))
        decision['modelPredictionPerformed'] = True
        # Keep the original close-goal metrics inside response; rank with a
        # separately named opening metric, without changing any learned weights.
        action = dict(kind='action', command=copy.deepcopy(command), response=prediction,
                      openingErrorPx=abs(prediction['predictedState'][2]-target))
        decision['candidates'].append(action)
        decision['candidates'].sort(key=lambda c: c['openingErrorPx'])
        decision['selected'] = decision['candidates'][0]
        decision.update(selectionOwner=('learned_OPEN_response_comparison' if local_open_model is None else
                                        'local_observed_delta_OPEN_response_comparison'),
                        modelSelectedAction=decision['selected']['kind'] == 'action',
                        modelSelectedStop=decision['selected']['kind'] == 'stop',
                        reason='minimum_predicted_original_opening_error')
    except (ValueError, KeyError, TypeError, IndexError, AttributeError) as error:
        reason = str(error) if isinstance(error, ValueError) else 'recovery_input_structure_invalid'
        decision.update(reason='recovery_model_input_not_supported')
        decision['rejected'].append(dict(command=copy.deepcopy(next(c for c in response.COMMANDS
                                                                  if c['direction'] == 'open')), reason=reason))
    return copy.deepcopy(decision)
