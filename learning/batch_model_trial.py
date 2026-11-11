"""배치 모델의 후보 비교와 관측 기반 단계 연결. 장치·실행 권한은 없다.

명령은 기존 실행기가 수행하고, 반환값은 기존 원본 감사기가 검증해야 한다.
이 모듈은 예측을 관측으로 바꾸거나 접촉·파지를 영상 좌표로 확정하지 않는다.
"""
from __future__ import annotations

import copy
import numpy as np

from learning.batch_learning import command_key, model_sha256, predict, validate_model, vector
from learning.batch_observation import digest, number, require


def select(model, observation, target, tolerance, commands):
    """같은 관측에서 예정 명령과 정지를 비교한다. 동률이면 정지를 택한다."""
    validate_model(model)
    require(observation['context'] == model['context'] and
            observation['stateFields'] == model['stateFields'], 'decision_observation_contract_changed')
    size = len(model['stateFields'])
    state = vector(observation['state'], size)
    goal, scale = vector(target, size), vector(tolerance, size)
    require(np.all(scale > 0), 'decision_positive_tolerance_required')
    require(isinstance(commands, list) and bool(commands), 'decision_candidates_required')
    keys = [command_key(c) for c in commands]
    require(len(set(keys)) == len(keys), 'decision_duplicate_candidate')
    score = lambda value: float(np.linalg.norm((np.asarray(value)-goal)/scale))
    ranked = [dict(kind='stop', command=None, predictedState=state.tolist(), error=score(state))]
    rejected = []
    for command in commands:
        try:
            prediction = predict(model, state.tolist(), command, observation['context'])
        except ValueError as exc:
            rejected.append(dict(command=copy.deepcopy(command), reason=str(exc)))
            continue
        ranked.append(dict(kind='action', command=copy.deepcopy(command), predictedState=prediction,
                           error=score(prediction)))
    ranked.sort(key=lambda item:item['error'])
    return dict(selected=copy.deepcopy(ranked[0]), candidates=ranked, rejected=rejected,
                modelSha256=model_sha256(model), observationSha256=digest(observation),
                controlEligible=False, executionAuthorized=False)


class BatchModelSequence:
    """고정 모델과 새 실측을 연결하는 단일 사용 의사결정 상태기계.

    propose()는 실행 요청만 반환한다. 실행은 기존 원장/출력 제한/승인을 갖춘
    실행기에서 수행한다. complete()에는 원본 감사 결과와 별도 새 관측을 넣는다.
    접촉·들림 판독은 기존 이중 카메라 검토 결과를 검증하는 review 콜백이 맡는다.
    콜백 없이는 close/lift 단계가 진행되지 않는다. 라이브 어댑터가 아니다.
    """
    def __init__(self, trial_id, stages, models, *, max_age_seconds=3, review=None):
        require(isinstance(trial_id,str) and bool(trial_id), 'decision_trial_id_required')
        require(isinstance(stages,list) and bool(stages), 'decision_stages_required')
        require(number(max_age_seconds) and 0 < max_age_seconds <= 10, 'decision_age_limit')
        require(review is None or callable(review), 'decision_review_callback')
        self.models, self.stages = copy.deepcopy(models), copy.deepcopy(stages)
        require(len({s['id'] for s in stages}) == len(stages), 'decision_unique_stage_ids')
        kinds = [s['kind'] for s in stages]
        require(all(k in ('position','close','lift') for k in kinds) and
                kinds == sorted(kinds,key=('position','close','lift').index) and
                kinds.count('close') <= 1 and kinds.count('lift') <= 1 and
                ('lift' not in kinds or 'close' in kinds), 'decision_stage_order')
        for stage in self.stages:
            require(set(stage) == {'id','kind','modelId','target','tolerance','commands','maxCommands'}, 'decision_stage_fields')
            model=self.models[stage['modelId']];validate_model(model)
            require(type(stage['maxCommands']) is int and 1 <= stage['maxCommands'] <= 4 and
                    (stage['kind']=='position' or stage['maxCommands']==1), 'decision_stage_budget')
            for command in stage['commands']:
                command_key(command)
                require(command['joint']==model['joint'], 'decision_stage_joint')
                is_close=command['joint']=='gripper' and command['direction']=='close'
                is_lift=command['joint']=='shoulder' and command['direction']=='up'
                require(is_close==(stage['kind']=='close') and is_lift==(stage['kind']=='lift'), 'decision_stage_command_kind')
            # Validate dimensions, positive tolerances and the command set without a prediction.
            vector(stage['target'],len(model['stateFields']))
            tol=vector(stage['tolerance'],len(model['stateFields']))
            require(np.all(tol>0) and bool(stage['commands']), 'decision_stage_target')
        self.trial_id,self.max_age,self.review=trial_id,max_age_seconds,review
        self.index=0;self.used=0;self.pending=None;self.halted=None;self.last_off=None
        self.frames=set();self.history=[];self.ordinal=0

    def _observation(self, observation, now, *, after=None):
        model=self.models[self.stages[self.index]['modelId']]
        require(isinstance(observation,dict) and observation.get('status')=='accepted', 'decision_observation_rejected')
        require(observation.get('context')==model['context'] and observation.get('stateFields')==model['stateFields'],
                'decision_observation_contract_changed')
        vector(observation.get('state'),len(model['stateFields']))
        ids=observation.get('frameIds')
        require(isinstance(ids,list) and bool(ids) and all(isinstance(v,str) and v for v in ids)
                and len(set(ids))==len(ids) and not self.frames.intersection(ids), 'decision_reused_frame')
        start,end=observation.get('observedAt'),observation.get('persistedAt')
        require(all(number(v) for v in (start,end,now)) and start<=end<=now and
                now-start<=self.max_age and (after is None or start>after), 'decision_stale_or_noncausal_observation')
        return ids

    def propose(self, observation, *, now):
        require(self.halted is None and self.pending is None and self.index<len(self.stages), 'decision_not_ready')
        ids=self._observation(observation,now,after=self.last_off)
        stage=self.stages[self.index];model=self.models[stage['modelId']]
        reached=all(abs(v-t)<=d for v,t,d in zip(observation['state'],stage['target'],stage['tolerance']))
        if stage['kind']=='position' and reached:
            self.frames.update(ids);self.index+=1;self.used=0
            return dict(status='observed_target_reached',stageId=stage['id'],complete=self.index==len(self.stages))
        if self.used>=stage['maxCommands']:
            self.halted='stage_budget_exhausted';return dict(status='halted',reason=self.halted)
        require(stage['kind']=='position' or self.review is not None, 'decision_interaction_review_required')
        decision=select(model,observation,stage['target'],stage['tolerance'],stage['commands'])
        if decision['selected']['kind']=='stop':
            self.halted='model_selected_stop';return dict(status='abstained',decision=decision)
        self.frames.update(ids);self.ordinal+=1;self.used+=1
        request=dict(requestId=digest([self.trial_id,self.ordinal]),stageId=stage['id'],
            command=decision['selected']['command'],decision=decision,before=copy.deepcopy(observation),preparedAt=now)
        self.pending=copy.deepcopy(request)
        return dict(status='execution_request',request=copy.deepcopy(request),executionAuthorized=False)

    def complete(self, request_id, audited, observation, *, now):
        require(self.pending is not None and self.halted is None, 'decision_no_pending_request')
        request=self.pending;stage=self.stages[self.index]
        # Any failure consumes the request and prevents automatic replacement/retry.
        self.pending=None;self.halted='execution_or_observation_unverified'
        require(request_id==request['requestId'], 'decision_request_changed')
        require(audited.get('status')=='verified' and audited.get('offVerified') is True and
                audited.get('command')==request['command'], 'decision_execution_not_verified')
        require(isinstance(audited.get('sourceSha256'),str) and len(audited['sourceSha256'])==64 and
                all(c in '0123456789abcdef' for c in audited['sourceSha256']), 'decision_source_hash_required')
        dispatch,off=audited.get('commandAt'),audited.get('offAt')
        require(number(dispatch) and number(off) and request['preparedAt']<dispatch<=off<=now,
                'decision_execution_time_changed')
        ids=self._observation(observation,now,after=off)
        self.frames.update(ids);self.last_off=off
        if stage['kind'] in ('close','lift'):
            require(self.review(stage,copy.deepcopy(request),copy.deepcopy(audited),copy.deepcopy(observation)) is True,
                    'decision_contact_or_retention_unverified')
            self.index+=1;self.used=0
        self.history.append(dict(request=request,audit=copy.deepcopy(audited),after=copy.deepcopy(observation)))
        self.halted=None
        return dict(status='observed_after',complete=self.index==len(self.stages),
                    physicalSuccessClaimed=False,executionAuthorized=False)
