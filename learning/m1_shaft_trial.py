"""주입된 포트로만 실행하는 유한 M1 scalar CLOSE 시험. 정리 작업은 호출자 소유다.

고정 shaft 모델·현재 원본·후보 결과를 재관측하며 최대 네 번 비교한다.
shaft 목표 도달은 접촉·힘·실제 턱끝·파지·들기 판정이 아니다. 실패 후 재시도,
OPEN 대체, 재학습, 장치 발견이나 포트 종료 기능은 포함하지 않는다.
observed95-local450-v1은 새 직접 관측 좌표의 별도 실험 범위다. 기존 row55
수치 한계의 변환이나 기계 안전 한계의 증명이 아니다.
"""
from __future__ import annotations

import copy
import math
import time

from learning import m1_shaft_learning as learning
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest

OBSERVED95_POLICY = 'observed95-local450-v1'
OBSERVED95_GOAL = 'manual_visible_jaw_configuration_reference_not_contact_or_grasp'

def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _hash(value):
    return isinstance(value, str) and len(value) == 64 and all(c in '0123456789abcdef' for c in value)


def validate_scope(scope):
    fields = {
        'targetShaftPx', 'tolerancePx', 'maxCommands', 'totalRemaining', 'maxSeconds',
        'minimumImprovementPx', 'maximumPredictionResidualPx', 'minimumObservedProgressPx',
        'shaftMinimum', 'shaftMaximum', 'maxObservationAgeSeconds', 'maxDecisionAgeSeconds'}
    observed95 = isinstance(scope, dict) and 'geometryPolicy' in scope
    extra = {'geometryPolicy', 'startIntervalPx', 'observationRegistrationSha256', 'goalSemantics'} if observed95 else set()
    require(isinstance(scope, dict) and set(scope) == fields | extra,
        'M1_scalar_trial_scope_fields')
    require(all(_finite(scope[key]) for key in fields), 'M1_scalar_trial_finite_scope_required')
    require(type(scope['maxCommands']) is int and 1 <= scope['maxCommands'] <= 4 and
            type(scope['totalRemaining']) is int and 0 <= scope['totalRemaining'] <= 4 and
            0 < scope['maxSeconds'] <= 45, 'M1_scalar_trial_finite_budget_required')
    fixed = dict(tolerancePx=1.5, minimumImprovementPx=.25, maximumPredictionResidualPx=3.,
        minimumObservedProgressPx=.2, shaftMinimum=118., shaftMaximum=155.5,
        maxObservationAgeSeconds=3., maxDecisionAgeSeconds=.25)
    if observed95:
        # 새 직접 row95 실험 계약이다. 구 row55 기구 한계의 변환이나 보장이 아니다.
        fixed.update(shaftMinimum=84., shaftMaximum=139.5, targetShaftPx=85.3)
        interval = scope['startIntervalPx']
        require(scope['geometryPolicy'] == OBSERVED95_POLICY and
                scope['goalSemantics'] == OBSERVED95_GOAL and
                _hash(scope['observationRegistrationSha256']) and
                isinstance(interval, list) and len(interval) == 2 and
                all(_finite(value) for value in interval) and
                100. <= interval[0] < interval[1] <= 137.,
                'M1_scalar_trial_observed95_contract_required')
    require(all(scope[key] == value for key, value in fixed.items()) and
            scope['shaftMinimum'] <= scope['targetShaftPx'] <= scope['shaftMaximum'],
            'M1_scalar_trial_fixed_policy_changed')
    return copy.deepcopy(scope)


def _dispatch_guard(check):
    state = dict(open=True, calls=0, at=None)
    def guard():
        require(state['open'] and state['calls']==0, 'M1_scalar_trial_dispatch_guard_closed_or_reused')
        state['calls'] += 1
        state['at'] = check()
        return state['at']
    return guard, state


class M1ShaftTrial:
    """observe(after_off, deadline, tick)와 execute(command, before_run, decision)를 주입한다.

    observe는 원본 검증·OFF+3초 대기·현장 검토를 마친 bound scalar 결과를 반환한다.
    execute는 동일 명령의 RUN 직전에 before_run을 정확히 한 번 호출하고, 실제
    출력과 OFF가 검증된 SequencePort receipt를 반환해야 한다. 실패하면 자동으로
    다시 부르지 않는다. verify_assets는 고정 원본·외부 실행 예산을 검사하며 RUN
    callback 안에서도 호출하므로 카메라·HTTP 요청이나 대기를 포함하면 안 된다.
    포트·카메라·터널 정리는 execute 소유자의 finally에서 수행한다.
    """
    def __init__(self, model, scope, *, observe, execute, tick, verify_assets, store, clock=time):
        self.model = learning.validate_model(model)
        require(self.model['context']['stateFields'] == ['shaftSeparationPx'] and
                self.model['context']['landmarks'].startswith(learning.SHAFT_PREFIX),
                'M1_native_scalar_model_required')
        self.scope = validate_scope(scope)
        if self.observed95:
            require(self.model['context']['landmarks'] == learning.SHAFT_PREFIX+
                    self.scope['observationRegistrationSha256'], 'M1_scalar_trial_registration_changed')
        self.commands = [copy.deepcopy(entry['command']) for entry in self.model['models']
            if entry['command']['direction']=='close' and entry['command']['percent']<=60
            and entry['command']['durationMs']<=650]
        if self.observed95:
            self.commands = [command for command in self.commands
                             if command['percent']==60 and command['durationMs']==450]
        require(bool(self.commands), 'M1_scalar_trial_supported_close_candidates_required')
        require(all(callable(value) for value in (observe, execute, tick, verify_assets)) and
                callable(getattr(store, 'write_json', None)), 'M1_scalar_trial_injected_ports_required')
        self.observe, self.execute, self.tick, self.verify_assets = observe, execute, tick, verify_assets
        self.store, self.clock = store, clock
        self.model_sha256 = learning.digest(self.model)
        self.scope_sha256 = plan_digest(self.scope)
        self._commands_sha256 = plan_digest(self.commands)
        self.used = False
        self._frames = set()
        self._host_clock = None

    @property
    def observed95(self):
        return self.scope.get('geometryPolicy') == OBSERVED95_POLICY

    def _check(self):
        self.verify_assets()
        require(learning.digest(self.model) == self.model_sha256 and
                plan_digest(self.scope) == self.scope_sha256 and
                plan_digest(self.commands) == self._commands_sha256, 'M1_scalar_trial_frozen_contract_changed')
        require(self.clock.monotonic() < self.deadline, 'M1_scalar_trial_deadline')

    def _bound(self, record, *, after_off, fresh):
        require(isinstance(record, dict) and record.get('context') == self.model['context'],
                'M1_scalar_trial_observation_context_changed')
        value = record.get('shaftSeparationPx')
        require(_finite(value) and value >= 0, 'M1_scalar_trial_observation_rejected')
        require(record.get('sourceFreshMetadataVerified') is True, 'M1_scalar_trial_bound_source_required')
        bracket, persisted = record.get('sourceHostBracket'), record.get('persistedAt')
        now = self.clock.monotonic()
        require(isinstance(bracket, list) and len(bracket) == 2 and
                all(_finite(x) for x in (*bracket, persisted)) and
                self.started <= bracket[0] <= bracket[1] <= persisted <= now and
                now-bracket[0] <= self.scope['maxObservationAgeSeconds'] and
                (after_off is None or bracket[0] >= after_off+3.), 'M1_scalar_trial_stale_or_noncausal_source')
        frame = record.get('sourceFrame')
        require(isinstance(frame, dict) and isinstance(frame.get('frameId'), str) and frame['frameId'] and
                _hash(frame.get('pixelSha256')) and
                [frame.get('height'), frame.get('width')] == self.model['context']['imageShape'],
                'M1_scalar_trial_frame_identity_required')
        if 'hostClockId' in record:
            require(isinstance(record['hostClockId'], str) and record['hostClockId'] and
                    (self._host_clock is None or record['hostClockId'] == self._host_clock),
                    'M1_scalar_trial_host_clock_changed')
            self._host_clock = record['hostClockId']
        if fresh:
            require(frame['frameId'] not in self._frames, 'M1_scalar_trial_frame_reused')
            self._frames.add(frame['frameId'])
        require(self.scope['shaftMinimum'] <= value <= self.scope['shaftMaximum'],
                'M1_scalar_trial_observed_shaft_outside_fence')
        return float(value)

    def _choose(self, value):
        # 명시적인 단일명령 시험은 사후 관측을 다음 명령 입력으로 사용하지 않는다.
        # 최초 입력 지원·예측 범위·사후 잔차와 실제 범위 검사는 그대로 적용한다.
        decision = learning.select(self.model, value, self.scope['targetShaftPx'], self.commands,
            self.model['context'], minimum_improvement_px=self.scope['minimumImprovementPx'])
        safe = []
        for item in decision['candidates']:
            predicted = item['predictedShaftSeparationPx']
            if item['kind'] == 'action' and not self.scope['shaftMinimum'] <= predicted <= self.scope['shaftMaximum']:
                decision['rejected'].append(dict(command=copy.deepcopy(item['command']),
                                                reason='M1_predicted_shaft_outside_fence'))
            elif self.observed95 and self.scope['maxCommands'] > 1 and item['kind'] == 'action' and abs(predicted-self.scope['targetShaftPx']) > self.scope['tolerancePx'] and not any(
                    entry['command'] == item['command'] and entry['minimum'] <= predicted <= entry['maximum']
                    for entry in self.model['models']):
                decision['rejected'].append(dict(command=copy.deepcopy(item['command']),
                                                reason='M1_predicted_nonterminal_shaft_outside_support'))
            else:
                safe.append(item)
        stop = next(item for item in safe if item['kind'] == 'stop')
        selected = safe[0]
        gain = stop['errorPx']-selected['errorPx']
        reason = 'minimum_predicted_shaft_error'
        if selected['kind'] == 'action' and gain <= self.scope['minimumImprovementPx']:
            selected, reason = stop, 'predicted_improvement_below_margin'
        elif selected['kind'] == 'stop':
            reason = 'stop_has_minimum_predicted_shaft_error'
        decision.update(candidates=safe, selected=copy.deepcopy(selected),
            status='ranked' if len(safe)>1 else 'abstained', selectionReason=reason,
            bestPredictedImprovementPx=gain)
        return decision

    def run(self):
        require(not self.used, 'M1_scalar_trial_single_use')
        self.used = True
        self.started = self.clock.monotonic()
        self.deadline = self.started+self.scope['maxSeconds']
        budget = min(self.scope['maxCommands'], self.scope['totalRemaining'])
        report = dict(status='failed', modelSha256=self.model_sha256, scopeSha256=self.scope_sha256,
            commandAttempts=0, confirmedCommands=0, modelSelectedCommands=0, steps=[], observations=[],
            shaftGoalReachedOnly=False, initialGoalAlreadySatisfied=False, goalBoundaryReached=False,
            graspVerified=False, contactVerified=False, liftVerified=False, automaticRetry=False,
            trainingPerformed=False, controlEligible=False, errors=[])
        off_at, pending = None, None
        try:
            self._check()
            self.store.write_json('trial-intent.json', dict(scope=self.scope, modelSha256=self.model_sha256,
                                                         scopeSha256=self.scope_sha256))
            while True:
                self._check(); self.tick(); self._check()
                record = self.observe(after_off=off_at, deadline=self.deadline, tick=self.tick)
                self._check()
                report['observations'].append(copy.deepcopy(record))
                value = self._bound(record, after_off=off_at, fresh=True)
                error = abs(value-self.scope['targetShaftPx'])
                if self.observed95 and len(report['observations']) == 1:
                    low, high = self.scope['startIntervalPx']
                    require(low <= value <= high and error > self.scope['tolerancePx'],
                            'M1_scalar_trial_independent_start_required')
                reached = error <= self.scope['tolerancePx']
                report['shaftGoalReachedOnly'] = reached
                report['goalBoundaryReached'] = reached and math.isclose(error,self.scope['tolerancePx'],rel_tol=0.,abs_tol=1e-9)
                if pending is not None:
                    reduction = pending['beforeGoalErrorPx']-error
                    residual = value-pending['predictedShaftSeparationPx']
                    pending.update(status='observed_after', after=copy.deepcopy(record),
                        actualShaftSeparationPx=value, predictionResidualPx=residual,
                        absolutePredictionResidualPx=abs(residual), afterGoalErrorPx=error,
                        observedGoalErrorReductionPx=reduction,
                        commandMeanPredictionErrorPx=abs(value-pending['commandMeanPredictedShaftSeparationPx']),
                        noChangePredictionErrorPx=abs(value-pending['beforeShaftSeparationPx']))
                    self.store.write_json(f"step-{pending['index']:03d}-after.json",pending)
                    if abs(residual) > self.scope['maximumPredictionResidualPx']:
                        report['status']='prediction_residual_exceeded';break
                    if not reached and reduction < self.scope['minimumObservedProgressPx']:
                        report['status']='observed_no_progress';break
                if reached:
                    report['initialGoalAlreadySatisfied'] = report['confirmedCommands']==0
                    report['status']='observed_target_reached';break
                if report['commandAttempts'] >= budget:
                    report['status']='budget_reached';break
                decision = self._choose(value)
                prepared_at = self.clock.monotonic()
                self.store.write_json(f"decision-{len(report['observations']):03d}.json",
                    dict(observation=record, decision=decision, preparedAt=prepared_at))
                if decision['selected']['kind'] != 'action':
                    report['status']='abstained' if decision['status']=='abstained' else 'model_selected_stop'
                    break
                command = copy.deepcopy(decision['selected']['command'])
                seal = plan_digest(record)
                actor_command, actor_decision = copy.deepcopy(command), copy.deepcopy(decision)
                decision_seal = plan_digest(actor_decision)
                def check_before_run():
                    self._check()
                    require(plan_digest(record) == seal and actor_command == command and
                            plan_digest(actor_decision) == decision_seal, 'M1_scalar_trial_dispatch_binding_changed')
                    current = self._bound(record, after_off=off_at, fresh=False)
                    selection_at = self.clock.monotonic()
                    fresh = self._choose(current)
                    require(fresh['selected']['kind']=='action' and fresh['selected']['command']==command,
                            'M1_scalar_trial_dispatch_choice_changed')
                    now = self.clock.monotonic()
                    require(0 <= now-selection_at <= self.scope['maxDecisionAgeSeconds'] and
                            0 <= now-record['sourceHostBracket'][0] <= self.scope['maxObservationAgeSeconds'] and
                            record['persistedAt'] <= now-.1 and
                            report['commandAttempts'] <= budget and
                            now+command['durationMs']/1000 < self.deadline,
                            'M1_scalar_trial_dispatch_expired_or_budget_exhausted')
                    return now

                before_run, guard_state = _dispatch_guard(check_before_run)
                step = dict(index=report['commandAttempts']+1, status='execution_attempted',
                    before=copy.deepcopy(record), decision=copy.deepcopy(decision), command=copy.deepcopy(command),
                    beforeShaftSeparationPx=value, beforeGoalErrorPx=error,
                    predictedShaftSeparationPx=decision['selected']['predictedShaftSeparationPx'],
                    commandMeanPredictedShaftSeparationPx=learning.predict(self.model,value,command,self.model['context'],baseline=True))
                report['steps'].append(step)
                report['commandAttempts'] += 1
                try:
                    receipt = self.execute(actor_command,before_run,actor_decision)
                finally:
                    guard_state['open'] = False
                guarded_at = guard_state['at']
                require(guard_state['calls']==1 and _finite(guarded_at) and isinstance(receipt,dict) and receipt.get('offVerified') is True and
                        _finite(receipt.get('offAt')) and guarded_at <= receipt['offAt'] <= self.clock.monotonic(),
                        'M1_scalar_trial_execution_or_OFF_unverified')
                pulse=receipt.get('requestedPulse')
                require(isinstance(pulse,dict) and {k:pulse.get(k) for k in command}==command,
                        'M1_scalar_trial_receipt_command_changed')
                step.update(status='OFF_verified',receipt=copy.deepcopy(receipt),dispatchGuardAt=guarded_at)
                self.store.write_json(f"step-{step['index']:03d}-receipt.json",step)
                report['confirmedCommands'] += 1
                report['modelSelectedCommands'] += 1
                off_at, pending = receipt['offAt'], step
        except BaseException as exc:
            report['status']='interrupted' if not isinstance(exc,Exception) else 'failed'
            report['errors'].append(dict(type=type(exc).__name__,
                reason=str(exc) if isinstance(exc,ValueError) else 'M1_scalar_trial_execution_or_observation_failed'))
        finally:
            self.store.write_json('result.json',report)
        return report
