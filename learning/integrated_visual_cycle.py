"""별도 ㄱ자 실험의 유한 상위 단계 진행기.

기본 후보는 70%·1000ms 펄스다. 접근 단계만 명시적 native 목표각
(shoulder DOWN75%, 최대12초)을 별도 명령으로 계수한다. native 목표 미달과
명령 전달·출력·OFF 확인, 전체 과업 단계 성공은 서로 다른 결과다.

장치 접근 기능은 없다. 콜백은 고정 모델·현재 두 영상·ARM 직전 재선택과
기존 actor의 정지/원장 검사를 소유한다. permit은 실행 허가를 대체하지 않는
추가 예산 검사다. grasp_ready/closeReady는 영상상 폐합 후보 판정이며 lift 후
retained_lift도 주입된 영상 기준 판독이다. 이 코어는 실제 파지·접촉·깊이·힘을
독립 검증하지 않으며 물리 결과 평가는 별도 원본 판독으로 남겨야 한다.
"""
from __future__ import annotations

import copy
import math
import time

from learning.manual_collection_dataset import require

SCHEMA = 'motionbrain.integrated_visual_cycle.L_pose_with_native_angle.v4'
PHASES = ('approach', 'close', 'lift', 'lower', 'release')
DEFAULT_STAGE_LIMITS = dict(approach=4, close=6, lift=1, lower=1, release=3)


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _command(phase, command):
    if isinstance(command, dict) and command.get('mode') == 'native_target_angle':
        require(phase == 'approach' and set(command) ==
                {'mode', 'joint', 'direction', 'percent', 'targetAngleDeg', 'maxMotionSeconds'}
                and command['joint'] == 'shoulder' and command['direction'] == 'down'
                and type(command['percent']) is int and command['percent'] == 75
                and _finite(command['targetAngleDeg']) and 127.58 < command['targetAngleDeg'] < 296.12
                and _finite(command['maxMotionSeconds']) and command['maxMotionSeconds'] == 12.,
                'integrated_native_angle_command_required')
        return
    require(isinstance(command, dict) and set(command) ==
            {'joint', 'direction', 'percent', 'durationMs'}, 'integrated_command_fields')
    expected = {'approach': (('elbow', 'up'), ('elbow', 'down'), ('gripper', 'open')),
                'close': (('gripper', 'close'),), 'lift': (('elbow', 'up'),),
                'lower': (('elbow', 'down'),), 'release': (('gripper', 'open'),)}[phase]
    require((command['joint'], command['direction']) in expected
            and type(command['percent']) is int and command['percent'] == 70
            and type(command['durationMs']) is int and command['durationMs'] == 1000,
            'integrated_exact_cohort_command_required')


class _Permit:
    def __init__(self, cycle, phase, expected_command=None):
        self.cycle, self.phase, self.open = cycle, phase, True
        self.expected_command = copy.deepcopy(expected_command)
        self.pending = None
        self.receipts = []

    def __call__(self, command):
        c = self.cycle
        require(self.open and self.pending is None, 'integrated_permit_closed_or_pending')
        c._check()
        _command(self.phase, command)
        require(self.expected_command is None or command == self.expected_command,
                'integrated_approach_selection_changed')
        require(sum(row['phase'] == self.phase for row in c.report['commands']) < c.stage_limits[self.phase] and
                c.report['commandAttempts'] < c.max_commands, 'integrated_command_budget')
        seconds = (command['maxMotionSeconds'] if command.get('mode') == 'native_target_angle'
                   else command['durationMs']/1000)
        require(c.clock.monotonic()+seconds < c.deadline,
                'integrated_insufficient_command_time')
        token = object()
        row = dict(phase=self.phase, command=copy.deepcopy(command), status='unconfirmed',
                   reservedAt=c.clock.monotonic())
        c.report['commands'].append(row)
        c.report['commandAttempts'] += 1
        self.pending = (token, row)
        c._event('command_reserved', phase=self.phase, command=command)
        return token

    def complete(self, token, receipt):
        require(self.open and self.pending is not None and self.pending[0] is token,
                'integrated_permit_completion_invalid')
        row = self.pending[1]
        require(isinstance(receipt, dict) and receipt.get('status') == 'completed'
                and receipt.get('offVerified') is True and receipt.get('command') == row['command'],
                'integrated_command_or_OFF_unverified')
        # 완료가 기한 뒤 도착해도 실제 완료/OFF 근거는 분모에 남긴다.
        saved = copy.deepcopy(receipt)
        row.update(status='completed', receipt=saved, completedAt=self.cycle.clock.monotonic())
        self.receipts.append(saved)
        self.cycle.report['modelSelectedCommands'] += 1
        self.pending = None
        self.cycle._event('command_completed', phase=self.phase)
        self.cycle._check()


class IntegratedVisualCycle:
    """동기식 콜백으로 접근→폐합→들기·보유→내리기→해제를 한 번 진행한다.

    observe(phase)는 observationId/observedAt를 가진 현재 관측을 반환한다.
    choose_approach(observation)는 selected를 반환하며 STOP 진입 사유는
    stopReason=current_near_gripper_phase 또는 goal_reached로 명시한다.
    execute_approach(decision, permit), close/lift/lower/release(observation,
    permit)는 매 실제 RUN 직전 permit(command), 확인된 OFF 뒤
    permit.complete(token, receipt)를 호출한다. receipt에는 command/status/
    offVerified가 필요하다. 반환 stage result는 status, modelSelectedCommands,
    commandReceipts를 가지며 permit 기록과 정확히 일치해야 한다.
    close에는 원본 afterDecision 및 별도 closeReady 판독이 필요하다. 모델
    STOP 또는 두 번 이상 폐합한 뒤의 명시적 영상 정체 규칙으로만 종료한다.
    후자는 모델 ACTION 판단을 유지하며 모델이 STOP을 선택했다고 기록하지 않는다.
    접근은 M3 UP/DOWN, M1 OPEN 단발 또는 별도 native M4 목표각을 비교할 수 있다.
    native 명령도 RUN 직전 예산을 예약하고 확인된 OFF 후 완료하며, 목표 미달을
    단계 성공으로 치환하지 않는다. release 콜백은
    최대 세 개의 기존 1OPEN 실행 단위를 연결한다. 매 단위의 새 탁상 판정·
    모델 OPEN/STOP 비교·ARM 재선택은 콜백 책임이며 단발 모델의 예산 STOP을
    전체 해제 성공으로 취급하지 않는다. 최종 released 영상 판정은 별도다.
    grasp_ready(observation), retained_lift/table_supported/released(observation,
    stage_result)는 accepted와 비어 있지 않은 evidence를 반환한다.
    cleanup()은 실패·정상 모두 호출되며 기존 actor의 OFF 책임은 유지된다.

    동기 콜백을 강제로 중단할 수는 없다. 콜백 내부 대기·출력은 같은 전역
    deadline을 closure로 공유하고, 자체 actor 기한을 지켜야 한다.
    """
    def __init__(self, callbacks, *, max_commands=12, max_seconds=900,
                 max_observation_age=30, stage_limits=None, clock=time, emit=None):
        names = ('observe', 'choose_approach', 'execute_approach', 'grasp_ready', 'close',
                 'lift', 'retained_lift', 'lower', 'table_supported', 'release', 'released', 'cleanup')
        require(isinstance(callbacks, dict) and all(callable(callbacks.get(n)) for n in names),
                'integrated_callbacks_required')
        require(type(max_commands) is int and 1 <= max_commands <= 12 and
                _finite(max_seconds) and 1 <= max_seconds <= 900 and
                _finite(max_observation_age) and 0 < max_observation_age <= 30,
                'integrated_finite_budget_required')
        limits = dict(DEFAULT_STAGE_LIMITS if stage_limits is None else stage_limits)
        require(set(limits) == set(PHASES) and all(type(v) is int and
                1 <= v <= DEFAULT_STAGE_LIMITS[k] for k, v in limits.items()),
                'integrated_stage_limits')
        require(emit is None or callable(emit), 'integrated_emit_callback')
        self.callbacks, self.clock, self.emit = dict(callbacks), clock, emit
        self.max_commands, self.max_seconds = max_commands, max_seconds
        self.max_observation_age, self.stage_limits = max_observation_age, limits
        self.used = False
        self._observation_ids = set()

    def _check(self):
        require(_finite(self.clock.monotonic()) and self.clock.monotonic() < self.deadline,
                'integrated_deadline')

    def _event(self, kind, **details):
        event = dict(kind=kind, at=self.clock.monotonic(), **copy.deepcopy(details))
        self.report['events'].append(event)
        if self.emit is not None:
            self.emit(copy.deepcopy(event))

    def _observe(self, phase):
        self._check()
        observation = self.callbacks['observe'](phase)
        self._check()
        require(isinstance(observation, dict) and isinstance(observation.get('observationId'), str)
                and bool(observation['observationId']) and
                observation['observationId'] not in self._observation_ids and
                _finite(observation.get('observedAt')) and
                self.started <= observation['observedAt'] <= self.clock.monotonic() and
                self.clock.monotonic()-observation['observedAt'] <= self.max_observation_age,
                'integrated_observation_stale_or_reused')
        self._observation_ids.add(observation['observationId'])
        self.report['observations'].append(dict(phase=phase, observation=copy.deepcopy(observation)))
        return observation

    def _gate(self, name, observation, result=None):
        self._check()
        self._fresh(observation)
        assessment = self.callbacks[name](copy.deepcopy(observation), *(() if result is None else (copy.deepcopy(result),)))
        self._check()
        self._fresh(observation)
        self.report['assessments'].append(dict(gate=name, assessment=copy.deepcopy(assessment)))
        self._assessment(assessment, name)

    @staticmethod
    def _assessment(assessment, name):
        require(isinstance(assessment, dict) and assessment.get('accepted') is True and
                isinstance(assessment.get('evidence'), dict) and bool(assessment['evidence']),
                'integrated_'+name+'_not_verified')

    def _fresh(self, observation):
        require(0 <= self.clock.monotonic()-observation['observedAt'] <= self.max_observation_age,
                'integrated_observation_stale_or_reused')

    def _close_termination(self, result):
        decision = result.get('afterDecision')
        require(isinstance(decision, dict) and isinstance(decision.get('selected'), dict),
                'integrated_close_original_decision_required')
        selected = decision['selected']
        if selected.get('kind') == 'stop':
            require('closeTaskTermination' not in result, 'integrated_close_termination_ambiguous')
            return dict(owner='model_stop', modelSelectedStop=True)
        require(selected.get('kind') == 'action', 'integrated_close_original_decision_required')
        _command('close', selected.get('command'))
        termination = result.get('closeTaskTermination')
        require(isinstance(termination, dict) and set(termination) == {'reason', 'evidence'} and
                termination['reason'] == 'image_candidate_with_observed_plateau' and
                isinstance(termination['evidence'], dict), 'integrated_close_plateau_rule_required')
        evidence = termination['evidence']
        fields = {'previousGapPx', 'currentGapPx', 'previousObservedAt', 'currentObservedAt',
                  'previousObservationId', 'currentObservationId'}
        require(set(evidence) == fields and all(_finite(evidence[k]) for k in
                ('previousGapPx', 'currentGapPx', 'previousObservedAt', 'currentObservedAt')) and
                evidence['previousGapPx'] >= 0 and evidence['currentGapPx'] >= 0 and
                abs(evidence['currentGapPx']-evidence['previousGapPx']) <= 2.,
                'integrated_close_plateau_measurement_invalid')
        now = self.clock.monotonic()
        require(self.started <= evidence['previousObservedAt'] < evidence['currentObservedAt'] <= now and
                now-evidence['previousObservedAt'] <= self.max_observation_age and
                all(isinstance(evidence[k], str) and evidence[k] for k in
                    ('previousObservationId', 'currentObservationId')) and
                evidence['previousObservationId'] != evidence['currentObservationId'],
                'integrated_close_plateau_observations_invalid')
        require(sum(row['phase'] == 'close' and row['status'] == 'completed'
                    for row in self.report['commands']) >= 2, 'integrated_close_plateau_two_actions_required')
        return dict(owner='upper_image_plateau_rule', modelSelectedStop=False,
                    closeTaskTermination=copy.deepcopy(termination))

    def _invoke(self, phase, callback, argument, *, expected_command=None):
        self._check()
        permit = _Permit(self, phase, expected_command)
        self._active_permit = permit
        try:
            result = callback(copy.deepcopy(argument), permit)
            self._check()
            require(isinstance(result, dict), 'integrated_stage_result_required')
            self.report['stageResults'].append(dict(phase=phase, result=copy.deepcopy(result)))
            require(result.get('status') == 'completed', 'integrated_stage_not_completed')
            require(permit.pending is None and type(result.get('modelSelectedCommands')) is int and
                    result['modelSelectedCommands'] == len(permit.receipts) and
                    result.get('commandReceipts') == permit.receipts,
                    'integrated_stage_command_accounting')
            require(len(permit.receipts) >= (0 if phase == 'close' else 1),
                    'integrated_stage_no_model_action')
            if phase not in ('close', 'release'):
                require(len(permit.receipts) == 1, 'integrated_stage_single_action')
            return result
        finally:
            permit.open = False
            self._active_permit = None

    def run(self):
        require(not self.used, 'integrated_cycle_single_use')
        self.used = True
        self.started = self.clock.monotonic()
        require(_finite(self.started), 'integrated_clock_invalid')
        self.deadline = self.started+self.max_seconds
        self.report = dict(schemaVersion=SCHEMA, status='failed', phase='approach',
            commandAttempts=0, modelSelectedCommands=0, commands=[], events=[], observations=[],
            assessments=[], stageResults=[], phaseTransitions=[], errors=[],
            attemptCount=1, completedAttempts=0, failedAttempts=0, automaticRetry=False,
            callbackEvidenceIndependentlyAudited=False, graspImageAssessmentPassed=False, retainedLiftImageAssessmentPassed=False,
            tableReturnImageAssessmentPassed=False, releaseImageAssessmentPassed=False,
            releaseMicrotransactionPolicy=dict(commandsPerMicrotransaction=1,
                maximumMicrotransactions=self.stage_limits['release'],
                freshTableAssessmentAndModelReselectionOwner='release_callback',
                originalSingleActionModelPolicyChanged=False,
                microtransactionBudgetStopIsReleaseSuccess=False,
                finalReleasedImageAssessmentRequired=True))
        try:
            self._event('cycle_started')
            while True:
                observation = self._observe('approach')
                decision = self.callbacks['choose_approach'](copy.deepcopy(observation))
                self._check()
                self._fresh(observation)
                require(isinstance(decision, dict) and isinstance(decision.get('selected'), dict),
                        'integrated_approach_decision')
                kind = decision['selected'].get('kind')
                self._event('approach_decision', decision=decision)
                if kind == 'stop':
                    require(decision.get('stopReason') in ('current_near_gripper_phase', 'goal_reached'),
                            'integrated_approach_STOP_not_phase_success')
                    self._gate('grasp_ready', observation)
                    break
                require(kind == 'action', 'integrated_approach_decision_kind')
                _command('approach', decision['selected'].get('command'))
                result = self._invoke('approach', self.callbacks['execute_approach'], decision,
                                      expected_command=decision['selected']['command'])
                require(result['commandReceipts'][0]['command'] == decision['selected']['command'],
                        'integrated_approach_selection_changed')
            self._transition('close')
            result = self._invoke('close', self.callbacks['close'], self._observe('close'))
            self.report['closeTermination'] = self._close_termination(result)
            self._assessment(result.get('closeReady'), 'close_ready')
            self._transition('lift')
            result = self._invoke('lift', self.callbacks['lift'], self._observe('lift'))
            self._gate('retained_lift', self._observe('retained_lift'), result)
            self.report.update(graspImageAssessmentPassed=True, retainedLiftImageAssessmentPassed=True)
            self._transition('lower')
            result = self._invoke('lower', self.callbacks['lower'], self._observe('lower'))
            self._gate('table_supported', self._observe('table_supported'), result)
            self.report['tableReturnImageAssessmentPassed'] = True
            self._transition('release')
            result = self._invoke('release', self.callbacks['release'], self._observe('release'))
            self._gate('released', self._observe('released'), result)
            self.report.update(status='completed', releaseImageAssessmentPassed=True, completedAttempts=1)
        except BaseException as exc:
            self.report['status'] = 'failed' if isinstance(exc, Exception) else 'interrupted'
            self.report['errors'].append(dict(type=type(exc).__name__,
                reason=str(exc) if isinstance(exc, ValueError) else 'integrated_callback_failed'))
        finally:
            try:
                self.callbacks['cleanup']()
                self._check()
            except BaseException as exc:
                self.report.update(status='failed', completedAttempts=0)
                self.report['errors'].append(dict(type=type(exc).__name__, reason='integrated_cleanup_failed'))
            self.report['failedAttempts'] = int(self.report['status'] != 'completed')
            self.report['unconfirmedCommands'] = self.report['commandAttempts']-self.report['modelSelectedCommands']
            self.report['elapsedSeconds'] = self.clock.monotonic()-self.started
        return copy.deepcopy(self.report)

    def _transition(self, phase):
        self._check()
        previous = self.report['phase']
        self.report['phaseTransitions'].append(dict(before=previous, after=phase, at=self.clock.monotonic()))
        self.report['phase'] = phase
        self._event('phase_entered', phase=phase)
