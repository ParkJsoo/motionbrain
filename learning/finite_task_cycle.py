"""주입된 기존 실행 포트로만 동작하는 유한 관측-선택 과업 실행기.

장치 검색·인증·자동 재학습·기본 실물 진입점이 없다. 실행 어댑터는 기존
SequencePort/관절별 scoped actor의 출력·정지·원장 검사를 그대로 소유한다.
특히 M4 UP을 일반 SequencePort로 보내거나 private 실행 경로를 호출하지 않는다.
"""
from __future__ import annotations

import copy
import math
import time

from learning.manual_collection_dataset import require
from learning.model_grasp_trial import ModelGraspTrial
from learning.supervised_collection import plan_digest, identifier

SCHEMA = 'motionbrain.finite_task_cycle.v1'
STAGES = ('approach', 'close', 'lift', 'lower', 'release')


def finite(x):
    return type(x) in (int, float) and math.isfinite(x)


def sha(x):
    return isinstance(x, str) and len(x) == 64 and all(c in '0123456789abcdef' for c in x)


def bounded_command(stage, command):
    """새 실행 허용이 아니라 상위 과업의 추가 제한이다."""
    require(isinstance(command, dict) and set(command) == {'joint', 'direction', 'percent', 'durationMs'},
            'cycle_command_fields')
    joint, direction = command['joint'], command['direction']
    limits = {'close': ('gripper', 'close', 60, 650, 1, -1),
              'release': ('gripper', 'open', 60, 650, 1, 1),
              'lift': ('shoulder', 'up', 50, 650, 4, 1),
              'lower': ('shoulder', 'down', 35, 400, 4, -1)}
    if stage == 'approach':
        # 첫 국소 과업은 기존 M3 지원 후보만. 다른 관절을 묵시적으로 허용하지 않는다.
        limits[stage] = ('elbow', direction, 60, 350, 3, -1 if direction == 'up' else 1)
        require(direction in ('up', 'down'), 'cycle_approach_direction')
    require(stage in limits, 'cycle_stage_unknown')
    j, dr, pct, ms, motor, sign = limits[stage]
    require(joint == j and direction == dr and type(command['percent']) is int
            and 0 < command['percent'] <= pct and type(command['durationMs']) is int
            and 0 < command['durationMs'] <= ms, 'cycle_command_limit')
    return dict(command, motor=motor, expectedPwm=sign*(255*command['percent']//100))


def validate_scope(scope):
    require(isinstance(scope, dict) and set(scope) == {
        'schemaVersion', 'trialId', 'hostClockId', 'controllerBootId', 'stages',
        'maxCommands', 'maxRequestedMs', 'maxSeconds', 'maxObservationAgeSeconds',
        'maxDecisionAgeSeconds', 'postOffSeconds', 'retentionSeconds'}, 'cycle_scope_fields')
    require(scope['schemaVersion'] == SCHEMA and identifier(scope['trialId'])
            and isinstance(scope['hostClockId'], str) and bool(scope['hostClockId'])
            and isinstance(scope['controllerBootId'], str) and len(scope['controllerBootId']) == 32
            and all(c in '0123456789abcdef' for c in scope['controllerBootId']), 'cycle_scope_identity')
    require(type(scope['maxCommands']) is int and 1 <= scope['maxCommands'] <= 20
            and type(scope['maxRequestedMs']) is int and 1 <= scope['maxRequestedMs'] <= 15000,
            'cycle_finite_budget')
    for key, lo, hi in [('maxSeconds', 1, 600), ('maxObservationAgeSeconds', .1, 3),
                        ('maxDecisionAgeSeconds', .01, .25), ('retentionSeconds', 1, 10)]:
        require(finite(scope[key]) and lo <= scope[key] <= hi, 'cycle_timing_limit')
    require(scope['postOffSeconds'] == 3., 'cycle_post_off_window')
    require(isinstance(scope['stages'], list) and len(scope['stages']) == len(STAGES), 'cycle_all_stages_required')
    for kind, stage in zip(STAGES, scope['stages']):
        require(isinstance(stage, dict) and set(stage) == {
            'kind', 'context', 'modelSha256', 'maxCommands', 'commands', 'target'}, 'cycle_stage_fields')
        require(stage['kind'] == kind and isinstance(stage['context'], dict) and bool(stage['context'])
                and sha(stage['modelSha256']) and type(stage['maxCommands']) is int
                and 1 <= stage['maxCommands'] <= 4, 'cycle_stage_contract')
        require(isinstance(stage['commands'], list) and bool(stage['commands']), 'cycle_candidates_required')
        keys = [plan_digest(c) for c in stage['commands']]
        require(len(keys) == len(set(keys)), 'cycle_duplicate_candidate')
        for command in stage['commands']:
            bounded_command(kind, command)
    require(len({s['context'].get('object') for s in scope['stages']}) == 1
            and isinstance(scope['stages'][0]['context'].get('object'), str)
            and bool(scope['stages'][0]['context']['object']), 'cycle_object_changed')
    # target/state interpretation is owned and validated by the bound model adapter.
    plan_digest(scope)
    return copy.deepcopy(scope)


class FiniteTaskCycle:
    """Each callback must use the same host monotonic clock and durable evidence.

    observe(stage, after_off, deadline) returns fresh two-camera evidence plus
    state. verify_observation(stage, record) re-audits its registered/raw bindings.
    choose(stage, record) returns selected(kind/command), modelSha256.
    goal(stage, record) returns reached plus phase proof. verify_goal(stage,
    record, proof) independently checks its source hashes and observation span.
    execute(stage, command, prepare, before_run, decision, safety) must call prepare ONCE
    after ARM/OFF preparation, outside RUN's HTTP deadline, then before_run ONCE
    immediately before RUN. prepare may capture; before_run is memory-only.
    check_idle() verifies same controller boot + all five OFF and returns bootId.
    stop() performs existing emergency STOP/OFF cleanup, never retries a RUN.
    No callback is supplied by default; this object grants no live authorization.
    """
    def __init__(self, scope, *, observe, verify_observation, choose, goal, verify_goal, execute,
                 verify_assets, check_idle, stop, store, clock=time):
        self.scope = validate_scope(scope)
        for value in (observe, verify_observation, choose, goal, verify_goal, execute, verify_assets, check_idle, stop):
            require(callable(value), 'cycle_callback_required')
        require(callable(getattr(store, 'write_json', None)), 'cycle_durable_store_required')
        self.observe, self.verify_observation, self.choose, self.goal = observe, verify_observation, choose, goal
        self.verify_goal = verify_goal
        self.execute, self.verify_assets, self.check_idle, self.stop = execute, verify_assets, check_idle, stop
        self.store, self.clock = store, clock
        self.scope_sha = plan_digest(self.scope)
        self.used, self.seen = False, set()
        self.m4_reference = None

    def _check(self):
        require(plan_digest(self.scope) == self.scope_sha, 'cycle_scope_changed')
        self.verify_assets()
        require(self.clock.monotonic() < self.deadline, 'cycle_deadline')

    def _idle(self):
        self._check()
        require(self.check_idle() == self.scope['controllerBootId'], 'cycle_boot_or_off_changed')
        self._check()

    def _record(self, stage, *, after_off):
        self._check()
        record = self.observe(copy.deepcopy(stage), after_off=after_off, deadline=self.deadline)
        self._validate_record(stage, record, after_off=after_off)
        ids = tuple(f['frameId'] for f in record['frames'])
        require(not self.seen.intersection(ids) and len(set(ids)) == 2, 'cycle_frame_reused')
        self.seen.update(ids)
        return copy.deepcopy(record)

    def _validate_record(self, stage, record, *, after_off):
        self._check()
        require(isinstance(record, dict) and record.get('context') == stage['context']
                and record.get('hostClockId') == self.scope['hostClockId']
                and record.get('controllerBootId') == self.scope['controllerBootId']
                and record.get('stable') is True, 'cycle_observation_contract')
        angle = record.get('m4AngleDeg')
        require(finite(angle) and 127.58 < angle < 296.12, 'cycle_M4_angle_required')
        if self.m4_reference is None:
            self.m4_reference = angle
        require(abs(angle-self.m4_reference) <= 3., 'cycle_M4_total_excursion')
        frames = record.get('frames')
        require(isinstance(frames, list) and len(frames) == 2
                and {f.get('camera') for f in frames} == {'external', 'gripper'}, 'cycle_two_cameras_required')
        for f in frames:
            require(isinstance(f.get('frameId'), str) and bool(f['frameId']) and sha(f.get('pixelSha256')),
                    'cycle_frame_binding')
        self._freshness(record, after_off=after_off)
        # A trusted, injected audit owns metrology; flags alone are insufficient.
        require(self.verify_observation(copy.deepcopy(stage), copy.deepcopy(record)) is True,
                'cycle_observation_audit_failed')
        self._freshness(record, after_off=after_off)

    def _freshness(self, record, *, after_off=None):
        self._check()
        now = self.clock.monotonic()
        for f in record['frames']:
            times = [f.get(k) for k in ('sourceEarliest', 'sourceLatest', 'persistedAt')]
            require(all(finite(v) for v in times) and times == sorted(times)
                    and self.started <= times[0] <= times[-1] <= now
                    and now-times[0] <= self.scope['maxObservationAgeSeconds'], 'cycle_stale_observation')
            if after_off is not None:
                require(times[0] >= after_off+self.scope['postOffSeconds'], 'cycle_after_off_window')

    def _decision(self, stage, record):
        decision = self.choose(copy.deepcopy(stage), copy.deepcopy(record))
        self._check()
        require(isinstance(decision, dict) and decision.get('modelSha256') == stage['modelSha256'],
                'cycle_model_binding_changed')
        selected = decision.get('selected', {})
        require(selected.get('kind') in ('stop', 'action'), 'cycle_selection_invalid')
        if selected['kind'] == 'action':
            require(selected.get('command') in stage['commands'], 'cycle_unlisted_command')
            bounded_command(stage['kind'], selected['command'])
        else:
            require(selected.get('command') is None, 'cycle_stop_has_command')
        return copy.deepcopy(decision)

    def _goal(self, stage, record, *, after_off=None):
        result = self.goal(copy.deepcopy(stage), copy.deepcopy(record))
        require(isinstance(result, dict) and type(result.get('reached')) is bool
                and sha(result.get('evidenceSha256'))
                and result.get('observationSha256') == plan_digest(record)
                and type(result.get('humanAssisted')) is bool, 'cycle_goal_proof_required')
        if result['reached']:
            predicates = {'approach': ('noncontact', 'clearance'),
                          'close': ('bilateralContact',),
                          'lift': ('airborne', 'retained'),
                          'lower': ('tableSupported',),
                          'release': ('tableSupported', 'bothJawsClear')}[stage['kind']]
            require(all(result.get(k) is True for k in predicates), 'cycle_phase_proof_missing')
            if stage['kind'] == 'lift':
                times = result.get('observationTimes', [])
                require(isinstance(times, list) and len(times) >= 2 and all(finite(t) for t in times)
                        and times == sorted(times) and times[-1]-times[0] >= self.scope['retentionSeconds']
                        and self.started <= times[0] and times[-1] <= self.clock.monotonic(),
                        'cycle_retention_span_missing')
                if after_off is not None:
                    require(times[0] >= after_off+self.scope['postOffSeconds'],
                            'cycle_retention_before_last_output_off')
        require(self.verify_goal(copy.deepcopy(stage), copy.deepcopy(record), copy.deepcopy(result)) is True,
                'cycle_goal_audit_failed')
        self._freshness(record)
        return copy.deepcopy(result)

    def _entry(self, stage, record):
        """감사된 접촉·지지 판단이며 모델의 픽셀 예측으로 대신하지 않는다."""
        proof = record.get('entry', {})
        require(proof.get('clearance') is True and sha(proof.get('evidenceSha256')),
                'cycle_clearance_proof_required')
        kind = stage['kind']
        required = {'approach': ('noncontact',), 'close': ('objectBetweenJaws', 'depthVerified'),
                    'lift': ('bilateralContact',),
                    'lower': ('bilateralContact', 'airborne'),
                    'release': ('tableSupported',)}[kind]
        require(all(proof.get(k) is True for k in required), 'cycle_entry_proof_required')
        if kind == 'lift':
            require(proof.get('tableSupported') is True or proof.get('airborne') is True,
                    'cycle_lift_support_unknown')

    def run(self):
        require(not self.used, 'cycle_single_use')
        self.used = True
        self.started = self.clock.monotonic()
        self.deadline = self.started+self.scope['maxSeconds']
        report = dict(schemaVersion=SCHEMA, trialId=self.scope['trialId'], scopeSha256=self.scope_sha,
                      status='failed', commandAttempts=0, confirmedCommands=0, requestedMs=0,
                      fullyModelSelectedCycle=False, humanAssisted=False, stages=[], errors=[])
        off_at = None
        try:
            # ArtifactStore refuses overwrite. A new object cannot resume an old consumed trial.
            self.store.write_json('cycle-consumed.json', dict(scope=self.scope, startedAt=self.started))
            self._idle()
            for stage in self.scope['stages']:
                stage_report = dict(kind=stage['kind'], status='observing', steps=[])
                report['stages'].append(stage_report)
                while True:
                    self._idle()
                    record = self._record(stage, after_off=off_at)
                    goal = self._goal(stage, record, after_off=off_at)
                    report['humanAssisted'] |= goal['humanAssisted']
                    stage_report['goal'] = goal
                    if goal['reached']:
                        stage_report['status'] = 'reached'
                        break
                    self._entry(stage, record)
                    require(len(stage_report['steps']) < stage['maxCommands']
                            and report['commandAttempts'] < self.scope['maxCommands'], 'cycle_command_budget')
                    decision = self._decision(stage, record)
                    require(decision['selected']['kind'] == 'action', 'cycle_abstained_before_goal')
                    command = decision['selected']['command']
                    require(report['requestedMs']+command['durationMs'] <= self.scope['maxRequestedMs'],
                            'cycle_requested_time_budget')
                    step = dict(index=report['commandAttempts']+1, command=command,
                                observation=record, decision=decision, status='not_dispatched')
                    stage_report['steps'].append(step)
                    report['commandAttempts'] += 1
                    report['requestedMs'] += command['durationMs']
                    self.store.write_json(f"cycle-step-{step['index']:03d}-intent.json", step)
                    prepared, guards = [], []
                    active = True

                    def prepare():
                        require(active and not prepared and not guards, 'cycle_prepare_reused')
                        current = self._record(stage, after_off=off_at)
                        require(all(b['sourceLatest'] >= a['sourceLatest'] for a, b in
                                    zip(sorted(record['frames'], key=lambda f:f['camera']),
                                        sorted(current['frames'], key=lambda f:f['camera']))),
                                'cycle_prepare_observation_regressed')
                        fresh_goal = self._goal(stage, current, after_off=off_at)
                        report['humanAssisted'] |= fresh_goal['humanAssisted']
                        require(not fresh_goal['reached'], 'cycle_goal_changed_before_run')
                        self._entry(stage, current)
                        fresh = self._decision(stage, current)
                        require(fresh['selected']['kind'] == 'action' and fresh['selected']['command'] == command,
                                'cycle_choice_changed_before_run')
                        prepared.append((current, fresh, self.clock.monotonic()))
                        self.store.write_json(f"cycle-step-{step['index']:03d}-prepared.json",
                                              dict(observation=current, decision=fresh))

                    def before_run():
                        require(active and len(prepared) == 1 and not guards, 'cycle_dispatch_guard_reused')
                        self._check()  # verify_assets must be memory-only in this callback.
                        current, fresh, at = prepared[0]
                        now = self.clock.monotonic()
                        require(now-at <= self.scope['maxDecisionAgeSeconds']
                                and all(now-f['sourceEarliest'] <= self.scope['maxObservationAgeSeconds']
                                        for f in current['frames'])
                                and now+command['durationMs']/1000 < self.deadline, 'cycle_dispatch_expired')
                        guards.append(now)
                        return now

                    try:
                        receipt = self.execute(copy.deepcopy(stage), copy.deepcopy(command),
                            prepare=prepare, before_run=before_run, decision=copy.deepcopy(decision),
                            safety=dict(controllerBootId=self.scope['controllerBootId'],
                                        M4ReferenceDeg=self.m4_reference, maximumM4ExcursionDeg=3.))
                        require(len(prepared) == len(guards) == 1, 'cycle_dispatch_callbacks_missing')
                        off_at = ModelGraspTrial._receipt(self, receipt, bounded_command(stage['kind'], command))
                        require(receipt['dispatchBefore'] >= guards[0], 'cycle_receipt_before_guard')
                        self._idle()
                        step.update(status='executed', receipt=copy.deepcopy(receipt))
                        report['confirmedCommands'] += 1
                    except (Exception, KeyboardInterrupt):
                        step['status'] = 'failed_or_unknown_no_retry'
                        raise
                    finally:
                        active = False
                        self.store.write_json(f"cycle-step-{step['index']:03d}-result.json", step)
            # 이미 사람이 닫거나 들어 둔 상태로 시작한 단계는 모델 완료로 세지 않는다.
            report.update(status='completed', fullyModelSelectedCycle=all(
                bool(s['steps']) for s in report['stages'] if s['kind'] != 'approach'))
        except (Exception, KeyboardInterrupt) as exc:
            report['errors'].append(dict(type=type(exc).__name__, reason=str(exc) if isinstance(exc, ValueError)
                                         else 'cycle_interrupted_or_failed'))
        finally:
            # STOP/OFF cleanup is allowed after failure, never another movement.
            try:
                self.stop()
                report['finalOffVerified'] = self.check_idle() == self.scope['controllerBootId']
                require(report['finalOffVerified'], 'cycle_final_off_unverified')
            except (Exception, KeyboardInterrupt):
                report['finalOffVerified'] = False
                report['errors'].append(dict(type='cleanup', reason='cycle_final_off_unverified'))
            if report['errors']:
                report['status'] = 'failed'
                report['fullyModelSelectedCycle'] = False
            self.store.write_json('cycle-result.json', report)
        return report
