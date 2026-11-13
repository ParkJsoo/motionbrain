"""개발 관측·고정 모델을 위한 유한 M3 비접촉 접근 실험.

기존 모델의 제어 적격이나 평가 판정을 승격하지 않는다. 현장 검토·현재 부팅·
관측 원본·정지 규칙을 묶은 별도 계약이 있어야 기존 반복 실행 코어에 연결한다.
생성자는 파일만 읽으며 촬영·인증·제어기 연결을 시작하지 않는다.
"""
from __future__ import annotations

import copy
import time
from pathlib import Path

import numpy as np

from learning.artifacts import _decode_json, _read
from learning.development_gripper_source import load_observer, observe_bound_frame, scene_context
from learning.manual_collection_dataset import require
from learning.model_grasp_trial import (APPROACH_SCHEMA, ModelGraspTrial, _hash,
                                       _number, validate_scope)
from learning.relative_action_learning import command_key, select, vector
from learning.relative_model_bundle import sha256, validate_model
from learning.supervised_collection import plan_digest
from learning.supervised_transport import (ELBOW_DEADLINE_POLICY, ELBOW_FINE_POLICY, ELBOW_PLATEAU_POLICY,
    WRIST_READ_BUDGET_REVISION, validate_monitor_policy, QUIET_IDLE_POLL_SECONDS, check_quiet_idle)

ADMISSION_SCHEMA = 'motionbrain.development_approach_admission.v1'


def _bound_json(root, binding):
    require(isinstance(binding, dict) and set(binding) == {'path', 'sha256'}
            and _hash(binding['sha256']), 'approach_hashed_artifact_required')
    raw = _read(root, binding['path'])
    require(sha256(raw) == binding['sha256'], 'approach_artifact_changed')
    return _decode_json(raw)


class ApproachModelBank:
    """M3 고정 모델 하나와 이번 실험 계약. 기본 FrozenModelBank와 분리한다."""
    def __init__(self, root, admission_binding):
        self.root = Path(root).resolve(strict=True)
        self.admission_binding = copy.deepcopy(admission_binding)
        a = _bound_json(self.root, admission_binding)
        require(isinstance(a, dict) and set(a) == {
            'schemaVersion', 'model', 'observer', 'scope', 'sceneReview', 'validUntil',
            'currentCameraBootId', 'minimumState', 'maximumState', 'stopping', 'externalGuard'},
            'approach_admission_fields')
        require(a['schemaVersion'] == ADMISSION_SCHEMA and _number(a['validUntil']),
                'approach_admission_schema')
        model = validate_model(_bound_json(self.root, a['model']), allow_development_observations=True)
        require(model.get('developmentObservationOnly') is True and
                model['context'].get('executionTimingPolicy') in
                (ELBOW_DEADLINE_POLICY, ELBOW_FINE_POLICY, ELBOW_PLATEAU_POLICY),
                'approach_M3_development_model_required')
        scope = a['scope']
        self._model, self._hashes = model, {'approach': a['model']['sha256']}
        observer = load_observer(self.root, a['observer']['path'], a['observer']['sha256'])
        actual = scene_context(observer)
        require(all(model['context'][k] == v for k, v in actual.items()) and
                model['context']['imageShape'] == observer.parameters['imageShape'],
                'approach_model_observer_contract_changed')
        self.observer = observer
        self.actual_context = copy.deepcopy(model['context'])
        boot = a['currentCameraBootId']
        require(isinstance(boot, str) and len(boot) == 32 and
                all(c in '0123456789abcdef' for c in boot), 'approach_current_camera_boot_required')
        # 원본 모델은 보존한다. 별도 현장 검토에 따라 실제 관측의 부팅 ID만 연결한다.
        self.actual_context['cameraIdentity']['cameraBootId'] = boot
        guard_config = _bound_json(self.root, a['externalGuard'])
        require(isinstance(guard_config, dict) and
                guard_config.get('schemaVersion') in ('motionbrain.approach_external_guard.v1',
                                                      'motionbrain.current_scene_external_guard.v1') and
                guard_config.get('controllerBootId') == scope['controllerBootId'] and
                guard_config.get('hostClockId') == scope['hostClockId'] and
                guard_config.get('gripperCameraBootId') == boot and
                guard_config.get('executionTimingPolicy') == model['context']['executionTimingPolicy'],
                'approach_external_guard_contract_changed')
        if guard_config['schemaVersion'] == 'motionbrain.current_scene_external_guard.v1':
            from learning.approach_external_guard import CURRENT_REFERENCE_MEANING
            require(guard_config.get('referenceMeaning') == CURRENT_REFERENCE_MEANING and
                    guard_config['executionTimingPolicy'] == ELBOW_DEADLINE_POLICY,
                    'approach_current_scene_guard_contract_changed')
        self.external_guard_config = guard_config
        review = _bound_json(self.root, a['sceneReview'])
        require(isinstance(review, dict) and set(review) == {
            'hostClockId', 'controllerBootId', 'cameraBootId', 'actualScene', 'reviewer',
            'externalFrame', 'gripperFrame', 'sceneMatched', 'noncontactClearanceVerified'},
            'approach_scene_review_fields')
        require(isinstance(review['reviewer'], str) and bool(review['reviewer']) and
                review['sceneMatched'] is True and review['noncontactClearanceVerified'] is True and
                review['hostClockId'] == scope['hostClockId'] and
                review['controllerBootId'] == scope['controllerBootId'] and
                review['cameraBootId'] == boot and review['actualScene'] == actual,
                'approach_current_scene_review_required')
        for side in ('externalFrame', 'gripperFrame'):
            binding = review[side]
            require(isinstance(binding, dict) and set(binding) == {'path', 'sha256'},
                    'approach_reviewed_frame_binding')
            require(sha256(_read(self.root, binding['path'])) == binding['sha256'],
                    'approach_reviewed_frame_changed')
        lo, hi = vector(a['minimumState']), vector(a['maximumState'])
        require(np.all(lo < hi), 'approach_noncontact_envelope_required')
        self.admission = copy.deepcopy(a)
        stopping = a['stopping']
        require(isinstance(stopping, dict) and set(stopping) == {
            'minimumPredictedImprovementPx', 'minimumObservedProgressPx', 'maxNoProgress',
            'revisitTolerancePx', 'maximumPredictionResidualPx'}, 'approach_stopping_fields')
        require(all(_number(v) and 0 < v <= 4 for k, v in stopping.items() if k != 'maxNoProgress')
                and type(stopping['maxNoProgress']) is int and 1 <= stopping['maxNoProgress'] <= 2,
                'approach_finite_stopping_policy')
        validate_scope(scope, self, position_only=True)
        require(len(scope['stages']) == 1 and scope['maxCommands'] <= 4 and
                scope['maxRequestedMs'] <= 2600 and scope['maxSeconds'] <= 120,
                'approach_single_M3_budget')
        stage = scope['stages'][0]
        require(stage['modelId'] == 'approach' and self.inside(stage['target']) and
                stage['maxCommands'] <= scope['maxCommands'], 'approach_stage_contract')
        for command in stage['commands']:
            command_key(command)
            require(command['joint'] == 'elbow' and command['direction'] in ('up', 'down'),
                    'approach_M3_candidates_only')
            validate_monitor_policy(command, model['context']['executionTimingPolicy'])
            require(scope['maxRequestedMs'] <= scope['maxCommands']*command['durationMs'],
                    'approach_requested_time_budget')
        self._snapshot = plan_digest([self.admission, self._model, self.actual_context, self._hashes,
                                      self.external_guard_config])

    @property
    def hashes(self):
        return copy.deepcopy(self._hashes)

    def context(self, model_id):
        require(model_id == 'approach', 'approach_single_model_required')
        return copy.deepcopy(self.actual_context)

    def verify(self):
        current = ApproachModelBank(self.root, self.admission_binding)
        require(current._snapshot == self._snapshot,
                'approach_admission_changed')
        self.check_frozen()

    def check_frozen(self):
        require(plan_digest([self.admission, self._model, self.actual_context, self._hashes,
                             self.external_guard_config]) == self._snapshot,
                'approach_frozen_contract_changed')

    def inside(self, state):
        x = vector(state)
        return bool(np.all(x >= self.admission['minimumState']) and
                    np.all(x <= self.admission['maximumState']))

    def choose(self, stage, state, context):
        self.check_frozen()
        require(context == self.context(stage['modelId']), 'approach_actual_context_changed')
        require(self.inside(state), 'approach_observed_noncontact_envelope_exceeded')
        result = select(self._model, state, stage['target'], stage['commands'], self._model['context'],
                        minimum_improvement_px=self.admission['stopping']['minimumPredictedImprovementPx'])
        safe = []
        for candidate in result['candidates']:
            if candidate['kind'] == 'action' and not self.inside(candidate['predictedState']):
                result['rejected'].append(dict(command=candidate['command'],
                                               reason='approach_predicted_noncontact_envelope_exceeded'))
            else:
                safe.append(candidate)
        result['candidates'] = safe
        result['selected'] = copy.deepcopy(safe[0])
        stop = next(c for c in safe if c['kind'] == 'stop')
        gain = stop['errorPx'] - safe[0]['errorPx']
        if safe[0]['kind'] == 'action' and gain <= self.admission['stopping']['minimumPredictedImprovementPx']:
            result['selected'] = copy.deepcopy(stop)
        result.update(status='ranked' if len(safe) > 1 else 'abstained',
                      bestPredictedImprovementPx=gain, sourceModelContext=copy.deepcopy(self._model['context']),
                      actualObservationContext=copy.deepcopy(context),
                      admissionSha256=self.admission_binding['sha256'], controlEligible=False)
        return result


class DevelopmentApproachTrial(ModelGraspTrial):
    """기존 재관측·OFF·무재송 코어를 재사용한다. 접촉 동작은 포함하지 않는다."""
    def __init__(self, scope, bank, observer, port, *args, **kwargs):
        require(isinstance(bank, ApproachModelBank) and isinstance(observer, ApproachObserver),
                'approach_bound_adapters_required')
        require(observer.bank is bank, 'approach_observer_bank_changed')
        require(observer.guard.port is port, 'approach_guard_execution_port_changed')
        self._states, self._no_progress, self._prediction = [], 0, None
        self._last_quiet_poll = None
        super().__init__(scope, bank, observer, port, *args, **kwargs)

    def _validate_scope(self, scope, bank):
        expected = dict(bank.admission['scope'], admissionSha256=bank.admission_binding['sha256'])
        require(scope == expected, 'approach_exact_admitted_scope_required')
        validate_scope(bank.admission['scope'], bank, position_only=True)
        return copy.deepcopy(scope)

    def run(self, *, confirmed_scope_sha256):
        self.bank.verify()
        self._validate_scope(self.scope, self.bank)
        require(self.clock.monotonic() + self.scope['maxSeconds'] <= self.bank.admission['validUntil'],
                'approach_admission_expired')
        return super().run(confirmed_scope_sha256=confirmed_scope_sha256)

    def _deadline(self):
        self.bank.check_frozen()
        self._validate_scope(self.scope, self.bank)
        require(self.observer.guard.port is self.port, 'approach_guard_execution_port_changed')
        return super()._deadline()

    def _idle_tick(self):
        self._deadline()
        self.observer.check()
        now = self.clock.monotonic()
        if self._last_quiet_poll is None or now-self._last_quiet_poll >= QUIET_IDLE_POLL_SECONDS:
            check_quiet_idle(self.port)
            self._last_quiet_poll = self.clock.monotonic()

    def _extract_state(self, annotation, frame):
        state = self.observer.state(annotation, frame)
        require(self.bank.inside(state), 'approach_observed_noncontact_envelope_exceeded')
        return state

    def _position_stop(self, stage, state):
        require(self.bank.inside(state), 'approach_observed_noncontact_envelope_exceeded')
        x = vector(state); policy = self.bank.admission['stopping']
        if self._prediction is not None and np.linalg.norm(x-self._prediction) > policy['maximumPredictionResidualPx']:
            return 'prediction_residual_exceeded'
        if self._states:
            before = np.linalg.norm(self._states[-1]-vector(stage['target']))
            after = np.linalg.norm(x-vector(stage['target']))
            self._no_progress = self._no_progress+1 if before-after < policy['minimumObservedProgressPx'] else 0
            if self._no_progress >= policy['maxNoProgress']:
                return 'observed_no_progress'
        if any(np.linalg.norm(x-previous) <= policy['revisitTolerancePx'] for previous in self._states[:-1]):
            return 'observed_revisit'
        self._states.append(x)
        return None

    def _execute(self, pulse, guard):
        def guarded():
            checked_at = guard()
            stage = self.scope['stages'][0]
            record = self.observer.latest(stage)
            state = self._observe(record, stage)
            choice = self.bank.choose(stage, state, record['context'])
            require(choice['selected']['kind'] == 'action' and
                    choice['selected']['command'] == {k: pulse[k] for k in
                        ('joint', 'direction', 'percent', 'durationMs')}, 'approach_dispatch_choice_changed')
            self.observer.guard.validate_command(choice['selected']['command'])
            self._prediction = vector(choice['selected']['predictedState'])
            self._deadline()
            now = self.clock.monotonic()
            require(now-checked_at <= self.scope['maxDecisionAgeSeconds'] and
                    0 <= now-record['sourceEarliest'] <= self._input_age_limit(record['context']) and
                    record['persistedAt'] <= now-.1, 'approach_dispatch_decision_expired')
        try:
            return self.port.execute(pulse, self._active_tick, before_run=guarded,
                                     monitor_policy=self.bank._model['context']['executionTimingPolicy'])
        finally:
            self.observer.guard.disarm()

    def _receipt(self, receipt, pulse):
        require(receipt.get('monitorTimingPolicy') == self.bank._model['context']['executionTimingPolicy'] and
                receipt.get('monitorStatusVerified') is True and
                receipt.get('monitorReadBudgetRevision') == WRIST_READ_BUDGET_REVISION,
                'approach_output_timing_contract_changed')
        return super()._receipt(receipt, pulse)


class ApproachObserver:
    """고정 개발 관측기와 실제 fresh 원본. guard는 현재 외부영상 안전검사를 맡는다.

    guard.check()는 RUN 안에서도 호출되므로 비차단이어야 한다.
    guard.review(record, result, deadline, tick)는 새 외부영상·현장 범위를 확인한다.
    두 콜백이 없는 경우 실행 어댑터를 만들 수 없다.
    """
    def __init__(self, bank, source, guard, *, clock=time):
        require(isinstance(bank, ApproachModelBank) and callable(getattr(guard, 'check', None)) and
                callable(getattr(guard, 'review', None)) and callable(getattr(guard, 'validate_command', None)) and
                callable(getattr(guard, 'disarm', None)),
                'approach_external_scene_guard_required')
        require(source.boot == bank.admission['currentCameraBootId'] and
                source.session_id == bank.admission['scope']['hostClockId'] and
                source._scene() == scene_context(bank.observer), 'approach_source_contract_changed')
        self.bank, self.source, self.guard, self.clock = bank, source, guard, clock
        self._check_guard_binding()
        self.cache, self.previous, self._sealed = None, None, {}

    def _check_guard_binding(self):
        expected = self.bank.admission['externalGuard']
        actual = self.guard.config_binding
        require((self.bank.root/expected['path']).resolve(strict=True) ==
                (self.guard.repository/actual['path']).resolve(strict=True) and
                actual['sha256'] == expected['sha256'] and
                self.guard.config == self.bank.external_guard_config and
                self.guard.port.boot_id == self.bank.admission['scope']['controllerBootId'],
                'approach_external_guard_binding_changed')

    def start(self, *, deadline):
        self.source.start(deadline=deadline)

    def check(self):
        self._check_guard_binding()
        self.source.check()
        self.guard.check()
        require(self.clock.monotonic() < self.bank.admission['validUntil'], 'approach_admission_expired')

    def observe(self, stage, *, after_off, deadline, tick):
        until = after_off+3. if after_off is not None else self.clock.monotonic()
        while self.clock.monotonic() < until:
            require(self.clock.monotonic() < deadline, 'approach_observation_deadline')
            tick(); self.clock.sleep(max(0., min(.05, until-self.clock.monotonic())))
        record = self.source.capture(deadline=deadline)
        self.source.verify(record)
        binding = self.source._bindings[record['frame']['frameId']]
        result = observe_bound_frame(self.bank.observer, self.source.store.root/binding['folder'], 'frame',
            session_id=self.source.session_id, camera_boot_id=self.source.boot,
            actual_scene=self.source._scene(), now=self.clock.monotonic(),
            max_age_seconds=self.bank.admission['scope']['maxObservationAgeSeconds'], previous=self.previous)
        require(result['status'] == 'complete_development' and result['state'] is not None and
                result['referenceImageSelfReplay'] is False, 'approach_observation_incomplete')
        snapshot = plan_digest([record, result])
        review = self.guard.review(copy.deepcopy(record), copy.deepcopy(result), deadline=deadline, tick=tick)
        require(isinstance(review, dict) and set(review) == {'accepted', 'observationSha256', 'evidence'}
                and review['accepted'] is True and review['observationSha256'] == snapshot,
                'approach_scene_guard_rejected')
        proof = _bound_json(self.source.store.root, review['evidence'])
        require(isinstance(proof, dict) and proof.get('accepted') is True and
                proof.get('observationSha256') == snapshot, 'approach_scene_proof_content_changed')
        require(plan_digest([record, result]) == snapshot, 'approach_guard_mutated_observation')
        self.source.verify(record)
        require(self.clock.monotonic() < deadline, 'approach_observation_deadline')
        context = dict(record['context'], executionTimingPolicy=self.bank._model['context']['executionTimingPolicy'])
        require(context == self.bank.context(stage['modelId']), 'approach_actual_context_changed')
        record.update(context=context, stable=True, annotation=dict(method='development_visible_state',
            frameId=record['frame']['frameId'], pixelSha256=record['frame']['pixelSha256'],
            observationSha256=plan_digest(result)))
        self.source.store.write_json(binding['folder']+'/approach-observation.json', result)
        self._sealed[record['frame']['frameId']] = (copy.deepcopy(record), copy.deepcopy(result))
        self.cache, self.previous = copy.deepcopy(record), copy.deepcopy(result)
        self.check()
        return copy.deepcopy(record)

    def state(self, annotation, frame):
        require(frame['frameId'] in self._sealed, 'approach_unverified_frame')
        record, result = self._sealed[frame['frameId']]
        require(frame == record['frame'] and annotation == record['annotation'] and
                plan_digest(result) == annotation['observationSha256'], 'approach_observation_changed')
        return copy.deepcopy(result['state'])

    def latest(self, stage):
        self.check()
        require(self.cache is not None, 'approach_observation_missing')
        return copy.deepcopy(self.cache)

    def close(self):
        return self.source.close()
