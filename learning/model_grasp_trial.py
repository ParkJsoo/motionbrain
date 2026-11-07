"""Bounded model-selected positioning/close trial with injected observations.

No device discovery, automatic training, perception model or default live entry
point. SequencePort owns bounded output, STOP/OFF and no-retry semantics.
Landmark and retention review remain explicit human observation assistance.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import re
import time

from learning.manual_collection_dataset import require
from learning.artifacts import _decode_json, _read
from learning.manual_collection_dataset import POLICY as EXTERNAL_POLICY
from learning.relative_action_learning import command_key, point_state, select, vector
from learning.relative_model_bundle import load_frozen_model, sha256
from learning.supervised_collection import PROFILES, identifier, plan_digest

SCHEMA = 'motionbrain.model_grasp_trial.v1'


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _hash(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


class FrozenModelBank:
    """Read exact files once. Never adapt a context or refit between stages."""
    def __init__(self, bindings):
        require(isinstance(bindings, list) and bool(bindings), 'model_bindings_required')
        self._models, self._hashes = {}, {}
        for binding in bindings:
            require(isinstance(binding, dict) and set(binding) == {'id', 'path', 'sha256'}
                    and identifier(binding['id']) and binding['id'] not in self._models,
                    'unique_model_binding_required')
            self._models[binding['id']] = load_frozen_model(binding['path'], binding['sha256'])
            self._hashes[binding['id']] = binding['sha256']

    @property
    def hashes(self):
        return copy.deepcopy(self._hashes)

    def context(self, model_id):
        require(model_id in self._models, 'stage_model_missing')
        return copy.deepcopy(self._models[model_id]['context'])

    def choose(self, stage, state, context):
        return select(self._models[stage['modelId']], state, stage['target'], stage['commands'], context)


def validate_scope(scope, bank):
    require(isinstance(scope, dict) and set(scope) == {
        'schemaVersion', 'trialId', 'hostClockId', 'controllerBootId', 'modelHashes', 'stages',
        'maxCommands', 'maxRequestedMs', 'maxSeconds', 'maxObservationAgeSeconds',
        'maxDecisionAgeSeconds', 'postOffSeconds', 'retentionSeconds'}, 'grasp_scope_fields')
    require(scope['schemaVersion'] == SCHEMA and identifier(scope['trialId'])
            and isinstance(scope['hostClockId'], str) and bool(scope['hostClockId'])
            and isinstance(scope['controllerBootId'], str)
            and re.fullmatch('[0-9a-f]{32}', scope['controllerBootId']) is not None
            and scope['modelHashes'] == bank.hashes, 'grasp_scope_bindings')
    require(type(scope['maxCommands']) is int and 1 <= scope['maxCommands'] <= 12
            and type(scope['maxRequestedMs']) is int and 100 <= scope['maxRequestedMs'] <= 12000,
            'bounded_command_budget_required')
    for key, low, high in [('maxSeconds', 1, 600), ('maxObservationAgeSeconds', .1, 5),
                           ('maxDecisionAgeSeconds', .01, .5), ('retentionSeconds', 1, 10)]:
        require(_number(scope[key]) and low <= scope[key] <= high, 'bounded_timing_required')
    require(type(scope['postOffSeconds']) in (int, float) and scope['postOffSeconds'] == 3.,
            'fixed_post_off_observation_wait_required')
    stages = scope['stages']
    require(isinstance(stages, list) and 1 <= len(stages) <= 4, 'position_then_close_required')
    seen, objects = set(), set()
    for index, stage in enumerate(stages):
        require(isinstance(stage, dict) and set(stage) == {
            'id', 'kind', 'modelId', 'target', 'tolerancePx', 'commands', 'maxCommands'}, 'stage_fields')
        require(identifier(stage['id']) and stage['id'] not in seen, 'unique_stage_required')
        seen.add(stage['id'])
        require(stage['kind'] == ('close' if index == len(stages)-1 else 'position'),
                'position_then_close_required')
        objects.add(bank.context(stage['modelId'])['object'])
        vector(stage['target'])
        tolerance = stage['tolerancePx']
        require(isinstance(tolerance, list) and len(tolerance) == 3
                and all(_number(v) and v > 0 for v in tolerance), 'positive_stage_tolerances_required')
        require(type(stage['maxCommands']) is int and 1 <= stage['maxCommands'] <= 4,
                'bounded_stage_commands_required')
        require(stage['kind'] != 'close' or stage['maxCommands'] == 1, 'single_close_attempt_only')
        commands = stage['commands']
        require(isinstance(commands, list) and bool(commands), 'stage_candidates_required')
        keys = [command_key(c) for c in commands]
        require(len(keys) == len(set(keys)), 'duplicate_candidate')
        for command in commands:
            is_close = command['joint'] == 'gripper' and command['direction'] == 'close'
            require(is_close == (stage['kind'] == 'close'), 'close_only_in_guarded_final_stage')
    require(len(objects) == 1, 'stage_object_identity_changed')
    return copy.deepcopy(scope)


def _pulse(command):
    command_key(command)
    profile = PROFILES[command['joint']]
    sign = profile['directions'][command['direction']][0]
    return dict(command, motor=profile['motor'], expectedPwm=sign*(command['percent']*255//100))


def _reached(state, stage):
    return all(abs(x-y) <= tolerance for x, y, tolerance in
               zip(state, stage['target'], stage['tolerancePx']))


class ModelGraspTrial:
    """One explicitly scoped, single-use sequence; never resumes failed commands.

    observer.start(deadline), observe(stage, after_off, deadline, tick), latest(stage),
    check(), verify_grasp(stage, after_off, deadline, tick), close() are injected.
    observe may wait for a new reviewed frame. latest/check must not block or
    open HTTP deadlines: they are called inside SequencePort's RUN deadline.
    Raw frame/metadata verification is the observation adapter's responsibility.
    The point record must use actual context, never copy a requested model's
    context onto an unverified image. This core does not invent perception.
    """
    def __init__(self, scope, bank, observer, port, store, *, evidence_root, clock=time):
        self.scope = validate_scope(scope, bank)
        require(port.boot_id == scope['controllerBootId'], 'controller_boot_binding_changed')
        self.bank, self.observer, self.port, self.store, self.clock = bank, observer, port, store, clock
        self.evidence_root = Path(evidence_root).resolve(strict=True)
        require(self.evidence_root.is_dir(), 'private_observation_evidence_directory_required')
        self.used = False

    def _deadline(self):
        require(self.clock.monotonic() < self.deadline, 'trial_deadline')

    def _idle_tick(self):
        """Observer waits must call this regularly, outside RUN's HTTP alarm."""
        self._deadline()
        self.observer.check()
        self.port.check_idle()

    def _active_tick(self):
        self._deadline()
        self.observer.check()
        self._deadline()

    def _input_age_limit(self, context):
        identity = context['cameraIdentity']
        policy_age = (identity['observationPolicy']['inputRequestMaximumSecondsBeforeCommand']
                      if identity.get('camera') == 'gripper' else -EXTERNAL_POLICY['preSeconds'][0])
        return min(self.scope['maxObservationAgeSeconds'], policy_age)

    def _observe(self, record, stage, *, after_off=None, history=False):
        self._deadline()
        require(isinstance(record, dict) and set(record) <= {
            'frame', 'annotation', 'context', 'hostClockId', 'sourceEarliest', 'sourceLatest',
            'receivedAt', 'persistedAt', 'stable', 'closeEntryEvidence'}, 'observation_fields')
        require(record['context'] == self.bank.context(stage['modelId']), 'observation_context_changed')
        require(record['hostClockId'] == self.scope['hostClockId'] and record['stable'] is True,
                'observation_clock_or_stability_changed')
        frame = record['frame']
        require(isinstance(frame, dict) and set(frame) == {
            'frameId', 'pixelSha256', 'width', 'height'}
            and isinstance(frame['frameId'], str) and bool(frame['frameId'])
            and _hash(frame['pixelSha256'])
            and [frame['height'], frame['width']] == record['context']['imageShape']
            and all(type(frame[k]) is int and frame[k] > 0 for k in ('width', 'height')),
            'observation_frame_binding_invalid')
        times = [record[k] for k in ('sourceEarliest', 'sourceLatest', 'receivedAt', 'persistedAt')]
        now = self.clock.monotonic()
        require(all(_number(v) for v in times) and self.started <= times[0]
                and times == sorted(times) and times[-1] <= now
                and (history or now-times[0] <= self._input_age_limit(record['context'])),
                'observation_stale_or_noncausal')
        if after_off is not None:
            require(times[0] >= after_off + self.scope['postOffSeconds'], 'post_off_window_not_ready')
        state = point_state(record['annotation'], frame)
        return state

    def _review_evidence(self, proof):
        require(isinstance(proof.get('evidenceFile'), str) and _hash(proof.get('evidenceSha256')),
                'saved_observation_review_required')
        raw = _read(self.evidence_root, proof['evidenceFile'])
        require(len(raw) <= 65536 and sha256(raw) == proof['evidenceSha256'],
                'observation_review_hash_changed')
        require(_decode_json(raw) == {k: v for k, v in proof.items()
                                    if k not in ('evidenceFile', 'evidenceSha256')},
                'observation_review_content_changed')

    def _close_entry(self, record):
        proof = record.get('closeEntryEvidence', {})
        require(isinstance(proof, dict) and set(proof) == {
            'frameId', 'pixelSha256', 'method', 'evidenceFile', 'evidenceSha256', 'objectBetweenJawsVerified',
            'depthVerified', 'clearanceVerified'}
            and proof['frameId'] == record['frame']['frameId']
            and proof['pixelSha256'] == record['frame']['pixelSha256']
            and proof['method'] == 'onsite_observation' and _hash(proof['evidenceSha256'])
            and all(proof[k] is True for k in
                    ('objectBetweenJawsVerified', 'depthVerified', 'clearanceVerified')),
            'close_requires_independent_onsite_evidence')
        self._review_evidence(proof)

    def _receipt(self, receipt, pulse):
        require(isinstance(receipt, dict) and receipt.get('offVerified') is True
                and receipt.get('requestedPulse') == pulse and receipt.get('motor') == pulse['motor'],
                'selected_command_receipt_mismatch')
        times = [receipt.get(k) for k in ('dispatchBefore', 'dispatchAfter', 'offAt')]
        require(all(_number(v) for v in times) and times == sorted(times)
                and times[0] >= self.started and times[-1] <= self.clock.monotonic(),
                'invalid_execution_receipt_timing')
        trace = receipt.get('outputTrace')
        require(isinstance(trace, list) and bool(trace), 'actual_output_trace_required')
        requested, stopped, last_outputs = False, False, {}
        for entry in trace:
            require(type(entry.get('motor')) is int and 1 <= entry['motor'] <= 5
                    and type(entry.get('pwm')) is int, 'actual_output_trace_required')
            if entry['pwm']:
                require(entry['motor'] == pulse['motor']
                        and entry['pwm'] * pulse['expectedPwm'] > 0
                        and abs(entry['pwm']) <= abs(pulse['expectedPwm']), 'unexpected_output_trace')
                requested = True
            elif requested and entry['motor'] == pulse['motor']:
                stopped = True
            last_outputs[entry['motor']] = entry['pwm']
        require(requested and stopped and all(v == 0 for v in last_outputs.values()),
                'actual_motion_and_off_required')
        return times[-1]

    def _retention(self, stage, off_at):
        proof = self.observer.verify_grasp(stage, after_off=off_at, deadline=self.deadline, tick=self._idle_tick)
        self._idle_tick()
        self._deadline()
        require(isinstance(proof, dict) and proof.get('status') in ('held', 'failed', 'unknown'),
                'explicit_retention_outcome_required')
        if proof['status'] == 'held':
            require(proof.get('method') == 'analyst_retention_review'
                    and _hash(proof.get('evidenceSha256'))
                    and all(proof.get(k) is True for k in
                            ('contactVerified', 'retentionVerified', 'supportDistinguished')),
                    'independent_retention_evidence_required')
            self._review_evidence(proof)
            observations = proof.get('observations')
            require(isinstance(observations, list) and len(observations) >= 2,
                    'retention_observation_span_required')
            for observation in observations:
                self._observe(observation, stage, after_off=off_at, history=True)
            self._observe(observations[-1], stage, after_off=off_at)
            require(len({r['frame']['frameId'] for r in observations}) == len(observations)
                    and all(b['sourceEarliest'] > a['sourceLatest']
                            for a, b in zip(observations, observations[1:]))
                    and observations[-1]['sourceEarliest'] - observations[0]['sourceLatest']
                        >= self.scope['retentionSeconds'], 'retention_observation_span_required')
        return copy.deepcopy(proof)

    def run(self, *, confirmed_scope_sha256):
        require(not self.used, 'grasp_trial_single_use')
        require(confirmed_scope_sha256 == plan_digest(self.scope), 'exact_grasp_scope_confirmation_required')
        self.used = True
        self.store.write_json('trial-intent.json', self.scope)
        self.started = self.clock.monotonic()
        self.deadline = self.started + self.scope['maxSeconds']
        report = dict(schemaVersion=SCHEMA, scopeSha256=confirmed_scope_sha256, status='failed',
            stages=[dict(id=s['id'], status='not_started', steps=[]) for s in self.scope['stages']],
            commandAttempts=0, confirmedCommands=0, requestedMs=0, modelSelectedMotorCommands=0,
            errors=[], automaticRetry=False, trainingPerformed=False, controlEligible=False,
            humanMotorSelectionCount=0, observationAssisted=True, fullyAutonomous=False,
            graspVerified=False, liftVerified=False, finalIdleOffVerified=False)
        camera_started, off_at = False, None
        try:
            require(self.port.boot_id == self.scope['controllerBootId'], 'controller_boot_binding_changed')
            self.port.preflight()
            camera_started = True
            self.observer.start(deadline=self.deadline)
            for stage, stage_report in zip(self.scope['stages'], report['stages']):
                stage_report['status'] = 'observing'
                used = 0
                while True:
                    self._deadline()
                    self.port.check_idle()
                    record = self.observer.observe(stage, after_off=off_at, deadline=self.deadline,
                                                   tick=self._idle_tick)
                    self._idle_tick()
                    state = self._observe(record, stage, after_off=off_at)
                    stage_report['lastObservation'] = copy.deepcopy(record)
                    stage_report['lastState'] = state
                    if _reached(state, stage):
                        require(stage['kind'] != 'close', 'closed_geometry_is_not_grasp_proof')
                        stage_report['status'] = 'observed_target_reached'
                        break
                    if used >= stage['maxCommands']:
                        stage_report['status'] = 'budget_reached'
                        report['status'] = 'budget_reached'
                        return report
                    decision = self.bank.choose(stage, state, record['context'])
                    if decision['selected']['kind'] != 'action':
                        stage_report.update(status='abstained', decision=decision)
                        report['status'] = 'abstained'
                        return report
                    if stage['kind'] == 'close':
                        self._close_entry(record)
                    command = decision['selected']['command']
                    pulse = _pulse(command)
                    require(report['commandAttempts'] < self.scope['maxCommands']
                            and report['requestedMs'] + command['durationMs'] <= self.scope['maxRequestedMs'],
                            'trial_command_budget_reached')
                    self._deadline()
                    used += 1
                    report['commandAttempts'] += 1
                    report['requestedMs'] += command['durationMs']
                    step = dict(index=report['commandAttempts'], status='not_dispatched',
                                observation=copy.deepcopy(record), decision=decision, pulse=pulse)
                    stage_report['steps'].append(step)
                    self.store.write_json(f"step-{step['index']:03d}-intent.json", step)
                    guard_calls = 0

                    def guard():
                        nonlocal guard_calls
                        guard_calls += 1
                        require(guard_calls == 1, 'single_dispatch_guard_required')
                        self._deadline()
                        require(self.port.boot_id == self.scope['controllerBootId'],
                                'controller_boot_binding_changed')
                        self.observer.check()
                        current = self.observer.latest(stage)
                        current_state = self._observe(current, stage, after_off=off_at)
                        require(current['sourceLatest'] >= record['sourceLatest'], 'observation_regressed')
                        require(not _reached(current_state, stage), 'target_reached_before_dispatch')
                        fresh = self.bank.choose(stage, current_state, current['context'])
                        require(fresh['selected']['kind'] == 'action'
                                and fresh['selected']['command'] == command, 'model_choice_changed_no_dispatch')
                        if stage['kind'] == 'close':
                            self._close_entry(current)
                        at = self.clock.monotonic()
                        self.store.write_json(f"step-{step['index']:03d}-dispatch-decision.json",
                            dict(at=at, observation=current, decision=fresh))
                        self.observer.check()
                        self._deadline()
                        now = self.clock.monotonic()
                        require(now-at <= self.scope['maxDecisionAgeSeconds']
                                and now-current['sourceEarliest'] <= self._input_age_limit(current['context']),
                                'dispatch_decision_expired')
                        require(current['persistedAt'] <= now-.1, 'input_not_persisted_before_dispatch')
                        require(now + command['durationMs']/1000 < self.deadline,
                                'insufficient_trial_time_for_command')
                        step['dispatchObservation'] = copy.deepcopy(current)

                    try:
                        receipt = self.port.execute(pulse, self._active_tick, before_run=guard)
                        require(guard_calls == 1, 'dispatch_guard_not_called')
                        off_at = self._receipt(receipt, pulse)
                        step.update(status='executed', receipt=copy.deepcopy(receipt))
                        report['confirmedCommands'] += 1
                        report['modelSelectedMotorCommands'] += 1
                    except BaseException:
                        step['status'] = 'outcome_unknown' if self.port.may_have_run else 'not_dispatched'
                        raise
                    finally:
                        self.store.write_json(f"step-{step['index']:03d}-result.json", step)
                    if stage['kind'] == 'close':
                        # Do not repeatedly close against possible contact to
                        # force a pixel target. Inspect retention after one close.
                        proof = self._retention(stage, off_at)
                        stage_report.update(status='close_observed', retention=proof)
                        report['graspVerified'] = proof['status'] == 'held'
                        report['status'] = 'held' if report['graspVerified'] else 'grasp_' + proof['status']
                        return report
        except (Exception, KeyboardInterrupt) as exc:
            report['errors'].append(dict(type=type(exc).__name__,
                reason=str(exc) if isinstance(exc, ValueError) else 'trial_interrupted_or_failed'))
            report['status'] = 'failed'
            for stage_report in report['stages']:
                if stage_report['status'] == 'observing':
                    stage_report['status'] = 'failed'
        finally:
            try:
                self.port.check_idle()
            except (Exception, KeyboardInterrupt):
                report['errors'].append(dict(type='cleanup', reason='final_idle_read_failed'))
            finally:
                try:
                    report['finalIdleOffVerified'] = self.port.finish() is True
                except (Exception, KeyboardInterrupt):
                    report['errors'].append(dict(type='cleanup', reason='off_not_verified'))
            if camera_started:
                try:
                    recording = self.observer.close()
                    require(isinstance(recording, dict) and recording.get('status') == 'completed',
                            'observation_finalization_failed')
                    report['recording'] = recording
                except (Exception, KeyboardInterrupt):
                    report['errors'].append(dict(type='cleanup', reason='observation_finalization_failed'))
            if not report['finalIdleOffVerified'] or report['errors']:
                report['status'] = 'failed'
                report['graspVerified'] = False
            self.store.write_json('result.json', report)
        return report
