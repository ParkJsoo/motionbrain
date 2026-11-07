"""Synthetic closed-loop integration; never a physical grasp-performance test."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from learning import relative_action_learning as dynamics
from learning.model_grasp_trial import FrozenModelBank, ModelGraspTrial, SCHEMA
from learning.relative_model_bundle import sha256
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort
from tests.test_supervised_transport import Clock, Store, Controller

POSITION = dict(joint='base', direction='left', percent=40, durationMs=350)
CLOSE = dict(joint='gripper', direction='close', percent=40, durationMs=350)


def context(kind):
    return dict(physicalView='synthetic-fixed', object='synthetic-block', poseRegime=kind,
                landmarks='synthetic-object-and-jaws', cameraIdentity={'deviceId': 'synthetic'},
                imageShape=[200, 200], stateFields=dynamics.STATE)


def fit_model(kind, command, delta):
    rows = []
    for i, state in enumerate(([-5., -5., 10.], [20., 20., 30.], [40., 40., 50.])):
        rows.append(dict(sessionId=f'{kind}-{i}', captureGroup=f'{kind}-train', context=context(kind),
            command=command, before=state, after=[v+d for v, d in zip(state, delta)],
            episode=['boot', f'{kind}-{i}'], frameIds=[f'{kind}-{i}-pre', f'{kind}-{i}-post']))
    return dynamics.fit(rows)


class Observer:
    def __init__(self, clock, evidence_root):
        self.clock = clock
        self.evidence_root = evidence_root
        self.state = [20., 20., 30.]
        self.cache = None
        self.frames = 0
        self.closed = False
        self.proof_status = 'held'
        self.with_close_proof = True
        self.bad_context = False
        self.unsettled = False
        self.guard_shift = False
        self.retention_bad = False

    def save_review(self, proof, filename):
        declaration = {k: v for k, v in proof.items() if k not in ('evidenceFile', 'evidenceSha256')}
        raw = json.dumps(declaration).encode()
        (self.evidence_root / filename).write_bytes(raw)
        return dict(declaration, evidenceFile=filename, evidenceSha256=sha256(raw))

    def start(self, *, deadline):
        self.deadline = deadline

    def check(self):
        if self.unsettled:
            raise ValueError('observation_lost')

    def frame(self, stage):
        self.clock.sleep(.1)
        self.frames += 1
        x, y, opening = self.state
        frame = dict(frameId=f'new-{self.frames}', pixelSha256='b' * 64, width=200, height=200)
        record = dict(frame=frame, context=context(stage['modelId']), hostClockId='host',
            sourceEarliest=self.clock.now-.03, sourceLatest=self.clock.now-.02,
            receivedAt=self.clock.now-.01, persistedAt=self.clock.now, stable=True,
            annotation=dict(frameId=frame['frameId'], pixelSha256=frame['pixelSha256'],
                accepted=True, method='analyst_points', inputFrameOnly=True,
                points=dict(object=[100+x, 100+y], jawLeft=[100-opening/2, 100],
                            jawRight=[100+opening/2, 100])))
        if self.bad_context:
            record['context']['poseRegime'] = 'actual-other-pose'
        if self.with_close_proof:
            record['closeEntryEvidence'] = self.save_review(dict(
                frameId=frame['frameId'], pixelSha256=frame['pixelSha256'],
                method='onsite_observation', objectBetweenJawsVerified=True,
                depthVerified=True, clearanceVerified=True), f'close-entry-{self.frames}.json')
        return record

    def observe(self, stage, *, after_off, deadline, tick):
        if after_off is not None:
            self.clock.sleep(max(0, after_off+3.1-self.clock.now))
        self.cache = self.frame(stage)
        tick()
        return copy.deepcopy(self.cache)

    def latest(self, stage):
        if self.guard_shift:
            result = copy.deepcopy(self.cache)
            result['annotation']['points']['object'] = [190, 190]
            return result
        return copy.deepcopy(self.cache)

    def verify_grasp(self, stage, *, after_off, deadline, tick):
        if self.proof_status != 'held':
            return dict(status=self.proof_status)
        self.clock.sleep(max(0, after_off+3.1-self.clock.now))
        first = self.frame(stage)
        self.clock.sleep(1.1)
        second = self.frame(stage)
        tick()
        if self.retention_bad:
            second = copy.deepcopy(first)
        return self.save_review(dict(status='held', method='analyst_retention_review',
                    contactVerified=True, retentionVerified=True, supportDistinguished=True,
                    observations=[first, second]), 'retention.json')

    def close(self):
        self.closed = True
        return dict(status='completed')


class Port:
    def __init__(self, clock, observer):
        self.clock, self.observer = clock, observer
        self.boot_id = 'a' * 32
        self.may_have_run = False
        self.calls = []
        self.off = True
        self.finish_calls = 0
        self.loss = False
        self.wrong_receipt = False
        self.intent_delay = 0
        self.move = True

    def preflight(self):
        pass

    def check_idle(self):
        pass

    def execute(self, pulse, tick, *, before_run):
        self.may_have_run = False
        tick()
        self.clock.sleep(.11 + self.intent_delay)
        before_run()
        self.may_have_run = True
        self.calls.append(pulse)
        before = self.clock.now
        self.clock.sleep(.4)
        if self.loss:
            raise TimeoutError('lost response')
        if self.move:
            delta = [-10., -10., 0.] if pulse['joint'] == 'base' else [0., 0., -10.]
            self.observer.state = [v+d for v, d in zip(self.observer.state, delta)]
        receipt = dict(offVerified=True, motor=pulse['motor'], requestedPulse=copy.deepcopy(pulse),
                       dispatchBefore=before, dispatchAfter=before+.01, offAt=self.clock.now,
                       outputTrace=[dict(motor=pulse['motor'], pwm=pulse['expectedPwm']),
                                    dict(motor=pulse['motor'], pwm=0)])
        if self.wrong_receipt:
            receipt['requestedPulse']['durationMs'] += 1
        return receipt

    def finish(self):
        self.finish_calls += 1
        return self.off


class ModelGraspTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        bindings = []
        for name, command, delta in [('position', POSITION, [-10., -10., 0.]),
                                     ('close', CLOSE, [0., 0., -10.])]:
            path = Path(self.temp.name) / (name + '.json')
            raw = json.dumps(fit_model(name, command, delta)).encode(); path.write_bytes(raw)
            bindings.append(dict(id=name, path=path, sha256=sha256(raw)))
        self.bank = FrozenModelBank(bindings)
        self.clock = Clock()
        self.observer = Observer(self.clock, Path(self.temp.name))
        self.port = Port(self.clock, self.observer)
        self.store = Store()
        self.scope = dict(schemaVersion=SCHEMA, trialId='synthetic', hostClockId='host',
            controllerBootId='a' * 32, modelHashes=self.bank.hashes,
            maxCommands=4, maxRequestedMs=1400, maxSeconds=60., maxObservationAgeSeconds=2.,
            maxDecisionAgeSeconds=.25, postOffSeconds=3., retentionSeconds=1.,
            stages=[dict(id='position', kind='position', modelId='position', target=[0., 0., 30.],
                         tolerancePx=[.1, .1, .1], commands=[POSITION], maxCommands=3),
                    dict(id='close', kind='close', modelId='close', target=[0., 0., 20.],
                         tolerancePx=[1., 1., 1.], commands=[CLOSE], maxCommands=1)])

    def trial(self):
        return ModelGraspTrial(self.scope, self.bank, self.observer, self.port, self.store,
                               evidence_root=self.temp.name, clock=self.clock)

    def run_trial(self):
        return self.trial().run(confirmed_scope_sha256=plan_digest(self.scope))

    def test_reobserved_two_position_steps_then_one_model_selected_close(self):
        report = self.run_trial()
        self.assertEqual(report['status'], 'held')
        self.assertEqual(report['commandAttempts'], 3)
        self.assertEqual(report['modelSelectedMotorCommands'], 3)
        self.assertEqual(report['stages'][0]['lastState'], [0., 0., 30.])
        self.assertTrue(report['graspVerified'])
        self.assertFalse(report['fullyAutonomous'])
        self.assertFalse(report['liftVerified'])
        self.assertTrue(report['finalIdleOffVerified'])
        self.assertTrue(self.observer.closed)
        self.assertEqual(self.store.rows['result.json'], report)

    def close_only_scope(self):
        self.scope['stages'] = [dict(self.scope['stages'][-1], target=[20., 20., 20.])]
        self.scope.update(maxCommands=1, maxRequestedMs=350)

    def test_aligned_close_only_uses_one_model_selected_command_and_retention(self):
        self.close_only_scope()
        report = self.run_trial()
        self.assertEqual(report['status'], 'held')
        self.assertEqual(report['modelSelectedMotorCommands'], 1)
        self.assertEqual(len(self.port.calls), 1)
        self.assertEqual(self.port.calls[0]['joint'], 'gripper')
        self.assertTrue(report['graspVerified'])
        self.assertTrue(report['finalIdleOffVerified'])
        self.assertFalse(report['liftVerified'])

    def test_contact_only_close_does_not_claim_retention_or_call_retention_reviewer(self):
        from learning.model_grasp_trial import CONTACT_SCHEMA
        self.close_only_scope()
        self.scope['schemaVersion'] = CONTACT_SCHEMA
        self.observer.verify_grasp = lambda *a, **k: self.fail('retention must be reviewed separately')
        report = self.run_trial()
        self.assertEqual(report['status'], 'closed_off')
        self.assertEqual(report['modelSelectedMotorCommands'], 1)
        self.assertTrue(report['finalIdleOffVerified'])
        self.assertFalse(report['graspVerified'])
        self.assertFalse(report['liftVerified'])

    def test_contact_only_rejects_positioning_and_multiple_commands(self):
        from learning.model_grasp_trial import CONTACT_SCHEMA
        self.scope['schemaVersion'] = CONTACT_SCHEMA
        with self.assertRaisesRegex(ValueError, 'contact_trial_single_close_required'):
            self.trial()

    def test_close_only_rejects_missing_depth_context_and_outside_range_without_run(self):
        for case in ('missing_proof', 'context', 'outside_range', 'stale'):
            with self.subTest(case=case):
                self.setUp(); self.close_only_scope()
                if case == 'missing_proof': self.observer.with_close_proof = False
                elif case == 'context': self.observer.bad_context = True
                elif case == 'outside_range': self.observer.state = [90., 90., 30.]
                else: self.port.intent_delay = 3.
                report = self.run_trial()
                self.assertEqual(self.port.calls, [])
                self.assertFalse(report['graspVerified'])
                self.assertTrue(report['finalIdleOffVerified'])

    def test_registered_annotation_cannot_enter_existing_analyst_context(self):
        self.close_only_scope()
        original_frame = self.observer.frame

        def wrong_annotation(stage):
            record = original_frame(stage)
            record['annotation'].update(method='registered_template_points',
                registrationSha256='c'*64, parametersSha256='d'*64,
                referenceImageSha256='e'*64,
                landmarkContract='registered-template-points-v1:'+'c'*64)
            return record

        self.observer.frame = wrong_annotation
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'registered_annotation_context_changed')
        self.assertEqual(self.port.calls, [])
        self.assertEqual(report['modelSelectedMotorCommands'], 0)
        self.assertTrue(report['finalIdleOffVerified'])

    def test_analyst_annotation_cannot_enter_registered_model_context(self):
        self.close_only_scope()
        contract = 'registered-template-points-v1:'+'c'*64
        self.bank._models['close']['context']['landmarks'] = contract
        original_frame = self.observer.frame

        def wrong_annotation(stage):
            record = original_frame(stage)
            record['context']['landmarks'] = contract
            return record

        self.observer.frame = wrong_annotation
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'registered_context_requires_registered_annotation')
        self.assertEqual(self.port.calls, [])
        self.assertEqual(report['modelSelectedMotorCommands'], 0)
        self.assertTrue(report['finalIdleOffVerified'])

    def test_matching_registered_context_and_annotation_preserve_close_path(self):
        self.close_only_scope()
        contract = 'registered-template-points-v1:'+'c'*64
        self.bank._models['close']['context']['landmarks'] = contract
        original_frame = self.observer.frame

        def registered_frame(stage):
            record = original_frame(stage)
            record['context']['landmarks'] = contract
            record['annotation'].update(method='registered_template_points',
                registrationSha256='c'*64, parametersSha256='d'*64,
                referenceImageSha256='e'*64, landmarkContract=contract)
            return record

        self.observer.frame = registered_frame
        report = self.run_trial()
        self.assertEqual(report['status'], 'held')
        self.assertEqual(report['modelSelectedMotorCommands'], 1)
        self.assertEqual(len(self.port.calls), 1)
        self.assertTrue(report['finalIdleOffVerified'])

    def test_close_only_unknown_results_never_retry_or_claim_success(self):
        for case in ('lost_response', 'retention_unknown', 'off_unverified'):
            with self.subTest(case=case):
                self.setUp(); self.close_only_scope()
                if case == 'lost_response': self.port.loss = True
                elif case == 'retention_unknown': self.observer.proof_status = 'unknown'
                else: self.port.off = False
                report = self.run_trial()
                self.assertEqual(len(self.port.calls), 1)
                self.assertFalse(report['graspVerified'])
                self.assertFalse(report['automaticRetry'])

    def test_close_only_scope_still_requires_one_close_and_nonempty_stages(self):
        for case in ('empty', 'position_only', 'two_closes', 'open_candidate'):
            with self.subTest(case=case):
                self.setUp(); self.close_only_scope()
                if case == 'empty': self.scope['stages'] = []
                elif case == 'position_only': self.scope['stages'][0]['kind'] = 'position'
                elif case == 'two_closes': self.scope['stages'][0]['maxCommands'] = 2
                else: self.scope['stages'][0]['commands'] = [dict(CLOSE, direction='open')]
                with self.assertRaises(ValueError): self.trial()
                self.assertEqual(self.port.calls, [])

    def test_close_only_existing_geometry_is_not_grasp_evidence(self):
        self.close_only_scope(); self.observer.state = [20., 20., 20.]
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'closed_geometry_is_not_grasp_proof')
        self.assertEqual(self.port.calls, [])
        self.assertFalse(report['graspVerified'])

    def test_sequence_transport_integration_preserves_stop_and_off(self):
        device = Controller(self.clock); device.boot = self.scope['controllerBootId']
        port = SequencePort('http://device', 'test-token', device.boot, self.store, clock=self.clock)
        port.opener = device
        def move(controller):
            delta = [-10., -10., 0.] if controller.queue[0]['joint'] == 'base' else [0., 0., -10.]
            self.observer.state = [v+d for v, d in zip(self.observer.state, delta)]
        device.after_run = move
        self.port = port
        report = self.run_trial()
        self.assertEqual(report['status'], 'held')
        self.assertEqual(sum(p == {'action': 'run'} for _, p in device.requests), 3)
        self.assertTrue(report['finalIdleOffVerified'])
        self.assertEqual(device.speeds, {i: 0 for i in range(1, 6)})
        self.assertEqual(device.queue, [])

    def test_prediction_does_not_advance_stage_when_actual_state_does_not_move(self):
        self.port.move = False
        report = self.run_trial()
        self.assertEqual(report['status'], 'budget_reached')
        self.assertEqual(len(self.port.calls), 3)
        self.assertTrue(all(p['joint'] == 'base' for p in self.port.calls))
        self.assertEqual(report['stages'][1]['status'], 'not_started')

    def test_stop_winner_and_oos_abstain_without_execution(self):
        for state in ([3., 3., 30.], [90., 90., 30.]):
            with self.subTest(state=state):
                self.setUp(); self.observer.state = list(state)
                report = self.run_trial()
                self.assertEqual(report['status'], 'abstained')
                self.assertEqual(self.port.calls, [])
                self.assertFalse(report['graspVerified'])

    def test_context_change_is_rejected_instead_of_relabelled(self):
        self.observer.bad_context = True
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'observation_context_changed')
        self.assertEqual(self.port.calls, [])

    def test_missing_close_entry_proof_prevents_close(self):
        self.observer.with_close_proof = False
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'close_requires_independent_onsite_evidence')
        self.assertEqual(len(self.port.calls), 2)
        self.assertFalse(report['graspVerified'])

    def test_guard_changed_choice_and_stale_observation_prevent_run(self):
        for change, reason in [('guard_shift', 'model_choice_changed_no_dispatch'),
                               ('intent_delay', 'observation_stale_or_noncausal')]:
            with self.subTest(change=change):
                self.setUp()
                if change == 'guard_shift': self.observer.guard_shift = True
                else: self.port.intent_delay = 3.
                report = self.run_trial()
                self.assertEqual(report['errors'][0]['reason'], reason)
                self.assertEqual(self.port.calls, [])
                self.assertEqual(report['commandAttempts'], 1)

    def test_decision_persistence_delay_blocks_run(self):
        write = self.store.write_json
        def delayed(name, value):
            write(name, value)
            if name.endswith('dispatch-decision.json'):
                self.clock.sleep(.3)
        self.store.write_json = delayed
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'dispatch_decision_expired')
        self.assertEqual(self.port.calls, [])

    def test_lost_response_is_unknown_and_never_retried(self):
        self.port.loss = True
        report = self.run_trial()
        self.assertEqual(len(self.port.calls), 1)
        self.assertEqual(report['stages'][0]['steps'][0]['status'], 'outcome_unknown')
        self.assertFalse(report['automaticRetry'])
        self.assertFalse(report['graspVerified'])

    def test_wrong_receipt_and_failed_cleanup_cannot_report_success(self):
        self.port.wrong_receipt = True
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'selected_command_receipt_mismatch')
        self.assertEqual(len(self.port.calls), 1)
        self.setUp(); self.port.off = False
        report = self.run_trial()
        self.assertEqual(report['status'], 'failed')
        self.assertFalse(report['graspVerified'])

    def test_unknown_retention_does_not_close_again_or_claim_success(self):
        self.observer.proof_status = 'unknown'
        report = self.run_trial()
        self.assertEqual(report['status'], 'grasp_unknown')
        self.assertEqual(sum(p['joint'] == 'gripper' for p in self.port.calls), 1)
        self.assertFalse(report['graspVerified'])

    def test_duplicate_retention_frame_is_not_evidence(self):
        self.observer.retention_bad = True
        report = self.run_trial()
        self.assertEqual(report['status'], 'failed')
        self.assertFalse(report['graspVerified'])

    def test_retention_history_may_span_longer_than_current_input_age(self):
        verify = self.observer.verify_grasp
        def longer(stage, **kwargs):
            proof = verify(stage, **kwargs)
            # The first image is historical proof; the final image is current.
            self.clock.sleep(2.)
            proof['observations'][-1] = self.observer.frame(stage)
            return self.observer.save_review(proof, 'retention.json')
        self.observer.verify_grasp = longer
        self.scope['retentionSeconds'] = 3.
        report = self.run_trial()
        self.assertEqual(report['status'], 'held')

    def test_unexpected_output_during_retention_invalidates_held(self):
        device = Controller(self.clock); device.boot = self.scope['controllerBootId']
        port = SequencePort('http://device', 'test-token', device.boot, self.store, clock=self.clock)
        port.opener = device
        def move(controller):
            delta = [-10., -10., 0.] if controller.queue[0]['joint'] == 'base' else [0., 0., -10.]
            self.observer.state = [v+d for v, d in zip(self.observer.state, delta)]
        device.after_run = move
        verify = self.observer.verify_grasp
        def interfered(stage, **kwargs):
            proof = verify(stage, **kwargs)
            device.append(5, 50); device.append(5, 0)
            return proof
        self.observer.verify_grasp = interfered
        self.port = port
        report = self.run_trial()
        self.assertEqual(report['status'], 'failed')
        self.assertFalse(report['graspVerified'])
        self.assertEqual(sum(p == {'action': 'run'} for _, p in device.requests), 3)

    def test_retention_last_frame_must_still_be_current(self):
        verify = self.observer.verify_grasp
        def old(stage, **kwargs):
            proof = verify(stage, **kwargs)
            self.clock.sleep(3.)
            return proof
        self.observer.verify_grasp = old
        report = self.run_trial()
        self.assertEqual(report['status'], 'failed')
        self.assertFalse(report['graspVerified'])

    def test_scope_confirmation_single_use_and_no_hidden_model_mutation(self):
        trial = self.trial()
        with self.assertRaisesRegex(ValueError, 'confirmation'):
            trial.run(confirmed_scope_sha256='0' * 64)
        self.assertFalse(self.store.rows)
        trial.run(confirmed_scope_sha256=plan_digest(self.scope))
        with self.assertRaisesRegex(ValueError, 'single_use'):
            trial.run(confirmed_scope_sha256=plan_digest(self.scope))

    def test_scope_rejects_close_in_position_stage_multi_close_and_hash_change(self):
        for mutate in [lambda s: s['stages'][0].update(commands=[CLOSE]),
                       lambda s: s['stages'][1].update(maxCommands=2),
                       lambda s: s['modelHashes'].update(position='0' * 64)]:
            with self.subTest(mutate=mutate):
                original = copy.deepcopy(self.scope); mutate(self.scope)
                with self.assertRaises(ValueError): self.trial()
                self.scope = original

    def test_review_hash_and_content_are_verified_before_closing(self):
        save = self.observer.save_review
        def changed(proof, filename):
            result = save(proof, filename)
            if filename.startswith('close-entry'):
                (Path(self.temp.name)/filename).write_bytes(b'changed evidence')
            return result
        self.observer.save_review = changed
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'observation_review_hash_changed')
        self.assertTrue(all(p['joint'] != 'gripper' for p in self.port.calls))

    def test_input_must_be_persisted_before_dispatch(self):
        latest = self.observer.latest
        def future_saved(stage):
            record = latest(stage)
            record['persistedAt'] = self.clock.monotonic()
            return record
        self.observer.latest = future_saved
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'input_not_persisted_before_dispatch')
        self.assertEqual(self.port.calls, [])

    def test_final_guard_keeps_camera_age_limit_after_decision_storage(self):
        self.scope['maxObservationAgeSeconds'] = 5.
        self.scope['maxDecisionAgeSeconds'] = .5
        frame = self.observer.frame
        def old_but_initially_eligible(stage):
            record = frame(stage)
            self.clock.sleep(2.68)
            return record
        self.observer.frame = old_but_initially_eligible
        write = self.store.write_json
        def slow(name, value):
            write(name, value)
            if name.endswith('dispatch-decision.json'):
                self.clock.sleep(.4)
        self.store.write_json = slow
        report = self.run_trial()
        self.assertEqual(report['errors'][0]['reason'], 'dispatch_decision_expired')
        self.assertEqual(self.port.calls, [])

    def test_active_deadline_interrupt_uses_cleanup_without_retry(self):
        device = Controller(self.clock); device.boot = self.scope['controllerBootId']
        port = SequencePort('http://device', 'test-token', device.boot, self.store, clock=self.clock)
        port.opener = device
        def interrupted():
            if device.running:
                self.clock.now = 1000.
        self.observer.check = interrupted
        self.port = port
        report = self.run_trial()
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(report['errors'][0]['reason'], 'trial_deadline')
        self.assertEqual(sum(p == {'action': 'run'} for _, p in device.requests), 1)
        self.assertEqual(report['stages'][0]['steps'][0]['status'], 'outcome_unknown')
        self.assertTrue(report['finalIdleOffVerified'])


if __name__ == '__main__':
    unittest.main()
