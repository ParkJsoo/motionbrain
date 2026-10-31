import copy
import unittest
from unittest.mock import Mock

import numpy as np

from learning.joint_action_selection import OfflineActionSelector, canonical_sha256
from learning.joint_goal_trial import SingleGoalTrial, pre_action_state, validate_scope, window, retain_queued_candidate
from tests.test_joint_action_selection import fixture


class Clock:
    def __init__(self):
        self.now = 100.
        self.camera = None
    def monotonic(self): return self.now
    def sleep(self, seconds):
        assert 0 < seconds <= 1
        self.now += seconds
        if self.camera: self.camera.pump()


class Camera:
    def __init__(self, clock):
        self.clock = clock; clock.camera = self
        self.first_source_at = None; self.history = []; self.started = False
        self.value = 6.; self.fault = None; self.closed = False
    def start(self): self.started = True; self.first_source_at = self.clock.now; self.pump()
    def pump(self):
        if not self.started: return
        while self.first_source_at+len(self.history)*.1 <= self.clock.now:
            t = self.first_source_at+len(self.history)*.1
            self.history.append(dict(sourceAt=t, receivedAt=t, angleDeg=self.value,
                                     valid=True, stable=t-self.first_source_at >= .8))
    def check(self):
        self.pump()
        if self.fault: raise ValueError(self.fault)
        if not self.history: raise ValueError('camera_not_live')
    def samples(self): return copy.deepcopy(self.history)
    def finish(self): self.closed = True


class Store:
    def __init__(self): self.data = {}; self.hook = lambda name: None
    def write_json(self, name, value):
        self.hook(name)
        if name in self.data: raise ValueError('existing_artifact')
        self.data[name] = copy.deepcopy(value)


class Port:
    def __init__(self, clock, camera):
        self.clock, self.camera = clock, camera
        self.executions = 0; self.runs = 0; self.finalized = False
        self.before_guard = lambda: None; self.fail_run = False; self.final_ok = True
    def preflight(self): pass
    def execute(self, pulse, guard, tick):
        self.executions += 1; self.before_guard()
        at = guard(); self.runs += 1
        if self.fail_run: raise TimeoutError('run_response_unknown')
        self.camera.value += 4 if pulse['direction']=='up' else -4
        self.clock.sleep(pulse['durationMs']/1000)
        tick()
        return dict(dispatchAt=at, offAt=self.clock.now, offVerified=True)
    def finalize(self): self.finalized = True; return self.final_ok


def setup():
    artifact, dataset = fixture()
    # Nonzero train velocity span avoids a zero-width synthetic support range.
    for i, row in enumerate(dataset['rows']): row['x'][1] = .02 if i%2 else -.02
    x = np.array([r['x'] for r in dataset['rows'] if r['split']=='train'])
    scale = x.std(axis=0); scale[scale < 1e-9] = 1
    artifact['selectedModel'].update(mean=x.mean(axis=0).tolist(), scale=scale.tolist())
    artifact['datasetSha256'] = canonical_sha256(dataset)
    selector = OfflineActionSelector(artifact, dataset)
    scope = dict(schemaVersion='motionbrain.single_m2_goal_trial.v1', trialId='synthetic-trial',
                 goalDeg=10., toleranceDeg=1., modelSha256=selector.model_sha256,
                 datasetSha256=selector.dataset_sha256, maxMotorCommands=1,
                 captureSeconds=30, maxFrames=320, pwmMagnitude=160, durationsMs=[700, 900])
    clock = Clock(); camera = Camera(clock); port = Port(clock, camera); store = Store()
    trial = SingleGoalTrial(scope, selector, camera, port, store, clock=clock)
    return trial, scope, clock, camera, port, store


class GoalTrialTests(unittest.TestCase):
    def setUp(self):
        self.trial, self.scope, self.clock, self.camera, self.port, self.store = setup()
    def run_trial(self): return self.trial.run(confirmed_scope_sha256=canonical_sha256(self.scope))

    def test_one_pulse_and_fixed_horizon_measurement(self):
        r = self.run_trial()
        self.assertEqual(r['status'], 'measured', r['errors'])
        self.assertEqual(self.port.runs, 1)
        self.assertTrue(r['measurement']['goalReached'])
        self.assertAlmostEqual(r['measurement']['errorReductionDeg'], 4.)
        self.assertTrue(self.port.finalized); self.assertTrue(self.camera.closed)
        self.assertFalse(r['trainingPerformed']); self.assertFalse(r['controlEligible'])
        self.assertTrue(r['scopeClosed'])

    def test_confirmation_and_changed_scope_fail_before_any_io(self):
        self.port.preflight = Mock()
        with self.assertRaises(ValueError): self.trial.run(confirmed_scope_sha256='wrong')
        self.port.preflight.assert_not_called(); self.assertFalse(self.camera.started)
        self.assertEqual(self.store.data, {})
        for key, value in [('maxMotorCommands', 2), ('pwmMagnitude', 180),
                           ('captureSeconds', 31), ('goalDeg', float('nan')),
                           ('durationsMs', [550, 900]), ('maxMotorCommands', True)]:
            scope = copy.deepcopy(self.scope); scope[key] = value
            with self.assertRaises(ValueError): validate_scope(scope)

    def test_single_use_even_after_failure(self):
        self.port.fail_run = True; self.run_trial()
        with self.assertRaises(ValueError): self.run_trial()
        self.assertEqual(self.port.runs, 1)

    def test_hold_and_abstain_never_invoke_executor(self):
        for angle, status in [(9.5, 'abstain'), (20., 'abstain'), (8.5, 'hold')]:
            # Synthetic training support ends at 8.53, unlike the physical model.
            trial, scope, clock, camera, port, store = setup(); camera.value = angle
            r = trial.run(confirmed_scope_sha256=canonical_sha256(scope))
            self.assertEqual(r['status'], status, r['errors']); self.assertEqual(port.executions, 0)

    def test_candidate_change_after_queue_preparation_prevents_run(self):
        def change():
            self.camera.value = 8.5
            self.clock.sleep(1.)
        self.port.before_guard = change
        r = self.run_trial(); self.assertEqual(r['status'], 'failed')
        self.assertEqual(self.port.runs, 0); self.assertTrue(self.port.finalized)

    def test_rank_flip_keeps_one_supported_improving_queued_pulse(self):
        select = self.trial.selector.select
        calls = 0
        def reorder(*args, **kwargs):
            nonlocal calls
            calls += 1
            decision = select(*args, **kwargs)
            if calls == 2:
                alternative = next(c for c in decision['candidates']
                                   if c['direction']=='up' and c['durationMs']==900)
                alternative.update(predictedEndDeg=10.1, predictedGoalErrorDeg=.1)
                decision['selected'] = copy.deepcopy(alternative)
            return decision
        self.trial.selector.select = reorder
        r = self.run_trial()
        self.assertEqual(r['status'], 'measured', r['errors'])
        decision = self.store.data['dispatch-decision.json']['decision']
        self.assertEqual(decision['selected']['durationMs'], 700)
        self.assertEqual(decision['bestCandidateAtDispatch']['durationMs'], 900)
        self.assertEqual(self.port.runs, 1)
        self.assertAlmostEqual(r['measurement']['predictedEndDeg'], 10.2)

    def test_queued_pulse_that_loses_support_or_improvement_still_rejected(self):
        decision = self.trial.selector.select(dict(angleDeg=6., velocityDegS=0.,valid=True,settled=True),10.)
        for change in ({'eligible':False}, {'predictedGoalErrorDeg':4.},
                       {'predictedGoalErrorDeg':5.}, {'predictedGoalErrorDeg':float('nan')}):
            d = copy.deepcopy(decision)
            next(c for c in d['candidates'] if c['direction']=='up' and c['durationMs']==700).update(change)
            with self.assertRaises(ValueError): retain_queued_candidate(d, ('up',700))

    def test_hold_or_abstain_cannot_keep_a_queued_pulse(self):
        for status in ('hold','abstain'):
            with self.assertRaises(ValueError): retain_queued_candidate({'status':status}, ('up',700))

    def test_camera_fault_before_run_is_not_retried(self):
        self.port.before_guard = lambda: setattr(self.camera, 'fault', 'tracking_lost')
        r = self.run_trial(); self.assertEqual(r['status'], 'failed')
        self.assertEqual(self.port.runs, 0); self.assertEqual(self.port.executions, 1)

    def test_newest_unsettled_frame_blocks_run_despite_stable_feature_history(self):
        def wobble():
            self.camera.history.append(dict(sourceAt=self.clock.now, receivedAt=self.clock.now,
                                            angleDeg=6., valid=True, stable=False))
        self.port.before_guard = wobble
        r = self.run_trial()
        self.assertEqual(r['status'], 'failed'); self.assertEqual(self.port.runs, 0)
        self.assertIn('latest_observation_not_settled', str(r['errors']))

    def test_slow_dispatch_storage_prevents_run(self):
        self.store.hook = lambda name: self.clock.sleep(.3) if name=='dispatch-decision.json' else None
        r = self.run_trial(); self.assertEqual(self.port.runs, 0)
        self.assertIn('dispatch_decision_expired', str(r['errors']))

    def test_latest_frame_loses_settling_during_intent_storage(self):
        def hook(name):
            if name == 'dispatch-decision.json':
                self.camera.history.append(dict(sourceAt=self.clock.now, receivedAt=self.clock.now,
                                                angleDeg=6., valid=True, stable=False))
        self.store.hook = hook
        r = self.run_trial()
        self.assertEqual(self.port.runs, 0)
        self.assertIn('latest_observation_not_settled', str(r['errors']))

    def test_lost_run_response_keeps_unknown_and_finalizes(self):
        self.port.fail_run = True; r = self.run_trial()
        self.assertEqual(r['status'], 'failed'); self.assertIsNone(r['measurement'])
        self.assertEqual(self.port.runs, 1); self.assertTrue(r['finalIdleOffVerified'])

    def test_interruption_still_finalizes_before_camera_finish(self):
        self.port.before_guard = Mock(side_effect=KeyboardInterrupt)
        self.camera.finish = lambda: self.assertTrue(self.port.finalized)
        r = self.run_trial(); self.assertEqual(r['status'], 'failed')
        self.assertEqual(self.port.runs, 0)

    def test_off_receipt_late_or_unknown_prevents_measurement(self):
        for field, value in [('offVerified', False), ('offAt', 9999.), ('dispatchAt', -1.)]:
            trial, scope, clock, camera, port, store = setup(); execute = port.execute
            def changed(*args, field=field, value=value):
                receipt = execute(*args); receipt[field] = value; return receipt
            port.execute = changed
            r = trial.run(confirmed_scope_sha256=canonical_sha256(scope))
            self.assertIsNone(r['measurement']); self.assertEqual(r['status'], 'failed')

    def test_final_off_failure_is_not_success(self):
        self.port.final_ok = False; r = self.run_trial()
        self.assertEqual(r['status'], 'failed'); self.assertFalse(r['finalIdleOffVerified'])

    def test_unsettled_fixed_window_is_not_shifted_to_later_frames(self):
        original = self.camera.samples
        def samples():
            records = original()
            if self.port.runs:
                for r in records: r['stable'] = False
            return records
        self.camera.samples = samples
        r = self.run_trial(); self.assertEqual(self.port.runs, 1)
        self.assertIsNone(r['measurement']); self.assertEqual(r['status'], 'failed')

    def test_initial_record_failure_prevents_camera_and_device_access(self):
        self.store.hook = Mock(side_effect=OSError('disk full')); self.port.preflight = Mock()
        with self.assertRaises(OSError): self.run_trial()
        self.port.preflight.assert_not_called(); self.assertFalse(self.camera.started)


class ObservationWindowTests(unittest.TestCase):
    def records(self):
        return [dict(sourceAt=i/10, receivedAt=i/10+.01, angleDeg=5+i*.01,
                     valid=True, stable=True) for i in range(20)]
    def test_velocity_uses_only_past_window(self):
        r = pre_action_state(self.records()[:10], 1.)
        self.assertAlmostEqual(r['velocityDegS'], .1)
        self.assertLessEqual(r['windowEnd'], .9)
    def test_noncausal_and_missing_windows_rejected(self):
        records = self.records()
        with self.assertRaises(ValueError): pre_action_state(records, .05)
        with self.assertRaises(ValueError): pre_action_state(records[:3], 1.)
        records[3]['sourceAt'] = 10**400
        with self.assertRaises(ValueError): window(records, .1, .9, 2., 5)
    def test_duplicate_or_invalid_window_rejected(self):
        records = self.records()[:10]; records[3] = records[2]
        with self.assertRaises(ValueError): pre_action_state(records, 1.)


if __name__ == '__main__': unittest.main()
