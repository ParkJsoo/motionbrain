"""새 현재 UP 목표각 중단의 모의 제어·단발·계획정책 감사 검증."""
import copy
import unittest
from unittest.mock import patch

from learning import m4_up_commissioning as core
from learning import task_transition_data as data
from learning.artifacts import ArtifactStore
from learning.supervised_collection import plan_digest
from tests import test_m4_up_commissioning as fixtures
from tests import test_task_transition_data as datafixtures


class TargetUpTests(unittest.TestCase):
    def setUp(self):
        fixtures.CommissioningTests.setUp(self)
        self.scope.update(schemaVersion=core.TARGET_UP_SCHEMA,pulse=copy.deepcopy(core.LEARNING_PULSE),
            expectedStartAngleDeg=196.69,startAngleToleranceDeg=.25,maxObservedDeltaDeg=3.,
            goalAngleDeg=197.5,returnGoalAngleDeg=196.59,globalReferenceDeg=195.29)
        self.device.angle=196.69
        self.trial=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,
            evidence_root=self.evidence,clock=self.clock)
        self.trial.port.opener=self.device
    run_trial=fixtures.CommissioningTests.run_trial
    posts=fixtures.CommissioningTests.posts

    def test_target_angle_normal_stop_keeps_cap_and_OFF(self):
        self.device.after_run=lambda device:setattr(device,'angle',197.55)
        result=self.run_trial()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(result['selectionKind'],'manual_target_up_angle_stop')
        self.assertTrue(result['receipt']['stoppedEarly'])
        self.assertEqual(result['receipt']['stopReason'],'observed_target_up_goal')
        self.assertEqual(result['receipt']['plannedTargetUp'],core.target_up_policy(self.scope))
        self.assertEqual(result['receipt']['requestedPulse'],core.LEARNING_PULSE)
        self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(sum(payload=={'action':'run'} for _,payload in self.posts()),1)
        self.assertFalse(result['liftVerified']);self.assertFalse(result['modelSelected'])

    def test_cap_shortfall_is_completed_without_automatic_retry(self):
        self.device.after_run=lambda device:setattr(device,'angle',196.9)
        result=self.run_trial();self.assertEqual(result['status'],'completed',result)
        self.assertFalse(result['receipt']['stoppedEarly'])
        self.assertEqual(result['receipt']['stopReason'],'requested_period_complete')
        self.assertEqual(sum(payload=={'action':'run'} for _,payload in self.posts()),1)

    def test_already_at_goal_or_precise_start_drift_rejected_before_RUN(self):
        self.device.angle=197.5
        result=self.run_trial();self.assertFalse(result['mayHaveRun']);self.assertFalse(self.posts())

    def test_original_global_upper_remains_and_failure_proves_OFF(self):
        self.device.after_run=lambda device:setattr(device,'angle',198.30)
        result=self.run_trial();self.assertEqual(result['status'],'failed')
        self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(sum(payload=={'action':'run'} for _,payload in self.posts()),1)

    def test_unknown_RUN_never_resends_or_gets_replaced(self):
        self.device.after_run=lambda device:(_ for _ in ()).throw(TimeoutError('lost RUN reply'))
        result=self.run_trial();self.assertEqual(result['commandOutcome'],'unknown')
        self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(sum(payload=={'action':'run'} for _,payload in self.posts()),1)
        count=len(self.device.requests)
        scope=dict(self.scope,trialId='no-replacement')
        other=core.M4UpCommissioning(scope,self.observer,'private-token',ArtifactStore(self.repo/'.local/other'),
            evidence_root=self.evidence,clock=self.clock)
        other.port.opener=self.device
        self.assertEqual(other.run(confirmed_scope_sha256=plan_digest(scope))['status'],'failed')
        self.assertEqual(count,len(self.device.requests))

    def test_boot_change_before_run_rejected(self):
        self.device.boot='c'*32
        result=self.run_trial();self.assertFalse(result['mayHaveRun']);self.assertFalse(self.posts())

    def test_scope_exact_goal_and_global_reference_and_legacy_unchanged(self):
        for changes in [dict(goalAngleDeg=196.79),dict(goalAngleDeg=198.29),
                        dict(globalReferenceDeg=196.),dict(startAngleToleranceDeg=.5),
                        dict(goalAngleDeg=True),dict(returnGoalAngleDeg=196.),dict(returnGoalAngleDeg=True),dict(pulse=core.LIFT_PULSE)]:
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                core.validate_scope(dict(self.scope,**changes))
        old=dict(self.scope,schemaVersion=core.CONTACT_MANUAL650_SCHEMA,startAngleToleranceDeg=.5)
        old.pop('goalAngleDeg');old.pop('returnGoalAngleDeg');old.pop('globalReferenceDeg')
        self.assertEqual(core.validate_scope(old),old)
        old['implementationSha256']=core.CURRENT_FIXED_RECORDED_DIGEST
        self.assertEqual(core.validate_recorded_scope(old),old)
        with self.assertRaisesRegex(ValueError,'implementation_changed'):core.validate_scope(old)
        with self.assertRaisesRegex(ValueError,'recorded_implementation_unknown'):
            core.validate_recorded_scope(dict(self.scope,implementationSha256=core.CURRENT_FIXED_RECORDED_DIGEST))


    def test_target_policy_parent_hooks_preserve_ARM_fresh_capture_and_normal_stop(self):
        original=self.observer.observe;events=[]
        def observe(*,after_off,**kwargs):
            if after_off is None:self.assertEqual(self.device.state,'ARMED')
            return original(after_off=after_off,**kwargs)
        self.observer.observe=observe
        hooks=dict(prepare=lambda:events.append('prepare'),before_run=lambda:events.append('RUN'),
                   status_guard=lambda status:None,decisionSha256='e'*64)
        self.trial=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,
            evidence_root=self.evidence,clock=self.clock,task_hooks=hooks)
        self.trial.port.opener=self.device
        self.device.after_run=lambda device:setattr(device,'angle',197.55)
        result=self.run_trial();self.assertEqual(result['status'],'completed',result)
        self.assertEqual(events,['prepare','RUN'])
        self.assertTrue(result['parentModelSelected']);self.assertEqual(result['parentModelDecisionSha256'],'e'*64)
        self.assertEqual(result['receipt']['plannedTargetUp'],core.target_up_policy(self.scope))
        self.assertTrue(result['receipt']['stoppedEarly']);self.assertTrue(result['finalIdleOffVerified'])

    def test_changed_goal_after_parent_binding_rejected_without_RUN(self):
        self.trial.scope['goalAngleDeg']=197.8
        result=self.run_trial();self.assertEqual(result['status'],'failed')
        self.assertFalse(result['mayHaveRun']);self.assertFalse(self.posts())


    def test_target_policy_validator_keeps_UP_and_RETURN_pair_finite_and_exact(self):
        policy=core.target_up_policy(self.scope)
        self.assertEqual(core.validate_target_up_policy(policy),policy)
        for changes in [dict(returnGoalAngleDeg=float('nan')),dict(returnGoalAngleDeg=True),
                        dict(returnGoalAngleDeg=197.5),dict(returnGoalAngleDeg=192.6),
                        dict(goalAngleDeg=198.05),dict(globalUpperDeg=199.),
                        dict(pulseDurationCapMs=True),dict(extra='unlisted')]:
            with self.subTest(changes=changes),self.assertRaisesRegex(ValueError,'policy_invalid'):
                core.validate_target_up_policy(dict(policy,**changes))


class TargetUpDataTests(unittest.TestCase):
    def setUp(self):
        owner=datafixtures.TaskTransitionDataTests();owner.pulse=core.LEARNING_PULSE;owner.setUp()
        self.addCleanup(owner.doCleanups);self.owner=owner;fx=owner.fixture
        scope=fx.read('scope.json');scope.update(schemaVersion=core.TARGET_UP_SCHEMA,
            startAngleToleranceDeg=.25,maxObservedDeltaDeg=3.,goalAngleDeg=197.5,returnGoalAngleDeg=194.9,globalReferenceDeg=195.29)
        fx.write('scope.json',scope);self.scope=scope
        result=fx.read('result.json');result.update(schemaVersion=core.TARGET_UP_SCHEMA,scopeSha256=plan_digest(scope))
        result['receipt'].update(stoppedEarly=False,stopReason='requested_period_complete',plannedTargetUp=core.target_up_policy(scope))
        fx.write('result.json',result)

    def test_planned_cap_row_preserves_target_policy_and_rejects_fixed_context(self):
        registration,annotation=self.owner.annotations()
        with self.assertRaisesRegex(ValueError,'execution_policy_changed'):
            data.build_manual_m4_row(self.owner.root,registration,annotation)
        registration['context']['executionPolicy']=core.target_up_policy(self.scope)
        annotation['registrationSha256']=plan_digest(registration)
        row=data.build_manual_m4_row(self.owner.root,registration,annotation)
        self.assertEqual(row['command'],{k:core.LEARNING_PULSE[k] for k in ('joint','direction','percent','durationMs')})
        self.assertEqual(row['context']['executionPolicy'],core.target_up_policy(self.scope))
        self.assertNotIn('receipt',row)

    def test_changed_stop_policy_in_receipt_rejected(self):
        result=self.owner.fixture.read('result.json');result['receipt']['plannedTargetUp']['goalAngleDeg']=198.
        self.owner.fixture.write('result.json',result)
        with self.assertRaisesRegex(ValueError,'receipt_policy_changed'):self.owner.audit()

    def test_early_stop_claim_requires_original_active_goal_angle(self):
        result=self.owner.fixture.read('result.json')
        result['receipt'].update(stoppedEarly=True,stopReason='observed_target_up_goal')
        self.owner.fixture.write('result.json',result)
        with self.assertRaisesRegex(ValueError,'stop_angle_missing'):self.owner.audit()

    def test_RETURN_row_requires_exact_paired_return_goal(self):
        owner=datafixtures.TaskTransitionDownDataTests();owner.setUp();self.addCleanup(owner.doCleanups)
        registration,annotation=owner.annotations()
        policy=core.target_up_policy(self.scope)
        registration['context']['executionPolicy']=policy
        annotation['registrationSha256']=plan_digest(registration)
        with self.assertRaisesRegex(ValueError,'execution_policy_changed'):
            data.build_manual_m4_row(owner.root,registration,annotation)
        policy['returnGoalAngleDeg']=owner.fixture.read('scope.json')['goalAngleDeg']
        annotation['registrationSha256']=plan_digest(registration)
        row=data.build_manual_m4_row(owner.root,registration,annotation)
        self.assertEqual(row['context']['executionPolicy']['returnGoalAngleDeg'],194.)
        self.assertEqual(row['command']['direction'],'down')

    def test_RETURN_registered_target_policy_rechecks_entire_observed_global_range(self):
        owner=datafixtures.TaskTransitionDownDataTests();owner.setUp();self.addCleanup(owner.doCleanups)
        report=owner.audit();registration,annotation=owner.annotations()
        policy=core.target_up_policy(self.scope);policy['returnGoalAngleDeg']=report['returnGoalAngleDeg']
        registration['context']['executionPolicy']=policy
        annotation['registrationSha256']=plan_digest(registration)
        report['observedAngleRangeDeg']=[195.,199.]
        with patch.object(data,'audit_manual_m4',return_value=report):
            with self.assertRaisesRegex(ValueError,'execution_policy_changed'):
                data.build_manual_m4_row(owner.root,registration,annotation)

    def test_goal_seen_only_afterSTOP_or_IDLE_or_cap_cannot_justify_early_stop(self):
        result=self.owner.fixture.read('result.json')
        result['receipt'].update(stoppedEarly=True,stopReason='observed_target_up_goal')
        self.owner.fixture.write('result.json',result)
        original=data._http
        def returned(mode):
            def inspect(ev,receipt,pulse):
                statuses,boot,first,stop_at=original(ev,receipt,pulse)
                event=copy.deepcopy(statuses[0])
                event['response']['shoulderAngle']['angleDeg']=197.55
                event['response']['state']='IDLE' if mode=='idle' else 'ARMED'
                event['responseReceivedAt']=(stop_at+.001 if mode=='after_stop' else
                    receipt['dispatchAfter']+.651 if mode=='after_cap' else receipt['dispatchAfter']+.1)
                event['requestStartedAt']=event['responseReceivedAt']-.001
                if mode=='after_cap':stop_at=receipt['dispatchAfter']+.7
                statuses.append(event)
                return statuses,boot,first,stop_at
            return inspect
        for mode in ('idle','after_stop','after_cap'):
            with self.subTest(mode=mode),patch.object(data,'_http',side_effect=returned(mode)):
                with self.assertRaisesRegex(ValueError,'stop_angle_missing'):self.owner.audit()
        with patch.object(data,'_http',side_effect=returned('valid')):
            self.assertEqual(self.owner.audit()['executionPolicy'],core.target_up_policy(self.scope))
