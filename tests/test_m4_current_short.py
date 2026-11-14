"""현재 짧은 paired 프로필: 모의 장치만 사용하며 실패를 재전송하지 않는다."""
import copy
import json
import unittest
from unittest.mock import patch
from learning import current_m4_short as short
from learning import m4_up_commissioning as core
from learning import supervised_transport as transport
from learning.pipeline_learning import digest
from tests import test_m4_up_commissioning as fixtures
from tests import test_m4_target_monitor as monitorfixtures
from tests import test_task_cycle_workflow as cyclefixtures

class ShortActorTests(unittest.TestCase):
    def setUp(self):
        fixtures.CommissioningTests.setUp(self)
        self.scope.update(schemaVersion=core.CURRENT_SHORT_TASK_SCHEMA,
            pulse=short.short_task_pulse('up'),expectedStartAngleDeg=197.22,
            startAngleToleranceDeg=.1,maxObservedDeltaDeg=1.,maxObservationAgeSeconds=3.,
            goalAngleDeg=197.85,shortTaskPolicy=short.current_short_policy())
        self.device.angle=197.22
        self.reset_actor()
    def reset_actor(self):
        self.trial=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,
                                        evidence_root=self.evidence,clock=self.clock)
        self.trial.port.opener=self.device
    run_trial=fixtures.CommissioningTests.run_trial
    posts=fixtures.CommissioningTests.posts
    def test_UP_goal_stops_once_OFF_and_policy_is_raw(self):
        self.device.after_run=lambda d:setattr(d,'angle',197.86)
        r=self.run_trial();self.assertEqual(r['status'],'completed',r)
        self.assertTrue(r['finalIdleOffVerified']);self.assertTrue(r['receipt']['stoppedEarly'])
        self.assertEqual(r['receipt']['monitorTimingPolicy'],short.UP_MONITOR_POLICY)
        self.assertEqual(r['receipt']['plannedShortTaskPolicy'],short.current_short_policy())
        self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),1)
        self.assertFalse(r['liftVerified']);self.assertFalse(r['modelSelected'])
    def test_DOWN_goal_has_separate_profile_and_support(self):
        self.scope.update(pulse=short.short_task_pulse('down'),goalAngleDeg=197.22,expectedStartAngleDeg=197.85)
        self.scope['context']['supportMode']='gripper_supported';self.device.angle=197.85
        def no_table(record):
            record['onsite']['tableSupportVerified']=False
            raw={k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
            record['evidenceSha256']=self.observer.save(record['evidenceFile'],raw)
            self.observer.latest_record=copy.deepcopy(record)
        self.observer.on_before=no_table;self.reset_actor()
        self.device.after_run=lambda d:setattr(d,'angle',197.21)
        r=self.run_trial();self.assertEqual(r['status'],'completed',r)
        self.assertEqual(r['receipt']['monitorTimingPolicy'],short.DOWN_MONITOR_POLICY)
        self.assertEqual(r['receipt']['stopReason'],'observed_current_short_down_goal')
        self.assertTrue(r['finalIdleOffVerified'])
    def test_precise_drift_or_headroom_never_RUN(self):
        self.device.angle=197.31 # precise tolerance passes, headroom fails
        r=self.run_trial();self.assertFalse(r['mayHaveRun']);self.assertFalse(self.posts())
        self.assertIn('m4_current_short_upper_headroom_insufficient',[e.get('code') for e in r['errors']])
    def test_observed_one_degree_excursion_OFF_no_retry(self):
        self.device.after_run=lambda d:setattr(d,'angle',198.24)
        r=self.run_trial();self.assertEqual(r['status'],'failed',r)
        self.assertTrue(r['finalIdleOffVerified']);self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),1)
    def test_changed_cap_or_policy_rejected(self):
        for changed in [dict(pulse=core.LEARNING_PULSE),dict(goalAngleDeg=198.),dict(maxObservationAgeSeconds=10.)]:
            with self.subTest(changed=changed),self.assertRaises(ValueError):core.validate_scope(dict(self.scope,**changed))
    def test_parent_hook_delay_cannot_bypass_last_freshness_check(self):
        hooks=dict(prepare=lambda:None,before_run=lambda:self.clock.sleep(.2),status_guard=lambda s:None,
                   decisionSha256='e'*64)
        self.trial=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,
            evidence_root=self.evidence,clock=self.clock,task_hooks=hooks)
        self.trial.port.opener=self.device
        r=self.run_trial();self.assertFalse(r['mayHaveRun']);self.assertTrue(r['finalIdleOffVerified'])
        self.assertFalse(any(p=={'action':'run'} for _,p in self.posts()))

class ShortMonitorTests(unittest.TestCase):
    setUp=monitorfixtures.TargetMonitorTests.setUp
    RUNs=monitorfixtures.TargetMonitorTests.RUNs
    STOPs=monitorfixtures.TargetMonitorTests.STOPs
    def execute(self,goal=lambda status:False):
        return self.port._execute_validated(self.pulse,lambda:None,stop_when=goal,monitor_policy=self.policy)
    def test_short_DOWN_delayed_first_safety_read_times_out_OFF_one_RUN(self):
        self.pulse=short.short_task_pulse('down');self.policy=short.DOWN_MONITOR_POLICY
        original=self.device.open;delayed=[]
        def slow(request,timeout):
            if self.device.running and request.data is None and not delayed:
                delayed.append(True);self.clock.sleep(.21)
            return original(request,timeout)
        self.device.open=slow
        with self.assertRaisesRegex(TimeoutError,'request_deadline'):self.execute()
        self.assertEqual(len(self.RUNs()),1);self.assertEqual(len(self.STOPs()),1)
        self.assertTrue(self.port.off_verified)
    def test_both_caps_have_complete_safety_status_and_STOP_priority(self):
        for direction in ('up','down'):
            self.setUp();self.pulse=short.short_task_pulse(direction)
            self.policy=short.UP_MONITOR_POLICY if direction=='up' else short.DOWN_MONITOR_POLICY
            r=self.execute();self.assertTrue(r['currentShortMonitor']['completeSafetyStatusVerified'])
            self.assertLessEqual(self.STOPs()[0]['startedAt'],r['dispatchAfter']+self.pulse['durationMs']/1000+.001)
            self.assertEqual(len(self.RUNs()),1);self.assertTrue(r['offVerified'])

class ShortModelBindingTests(unittest.TestCase):
    def setUp(self):
        self.o=cyclefixtures.ScopedExecutorTests();self.o.setUp();self.base=self.o.executor.m4_factory
        self.policy=short.current_short_policy();self.changed=False;self.lower=False;self.legacy=False
        def factory(stage,command,hooks):
            actor,_=self.base(stage,command,hooks)
            if not self.legacy:actor.scope.update(schemaVersion=short.SCHEMA,shortTaskPolicy=copy.deepcopy(self.policy),
                                                 goalAngleDeg=197.22 if self.lower else 197.85)
            if self.changed:actor.scope['shortTaskPolicy']['upDurationCapMs']=650
            actor.digest=digest(actor.scope);return actor,actor.digest
        self.o.executor.m4_factory=factory
    def call(self):
        command={k:v for k,v in short.short_task_pulse('down' if self.lower else 'up').items()
                 if k in ('joint','direction','percent','durationMs')}
        return self.o.executor(dict(kind='lower' if self.lower else 'lift',context={'executionPolicy':self.policy}),
            command,prepare=lambda:None,before_run=lambda:None,decision={'frozen':True},safety=self.o.safety)
    def test_both_model_stages_require_exact_policy(self):
        self.assertEqual(self.call(),{'testReceipt':True});self.lower=True
        self.assertEqual(self.call(),{'testReceipt':True});self.assertEqual(self.o.run_calls,2)
    def test_wrong_actor_or_changed_policy_zero_RUN(self):
        self.changed=True
        with self.assertRaisesRegex(ValueError,'short_policy_mismatch'):self.call()
        self.changed=False;self.legacy=True
        with self.assertRaisesRegex(ValueError,'short_actor_required'):self.call()
        self.assertEqual(self.o.run_calls,0)

class ShortRawDataTests(unittest.TestCase):
    def setUp(self):
        from tests import test_task_transition_data as df
        from learning import task_transition_data as data
        self.data=data;self.o=df.TaskTransitionDataTests();self.o.pulse=short.short_task_pulse('up');self.o.setUp()
        self.addCleanup(self.o.doCleanups);fx=self.o.fixture
        self.o.angle_patch.stop();p=patch.object(data,'_angle',return_value=197.22);p.start();self.addCleanup(p.stop)
        s=fx.read('scope.json');s.update(schemaVersion=short.SCHEMA,expectedStartAngleDeg=197.22,
            startAngleToleranceDeg=.1,maxObservationAgeSeconds=3.,goalAngleDeg=197.85,shortTaskPolicy=short.current_short_policy())
        fx.write('scope.json',s)
        r=fx.read('result.json');r.update(schemaVersion=short.SCHEMA,scopeSha256=digest(s))
        r['receipt'].update(stoppedEarly=False,stopReason='requested_period_complete',
            monitorTimingPolicy=short.UP_MONITOR_POLICY,plannedShortTaskPolicy=short.current_short_policy())
        fx.write('result.json',r)
        for path in self.o.root.glob('http-*-response.json'):
            value=fx.read(path.name)
            if 'shoulderAngle' in value.get('parsed',{}):
                value['parsed']['shoulderAngle']['angleDeg']=197.22
                value['rawUtf8']=json.dumps(value['parsed'])
            fx.write(path.name,value)
    def test_new_registration_policy_required_and_raw_immutable(self):
        reg,ann=self.o.annotations()
        with self.assertRaisesRegex(ValueError,'execution_policy_changed'):self.data.build_manual_m4_row(self.o.root,reg,ann)
        reg['context']['executionPolicy']=short.current_short_policy();ann['registrationSha256']=digest(reg)
        row=self.data.build_manual_m4_row(self.o.root,reg,ann)
        self.assertEqual(row['context']['executionPolicy'],short.current_short_policy())
        self.assertEqual(row['command']['durationMs'],350)
        reg['context']['executionPolicy']['upDurationCapMs']=650;ann['registrationSha256']=digest(reg)
        with self.assertRaisesRegex(ValueError,'execution_policy_changed'):self.data.build_manual_m4_row(self.o.root,reg,ann)
    def test_old_timing_or_false_early_goal_cannot_become_row(self):
        original=self.o.fixture.read('result.json')
        r=copy.deepcopy(original);r['receipt']['monitorTimingPolicy']=transport.M4_TARGET_UP_DEADLINE_POLICY
        self.o.fixture.write('result.json',r)
        with self.assertRaisesRegex(ValueError,'short_receipt_policy_changed'):self.o.audit()
        r=copy.deepcopy(original);r['receipt'].update(stoppedEarly=True,stopReason='observed_current_short_up_goal')
        self.o.fixture.write('result.json',r)
        with self.assertRaisesRegex(ValueError,'short_stop_angle_missing'):self.o.audit()

class ShortCandidateSupportTests(unittest.TestCase):
    def test_short_policy_does_not_turn_uncollected_DOWN_into_supported_candidate(self):
        from learning import task_action_models as models
        from tests import test_task_action_models as fx
        context=dict(fx.CONTEXT,executionPolicy=short.current_short_policy())
        up={k:v for k,v in short.short_task_pulse('up').items() if k in ('joint','direction','percent','durationMs')}
        down={k:v for k,v in short.short_task_pulse('down').items() if k in up}
        rows=[fx.sample(i,up) for i in (0,1)]
        for row in rows:row['context']=copy.deepcopy(context)
        model=models.fit_m4(rows,context)
        choice=models.select_m4(model,fx.state(.5,196.05),fx.state(5.,197.),[down],context,'gripper_supported')
        self.assertEqual(choice['selected']['kind'],'stop')
        self.assertEqual(choice['rejected'][0]['reason'],'task_candidate_support_missing')
        self.assertFalse(choice['executionAuthorized'])
