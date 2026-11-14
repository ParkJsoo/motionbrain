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
            pulse=short.short_task_pulse('up',short.current_short_policy(getattr(self,'up_cap',350),getattr(self,'down_cap',200))),expectedStartAngleDeg=197.22,
            startAngleToleranceDeg=.1,maxObservedDeltaDeg=.9,maxObservationAgeSeconds=3.,
            goalAngleDeg=197.85,shortTaskPolicy=short.current_short_policy(getattr(self,'up_cap',350),getattr(self,'down_cap',200)))
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
        self.assertEqual(r['receipt']['monitorTimingPolicy'],self.scope['shortTaskPolicy']['upMonitorTimingPolicy'])
        self.assertEqual(r['receipt']['plannedShortTaskPolicy'],self.scope['shortTaskPolicy'])
        self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),1)
        self.assertFalse(r['liftVerified']);self.assertFalse(r['modelSelected'])
    def test_DOWN_goal_has_separate_profile_and_support(self):
        self.scope.update(pulse=short.short_task_pulse('down',self.scope['shortTaskPolicy']),goalAngleDeg=197.22,expectedStartAngleDeg=197.85)
        self.scope['context']['supportMode']='gripper_supported';self.device.angle=197.85
        def no_table(record):
            record['onsite']['tableSupportVerified']=False
            raw={k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
            record['evidenceSha256']=self.observer.save(record['evidenceFile'],raw)
            self.observer.latest_record=copy.deepcopy(record)
        self.observer.on_before=no_table;self.reset_actor()
        self.device.after_run=lambda d:setattr(d,'angle',197.21)
        r=self.run_trial();self.assertEqual(r['status'],'completed',r)
        self.assertEqual(r['receipt']['monitorTimingPolicy'],self.scope['shortTaskPolicy']['downMonitorTimingPolicy'])
        self.assertEqual(r['receipt']['stopReason'],'observed_current_short_down_goal')
        self.assertTrue(r['finalIdleOffVerified'])
    def test_precise_drift_never_RUN(self):
        self.device.angle=197.33
        r=self.run_trial();self.assertFalse(r['mayHaveRun']);self.assertFalse(self.posts())
        self.assertIn('m4_start_context_changed',[e.get('code') for e in r['errors']])
    def test_observed_point9_excursion_OFF_no_retry(self):
        self.device.after_run=lambda d:setattr(d,'angle',198.13)
        r=self.run_trial();self.assertEqual(r['status'],'failed',r)
        self.assertTrue(r['finalIdleOffVerified']);self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),1)
    def test_entire_start_tolerance_domain_has_original_headroom(self):
        for start in (197.12,197.22,197.31,197.32):
            with self.subTest(start=start):
                self.assertLessEqual(start+self.scope['maxObservedDeltaDeg'],198.29)
                self.device.angle=start
                self.trial.port._check_status(self.device.status(),'IDLE')
    def test_reverse_direction_observation_still_rejected(self):
        self.device.after_run=lambda d:setattr(d,'angle',197.10)
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
    def test_DOWN250_slow_first_safety_read_times_out_OFF_no_retry(self):
        self.pulse=short.short_task_pulse('down',short.current_short_policy(550,250));self.policy=short.DOWN_MONITOR_POLICY250
        original=self.device.open;delayed=[]
        def slow(request,timeout):
            if self.device.running and request.data is None and not delayed:
                delayed.append(True);self.clock.sleep(.251)
            return original(request,timeout)
        self.device.open=slow
        with self.assertRaisesRegex(TimeoutError,'request_deadline'):self.execute()
        self.assertEqual(len(self.RUNs()),1);self.assertEqual(len(self.STOPs()),1);self.assertTrue(self.port.off_verified)
        with self.assertRaises(ValueError):
            transport.validate_monitor_policy(self.pulse,short.DOWN_MONITOR_POLICY)
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
        for direction,cap in (('up',350),('up',450),('up',550),('up',600),('down',350),('down',450),('down',550),('down',600)):
            self.setUp();policy=short.current_short_policy(cap);self.pulse=short.short_task_pulse(direction,policy)
            self.policy=policy[direction+'MonitorTimingPolicy']
            r=self.execute();self.assertTrue(r['currentShortMonitor']['completeSafetyStatusVerified'])
            self.assertLessEqual(self.STOPs()[0]['startedAt'],r['dispatchAfter']+self.pulse['durationMs']/1000+.001)
            self.assertEqual(len(self.RUNs()),1);self.assertTrue(r['offVerified'])
    def test350_and450_monitor_profiles_cannot_be_swapped(self):
        for cap,wrong in ((350,short.UP_MONITOR_POLICY450),(450,short.UP_MONITOR_POLICY),(550,short.UP_MONITOR_POLICY450),(450,short.UP_MONITOR_POLICY550),(600,short.UP_MONITOR_POLICY550),(550,short.UP_MONITOR_POLICY600)):
            with self.subTest(cap=cap),self.assertRaises(ValueError):
                transport.validate_monitor_policy(short.short_task_pulse('up',short.current_short_policy(cap)),wrong)

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
        command={k:v for k,v in short.short_task_pulse('down' if self.lower else 'up',self.policy).items()
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
        self.data=data;self.policy=short.current_short_policy(getattr(self,'up_cap',350),getattr(self,'down_cap',200));self.o=df.TaskTransitionDataTests();self.o.pulse=short.short_task_pulse('up',self.policy);self.o.setUp()
        self.addCleanup(self.o.doCleanups);fx=self.o.fixture
        self.o.angle_patch.stop();p=patch.object(data,'_angle',return_value=197.22);p.start();self.addCleanup(p.stop)
        s=fx.read('scope.json');s.update(schemaVersion=short.SCHEMA,expectedStartAngleDeg=197.22,
            startAngleToleranceDeg=.1,maxObservedDeltaDeg=.9,maxObservationAgeSeconds=3.,goalAngleDeg=197.85,shortTaskPolicy=short.current_short_policy(getattr(self,'up_cap',350),getattr(self,'down_cap',200)))
        fx.write('scope.json',s)
        r=fx.read('result.json');r.update(schemaVersion=short.SCHEMA,scopeSha256=digest(s))
        r['receipt'].update(stoppedEarly=False,stopReason='requested_period_complete',
            monitorTimingPolicy=self.policy['upMonitorTimingPolicy'],plannedShortTaskPolicy=self.policy)
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
        reg['context']['executionPolicy']=copy.deepcopy(self.policy);ann['registrationSha256']=digest(reg)
        row=self.data.build_manual_m4_row(self.o.root,reg,ann)
        self.assertEqual(row['context']['executionPolicy'],self.policy)
        self.assertEqual(row['command']['durationMs'],self.policy['upDurationCapMs'])
        reg['context']['executionPolicy']=short.current_short_policy(450 if self.policy['upDurationCapMs']==350 else 350);ann['registrationSha256']=digest(reg)
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

class RecordedShortCompatibilityTests(unittest.TestCase):
    def test_recorded_zero_effect_sources_are_preserved_but_not_executable(self):
        for cap,old_sha,next_cap in [(350,core.CURRENT_SHORT_350_RECORDED_DIGEST,450),
                                     (450,core.CURRENT_SHORT_450_RECORDED_DIGEST,550)]:
            with self.subTest(cap=cap):
                policy=short.current_short_policy(cap)
                original=dict(fixtures.scope(),schemaVersion=core.CURRENT_SHORT_TASK_SCHEMA,
                    pulse=short.short_task_pulse('up',policy),expectedStartAngleDeg=197.22,
                    startAngleToleranceDeg=.1,maxObservedDeltaDeg=.9,maxObservationAgeSeconds=3.,
                    goalAngleDeg=197.85,shortTaskPolicy=policy,implementationSha256=old_sha)
                self.assertEqual(core.validate_recorded_scope(original),original)
                with self.assertRaisesRegex(ValueError,'implementation_changed'):
                    core.validate_scope(original)
                other=copy.deepcopy(original);other['shortTaskPolicy']=short.current_short_policy(next_cap)
                other['pulse']=short.short_task_pulse('up',other['shortTaskPolicy'])
                with self.assertRaisesRegex(ValueError,'recorded_implementation_unknown'):
                    core.validate_recorded_scope(other)
                other=copy.deepcopy(original);other['shortTaskPolicy']=short.current_short_policy(cap,250)
                with self.assertRaisesRegex(ValueError,'recorded_implementation_unknown'):
                    core.validate_recorded_scope(other)


class ShortActor450Tests(ShortActorTests):
    up_cap=450

class ShortRawData450Tests(ShortRawDataTests):
    up_cap=450

class ShortModelBinding450Tests(ShortModelBindingTests):
    def setUp(self):
        super().setUp();self.policy=short.current_short_policy(450)

class ShortActor550Tests(ShortActorTests):
    up_cap=550

class ShortRawData550Tests(ShortRawDataTests):
    up_cap=550
    def test450_registration_cannot_relabel550_source(self):
        reg,ann=self.o.annotations();reg['context']['executionPolicy']=short.current_short_policy(450)
        ann['registrationSha256']=digest(reg)
        with self.assertRaisesRegex(ValueError,'execution_policy_changed'):
            self.data.build_manual_m4_row(self.o.root,reg,ann)

class ShortModelBinding550Tests(ShortModelBindingTests):
    def setUp(self):
        super().setUp();self.policy=short.current_short_policy(550)
    def test450_actor_policy_cannot_execute550_model_command(self):
        original=self.o.executor.m4_factory
        def wrong(stage,command,hooks):
            actor,_=original(stage,command,hooks);actor.scope['shortTaskPolicy']=short.current_short_policy(450)
            actor.digest=digest(actor.scope);return actor,actor.digest
        self.o.executor.m4_factory=wrong
        with self.assertRaisesRegex(ValueError,'short_policy_mismatch'):self.call()
        self.assertEqual(self.o.run_calls,0)

class ShortActorDown250Tests(ShortActorTests):
    up_cap=550
    down_cap=250

class ShortRawDataDown250Tests(ShortRawDataTests):
    up_cap=550
    down_cap=250
    def test200_registration_cannot_relabel250_paired_source(self):
        reg,ann=self.o.annotations();reg['context']['executionPolicy']=short.current_short_policy(550,200)
        ann['registrationSha256']=digest(reg)
        with self.assertRaisesRegex(ValueError,'execution_policy_changed'):
            self.data.build_manual_m4_row(self.o.root,reg,ann)

class ShortModelBindingDown250Tests(ShortModelBindingTests):
    def setUp(self):
        super().setUp();self.policy=short.current_short_policy(550,250)

class ShortActor600Tests(ShortActorTests):
    up_cap=600
    down_cap=250

class ShortRawData600Tests(ShortRawDataTests):
    up_cap=600
    down_cap=250
    def test550_registration_cannot_relabel600_source(self):
        reg,ann=self.o.annotations();reg['context']['executionPolicy']=short.current_short_policy(550,250)
        ann['registrationSha256']=digest(reg)
        with self.assertRaisesRegex(ValueError,'execution_policy_changed'):
            self.data.build_manual_m4_row(self.o.root,reg,ann)

class ShortModelBinding600Tests(ShortModelBindingTests):
    def setUp(self):
        super().setUp();self.policy=short.current_short_policy(600,250)

class Recorded550CompatibilityTests(unittest.TestCase):
    def test_only_exact550_250_recorded_contract_is_readable(self):
        policy=short.current_short_policy(550,250)
        original=dict(fixtures.scope(),schemaVersion=core.CURRENT_SHORT_TASK_SCHEMA,
            pulse=short.short_task_pulse('up',policy),expectedStartAngleDeg=197.22,
            startAngleToleranceDeg=.1,maxObservedDeltaDeg=.9,maxObservationAgeSeconds=3.,
            goalAngleDeg=197.85,shortTaskPolicy=policy,implementationSha256=core.CURRENT_SHORT_550_DOWN250_RECORDED_DIGEST)
        self.assertEqual(core.validate_recorded_scope(original),original)
        with self.assertRaisesRegex(ValueError,'implementation_changed'):core.validate_scope(original)
        for policy in (short.current_short_policy(600,250),short.current_short_policy(550,200)):
            other=dict(original,shortTaskPolicy=policy,pulse=short.short_task_pulse('up',policy))
            with self.assertRaisesRegex(ValueError,'recorded_implementation_unknown'):core.validate_recorded_scope(other)
