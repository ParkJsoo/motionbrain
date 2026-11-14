"""1200ms 명시 정책: ARM후 영상, 목표/기한/OFF, 모델·원본 혼합 거절. 모의장치만."""
import copy,json,unittest
from unittest.mock import patch
from learning import m4_effective_policy as policy
from learning import m4_up_commissioning as core
from learning import task_transition_data as data
from learning import task_action_models as models
from learning.finite_task_cycle import bounded_command
from learning.pipeline_learning import digest
from learning.supervised_collection import plan_digest
from tests import test_m4_up_commissioning as fx
from tests import test_task_cycle_workflow as cyclefx
from tests import test_task_action_models as modelfx
from tests import test_task_transition_data as datafx

class EffectiveActorTests(unittest.TestCase):
 def setUp(self):
  fx.CommissioningTests.setUp(self)
  self.scope.update(schemaVersion=core.EFFECTIVE_TASK_SCHEMA,pulse=policy.effective_task_pulse('up'),
   expectedStartAngleDeg=197.22,startAngleToleranceDeg=.1,maxObservedDeltaDeg=.9,
   maxObservationAgeSeconds=3.,goalAngleDeg=197.85,effectivePolicy=policy.effective_policy())
  self.device.angle=197.22;self.reset()
 def reset(self,hooks=None):
  self.trial=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.evidence,clock=self.clock,task_hooks=hooks)
  self.trial.port.opener=self.device
 run_trial=fx.CommissioningTests.run_trial
 posts=fx.CommissioningTests.posts
 def test_manual_before_only_after_ARM_epoch_and_goal_OFF(self):
  seen=[]
  def before(record):
   seen.append(self.device.state)
   epoch=self.trial.read_armed_off_epoch(self.trial.port.last_sequence)
   self.assertTrue(epoch['allFiveOff'])
  self.observer.on_before=before;self.device.after_run=lambda d:setattr(d,'angle',197.85)
  r=self.run_trial();self.assertEqual(r['status'],'completed',r)
  self.assertEqual(seen,['ARMED']);self.assertEqual(self.observer.index,2)
  self.assertEqual(r['receipt']['plannedEffectivePolicy'],policy.effective_policy())
  self.assertTrue(r['receipt']['stoppedEarly']);self.assertTrue(r['finalIdleOffVerified'])
  self.assertFalse(r['liftVerified']);self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)
 def test_DOWN_has_same_fresh_ARM_path_and_exact_goal(self):
  self.scope.update(pulse=policy.effective_task_pulse('down'),expectedStartAngleDeg=197.75,goalAngleDeg=197.22)
  self.scope['context']['supportMode']='gripper_supported';self.device.angle=197.75
  def before(record):
   self.assertEqual(self.device.state,'ARMED');record['onsite']['tableSupportVerified']=False
   record['evidenceSha256']=self.observer.save(record['evidenceFile'],{k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')})
   self.observer.latest_record=copy.deepcopy(record)
  self.observer.on_before=before;self.reset();self.device.after_run=lambda d:setattr(d,'angle',197.22)
  r=self.run_trial();self.assertEqual(r['status'],'completed',r)
  self.assertEqual(r['receipt']['stopReason'],'observed_effective_down_goal');self.assertTrue(r['finalIdleOffVerified'])
 def test_stale_ARM_before_is_known_not_started_no_retake(self):
  self.observer.on_before=lambda record:self.clock.sleep(3.1)
  r=self.run_trial();self.assertEqual(r['status'],'failed',r)
  self.assertEqual(r['commandOutcome'],'not_started')
  self.assertFalse(r['mayHaveRun']);self.assertTrue(r['finalIdleOffVerified']);self.assertEqual(self.observer.index,1)
  self.assertFalse(any(v=={'action':'run'} for _,v in self.posts()))
 def test_wrong_ARM_cursor_aborts_without_RUN(self):
  self.observer.on_before=lambda record:self.trial.read_armed_off_epoch(self.trial.port.last_sequence+1)
  r=self.run_trial();self.assertFalse(r['mayHaveRun']);self.assertTrue(r['finalIdleOffVerified'])
 def test_fresh_cached_external_before_ARM_is_rejected_without_retake(self):
  original=self.observer.start
  def start(**kwargs):original(**kwargs);self.clock.sleep(.5)
  self.observer.start=start
  def cached(record):
   t=10.1;lo,hi=core.validate_sample(self.observer.bridge)
   self.observer.rebind_external(record,native_changes=dict(convertedHostPtsSeconds=t,callbackHostSeconds=t+.01,
    pts=dict(value=int(t*1000000),timescale=1000000,epoch=0,flags=1)),
    ingress_changes=dict(convertedHostPtsSeconds=t,callbackHostSeconds=t+.01,sourceEarliest=t+lo,sourceLatest=t+hi,stdoutReadAt=t+.03))
  self.observer.on_before=cached
  r=self.run_trial();self.assertFalse(r['mayHaveRun']);self.assertTrue(r['finalIdleOffVerified'])
  self.assertFalse(any(v=={'action':'run'} for _,v in self.posts()));self.assertEqual(self.observer.index,1)
  self.assertIn('m4_effective_source_before_ARM',[e.get('code') for e in r['errors']])
 def test_parent_model_prepare_runs_after_fresh_capture_and_delay_rejects(self):
  events=[];hooks=dict(prepare=lambda:events.append(self.observer.index),before_run=lambda:self.clock.sleep(.2),status_guard=lambda s:None,decisionSha256='f'*64)
  self.reset(hooks);r=self.run_trial()
  self.assertEqual(events,[1]);self.assertFalse(r['mayHaveRun']);self.assertTrue(r['finalIdleOffVerified'])
 def test_cap_shortfall_completes_once_without_height_success(self):
  r=self.run_trial();self.assertEqual(r['status'],'completed',r);self.assertFalse(r['receipt']['stoppedEarly'])
  self.assertEqual(r['receipt']['requestedPulse']['durationMs'],1200);self.assertTrue(r['finalIdleOffVerified'])
  self.assertFalse(r['liftVerified']);self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)

class EffectiveModelsTests(unittest.TestCase):
 def setUp(self):
  self.context=dict(modelfx.CONTEXT,executionPolicy=policy.effective_policy())
  self.up={k:v for k,v in policy.effective_task_pulse('up').items() if k in ('joint','direction','percent','durationMs')}
  self.down=dict(self.up,direction='down',percent=35)
 def rows(self):
  rows=[modelfx.sample(i,self.up) for i in (0,1)]+[modelfx.sample(i,self.down,'gripper_supported') for i in (2,3)]
  for row in rows:row['context']=copy.deepcopy(self.context)
  return rows
 def test_four_new_rows_fit_and_select_and_old_scope_rejects1200(self):
  rows=self.rows();model=models.fit_m4(rows,self.context)
  result=models.select_m4(model,modelfx.state(.5,196.05),modelfx.state(5.5,197.05),[self.up],self.context,'table_supported')
  self.assertEqual(result['selected']['command'],self.up)
  self.assertEqual(bounded_command('lift',self.up,policy.effective_policy())['expectedPwm'],127)
  for call in (lambda:bounded_command('lift',self.up),lambda:models.fit_m4(rows,modelfx.CONTEXT)):
   with self.assertRaises(ValueError):call()
 def test_missing_candidate_support_or_new_policy_mismatch_stops(self):
  model=models.fit_m4(self.rows()[:2],self.context)
  choice=models.select_m4(model,modelfx.state(),modelfx.state(10.),[self.down],self.context,'gripper_supported')
  self.assertEqual(choice['selected']['kind'],'stop');self.assertFalse(choice['executionAuthorized'])
  changed=dict(self.context,executionPolicy=dict(policy.effective_policy(),upPercent=75))
  with self.assertRaises(ValueError):models.fit_m4(self.rows(),changed)
 def test_policy_and_commands_exact_only(self):
  for key,value in [('upPercent',75),('downPercent',75),('upDurationCapMs',1201),('maxObservedDeltaDeg',1.),('observationAcquisition','before_ARM')]:
   with self.subTest(key=key),self.assertRaises(ValueError):policy.validate_effective_policy(dict(policy.effective_policy(),**{key:value}))

class EffectiveBindingTests(unittest.TestCase):
 def test_exact_model_policy_before_actor_and_old_actor_RUN0(self):
  o=cyclefx.ScopedExecutorTests();o.setUp();base=o.executor.m4_factory;bad=[False]
  def factory(stage,command,hooks):
   actor,_=base(stage,command,hooks)
   actor.scope.update(schemaVersion=core.EFFECTIVE_TASK_SCHEMA,effectivePolicy=policy.effective_policy(),goalAngleDeg=197.85)
   if bad[0]:actor.scope['effectivePolicy']['upDurationCapMs']=650
   actor.digest=digest(actor.scope);return actor,actor.digest
  o.executor.m4_factory=factory
  stage=dict(kind='lift',context=dict(executionPolicy=policy.effective_policy()))
  command={k:v for k,v in policy.effective_task_pulse('up').items() if k in ('joint','direction','percent','durationMs')}
  def call():return o.executor(stage,command,prepare=lambda:None,before_run=lambda:None,decision={'model':'fixed'},safety=o.safety)
  self.assertEqual(call(),{'testReceipt':True});self.assertEqual(o.run_calls,1)
  bad[0]=True
  with self.assertRaisesRegex(ValueError,'effective_policy_mismatch'):call()
  self.assertEqual(o.run_calls,1)

class EffectiveTransportTests(unittest.TestCase):
 def setUp(self):
  from tests.test_m4_target_monitor import TargetMonitorTests
  owner=TargetMonitorTests();owner.setUp();self.owner=owner
  self.port=owner.port;self.device=owner.device;self.clock=owner.clock
  self.pulse=policy.effective_task_pulse('up');self.device.angle=197.22
 def execute(self,goal):
  return self.port._execute_validated(self.pulse,lambda:None,stop_when=goal,monitor_policy=policy.UP_MONITOR_POLICY)
 def test_startup_after600ms_is_still_actively_observed_and_goal_stops(self):
  def goal(status):
   if self.clock.now-self.port.last_dispatch_after>=.6:self.device.angle=197.85
   return status['shoulderAngle']['angleDeg']>=197.85
  r=self.execute(goal)
  self.assertTrue(r['stoppedEarly']);self.assertTrue(r['offVerified'])
  when=r['effectiveMonitor']['goalObserved']['evaluatedAt']-r['dispatchAfter']
  self.assertGreaterEqual(when,.6);self.assertLess(when,1.0)
  self.assertEqual(len(self.owner.RUNs()),1)
 def test_late_first_GET_times_out_OFF_no_retry(self):
  original=self.device.open;seen=[]
  def slow(request,timeout):
   if self.device.running and request.data is None and not seen:
    seen.append(True);self.clock.sleep(1.201)
   return original(request,timeout)
  self.device.open=slow
  with self.assertRaises(TimeoutError):self.execute(lambda s:False)
  self.assertEqual(len(self.owner.RUNs()),1);self.assertEqual(len(self.owner.STOPs()),1)
  self.assertTrue(self.port.off_verified)

class EffectiveRawTests(unittest.TestCase):
 def setUp(self):
  from learning.supervised_transport import SequencePort
  original=SequencePort._execute_validated
  def delayed(port,pulse,tick,**kw):
   kw['before_run']=lambda:port.clock.sleep(getattr(self,'before_run_delay',.7))
   kw.update(monitor_policy=policy.UP_MONITOR_POLICY,stop_when=lambda status:False)
   return original(port,pulse,tick,**kw)
  owner=datafx.TaskTransitionDataTests();owner.pulse=policy.effective_task_pulse('up')
  with patch.object(SequencePort,'_execute_validated',delayed):owner.setUp()
  self.addCleanup(owner.doCleanups);self.owner=owner;f=owner.fixture
  owner.angle_patch.stop();angle=patch.object(data,'_angle',return_value=197.22);angle.start();self.addCleanup(angle.stop)
  s=f.read('scope.json');s.update(schemaVersion=core.EFFECTIVE_TASK_SCHEMA,pulse=policy.effective_task_pulse('up'),
   expectedStartAngleDeg=197.22,startAngleToleranceDeg=.1,maxObservedDeltaDeg=.9,maxObservationAgeSeconds=3.,goalAngleDeg=197.85,effectivePolicy=policy.effective_policy())
  f.write('scope.json',s);r=f.read('result.json');r.update(schemaVersion=core.EFFECTIVE_TASK_SCHEMA,scopeSha256=plan_digest(s))
  r['receipt'].update(plannedEffectivePolicy=policy.effective_policy(),monitorTimingPolicy=policy.UP_MONITOR_POLICY,
   stoppedEarly=False,stopReason='requested_period_complete');f.write('result.json',r)
  for p in owner.root.glob('http-*-response.json'):
   x=f.read(p.name)
   if 'shoulderAngle' in x.get('parsed',{}):
    x['parsed']['shoulderAngle']['angleDeg']=197.22;x['rawUtf8']=json.dumps(x['parsed']);f.write(p.name,x)
 def test_effective_raw_policy_and_ARM_source_time_are_audited(self):
  report=self.owner.audit();self.assertEqual(report['executionPolicy'],policy.effective_policy())
  reg,ann=self.owner.annotations();reg['context']['executionPolicy']=policy.effective_policy();ann['registrationSha256']=plan_digest(reg)
  row=data.build_manual_m4_row(self.owner.root,reg,ann);self.assertEqual(row['command']['durationMs'],1200)
  reg['context']['executionPolicy']={'schemaVersion':'old-short'};ann['registrationSha256']=plan_digest(reg)
  with self.assertRaisesRegex(ValueError,'execution_policy_changed'):data.build_manual_m4_row(self.owner.root,reg,ann)
 def test_old_timing_policy_cannot_be_relabelled_effective(self):
  r=self.owner.fixture.read('result.json');r['receipt']['monitorTimingPolicy']='m4-current-short-up-deadline-600-v1'
  self.owner.fixture.write('result.json',r)
  with self.assertRaisesRegex(ValueError,'receipt_policy_changed'):self.owner.audit()
 def test_missing_actual_active_monitor_proof_cannot_freeze(self):
  r=self.owner.fixture.read('result.json');r['receipt']['effectiveMonitor']['completeSafetyStatusVerified']=False
  self.owner.fixture.write('result.json',r)
  with self.assertRaisesRegex(ValueError,'active_monitor_or_STOP_deadline_missing'):self.owner.audit()

class EffectiveRawBeforeARMTests(unittest.TestCase):
 def test_camera_sample_before_ARM_fails_even_when_fresh3seconds(self):
  fixture=EffectiveRawTests();fixture.before_run_delay=0;fixture.setUp();self.addCleanup(fixture.doCleanups)
  with self.assertRaisesRegex(ValueError,'before_must_follow_ARM'):fixture.owner.audit()
