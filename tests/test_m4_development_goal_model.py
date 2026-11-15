import copy
import unittest
from learning import m4_development_goal_model as model
from learning.pipeline_learning import digest
from learning.artifacts import ArtifactStore
from learning import m4_device_guard_policy as policy
from learning import m4_device_guard_transport as transport
from learning import m4_up_commissioning as core
from tests.test_m4_device_guard_integration import ActorTests


def fitted():
 visual=[dict(sourceSha256=str(i)*64,offVerified=True,originalStatus='completed',direction='up',percent=50,
  beforeAngleDeg=197.31,afterAngleDeg=197.31+delta,objectRiseDeltaPx=rise,beforeObjectWidthPx=59.,durationMs=duration)
  for i,(delta,rise,duration) in enumerate([(.09,0.,350),(0.,0.,450),(0.,0.,550),(.44,6.,600)])]
 sensor=[dict(sourceSha256='f'*64,offVerified=True,originalStatus='failed',commandOutcome='receipt_recorded',terminationReason='goal',
  direction='up',percent=50,beforeAngleDeg=194.32187,stopAngleDeg=195.02499,postOffAngleDeg=195.38,runMs=596.,
  postOffAgeSeconds=.653,postWindowSatisfied=False)]
 return model.fit_outcome_model(visual,sensor,audit_sources=lambda x:x)


def current(angle=195.38,rise=0.):
 return dict(m4AngleDeg=angle,objectRiseFraction=rise,observationSha256='c'*64,tableSupported=True,bilateralContact=True)


class ModelTests(unittest.TestCase):
 def test_model_compares_goals_stop_and_rejected_with_uncertainty(self):
  result=model.choose_goal(fitted(),current(),[195.8,196.2,196.6,197.,197.3,197.6],8/57)
  self.assertEqual(result['selected']['goalAngleDeg'],195.8)
  self.assertIn('stop',[x['kind'] for x in result['candidates']]);self.assertEqual(len(result['rejected']),3)
  self.assertTrue(result['uncertainty']['laterCoastUnobserved']);self.assertIsNone(result['uncertainty']['interval'])
  self.assertNotIn('predictedSettledAngleDeg',result['selected']);self.assertFalse(result['liftVerified'])
 def test_high_goals_do_not_force_action(self):
  result=model.choose_goal(fitted(),current(),[197.,197.3,197.6],8/57)
  self.assertEqual(result['selected']['kind'],'stop')
 def test_observed_target_is_stop_without_another_pulse(self):
  result=model.choose_goal(fitted(),current(196.1,8/57),[196.6,197.],8/57)
  self.assertEqual(result['selected']['kind'],'stop')
 def test_bad_raw_audit_and_unknown_command_rejected(self):
  m=fitted();e=m['sourceEvidence']
  with self.assertRaisesRegex(ValueError,'raw_audit'):model.fit_outcome_model(e['visualRows'],e['guardSensorRows'],audit_sources=lambda x:False)
  e['guardSensorRows'][0]['commandOutcome']='unknown'
  with self.assertRaisesRegex(ValueError,'sensor_evidence_invalid'):model.fit_outcome_model(e['visualRows'],e['guardSensorRows'],audit_sources=lambda x:x)
 def test_goal_boundary_and_missing_contact_not_silently_admitted(self):
  result=model.choose_goal(fitted(),current(),[198.,195.3],8/57)
  self.assertEqual(result['selected']['kind'],'stop');self.assertEqual(len(result['rejected']),2)
  c=current();c['bilateralContact']=False
  with self.assertRaisesRegex(ValueError,'current_observation'):model.choose_goal(fitted(),c,[195.8],8/57)


class SessionTests(unittest.TestCase):
 def session(self,failed=False):
  from tests.test_task_cycle_workflow import Store
  m=fitted();created=[];observed=[]
  def factory(c,goals,index,**kwargs):
   trial=object.__new__(model.BoundedGoalTrial);trial.model=m;trial.target=8/57;trial.current=c
   trial.decision=model.choose_goal(m,c,goals,8/57)
   created.append(trial.decision['selected']['goalAngleDeg'])
   trial.run=lambda **kwargs:dict(status='failed' if failed else 'completed',commandOutcome='receipt_recorded',finalIdleOffVerified=True,modelSelectedCommands=1)
   return trial
  def after(trial,**kwargs):
   observed.append(1);i=len(observed)
   c=current(created[-1],.04 if i==1 else .15);c['observationSha256']=str(i)*64;return c
  session=model.BoundedGoalSession(m,current(),[195.8,196.2,196.6],8/57,store=Store(),trial_factory=factory,observe_after=after,max_seconds=300.)
  return session,created,observed
 def test_normal_completion_reobserves_then_selects_different_goal(self):
  from tests.test_supervised_transport import Clock
  session,created,observed=self.session();result=session.run(confirmed_session_sha256=digest(session.plan),clock=Clock())
  self.assertEqual(result['reason'],'observed_visual_target_reached');self.assertEqual(created,[195.8,196.2]);self.assertEqual(len(observed),2)
  self.assertFalse(result['fullyModelSelectedCycle']);self.assertFalse(result['liftVerified'])
 def test_failed_attempt_never_reobserves_or_constructs_replacement(self):
  from tests.test_supervised_transport import Clock
  session,created,observed=self.session(True);result=session.run(confirmed_session_sha256=digest(session.plan),clock=Clock())
  self.assertEqual(result['reason'],'failed_attempt_no_retry');self.assertEqual(len(created),1);self.assertEqual(observed,[])
 def test_confirmed_plan_cannot_be_changed_through_model_or_current(self):
  from tests.test_supervised_transport import Clock
  for field in ('model','current','target','max_seconds','goals'):
   session,created,_=self.session();confirmed=digest(session.plan)
   if field=='model':session.model['coastMeanDeg']+=.1
   elif field=='current':session.current['m4AngleDeg']+=.1
   elif field=='goals':session.goals.append(197.)
   else:setattr(session,field,getattr(session,field)+1.)
   with self.subTest(field=field),self.assertRaises(ValueError):session.run(confirmed_session_sha256=confirmed,clock=Clock())
   self.assertEqual(created,[])
 def test_after_exception_keeps_completed_attempt_and_blocks_next(self):
  from tests.test_supervised_transport import Clock
  session,created,_=self.session()
  session.observe_after=lambda *args,**kwargs:(_ for _ in ()).throw(TimeoutError())
  result=session.run(confirmed_session_sha256=digest(session.plan),clock=Clock())
  self.assertEqual(result['reason'],'exception_no_retry');self.assertEqual(result['modelSelectedCommands'],1)
  self.assertEqual(len(result['trials']),1);self.assertEqual(len(created),1)
  self.assertFalse(result['unknownAttemptOutcome'])


class ActualActorTests(unittest.TestCase):
 def setUp(self):
  self.fx=ActorTests();self.fx.setUp();self.addCleanup(self.fx.doCleanups)
  f=self.fx;f.device.angle=195.38
  envelope=f.device.envelope
  f.device.envelope=lambda:dict(envelope(),supportedPolicySha256=[transport.POLICY_SHA256,transport.CURRENT_POLICY_SHA256,transport.FINITE_POLICY_SHA256])
  self.store=ArtifactStore(f.repo/'.local/goal-model')
 def trial(self,changed=False):
  f=self.fx
  def factory(decision,hooks):
   goal=decision['selected']['goalAngleDeg'];p=policy.finite_goal_device_guard_policy(goal)
   f.scope.update(expectedStartAngleDeg=195.38,goalAngleDeg=goal,deviceGuardPolicy=p,maxObservedDeltaDeg=3.,pulse=policy.device_guard_task_pulse('up',p))
   actor=core.M4UpCommissioning(f.scope,f.observer,'private-token',f.store,evidence_root=f.evidence,clock=f.clock,task_hooks=hooks)
   actor.port.opener=f.device;return actor,actor.digest
  def armed(actor):
   self.assertIsNotNone(actor.before);self.assertEqual(f.device.state,'ARMED')
   return current(rise=8/57 if changed else 0.)
  return model.BoundedGoalTrial(fitted(),current(),[195.8,196.2,196.6],8/57,store=self.store,
      actor_factory=factory,observe_armed=armed,validate_memory=lambda actor,sha:True)
 def test_actual_model_choice_goes_through_v3_actor_once(self):
  trial=self.trial();result=trial.run(confirmed_decision_sha256=digest(trial.decision))
  self.assertEqual(result['status'],'completed',result);self.assertEqual(result['modelSelectedCommands'],1)
  self.assertEqual(len(self.fx.runs()),1);self.assertFalse(result['liftVerified'])
  with self.assertRaisesRegex(ValueError,'unconsumed'):trial.run(confirmed_decision_sha256=digest(trial.decision))
 def test_failed_sensor_short_post_OFF_is_not_a_settled_label(self):
  original=self.fx.observer.observe
  def observe(**kwargs):
   if kwargs['after_off'] is not None:raise ValueError('m4_observed_excursion')
   return original(**kwargs)
  self.fx.observer.observe=observe
  trial=self.trial();result=trial.run(confirmed_decision_sha256=digest(trial.decision))
  self.assertEqual(result['status'],'failed')
  evidence=model.audit_failed_guard_sensor(self.fx.store.root)
  self.assertEqual(evidence['originalStatus'],'failed');self.assertFalse(evidence['postWindowSatisfied'])
  self.assertLess(evidence['postOffAgeSeconds'],3.)
  self.assertNotIn('settledAngleDeg',evidence);self.assertFalse(evidence['visualAfterAvailable'])

 def test_session_deadline_reserves_full_actor_before_any_request(self):
  trial=self.trial()
  with self.assertRaisesRegex(ValueError,'time_reservation'):
   trial.run(confirmed_decision_sha256=digest(trial.decision),deadline=self.fx.clock.monotonic()+20,clock=self.fx.clock)
  self.assertEqual(self.fx.device.requests,[])

 def test_ARM_reselection_stop_prevents_RUN_without_replacement(self):
  trial=self.trial(changed=True);result=trial.run(confirmed_decision_sha256=digest(trial.decision))
  self.assertEqual(result['status'],'failed');self.assertEqual(result['modelSelectedCommands'],0)
  self.assertEqual(self.fx.runs(),[])
