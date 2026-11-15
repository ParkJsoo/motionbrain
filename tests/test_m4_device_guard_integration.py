"""장치 가드 actor·원본·모델 경로의 모의 통합. 실물/학습 자료 접근 없음."""
import copy
import json
import shutil
import unittest
from unittest.mock import patch
from learning import m4_up_commissioning as core
from learning import m4_device_guard_policy as policy
from learning import task_action_models as models
from learning import task_transition_data as data
from learning.finite_task_cycle import bounded_command
from tests import test_m4_up_commissioning as fx
from tests import test_task_action_models as modelfx
from tests.test_m4_device_guard_transport import GuardController

class Device(GuardController,fx.Device):
 pass

class ActorTests(unittest.TestCase):
 def setUp(self):
  fx.CommissioningTests.setUp(self)
  self.scope.update(schemaVersion=core.DEVICE_GUARD_TASK_SCHEMA,pulse=policy.device_guard_task_pulse('up'),
   expectedStartAngleDeg=197.22,startAngleToleranceDeg=.1,maxObservedDeltaDeg=.9,
   maxObservationAgeSeconds=3.,goalAngleDeg=197.85,deviceGuardPolicy=policy.device_guard_policy())
  self.device=Device(self.clock)
  self.trial=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.evidence,clock=self.clock)
  self.trial.port.opener=self.device
 run_trial=fx.CommissioningTests.run_trial
 def runs(self):return [v for _,v in self.device.requests if v and v.get('action')=='run_m4_guarded']
 def test_actor_after_ARM_and_device_terminal(self):
  seen=[];self.observer.on_before=lambda r:seen.append(self.device.state)
  result=self.run_trial()
  self.assertEqual(result['status'],'completed',result)
  self.assertEqual(seen,['ARMED']);self.assertEqual(len(self.runs()),1)
  self.assertEqual(result['receipt']['plannedDeviceGoalGuard'],policy.guard_request(self.scope))
  self.assertEqual(result['receipt']['stopReason'],'device_goal')
  self.assertFalse(result['liftVerified']);self.assertTrue(result['finalIdleOffVerified'])
 def test_raw_HTTP_audit_binds_nonce_and_terminal(self):
  result=self.run_trial()
  from learning.manual_collection_dataset import Evidence
  statuses,boot,first,stop=data._http(Evidence(self.store.root),result['receipt'],self.scope['pulse'])
  self.assertEqual(boot,self.scope['controllerBootId'])
  bad=copy.deepcopy(result['receipt']);bad['deviceGoalGuard']['nonce']='f'*32
  with self.assertRaises(ValueError):data._http(Evidence(self.store.root),bad,self.scope['pulse'])
 def test_raw_HTTP_ignores_prior_completed_nonce_before_add(self):
  self.device.record=dict(nonce='f'*32,state='terminal')
  result=self.run_trial();self.assertEqual(result['status'],'completed',result)
  from learning.manual_collection_dataset import Evidence
  data._http(Evidence(self.store.root),result['receipt'],self.scope['pulse'])
 def audit_full(self):
  # Native recorder lifecycle has separate tests; retain real source/pixel,
  # HTTP, actor, scope and before/after checks in this adapter integration.
  frames=[json.loads(p.read_text()) for p in sorted((self.evidence/'external/host-ingress').glob('0*.json'))]
  shutil.copytree(self.evidence,self.store.root,dirs_exist_ok=True)
  with patch.object(data,'_native',return_value=(frames,dict(validPythonBounds=[0.,1000.]))):
   return data.audit_manual_m4(self.store.root)
 def test_full_raw_schema_success_and_old_policy_relabel_rejected(self):
  result=self.run_trial();report=self.audit_full()
  self.assertEqual(report['executionPolicy'],policy.device_guard_policy())
  scope=copy.deepcopy(self.scope)
  from learning.m4_effective_policy import effective_policy
  scope.update(schemaVersion=core.EFFECTIVE_TASK_SCHEMA,effectivePolicy=effective_policy());scope.pop('deviceGuardPolicy')
  from learning.supervised_collection import plan_digest
  result.update(schemaVersion=scope['schemaVersion'],scopeSha256=plan_digest(scope))
  (self.store.root/'scope.json').write_text(json.dumps(scope));(self.store.root/'result.json').write_text(json.dumps(result))
  with self.assertRaises(ValueError):self.audit_full()
 def test_forged_matching_request_terminal_cannot_escape_raw_or_scope_binding(self):
  result=self.run_trial();self.audit_full()
  result['receipt']['plannedDeviceGoalGuard']['scopeSha256']='f'*64
  result['receipt']['deviceGoalGuard']['scopeSha256']='f'*64
  (self.store.root/'result.json').write_text(json.dumps(result))
  with self.assertRaises(ValueError):self.audit_full()
 def test_bad_terminal_preserves_unknown_and_pending(self):
  self.device.bad=dict(reason='tick_gap')
  result=self.run_trial();self.assertEqual(result['status'],'failed',result)
  self.assertEqual(result['commandOutcome'],'unknown');self.assertEqual(len(self.runs()),1)
  self.assertEqual(fx.CommissioningTests.row(self),'pending')
 def test_stale_before_never_runs(self):
  self.observer.on_before=lambda r:self.clock.sleep(3.1)
  result=self.run_trial();self.assertEqual(result['commandOutcome'],'not_started');self.assertEqual(self.runs(),[])
 def test_failed_scope_is_not_replayed(self):
  self.device.bad=dict(reason='sensor')
  self.run_trial();n=len(self.device.requests)
  with self.assertRaises(ValueError):self.trial.run(confirmed_scope_sha256=self.trial.digest)
  self.assertEqual(len(self.device.requests),n)

class ModelTests(unittest.TestCase):
 def test_policy_separates_model_rows_and_bounds(self):
  context=dict(modelfx.CONTEXT,executionPolicy=policy.device_guard_policy())
  up={k:v for k,v in policy.device_guard_task_pulse('up').items() if k in ('joint','direction','percent','durationMs')}
  down=dict(up,direction='down',percent=35)
  rows=[modelfx.sample(i,up) for i in (0,1)]+[modelfx.sample(i,down,'gripper_supported') for i in (2,3)]
  for row in rows:row['context']=copy.deepcopy(context)
  model=models.fit_m4(rows,context)
  choice=models.select_m4(model,modelfx.state(.5,196.05),modelfx.state(5.5,197.05),[up],context,'table_supported')
  self.assertEqual(choice['selected']['command'],up)
  self.assertEqual(bounded_command('lift',up,context['executionPolicy'])['expectedPwm'],127)
  from learning.m4_effective_policy import effective_policy
  with self.assertRaises(ValueError):models.fit_m4(rows,dict(context,executionPolicy=effective_policy()))
  with self.assertRaises(ValueError):bounded_command('close',up,context['executionPolicy'])

if __name__=='__main__':unittest.main()
