"""현재 탁상 기준의 명시 정책. 장치 요청·새 학습 없이 모의 계약을 검증한다."""
import copy
import hashlib
import unittest
from learning import m4_device_guard_policy as policy
from learning import m4_device_guard_transport as wire
from learning import m4_up_commissioning as core
from tests.test_m4_device_guard_integration import ActorTests,Device
from tests.test_m4_device_guard_transport import GuardTests

class CurrentTableTests(unittest.TestCase):
 def test_hash_and_explicit_reference_bounds(self):
  self.assertEqual(hashlib.sha256(wire.CURRENT_POLICY_CONTRACT.encode()).hexdigest(),wire.CURRENT_POLICY_SHA256)
  for table in (192.29,194.32,197.39):
   p=policy.current_table_device_guard_policy(table)
   self.assertEqual(policy.validate_device_guard_policy(p),p)
  for table in (None,True,float('nan'),192.28,197.40):
   with self.assertRaises(ValueError):policy.current_table_device_guard_policy(table)
  self.assertEqual(policy.device_guard_policy()['tableReferenceDeg'],197.22)
 def setup_request(self):
  fx=GuardTests();port,device,store,request=fx.setup_port();device.angle=194.32
  request.update(policySha256=wire.CURRENT_POLICY_SHA256,tableReferenceDeg=194.32,
                 expectedStartDeg=194.32,goalDeg=194.95)
  return fx,port,device,store,request
 def test_actual_transport_new_reference_and_old_capability_rejection(self):
  fx,p,c,s,r=self.setup_request();result=fx.run_guard(p,c,r)
  self.assertEqual(result['deviceGoalGuard']['tableReferenceDeg'],194.32)
  fx,p,c,s,r=self.setup_request();envelope=c.envelope
  def old():
   v=envelope();v.pop('supportedPolicySha256');return v
  c.envelope=old
  with self.assertRaisesRegex(ValueError,'policy_capability'):fx.run_guard(p,c,r)
  self.assertFalse(any(d and d.get('action') in ('add_m4_guarded','run_m4_guarded','default') for _,d in c.requests))
 def test_changed_table_and_old_policy_cannot_relabel(self):
  fx,p,c,s,r=self.setup_request()
  for change in ({'tableReferenceDeg':194.31},{'policySha256':wire.POLICY_SHA256},{'goalDeg':197.85}):
   with self.assertRaises(ValueError):wire.validate_request(dict(r,**change),policy.device_guard_task_pulse('up'))
  r.update(expectedStartDeg=195.23,goalDeg=194.32)
  with self.assertRaises(ValueError):wire.validate_request(r,policy.device_guard_task_pulse('down'))

class CurrentTableActorTests(ActorTests):
 def setUp(self):
  super().setUp()
  self.scope.update(expectedStartAngleDeg=194.32,goalAngleDeg=194.95,
                    deviceGuardPolicy=policy.current_table_device_guard_policy(194.32))
  self.device.angle=194.32
  self.trial=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.evidence,clock=self.clock)
  self.trial.port.opener=self.device
 def test_full_raw_schema_success_and_old_policy_relabel_rejected(self):
  result=self.run_trial();report=self.audit_full()
  self.assertEqual(report['executionPolicy'],self.scope['deviceGuardPolicy'])
  changed=copy.deepcopy(result['receipt']);changed['deviceGoalGuard']['tableReferenceDeg']=197.22
  from learning.manual_collection_dataset import Evidence
  from learning import task_transition_data as data
  with self.assertRaises(ValueError):data._http(Evidence(self.store.root),changed,self.scope['pulse'])
 def test_down_actual_angle_cannot_make_goal_unreachable_within_local_limit(self):
  self.scope.update(expectedStartAngleDeg=195.22,goalAngleDeg=194.32,
                    pulse=policy.device_guard_task_pulse('down',self.scope['deviceGuardPolicy']))
  self.scope['context']['supportMode']='gripper_supported'
  self.trial=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.evidence,clock=self.clock)
  self.trial.port.opener=self.device;self.device.angle=195.30
  result=self.run_trial()
  self.assertEqual(result['commandOutcome'],'not_started')
  self.assertTrue(any('current_down_goal_unreachable' in row.get('code','') for row in result['errors']),result)
  self.assertEqual(self.runs(),[])
 def test_camera_boot_and_table_source_mismatch_never_runs(self):
  self.scope['context']['gripperCameraBootId']='f'*32
  self.trial=core.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.evidence,clock=self.clock)
  self.trial.port.opener=self.device
  result=self.run_trial();self.assertEqual(result['commandOutcome'],'not_started');self.assertEqual(self.runs(),[])
