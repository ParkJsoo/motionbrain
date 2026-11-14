"""미래 DOWN 선언만 바꾸는 명시 proof. 합성 원본, 장치/네트워크 요청 없음."""
import copy
import unittest
from learning import current_short_policy_transition as transition
from learning import task_transition_data as data
from learning.current_m4_short import current_short_policy
from learning.pipeline_learning import digest
from learning.task_action_models import fit_m4
from tests import test_m4_current_short as fixtures

class FutureDownTransitionTests(unittest.TestCase):
 def setUp(self):
  owner=fixtures.ShortRawData600Tests();owner.setUp();self.addCleanup(owner.doCleanups)
  self.owner=owner;self.root=owner.o.root
  self.registration,self.annotation=owner.o.annotations()
  self.registration['context']['executionPolicy']=current_short_policy(600,250)
  self.annotation['registrationSha256']=digest(self.registration)
  self.original=data.build_manual_m4_row(self.root,self.registration,self.annotation)
  report=owner.o.audit();self.target=transition.target_registration(self.registration,report['originalContext'],down_duration_ms=350)
 def proof(self):
  return transition.make_equivalence_proof(self.root,self.registration,self.target,self.annotation,
   review_statement='독립 검토용 원UP600 실행계약 동일, 미래DOWN250→350만 명시 변경')
 def test_explicit_proof_preserves_raw_row_state_and_source_identity(self):
  before={str(p):p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
  proof=self.proof();derived=transition.derive_up_row(self.root,self.registration,self.target,self.annotation,proof=proof)
  row=derived['row']
  for key in ('id','command','before','after','episode','frameIds','supportMode'):
   self.assertEqual(row[key],self.original[key])
  self.assertEqual(row['context']['executionPolicy'],current_short_policy(600,350))
  self.assertNotEqual(row['sourceHash'],self.original['sourceHash'])
  self.assertFalse(derived['audit']['independentNewSource'])
  self.assertEqual(before,{str(p):p.read_bytes() for p in self.root.rglob('*') if p.is_file()})
  with self.assertRaisesRegex(ValueError,'reused_id'):fit_m4([row,row],row['context'])
 def test_missing_proof_and_old_strict_builder_still_reject(self):
  with self.assertRaisesRegex(ValueError,'proof_required'):
   transition.derive_up_row(self.root,self.registration,self.target,self.annotation)
  annotation=dict(self.annotation,registrationSha256=digest(self.target))
  with self.assertRaisesRegex(ValueError,'execution_policy_changed'):
   data.build_manual_m4_row(self.root,self.target,annotation)
 def test_tampered_proof_rejected(self):
  proof=self.proof();proof['sourceRowSha256']='f'*64
  with self.assertRaisesRegex(ValueError,'proof_changed'):
   transition.derive_up_row(self.root,self.registration,self.target,self.annotation,proof=proof)
 def test_changed_UP_goal_camera_or_measurement_cannot_transfer(self):
  for mode in ('goal','camera','method'):
   target=copy.deepcopy(self.target)
   if mode=='goal':target['context']['executionPolicy']['upGoalAngleDeg']=197.9
   elif mode=='camera':target['cameraBinding']['gripper']['bootId']='f'*32
   else:target['measurementMethod']='other_method'
   with self.subTest(mode=mode),self.assertRaisesRegex(ValueError,'target_registration_changed'):
    transition.make_equivalence_proof(self.root,self.registration,target,self.annotation,review_statement='명시검토')
 def test_failed_or_model_selected_original_never_transfers(self):
  original=self.owner.o.fixture.read('result.json')
  for changes in ({'status':'failed'},{'modelSelected':True}):
   self.owner.o.fixture.write('result.json',dict(original,**changes))
   with self.subTest(changes=changes),self.assertRaisesRegex(ValueError,'completed_manual_raw_required'):
    self.proof()
   self.owner.o.fixture.write('result.json',original)
 def test_only_future_down300_or350_accepted(self):
  for cap in (300,350):
   target=transition.target_registration(self.registration,self.owner.o.audit()['originalContext'],down_duration_ms=cap)
   proof=transition.make_equivalence_proof(self.root,self.registration,target,self.annotation,review_statement='명시미래방향검토')
   self.assertEqual(proof['changedFutureFields'],['downDurationCapMs','downMonitorTimingPolicy'])
  with self.assertRaises(ValueError):transition.target_registration(self.registration,{},down_duration_ms=400)
 def test_UTF_measurement_contract_uses_original_plan_digest(self):
  from learning.supervised_collection import plan_digest
  registration=copy.deepcopy(self.registration);annotation=copy.deepcopy(self.annotation)
  registration['measurementMethod']='수동_픽셀_계측'
  annotation.update(measurementMethod=registration['measurementMethod'],registrationSha256=plan_digest(registration))
  source=data.build_manual_m4_row(self.root,registration,annotation)
  target=transition.target_registration(registration,self.owner.o.audit()['originalContext'],down_duration_ms=350)
  proof=transition.make_equivalence_proof(self.root,registration,target,annotation,review_statement='한국어 원계측 계약을 보존한다')
  derived=transition.derive_up_row(self.root,registration,target,annotation,proof=proof)['row']
  self.assertEqual(derived['context']['registrationSha256'],plan_digest(target))
  self.assertEqual(derived['before'],source['before']);self.assertEqual(derived['frameIds'],source['frameIds'])
