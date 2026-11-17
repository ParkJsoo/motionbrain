"""복구 영상 반응의 출처·누락·단발 선택 회귀. 합성자료는 실물 성과가 아니다."""
import copy
import unittest
import cv2,numpy as np
from learning import gripper_recovery_response as family
from learning.gripper_recovery_observer import GripperRecoveryObserver
from tests.test_limited_scene_cycle_assessment import external


def near(opened=False):
 image=np.full((240,320,3),180,np.uint8)
 if opened:
  for x in (100,210):cv2.rectangle(image,(x,65),(x+15,145),(20,20,20),-1)
  cv2.rectangle(image,(145,90),(170,120),(100,20,240),-1)
 else:cv2.rectangle(image,(145,65),(175,145),(20,20,20),-1)
 ok,raw=cv2.imencode('.png',image);assert ok
 return GripperRecoveryObserver().observe(raw.tobytes())

class RecoveryResponseTests(unittest.TestCase):
 def setUp(self):
  ext=external();ext.update(virtualProxyPx=[790.,640.],toolAxisImageAngleDeg=-25.)
  self.closed=family.fused(ext,near(),195.11);self.open=family.fused(ext,near(True),195.11)
  self.rows=[]
  for i,direction in enumerate(('open','close','open','close')):
   before,after=(self.closed,self.open) if direction=='open' else (self.open,self.closed)
   self.rows.append(dict(sessionId=str(i),sourceSha256=str(i)*64,command=copy.deepcopy(next(c for c in family.COMMANDS if c['direction']==direction)),before=copy.deepcopy(before),after=copy.deepcopy(after)))
  self.model=family.fit(self.rows,goal=self.open,audit_sources=lambda rows:rows)
 def test_absent_near_object_is_observed_missingness_and_learned_OPEN_beats_STOP(self):
  self.assertEqual(self.closed['near']['state'][-1],0)
  d=family.select(self.model,self.closed,commands_used=0)
  self.assertEqual(d['selected']['command']['direction'],'open');self.assertFalse(d['recoveryCompleted'])
  self.assertFalse(d['physicalGraspVerified']);self.assertFalse(self.model['oldModelsChanged'])
 def test_missing_feature_contract_is_not_replaced_by_external_or_zero(self):
  for near_value in (None,{'rejected':'missing'}, {'state':[0,0,0]}):
   with self.assertRaises((ValueError,TypeError,KeyError)):family.fused(self.closed['external'],near_value,195.11)
  bad=copy.deepcopy(self.closed);bad['state'][4]=0
  with self.assertRaises(ValueError):family.select(self.model,bad,commands_used=0)
 def test_fit_requires_distinct_audited_sources_and_training_goal(self):
  with self.assertRaises(ValueError):family.fit(self.rows,goal=self.open,audit_sources=lambda rows:[])
  with self.assertRaises(ValueError):family.fit(self.rows[:1],goal=self.open,audit_sources=lambda rows:rows)
  changed=copy.deepcopy(self.rows);changed[1]['sourceSha256']=changed[0]['sourceSha256']
  with self.assertRaises(ValueError):family.fit(changed,goal=self.open,audit_sources=lambda rows:rows)
  with self.assertRaises(ValueError):family.fit(self.rows,goal=self.closed,audit_sources=lambda rows:rows)
 def test_pose_or_external_state_out_of_support_rejected(self):
  for field,value in [('m4Deg',196.),('toolAxisImageAngleDeg',-40.)]:
   if field=='m4Deg':bad=family.fused(self.closed['external'],self.closed['near'],value)
   else:
    ext=copy.deepcopy(self.closed['external']);ext[field]=value;bad=family.fused(ext,self.closed['near'],195.11)
   d=family.select(self.model,bad,commands_used=0)
   self.assertEqual(d['selected']['kind'],'stop');self.assertTrue(d['rejected'])
   self.assertEqual(d['selectionOwner'],'admission_guard');self.assertFalse(d['modelSelectedStop']);self.assertFalse(d['modelPredictionPerformed'])
 def test_goal_state_model_STOP_is_distinct_from_support_and_budget_stop(self):
  d=family.select(self.model,self.open,commands_used=0)
  self.assertEqual(d['selected']['kind'],'stop');self.assertTrue(d['modelSelectedStop'])
  self.assertEqual(d['selectionOwner'],'learned_response_comparison');self.assertTrue(d['modelPredictionPerformed'])

 def test_single_use_budget_and_original_weights_remain_unchanged(self):
  before=copy.deepcopy(self.model);d=family.select(self.model,self.closed,commands_used=1)
  self.assertEqual(d['selectionOwner'],'budget_guard');self.assertFalse(d['modelSelectedStop'])
  self.assertEqual(before,self.model)
  for value in (2,-1,True):
   with self.assertRaises(ValueError):family.select(self.model,self.closed,commands_used=value)
