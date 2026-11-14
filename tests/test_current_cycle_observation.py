"""새 정지판독→현재 두영상+연속PWM0 cursor 결합. 장치 요청 없음."""
import copy
import unittest
from learning.current_cycle_observation import (StationaryReviewAnchor,ANCHOR_SCHEMA,
                                               verify_zero_output_bridge)

class Clock:
 def __init__(self):self.at=100.
 def monotonic(self):return self.at

class StationaryAnchorTests(unittest.TestCase):
 def setUp(self):
  self.clock=Clock();self.audits=[]
  self.old=self.pair(12,90.,'anchor')
  self.current=self.pair(12,99.,'fresh')
  self.anchor=dict(schemaVersion=ANCHOR_SCHEMA,anchorId='new-static-anchor-01',pair=self.old,
   review=dict(context=copy.deepcopy(self.old['context']),stable=True,
    onsite=dict(operatorPresent=True,clearanceVerified=True,jawContactVerified=True,tableSupportVerified=True)),
   tolerances=dict(externalObjectBox=1.,externalToolCenter=1.,gripperObjectBox=1.,gripperReferencePoints=.5,M4AngleDeg=.1),maxFrameAgeSeconds=3.)
  self.proof=dict(controllerBootId='a'*32,beforeSequence=12,afterSequence=12,dropped=0,
                  allFiveOff=True,state='ARMED',queueCount=1,entries=[],observedAt=99.1)
  self.guard=StationaryReviewAnchor(self.anchor,audit_pair=self.audit_pair,
                  audit_cursor=self.audit_cursor,clock=self.clock)
 def pair(self,seq,at,prefix):
  return dict(controllerBootId='a'*32,cameraBootId='b'*32,outputSequence=seq,
   outputProofObservedAt=at+.1,context=dict(object='current-pink',physicalView='current-external',poseRegime='current-M4'),
   reviewFeatures=dict(externalObjectBox=[100.,100.,50.,100.],externalToolCenter=[120.,150.],gripperObjectBox=[130.,90.,50.,55.],gripperReferencePoints=[110.,75.,200.,75.],M4AngleDeg=195.29),
   frames=[dict(camera=cam,frameId=prefix+'-'+cam,pixelSha256=('c' if cam=='external' else 'd')*64,
                 sourceEarliest=at,sourceLatest=at+.01,persistedAt=at+.02) for cam in ['external','gripper']])
 def audit_pair(self,pair):self.audits.append(('pair',copy.deepcopy(pair)));return True
 def audit_cursor(self,proof):self.audits.append(('cursor',copy.deepcopy(proof)));return True
 def review(self):return self.guard.review(self.current,self.proof,deadline=110.,tick=lambda:None)
 def test_same_output_epoch_uses_fresh_pair_and_keeps_original_review(self):
  original=copy.deepcopy(self.anchor)
  decision,evidence=self.review()
  self.assertEqual(decision,self.anchor['review'])
  self.assertTrue(evidence['humanAssisted']);self.assertFalse(evidence['originalActorOutcomeChanged'])
  self.assertEqual(self.anchor,original)
  self.assertEqual([kind for kind,_ in self.audits],['pair','pair','pair','cursor'])
 def test_zero_only_contiguous_bridge_accepts_ARM_OFF_writes(self):
  self.current['outputSequence']=14;self.proof.update(afterSequence=14,entries=[
    dict(sequence=13,motor=1,pwm=0),dict(sequence=14,motor=4,pwm=0)])
  self.assertTrue(self.review()[0]['onsite']['jawContactVerified'])
 def test_nonzero_output_forbids_label_reuse_even_if_images_look_same(self):
  self.current['outputSequence']=13;self.proof.update(afterSequence=13,entries=[dict(sequence=13,motor=4,pwm=1)])
  with self.assertRaisesRegex(ValueError,'new_motion'):self.review()
 def test_dropped_or_gapped_or_reordered_cursor_forbidden(self):
  for mutation in [dict(dropped=1),dict(afterSequence=13,entries=[]),
                   dict(afterSequence=14,entries=[dict(sequence=14,motor=1,pwm=0),dict(sequence=13,motor=2,pwm=0)])]:
   with self.subTest(mutation=mutation):
    proof=copy.deepcopy(self.proof);proof.update(mutation)
    current=copy.deepcopy(self.current);current['outputSequence']=proof['afterSequence']
    guard=StationaryReviewAnchor(self.anchor,audit_pair=self.audit_pair,audit_cursor=self.audit_cursor,clock=self.clock)
    with self.assertRaises(ValueError):guard.review(current,proof,deadline=110.,tick=lambda:None)
 def test_changed_current_scene_or_stationary_geometry_is_rejected(self):
  for key,value in [('cameraBootId','e'*32),('controllerBootId','e'*32),
                    ('context',dict(object='different'))]:
   current=copy.deepcopy(self.current);current[key]=value
   guard=StationaryReviewAnchor(self.anchor,audit_pair=self.audit_pair,audit_cursor=self.audit_cursor,clock=self.clock)
   with self.assertRaises(ValueError):guard.review(current,self.proof,deadline=110.,tick=lambda:None)
  self.current['reviewFeatures']['gripperReferencePoints'][0]=110.6
  with self.assertRaisesRegex(ValueError,'stationary_changed'):self.review()
 def test_stale_or_duplicate_or_regressed_source_never_becomes_current(self):
  for mutation in ['stale','duplicate','regressed']:
   current=copy.deepcopy(self.current)
   if mutation=='stale':
    for f in current['frames']:f.update(sourceEarliest=96.,sourceLatest=96.01,persistedAt=96.02)
   elif mutation=='duplicate':current['frames'][0]['frameId']=self.old['frames'][0]['frameId']
   else:
    for f in current['frames']:f.update(sourceEarliest=89.,sourceLatest=89.01,persistedAt=89.02)
   guard=StationaryReviewAnchor(self.anchor,audit_pair=self.audit_pair,audit_cursor=self.audit_cursor,clock=self.clock)
   with self.assertRaises(ValueError):guard.review(current,self.proof,deadline=110.,tick=lambda:None)
 def test_raw_audit_is_required_not_only_flags(self):
  self.guard.audit_cursor=lambda proof:False
  with self.assertRaisesRegex(ValueError,'raw_audit'):self.review()
 def test_latest_current_OFF_and_queue_are_mandatory(self):
  for mutation in [dict(allFiveOff=False),dict(queueCount=0),dict(state='RUNNING'),dict(controllerBootId='f'*32)]:
   proof=copy.deepcopy(self.proof);proof.update(mutation)
   guard=StationaryReviewAnchor(self.anchor,audit_pair=self.audit_pair,audit_cursor=self.audit_cursor,clock=self.clock)
   with self.assertRaises(ValueError):guard.review(self.current,proof,deadline=110.,tick=lambda:None)
 def test_policy_mutation_or_oversized_stationary_tolerance_is_rejected(self):
  self.guard.anchor['tolerances']['gripperReferencePoints']=1.
  with self.assertRaisesRegex(ValueError,'anchor_changed'):self.review()
  bad=copy.deepcopy(self.anchor);bad['tolerances']['gripperReferencePoints']=2.
  with self.assertRaisesRegex(ValueError,'tolerances'):
   StationaryReviewAnchor(bad,audit_pair=self.audit_pair,audit_cursor=self.audit_cursor,clock=self.clock)
 def test_tick_or_audit_time_cannot_silently_expire_current_frame(self):
  def tick():self.clock.at+=3.
  with self.assertRaisesRegex(ValueError,'pair_stale'):
   self.guard.review(self.current,self.proof,deadline=110.,tick=tick)
 def test_no_automatic_positive_physical_labels(self):
  bad=copy.deepcopy(self.anchor);bad['review'].pop('onsite')
  with self.assertRaisesRegex(ValueError,'visual_labels'):
   StationaryReviewAnchor(bad,audit_pair=self.audit_pair,audit_cursor=self.audit_cursor,clock=self.clock)

 def test_incomplete_dual_geometry_cannot_transfer_contact_label(self):
  bad=copy.deepcopy(self.anchor)
  bad['pair']['reviewFeatures']={'shaft':69.}
  bad['tolerances']={'shaft':.5}
  with self.assertRaisesRegex(ValueError,'dual_view_features'):
   StationaryReviewAnchor(bad,audit_pair=self.audit_pair,audit_cursor=self.audit_cursor,clock=self.clock)

 def test_success_and_rejected_review_are_both_single_use(self):
  self.review()
  with self.assertRaisesRegex(ValueError,'one_shot_consumed'):self.review()
  guard=StationaryReviewAnchor(self.anchor,audit_pair=self.audit_pair,audit_cursor=self.audit_cursor,clock=self.clock)
  bad=copy.deepcopy(self.current);bad['reviewFeatures']['M4AngleDeg']+=.2
  with self.assertRaisesRegex(ValueError,'stationary_changed'):
   guard.review(bad,self.proof,deadline=110.,tick=lambda:None)
  with self.assertRaisesRegex(ValueError,'one_shot_consumed'):
   guard.review(self.current,self.proof,deadline=110.,tick=lambda:None)

 def test_cursor_proof_must_be_after_current_images_and_fresh(self):
  for at in [98.,101.,float('nan')]:
   guard=StationaryReviewAnchor(self.anchor,audit_pair=self.audit_pair,audit_cursor=self.audit_cursor,clock=self.clock)
   proof=copy.deepcopy(self.proof);proof['observedAt']=at
   current=copy.deepcopy(self.current);current['outputProofObservedAt']=at
   with self.assertRaisesRegex(ValueError,'cursor_not_current'):
    guard.review(current,proof,deadline=110.,tick=lambda:None)
