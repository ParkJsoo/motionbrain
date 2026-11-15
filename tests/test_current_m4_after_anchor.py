"""완료 actor after 원본의 후속 정지 기준. 새 촬영·원본 session 재표기 없음."""
import copy,json
import unittest
from unittest.mock import patch
from learning import current_m4_review as review
from learning import m4_up_commissioning as core
from learning.artifacts import ArtifactStore
from tests.test_m4_up_observer import AdapterTests

class AfterAnchorTests(unittest.TestCase):
 def setUp(self):
  f=AdapterTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
  original_save=f.producer.save
  def save(name,value):
   if name.startswith('gripper/') and name.endswith('.json'):
    value=dict(value,method='GET',path='/capture',query={'fresh':'1'})
   return original_save(name,value)
  f.producer.save=save
  observer=f.adapter();actor=core.M4UpCommissioning(f.scope,observer,'private-token',f.store,evidence_root=f.store.root,clock=f.clock)
  actor.port.opener=f.device;result=actor.run(confirmed_scope_sha256=actor.digest)
  self.assertEqual(result['status'],'completed',result)
  self.private=f.repo/'.local';p=self.private/'metrology.json';p.write_text('{}')
  self.metrology=dict(path='metrology.json',sha256=review._sha(p.read_bytes()))
  self.epoch=dict(controllerBootId=f.scope['controllerBootId'],cameraBootId=f.scope['context']['gripperCameraBootId'],
                  outputSequence=f.device.entries[-1]['sequence'],allFiveOff=True)
 def anchor(self):return review.verified_after_anchor(self.f.store.root,self.private,self.epoch,self.metrology)
 def reviewer(self,anchor):
  p=self.private/'anchor.json';p.write_text(json.dumps(anchor));binding=dict(path='anchor.json',sha256=review._sha(p.read_bytes()))
  scope=copy.deepcopy(self.f.scope);scope['hostClockId']='different-next-clock';scope['context']=copy.deepcopy(anchor['context'])
  with patch.object(review,'measure',return_value={}):
   return review.StationaryM4BeforeReviewer(scope,self.private,binding,ArtifactStore(self.private/'next'),
              lambda:self.epoch,lambda *args,**kwargs:None,clock=self.f.clock)
 def test_original_actor_sessions_and_labels_are_preserved(self):
  anchor=self.anchor();reviewer=self.reviewer(anchor)
  self.assertEqual(anchor['schemaVersion'],review.AFTER_SCHEMA)
  self.assertEqual(anchor['context']['supportMode'],'gripper_supported')
  for source in anchor['sources'].values():
   metadata=json.loads((self.private/source['metadata']['path']).read_bytes())
   self.assertEqual(metadata['sessionId'],self.f.scope['hostClockId'])
  self.assertEqual(reviewer.anchor['labels']['tableSupportVerified'],False)
 def test_failed_actor_cannot_be_an_after_anchor(self):
  p=self.f.store.root/'result.json';result=json.loads(p.read_bytes());result['status']='failed';p.write_text(json.dumps(result))
  with self.assertRaisesRegex(ValueError,'completed_actor'):self.anchor()
 def test_changed_cursor_and_labels_are_rejected(self):
  self.epoch['outputSequence']+=1
  with self.assertRaisesRegex(ValueError,'epoch_changed'):self.anchor()
  self.epoch['outputSequence']-=1;anchor=self.anchor();anchor['labels']['tableSupportVerified']=True
  with self.assertRaisesRegex(ValueError,'labels_changed'):self.reviewer(anchor)
 def test_source_preview_from_before_cannot_replace_after(self):
  anchor=self.anchor();old=anchor['sources']['gripper']['image'];path=self.f.store.root/'gripper/esp32-before.jpg'
  old.update(path=path.relative_to(self.private).as_posix(),sha256=review._sha(path.read_bytes()))
  with self.assertRaisesRegex(ValueError,'gripper_binding'):self.reviewer(anchor)
