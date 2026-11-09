import copy,hashlib,json
from pathlib import Path
import unittest
import cv2,numpy as np
from tests.test_gripper_camera_dataset import GripperPairTest
from learning.registered_gripper_crosssection import register
from learning.m1_shaft_observations import observe_transition

sha=lambda raw:hashlib.sha256(raw).hexdigest()

class ShaftObservationTests(unittest.TestCase):
 def setUp(self):
  self.fixture=GripperPairTest();self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
  self.root=self.fixture.root;self.source=self.root/'source';self.source.mkdir();(self.source/'session.json').write_text('{"phase":"completed"}')
  self.pair=self.root/'pair';self.pair.mkdir()
  for f in list(self.root.glob('esp32-*')):f.rename(self.pair/f.name)
  self.row=copy.deepcopy(self.fixture.row);self.row['inputs']['command']=dict(joint='gripper',direction='close',percent=40,durationMs=350)
  self.row['sourceHash']=sha((self.source/'session.json').read_bytes());self.row['evidence'].update(controllerBootId='boot',commandId='one')
  self.image=np.full((240,320,3),220,np.uint8);self.image[67:133,140:206]=np.random.default_rng(42).integers(40,220,(66,66,3),dtype=np.uint8)
  self.image[40:67,76:103]=20;self.image[40:67,220:247]=20
  self.context=dict(physicalView='synthetic-shaft',object='block',poseRegime='M1-only')
  reference=cv2.imencode('.png',self.image)[1].tobytes();reg=register(reference,[166,95],self.context);raw=json.dumps(reg).encode()
  (self.root/'reg.json').write_bytes(raw);(self.root/'ref.png').write_bytes(reference)
  self.spec=dict(kind='registered_gripper_shaft_only',context=self.context,captureGroup='development',gripperPairRoot='pair',registrationPath='reg.json',registrationSha256=sha(raw),referenceImagePath='ref.png',referenceImageSha256=sha(reference))
  for side in ['before','after']:self.frame(side,self.image)
 def frame(self,side,image):
  blob=cv2.imencode('.jpg',image)[1].tobytes();(self.pair/f'esp32-{side}.jpg').write_bytes(blob)
  path=self.pair/f'esp32-{side}.json';meta=json.loads(path.read_text());meta['sha256']=sha(blob);path.write_text(json.dumps(meta))
 def observe(self):return observe_transition(self.source,self.row,self.spec,artifact_root=self.root)
 def test_scalar_state_without_object_and_deterministic_processing(self):
  first=self.observe();self.assertEqual(first['status'],'eligible',first.get('reason'));self.assertEqual(first,self.observe())
  row=first['transition'];self.assertEqual(row['context']['stateFields'],['shaftSeparationPx'])
  self.assertNotIn('before',row);self.assertNotIn('after',row);self.assertGreater(row['beforeShaftPx'],140)
  self.assertEqual(first['perFrame']['before']['perPoint']['object'],dict(status='not_required'))
 def test_missing_object_does_not_fill_or_block_valid_shaft_state(self):
  changed=self.image.copy();changed[67:140,130:208]=220;self.frame('after',changed)
  result=self.observe();self.assertEqual(result['status'],'eligible');self.assertNotIn('object',result['perFrame']['after']['annotation']['points'])
 def test_missing_jaw_preserves_partial_without_transition(self):
  changed=self.image.copy();changed[35:90,205:260]=220;self.frame('after',changed)
  result=self.observe();self.assertEqual(result['status'],'partial');self.assertIsNone(result['transition'])
  self.assertEqual(result['perFrame']['after']['perPoint']['jawRight']['status'],'unavailable')
 def test_source_hash_and_causal_time_are_enforced(self):
  self.row['sourceHash']='0'*64;self.assertEqual(self.observe()['reason'],'audited_source_hash_changed')
  self.row['sourceHash']=sha((self.source/'session.json').read_bytes());path=self.pair/'esp32-after.json';meta=json.loads(path.read_text());meta['requestStartedAt']=24.;path.write_text(json.dumps(meta))
  self.assertIn('outside_window',self.observe()['reason'])
 def test_other_joint_is_rejected_without_model_or_transition(self):
  self.row['inputs']['command']=dict(joint='shoulder',direction='down',percent=25,durationMs=350)
  self.assertEqual(self.observe()['reason'],'M1_only')

if __name__=='__main__':unittest.main()
