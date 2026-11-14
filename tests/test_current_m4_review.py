"""새 정지 anchor와 현재 원본 검증. 실제 카메라·컨트롤러 요청은 없다."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.artifacts import ArtifactStore
from learning.current_m4_review import StationaryM4BeforeReviewer,SCHEMA,same_picture


def sha(raw):return hashlib.sha256(raw).hexdigest()
def encode(image,extension='.png'):
    ok,raw=cv2.imencode(extension,image);assert ok
    return raw.tobytes()


class StationaryM4ReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.store=ArtifactStore(self.root/'current')
        rng=np.random.default_rng(13)
        self.ext=encode(cv2.cvtColor(rng.integers(0,255,(1080,1920,3),np.uint8),cv2.COLOR_BGR2BGRA))
        self.grip=encode(rng.integers(0,255,(240,320,3),np.uint8),'.jpg')
        self.scope=dict(controllerBootId='a'*32,hostClockId='current-session',maxObservationAgeSeconds=10.,
            context=dict(object='current-object',gripperCameraBootId='b'*32))
        self.epoch=dict(controllerBootId='a'*32,cameraBootId='b'*32,outputSequence=17,allFiveOff=True)
        self.extmeta=dict(sessionId='M4-stationary-anchor-synthetic',pixelSha256=sha(cv2.imdecode(np.frombuffer(self.ext,np.uint8),cv2.IMREAD_UNCHANGED).tobytes()),
            configurationSha256='c'*64,videoMirrored=False,videoRotationAngle=0)
        self.gripmeta=dict(sessionId='M4-stationary-anchor-synthetic',sha256=sha(self.grip),method='GET',path='/capture',query={'fresh':'1'},
            headers={'X-MotionBrain-Camera-Boot-Id':'b'*32,'X-MotionBrain-Capture-Mode':'after_request',
                     'X-MotionBrain-Timestamp-Semantics':'first_dma_buffer'})
        self.anchor=dict(schemaVersion=SCHEMA,context=copy.deepcopy(self.scope['context']),epoch=copy.deepcopy(self.epoch),
            labels=dict(operatorPresent=True,clearanceVerified=True,jawContactVerified=True,tableSupportVerified=False),
            sources=dict(external=dict(image=self.write('anchor-external.png',self.ext),metadata=self.json('anchor-external.json',self.extmeta)),
                         gripper=dict(image=self.write('anchor-gripper.jpg',self.grip),metadata=self.json('anchor-gripper.json',self.gripmeta))),
            metrologyRegistration=self.json('metrology.json',dict(synthetic=True)),
            reviewBasis='root_visual_stationary_anchor_review',humanAssisted=True)
        self.binding=self.json('anchor.json',self.anchor)
        self.store._write('before-external.png',self.ext);self.store._write('before-gripper.jpg',self.grip)
        self.request=dict(label='before',hostClockId='current-session',sources=dict(external={},gripper=dict(imageSha256=sha(self.grip))),
            previews=dict(external='before-external.png',gripper='before-gripper.jpg'),sourceEarliest=99.)
        self.after_calls=[]
        self.clock=type('Clock',(),{'monotonic':lambda owner:100.})()
        self.measure=patch('learning.current_m4_review.measure',return_value=dict(state=dict(objectRisePx=0.,toolRisePx=0.,slipDyPx=0.)))
        self.mock_measure=self.measure.start();self.addCleanup(self.measure.stop)
        self.extpatch=patch('learning.current_m4_review._external_frame',return_value=dict(self.extmeta,sourceEarliest=99.))
        self.extpatch.start();self.addCleanup(self.extpatch.stop)
        self.grippatch=patch('learning.current_m4_review._gripper_frame',return_value=dict(sourceEarliest=99.))
        self.grippatch.start();self.addCleanup(self.grippatch.stop)
    def write(self,name,raw):
        (self.root/name).write_bytes(raw);return dict(path=name,sha256=sha(raw))
    def json(self,name,value):return self.write(name,json.dumps(value).encode())
    def reviewer(self,read_epoch=None):
        return StationaryM4BeforeReviewer(self.scope,self.root,self.binding,self.store,
            read_epoch or (lambda:copy.deepcopy(self.epoch)),
            lambda request,**kwargs:self.after_calls.append(request) or {'after':'existing-reviewer'},clock=self.clock)
    def call(self,reviewer,request=None):return reviewer(request or self.request,deadline=105.,tick=lambda:None)

    def test_fresh_stationary_before_preserves_false_labels_then_after_uses_existing_reviewer(self):
        reviewer=self.reviewer();decision=self.call(reviewer)
        self.assertTrue(decision['stable']);self.assertFalse(decision['onsite']['tableSupportVerified'])
        self.assertEqual(decision['onsite'],self.anchor['labels'])
        self.assertEqual(self.call(reviewer,dict(label='after')),{'after':'existing-reviewer'})
        with self.assertRaisesRegex(ValueError,'consumed'):self.call(reviewer)
        proof=json.loads((self.store.root/'before-stationary-anchor-verification.json').read_text())
        self.assertTrue(proof['humanAssisted']);self.assertEqual(proof['anchorBinding'],self.binding)

    def test_any_cursor_change_even_zero_only_and_reboot_rejected(self):
        for key,value in [('outputSequence',18),('controllerBootId','d'*32),('cameraBootId','d'*32),('allFiveOff',False)]:
            epoch=dict(self.epoch);epoch[key]=value
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'epoch_changed'):
                self.call(self.reviewer(lambda:epoch))

    def test_expired_FileReviewer_format_cannot_be_used_as_anchor(self):
        old=dict(requestSha256='e'*64,nonce='expired',decision=dict(context=self.scope['context'],stable=True,onsite=self.anchor['labels']))
        self.binding=self.json('old-expired-review.json',old)
        with self.assertRaisesRegex(ValueError,'new_stationary_anchor'):self.reviewer()

    def test_bound_anchor_or_image_change_rejected_before_labels(self):
        reviewer=self.reviewer();(self.root/'anchor-gripper.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'source_changed'):self.call(reviewer)

    def test_fresh_preview_must_match_native_raw_pixel_SHA(self):
        reviewer=self.reviewer();image=cv2.imdecode(np.frombuffer(self.ext,np.uint8),cv2.IMREAD_UNCHANGED)
        image[100,100,0]^=1
        (self.store.root/'before-external.png').write_bytes(encode(image))
        with self.assertRaisesRegex(ValueError,'preview_source_changed'):self.call(reviewer)

    def test_gray_same_picture_gate_rejects_visible_change_without_threshold_relaxation(self):
        image=cv2.imdecode(np.frombuffer(self.grip,np.uint8),cv2.IMREAD_UNCHANGED)
        image[70:150,80:220]=255
        with self.assertRaisesRegex(ValueError,'picture_changed'):
            same_picture(self.grip,encode(image,'.jpg'),'gripper')
        self.assertGreaterEqual(same_picture(self.grip,self.grip,'gripper')['ncc'],.995)

    def test_metrology_difference_rejects_despite_same_picture_gate(self):
        reviewer=self.reviewer();zero=dict(state=dict(objectRisePx=0.,toolRisePx=0.,slipDyPx=0.))
        changed=dict(state=dict(objectRisePx=1.01,toolRisePx=0.,slipDyPx=0.))
        self.mock_measure.side_effect=[zero,zero,changed]
        with self.assertRaisesRegex(ValueError,'metrology_changed'):self.call(reviewer)

    def test_root_labels_and_current_external_configuration_cannot_change(self):
        self.anchor['labels']['tableSupportVerified']=1;self.binding=self.json('bad-label.json',self.anchor)
        with self.assertRaisesRegex(ValueError,'explicit_root_labels'):self.reviewer()

    def test_anchor_external_PNG_must_match_original_metadata_pixels(self):
        self.extmeta['pixelSha256']='f'*64
        self.anchor['sources']['external']['metadata']=self.json('bad-ext-meta.json',self.extmeta)
        self.binding=self.json('bad-anchor.json',self.anchor)
        with self.assertRaisesRegex(ValueError,'anchor_external_pixel_binding'):self.reviewer()

    def test_failed_before_cannot_delegate_after_or_retake_before(self):
        reviewer=self.reviewer(lambda:dict(self.epoch,outputSequence=18))
        with self.assertRaises(ValueError):self.call(reviewer)
        with self.assertRaisesRegex(ValueError,'before_not_accepted'):self.call(reviewer,dict(label='after'))
        with self.assertRaisesRegex(ValueError,'consumed'):self.call(reviewer)
        self.assertEqual(self.after_calls,[])

    def test_three_second_freshness_is_not_relaxed_to_actor_ten_seconds(self):
        self.clock.monotonic=lambda:102.01
        with self.assertRaisesRegex(ValueError,'fresh_frames_expired'):self.call(self.reviewer())

    def test_slow_evidence_write_cannot_return_accepted_decision(self):
        now=[100.];self.clock.monotonic=lambda:now[0]
        original=self.store.write_json
        def slow(name,value):
            result=original(name,value);now[0]=102.01;return result
        self.store.write_json=slow;reviewer=self.reviewer()
        with self.assertRaisesRegex(ValueError,'evidence_write_expired'):self.call(reviewer)
        self.assertFalse(reviewer.accepted)

    def test_RUN_gate_is_memory_only_and_single_use(self):
        reviewer=self.reviewer();self.call(reviewer)
        with patch('learning.current_m4_review._read',side_effect=AssertionError('RUN file IO')):
            reviewer.check_before_run()
        with self.assertRaisesRegex(ValueError,'RUN_gate_consumed'):reviewer.check_before_run()

    def test_RUN_gate_rejects_elapsed_three_seconds_or_mutated_memory(self):
        reviewer=self.reviewer();self.call(reviewer);self.clock.monotonic=lambda:102.01
        with self.assertRaisesRegex(ValueError,'RUN_freshness_expired'):reviewer.check_before_run()
        self.store=ArtifactStore(self.root/'second-current')
        self.store._write('before-external.png',self.ext);self.store._write('before-gripper.jpg',self.grip)
        self.clock.monotonic=lambda:100.
        reviewer=self.reviewer();self.call(reviewer)
        reviewer.anchor['labels']['jawContactVerified']=False
        with self.assertRaisesRegex(ValueError,'RUN_memory_changed'):reviewer.check_before_run()

    def test_expired_actor_raw_images_cannot_be_repackaged_as_new_anchor(self):
        self.extmeta['sessionId']='expired-M4-trial'
        self.anchor['sources']['external']['metadata']=self.json('old-actor-meta.json',self.extmeta)
        self.binding=self.json('repackaged-anchor.json',self.anchor)
        with self.assertRaisesRegex(ValueError,'independent_anchor_session'):self.reviewer()
