"""정지·구동 없는 관측 binding과 부분자료·누출·문맥 분리 회귀 검사."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.pipeline_observations import observe_transition


def sha(blob):
    return hashlib.sha256(blob).hexdigest()


class PipelineObservationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()
        (self.root/'native-camera').mkdir()
        (self.root/'session.json').write_text('{"phase":"failed"}')
        self.pixels = bytes([50, 80, 120, 255])*32*32
        self.frames = {}
        for side, lo, hi in [('before', 7., 7.1), ('after', 15., 15.1)]:
            data = side.encode(); name = side+'.raw'
            (self.root/'native-camera'/name).write_bytes(data)
            self.frames[side] = dict(frameId=side, pixelSha256=sha(self.pixels), imageFile=name,
                imageSha256=sha(data), rawByteLength=len(self.pixels), width=32, height=32,
                sourceEarliest=lo, sourceLatest=hi, stdoutReadAt=hi+.01, persistenceMs=1.)
        self.row = dict(sessionId='one', inputs=dict(image=self.frames['before'],
            command=dict(joint='gripper', direction='close', percent=40, durationMs=350)),
            targets=dict(image=self.frames['after']), evidence=dict(commandAt=10., offReceiptAt=12.,
            controllerBootId='boot', commandId='one-001', cameraIdentity=dict(viewId='test')))
        self.context = dict(physicalView='fixed', object='block', poseRegime='M1-only', landmarks='manual-v1')
        self.spec = dict(kind='analyst_points', context=self.context, captureGroup='development', annotations={})
        for side in ('before', 'after'):
            self.spec['annotations'][side] = dict(frameId=side, pixelSha256=sha(self.pixels),
                accepted=True, method='analyst_points', inputFrameOnly=True, sourceFrameIds=[side],
                points=dict(object=[15., 9.], jawLeft=[10., 15.], jawRight=[20., 15.]))
        self.decoder = patch('learning.pipeline_observations.decode', return_value=self.pixels)
        self.decoder.start();self.addCleanup(self.decoder.stop)

    def run_observation(self, row=None, spec=None):
        return observe_transition(self.root, row or self.row, spec or self.spec)

    def test_manual_complete_transitions_are_development_only_and_immutable(self):
        result = self.run_observation()
        self.assertEqual(result['status'], 'eligible')
        self.assertEqual(result['transition']['before'], [0., -6., 10.])
        self.assertTrue(result['trainingEligible'])
        self.assertTrue(result['recoveryDevelopmentOnly'])
        self.assertFalse(result['controlEligible'])
        binding=result['perFrame']['before']['imageBinding']
        self.assertEqual(binding['path'],str(self.root/'native-camera/before.raw'))
        self.assertEqual(binding['pixelSha256'],self.frames['before']['pixelSha256'])
        self.assertEqual(binding['encoding'],'native_bgra_lossless')
        self.assertEqual(json.loads((self.root/'session.json').read_text()), {'phase':'failed'})

    def test_missing_jaw_stays_partial_without_imputation(self):
        spec=copy.deepcopy(self.spec);del spec['annotations']['after']['points']['jawRight']
        result=self.run_observation(spec=spec)
        self.assertEqual(result['status'], 'partial');self.assertIsNone(result['transition'])
        self.assertEqual(result['perFrame']['after']['perPoint']['jawRight']['status'], 'missing_or_invalid')
        self.assertFalse(result['trainingEligible'])

    def test_target_frame_cannot_be_declared_as_input_source(self):
        spec=copy.deepcopy(self.spec);spec['annotations']['before']['sourceFrameIds']=['before','after']
        result=self.run_observation(spec=spec)
        self.assertNotEqual(result['status'], 'eligible')
        self.assertIn('source_mismatch', result['perFrame']['before']['reason'])

    def test_late_input_or_wrong_source_hash_is_rejected(self):
        row=copy.deepcopy(self.row);row['inputs']['image']['stdoutReadAt']=10.
        self.assertIn('not_available',self.run_observation(row=row)['reason'])
        row=copy.deepcopy(self.row);row['sourceHash']='0'*64
        self.assertEqual(self.run_observation(row=row)['reason'],'audited_source_hash_changed')

    def test_manual_contract_cannot_impersonate_registered_tracker(self):
        spec=copy.deepcopy(self.spec);spec['context']['landmarks']='registered-scene-rigid-points-v1:'+'a'*64
        self.assertIn('impersonate', self.run_observation(spec=spec)['reason'])

    def test_annotation_revision_does_not_change_contract_but_scene_does(self):
        first=self.run_observation()
        spec=copy.deepcopy(self.spec);spec['annotations']['after']['points']['jawRight']=[18.,15.]
        second=self.run_observation(spec=spec)
        self.assertEqual(first['contractSha256'],second['contractSha256'])
        spec['context']['physicalView']='camera-moved'
        third=self.run_observation(spec=spec)
        self.assertNotEqual(second['contractSha256'],third['contractSha256'])

    def test_different_manual_protocol_cannot_share_the_same_learning_context(self):
        spec=copy.deepcopy(self.spec);spec['annotationProtocol']='target_assisted_unknown.v2'
        result=self.run_observation(spec=spec)
        self.assertEqual(result['status'],'rejected')
        self.assertEqual(result['reason'],'unsupported_manual_annotation_protocol')
        self.assertIsNone(result['transition'])

    def test_tampered_source_frame_is_not_accepted_even_for_manual_annotations(self):
        (self.root/'native-camera/after.raw').write_bytes(b'tampered')
        result=self.run_observation()
        self.assertNotEqual(result['status'],'eligible')
        self.assertEqual(result['perFrame']['after']['reason'],'observation_asset_hash_changed')

    def test_registered_tracker_failure_is_reported_without_changing_collection(self):
        assets=self.root/'assets';assets.mkdir();reg=b'{}';ref=b'reference'
        (assets/'reg.json').write_bytes(reg);(assets/'ref.jpg').write_bytes(ref)
        spec=dict(kind='registered_scene',context={k:v for k,v in self.context.items() if k!='landmarks'},
            captureGroup='development',registrationPath='reg.json',registrationSha256=sha(reg),
            referenceImagePath='ref.jpg',referenceImageSha256=sha(ref))
        fake=unittest.mock.Mock();fake.landmark_contract='registered-scene-template-points-v1:'+'a'*64
        fake.observe.side_effect=ValueError('scene_match_ambiguous')
        with patch('learning.registered_scene_landmarks.RegisteredSceneLandmarks',return_value=fake):
            result=observe_transition(self.root,self.row,spec,artifact_root=assets)
        self.assertEqual(result['status'],'rejected')
        self.assertEqual(result['perFrame']['after']['reason'],'scene_match_ambiguous')
        self.assertFalse(result['originalOutcomeChanged'])

    def test_real_registered_scene_adapter_binds_both_images_and_automatic_contract(self):
        from learning.registered_scene_landmarks import register
        image=np.random.default_rng(12).integers(0,256,(32,32,3),dtype=np.uint8)
        pixels=cv2.cvtColor(image,cv2.COLOR_BGR2BGRA).tobytes()
        _,encoded=cv2.imencode('.png',image);reference=encoded.tobytes()
        parameters=dict(imageShape=[32,32],
            patchRois=dict(object=[11,4,19,12],jawLeft=[5,14,13,22],jawRight=[20,14,28,22]),
            searchRois=dict(object=[7,0,23,16],jawLeft=[1,10,17,26],jawRight=[16,10,32,26]),
            minimumNcc=dict(object=.85,jawLeft=.94,jawRight=.94),minimumPeakGap=.04,
            peakExclusionRadius=2,minimumTemplateStd=1.5)
        context={k:v for k,v in self.context.items() if k!='landmarks'}
        registration=register(reference,dict(object=[15.,9.],jawLeft=[10.,15.],jawRight=[24.,15.]),
                              parameters,context)
        reg=json.dumps(registration).encode();(self.root/'registration.json').write_bytes(reg)
        (self.root/'reference.png').write_bytes(reference)
        row=copy.deepcopy(self.row)
        for side in ('inputs','targets'):row[side]['image']['pixelSha256']=sha(pixels)
        spec=dict(kind='registered_scene',context=context,captureGroup='development',
            registrationPath='registration.json',registrationSha256=sha(reg),
            referenceImagePath='reference.png',referenceImageSha256=sha(reference))
        with patch('learning.pipeline_observations.decode',return_value=pixels):
            result=self.run_observation(row=row,spec=spec)
        self.assertEqual(result['status'],'eligible',result)
        self.assertTrue(result['transition']['context']['landmarks'].startswith('registered-scene-template-points-v1:'))
        self.assertFalse(result['controlEligible'])

    def test_gripper_bindings_reference_gripper_jpeg_not_external_native_frame(self):
        pair=self.root/'pair';pair.mkdir();(pair/'esp32-before.json').write_text('{}')
        row=copy.deepcopy(self.row)
        row['evidence']['gripperFrameEvidence']=dict(before=dict(persistedAt=8.),
                                                    after=dict(requestStartedAt=16.))
        spec=copy.deepcopy(self.spec);spec['gripperPairRoot']='pair'
        for side in ('before','after'):
            blob=side.encode()+b'-jpeg';(pair/('esp32-'+side+'.jpg')).write_bytes(blob)
            image=row['inputs' if side=='before' else 'targets']['image']
            image['imageSha256']=sha(blob);image['frameId']='gripper-'+side
            spec['annotations'][side]['frameId']='gripper-'+side
            spec['annotations'][side]['sourceFrameIds']=['gripper-'+side]
        with patch('learning.pipeline_observations.bind_gripper_pair',return_value=row):
            result=self.run_observation(spec=spec)
        self.assertEqual(result['status'],'eligible',result)
        binding=result['perFrame']['before']['imageBinding']
        self.assertEqual(binding['path'],str(pair/'esp32-before.jpg'))
        self.assertEqual(binding['frameId'],'gripper-before')
        self.assertEqual(binding['encoding'],'jpeg')
        self.assertNotIn('rawByteLength',binding)


if __name__ == '__main__':unittest.main()
