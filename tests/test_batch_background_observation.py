"""합성 카메라 변형과 독립 전경 움직임을 구분한다. 실물 정확도 주장이 아님."""
import copy
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from learning.batch_background_observation import fit_background, validate_transform, REGIONS, PARAMETERS

class BackgroundTests(unittest.TestCase):
    def setUp(self):
        self.reference=np.random.default_rng(123).integers(0,256,(1080,1920,3),dtype=np.uint8)
        self.camera=cv2.getRotationMatrix2D((960,540),.02,1.008)

    def test_camera_affine_keeps_independent_foreground_motion(self):
        scene=self.reference.copy();patch_image=scene[400:500,800:900].copy()
        scene[400:500,800:900]=0;scene[400:500,812:912]=patch_image
        current=cv2.warpAffine(scene,self.camera,(1920,1080))
        fit=fit_background(self.reference,current)
        actual=np.asarray(fit['referenceToCurrent'])
        probe=np.array([900.,450.,1.])
        self.assertLess(np.linalg.norm(actual@probe-self.camera@probe),.3)
        normalized=cv2.warpAffine(current,np.asarray(fit['currentToReference']),(1920,1080))
        scores=cv2.matchTemplate(normalized[390:510,790:930],patch_image,cv2.TM_CCOEFF_NORMED)
        _,score,_,location=cv2.minMaxLoc(scores)
        self.assertGreater(score,.7)
        self.assertEqual(location,(22,10)) # original foreground moved +12 in reference coordinates
        x=np.asarray(fit['referenceFeatures']);y=np.asarray(fit['currentFeatures'])
        for points in (x,y):
            inside=np.zeros(len(points),bool)
            for x0,y0,x1,y1 in REGIONS:
                margin=PARAMETERS['maskInsetPx']
                inside|=(points[:,0]>=x0+margin)&(points[:,0]<x1-margin)&(points[:,1]>=y0+margin)&(points[:,1]<y1-margin)
            self.assertTrue(inside.all())

    def test_large_camera_change_and_nonrigid_background_fail_closed(self):
        for matrix in (cv2.getRotationMatrix2D((960,540),0,1.03),
                       cv2.getRotationMatrix2D((960,540),1,1),np.array([[1.,.02,0],[0,1,0]]),
                       np.array([[1.,0,3],[0,1,0]])):
            with self.assertRaises(ValueError):validate_transform(matrix,(1080,1920))
        current=self.reference.copy()
        current[:900,:430]=np.roll(current[:900,:430],7,axis=0)
        with self.assertRaises(ValueError):fit_background(self.reference,current)

    def test_textureless_or_missing_background_never_fits_foreground(self):
        blank=np.zeros_like(self.reference);blank[300:800,600:1400]=self.reference[300:800,600:1400]
        with self.assertRaisesRegex(ValueError,'features_insufficient'):fit_background(blank,blank)

    def test_runtime_rejects_new_schema_even_with_legacy_admission(self):
        from tests.test_batch_runtime import BatchRuntimeTests
        from learning.batch_runtime import validate_runtime,sha
        from learning.experiment_pipeline import encoded
        fixture=BatchRuntimeTests();fixture.setUp();self.addCleanup(fixture.doCleanups)
        spec=copy.deepcopy(fixture.spec);spec['schemaVersion']='motionbrain.batch_background_registered_observer.v1'
        fixture.spec_path.write_bytes(encoded(spec))
        for key in ('motionAdmission','collectionAdmission'):
            plan=copy.deepcopy(fixture.plan);plan['runtime'][key]={}
            plan['runtime']['observerSpecSha256']=sha(fixture.spec_path.read_bytes())
            with patch('learning.batch_runtime.SequencePort',side_effect=AssertionError('hardware')):
                with self.assertRaisesRegex(ValueError,'development_only_no_live_dispatch'):
                    validate_runtime(plan,fixture.repository)

    def test_observation_preserves_source_binding_and_separates_derived_coordinates(self):
        import base64
        import hashlib
        from tests import test_batch_template_observation as fixture
        from learning.batch_background_observation import make_spec
        from learning.batch_observation import observe_frame,digest
        fixture.RegisteredBatchObservationTests.setUpClass()
        legacy=copy.deepcopy(fixture.RegisteredBatchObservationTests.spec)
        image=fixture.RegisteredBatchObservationTests.image.copy()
        rng=np.random.default_rng(14)
        for x0,y0,x1,y1 in REGIONS:
            gray=rng.integers(40,220,(y1-y0,x1-x0),dtype=np.uint8)
            image[y0:y1,x0:x1]=np.repeat(gray[:,:,None],3,axis=2)
        bgra=cv2.cvtColor(image,cv2.COLOR_BGR2BGRA);png=cv2.imencode('.png',bgra)[1].tobytes()
        legacy.update(referenceImageBase64=base64.b64encode(png).decode(),referenceImageSha256=hashlib.sha256(png).hexdigest(),
                      referencePixelSha256=hashlib.sha256(bgra.tobytes()).hexdigest())
        legacy['joints']={'gripper':legacy['joints']['wrist']}
        spec=make_spec(legacy);current=cv2.warpAffine(image,self.camera,(1920,1080))
        raw=cv2.imencode('.png',current)[1].tobytes()
        frame=dict(frameId='raw:1',imageSha256=hashlib.sha256(raw).hexdigest(),pixelSha256=hashlib.sha256(current.tobytes()).hexdigest(),
            capturedAt=10.,persistedAt=10.1,clockId='clock',cameraIdentity='external')
        observed=observe_frame(raw,frame,spec,fixture.RegisteredBatchObservationTests.context,11.,'clock')
        self.assertEqual(observed['status'],'accepted',observed.get('reason'))
        self.assertEqual(observed['frameBinding'],frame)
        self.assertNotEqual(observed['normalizedFrameBinding']['imageSha256'],frame['imageSha256'])
        self.assertEqual(observed['specSha256'],digest(spec))
        self.assertIn('derived_pixels',observed['normalizedFrameBindingSemantics'])
        self.assertTrue(observed['rawCurrentLandmarks']);self.assertTrue(observed['mappedCurrentLandmarks'])
        self.assertIn('excludes_uncalibrated',observed['landmarks']['wrist']['uncertaintyMeaning'])
        for key in ('controlEligible','trainingEligible','motionTrackingVerified','physicalPoseEstablished'):
            self.assertFalse(observed[key])
        altered=copy.deepcopy(spec);altered['backgroundNormalization']['parameters']['maximumCenterShiftPx']=10
        self.assertEqual(observe_frame(raw,frame,altered,fixture.RegisteredBatchObservationTests.context,11.,'clock')['status'],'rejected')

    def test_pixels_outside_declared_background_do_not_influence_fit(self):
        current=cv2.warpAffine(self.reference,self.camera,(1920,1080))
        baseline=fit_background(self.reference,current)
        mask=np.zeros((1080,1920),bool)
        for x0,y0,x1,y1 in REGIONS:mask[y0:y1,x0:x1]=True
        changed_reference=self.reference.copy();changed_current=current.copy()
        changed_reference[~mask]=0;changed_current[~mask]=255
        changed=fit_background(changed_reference,changed_current)
        np.testing.assert_allclose(changed['referenceToCurrent'],baseline['referenceToCurrent'],atol=1e-9)
        np.testing.assert_array_equal(changed['referenceFeatures'],baseline['referenceFeatures'])
        np.testing.assert_array_equal(changed['currentFeatures'],baseline['currentFeatures'])

    def test_replay_source_paths_cannot_escape_declared_capture(self):
        import tempfile
        from pathlib import Path
        from tools.replay_background_observation import source_file
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory).resolve()
            for name in ('../outside.json',str(root/'absolute.json')):
                with self.assertRaises(ValueError):source_file(root,name)
