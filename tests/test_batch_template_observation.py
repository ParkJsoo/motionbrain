import base64
import copy
import hashlib
import unittest

import cv2
import numpy as np

from learning.batch_observation import observe_frame, validate_spec
from learning.batch_template_observation import SCHEMA, registration_implementation_sha256


class RegisteredBatchObservationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.image=np.full((1080,1920,3),40,np.uint8)
        cls.points=dict(backgroundA=(120,120),backgroundB=(220,120),forearmA=(450,350),
                        forearmB=(650,350),wrist=(850,350),upperArm=(1050,350))
        rng=np.random.default_rng(23)
        for x,y in cls.points.values():
            gray=rng.integers(40,220,(69,69),dtype=np.uint8)
            cls.image[y-34:y+35,x-34:x+35]=np.repeat(gray[:,:,None],3,axis=2)
        pink=cv2.cvtColor(np.uint8([[[170,255,255]]]),cv2.COLOR_HSV2BGR)[0,0]
        cls.image[680:720,1190:1210]=pink
        reference=cv2.cvtColor(cls.image,cv2.COLOR_BGR2BGRA)
        reference_png=cv2.imencode('.png',reference)[1].tobytes()
        cls.context=dict(viewId='stationary-view',poseRegime='local',object='pink-lower',load='empty')
        landmarks={name:dict(kind='template',point=list(point),reference=name.startswith('background'),
            uncertaintyPx=2,role='fixed_view_anchor' if name.startswith('background') else 'housing_not_tip')
            for name,point in cls.points.items()}
        landmarks['object']=dict(kind='hsv',hsvLower=[164,150,100],hsvUpper=[175,255,255],
            minAreaPx=600,maxAreaPx=1000,uncertaintyPx=2,role='visible_lower_surface_not_body_center')
        pose=['forearmA','forearmB','wrist','upperArm']
        features=[dict(name=name+axis,kind='relative_'+axis,a=name,b='backgroundA')
                  for name in pose+['object'] for axis in ('x','y')]
        # Subpixel centroid of this even-width visible patch is (1199.5,699.5).
        coordinates={**cls.points,'object':(1199.5,699.5)}
        bounds={f['name']:[coordinates[f['a']][0 if f['kind']=='relative_x' else 1]-120-10,
                           coordinates[f['a']][0 if f['kind']=='relative_x' else 1]-120+10] for f in features}
        cls.spec=dict(schemaVersion=SCHEMA,cameraIdentity='external',imageShape=[1080,1920],**cls.context,
            maxAgeSeconds=3,maxLandmarkUncertaintyPx=2,landmarks=landmarks,poseLandmarks=pose,
            fixedViewAnchors={name:[x-3,x+3,y-3,y+3] for name,(x,y) in cls.points.items() if name.startswith('background')},
            joints=dict(wrist=dict(stateFields=features,nodes=[dict(id='static',bounds=bounds)])),
            referenceImageBase64=base64.b64encode(reference_png).decode(),
            referenceImageSha256=hashlib.sha256(reference_png).hexdigest(),
            referencePixelSha256=hashlib.sha256(reference.tobytes()).hexdigest(),
            registrationImplementationSha256=registration_implementation_sha256(),maximumReferenceShiftPx=12)

    def observe(self, *, image=None, spec=None, frame_change=None):
        image=self.image if image is None else image
        raw=cv2.imencode('.png',image)[1].tobytes()
        frame=dict(frameId='static:1',imageSha256=hashlib.sha256(raw).hexdigest(),
            pixelSha256=hashlib.sha256(image.tobytes()).hexdigest(),capturedAt=10.,persistedAt=10.1,
            clockId='clock',cameraIdentity='external')
        frame.update(frame_change or {})
        return observe_frame(raw,frame,self.spec if spec is None else spec,self.context,11.,'clock')

    def test_existing_api_routes_registered_and_hsv_points_without_motion_claim(self):
        self.assertEqual(validate_spec(self.spec),self.spec)
        result=self.observe()
        self.assertEqual(result['status'],'accepted',result['reason'])
        self.assertEqual(result['nodeMatches'],['static'])
        self.assertEqual(set(result['landmarks']),set(self.spec['landmarks']))
        self.assertAlmostEqual(result['landmarks']['object']['point'][0],1199.5)
        self.assertIn('registrationEvidence',result['landmarks']['wrist'])
        self.assertTrue(result['readonlyStationaryEligible'])
        self.assertFalse(result['motionTrackingVerified'])
        self.assertFalse(result['controlEligible'])
        self.assertFalse(result['trainingEligible'])

    def test_global_camera_translation_is_not_canceled_by_relative_features(self):
        image=cv2.warpAffine(self.image,np.float32([[1,0,3],[0,1,0]]),(1920,1080),borderValue=(40,40,40))
        result=self.observe(image=image)
        self.assertEqual(result['status'],'rejected')
        self.assertIn('fixed_view_anchor_moved',result['reason'])

    def test_common_pose_occlusion_is_rejected(self):
        image=self.image.copy();image[310:391,810:891]=40
        result=self.observe(image=image)
        self.assertEqual(result['status'],'rejected')
        self.assertEqual(result['states'],{})
        self.assertFalse(result['readonlyStationaryEligible'])

    def test_loss_of_lighting_contrast_cannot_pass_registration(self):
        image=np.full_like(self.image,40)
        result=self.observe(image=image)
        self.assertEqual(result['status'],'rejected')
        self.assertIn('texture_missing',result['reason'])

    def test_optional_object_occlusion_is_scoped_to_dependent_joint(self):
        spec=copy.deepcopy(self.spec)
        other=copy.deepcopy(spec['joints']['wrist']);other['stateFields']=other['stateFields'][:-2]
        other['nodes'][0]['id']='base-static'
        other['nodes'][0]['bounds']={k:v for k,v in other['nodes'][0]['bounds'].items() if not k.startswith('object')}
        spec['joints']['base']=other
        image=self.image.copy();image[680:720,1190:1210]=40
        result=self.observe(spec=spec,image=image)
        self.assertEqual(result['status'],'accepted')
        self.assertNotIn('wrist',result['states'])
        self.assertEqual(result['nodeMatches'],['base-static'])

    def test_reference_image_pixel_and_implementation_hash_changes_are_rejected(self):
        for field in ('referenceImageSha256','referencePixelSha256','registrationImplementationSha256'):
            spec=copy.deepcopy(self.spec);spec[field]='0'*64
            with self.subTest(field=field):
                result=self.observe(spec=spec)
                self.assertEqual(result['status'],'rejected')

    def test_reference_base64_and_search_radius_are_not_silently_repaired(self):
        spec=copy.deepcopy(self.spec);spec['referenceImageBase64']='invalid!'
        self.assertEqual(self.observe(spec=spec)['reason'],'registered_reference_base64')
        spec=copy.deepcopy(self.spec);spec['maximumReferenceShiftPx']=13
        self.assertEqual(self.observe(spec=spec)['reason'],'registered_fixed_geometry_required')

    def test_background_reference_roles_and_patch_margin_are_fixed(self):
        spec=copy.deepcopy(self.spec);spec['landmarks']['backgroundA']['reference']=False
        with self.assertRaisesRegex(ValueError,'registered_reference_role'):validate_spec(spec)
        spec=copy.deepcopy(self.spec);spec['landmarks']['wrist']['point']=[10,10]
        with self.assertRaisesRegex(ValueError,'registration_patch_margin_or_role'):validate_spec(spec)
        spec=copy.deepcopy(self.spec);spec['landmarks']['backgroundB']['point']=spec['landmarks']['backgroundA']['point']
        with self.assertRaisesRegex(ValueError,'duplicate_registration_point'):validate_spec(spec)

    def test_shared_frame_hash_and_context_checks_are_preserved(self):
        self.assertEqual(self.observe(frame_change=dict(pixelSha256='0'*64))['reason'],'frame_pixel_hash_changed')
        spec=copy.deepcopy(self.spec);spec['load']='held'
        self.assertEqual(self.observe(spec=spec)['reason'],'observer_context_changed')


if __name__=='__main__':unittest.main()
