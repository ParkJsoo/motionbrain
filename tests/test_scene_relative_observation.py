"""현재 등록 상대관측: 알려진 이동/확대, 실제 상대운동, 실패 시 거절."""
import copy
import unittest
import cv2
import numpy as np
from learning.scene_relative_observation import register,SceneRelativeObserver,FIELDS,rebind


def encoded(image):
    ok,data=cv2.imencode('.png',image);assert ok
    return data.tobytes()


class RelativeObservationTests(unittest.TestCase):
    def setUp(self):
        self.image=np.full((180,240,3),40,np.uint8)
        self.points=dict(object=[120,45],jawLeft=[60,115],jawRight=[180,115])
        self.rois={k:[x-12,y-12,x+12,y+12] for k,(x,y) in self.points.items()}
        rng=np.random.default_rng(95)
        for box in self.rois.values():
            x0,y0,x1,y1=box
            self.image[y0:y1,x0:x1]=rng.integers(0,255,(24,24,3),dtype=np.uint8)
        self.raw=encoded(self.image)
        self.context=dict(physicalView='synthetic-current-view',object='synthetic-target',poseRegime='current',cameraBootId='a'*32)
        self.reg=register(self.raw,self.points,self.rois,self.context)
        self.observer=SceneRelativeObserver(self.reg,self.raw)

    def observe(self,image,context=None):
        return self.observer.observe(encoded(image),'a'*32+':test',self.context if context is None else context)

    def transformed(self,scale,shift=(0,0),relative=None):
        result=np.full((240,320,3),40,np.uint8)
        expected={}
        for name,roi in self.rois.items():
            x0,y0,x1,y1=roi;dx,dy=(relative or {}).get(name,(0,0))
            patch=self.image[y0:y1,x0:x1]
            patch=cv2.resize(patch,(round(24*scale),round(24*scale)),interpolation=cv2.INTER_LINEAR)
            x,y=round((x0+dx)*scale+shift[0]),round((y0+dy)*scale+shift[1])
            result[y:y+patch.shape[0],x:x+patch.shape[1]]=patch
            expected[name]=np.array([x+patch.shape[1]/2,y+patch.shape[0]/2])
        return result,expected

    def test_known_translation_and_scale_preserve_canonical_relative_state(self):
        base=self.observe(self.image)['state']
        for scale,shift in ((1.,(35,25)),(.8,(40,40)),(1.2,(12,10))):
            with self.subTest(scale=scale):
                image,_=self.transformed(scale,shift)
                observed=self.observe(image)
                np.testing.assert_allclose(observed['state'],base,atol=1.5)
                self.assertAlmostEqual(observed['observedScale'],scale)
                self.assertEqual(observed['stateFields'],list(FIELDS))
                self.assertFalse(observed['modelTransferVerified']);self.assertFalse(observed['controlEligible'])

    def test_relative_motion_is_retained_and_closing_is_not_normalized_away(self):
        image,points=self.transformed(1.2,(10,10),dict(object=(0,8),jawLeft=(6,0),jawRight=(-6,0)))
        result=self.observe(image)
        expected=[*((points['object']-(points['jawLeft']+points['jawRight'])/2)/1.2),
                  (points['jawRight'][0]-points['jawLeft'][0])/1.2]
        np.testing.assert_allclose(result['state'],expected,atol=.1)
        self.assertLess(result['state'][2],self.observe(self.image)['state'][2]-10)
        self.assertGreater(result['state'][1],self.observe(self.image)['state'][1]+6)

    def test_occlusion_deformation_and_duplicate_are_rejected(self):
        for kind in ('occlusion','deformation','duplicate'):
            image=self.image.copy();x0,y0,x1,y1=self.rois['jawLeft']
            if kind=='occlusion':image[y0:y0+8,x0:x1]=40
            if kind=='deformation':
                image[y0:y1,x0:x1]=cv2.resize(self.image[y0:y1,x0:x1],(30,24))[:,:24]
            if kind=='duplicate':image[130:154,100:124]=self.image[y0:y1,x0:x1]
            with self.subTest(kind=kind),self.assertRaises(ValueError):self.observe(image)

    def test_nonuniform_patch_scale_cannot_be_explained_as_camera_zoom(self):
        image,_=self.transformed(1.,(10,10));x0,y0,x1,y1=self.rois['object']
        image[y0+10:y1+10,x0+10:x1+10]=40
        image[y0+10:y0+10+29,x0+10:x0+10+29]=cv2.resize(self.image[y0:y1,x0:x1],(29,29))
        with self.assertRaisesRegex(ValueError,'nonuniform_scale'):self.observe(image)

    def test_new_boot_scene_or_modified_registration_is_not_model_transfer(self):
        for key,value in [('cameraBootId','b'*32),('physicalView','different')]:
            with self.assertRaisesRegex(ValueError,'source_context_changed'):self.observe(self.image,dict(self.context,**{key:value}))
        with self.assertRaisesRegex(ValueError,'frame_boot_changed'):
            self.observer.observe(self.raw,'b'*32+':test',self.context)
        changed=copy.deepcopy(self.reg);changed['points']['object'][0]+=1
        with self.assertRaisesRegex(ValueError,'registration_changed'):SceneRelativeObserver(changed,self.raw)

    def test_clipped_target_and_large_unregistered_scale_reject(self):
        image=self.image.copy();x0,y0,x1,y1=self.rois['jawLeft']
        image[y0:y1,x0:x1]=40;image[:24,:24]=self.image[y0:y1,x0:x1]
        with self.assertRaisesRegex(ValueError,'clipped'):self.observe(image)
        image=cv2.resize(self.image,None,fx=1.4,fy=1.4)
        with self.assertRaises(ValueError):self.observe(image)

    def test_explicit_rebind_keeps_canonical_coordinates_and_checks_current_raw(self):
        context=dict(self.context,cameraBootId='b'*32,physicalView='explicit-new-view')
        image,_=self.transformed(1.2,(10,10))
        rebound=rebind(self.reg,self.raw,encoded(image),'b'*32+':fresh',context)
        self.assertEqual(rebound['canonicalFrameSha256'],self.reg['canonicalFrameSha256'])
        self.assertNotEqual(rebound['registrationSha256'],self.reg['registrationSha256'])
        observed=SceneRelativeObserver(rebound,self.raw).observe(encoded(image),'b'*32+':fresh',context)
        np.testing.assert_allclose(observed['state'],self.observe(self.image)['state'],atol=1.5)
        self.assertEqual(self.reg['sourceContext'],self.context)
        with self.assertRaises(ValueError):rebind(self.reg,self.raw,encoded(np.zeros_like(image)),'b'*32+':bad',context)
        with self.assertRaises(ValueError):rebind(self.reg,self.raw,encoded(image),'a'*32+':wrong',context)

    def test_matched_patch_overlap_rejects_even_when_points_are_ordered(self):
        from unittest.mock import patch
        from learning import scene_relative_observation as module
        real=module._match
        def overlap(*args,**kwargs):
            result=real(*args,**kwargs)
            result['bbox']=[20,20,30,30]
            return result
        with patch.object(module,'_match',side_effect=overlap):
            with self.assertRaisesRegex(ValueError,'matched_landmark_patches_overlap'):self.observe(self.image)
