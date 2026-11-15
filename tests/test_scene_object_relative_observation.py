"""물체 폭 좌표: 독립 턱 이동, 새 구도 등록, 가림/변형/모호성 거절."""
import copy
import unittest
import cv2
import numpy as np
from learning.scene_object_relative_observation import register,SceneObjectRelativeObserver,FIELDS
from tests.test_scene_relative_observation import encoded


class ObjectRelativeTests(unittest.TestCase):
    def setUp(self):
        self.image=np.full((200,260,3),40,np.uint8)
        self.image[45:85,110:150]=[160,50,220]
        self.points=dict(jawLeft=[60,120],jawRight=[200,120])
        self.rois={k:[x-12,y-12,x+12,y+12] for k,(x,y) in self.points.items()}
        rng=np.random.default_rng(23)
        for x,y,z,w in self.rois.values():
            texture=rng.integers(0,255,(24,24),dtype=np.uint8)
            self.image[y:w,x:z]=texture[:,:,None]
        self.raw=encoded(self.image)
        self.context=dict(physicalView='scene-a',object='same-visible-pink-face',poseRegime='table',cameraBootId='a'*32)
        self.registration=register(self.raw,self.points,self.rois,self.context)
        self.observer=SceneObjectRelativeObserver(self.registration,self.raw)

    def observe(self,image):
        return self.observer.observe(encoded(image),'a'*32+':fresh',self.context)

    def test_uniform_object_translation_and_scale_do_not_need_texture(self):
        base=self.observe(self.image)
        for scale in (.8,1.,1.2):
            out=np.full((280,360,3),40,np.uint8)
            out[round(45*scale)+15:round(85*scale)+15,round(110*scale)+20:round(150*scale)+20]=[160,50,220]
            for x,y,z,w in self.rois.values():
                patch=cv2.resize(self.image[y:w,x:z],(round(24*scale),round(24*scale)))
                xx,yy=round(x*scale)+20,round(y*scale)+15
                out[yy:yy+patch.shape[0],xx:xx+patch.shape[1]]=patch
            result=self.observe(out)
            np.testing.assert_allclose(result['state'],base['state'],atol=.06)
            self.assertEqual(result['stateFields'],list(FIELDS))
            self.assertFalse(result['modelTransferVerified'])

    def test_independent_jaw_closing_is_preserved(self):
        out=self.image.copy()
        for name,dx in [('jawLeft',10),('jawRight',-10)]:
            x,y,z,w=self.rois[name];patch=out[y:w,x:z].copy();out[y:w,x:z]=40
            out[y:w,x+dx:z+dx]=patch
        self.assertAlmostEqual(self.observe(out)['state'][2],3.)
        self.assertAlmostEqual(self.observe(self.image)['state'][2],3.5)

    def test_duplicate_clipped_occluded_and_deformed_object_reject(self):
        for mode in ('duplicate','clipped','hole','deformed'):
            image=self.image.copy()
            if mode=='duplicate':image[150:175,100:125]=[160,50,220]
            if mode=='clipped':image[45:85,110:150]=40;image[0:40,110:150]=[160,50,220]
            if mode=='hole':image[55:75,120:140]=40
            if mode=='deformed':image[45:85,110:150]=40;image[35:95,110:150]=[160,50,220]
            with self.subTest(mode=mode),self.assertRaises(ValueError):self.observe(image)

    def test_color_spur_removal_does_not_fill_occlusion(self):
        image=self.image.copy();image[44,115]=[160,50,220]
        self.assertEqual(self.observe(image)['object']['box'],self.observe(self.image)['object']['box'])
        image[55:75,120:140]=40
        with self.assertRaisesRegex(ValueError,'shape_or_occlusion'):self.observe(image)

    def test_explicit_new_registration_retains_semantics_without_boot_alias(self):
        ctx=dict(self.context,physicalView='scene-b',cameraBootId='b'*32)
        reg=register(self.raw,self.points,self.rois,ctx)
        self.assertEqual(reg['canonicalFrameSha256'],self.registration['canonicalFrameSha256'])
        self.assertNotEqual(reg['registrationSha256'],self.registration['registrationSha256'])
        with self.assertRaisesRegex(ValueError,'source_context_changed'):
            self.observer.observe(self.raw,'b'*32+':fresh',ctx)
        changed=copy.deepcopy(reg);changed['points']['jawLeft'][0]+=1
        with self.assertRaisesRegex(ValueError,'registration_changed'):SceneObjectRelativeObserver(changed,self.raw)

    def test_object_and_jaw_scale_disagreement_rejects(self):
        image=self.image.copy();image[45:85,110:150]=40;image[40:90,105:155]=[160,50,220]
        with self.assertRaisesRegex(ValueError,'nonuniform_scale'):self.observe(image)

    def test_independent_shifted_registration_has_same_units_but_distinct_raw_identity(self):
        image=np.full_like(self.image,40);image[10:,15:]=self.image[:-10,:-15]
        points={k:[p[0]+15,p[1]+10] for k,p in self.points.items()}
        rois={k:[r[0]+15,r[1]+10,r[2]+15,r[3]+10] for k,r in self.rois.items()}
        context=dict(self.context,physicalView='explicit-shifted-source',cameraBootId='b'*32)
        raw=encoded(image);reg=register(raw,points,rois,context)
        actual=SceneObjectRelativeObserver(reg,raw).observe(raw,'b'*32+':fresh',context)
        np.testing.assert_allclose(actual['state'],self.observe(self.image)['state'])
        self.assertEqual(reg['canonicalFrameSha256'],self.registration['canonicalFrameSha256'])
        self.assertNotEqual(reg['referenceImageSha256'],self.registration['referenceImageSha256'])
        self.assertFalse(actual['measurementAccuracyVerified'])
