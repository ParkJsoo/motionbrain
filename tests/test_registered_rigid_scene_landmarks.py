"""강체 광류의 기준점 전달·관측 상실·배경 이동 거절을 검사한다."""
import copy
import unittest
import cv2
import numpy as np
from learning.registered_scene_landmarks import register
from learning.registered_rigid_scene_landmarks import register as rigid_register,RegisteredRigidSceneLandmarks
from learning.relative_action_learning import point_state


def png(image):return cv2.imencode('.png',image)[1].tobytes()


class RigidSceneTests(unittest.TestCase):
    def setUp(self):
        self.image=np.random.default_rng(3).integers(10,245,(160,240,3),dtype=np.uint8)
        self.raw=png(self.image)
        points=dict(object=[120.,35.],jawLeft=[60.,110.],jawRight=[180.,110.])
        p=dict(imageShape=[160,240],patchRois=dict(object=[110,25,130,45],jawLeft=[50,100,70,120],jawRight=[170,100,190,120]),
            searchRois=dict(object=[100,15,140,55],jawLeft=[40,90,80,130],jawRight=[160,90,200,130]),
            minimumNcc=dict(object=.85,jawLeft=.94,jawRight=.94),minimumPeakGap=.04,peakExclusionRadius=2,minimumTemplateStd=1.5)
        base=register(self.raw,points,p,dict(object='synthetic',physicalView='test',poseRegime='M4-only'))
        self.reg=rigid_register(base,self.raw,[[25,80],[215,80],[215,140],[25,140]],
            [[[10,10],[65,10],[65,65],[10,65]],[[175,10],[230,10],[230,65],[175,65]]])
        self.tracker=RegisteredRigidSceneLandmarks(self.reg,self.raw)

    def moved(self):
        image=self.image.copy();image[80:140,25:215]=0;image[84:144,28:218]=self.image[80:140,25:215];return image

    def test_fixed_targets_follow_visible_rigid_features_and_keep_new_contract(self):
        result=self.tracker.observe(png(self.moved()),'moved')
        np.testing.assert_allclose(result['annotation']['points']['jawLeft'],[63,114],atol=.15)
        np.testing.assert_allclose(result['annotation']['points']['jawRight'],[183,114],atol=.15)
        self.assertEqual(point_state(result['annotation'],dict(frameId='moved',pixelSha256=result['pixelSha256'],width=240,height=160)),result['state'])
        self.assertFalse(result['controlEligible'])

    def test_occlusion_and_background_translation_are_rejected(self):
        image=self.image.copy();image[80:145,20:220]=0
        with self.assertRaisesRegex(ValueError,'tracking_rejected'):
            self.tracker.observe(png(image),'occluded')
        image=self.moved();image[:75]=cv2.warpAffine(self.image[:75],np.float32([[1,0,5],[0,1,0]]),(240,75))
        with self.assertRaisesRegex(ValueError,'background'):
            self.tracker.observe(png(image),'camera-moved')

    def test_changed_polygons_or_reference_are_rejected(self):
        reg=copy.deepcopy(self.reg);reg['parameters']['movingPolygon'][0][0]+=1
        with self.assertRaisesRegex(ValueError,'contract_changed'):
            RegisteredRigidSceneLandmarks(reg,self.raw)


if __name__=='__main__':unittest.main()
