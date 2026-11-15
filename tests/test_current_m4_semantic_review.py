import copy
import unittest
import cv2
import numpy as np
from learning.current_m4_semantic_review import semantic_picture,SCHEMA

class SemanticGeometryTests(unittest.TestCase):
    def setUp(self):
        rng=np.random.default_rng(3)
        self.image=rng.integers(50,200,(240,360,3),dtype=np.uint8)
        self.regions=dict(distalArm=[0,0,120,240],elbow=[120,0,240,240],proximalArm=[240,0,360,240])
        self.contract=dict(schemaVersion=SCHEMA,anchorSha256='a'*64,externalArmRegions=self.regions)
        self.measure=dict(landmarks={name:dict(centroid=[50.,50.],bbox=[40,40,20,20],area=300) for name in ('externalObject','externalTool')})
    def check(self,image,measurement=None):
        encode=lambda x:cv2.imencode('.png',x)[1].tobytes()
        return semantic_picture(encode(self.image),encode(image),self.measure,measurement or self.measure,self.contract)
    def test_unregistered_background_change_allowed(self):
        self.image=np.pad(self.image,((0,0),(0,40),(0,0)),constant_values=100)
        changed=self.image.copy();changed[:,365:]=220
        self.check(changed)
    def test_additive_brightness_preserves_geometry(self):
        self.check(self.image+8)
    def test_translation_rejected(self):
        with self.assertRaises(ValueError):
            self.check(cv2.warpAffine(self.image,np.float32([[1,0,3],[0,1,0]]),(360,240)))
    def test_each_arm_local_move_rejected(self):
        for x in (0,120,240):
            image=self.image.copy();image[:,x:x+120]=np.roll(image[:,x:x+120],4,axis=0)
            with self.subTest(x=x),self.assertRaises(ValueError):self.check(image)
    def test_occlusion_rejected(self):
        image=self.image.copy();image[:120,:120]=100
        with self.assertRaises(ValueError):self.check(image)
    def test_low_texture_rejected(self):
        self.image[:,:120]=100
        with self.assertRaisesRegex(ValueError,'texture'):self.check(self.image)
    def test_object_horizontal_move_rejected(self):
        result=copy.deepcopy(self.measure);result['landmarks']['externalObject']['centroid'][0]+=2
        with self.assertRaisesRegex(ValueError,'landmark_moved'):self.check(self.image,result)
    def test_tool_horizontal_move_rejected(self):
        result=copy.deepcopy(self.measure);result['landmarks']['externalTool']['centroid'][0]+=2
        with self.assertRaisesRegex(ValueError,'landmark_moved'):self.check(self.image,result)
    def test_shape_occlusion_rejected(self):
        result=copy.deepcopy(self.measure);result['landmarks']['externalObject']['area']=200
        with self.assertRaisesRegex(ValueError,'shape_changed'):self.check(self.image,result)
    def test_segmentation_edge_jitter_allowed(self):
        result=copy.deepcopy(self.measure);result['landmarks']['externalTool'].update(centroid=[50.8,50.1],bbox=[40,40,22,22],area=310)
        self.check(self.image,result)
    def test_missing_arm_region_rejected(self):
        del self.contract['externalArmRegions']['elbow']
        with self.assertRaises(ValueError):self.check(self.image)
