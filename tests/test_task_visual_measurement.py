"""고정 기하의 픽셀 변화를 관측하며 접촉·물리 미끄럼은 주장하지 않는다."""
import copy
import unittest

import cv2
import numpy as np

from learning import task_visual_measurement as visual


def encode(image):
    ok,raw=cv2.imencode('.png',image)
    assert ok
    return raw.tobytes()


def images(*,object_up=0,tool_up=0,grip_object_up=0,grip_jaw_up=0):
    ext=np.full((300,400,3),210,np.uint8)
    cv2.rectangle(ext,(45,170-object_up),(80,250-object_up),(150,20,235),-1)
    cv2.circle(ext,(180,130-tool_up),8,(150,20,235),-1)
    grip=np.full((240,320,3),175,np.uint8)
    cv2.rectangle(grip,(140,110-grip_object_up),(185,150-grip_object_up),(150,20,235),-1)
    random=np.random.default_rng(93)
    for x in (92,200):
        patch=random.integers(20,230,(20,24),np.uint8)
        grip[63-grip_jaw_up:83-grip_jaw_up,x:x+24]=cv2.cvtColor(patch,cv2.COLOR_GRAY2BGR)
    return ext,grip


def parameters():
    def component(roi,area,width,height):
        return dict(roi=roi,hsvLower=[150,100,70],hsvUpper=[179,255,255],areaRange=area,
                    widthRange=width,heightRange=height,ambiguityRatio=.5)
    return dict(externalObject=component([25,145,125,275],[1500,5000],[25,45],[60,100]),
                externalTool=component([145,100,220,155],[100,400],[10,25],[10,25]),
                gripperObject=component([125,90,205,170],[1000,3000],[30,55],[30,55]),
                gripperReferences=[dict(templateRoi=[x,63,x+24,83],searchRoi=[x-6,57,x+30,89],
                                        minNcc=.95,minPeakGap=.01) for x in (92,200)])


class VisualMeasurementTests(unittest.TestCase):
    def setUp(self):
        ext,grip=images();self.ext,self.grip=encode(ext),encode(grip)
        self.parameters=parameters();self.registration=visual.register(self.ext,self.grip,self.parameters)

    def test_reference_is_one_shared_zero_baseline(self):
        r=visual.measure(self.ext,self.grip,self.registration)
        self.assertEqual(r['state'],dict(objectRisePx=0.,toolRisePx=0.,slipDyPx=0.))
        self.assertFalse(r['quality']['actualPhysicalSlipVerified'])
        self.assertFalse(r['quality']['calibratedHeight'])

    def test_known_projected_motion_and_jaw_relative_motion(self):
        e,g=images(object_up=10,tool_up=8,grip_object_up=4,grip_jaw_up=1)
        r=visual.measure(encode(e),encode(g),self.registration)
        self.assertAlmostEqual(r['state']['objectRisePx'],10.)
        self.assertAlmostEqual(r['state']['toolRisePx'],8.)
        self.assertAlmostEqual(r['state']['slipDyPx'],-3.,places=2)

    def test_camera_jaw_and_object_common_shift_is_not_slip(self):
        e,g=images(grip_object_up=2,grip_jaw_up=2)
        r=visual.measure(encode(e),encode(g),self.registration)
        self.assertAlmostEqual(r['state']['slipDyPx'],0.,places=2)

    def test_occluded_object_top_does_not_move_bottom_landmark(self):
        ext,_=images();ext[170:185,45:81]=30
        r=visual.measure(encode(ext),self.grip,self.registration)
        self.assertEqual(r['state']['objectRisePx'],0.)

    def test_clipped_missing_and_competing_components_rejected(self):
        for mode in ('clipped','missing','competing'):
            ext,_=images()
            if mode=='clipped':ext[145:251,25:65]=(150,20,235)
            if mode=='missing':ext[145:275,25:125]=210
            if mode=='competing':cv2.rectangle(ext,(87,170),(122,250),(150,20,235),-1)
            with self.subTest(mode=mode),self.assertRaises(ValueError):
                visual.measure(encode(ext),self.grip,self.registration)

    def test_missing_reference_and_search_edge_rejected(self):
        ext,grip=images();grip[57:89,86:122]=175
        with self.assertRaisesRegex(ValueError,'reference_ncc'):
            visual.measure(encode(ext),encode(grip),self.registration)
        ext,grip=images(grip_jaw_up=6)
        with self.assertRaisesRegex(ValueError,'reference_clipped'):
            visual.measure(encode(ext),encode(grip),self.registration)

    def test_shape_parameter_and_template_mutation_rejected(self):
        with self.assertRaisesRegex(ValueError,'image_shape_changed'):
            visual.measure(encode(np.zeros((301,400,3),np.uint8)),self.grip,self.registration)
        r=copy.deepcopy(self.registration);r['parameters']['externalObject']['hsvLower'][0]-=1
        with self.assertRaisesRegex(ValueError,'parameters_changed'):visual.measure(self.ext,self.grip,r)
        r=copy.deepcopy(self.registration);r['templates'][0]['sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'template_changed'):visual.measure(self.ext,self.grip,r)

    def test_measure_does_not_mutate_registration(self):
        original=copy.deepcopy(self.registration)
        visual.measure(self.ext,self.grip,self.registration)
        self.assertEqual(original,self.registration)


if __name__=='__main__':unittest.main()
