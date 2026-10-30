"""Known motion, disappearance and limits; no private frames in tests."""
import copy
import unittest

from learning.visual_features import PointTracker

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


@unittest.skipIf(cv2 is None,'optional OpenCV/numpy unavailable')
class FeatureTest(unittest.TestCase):
    def setUp(self):
        rng=np.random.default_rng(81)
        self.gray=cv2.GaussianBlur(rng.integers(0,256,(512,512),dtype=np.uint8),(3,3),0)
        mask=np.zeros_like(self.gray);mask[180:330,180:330]=255
        coords=cv2.goodFeaturesToTrack(self.gray,3,.1,20,mask=mask).reshape(-1,2)
        self.points=[dict(id=f'p{i}',x=float(p[0]),y=float(p[1]),reference=i==2) for i,p in enumerate(coords)]
    def image(self,gray):return cv2.cvtColor(gray,cv2.COLOR_GRAY2BGRA).tobytes()
    def tracker(self,**kwargs):
        return PointTracker(512,512,self.points,max_displacement_px=kwargs.get('travel',40),reference_limit_px=kwargs.get('reference',10))
    def test_translation_tracks_material_points_with_fixed_ids(self):
        t=self.tracker();a=t.update(self.image(self.gray))
        shifted=cv2.warpAffine(self.gray,np.float32([[1,0,4],[0,1,-3]]),(512,512))
        b=t.update(self.image(shifted))
        self.assertIsNone(t.fault)
        for first,last in zip(a,b):
            self.assertEqual(first['id'],last['id']);self.assertTrue(last['visible'])
            self.assertAlmostEqual(last['x']-first['x'],4,delta=.05)
            self.assertAlmostEqual(last['y']-first['y'],-3,delta=.05)
    def test_occlusion_is_null_and_never_reacquired(self):
        t=self.tracker();t.update(self.image(self.gray))
        missing=t.update(self.image(np.zeros_like(self.gray)))
        self.assertEqual(t.fault,'tracking_lost')
        self.assertTrue(all(not p['visible'] and p['x'] is None and p['y'] is None for p in missing))
        self.assertTrue(all(not p['visible'] for p in t.update(self.image(self.gray))))
    def test_camera_or_reference_motion_trips_limit(self):
        t=self.tracker(reference=1);t.update(self.image(self.gray))
        shifted=cv2.warpAffine(self.gray,np.float32([[1,0,4],[0,1,0]]),(512,512))
        t.update(self.image(shifted));self.assertEqual(t.fault,'visual_travel_limit')
    def test_textureless_initial_frame_is_not_valid_tracking(self):
        t=self.tracker();p=t.update(self.image(np.zeros_like(self.gray)))
        self.assertEqual(t.fault,'tracking_lost');self.assertFalse(any(x['visible'] for x in p))
    def test_invalid_shape_duplicate_ids_and_missing_reference_fail_before_use(self):
        for change in ('id','reference','x'):
            points=copy.deepcopy(self.points)
            if change=='id':points[1]['id']=points[0]['id']
            if change=='reference':
                for p in points:p['reference']=False
            if change=='x':points[0]['x']=-1
            with self.subTest(change=change),self.assertRaises(ValueError):
                PointTracker(512,512,points,max_displacement_px=40,reference_limit_px=3)
        with self.assertRaises(ValueError):self.tracker().update(b'wrong length')
