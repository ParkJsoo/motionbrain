import unittest
from unittest.mock import patch
import numpy as np
try:
 import cv2
except ImportError:cv2=None
from learning.visual_features import PointTracker

@unittest.skipIf(cv2 is None,'OpenCV required')
class FixedBackgroundTests(unittest.TestCase):
 def setup_tracker(self):
  gray=np.random.default_rng(8).integers(0,256,(128,128),dtype=np.uint8)
  a=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGRA)
  t=PointTracker(128,128,[dict(id=n,x=x,y=y,reference=r) for n,x,y,r in [('bg',50.,50.,True),('a',80.,70.,False),('b',80.,90.,False)]],max_displacement_px=80,reference_limit_px=2)
  t.update(a.tobytes());t.anchor_references(t.previous)
  return t,a
 def test_fixed_template_does_not_feed_background_to_lk(self):
  t,a=self.setup_tracker();real=cv2.calcOpticalFlowPyrLK;counts=[]
  def call(*args,**kw):counts.append(len(args[2]));return real(*args,**kw)
  with patch.object(cv2,'calcOpticalFlowPyrLK',side_effect=call):out=t.update(a.tobytes())
  self.assertEqual(counts,[2,2]);self.assertIsNone(t.fault);self.assertAlmostEqual(out[0]['x'],50,delta=.1)
 def test_camera_shift_still_stops_and_fault_latches(self):
  t,a=self.setup_tracker();t.update(np.roll(a,4,axis=1).tobytes());self.assertEqual(t.fault,'visual_travel_limit')
  t.update(a.tobytes());self.assertEqual(t.fault,'visual_travel_limit')
  with self.assertRaises(ValueError):t.anchor_references(t.previous)
 def test_background_occlusion_is_not_reacquired(self):
  t,a=self.setup_tracker();bad=a.copy();bad[15:85,15:85,:3]=0;t.update(bad.tobytes())
  self.assertEqual(t.fault,'tracking_lost');self.assertFalse(t.alive[0]);t.update(a.tobytes());self.assertFalse(t.alive[0])
 def test_reference_cannot_be_reset(self):
  t,a=self.setup_tracker()
  with self.assertRaises(ValueError):t.anchor_references(t.previous)
