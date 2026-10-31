import unittest
from unittest.mock import patch
import numpy as np
try:
    from learning import visual_registration as m
except ImportError:
    m=None

@unittest.skipIf(m is None,'OpenCV required')
class RegistrationTests(unittest.TestCase):
    def evidence(self):return dict(score=.9,uniquenessGap=.2)
    def test_translation_preserves_fixed_ids(self):
        rng=np.random.default_rng(4);gray=rng.integers(0,256,(1080,1920),dtype=np.uint8)
        source=np.repeat(gray[:,:,None],4,axis=2);source[:,:,3]=255
        target=np.roll(source,(1,2),axis=(0,1))
        points=[dict(id='background',x=500.,y=500.,reference=True),dict(id='link',x=900.,y=700.,reference=False)]
        features,e=m.register(source.tobytes(),target.tobytes(),points)
        self.assertEqual([f['id'] for f in features],['background','link'])
        for p,f in zip(points,features):self.assertAlmostEqual(f['x']-p['x'],2,delta=.1);self.assertAlmostEqual(f['y']-p['y'],1,delta=.1)
        self.assertTrue(all(r['patchSize']==41 for r in e))
    def test_arm_never_uses_background_fallback(self):
        with patch.object(m,'locate',side_effect=ValueError('weak_or_ambiguous_patch')) as call:
            with self.assertRaisesRegex(ValueError,'weak_or_ambiguous'):m.match_point(None,None,np.array([100.,100.]),background=False)
            self.assertEqual(call.call_count,1)
    def matching(self,sizes,shifts=None):
        counts={}
        def locate(source,target,xy,size=41):
            if size not in sizes:raise ValueError('weak_or_ambiguous_patch')
            counts[size]=counts.get(size,0)+1
            delta=np.array([1.+(shifts or {}).get(size,0),0])
            return (np.asarray(xy)+delta if counts[size]%2 else np.asarray(xy)-delta),self.evidence()
        return locate
    def test_requires_two_scales_and_preserves_evidence(self):
        with patch.object(m,'locate',side_effect=self.matching({21,15})):
            q,e=m.match_point(None,None,np.array([100.,100.]),background=True)
        self.assertEqual(e['patchSize'],21);self.assertEqual(len(e['supportingScales']),2)
        self.assertEqual(q.tolist(),[101.,100.])
    def test_one_good_small_scale_is_insufficient(self):
        with patch.object(m,'locate',side_effect=self.matching({15})):
            with self.assertRaisesRegex(ValueError,'support_missing'):m.match_point(None,None,np.array([100.,100.]),background=True)
    def test_conflicting_scales_are_not_cherry_picked(self):
        with patch.object(m,'locate',side_effect=self.matching({21,15,13},{13:1.})):
            with self.assertRaisesRegex(ValueError,'disagreement'):m.match_point(None,None,np.array([100.,100.]),background=True)
    def test_search_boundary_does_not_trigger_fallback(self):
        with patch.object(m,'locate',side_effect=ValueError('search_boundary')) as call:
            with self.assertRaisesRegex(ValueError,'search_boundary'):m.match_point(None,None,np.array([100.,100.]),background=True)
            self.assertEqual(call.call_count,1)
    def test_roundtrip_violation_is_fatal(self):
        with patch.object(m,'locate',return_value=(np.array([102.,100.]),self.evidence())):
            with self.assertRaisesRegex(ValueError,'roundtrip'):m.match_point(None,None,np.array([100.,100.]),background=True)
    def test_blank_patch_is_not_registered(self):
        blank=np.zeros((100,100),np.uint8)
        with self.assertRaisesRegex(ValueError,'texture_missing'):m.locate(blank,blank,(50.,50.))
    def test_raw_size_rejected(self):
        with self.assertRaisesRegex(ValueError,'raw_size'):m.register(b'bad',b'bad',[])
