"""고정 영상 진입 검사. 합성 반례이며 실물 정확도 시험이 아니다."""
import copy
import unittest
from learning import reference_entry_alignment as gate
from tests.test_limited_scene_cycle_assessment import near, external


class ReferenceEntryTests(unittest.TestCase):
    def setUp(self):
        self.external = external(); self.external['virtualProxyPx'] = [800., 650.]
        self.near = near(version=3)
        self.refs = [dict(external=copy.deepcopy(self.external), near=copy.deepcopy(self.near)) for _ in range(2)]

    def assess(self, **kwargs):
        return gate.assess(kwargs.get('refs', self.refs), kwargs.get('external', self.external),
                           kwargs.get('near', self.near), kwargs.get('goal', [800., 650.]))

    def test_reference_passes_without_depth_or_grasp_claim_or_mutation(self):
        before = copy.deepcopy(self.refs)
        result = self.assess()
        self.assertTrue(result['accepted'], result)
        self.assertFalse(result['evidence']['physicalGraspVerified'])
        self.assertFalse(result['evidence']['metricDepthVerified'])
        self.assertEqual(before, self.refs)

    def test_scalar_improvement_does_not_admit_crossing_or_aligned_axis_exit(self):
        for a,b in [([47.,47.],[-11.,-44.]), ([-47.,-47.],[11.,44.]), ([1.,47.],[3.,20.])]:
            self.assertFalse(gate.prediction(a,b,[0.,0.])['accepted'])
        self.assertTrue(gate.prediction([47.,47.],[20.,30.],[0.,0.])['accepted'])
        self.assertTrue(gate.prediction([47.,47.],[1.,-2.],[0.,0.])['accepted'])

    def test_entry_requires_both_axes_and_original_opening(self):
        self.assertFalse(self.assess(goal=[800.,653.])['accepted'])
        self.assertFalse(self.assess(near=near(80, version=3))['accepted'])

    def test_apparent_size_and_observer_contract_changes_rejected(self):
        # A valid second observer image with same opening but different object size.
        import cv2, numpy as np
        frame=np.full((240,320,3),180,np.uint8)
        for x in (130,190): cv2.rectangle(frame,(x-5,20),(x+5,95),(30,30,30),-1)
        cv2.rectangle(frame,(148,85),(171,105),(100,20,240),-1)
        from learning.near_gripper_tip_observer import NearGripperTipObserver
        ok,raw=cv2.imencode('.png',frame); self.assertTrue(ok)
        report=self.assess(near=NearGripperTipObserver(policy_version=3).observe(raw.tobytes()))
        self.assertFalse(report['accepted'])
        self.assertEqual(report['evidence']['reason'], 'entry_apparent_width_mismatch')
        self.assertFalse(self.assess(near=near(version=2))['accepted'])

    def test_missing_malformed_reference_and_forged_visibility_fail_closed(self):
        for refs in (None, [], [self.refs[0]], [{},{}]):
            self.assertFalse(self.assess(refs=refs)['accepted'])
        bad=copy.deepcopy(self.near);bad['nearClosingVisibilityGate']['passed']=False
        self.assertFalse(self.assess(near=bad)['accepted'])
        self.assertFalse(gate.prediction([float('nan'),1],[1,1],[0,0])['accepted'])
        bad=copy.deepcopy(self.refs);bad[1]['external']['registrationSha256']='c'*64
        self.assertFalse(self.assess(refs=bad)['accepted'])
