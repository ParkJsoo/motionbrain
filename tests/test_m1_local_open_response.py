"""고정 원본 기반 1-NN OPEN 응답과 원 실험·지원 한도의 무장치 회귀."""
import copy
import unittest
import numpy as np
import cv2
from learning.near_gripper_tip_observer import NearGripperTipObserver
from learning import m1_local_open_response as local
from learning import m1_near_action_model as frozen
from tests import test_gripper_recovery as fixture
from tests.test_limited_scene_cycle_assessment import near


def long_jaw_observation():
    image=np.full((240,320,3),180,np.uint8)
    for x in (130,190):cv2.rectangle(image,(x-5,20),(x+5,135),(30,30,30),-1)
    cv2.rectangle(image,(140,80),(179,120),(100,20,240),-1)
    raw=cv2.imencode('.png',image)[1].tobytes()
    return NearGripperTipObserver(policy_version=3).observe(raw)


class LocalOpenTests(unittest.TestCase):
    def setUp(self):
        setup=fixture.RecoveryTests();setup.setUp()
        self.source=setup.model;self.current=setup.partial
    def fit(self):return local.fit(self.source,audit_sources=lambda rows:rows)
    def test_fixed_source_binding_nearest_delta_and_single_donor_are_explicit(self):
        before=copy.deepcopy(self.source);model=self.fit();local.validate_source(model,self.source)
        p=local.predict(model,self.current)
        self.assertEqual(p['predictedState'],self.source['referenceObservation']['state'])
        self.assertEqual(p['support']['nearestSourceSessionId'],'2')
        self.assertTrue(p['candidateEligible']);self.assertFalse(p['controlEligible'])
        self.assertFalse(p['prospectivelyTested']);self.assertFalse(p['support']['independentValidation'])
        self.assertEqual(self.source,before);self.assertFalse(model['supportPolicy']['outputClipped'])
        self.assertNotIn('minimumNearbySourcesForCandidate',model['supportPolicy'])
    def test_original_experiment_cap_rejects_without_clipping(self):
        with self.assertRaisesRegex(ValueError,'predicted_gap_outside_original_experiment'):
            local.predict(self.fit(),near(80,version=3))
    def test_changed_source_reference_contract_or_policy_is_rejected(self):
        model=self.fit();changed=copy.deepcopy(self.source);changed['models'][0]['deltaMean'][0]+=1
        with self.assertRaisesRegex(ValueError,'frozen_source_changed'):local.validate_source(model,changed)
        changed=copy.deepcopy(model);changed['maximumOpeningGapPx']+=10
        with self.assertRaisesRegex(ValueError,'reference_bounds_changed'):local.predict(changed,self.current)
        changed=copy.deepcopy(model);changed['supportPolicy']['maximumDistance']=1.1
        with self.assertRaisesRegex(ValueError,'development_model_required'):local.predict(changed,self.current)
        current=copy.deepcopy(self.current);current['observationContractSha256']='f'*64
        with self.assertRaises(ValueError):local.predict(model,current)
    def test_reaudit_missing_or_changed_originals_do_not_fit(self):
        with self.assertRaisesRegex(ValueError,'reaudit_changed'):local.fit(self.source,audit_sources=lambda rows:[])
        changed=copy.deepcopy(self.source);changed['trainingRows']=[r for r in changed['trainingRows'] if r['sessionId']!='3']
        with self.assertRaisesRegex(ValueError,'two_audited_sources'):local.fit(changed,audit_sources=lambda rows:rows)
        changed=copy.deepcopy(self.source);changed['trainingRows'][-1]['sourceEvidence']['beforeGripperImageSha256']='f'*64
        with self.assertRaisesRegex(ValueError,'two_camera_original'):local.fit(changed,audit_sources=lambda rows:rows)
    def test_local_radius_remains_original_radius_and_result_does_not_alias_source(self):
        model=self.fit();current=long_jaw_observation()
        with self.assertRaisesRegex(ValueError,'outside_local_support'):local.predict(model,current)
        p=local.predict(model,self.current);p['observedSourceDeltaState'][2]=999
        self.assertNotEqual(local.predict(model,self.current)['observedSourceDeltaState'][2],999)
    def test_one_nearby_source_is_hypothesis_not_new_quorum_or_automatic_validation(self):
        # Separate the donor states in the original y coordinate while keeping
        # both raw observations structurally valid, as the existing family permits.
        src=copy.deepcopy(self.source)
        src['trainingRows'][-1]['before']=long_jaw_observation()
        src['trainingRows'][-1]['sourceEvidence']['beforeGripperImageSha256']=src['trainingRows'][-1]['before']['imageSha256']
        model=local.fit(src,audit_sources=lambda rows:rows)
        p=local.predict(model,self.current)
        self.assertEqual(p['support']['nearbySourceCount'],1)
        self.assertTrue(p['candidateEligible']);self.assertFalse(p['controlEligible'])
        self.assertIn('single_nearest_source_hypothesis',p['support']['transferMeaning'])

if __name__=='__main__':unittest.main()
