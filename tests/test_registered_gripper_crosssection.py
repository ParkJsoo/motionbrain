import copy
import unittest

import cv2
import numpy as np

from learning.registered_gripper_crosssection import (
    METHOD, PREFIX, STATE, RegisteredGripperCrosssection, register)


class CrosssectionTests(unittest.TestCase):
    def setUp(self):
        self.image = np.full((240,320,3), 220, np.uint8)
        rng = np.random.default_rng(42)
        self.image[67:133,140:206] = rng.integers(40,220,(66,66,3),dtype=np.uint8)
        self.image[40:67,76:103] = 20
        self.image[40:67,220:247] = 20
        self.context = dict(physicalView='test',object='cap',poseRegime='M1-only')
        self.raw = self.encode(self.image)
        self.registration = register(self.raw,[166,95],self.context)
        self.tracker = RegisteredGripperCrosssection(self.registration,self.raw)

    @staticmethod
    def encode(image):
        return cv2.imencode('.png',image)[1].tobytes()

    def test_object_occlusion_rejects_full_but_keeps_typed_shaft_measurement(self):
        changed = self.image.copy()
        changed[67:133,140:206] = 220
        raw = self.encode(changed)
        with self.assertRaises(ValueError):
            self.tracker.observe(raw,'object-occluded')
        result = self.tracker.observe_shaft(raw,'object-occluded')
        self.assertEqual(result['stateFields'],['shaftSeparationPx'])
        self.assertEqual(len(result['state']),1)
        self.assertEqual(result['state'],[result['shaftSeparationPx']])
        self.assertEqual(set(result['annotation']['points']),{'jawLeft','jawRight'})
        self.assertEqual(set(result['perPoint']),{'jawLeft','jawRight'})
        self.assertEqual(result['objectObservation']['status'],'not_required')
        self.assertTrue(result['annotation']['landmarkContract'].startswith('registered-gripper-shaft-only-v1:'))
        self.assertEqual(result['annotation']['method'],'registered_gripper_shaft_only_points')
        self.assertNotEqual(result['annotation']['landmarkContract'],self.tracker.landmark_contract)
        self.assertEqual(result,self.tracker.observe_shaft(raw,'object-occluded'))

    def test_reprocessing_identical_source_is_deterministic(self):
        self.assertEqual(self.tracker.observe(self.raw,'same'),self.tracker.observe(self.raw,'same'))

    def test_independent_shaft_movement_and_explicit_semantics(self):
        before = self.tracker.observe(self.raw,'before')
        changed = self.image.copy()
        changed[40:67,60:120] = 220
        changed[40:67,80:107] = 20
        after = self.tracker.observe(self.encode(changed),'after')
        self.assertAlmostEqual(after['annotation']['points']['jawLeft'][0]-before['annotation']['points']['jawLeft'][0],4)
        self.assertEqual(after['annotation']['points']['jawRight'],before['annotation']['points']['jawRight'])
        self.assertAlmostEqual(after['state'][2]-before['state'][2],-4)
        self.assertEqual(after['stateFields'],STATE)
        self.assertEqual(after['annotation']['method'],METHOD)
        self.assertTrue(after['annotation']['landmarkContract'].startswith(PREFIX))
        self.assertIn('not_tips',self.registration['seedSemantics'])
        self.assertFalse(after['controlEligible'])

    def test_cap_over_old_right_tip_patch_does_not_supply_shaft(self):
        changed = self.image.copy()
        changed[78:112,210:245] = 10
        before = self.tracker.observe(self.raw,'before')
        after = self.tracker.observe(self.encode(changed),'after')
        for name in ('jawLeft','jawRight'):
            self.assertEqual(before['annotation']['points'][name],after['annotation']['points'][name])

    def test_two_plausible_runs_rejected(self):
        changed = self.image.copy()
        changed[40:67,60:120] = 220
        changed[40:67,64:83] = 20
        changed[40:67,95:114] = 20
        with self.assertRaisesRegex(ValueError,'crosssection_ambiguous_run') as found:
            self.tracker.observe(self.encode(changed),'ambiguous')
        self.assertEqual(found.exception.diagnostics['failedJaw'],'jawLeft')

    def test_blurred_edge_uses_registered_two_pixel_probe(self):
        changed = self.image.copy()
        changed[45:66,60:120] = 130
        changed[45:66,76:103] = 97
        for x, value in [(75,110),(76,99),(77,98),(101,98),(102,99),(103,110)]:
            changed[45:66,x] = value
        result = self.tracker.observe(self.encode(changed),'blurred-edge')
        self.assertEqual(result['quality']['jawLeft']['acceptedRows'],21)
        self.assertEqual(result['quality']['jawLeft']['rows'][0]['selected']['edgeContrast'],[33.,33.])

    def test_low_edge_contrast_rejected(self):
        changed = self.image.copy()
        changed[45:66,205:260] = 105
        changed[45:66,220:247] = 90
        with self.assertRaisesRegex(ValueError,'crosssection_rows_insufficient'):
            self.tracker.observe(self.encode(changed),'low-contrast')

    def test_missing_rows_never_filled_from_other_jaw(self):
        changed = self.image.copy()
        changed[45:51,205:260] = 220
        with self.assertRaisesRegex(ValueError,'crosssection_rows_insufficient'):
            self.tracker.observe(self.encode(changed),'missing')

    def test_inconsistent_rows_rejected(self):
        changed = self.image.copy()
        changed[45:66,60:120] = 220
        for y in range(45,66):
            x=76 if y%2 else 84
            changed[y,x:x+27] = 20
        with self.assertRaisesRegex(ValueError,'crosssection_rows_inconsistent'):
            self.tracker.observe(self.encode(changed),'zigzag')
        with self.assertRaisesRegex(ValueError,'crosssection_rows_inconsistent'):
            self.tracker.observe_shaft(self.encode(changed),'zigzag-shaft-only')

    def test_reference_and_mutated_contract_rejected(self):
        changed = self.image.copy(); changed[0,0] = 0
        with self.assertRaisesRegex(ValueError,'crosssection_contract_changed'):
            RegisteredGripperCrosssection(self.registration,self.encode(changed))
        self.tracker.registration['parameters']['shaft']['darkThreshold']=101
        with self.assertRaisesRegex(ValueError,'crosssection_contract_changed'):
            self.tracker.observe(self.raw,'tampered')
        changed_reg=copy.deepcopy(self.registration)
        changed_reg['stateFields'][-1]='jawOpeningPx'
        with self.assertRaisesRegex(ValueError,'crosssection_contract_fields'):
            RegisteredGripperCrosssection(changed_reg,self.raw)

    def test_object_search_translation_and_no_posthoc_alignment(self):
        changed = self.image.copy()
        changed[67:133,140:216] = 220
        changed[67:133,150:216] = self.image[67:133,140:206]
        result=self.tracker.observe(self.encode(changed),'object-moved')
        self.assertAlmostEqual(result['annotation']['points']['object'][0],176,delta=.1)
        self.assertAlmostEqual(result['state'][2],144,delta=.1)


if __name__ == '__main__':
    unittest.main()
