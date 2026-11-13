import copy
import unittest

import cv2
import numpy as np

from learning.registered_gripper_crosssection import (
    METHOD, PREFIX, STATE, RegisteredGripperCrosssection, register, register_shaft_only, default_parameters)


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

    def ambiguous_object_image(self):
        image=self.image.copy()
        image[43:157,116:219]=220
        yy,xx=np.indices((106,99))
        image[47:153,120:219]=np.repeat((140+80*((xx//2+yy//2)%2))[:,:,None],3,axis=2)
        return image

    def test_shaft_only_registration_skips_only_ambiguous_object_NCC_selfcheck(self):
        raw=self.encode(self.ambiguous_object_image())
        with self.assertRaisesRegex(ValueError,'scene_match_ambiguous'):
            register(raw,[166,95],self.context)
        registration=register_shaft_only(raw,[166,95],self.context)
        self.assertEqual(registration['objectSeedStatus'],'unverified_seed_only_shaft_registration')
        self.assertEqual(registration['stateFields'],STATE)
        tracker=RegisteredGripperCrosssection(registration,raw)
        result=tracker.observe_shaft(raw,'ambiguous-object-shaft-only')
        self.assertEqual(result['objectObservation']['status'],'not_required')
        self.assertEqual(set(result['annotation']['points']),{'jawLeft','jawRight'})
        self.assertFalse(result['contactVerified'])
        with self.assertRaisesRegex(ValueError,'full_observation_requires_object_registration'):
            tracker.observe(raw,'cannot_claim_object_tracking')

    def test_shaft_only_registration_preserves_all_common_reference_quality_and_hash_gates(self):
        registration=register_shaft_only(self.raw,[166,95],self.context)
        self.assertNotEqual(registration['registrationSha256'],self.registration['registrationSha256'])
        self.assertEqual(registration['parametersSha256'],self.registration['parametersSha256'])
        for mutation in ('missing_shaft','no_texture','invalid_seed'):
            image=self.image.copy();seed=[166,95]
            if mutation=='missing_shaft':image[40:67,60:120]=220
            if mutation=='no_texture':image[67:133,140:206]=220
            if mutation=='invalid_seed':seed=[0,0]
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):
                register_shaft_only(self.encode(image),seed,self.context)
        for mutation in ('raw','hash','metadata','shaft'):
            reg=copy.deepcopy(registration);raw=self.raw
            if mutation=='raw':raw=self.encode(self.ambiguous_object_image())
            if mutation=='hash':reg['referenceImageSha256']='0'*64
            if mutation=='metadata':reg['objectSeedStatus']='object_tracking_verified'
            if mutation=='shaft':reg['points']['jawLeft'][0]+=1
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):
                RegisteredGripperCrosssection(reg,raw)

    def test_full_registration_stays_unchanged_and_shaft_only_context_is_separate(self):
        self.assertNotIn('objectSeedStatus',self.registration)
        self.assertEqual(register(self.raw,[166,95],self.context),self.registration)
        new=register_shaft_only(self.raw,[166,95],self.context)
        self.assertEqual(RegisteredGripperCrosssection(new,self.raw).observe_shaft(self.raw,'shaft')['state'],
                         self.tracker.observe_shaft(self.raw,'shaft')['state'])
        self.assertNotEqual(RegisteredGripperCrosssection(new,self.raw).shaft_landmark_contract,
                            self.tracker.shaft_landmark_contract)
        self.assertEqual(self.tracker.observe(self.raw,'full')['stateFields'],STATE)

    def test_reprocessing_identical_source_is_deterministic(self):
        self.assertEqual(self.tracker.observe(self.raw,'same'),self.tracker.observe(self.raw,'same'))

    def test_current_M1_close_preset_is_separate_and_only_changes_right_ROI(self):
        old=default_parameters();current=default_parameters(preset='current_m1_close')
        expected=copy.deepcopy(old);expected['shaft']['rois']['jawRight']=[190,260]
        self.assertEqual(current,expected)
        self.assertEqual(old['shaft']['rois']['jawRight'],[205,260])
        reg=register(self.raw,[166,95],self.context,current)
        self.assertNotEqual(reg['parametersSha256'],self.registration['parametersSha256'])
        self.assertNotEqual(reg['registrationSha256'],self.registration['registrationSha256'])
        with self.assertRaisesRegex(ValueError,'unknown_parameter_preset'):default_parameters(preset='auto_expand')

    def test_closed_right_shaft_clipped_in_old_ROI_is_valid_only_under_new_registration(self):
        parameters=default_parameters(preset='current_m1_close')
        reg=register(self.raw,[166,95],self.context,parameters)
        current=RegisteredGripperCrosssection(reg,self.raw)
        for center in (208,213,218):
            changed=self.image.copy();changed[40:67,190:260]=220
            changed[40:67,center-13:center+14]=20
            raw=self.encode(changed)
            with self.subTest(center=center),self.assertRaisesRegex(ValueError,'crosssection_rows_insufficient'):
                self.tracker.observe_shaft(raw,f'old-closed-{center}')
            result=current.observe_shaft(raw,f'new-closed-{center}')
            self.assertAlmostEqual(result['annotation']['points']['jawRight'][0],center)
            self.assertEqual(result['quality']['jawRight']['acceptedRows'],21)

    def test_new_ROI_still_rejects_two_runs_missing_rows_and_weaker_quality(self):
        parameters=default_parameters(preset='current_m1_close')
        reg=register(self.raw,[166,95],self.context,parameters)
        current=RegisteredGripperCrosssection(reg,self.raw)
        changed=self.image.copy();changed[40:67,190:260]=220
        changed[40:67,193:213]=20;changed[40:67,233:253]=20
        with self.assertRaisesRegex(ValueError,'crosssection_ambiguous_run'):
            current.observe_shaft(self.encode(changed),'two-current-runs')
        changed=self.image.copy();changed[45:51,190:260]=220
        with self.assertRaisesRegex(ValueError,'crosssection_rows_insufficient'):
            current.observe_shaft(self.encode(changed),'current-missing-rows')
        for mutation in ('contrast','minimumRows','leftROI','rightROI'):
            changed_parameters=copy.deepcopy(parameters)
            if mutation=='contrast':changed_parameters['shaft']['minimumEdgeContrast']=29.
            if mutation=='minimumRows':changed_parameters['shaft']['minimumRows']=15
            if mutation=='leftROI':changed_parameters['shaft']['rois']['jawLeft']=[59,120]
            if mutation=='rightROI':changed_parameters['shaft']['rois']['jawRight']=[189,260]
            with self.subTest(mutation=mutation),self.assertRaisesRegex(ValueError,'fixed_configuration_changed'):
                register(self.raw,[166,95],self.context,changed_parameters)

    def distal_image(self):
        image=self.image.copy()
        image[35:67,60:120]=220;image[35:67,190:260]=220
        image[70:91,80:107]=20;image[70:91,220:247]=20
        return image

    def test_distal_preset_is_explicit_extrapolation_to55_with_new_semantics(self):
        raw=self.encode(self.distal_image())
        parameters=default_parameters(preset='current_m1_distal_shaft')
        self.assertEqual(parameters['shaft']['rows'],[70,90])
        self.assertEqual(parameters['shaft']['referenceRow'],55)
        reg=register(raw,[166,95],self.context,parameters)
        self.assertIn('rows70_90_extrapolated_to_image_row55',reg['seedSemantics'])
        self.assertIn('not_direct_row55_pixels',reg['seedSemantics'])
        tracker=RegisteredGripperCrosssection(reg,raw)
        result=tracker.observe_shaft(raw,'distal-fit')
        for name in ('jawLeft','jawRight'):
            self.assertEqual(result['annotation']['points'][name][1],55)
            q=result['quality'][name]
            self.assertEqual(q['acceptedRows'],21)
            self.assertEqual(q['extrapolationBeyondNearestObservedRowPx'],15)
            self.assertEqual(q['distanceFromWindowCenterPx'],25)
            self.assertFalse(q['projectionUncertaintyCalibrated'])
            self.assertFalse(q['measurementAccuracyVerified'])
        self.assertFalse(result['measurementAccuracyVerified'])
        self.assertNotEqual(reg['parametersSha256'],self.registration['parametersSha256'])
        with self.assertRaisesRegex(ValueError,'crosssection_rows_insufficient'):
            self.tracker.observe_shaft(raw,'no-row55-shaft')

    def test_distal_preset_preserves_quality_gates_and_rejects_unregistered_row80(self):
        image=self.distal_image();raw=self.encode(image)
        parameters=default_parameters(preset='current_m1_distal_shaft')
        reg=register(raw,[166,95],self.context,parameters);tracker=RegisteredGripperCrosssection(reg,raw)
        changed=image.copy();changed[70:91,190:260]=220
        changed[70:91,193:213]=20;changed[70:91,233:253]=20
        with self.assertRaisesRegex(ValueError,'crosssection_ambiguous_run'):
            tracker.observe_shaft(self.encode(changed),'ambiguous-distal')
        changed=image.copy();changed[70:76,190:260]=220
        with self.assertRaisesRegex(ValueError,'crosssection_rows_insufficient'):
            tracker.observe_shaft(self.encode(changed),'missing-distal')
        for key,value in [('minimumRows',15),('minimumRowSpan',15),('maximumCenterLineResidualPx',2.),
                          ('maximumWidthRangePx',9),('referenceRow',80),('rows',[69,90])]:
            changed_parameters=copy.deepcopy(parameters);changed_parameters['shaft'][key]=value
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'fixed_configuration_changed'):
                register(raw,[166,95],self.context,changed_parameters)

    def test_distal_projection_reports_distance_from_actual_accepted_rows(self):
        image=self.distal_image();raw=self.encode(image)
        reg=register(raw,[166,95],self.context,default_parameters(preset='current_m1_distal_shaft'))
        tracker=RegisteredGripperCrosssection(reg,raw)
        changed=image.copy();changed[70:74,190:260]=220
        observed=tracker.observe_shaft(self.encode(changed),'partial-distal-window')
        quality=observed['quality']['jawRight']
        self.assertEqual(quality['acceptedRows'],17)
        self.assertEqual(quality['acceptedRowRange'],[74,90])
        self.assertEqual(quality['extrapolationBeyondNearestObservedRowPx'],19)
        self.assertFalse(quality['projectionUncertaintyCalibrated'])

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


class ObservedShaft95Tests(unittest.TestCase):
    def setUp(self):
        self.p = default_parameters(preset='current_m1_observed_shaft95')
        self.image = np.full((240, 320, 3), 220, np.uint8)
        self.image[67:133, 150:164] = np.random.default_rng(5).integers(120, 220, (66, 14, 3), dtype=np.uint8)
        self.image[70:116, 80:107] = 20
        self.image[70:116, 200:227] = 20
        self.raw = CrosssectionTests.encode(self.image)
        self.reg = register(self.raw, [160, 95], dict(physicalView='synthetic', object='test', poseRegime='M1'), self.p)
        self.tracker = RegisteredGripperCrosssection(self.reg, self.raw)

    def observe(self, image):
        return self.tracker.observe_shaft(CrosssectionTests.encode(image), 'fixed-test')

    def test_same_run_multiple_witnesses_report_direct95_deterministically(self):
        image = self.image.copy()
        # 합집합 폭 변화9는 통과창이 아니지만 양쪽의 local 창은 각각 엄격하게 유효하다.
        image[70:116, 200:227] = 220
        for y in range(70, 116):
            width = 33 if y == 70 else 24 if y == 115 else 27
            center = 214 if y == 95 else 213
            image[y, center-width//2:center-width//2+width] = 20
        result = self.observe(image)
        self.assertEqual(result, self.observe(image))
        q = result['quality']['jawRight']
        self.assertGreater(q['qualityWitnessCount'], 1)
        self.assertEqual(q['point'], [q['primaryRun']['centerPx'], 95])
        self.assertAlmostEqual(q['point'][0], 214)
        self.assertTrue(any(abs(w['fittedPoint'][0]-q['point'][0]) > .1 for w in q['qualityWitnesses']))
        self.assertTrue(all(w['widthRangePx'][1]-w['widthRangePx'][0] <= 8 for w in q['qualityWitnesses']))
        self.assertFalse(q['sameRunWitnessesAreIndependentObservations'])
        self.assertFalse(q['extrapolationUsed'])
        self.assertIn('unique_direct_observed_image_row95', self.reg['seedSemantics'])

    def test_real_two_runs_any_scan_row_reject(self):
        image = self.image.copy(); image[70, 235:255] = 20
        with self.assertRaisesRegex(ValueError, 'crosssection_ambiguous_run'):
            self.observe(image)
        image = self.image.copy(); image[95, 235:255] = 20
        with self.assertRaisesRegex(ValueError, 'crosssection_ambiguous_run'):
            self.observe(image)

    def test_primary_row_missing_bad_contrast_and_width_reject(self):
        for value, width in ((220,27), (90,27), (20,17)):
            image = self.image.copy(); image[95, 165:260] = 110 if value == 90 else 220
            image[95, 200:200+width] = value
            with self.subTest(value=value, width=width), self.assertRaisesRegex(ValueError, 'reference_row_not_observed'):
                self.observe(image)

    def test_insufficient_count_span_residual_and_width_support_reject(self):
        for kind in ('count', 'span', 'residual', 'width'):
            image = self.image.copy(); image[70:116, 165:260] = 220
            rows = range(88,103) if kind == 'count' else range(88,104) if kind == 'span' else range(70,116)
            for y in rows:
                width = (18 if y%2 else 36) if kind == 'width' else 27
                center = (205 if y%2 else 215) if kind == 'residual' else 213
                image[y, center-width//2:center-width//2+width] = 20
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, 'rows_insufficient|local_support_insufficient'):
                self.observe(image)

    def test_parameter_contract_is_fixed_and_old_semantics_unchanged(self):
        from learning.registered_gripper_crosssection import SEMANTICS, DISTAL_SEMANTICS, _seed_semantics
        self.assertEqual(_seed_semantics(default_parameters()), SEMANTICS)
        self.assertEqual(_seed_semantics(default_parameters(preset='current_m1_close')), SEMANTICS)
        self.assertEqual(_seed_semantics(default_parameters(preset='current_m1_distal_shaft')), DISTAL_SEMANTICS)
        p = copy.deepcopy(self.p); p['shaft']['maximumWidthRangePx'] = 9
        with self.assertRaisesRegex(ValueError, 'fixed_configuration_changed'):
            register(self.raw, [160,95], self.reg['sceneContext'], p)


if __name__ == '__main__':
    unittest.main()
