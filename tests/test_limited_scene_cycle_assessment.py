"""고정 구도의 영상 전환 정책 반례. 실물·접촉력 검증이 아니다."""
import copy
import unittest

import cv2
import numpy as np

from learning import limited_scene_cycle_assessment as gates
from learning.marker_virtual_observer import SCHEMA, PREFIX
from learning.near_gripper_tip_observer import NearGripperTipObserver


def near(gap=60, offset=0, version=2):
    image = np.full((240, 320, 3), 180, np.uint8)
    for x in (160-gap//2, 160+gap//2):
        cv2.rectangle(image, (x-5, 20), (x+5, 95), (30, 30, 30), -1)
    cv2.rectangle(image, (140+offset, 80), (179+offset, 120), (100, 20, 240), -1)
    ok, raw = cv2.imencode('.png', image)
    assert ok
    return NearGripperTipObserver(policy_version=version).observe(raw.tobytes())


def external(rise=0, dx=0, width=40, height=160):
    return dict(schemaVersion=SCHEMA, registrationSha256='a'*64, landmarkContract=PREFIX+'a'*64,
                imageSha256='b'*64, object=dict(bbox=[780+dx, 620-rise, width, height],
                                              center=[780+dx+width/2, 620-rise+height/2], area=width*height))


def record(rise=0, dx=0, gap=60, offset=0):
    return dict(external=external(rise, dx), near=near(gap, offset))


class LimitedSceneAssessmentTests(unittest.TestCase):
    def assertRejected(self, report, reason=None):
        self.assertFalse(report['accepted'])
        if reason:
            self.assertIn(reason, report['evidence']['reason'])

    def test_grasp_and_close_are_image_candidates_without_force_or_grasp_claim(self):
        initial = near(100); current = record()
        for report in (gates.grasp_candidate(**current),
                       gates.close_candidate(**current, initial_near=initial)):
            self.assertTrue(report['accepted'], report)
            self.assertFalse(report['evidence']['physicalGraspVerified'])
            self.assertFalse(report['evidence']['metricDepthVerified'])
            self.assertFalse(report['evidence']['contactForceVerified'])
            self.assertFalse(report['evidence']['rawSourceFreshnessVerified'])

    def test_missing_near_false_visibility_or_flat_external_rejects_grasp(self):
        self.assertRejected(gates.grasp_candidate(external(), None))
        bad = near(); bad['nearClosingVisibilityGate']['passed'] = False
        self.assertRejected(gates.grasp_candidate(external(), bad))
        self.assertRejected(gates.grasp_candidate(external(width=100, height=100), near()), 'not_upright')

    def test_close_requires_actual_decrease_and_current_width_not_fixed_old_reference(self):
        self.assertRejected(gates.close_candidate(external(), near(60), near(60)), 'did_not_decrease')
        self.assertRejected(gates.close_candidate(external(), near(80), near(100)), 'still_too_wide')
        self.assertRejected(gates.close_candidate(external(), near(60, 45), near(100)))

    def test_retained_lift_needs_both_rise_and_three_seconds(self):
        before, early, late = record(), record(40), record(39, 1)
        result = gates.retained_lift(before, early, late, 3.)
        self.assertTrue(result['accepted'], result)
        self.assertEqual(result['evidence']['objectBottomRisePx'], [40, 39])
        self.assertEqual(len(result['evidence']['sourceImages']), 3)
        self.assertRejected(gates.retained_lift(before, early, record(19), 3.), 'both_observations')
        for span in (2.99, float('nan'), float('inf'), True, '3'):
            self.assertRejected(gates.retained_lift(before, early, late, span), 'source_separation')

    def test_risen_but_dropped_or_slipping_object_is_not_retained(self):
        self.assertRejected(gates.retained_lift(record(), record(40), record(40, 9), 3.), 'not_stable')
        self.assertRejected(gates.retained_lift(record(), record(40, offset=26), record(40), 3.))
        self.assertRejected(gates.retained_lift(record(), record(40, gap=76), record(40), 3.), 'not_retained')

    def test_table_return_requires_saved_position_stability_upright_and_valid_dual(self):
        base, early, late = record(), record(6, 12), record(5, 11)
        self.assertTrue(gates.table_return(base, early, late, 3.)['accepted'])
        self.assertRejected(gates.table_return(base, record(7), record(), 3.), 'saved_table')
        self.assertRejected(gates.table_return(base, record(0, 13), record(), 3.), 'saved_table')
        self.assertRejected(gates.table_return(base, record(0, -5), record(0, 5), 3.), 'not_stable')
        flat = record(); flat['external'] = external(width=100, height=100)
        self.assertRejected(gates.table_return(base, flat, record(), 3.), 'not_upright')
        missing = record(); missing['near'] = None
        self.assertRejected(gates.table_return(base, missing, record(), 3.))

    def test_release_needs_opening_both_jaw_margins_and_table_position(self):
        before, after, table = record(gap=60), record(gap=100), record()
        result = gates.released(before, after, table)
        self.assertTrue(result['accepted'], result)
        self.assertEqual(result['evidence']['gapIncreasePx'], 40.)
        self.assertRejected(gates.released(before, record(gap=78), table), 'did_not_increase')
        self.assertRejected(gates.released(before, record(gap=100, offset=30), table), 'both_clear')
        self.assertRejected(gates.released(before, record(rise=7, gap=100), table), 'saved_table')

    def test_cross_registration_and_near_policy_contracts_cannot_mix(self):
        bad = record(40); bad['external']['registrationSha256'] = 'c'*64
        bad['external']['landmarkContract'] = PREFIX+'c'*64
        self.assertRejected(gates.retained_lift(record(), bad, record(40), 3.), 'contract_changed')
        different = record(); different['near'] = near(version=1)
        self.assertRejected(gates.table_return(record(), different, record(), 3.), 'contract_changed')

    def test_nan_outside_geometry_wrong_schema_and_missing_inputs_fail_closed(self):
        for mutation in ('nan', 'outside', 'schema', 'center', 'negative_width'):
            ext = external()
            if mutation == 'nan': ext['object']['bbox'][0] = float('nan')
            if mutation == 'outside': ext['object']['bbox'][0] = 1910
            if mutation == 'schema': ext['schemaVersion'] = 'unregistered'
            if mutation == 'center': ext['object']['center'][0] = 1200
            if mutation == 'negative_width': ext['object']['bbox'][2] = -1
            with self.subTest(mutation=mutation):
                self.assertRejected(gates.grasp_candidate(ext, near()))
        self.assertRejected(gates.table_return({}, record(), record(), 3.))
        malformed = near(); malformed['components']['left'] = None
        self.assertRejected(gates.grasp_candidate(external(), malformed))

    def test_assessments_do_not_mutate_inputs_or_share_policy_objects(self):
        before, early, late = record(), record(40), record(40)
        original = copy.deepcopy([before, early, late])
        report = gates.retained_lift(before, early, late, 3.)
        self.assertEqual([before, early, late], original)
        report['evidence']['policy']['minimumObjectRisePx'] = 0
        self.assertEqual(gates.POLICY['minimumObjectRisePx'], 20.)


if __name__ == '__main__':
    unittest.main()
