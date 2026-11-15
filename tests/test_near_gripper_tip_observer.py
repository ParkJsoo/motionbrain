from pathlib import Path
import copy
import unittest

import cv2
import numpy as np

from learning.near_gripper_tip_observer import NearGripperTipObserver, validate_observation


def image(gap=140., object_width=55):
    pixels = np.full((240, 320, 3), 180, np.uint8)
    for x in [int(160-gap/2), int(160+gap/2)]:
        cv2.rectangle(pixels, (x-7, 20), (x+7, 95), (30, 30, 30), -1)
    cv2.rectangle(pixels, (160-object_width//2, 110), (160+(object_width-1)//2, 165), (100, 20, 240), -1)
    return pixels


def encoded(pixels):
    ok, raw = cv2.imencode('.png', pixels)
    if not ok: raise RuntimeError('encoding failed')
    return raw.tobytes()


class TipObserverTests(unittest.TestCase):
    def test_open_and_partial_close_track_visible_tips(self):
        observer = NearGripperTipObserver()
        for gap in [140., 80., 50.]:
            result = observer.observe(encoded(image(gap)))
            self.assertAlmostEqual(result['state'][2], gap)
            self.assertEqual(result['objectVisibleWidthPx'], 55.)
            self.assertTrue(result['nearClosingVisibilityGate']['passed'])
            self.assertEqual(len(validate_observation(result)), 3)
            self.assertFalse(result['contactVerified']); self.assertFalse(result['forceVerified'])

    def test_missing_occluded_ambiguous_or_merged_tips_rejected(self):
        for mode in ['missing', 'split_occluded', 'ambiguous', 'merged']:
            pixels = image()
            if mode == 'missing': pixels[20:100, 80:100] = 180
            elif mode == 'split_occluded': pixels[65:72, 80:100] = 180
            elif mode == 'ambiguous': cv2.rectangle(pixels, (110, 20), (125, 90), (30, 30, 30), -1)
            else: pixels = image(8.)
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                NearGripperTipObserver().observe(encoded(pixels))

    def test_missing_or_duplicate_object_rejected(self):
        for mode in ['missing', 'duplicate']:
            pixels = image()
            if mode == 'missing': pixels[108:168, 130:190] = 180
            else: cv2.rectangle(pixels, (50, 115), (75, 150), (100, 20, 240), -1)
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, 'object_missing_or_ambiguous'):
                NearGripperTipObserver().observe(encoded(pixels))

    def test_state_or_gate_tampering_is_not_accepted_by_model_input_validator(self):
        report = NearGripperTipObserver().observe(encoded(image()))
        changed = copy.deepcopy(report); changed['state'][2] -= 10.
        with self.assertRaisesRegex(ValueError, 'state_changed'): validate_observation(changed)
        changed = copy.deepcopy(report); changed['nearClosingVisibilityGate']['passed'] = False
        with self.assertRaisesRegex(ValueError, 'visibility_gate_changed'): validate_observation(changed)

    def test_real_QVGA_current_raw_when_available(self):
        root = Path(__file__).resolve().parents[1]/'.local/current-entry-20261114'
        matches = list((root/'L-pose-M4-down70-1000-M420039-02/sessions').glob('*/gripper-pair/esp32-after.jpg'))
        if not matches: self.skipTest('private current frame unavailable')
        report = NearGripperTipObserver().observe(matches[0].read_bytes())
        np.testing.assert_allclose(report['points']['left'], [89.235, 94.], atol=1.)
        np.testing.assert_allclose(report['points']['right'], [224.055, 95.], atol=1.)
        self.assertAlmostEqual(report['state'][2], 134.82, delta=1.)
        self.assertTrue(report['nearClosingVisibilityGate']['passed'])

    def test_profile_v2_preserves_v1_and_requires_both_finger_vertical_overlap(self):
        pixels = image()
        old = NearGripperTipObserver().observe(encoded(pixels))
        current = NearGripperTipObserver(policy_version=2).observe(encoded(pixels))
        self.assertTrue(old['nearClosingVisibilityGate']['passed'])
        self.assertFalse(current['nearClosingVisibilityGate']['passed'])
        self.assertNotEqual(old['policySha256'], current['policySha256'])
        self.assertEqual(NearGripperTipObserver().binding,
                         'ce02359a9178f0357175b9e35f2a1bdc36086e5b5d240151b2e12ff7e9ad3102')
        pixels[110:166, 130:190] = 180
        cv2.rectangle(pixels, (142, 82), (180, 122), (100, 20, 240), -1)
        report = NearGripperTipObserver(policy_version=2).observe(encoded(pixels))
        self.assertTrue(report['nearClosingVisibilityGate']['passed'])
        self.assertTrue(all(v >= 3. for v in report['nearClosingVisibilityGate']['verticalProfileOverlapPx'].values()))
        self.assertEqual(len(validate_observation(report)), 3)

    def test_real_close_scenes_v2_admits_object_between_long_finger_profiles(self):
        root = Path(__file__).resolve().parents[1]/'.local/current-entry-20261114'
        paths = [list((root/name/'sessions').glob('*/gripper-pair/esp32-after.jpg')) for name in
                 ['L-pose-M1-close70-1000-01', 'L-pose-M1-close70-1000-02']]
        if not all(paths): self.skipTest('private current frames unavailable')
        old = NearGripperTipObserver(); observer = NearGripperTipObserver(policy_version=2)
        reports = [observer.observe(paths[i][0].read_bytes()) for i in range(2)]
        self.assertTrue(all(r['nearClosingVisibilityGate']['passed'] for r in reports))
        self.assertFalse(old.observe(paths[1][0].read_bytes())['nearClosingVisibilityGate']['passed'])
        for report in reports: validate_observation(report)
        self.assertAlmostEqual(reports[1]['state'][2], 66.2443, delta=1.)


if __name__ == '__main__': unittest.main()
