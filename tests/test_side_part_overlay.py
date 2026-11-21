import json
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.side_part_overlay import COLORS, _base_rings, _housing_pair, render_side
from learning.open_side_approach import WIDE_SIDE_VIEW


class SidePartOverlayTest(unittest.TestCase):
    def test_wide_view_keeps_verified_flange_pair_when_single_cap_is_hidden(self):
        frame = np.full((1080, 1920, 3), 35, np.uint8)
        shoulder, elbow = np.array([450., 360.]), np.array([425., 218.])
        with patch('learning.side_part_overlay._base_rings', return_value=(360, 425, 160, 36)), \
             patch('learning.side_part_overlay._housing_pair',
                   side_effect=[(shoulder, None), (shoulder, elbow)]):
            _, info = render_side(frame, view_id=WIDE_SIDE_VIEW)
        self.assertEqual(info['landmarks']['elbowHousingCenterEstimate'], [850, 436])
        self.assertEqual(info['parts']['upper_arm'], 'estimated_visible')

    def test_wide_link_mask_is_displayed_even_when_the_jaw_tip_is_hidden(self):
        frame = np.full((1080, 1920, 3), 210, np.uint8)
        mask = np.zeros((540, 960), np.uint8)
        mask[200:220, 300:330] = 255
        with patch('learning.side_part_overlay.wide_distal_parts',
                   return_value=({'forearm': mask}, {})) as detector:
            out, info = render_side(frame, view_id=WIDE_SIDE_VIEW)
            self.assertEqual(info['parts']['forearm'], 'estimated_visible')
            self.assertNotIn('visibleJawTip', info['landmarks'])
            self.assertFalse(np.array_equal(out[420, 630], frame[420, 630]))
            self.assertFalse(info['controlEligible'])
            _, legacy = render_side(frame)
            self.assertEqual(legacy['parts']['forearm'], 'unresolved')
            detector.assert_called_once()

    def test_target_contour_is_from_current_pixels_and_input_is_unchanged(self):
        rgb = np.full((1080, 1920, 3), 222, np.uint8)
        for x, y in [(320, 650), (810, 500)]:
            frame = rgb.copy()
            frame[y:y+160, x:x+52] = (235, 23, 106)
            raw = frame.copy()
            out, meta = render_side(frame)
            np.testing.assert_array_equal(raw, frame)
            self.assertEqual(meta['landmarks']['objectBox'], [x, y, 52, 160])
            self.assertFalse(meta['controlEligible'])
            self.assertEqual(meta['parts']['object'], 'estimated_visible')
            self.assertTrue(all(meta['parts'][k] == 'unresolved' for k in COLORS if k != 'object'))
            self.assertFalse(np.array_equal(out[y+70, x+18], raw[y+70, x+18]))
            np.testing.assert_array_equal(out[:y-5], raw[:y-5])
            json.dumps(meta, allow_nan=False)
        absent, meta = render_side(rgb)
        np.testing.assert_array_equal(absent, rgb)
        self.assertEqual(meta['parts']['object'], 'unresolved')

    def test_truncated_or_background_bands_do_not_form_a_base(self):
        for y in (0, 500):
            rgb = np.full((1080, 1920, 3), 210, np.uint8)
            for dy in (0, 32):
                rgb[y+dy:y+dy+12, 470:810] = (230, 170, 42)
            out, meta = render_side(rgb)
            self.assertTrue(all(s == 'unresolved' for s in meta['parts'].values()))
            np.testing.assert_array_equal(out, rgb)

    def test_joined_bands_require_distinct_slats(self):
        mask = np.zeros((540, 960), np.uint8)
        for y in (330, 347, 364):
            mask[y:y+8, 400:560] = 255
        mask[330:372, 525:565] = 255
        self.assertIsNotNone(_base_rings(mask))
        mask[330:372, 400:565] = 255
        self.assertIsNone(_base_rings(mask))

    def test_flange_and_shaft_must_both_be_visible(self):
        gold = np.zeros((540, 960), np.uint8)
        cv2.ellipse(gold, (351,226), (35,31), 0, 0, 360, 255, -1)
        cv2.ellipse(gold, (359,224), (28,25), 0, 0, 360, 0, -1)
        cv2.ellipse(gold, (353,90), (51,32), 0, 0, 360, 255, -1)
        cv2.circle(gold, (353,90), 23, 0, -1)
        rgb = np.full((540,960,3), 210, np.uint8)
        cv2.line(rgb, (355,226), (353,90), (45,45,45), 40)
        cv2.circle(rgb, (353,90), 28, (45,45,45), -1)
        cv2.circle(rgb, (352,226), 24, (45,45,45), -1)
        rgb[gold != 0] = (210,157,43)
        rings = (275,283,150,40)
        _, elbow = _housing_pair(rgb, gold, rings)
        self.assertLess(np.linalg.norm(elbow - [353,90]), 12)
        rgb[125:175] = 210
        _, elbow = _housing_pair(rgb, gold, rings)
        self.assertIsNone(elbow)
        rgb[:210] = 45  # Dark background offers no independent shaft edges.
        _, elbow = _housing_pair(rgb, gold, rings)
        self.assertIsNone(elbow)

    def test_tip_remains_observable_without_object_or_housing(self):
        rgb = np.full((1080, 1920, 3), 238, np.uint8)
        for center in [(1154, 353), (1218, 322)]:
            cv2.circle(rgb, center, 15, (85,150,97), -1)
        rgb[460:492,762:1030] = 38
        _, meta = render_side(rgb)
        self.assertEqual(meta['landmarks']['visibleJawTip'], [762,475.5])
        self.assertEqual(meta['parts']['upper_arm'], 'unresolved')

    def test_rejects_input_outside_capture_contract(self):
        with self.assertRaisesRegex(ValueError, 'side_1080p'):
            render_side(np.zeros((240,320,3),np.uint8))
