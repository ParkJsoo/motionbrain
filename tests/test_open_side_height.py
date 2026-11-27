import unittest

import cv2
import numpy as np

from learning.open_side_approach import LEGACY_SIDE_VIEW, observe_open_side, observe_open_side_height


def check_height(image):
    return observe_open_side_height(image, view_id=LEGACY_SIDE_VIEW)


def image_with_jaw(*, marked_wrist=False):
    rgb = np.full((1080, 1920, 3), 225, np.uint8)
    rgb[440:630, 790:850] = (235, 23, 106)
    rgb[465:510, 770:1030] = 38
    if marked_wrist:
        cv2.ellipse(rgb, (961, 481), (15, 8), 0, 0, 360, (141, 87, 121), -1)
        cv2.ellipse(rgb, (993, 474), (13, 10), 0, 0, 360, (85, 150, 97), -1)
        cv2.circle(rgb, (789, 487), 9, (190, 168, 49), -1)
    else:
        for center in ((1154, 353), (1218, 322)):
            cv2.circle(rgb, center, 15, (85, 150, 97), -1)
    return rgb


class OpenSideHeightTest(unittest.TestCase):
    def test_observed_extents_survive_occlusion_only_in_height_readout(self):
        frame = image_with_jaw()
        with self.assertRaisesRegex(ValueError, 'upright_prism'):
            observe_open_side(frame, view_id=LEGACY_SIDE_VIEW)
        g = check_height(frame)
        self.assertEqual(g['box'], [790, 440, 60, 190])
        self.assertEqual(g['toolPoint'], [770, 487.0])
        self.assertTrue(g['withinHeight'])

    def test_gap_without_contiguous_jaw_support_is_not_filled(self):
        frame = image_with_jaw()
        frame[457:465, 790:850] = 225
        with self.assertRaisesRegex(ValueError, 'upright_prism'):
            check_height(frame)

    def test_second_prism_or_clipped_candidate_still_abstains(self):
        for x in (0, 1330):
            frame = image_with_jaw()
            frame[650:840, x:x+60] = (235, 23, 106)
            with self.assertRaisesRegex(ValueError, 'upright_prism'):
                check_height(frame)

    def test_repair_cannot_discard_multiple_raw_candidates_to_select_a_distractor(self):
        frame = image_with_jaw()
        frame[440:465, 790:850] = 225
        frame[510:630, 790:850] = 225
        for region in (np.s_[425:465, 795:808], np.s_[510:550, 795:808],
                       np.s_[435:625, 1370:1430]):
            frame[region] = (235, 23, 106)
        with self.assertRaisesRegex(ValueError, 'upright_prism'):
            check_height(frame)

    def test_marked_wrist_measures_height_gap_without_object_as_anchor(self):
        frame = image_with_jaw(marked_wrist=True)
        frame[440:465, 790:850] = 225
        frame[510:630, 790:850] = 225
        frame[580:770, 805:865] = (235, 23, 106)
        result = check_height(frame)
        self.assertEqual(result['trackingMode'], 'wrist_and_distal_mark')
        self.assertEqual(result['tipOffsetFromTopPx'], -93.)
        self.assertFalse(result['withinHeight'])
        with self.assertRaisesRegex(ValueError, 'single_marker_jaw'):
            observe_open_side(frame, view_id=LEGACY_SIDE_VIEW)

    def test_distal_mark_must_be_inside_wrist_connected_dark_digit(self):
        for fault in ('missing', 'background', 'disconnect'):
            frame = image_with_jaw(marked_wrist=True)
            if fault == 'disconnect':
                frame[455:520, 905:918] = 225
            else:
                cv2.circle(frame, (789, 487), 10, (38, 38, 38), -1)
                if fault == 'background':
                    cv2.circle(frame, (789, 460), 7, (190, 168, 49), -1)
            with self.assertRaisesRegex(ValueError, 'single_marker_jaw'):
                check_height(frame)

    def test_height_check_requires_registered_view(self):
        with self.assertRaisesRegex(ValueError, 'registered_legacy_view'):
            observe_open_side_height(image_with_jaw(), view_id=None)
