import unittest

import cv2
import numpy as np

from learning.open_side_approach import LEGACY_SIDE_VIEW, observe_open_side, observe_side_tip


def scene(object_x=849, object_y=662, object_height=206):
    frame = np.full((1080, 1920, 3), 238, np.uint8)
    for center in ((1154, 353), (1218, 322)):
        cv2.circle(frame, center, 15, (85, 150, 97), -1)
    frame[460:492, 762:1030] = 38
    frame[object_y:object_y+object_height, object_x:object_x+60] = (235, 23, 106)
    return frame


class OpenSideApproachTest(unittest.TestCase):
    def test_unknown_view_abstains_even_when_legacy_scene_is_visible(self):
        frame = scene()
        self.assertEqual(observe_side_tip(frame), observe_side_tip(frame, view_id=LEGACY_SIDE_VIEW))
        with self.assertRaisesRegex(ValueError, 'unregistered_side_view'):
            observe_side_tip(frame, view_id='typo')

    def test_target_moves_but_tool_does_not(self):
        for x, y, height in [(350, 635, 177), (849, 662, 206), (1150, 540, 196)]:
            result = observe_open_side(scene(x, y, height))
            self.assertEqual(result['toolPoint'], [762, 475.5])
            self.assertEqual(result['box'], [x, y, 60, height])
            self.assertEqual(result['sideOffsetPx'], [762-x-30, 475.5-y])

    def test_ambiguous_forearm_or_tool_does_not_produce_a_point(self):
        frame = scene()
        cv2.circle(frame, (1282, 291), 15, (85, 150, 97), -1)
        with self.assertRaisesRegex(ValueError, 'forearm_pair'):
            observe_open_side(frame)
        frame = scene()
        frame[526:549, 774:1035] = 35
        with self.assertRaisesRegex(ValueError, 'jaw_contour'):
            observe_open_side(frame)

    def test_single_forearm_mark_needs_wrist_and_connected_jaw(self):
        for hidden in ((1154, 353), (1218, 322)):
            frame = scene()
            cv2.circle(frame, hidden, 20, (238, 238, 238), -1)
            with self.assertRaisesRegex(ValueError, 'single_marker_jaw'):
                observe_open_side(frame)
            cv2.ellipse(frame, (961, 475), (15, 8), 0, 0, 360, (141, 87, 121), -1)
            result = observe_open_side(frame)
            self.assertEqual(result['trackingMode'], 'single_marker_wrist_contour')
            self.assertEqual(result['toolPoint'], [762, 475.5])
            frame[452:495, 929:941] = 238
            with self.assertRaisesRegex(ValueError, 'single_marker_jaw'):
                observe_open_side(frame)

    def test_unrelated_pink_object_or_disconnected_wrist_cannot_stand_in_for_jaw(self):
        frame = scene()
        cv2.circle(frame, (1218, 322), 20, (238, 238, 238), -1)
        cv2.ellipse(frame, (961, 503), (15, 8), 0, 0, 360, (141, 87, 121), -1)
        with self.assertRaisesRegex(ValueError, 'single_marker_jaw'):
            observe_open_side(frame)

    def test_missing_target_and_wrong_image_contract(self):
        frame = scene()
        frame[662:870] = 238
        with self.assertRaisesRegex(ValueError, 'upright_prism'):
            observe_open_side(frame)
        with self.assertRaisesRegex(ValueError, '1080p_rgb'):
            observe_open_side(frame[::2, ::2])

    def test_clipped_competitor_cannot_disambiguate_a_visible_contour(self):
        frame = scene()
        frame[337:393, 740:1030] = 38
        with self.assertRaisesRegex(ValueError, 'tip_clipped'):
            observe_open_side(frame)
        frame = scene()
        frame[530:552, 620:1030] = 38
        with self.assertRaisesRegex(ValueError, 'tip_clipped'):
            observe_open_side(frame)

    def test_truncated_marker_cannot_locate_search_window(self):
        frame = scene()
        frame[300:380] = 238
        cv2.circle(frame, (1845, 353), 15, (85, 150, 97), -1)
        cv2.circle(frame, (1919, 322), 25, (85, 150, 97), -1)
        with self.assertRaisesRegex(ValueError, 'marker_clipped'):
            observe_open_side(frame)

    def test_wide_wrist_markers_require_connected_unclipped_distal_jaw(self):
        from learning.open_side_approach import WIDE_SIDE_VIEW, observe_side_tip
        rgb = np.full((1080, 1920, 3), 220, np.uint8)
        rgb[469:500, 782:1010] = 48
        cv2.ellipse(rgb, (940, 480), (13, 8), 0, 0, 360, (141, 87, 121), -1)
        cv2.ellipse(rgb, (968, 480), (10, 9), 0, 0, 360, (85, 150, 97), -1)
        tip, mode = observe_side_tip(rgb, view_id=WIDE_SIDE_VIEW)
        self.assertEqual((tip, mode), ((782, 484), 'wide_wrist_jaw_contour'))
        with self.assertRaises(ValueError):
            observe_side_tip(rgb)
        # Rotation changes the physical digit that forms the leftmost corner.
        rotated = cv2.warpAffine(rgb, cv2.getRotationMatrix2D((940, 480), 80, 1),
                                 (1920, 1080), borderValue=(220, 220, 220))
        with self.assertRaisesRegex(ValueError, 'wide_wrist_jaw'):
            observe_side_tip(rotated, view_id=WIDE_SIDE_VIEW)
        clipped = np.roll(rgb, -785, axis=1)
        with self.assertRaisesRegex(ValueError, 'wide_wrist_jaw'):
            observe_side_tip(clipped, view_id=WIDE_SIDE_VIEW)
        rgb[450:510, 904:910] = 220
        with self.assertRaisesRegex(ValueError, 'wide_wrist_jaw'):
            observe_side_tip(rgb, view_id=WIDE_SIDE_VIEW)
