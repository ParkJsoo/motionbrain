import unittest

import numpy as np

from learning.near_part_overlay import COLORS, render_near


def scene():
    rgb = np.full((240, 320, 3), 180, np.uint8)
    rgb[20:120, 80:100] = 30
    rgb[20:120, 218:236] = 30
    rgb[76:100, 150:173] = [238, 22, 103]
    return rgb


class NearPartOverlayTest(unittest.TestCase):
    def test_wide_approach_is_opt_in_and_does_not_set_close_geometry(self):
        rgb = scene()
        rgb[76:100, 150:173] = 180
        rgb[40:62, 149:167] = [238, 22, 103]
        _, old = render_near(rgb)
        self.assertNotIn('farTarget', old)
        painted, wide = render_near(rgb, wide_approach=True)
        obs = wide['farTarget']
        self.assertEqual(obs['bbox'], [149, 40, 18, 22])
        self.assertEqual(obs['status'], 'observed')
        self.assertFalse(obs['controlEligible'])
        self.assertFalse(wide['parts']['target']['visible'])
        self.assertIsNone(wide['projectedMargins'])
        np.testing.assert_array_equal(painted[40, 149], COLORS['target'])

    def test_wide_approach_omits_fragment_ambiguity_and_missing_jaw(self):
        rgb = scene()
        rgb[76:100, 150:173] = 180
        rgb[36:43, 149:167] = [238, 22, 103]
        for condition in ['fragment', 'multiple', 'missing_jaw', 'off_center', 'closed_scene']:
            with self.subTest(condition=condition):
                sample = rgb.copy()
                expected = 'unresolved'
                if condition == 'fragment':
                    expected = 'uncertain_fragment'
                elif condition == 'multiple':
                    sample[40:57, 125:140] = [238, 22, 103]
                elif condition == 'missing_jaw':
                    sample[:, 218:236] = 180
                elif condition == 'off_center':
                    sample[36:43, 149:167] = 180
                    sample[41:56, 52:70] = [238, 22, 103]
                else:
                    sample[76:100, 150:173] = [238, 22, 103]
                _, info = render_near(sample, wide_approach=True)
                self.assertEqual(info['farTarget'], dict(status=expected, controlEligible=False))

    def test_pixel_masks_holes_and_same_row_edges(self):
        src = scene()
        src[65:75, 86:92] = 180
        original = src.copy()
        painted, info = render_near(src)
        np.testing.assert_array_equal(src, original)
        np.testing.assert_array_equal(painted[70, 89], original[70, 89])
        np.testing.assert_array_equal(painted[50, 80], COLORS['leftJaw'])
        self.assertTrue(np.all(painted[51, 85] > original[51, 85]))
        self.assertEqual(info['parts']['rightJaw']['pixels'], 18*85)
        segment = info['projectedMargins']
        self.assertEqual(segment['leftEdge'], [99, 150])
        self.assertEqual(segment['rightEdge'], [172, 218])
        self.assertEqual([segment['leftPx'], segment['rightPx']], [51, 46])

    def test_missing_ambiguous_and_non_overlapping(self):
        pixels = scene()
        pixels[75:105, 80:100] = 180
        _, info = render_near(pixels)
        self.assertFalse(info['parts']['leftJaw']['visible'])
        self.assertTrue(info['parts']['rightJaw']['visible'])
        self.assertTrue(info['parts']['target']['visible'])
        self.assertIsNone(info['projectedMargins'])
        other = scene()
        other[76:100, 150:173] = 180
        other[142:166, 150:173] = [238, 22, 103]
        _, info = render_near(other)
        self.assertTrue(all(p['visible'] for p in info['parts'].values()))
        self.assertIsNone(info['projectedMargins'])

    def test_fragment_gap_stays_empty_and_ambiguous_target_is_omitted(self):
        src = scene()
        src[105:113, 154:166] = [238, 22, 103]
        painted, info = render_near(src)
        self.assertEqual(info['parts']['target']['fragmentCount'], 2)
        np.testing.assert_array_equal(painted[103, 159], src[103, 159])
        src[130:152, 70:100] = [238, 22, 103]
        painted, info = render_near(src)
        self.assertFalse(info['parts']['target']['visible'])
        np.testing.assert_array_equal(painted[90, 155], src[90, 155])
        self.assertIsNone(info['projectedMargins'])

    def test_pair_consistency_and_image_contract(self):
        src = scene()
        src[89:120, 218:236] = 180
        painted, info = render_near(src)
        self.assertFalse(info['parts']['leftJaw']['visible'])
        self.assertTrue(info['parts']['target']['visible'])
        np.testing.assert_array_equal(painted[50, 80], src[50, 80])
        with self.assertRaisesRegex(ValueError, 'qvga_rgb'):
            render_near(src[:, ::2])
