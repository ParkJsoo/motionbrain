import unittest

import cv2
import numpy as np

from learning.wide_arm_overlay import _shaft_edges, wide_elbow, wide_upper_region


class WideElbowTest(unittest.TestCase):
    def test_pixel_quantization_does_not_drop_rail_at_upper_span(self):
        shoulder, elbow = np.array([160., 230.]), np.array([160., 80.])
        dark = np.zeros((280, 320), dtype=bool)
        dark[80:231, 122:199] = True
        for base_width in (160, 161):
            with self.subTest(base_width=base_width):
                self.assertEqual(len(_shaft_edges(dark, shoulder, elbow, base_width)), 4)
        dark[80:231, 199] = True
        self.assertEqual(_shaft_edges(dark, shoulder, elbow, 160), [])

    def test_requires_exposed_flange_dark_cap_and_distinct_shaft_edges(self):
        shoulder = np.array([280., 260.])
        elbow = (247, 126)
        rings = (200, 330, 144, 36)
        image = np.full((400, 500, 3), 205, np.uint8)
        cv2.line(image, tuple(shoulder.astype(int)), elbow, (30, 30, 30), 40)
        cv2.circle(image, elbow, 26, (30, 30, 30), -1)
        gold = np.zeros(image.shape[:2], np.uint8)
        cv2.ellipse(gold, elbow, (35, 34), 0, -115, 10, 255, 12)
        image[gold != 0] = (208, 158, 30)
        detected = wide_elbow(image, gold, rings, shoulder)
        self.assertIsNotNone(detected)
        self.assertLess(np.linalg.norm(detected - elbow), 7)
        region = wide_upper_region(image, shoulder, detected, rings[2])
        self.assertTrue(region[193, 264])
        self.assertFalse(region[193, 300])
        self.assertIsNone(wide_elbow(image, np.zeros_like(gold), rings, shoulder))
        image[168:215] = 205
        self.assertIsNone(wide_elbow(image, gold, rings, shoulder))
        image[:] = 35
        self.assertIsNone(wide_elbow(image, gold, rings, shoulder))
