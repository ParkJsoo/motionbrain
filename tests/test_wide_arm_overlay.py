import unittest

import cv2
import numpy as np

from learning.wide_arm_overlay import wide_elbow, wide_upper_region


class WideElbowTest(unittest.TestCase):
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
