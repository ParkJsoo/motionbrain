import unittest

import cv2
import numpy as np

from learning.wide_forearm_overlay import wide_distal_parts


class WideDistalOverlayTest(unittest.TestCase):
    def frame(self):
        rgb = np.full((540, 960, 3), 225, np.uint8)
        cv2.line(rgb, (715, 265), (535, 265), (45,45,45), 42)
        cv2.rectangle(rgb, (484, 249), (540, 283), (50,50,50), -1)
        cv2.rectangle(rgb, (425, 254), (490, 267), (43,43,43), -1)
        for x in (605, 633):
            cv2.circle(rgb, (x, 265), 8, (74,115,89), -1)
        cv2.ellipse(rgb, (510, 266), (6,4), 0, 0, 360, (135,80,109), -1)
        return rgb

    def test_connected_parts_do_not_require_an_identifiable_jaw_tip(self):
        image = self.frame()
        cv2.rectangle(image, (408, 252), (419, 267), (40,40,40), -1)
        cv2.rectangle(image, (435, 268), (455, 312), (126,12,58), -1)
        masks, landmarks = wide_distal_parts(image, np.array([715.,265.]))
        self.assertEqual(set(masks), {'forearm','wrist_body','visible_jaws'})
        self.assertIn('wristSurfaceMarker', landmarks)
        self.assertNotIn('visibleJawTip', landmarks)
        self.assertTrue(masks['forearm'][265, 635])
        self.assertTrue(masks['wrist_body'][271, 514])
        self.assertTrue(masks['visible_jaws'][261, 448])
        self.assertFalse(masks['visible_jaws'][261, 414])
        self.assertFalse(masks['visible_jaws'][269, 442])
        self.assertFalse(masks['forearm'][265, 713])

    def test_ambiguous_or_missing_evidence_omits_masks(self):
        image = self.frame()
        masks, _ = wide_distal_parts(image, None)
        self.assertNotIn('forearm', masks)
        cv2.circle(image, (633, 265), 13, (45,45,45), -1)
        self.assertEqual(wide_distal_parts(image, None), ({}, {}))

    def test_one_green_with_measured_tip_preserves_wrist_and_jaws(self):
        image = self.frame()
        cv2.circle(image, (633, 265), 13, (45,45,45), -1)
        masks, landmarks = wide_distal_parts(image, np.array([715.,265.]), (850, 520))
        self.assertEqual(set(masks), {'wrist_body','visible_jaws'})
        self.assertNotIn('visibleJawTip', landmarks)
        cv2.circle(image, (601, 292), 8, (74,115,89), -1)
        self.assertEqual(wide_distal_parts(image, None, (850,520)), ({},{}))

    def test_missing_distal_blade_is_omitted(self):
        image = self.frame()
        image[:, :482] = 225
        masks, _ = wide_distal_parts(image, np.array([715.,265.]))
        self.assertNotIn('visible_jaws', masks)
        self.assertIn('wrist_body', masks)
