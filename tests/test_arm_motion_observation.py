"""투영 회전의 부호·배경 이동·추적 소실을 합성 영상으로 검증한다."""
import copy
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.arm_motion_observation import observe_pair


class ArmMotionObservationTest(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(221)
        self.background = cv2.GaussianBlur(rng.integers(0, 256, (480, 640), dtype=np.uint8), (3, 3), 0)
        self.texture = cv2.GaussianBlur(rng.integers(0, 256, (480, 640), dtype=np.uint8), (3, 3), 0)
        self.object_mask = np.zeros((480, 640), np.uint8)
        self.object_mask[170:350, 240:430] = 255
        self.moving = [[255, 185], [415, 185], [415, 335], [255, 335]]
        self.backgrounds = [[[30, 30], [180, 30], [180, 140], [30, 140]],
                            [[460, 330], [610, 330], [610, 450], [460, 450]]]

    def frames(self, raw_rotation=2., scale=1.):
        before = self.background.copy()
        before[self.object_mask != 0] = self.texture[self.object_mask != 0]
        matrix = cv2.getRotationMatrix2D((335, 260), -raw_rotation, scale)
        moved_mask = cv2.warpAffine(self.object_mask, matrix, (640, 480), flags=cv2.INTER_NEAREST)
        moved = cv2.warpAffine(self.texture, matrix, (640, 480))
        after = self.background.copy()
        after[moved_mask != 0] = moved[moved_mask != 0]
        return before, after

    def observe(self, before, after, sign=-1, **kwargs):
        return observe_pair(before, after, self.moving, self.backgrounds, sign, **kwargs)

    def test_known_rigid_rotation_both_forward_signs(self):
        for rotation in (-3., 0., 2.5):
            before, after = self.frames(rotation)
            for sign in (-1, 1):
                with self.subTest(rotation=rotation, sign=sign):
                    result = self.observe(before, after, sign)
                    self.assertTrue(result["valid"], result)
                    self.assertAlmostEqual(result["projectedDeltaDeg"], -sign * rotation, delta=.12)
                    self.assertGreater(result["moving"]["fit"]["inliers"], 20)
                    self.assertEqual(len(result["background"]["fit"]["inliersPerRegion"]), 2)
                    self.assertFalse(result["controlEligible"])
                    self.assertFalse(result["physicallyCalibrated"])

    def test_scale_change_is_rejected_without_angle_candidate(self):
        result = self.observe(*self.frames(2, 1.08))
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "moving_scale_changed")
        self.assertIsNone(result["projectedDeltaDeg"])

    def test_whole_camera_translation_or_rotation_is_rejected(self):
        before, after = self.frames()
        for matrix in [np.array([[1., 0, 6], [0, 1., 0]]),
                       cv2.getRotationMatrix2D((320, 240), -1.5, 1)]:
            with self.subTest(matrix=matrix):
                observed = cv2.warpAffine(after, matrix, (640, 480))
                result = self.observe(before, observed)
                self.assertFalse(result["valid"])
                self.assertIn(result["reason"], ["background_translated", "background_rotated"])
                self.assertIsNone(result["projectedDeltaDeg"])

    def test_textureless_baseline_cannot_acquire_features_from_after(self):
        _, after = self.frames()
        before = np.full_like(after, 127)
        with patch("learning.arm_motion_observation.cv2.goodFeaturesToTrack", wraps=cv2.goodFeaturesToTrack) as detector:
            result = self.observe(before, after)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "moving:insufficient_baseline_features")
        self.assertEqual(result["moving"]["tracking"]["seeded"], 0)
        self.assertEqual(detector.call_count, 1)
        self.assertIsNone(result["projectedDeltaDeg"])

    def test_tracking_dropout_is_preserved_without_reacquisition(self):
        before, after = self.frames()
        after[155:365, 225:445] = 128
        with patch("learning.arm_motion_observation.cv2.goodFeaturesToTrack", wraps=cv2.goodFeaturesToTrack) as detector:
            result = self.observe(before, after)
        self.assertFalse(result["valid"])
        self.assertEqual(detector.call_count, 1)
        self.assertGreaterEqual(result["moving"]["tracking"]["seeded"], 12)
        self.assertIsNone(result["projectedDeltaDeg"])

    def test_one_background_region_dropout_cannot_be_hidden_by_the_other(self):
        before, after = self.frames()
        after[20:150, 20:190] = 128
        result = self.observe(before, after)
        self.assertFalse(result["valid"])
        self.assertTrue(result["reason"].startswith("background:"), result)
        self.assertIsNone(result["projectedDeltaDeg"])

    def test_baseline_features_are_seeded_once_per_declared_region(self):
        before, after = self.frames()
        with patch("learning.arm_motion_observation.cv2.goodFeaturesToTrack", wraps=cv2.goodFeaturesToTrack) as detector:
            result = self.observe(before, after)
        self.assertTrue(result["valid"], result)
        self.assertEqual(detector.call_count, 3)
        self.assertTrue(all(np.array_equal(call.args[0], before) for call in detector.call_args_list))
        self.assertEqual(result["featurePolicy"], "baseline_only_no_reacquisition")

    def test_many_collinear_features_do_not_establish_rotation(self):
        before, _ = self.frames()
        before[170:350, 240:430] = 128
        before[185:335, 332:336] = np.random.default_rng(51).integers(0, 256, (150, 4), dtype=np.uint8)
        result = self.observe(before, before.copy())
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "moving:insufficient_spatial_spread")
        self.assertGreaterEqual(result["moving"]["fit"]["inliers"], 12)
        self.assertIsNone(result["projectedDeltaDeg"])

    def test_bad_input_and_polygons_are_rejected(self):
        before, after = self.frames()
        cases = [
            dict(forward_image_sign=True),
            dict(moving_polygon=[[0, 0], [1, 0], [1, 1]]),
            dict(moving_polygon=[[0, 0], [640, 0], [640, 100], [0, 100]]),
            dict(moving_polygon=[[0, 0], [100, 100], [0, 100], [100, 0]]),
            dict(moving_polygon=[[True, 100], [120, 100], [120, 120], [100, 120]]),
            dict(moving_polygon=[[float("nan"), 100], [120, 100], [120, 120], [100, 120]]),
            dict(background_polygons=[self.backgrounds[0]]),
            dict(background_polygons=[self.backgrounds[0], self.backgrounds[0]]),
            dict(parameters={"max_features": True}),
            dict(parameters={"max_fb_error_px": float("inf")}),
            dict(parameters={"unknown": 1}),
        ]
        for index, changes in enumerate(cases):
            with self.subTest(case=index):
                kwargs = dict(moving_polygon=self.moving, background_polygons=self.backgrounds,
                              forward_image_sign=-1)
                kwargs.update(copy.deepcopy(changes))
                with self.assertRaises(ValueError):
                    observe_pair(before, after, **kwargs)
        for invalid in (before.astype(float), before.astype(bool), before[:20]):
            with self.assertRaises(ValueError):
                self.observe(invalid, after)


if __name__ == "__main__":
    unittest.main()
