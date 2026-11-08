"""새 구도 등록 무결성·유한 검색·특징 상실을 합성 원본으로 검증한다."""
import copy
import unittest

import cv2
import numpy as np

from learning.registered_scene_landmarks import RegisteredSceneLandmarks, register, METHOD, LANDMARK_PREFIX


def encoded(image):
    ok, raw = cv2.imencode('.png', image)
    assert ok
    return raw.tobytes()


class RegisteredSceneLandmarksTests(unittest.TestCase):
    def setUp(self):
        self.image = np.random.default_rng(32).integers(10, 245, (120, 180, 3), dtype=np.uint8)
        self.raw = encoded(self.image)
        self.points = dict(object=[90, 30], jawLeft=[40, 80], jawRight=[140, 80])
        self.p = dict(imageShape=[120, 180],
            patchRois=dict(object=[82, 22, 98, 38], jawLeft=[32, 72, 48, 88], jawRight=[132, 72, 148, 88]),
            searchRois=dict(object=[62, 12, 118, 48], jawLeft=[12, 52, 68, 108], jawRight=[112, 52, 168, 108]),
            minimumNcc=dict(object=.85, jawLeft=.94, jawRight=.94),
            minimumPeakGap=.04, peakExclusionRadius=2, minimumTemplateStd=1.5)
        self.context = dict(physicalView='new-synthetic-scene', object='test-object', poseRegime='fixed-other-axes')
        self.reg = register(self.raw, self.points, self.p, self.context)
        self.tracker = RegisteredSceneLandmarks(self.reg, self.raw)

    def moved(self, shifts):
        image = self.image.copy()
        for name, (dx, dy) in shifts.items():
            x0, y0, x1, y1 = self.p['patchRois'][name]
            image[y0:y1, x0:x1] = 0
            image[y0+dy:y1+dy, x0+dx:x1+dx] = self.image[y0:y1, x0:x1]
        return encoded(image)

    def test_independent_translation_and_explicit_new_contract(self):
        shifts = dict(object=[3, 2], jawLeft=[-7, -4], jawRight=[6, -3])
        result = self.tracker.observe(self.moved(shifts), 'current-only')
        expected = {n: np.asarray(self.points[n])+shifts[n] for n in self.points}
        for name, point in expected.items():
            np.testing.assert_allclose(result['annotation']['points'][name], point, atol=.05)
        state = [*(expected['object']-(expected['jawLeft']+expected['jawRight'])/2),
                 np.linalg.norm(expected['jawRight']-expected['jawLeft'])]
        np.testing.assert_allclose(result['state'], state, atol=.05)
        self.assertEqual(result['annotation']['method'], METHOD)
        self.assertTrue(result['annotation']['landmarkContract'].startswith(LANDMARK_PREFIX))
        for key in ('trainingEligible', 'controlEligible', 'measurementAccuracyVerified', 'depthVerified', 'contactVerified'):
            self.assertFalse(result[key])

    def test_reference_parameters_and_scene_are_immutable(self):
        for key, value in [('parametersSha256', '0'*64), ('referencePixelSha256', '0'*64),
                           ('referenceImageSha256', '0'*64), ('sceneContext', dict(self.context, object='other'))]:
            reg = copy.deepcopy(self.reg); reg[key] = value
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'contract_changed'):
                RegisteredSceneLandmarks(reg, self.raw)
        altered = self.image.copy(); altered[0, 0] = 0
        with self.assertRaisesRegex(ValueError, 'contract_changed'):
            RegisteredSceneLandmarks(self.reg, encoded(altered))

    def test_invalid_roi_seeds_threshold_and_texture_are_rejected(self):
        mutations = [lambda p: p['searchRois']['object'].__setitem__(0, -1),
                     lambda p: p['searchRois'].__setitem__('object', p['patchRois']['object']),
                     lambda p: p['minimumNcc'].__setitem__('jawLeft', .90),
                     lambda p: p.__setitem__('minimumPeakGap', .01)]
        for change in mutations:
            p = copy.deepcopy(self.p); change(p)
            with self.subTest(p=p), self.assertRaises(ValueError):
                register(self.raw, self.points, p, self.context)
        with self.assertRaisesRegex(ValueError, 'seed_outside_patch'):
            register(self.raw, dict(self.points, object=[0, 0]), self.p, self.context)
        with self.assertRaisesRegex(ValueError, 'texture_insufficient'):
            register(encoded(np.zeros_like(self.image)), self.points, self.p, self.context)

    def test_duplicate_patch_occlusion_and_color_replacement_are_rejected(self):
        # Avoid the search boundary so the second identical location reaches the uniqueness guard.
        duplicated = self.image.copy(); duplicated[22:38, 100:116] = self.image[22:38, 82:98]
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            self.tracker.observe(encoded(duplicated), 'duplicate')
        for change in ('occluded', 'color'):
            image = self.image.copy()
            image[72:88, 32:48] = (127 if change == 'occluded'
                                  else self.image[72:88, 32:48, ::-1])
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.tracker.observe(encoded(image), change)

    def test_search_edge_and_wrong_image_shape_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'search_boundary'):
            self.tracker.observe(self.moved(dict(jawLeft=[-20, 0])), 'edge')
        with self.assertRaisesRegex(ValueError, 'shape_changed'):
            self.tracker.observe(encoded(self.image[:100]), 'wrong-shape')


if __name__ == '__main__':
    unittest.main()
