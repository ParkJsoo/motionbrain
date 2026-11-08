"""새 구도 등록 무결성·유한 검색·특징 상실을 합성 원본으로 검증한다."""
import copy
import unittest

import cv2
import numpy as np

from learning.registered_scene_landmarks import RegisteredSceneLandmarks, register, METHOD, LANDMARK_PREFIX, _jaw_pair


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

    def test_explicit_bank_tracks_image_plane_rigid_rotation(self):
        reference = cv2.GaussianBlur(self.image, (0, 0), 1.)
        raw = encoded(reference)
        p = copy.deepcopy(self.p)
        angles = list(range(-34,35,2))
        p['rotationDegrees'] = {name: angles.copy() for name in self.points}
        registration = register(raw, self.points, p, self.context)
        tracker = RegisteredSceneLandmarks(registration, raw)
        # 합성 영상 전체에 동일한 영상 평면 강체 변환을 적용한다. 관절 동작을 뜻하지 않는다.
        matrix = cv2.getRotationMatrix2D((90., 80.), 8., 1.)
        current = cv2.warpAffine(reference, matrix, (180, 120))
        result = tracker.observe(encoded(current), 'rotated-current')
        for name in self.points:
            expected = matrix @ np.array([*self.points[name], 1.])
            np.testing.assert_allclose(result['annotation']['points'][name], expected, atol=.15)
            self.assertEqual(result['quality'][name]['templateRotationDeg'], 8)
            self.assertGreaterEqual(result['quality'][name]['ncc'], p['minimumNcc'][name])
            self.assertGreaterEqual(result['quality'][name]['peakGap'], .04)
        self.assertAlmostEqual(result['state'][2], 100., delta=.15)
        self.assertNotEqual(registration['parametersSha256'], self.reg['parametersSha256'])
        for invalid in ([0, 0], [0, 36], [2], [0, float('nan')]):
            changed = copy.deepcopy(p); changed['rotationDegrees']['jawLeft'] = invalid
            with self.subTest(angles=invalid), self.assertRaisesRegex(ValueError, 'rotation_bank_invalid'):
                register(raw, self.points, changed, self.context)

    def pair_parameters(self):
        p = copy.deepcopy(self.p)
        for name in ('jawLeft', 'jawRight'):
            p['searchRois'][name] = [12, 42, 168, 108]
        p['jawPair'] = dict(minimumSpanPx=95., maximumSpanPx=105.,
                            minimumMeanNccGap=.04, maxCandidatesPerJaw=64)
        return p

    def test_registered_pair_rejects_wrong_span_without_hiding_individual_failure(self):
        p = self.pair_parameters(); reg = register(self.raw, self.points, p, self.context)
        image = self.image.copy()
        image[72:88, 58:74] = self.image[72:88, 132:148]
        raw = encoded(image)
        result = RegisteredSceneLandmarks(reg, self.raw).observe(raw, 'pair-current')
        np.testing.assert_allclose(result['state'], [0., -50., 100.], atol=.05)
        self.assertFalse(result['quality']['jawRight']['individualUniquenessPassed'])
        self.assertLess(result['quality']['jawRight']['peakGap'], .04)
        pair = result['quality']['jawPair']
        self.assertEqual(pair['uniquenessBasis'], 'no_second_eligible_pair')
        self.assertTrue(any(x['reason'] == 'outside_registered_span_band' for x in pair['pairs']))
        self.assertFalse(pair['spanIsPhysicalInvariant'])
        self.assertNotEqual(reg['registrationSha256'], self.reg['registrationSha256'])
        unpaired = copy.deepcopy(p); unpaired.pop('jawPair')
        original = RegisteredSceneLandmarks(register(self.raw, self.points, unpaired, self.context), self.raw)
        with self.assertRaisesRegex(ValueError, 'scene_match_ambiguous'):
            original.observe(raw, 'original-contract-still-rejected')

    def test_registered_pair_ambiguity_occlusion_and_candidate_overflow_fail_closed(self):
        p = self.pair_parameters(); tracker = RegisteredSceneLandmarks(register(self.raw, self.points, p, self.context), self.raw)
        duplicated = self.image.copy()
        duplicated[52:68, 32:48] = self.image[72:88, 32:48]
        duplicated[52:68, 132:148] = self.image[72:88, 132:148]
        with self.assertRaisesRegex(ValueError, 'scene_jaw_pair_ambiguous') as caught:
            tracker.observe(encoded(duplicated), 'two-real-looking-pairs')
        self.assertGreaterEqual(sum(x['eligible'] for x in caught.exception.diagnostics['pairs']), 2)
        hidden = self.image.copy(); hidden[72:88, 32:48] = 127
        with self.assertRaisesRegex(ValueError, 'scene_jaw_candidates_missing'):
            tracker.observe(encoded(hidden), 'occluded')
        p['jawPair']['maxCandidatesPerJaw'] = 1
        capped = RegisteredSceneLandmarks(register(self.raw, self.points, p, self.context), self.raw)
        with self.assertRaisesRegex(ValueError, 'candidate_budget_exceeded') as caught:
            capped.observe(encoded(duplicated), 'overflow-no-truncation')
        self.assertIn('overflow', caught.exception.diagnostics)

    def test_registered_pair_parameters_are_explicit_finite_and_bounded(self):
        for field, value in [('minimumSpanPx', 0), ('maximumSpanPx', float('inf')),
                             ('minimumMeanNccGap', .03), ('maxCandidatesPerJaw', 65),
                             ('maxCandidatesPerJaw', True)]:
            p = self.pair_parameters(); p['jawPair'][field] = value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'jaw_pair_parameters_invalid'):
                register(self.raw, self.points, p, self.context)

    def test_near_band_competitor_is_not_hidden_before_subpixel_localization(self):
        p = self.pair_parameters()
        left = np.zeros((51, 141), np.float32); left[30, 20] = .99
        right = np.zeros_like(left); right[30, 120] = .99; right[30, 113] = .98
        # 정수 span93px는 band95..105 밖이지만 subpixel 허용량으로 안에 들 수 있다.
        matches = {name: (scores, np.zeros_like(scores, dtype=np.uint8), [0])
                   for name, scores in [('jawLeft', left), ('jawRight', right)]}
        with self.assertRaisesRegex(ValueError, 'scene_jaw_pair_ambiguous'):
            _jaw_pair(matches, dict(parameters=p, points=self.points))


if __name__ == '__main__':
    unittest.main()
