from pathlib import Path
import copy
import unittest

import cv2
import numpy as np

from learning.marker_virtual_observer import register_virtual, VirtualMarkerObserver, MEANING


class VirtualMarkerTests(unittest.TestCase):
    def encode(self, image):
        ok, data = cv2.imencode('.png', image); self.assertTrue(ok); return data.tobytes()

    def setUp(self):
        self.image = np.full((1080, 1920, 3), 180, np.uint8)
        cv2.rectangle(self.image, (915, 340), (1160, 420), (30, 30, 30), -1)
        self.draw_pair(self.image, (1083, 386), (1115, 381))
        cv2.rectangle(self.image, (789, 627), (844, 795), (100, 20, 240), -1)
        self.raw = self.encode(self.image)
        self.registration = register_virtual(self.raw, virtual_point=[938, 375])
        self.observer = VirtualMarkerObserver(self.registration, self.raw)

    def draw_pair(self, image, purple, teal):
        cv2.ellipse(image, purple, (11, 8), 0, 0, 360, (110, 65, 110), -1)
        cv2.ellipse(image, teal, (9, 10), 0, 0, 360, (80, 110, 65), -1)

    def test_direct_pair_basis_is_virtual_and_has_no_actual_tip_claim(self):
        result = self.observer.observe(self.raw)
        np.testing.assert_allclose(result['virtualProxyPx'], [938, 375], atol=.001)
        self.assertEqual(result['state'][2], 1.)
        self.assertEqual(result['stateMeaning'], MEANING)
        self.assertEqual(result['quality']['eligiblePairs'], 1)
        for field in ['actualFingertipMeasured', 'occludedTipReconstructed', 'contactVerified',
                      'graspVerified', 'depthVerified', 'clearanceVerified']:
            self.assertFalse(result[field])

    def test_translated_rotated_pair_does_not_require_new_template(self):
        moving = self.image.copy(); moving[627:796, 789:845] = 180
        affine = cv2.getRotationMatrix2D((1083, 386), 30, 1.)
        affine[:, 2] += [-50, 200]
        image = cv2.warpAffine(moving, affine, (1920, 1080), borderValue=(180, 180, 180))
        cv2.rectangle(image, (789, 627), (844, 795), (100, 20, 240), -1)
        result = self.observer.observe(self.encode(image))
        expected = cv2.transform(np.float32([[[938, 375]]]), affine)[0, 0]
        self.assertLess(float(np.linalg.norm(np.array(result['virtualProxyPx'])-expected)), 3.)
        self.assertEqual(result['registrationSha256'], self.observer.binding)

    def test_missing_or_duplicate_pair_rejected(self):
        for kind in ['purple', 'teal', 'duplicate']:
            image = self.image.copy()
            if kind == 'purple': image[370:400, 1065:1100] = 30
            elif kind == 'teal': image[365:400, 1100:1130] = 30
            else: self.draw_pair(image, (650, 200), (682, 195))
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, 'pair_missing_or_ambiguous'):
                self.observer.observe(self.encode(image))

    def test_object_occlusion_duplicate_and_local_dark_support_missing_rejected(self):
        for kind in ['object', 'duplicate', 'dark_support']:
            image = self.image.copy()
            if kind == 'object': image[680:800, 780:855] = 180
            elif kind == 'duplicate': cv2.rectangle(image, (650, 627), (705, 795), (100, 20, 240), -1)
            else: image[350:401, 910:965] = 180
            with self.subTest(kind=kind), self.assertRaises(ValueError): self.observer.observe(self.encode(image))

    def test_mutated_seed_policy_or_registration_rejected(self):
        with self.assertRaisesRegex(ValueError, 'seed_changed'):
            VirtualMarkerObserver(self.registration, self.raw+b' ')
        changed = copy.deepcopy(self.registration); changed['detectorPolicy']['minimumSolidity'] = .1
        with self.assertRaisesRegex(ValueError, 'registration_contract'): VirtualMarkerObserver(changed, self.raw)
        self.observer.registration['basisCoefficients'][0] += 1.
        with self.assertRaisesRegex(ValueError, 'registration_changed'): self.observer.observe(self.raw)

    def test_real_four_states_keep_unique_pair_without_template_extension(self):
        root = Path(__file__).resolve().parents[1]/'.local/current-entry-20261114'
        paths = [root/'L-pose-current-20261115-05-after-failed-M3/external.png',
                 root/'L-pose-M3-down70-1000-direct-01/external-after.png',
                 root/'L-pose-M3-down70-1000-direct-03/external-after.png',
                 root/'L-pose-current-after-M4-20261115-01/external.png']
        if not all(path.exists() for path in paths): self.skipTest('private current frames unavailable')
        seed = paths[0].read_bytes(); observer = VirtualMarkerObserver(register_virtual(seed, virtual_point=[938, 375]), seed)
        results = [observer.observe(path.read_bytes()) for path in paths]
        self.assertTrue(all(row['quality']['eligiblePairs'] == 1 for row in results))
        self.assertTrue(all(row['registrationSha256'] == observer.binding for row in results))
        np.testing.assert_allclose(results[-1]['virtualProxyPx'], [1004.384, 748.185], atol=1.)
        self.assertLess(results[-1]['toolAxisImageAngleDeg'], -40.)


if __name__ == '__main__': unittest.main()
