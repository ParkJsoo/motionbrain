import unittest
from pathlib import Path

import cv2
import numpy as np

from learning import held_object_observation as held
from learning.coarse_external_observer import _object
from learning.near_gripper_tip_observer import NearGripperTipObserver, validate_observation


def encoded(image):
    return cv2.imencode('.png', image)[1].tobytes()


class HeldObservationTests(unittest.TestCase):
    def test_rotated_object_and_ambiguity(self):
        image = np.full((1080, 1920, 3), 180, np.uint8)
        box = cv2.boxPoints(((750., 650.), (48., 145.), 25.)).astype(int)
        cv2.fillConvexPoly(image, box, (100, 20, 240))
        result = held.observe(encoded(image))
        self.assertFalse(result['supportVerified'])
        self.assertFalse(result['existingModelInputAdmissionVerified'])
        self.assertAlmostEqual(result['object']['center'][0], 750., delta=1.)
        cv2.fillConvexPoly(image, box+[250, 0], (100, 20, 240))
        with self.assertRaisesRegex(ValueError, 'missing_or_ambiguous'):
            held.observe(encoded(image))

    def test_v4_only_ignores_small_outer_background_below_finger(self):
        pixels = np.full((240, 320, 3), 180, np.uint8)
        for x in (100, 210):
            cv2.rectangle(pixels, (x, 20), (x+15, 140), (30, 30, 30), -1)
        cv2.rectangle(pixels, (140, 90), (175, 125), (100, 20, 240), -1)
        for kind, rect, accepted in [
            ('outer_small', (20, 147, 34, 161), True),
            ('inner_small', (145, 147, 159, 161), False),
            ('outer_large', (20, 140, 34, 169), False),
            ('above_bottom', (20, 100, 34, 114), False),
            ('interior', (40, 147, 54, 161), False),
        ]:
            with self.subTest(kind=kind):
                image = pixels.copy()
                cv2.rectangle(image, rect[:2], rect[2:], (30, 30, 30), -1)
                with self.assertRaisesRegex(ValueError, 'near_tip_missing_or_ambiguous'):
                    NearGripperTipObserver(policy_version=3).observe(encoded(image))
                observer = NearGripperTipObserver(policy_version=4)
                if accepted:
                    observation = observer.observe(encoded(image))
                    validate_observation(observation)
                    self.assertEqual(len(observation['components']['left']['ignoredBoundaryBackground']), 1)
                    self.assertNotEqual(observer.binding, NearGripperTipObserver(policy_version=3).binding)
                else:
                    with self.assertRaisesRegex(ValueError, 'near_tip_missing_or_ambiguous'):
                        observer.observe(encoded(image))

    def test_actual_held_frames_preserve_original_rejections(self):
        root = Path(__file__).resolve().parents[1]/'.local/current-entry-20261117/lift-post-command-hold-readonly-01'
        if not root.exists():
            self.skipTest('private held frames unavailable')
        for index in (1, 2):
            external = (root/f'current-external-{index}.png').read_bytes()
            with self.assertRaisesRegex(ValueError, 'coarse_object_missing_or_ambiguous'):
                _object(cv2.imdecode(np.frombuffer(external, np.uint8), cv2.IMREAD_COLOR), held.policy()['objectRoi'])
            self.assertGreater(held.observe(external)['object']['area'], 7000)
            raw = (root/f'current-gripper-{index}.jpg').read_bytes()
            validate_observation(NearGripperTipObserver(policy_version=4).observe(raw))
        with self.assertRaisesRegex(ValueError, 'near_tip_missing_or_ambiguous'):
            NearGripperTipObserver(policy_version=3).observe(raw)
