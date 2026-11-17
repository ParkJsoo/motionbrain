import copy
import unittest

import cv2
import numpy as np

from learning.gripper_recovery_observer import GripperRecoveryObserver, validate_observation, STATE_FIELDS
from learning.near_gripper_tip_observer import NearGripperTipObserver


def image(separation=0, *, pink=False):
    pixels = np.full((240, 320, 3), 180, np.uint8)
    if separation == 0:
        cv2.rectangle(pixels, (145, 25), (175, 145), (25, 25, 25), -1)
    else:
        for x in [160-separation//2, 160+separation//2]:
            cv2.rectangle(pixels, (x-8, 25), (x+8, 125), (25, 25, 25), -1)
    if pink:
        cv2.rectangle(pixels, (141, 155), (179, 190), (100, 20, 240), -1)
    return pixels


def encoded(pixels):
    ok, raw = cv2.imencode('.png', pixels)
    if not ok:
        raise RuntimeError('encoding failed')
    return raw.tobytes()


class RecoveryObserverTests(unittest.TestCase):
    def test_merged_empty_jaws_are_valid_without_fabricated_gap_or_object_coordinates(self):
        raw = encoded(image())
        with self.assertRaises(ValueError):
            NearGripperTipObserver(policy_version=3).observe(raw)
        result = GripperRecoveryObserver().observe(raw)
        self.assertEqual(len(validate_observation(result)), 3)
        self.assertEqual(result['state'][2], 0.)
        self.assertGreater(result['state'][0], 0.)
        self.assertFalse(result['actualGapMeasured']); self.assertFalse(result['OBS3Compatible'])
        self.assertNotIn('gapPx', result); self.assertNotIn('points', result); self.assertNotIn('object', result)

    def test_partial_and_open_silhouettes_keep_same_contract_and_distinct_image_widths(self):
        observer = GripperRecoveryObserver()
        observations = [observer.observe(encoded(image(v))) for v in (0, 60, 130)]
        widths = [o['state'][0] for o in observations]
        self.assertLess(widths[0], widths[1]); self.assertLess(widths[1], widths[2])
        for observation in observations:
            validate_observation(observation)
            self.assertEqual(observation['observationContractSha256'], observer.binding)
            self.assertFalse(observation['depthVerified'])

    def test_pink_appearance_changes_color_presence_without_changing_jaw_measurement(self):
        observer = GripperRecoveryObserver()
        absent = observer.observe(encoded(image(80)))
        present = observer.observe(encoded(image(80, pink=True)))
        self.assertEqual(absent['state'][:2], present['state'][:2])
        self.assertEqual(absent['state'][2], 0.); self.assertEqual(present['state'][2], 1.)
        self.assertFalse(present['pink']['objectIdentityVerified'])
        validate_observation(present)

    def test_multiple_color_components_are_explicit_without_inventing_single_object_identity(self):
        pixels = image(80, pink=True)
        cv2.rectangle(pixels, (65, 170), (95, 210), (100, 20, 240), -1)
        result = GripperRecoveryObserver().observe(encoded(pixels))
        self.assertEqual(result['pink']['qualifiedComponentCount'], 2)
        self.assertTrue(result['pink']['multipleQualifiedComponents'])
        self.assertFalse(result['pink']['objectIdentityVerified'])
        validate_observation(result)

    def test_blank_underexposed_wrong_shape_and_invalid_payload_are_rejected(self):
        for raw in [b'not an image', b'', encoded(np.full((240, 320, 3), 180, np.uint8)),
                    encoded(np.zeros((240, 320, 3), np.uint8)), encoded(np.full((100, 100, 3), 180, np.uint8))]:
            with self.subTest(length=len(raw)), self.assertRaises(ValueError):
                GripperRecoveryObserver().observe(raw)

    def test_policy_and_derived_state_cannot_be_silently_changed(self):
        observer = GripperRecoveryObserver(); result = observer.observe(encoded(image(80, pink=True)))
        mutations = [lambda o: o['state'].__setitem__(0, .9),
                     lambda o: o['jawImage'].__setitem__('central80WidthFraction', .9),
                     lambda o: o['pink'].__setitem__('qualifiedPresent', False),
                     lambda o: o.__setitem__('actualGapMeasured', True),
                     lambda o: o.__setitem__('observationContractSha256', '0'*64),
                     lambda o: o.__setitem__('stateFields', ['fake-gap']*3)]
        for mutate in mutations:
            changed = copy.deepcopy(result); mutate(changed)
            with self.assertRaises(ValueError):
                validate_observation(changed)
        observer.policy['darkRoi'][0] += 1
        with self.assertRaisesRegex(ValueError, 'policy_changed'):
            observer.observe(encoded(image()))
        self.assertEqual(result['stateFields'], STATE_FIELDS)

    def test_nonqualified_color_noise_is_not_reported_as_recovered_object(self):
        pixels = image()
        cv2.rectangle(pixels, (145, 170), (149, 174), (100, 20, 240), -1)
        result = GripperRecoveryObserver().observe(encoded(pixels))
        self.assertGreater(result['pink']['maskPixelCount'], 0)
        self.assertEqual(result['pink']['qualifiedComponentCount'], 0)
        self.assertEqual(result['state'][2], 0.)
        self.assertNotIn('recoverySucceeded', result)
        validate_observation(result)

    def test_background_edge_below_fixed_jaw_roi_does_not_change_jaw_state(self):
        observer = GripperRecoveryObserver(); pixels = image()
        before = observer.observe(encoded(pixels))
        cv2.rectangle(pixels, (0, 190), (319, 215), (20, 20, 20), -1)
        after = observer.observe(encoded(pixels))
        self.assertEqual(before['state'], after['state'])
        self.assertNotEqual(before['imageSha256'], after['imageSha256'])

    def test_new_observation_is_not_accepted_as_legacy_OBS3(self):
        from learning.near_gripper_tip_observer import validate_observation as validate_near
        result = GripperRecoveryObserver().observe(encoded(image(80, pink=True)))
        with self.assertRaisesRegex(ValueError, 'near_observation_contract'):
            validate_near(result)


if __name__ == '__main__':
    unittest.main()
