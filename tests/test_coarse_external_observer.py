from pathlib import Path
import copy
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.coarse_external_observer import register, CoarseExternalObserver, CoarseExternalBank, MEANING


class ObserverTests(unittest.TestCase):
    def setUp(self):
        self.image = np.full((1080, 1920, 3), 180, np.uint8)
        rng = np.random.default_rng(5)
        texture = rng.integers(20, 150, (140, 245), np.uint8)
        self.image[140:280, 915:1160] = cv2.cvtColor(texture, cv2.COLOR_GRAY2BGR)
        cv2.ellipse(self.image, (1068, 234), (11, 9), 0, 0, 360, (110, 65, 110), -1)
        cv2.ellipse(self.image, (1100, 237), (10, 10), 0, 0, 360, (80, 110, 65), -1)
        cv2.rectangle(self.image, (789, 627), (844, 795), (100, 20, 240), -1)
        self.raw = self.encode(self.image)
        self.registration = self.register(self.raw)
        self.observer = CoarseExternalObserver(self.registration, self.raw)

    def encode(self, image):
        ok, raw = cv2.imencode('.png', image)
        self.assertTrue(ok)
        return raw.tobytes()

    def register(self, raw):
        return register(raw, tool_point=[936, 200], feature_roi=[915, 140, 1160, 280],
                        tool_patch_roi=[920, 170, 980, 224],
                        marker_rois={'purple': [1051, 215, 1088, 253], 'teal': [1084, 218, 1118, 256]})

    def test_rotation_translation_tracks_proxy_and_preserves_constant_semantics(self):
        affine = cv2.getRotationMatrix2D((1080, 230), 15, 1.)
        affine[:, 2] += [10, 150]
        moving = self.image.copy(); moving[627:796, 789:845] = 180
        after = cv2.warpAffine(moving, affine, (1920, 1080), borderValue=(180, 180, 180))
        cv2.rectangle(after, (789, 627), (844, 795), (100, 20, 240), -1)
        result = self.observer.observe(self.encode(after))
        expected = cv2.transform(np.float32([[[936, 200]]]), affine)[0, 0]
        self.assertLess(np.linalg.norm(np.array(result['toolProxyPx'])-expected), 3.)
        self.assertEqual(result['state'][2], 1.)
        self.assertEqual(result['stateMeaning'], MEANING)
        self.assertFalse(result['contactVerified'])
        self.assertFalse(result['graspVerified'])
        self.assertEqual(result['registrationSha256'], self.observer.binding)

    def test_missing_either_marker_is_rejected(self):
        for kind, roi in self.registration['markerRois'].items():
            with self.subTest(kind=kind):
                image = self.image.copy(); x, y, xx, yy = roi; image[y:yy, x:xx] = 180
                with self.assertRaises(ValueError): self.observer.observe(self.encode(image))

    def test_missing_duplicate_or_occluded_object_is_rejected(self):
        missing = self.image.copy(); missing[620:810, 780:855] = 180
        duplicate = self.image.copy(); cv2.rectangle(duplicate, (650, 627), (705, 795), (100, 20, 240), -1)
        occluded = self.image.copy(); occluded[670:800, 780:855] = 180
        for image in [missing, duplicate, occluded]:
            with self.assertRaises(ValueError): self.observer.observe(self.encode(image))

    def test_tool_occlusion_rejected_even_when_wrist_markers_remain(self):
        image = self.image.copy(); image[150:240, 910:1000] = 180
        with self.assertRaisesRegex(ValueError, 'coarse_tool_not_visible'):
            self.observer.observe(self.encode(image))

    def test_reference_or_registration_mutation_rejected(self):
        with self.assertRaisesRegex(ValueError, 'reference_changed'):
            CoarseExternalObserver(self.registration, self.raw+b' ')
        self.observer.registration['toolPoint'][0] += 1
        with self.assertRaisesRegex(ValueError, 'registration_changed'): self.observer.observe(self.raw)

    def test_wrong_shape_or_blank_rejected(self):
        for image in [np.zeros((240, 320, 3), np.uint8), np.zeros_like(self.image)]:
            with self.assertRaises(ValueError): self.observer.observe(self.encode(image))

    def test_relative_model_keeps_coarse_fields_distinct_from_jaw_opening(self):
        from learning import relative_action_learning as relative
        from learning.attended_collection_profile import ATTENDED_60_1000, collection_profile
        from learning.relative_model_bundle import validate_model
        context = dict(physicalView='current', object='pink-block', poseRegime='L-pose',
            landmarks=self.observer.landmark_contract, cameraIdentity=dict(camera='external'),
            imageShape=[1080, 1920], stateFields=relative.state_fields(dict(landmarks=self.observer.landmark_contract)),
            collectionProfile=collection_profile(ATTENDED_60_1000))
        command = dict(joint='elbow', direction='up', percent=75, durationMs=1000)
        rows = [dict(sessionId=f's{i}', captureGroup='development', context=context,
                     command=command, before=[-120., y, 1.], after=[-120., y-150., 1.],
                     episode=['boot', f'c{i}'], frameIds=[f'b{i}', f'a{i}']) for i, y in enumerate([500., 350.])]
        model = validate_model(relative.fit(rows))
        result = relative.select(model, [-120., 400., 1.], [-120., 250., 1.], [command], context)
        self.assertEqual(result['selected']['kind'], 'action')
        self.assertEqual(result['selected']['predictedState'][2], 1.)
        self.assertNotIn('jawOpeningPx', model['context']['stateFields'])
        with self.assertRaisesRegex(ValueError, 'coarse_external_elbow_only'):
            relative.validate_motion_scope(context, dict(joint='gripper'))
        with self.assertRaisesRegex(ValueError, 'compatibility_constant_changed'):
            relative.select(model, [-120., 400., 1.], [-120., 250., 2.], [command], context)

    def test_real_current_saved_frames_when_available(self):
        root = Path(__file__).resolve().parents[1]/'.local/current-entry-20261114'
        before = root/'L-pose-current-20261115-04/external.png'
        after = root/'L-pose-current-20261115-05-after-failed-M3/external.png'
        if not before.exists() or not after.exists(): self.skipTest('private current frames unavailable')
        raw = before.read_bytes(); observer = CoarseExternalObserver(self.register(raw), raw)
        a, b = [observer.observe(p.read_bytes()) for p in (before, after)]
        self.assertLess(np.linalg.norm(np.array(a['object']['center'])-b['object']['center']), 2.)
        self.assertGreater(b['toolProxyPx'][1]-a['toolProxyPx'][1], 150.)
        self.assertLess(b['quality']['medianResidualPx'], 2.)
        self.assertGreater(b['quality']['toolNcc'], .5)
        self.assertEqual(b['state'][2], 1.)

    def test_bank_fixed_correspondences_and_missing_observation(self):
        second = copy.deepcopy(self.registration)
        second['toolPoint'] = self.observer.observe(self.raw)['toolProxyPx']
        second['featureRoi'] = [916, 140, 1160, 280]
        bank = CoarseExternalBank([self.registration, second], [self.raw, self.raw])
        result = bank.observe(self.raw)
        self.assertEqual(len(result['acceptedReferences']), 2)
        self.assertEqual(result['registrationSha256'], bank.binding)
        self.assertEqual(result['landmarkContract'], bank.landmark_contract)
        self.assertEqual(len(result['referenceSha256s']), 2)
        missing = self.image.copy(); missing[620:810, 780:855] = 180
        with self.assertRaisesRegex(ValueError, 'no_supported_reference'): bank.observe(self.encode(missing))
        second['toolPoint'][0] += 5.
        with self.assertRaisesRegex(ValueError, 'unbound_proxy_correspondence'):
            CoarseExternalBank([self.registration, second], [self.raw, self.raw])

    def test_bank_disagreement_rejects_instead_of_choosing_better_score(self):
        second = copy.deepcopy(self.registration)
        second['toolPoint'] = self.observer.observe(self.raw)['toolProxyPx']
        second['featureRoi'] = [916, 140, 1160, 280]
        bank = CoarseExternalBank([self.registration, second], [self.raw, self.raw])
        discrepant = bank.observers[1].observe(self.raw)
        discrepant['toolProxyPx'][0] += 11.
        with patch.object(bank.observers[1], 'observe', return_value=discrepant):
            with self.assertRaisesRegex(ValueError, 'proxy_disagreement'): bank.observe(self.raw)

    def test_bank_mutated_reference_and_contract_rejected(self):
        bank = CoarseExternalBank([self.registration], [self.raw])
        bank.references = (self.raw+b' ',)
        with self.assertRaisesRegex(ValueError, 'reference_changed'): bank.observe(self.raw)
        bank = CoarseExternalBank([self.registration], [self.raw])
        bank.registration['maximumDisagreementPx'] = 11.
        with self.assertRaisesRegex(ValueError, 'registration_changed'): bank.observe(self.raw)

    def test_real_fixed_bank_supports_current_three_states_when_available(self):
        root = Path(__file__).resolve().parents[1]/'.local/current-entry-20261114'
        paths = [root/'L-pose-current-20261115-05-after-failed-M3/external.png',
                 root/'L-pose-M3-down70-1000-direct-01/external-after.png',
                 root/'L-pose-M3-down70-1000-direct-03/external-after.png']
        if not all(p.exists() for p in paths): self.skipTest('private current frames unavailable')
        raws = [p.read_bytes() for p in paths]
        first = register(raws[0], tool_point=[938, 375], feature_roi=[920, 300, 1170, 430],
            tool_patch_roi=[930, 350, 990, 400],
            marker_rois={'purple': [1066, 370, 1100, 404], 'teal': [1098, 361, 1129, 400]})
        correspondence = CoarseExternalObserver(first, raws[0]).observe(raws[1])['toolProxyPx']
        second = register(raws[1], tool_point=correspondence, feature_roi=[950, 400, 1210, 530],
            tool_patch_roi=[958, 475, 1018, 525],
            marker_rois={'purple': [1100, 470, 1136, 507], 'teal': [1130, 457, 1167, 494]})
        bank = CoarseExternalBank([first, second], raws[:2])
        results = [bank.observe(raw) for raw in raws]
        self.assertTrue(all(r['registrationSha256'] == bank.binding for r in results))
        self.assertTrue(all(r['maximumProxyDisagreementPx'] <= 10. for r in results))
        self.assertEqual(len(results[0]['acceptedReferences']), 2)
        self.assertEqual(len(results[2]['acceptedReferences']), 1)
        self.assertLess(results[2]['state'][1], 65.)
        self.assertGreater(results[2]['state'][1], 45.)


if __name__ == '__main__': unittest.main()
