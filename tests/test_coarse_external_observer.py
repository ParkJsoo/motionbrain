from pathlib import Path
import unittest

import cv2
import numpy as np

from learning.coarse_external_observer import register, CoarseExternalObserver, MEANING


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


if __name__ == '__main__': unittest.main()
