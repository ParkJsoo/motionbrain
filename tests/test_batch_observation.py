import copy
import hashlib
import unittest

import cv2
import numpy as np

from learning.batch_observation import SCHEMA, observe_frame, validate_spec


class BatchObservationTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((100, 120, 3), np.uint8)
        self.context = dict(viewId='fixed', poseRegime='local', object='none', load='empty')
        markers = {}
        for name, hue, position in [('body', 0, (20, 20)), ('elbow', 60, (50, 50)),
                                     ('tip', 120, (80, 70)),
                                     ('anchorA', 30, (20, 80)), ('anchorB', 90, (100, 20))]:
            color = cv2.cvtColor(np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
            cv2.circle(self.image, position, 4, tuple(int(v) for v in color), -1)
            markers[name] = dict(hsvLower=[hue, 200, 200], hsvUpper=[hue, 255, 255],
                minAreaPx=20, maxAreaPx=70, uncertaintyPx=.5,
                role='fixed_view_anchor' if name.startswith('anchor') else name+'_marker')
        features = [dict(name=n+axis, kind='relative_'+axis, a=n, b='body')
                    for n in ('elbow', 'tip') for axis in ('x', 'y')]
        bounds = dict(elbowx=[28, 32], elbowy=[28, 32], tipx=[58, 62], tipy=[48, 52])
        self.spec = dict(schemaVersion=SCHEMA, cameraIdentity='camera', imageShape=[100, 120],
            **self.context, maxAgeSeconds=2, maxLandmarkUncertaintyPx=1,
            landmarks=markers, poseLandmarks=['body', 'elbow', 'tip'],
            fixedViewAnchors=dict(anchorA=[18, 22, 78, 82], anchorB=[98, 102, 18, 22]),
            joints={j:dict(stateFields=copy.deepcopy(features), nodes=[dict(id=j+'-node', bounds=copy.deepcopy(bounds))])
                    for j in ('wrist', 'elbow')})

    def observe(self, *, image=None, spec=None, frame_change=None, now=11):
        image = self.image if image is None else image
        raw = cv2.imencode('.png', image)[1].tobytes()
        frame = dict(frameId='one', imageSha256=hashlib.sha256(raw).hexdigest(),
            pixelSha256=hashlib.sha256(image.tobytes()).hexdigest(), capturedAt=10,
            persistedAt=10.2, clockId='clock', cameraIdentity='camera')
        frame.update(frame_change or {})
        return observe_frame(raw, frame, spec or self.spec, self.context, now, 'clock')

    def test_actual_color_positions_and_all_joint_node_matches(self):
        result = self.observe()
        self.assertEqual(result['status'], 'accepted')
        self.assertEqual(result['states']['wrist']['state'], [30, 30, 60, 50])
        self.assertEqual(result['nodeMatches'], ['wrist-node', 'elbow-node'])
        self.assertFalse(result['controlEligible'])
        self.assertFalse(result['physicalValidationEstablished'])

    def test_duplicate_marker_is_not_arbitrarily_selected(self):
        image = self.image.copy()
        cv2.circle(image, (90, 20), 4, (0, 0, 255), -1)
        self.assertIn('marker_missing_or_ambiguous:body', self.observe(image=image)['reason'])

    def test_occlusion_is_not_filled_from_other_pose_markers(self):
        image = self.image.copy(); image[60:80, 70:90] = 0
        result = self.observe(image=image)
        self.assertEqual(result['states'], {})
        self.assertEqual(result['nodeMatches'], [])
        self.assertIn('marker_missing_or_ambiguous:tip', result['reason'])

    def test_same_tip_xy_different_elbow_does_not_match_old_pose(self):
        image = self.image.copy(); image[40:60, 40:60] = 0
        cv2.circle(image, (55, 50), 4, (0, 255, 0), -1)
        result = self.observe(image=image)
        self.assertEqual(result['status'], 'accepted')
        self.assertEqual(result['nodeMatches'], [])

    def test_uncertainty_must_fit_whole_interval(self):
        spec = copy.deepcopy(self.spec)
        spec['joints']['wrist']['nodes'][0]['bounds']['tipx'] = [60, 62]
        self.assertEqual(self.observe(spec=spec)['nodeMatches'], ['elbow-node'])

    def test_each_node_requires_all_features_and_unique_id(self):
        spec = copy.deepcopy(self.spec)
        del spec['joints']['wrist']['nodes'][0]['bounds']['elbowx']
        with self.assertRaisesRegex(ValueError, 'node_bounds_all_features_required'):
            validate_spec(spec)
        spec = copy.deepcopy(self.spec)
        spec['joints']['elbow']['nodes'][0]['id'] = 'wrist-node'
        with self.assertRaisesRegex(ValueError, 'campaign_unique_node_id_required'):
            validate_spec(spec)

    def test_whole_pose_xy_declaration_required_for_every_joint(self):
        spec = copy.deepcopy(self.spec)
        spec['joints']['wrist']['stateFields'] = spec['joints']['wrist']['stateFields'][2:]
        with self.assertRaisesRegex(ValueError, 'whole_pose_features_required'):
            validate_spec(spec)

    def test_hash_clock_age_and_camera_identity_fail_closed(self):
        for change, expected in [({'imageSha256':'bad'}, 'frame_image_hash_changed'),
                                 ({'pixelSha256':'bad'}, 'frame_pixel_hash_changed'),
                                 ({'clockId':'other'}, 'frame_clock_mismatch'),
                                 ({'capturedAt':1}, 'frame_stale_or_noncausal'),
                                 ({'persistedAt':12}, 'frame_stale_or_noncausal'),
                                 ({'cameraIdentity':'other'}, 'frame_camera_changed')]:
            with self.subTest(change=change):
                result = self.observe(frame_change=change)
                self.assertEqual(result['status'], 'rejected')
                self.assertEqual(result['reason'], expected)

    def test_overlapping_color_identity_rejected(self):
        spec = copy.deepcopy(self.spec)
        spec['landmarks']['tip']['hsvLower'] = [0, 200, 200]
        spec['landmarks']['tip']['hsvUpper'] = [0, 255, 255]
        self.assertIn('marker_color_identity_overlap:tip', self.observe(spec=spec)['reason'])

    def test_context_change_rejected(self):
        spec = copy.deepcopy(self.spec); spec['load'] = 'held'
        self.assertEqual(self.observe(spec=spec)['reason'], 'observer_context_changed')

    def test_same_joint_overlapping_nodes_rejected(self):
        spec = copy.deepcopy(self.spec)
        other = copy.deepcopy(spec['joints']['wrist']['nodes'][0]); other['id'] = 'wrist-other'
        spec['joints']['wrist']['nodes'].append(other)
        result = self.observe(spec=spec)
        self.assertEqual(result['status'], 'accepted')
        self.assertEqual(result['perJointRejected']['wrist'], 'ambiguous_node:wrist')
        self.assertNotIn('wrist', result['states'])
        self.assertEqual(result['nodeMatches'], ['elbow-node'])

    def test_camera_translation_cannot_cancel_in_relative_features(self):
        # All moving-marker relative features remain unchanged after translation,
        # but fixed background anchors leave the allowed registration intervals.
        shifted = cv2.warpAffine(self.image, np.float32([[1,0,4],[0,1,0]]), (120,100))
        result = self.observe(image=shifted)
        self.assertEqual(result['status'], 'rejected')
        self.assertIn('fixed_view_anchor_moved:', result['reason'])
        self.assertEqual(result['nodeMatches'], [])

    def test_two_declared_stationary_anchors_required(self):
        spec = copy.deepcopy(self.spec); del spec['fixedViewAnchors']['anchorB']
        with self.assertRaisesRegex(ValueError, 'fixed_view_anchors_required'):
            validate_spec(spec)
        spec = copy.deepcopy(self.spec); spec['landmarks']['anchorB']['role'] = 'wrist_marker'
        with self.assertRaisesRegex(ValueError, 'fixed_view_anchor_role_required'):
            validate_spec(spec)

    def test_optional_jaw_occlusion_keeps_unrelated_base_observation(self):
        spec = copy.deepcopy(self.spec)
        spec['landmarks']['jawExtra'] = dict(hsvLower=[150,200,200], hsvUpper=[150,255,255],
            minAreaPx=20, maxAreaPx=70, uncertaintyPx=.5, role='jaw_tip_marker')
        spec['joints']['base'] = copy.deepcopy(spec['joints']['wrist'])
        spec['joints']['base']['nodes'][0]['id'] = 'base-node'
        spec['joints']['gripper'] = copy.deepcopy(spec['joints']['wrist'])
        gripper = spec['joints']['gripper']
        gripper['stateFields'].append(dict(name='jawExtraX', kind='relative_x', a='jawExtra', b='body'))
        gripper['nodes'][0]['id'] = 'gripper-node'
        gripper['nodes'][0]['bounds']['jawExtraX'] = [78,82]
        # There is deliberately no purple jaw marker in this frame.
        result = self.observe(spec=spec)
        self.assertEqual(result['status'], 'accepted')
        self.assertIn('base-node', result['nodeMatches'])
        self.assertNotIn('gripper', result['states'])
        self.assertIn('gripper', result['perJointRejected'])
        self.assertIn('jawExtra', result['perLandmarkRejected'])
        self.assertNotIn('jawExtra', result['landmarks'])

    def test_optional_color_collision_rejects_only_dependent_joints(self):
        spec = copy.deepcopy(self.spec)
        for name in ('optionalA', 'optionalB'):
            spec['landmarks'][name] = dict(hsvLower=[150,200,200], hsvUpper=[150,255,255],
                minAreaPx=20, maxAreaPx=70, uncertaintyPx=.5, role='optional_marker')
        image = self.image.copy()
        color = cv2.cvtColor(np.uint8([[[150,255,255]]]), cv2.COLOR_HSV2BGR)[0,0]
        cv2.circle(image, (100,80), 4, tuple(int(v) for v in color), -1)
        result = self.observe(spec=spec, image=image)
        self.assertEqual(result['status'], 'accepted')
        self.assertEqual(set(result['perLandmarkRejected']), {'optionalA','optionalB'})
        self.assertNotIn('optionalA', result['landmarks'])
        self.assertNotIn('optionalB', result['landmarks'])


if __name__ == '__main__':
    unittest.main()
