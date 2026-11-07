"""Bounded appearance tracking checks; no physical accuracy/retention claims."""
import copy
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.registered_gripper_landmarks import (
    RegisteredGripperLandmarks, default_parameters, register,
)
from learning.relative_action_learning import annotated_transition, fit, point_state


def encoded(image):
    ok, raw = cv2.imencode('.png', image)
    assert ok
    return raw.tobytes()


class RegisteredLandmarksTests(unittest.TestCase):
    def setUp(self):
        self.image = np.random.default_rng(19).integers(10, 245, (240, 320, 3), dtype=np.uint8)
        self.raw = encoded(self.image)
        self.points = dict(object=[158, 98], jawLeft=[121, 135], jawRight=[193, 135])
        self.reg = register(self.raw, self.points)
        self.tracker = RegisteredGripperLandmarks(self.reg, self.raw)

    def move_patches(self, shifts):
        image = self.image.copy()
        for name, shift in shifts.items():
            x0, y0, x1, y1 = self.reg['parameters']['rois'][name]
            dx, dy = shift
            image[y0:y1, x0:x1] = 0
            image[y0+dy:y1+dy, x0+dx:x1+dx] = self.image[y0:y1, x0:x1]
        return encoded(image)

    def test_original_pixel_binding_and_small_translation(self):
        shifts = dict(object=[1, 1], jawLeft=[-7, -5], jawRight=[7, -5])
        result = self.tracker.observe(self.move_patches(shifts), 'fresh-frame')
        for name in shifts:
            expected = np.asarray(self.points[name])+shifts[name]
            np.testing.assert_allclose(result['annotation']['points'][name], expected, atol=.08)
        frame = dict(frameId='fresh-frame', pixelSha256=result['pixelSha256'], width=320, height=240)
        np.testing.assert_allclose(point_state(result['annotation'], frame), result['state'])
        self.assertFalse(result['measurementAccuracyVerified'])
        self.assertFalse(result['liveMotionTrackingVerified'])
        self.assertLess(result['elapsedSeconds'], 2.)

    def test_two_open_hypothesis_keeps_vertical_midpoint_motion(self):
        result = self.tracker.observe(self.move_patches(dict(jawLeft=[-9, -8], jawRight=[9, -8])), 'two-open')
        self.assertAlmostEqual(result['state'][2], 90., delta=.1)
        self.assertAlmostEqual(result['state'][1], -29., delta=.1)

    def test_search_boundary_is_not_accepted_as_full_match(self):
        with self.assertRaisesRegex(ValueError, 'search_boundary'):
            self.tracker.observe(self.move_patches(dict(jawLeft=[-24, 0])), 'boundary')

    def test_disappearing_or_deformed_object_is_rejected(self):
        changed = self.image.copy(); changed[69:125, 134:184] = 127
        with self.assertRaises(ValueError):
            self.tracker.observe(encoded(changed), 'hidden-object')

    def test_second_strong_peak_is_rejected(self):
        scores = np.zeros((9, 9), np.float32)
        scores[2, 2] = .99; scores[6, 6] = .98
        with patch('learning.registered_gripper_landmarks.cv2.matchTemplate', return_value=scores):
            with self.assertRaisesRegex(ValueError, 'match_ambiguous'):
                self.tracker.observe(self.raw, 'ambiguous')

    def test_incompatible_jaw_vertical_motion_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'geometry_changed'):
            self.tracker.observe(self.move_patches(dict(jawLeft=[-4, -5], jawRight=[4, 5])), 'deformation')

    def test_registration_parameters_and_reference_are_pinned(self):
        reg = copy.deepcopy(self.reg); reg['parameters']['minimumPeakGap'] = .01
        with self.assertRaisesRegex(ValueError, 'contract_changed'):
            RegisteredGripperLandmarks(reg, self.raw)
        changed = self.image.copy(); changed[0, 0] = 255
        with self.assertRaisesRegex(ValueError, 'contract_changed'):
            RegisteredGripperLandmarks(self.reg, encoded(changed))
        self.tracker.registration['points']['object'][0] += 1
        with self.assertRaisesRegex(ValueError, 'contract_changed'):
            self.tracker.observe(self.raw, 'mutated')

    def test_clipped_registration_and_low_texture_are_rejected(self):
        p = default_parameters(); p['rois']['jawLeft'][0] = 10
        with self.assertRaisesRegex(ValueError, 'clipped'):
            register(self.raw, self.points, p)
        with self.assertRaisesRegex(ValueError, 'texture_insufficient'):
            register(encoded(np.zeros_like(self.image)), self.points)

    def test_different_resolution_and_wrong_frame_binding_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'shape_changed'):
            self.tracker.observe(encoded(self.image[:200]), 'wrong-resolution')
        result = self.tracker.observe(self.raw, 'right')
        frame = dict(frameId='wrong', pixelSha256=result['pixelSha256'], width=320, height=240)
        with self.assertRaisesRegex(ValueError, 'frame_mismatch'):
            point_state(result['annotation'], frame)

    def test_new_annotation_requires_all_registration_hashes(self):
        result = self.tracker.observe(self.raw, 'right')
        frame = dict(frameId='right', pixelSha256=result['pixelSha256'], width=320, height=240)
        for name in ('registrationSha256', 'parametersSha256', 'referenceImageSha256', 'landmarkContract'):
            ann = copy.deepcopy(result['annotation']); ann.pop(name)
            with self.assertRaisesRegex(ValueError, 'contract_required'):
                point_state(ann, frame)

    def transition(self, session):
        annotations = []; frames = []
        for label in ('before', 'after'):
            result = self.tracker.observe(self.raw, session+'-'+label)
            frames.append(dict(frameId=session+'-'+label, pixelSha256=result['pixelSha256'], width=320, height=240))
            annotations.append(result['annotation'])
        row = dict(sessionId=session,
            inputs=dict(image=frames[0], command=dict(joint='gripper', direction='close', percent=40, durationMs=350)),
            targets=dict(image=frames[1]),
            evidence=dict(cameraIdentity=dict(camera='gripper'), controllerBootId='a'*32, commandId=session))
        ann = dict(sessionId=session, captureGroup='new-view-training',
            context=dict(physicalView='new-view', object='same-cylinder', poseRegime='new-table-pose',
                         landmarks=self.tracker.landmark_contract), before=annotations[0], after=annotations[1])
        return row, ann

    def test_registered_method_and_context_cannot_be_mixed(self):
        row, ann = self.transition('one')
        valid = annotated_transition(row, ann)
        self.assertEqual(valid['context']['landmarks'], self.tracker.landmark_contract)
        changed = copy.deepcopy(ann); changed['context']['landmarks'] = 'old-analyst-landmarks'
        with self.assertRaisesRegex(ValueError, 'context_changed'):
            annotated_transition(row, changed)
        changed = copy.deepcopy(ann); changed['after']['method'] = 'analyst_points'
        with self.assertRaisesRegex(ValueError, 'method_changed'):
            annotated_transition(row, changed)

    def test_model_semantics_do_not_claim_manual_analyst_points(self):
        rows = [annotated_transition(*self.transition(s)) for s in ('one', 'two')]
        model = fit(rows)
        self.assertIn('registered_template_translation', model['annotationSemantics'])
        self.assertFalse(model['controlEligible'])


if __name__ == '__main__':
    unittest.main()
