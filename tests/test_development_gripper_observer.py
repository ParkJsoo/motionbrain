"""합성 영상의 변화·모호함·가림·잘림과 오프라인 경계를 검사한다."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from learning.development_gripper_observer import DevelopmentGripperObserver, default_parameters
from learning.development_gripper_replay import run_replay


CONTEXT = dict(viewId='synthetic', poseRegime='synthetic-only', object='synthetic-pink')
ROIS = dict(jawLeft=[90, 90, 118, 118], jawRight=[200, 90, 228, 118])
POINTS = dict(jawLeft=[106, 110], jawRight=[210, 110])


def encode(image):
    ok, raw = cv2.imencode('.png', image)
    assert ok
    return raw.tobytes()


def image():
    im = np.full((240, 320, 3), 200, np.uint8)
    for i, roi in enumerate(ROIS.values()):
        x0, y0, x1, y1 = roi
        texture = np.random.default_rng(50+i).integers(20, 140, (y1-y0, x1-x0), dtype=np.uint8)
        im[y0:y1, x0:x1] = texture[:, :, None]
    cv2.rectangle(im, (150, 165), (174, 205), (120, 40, 240), -1)
    return im


def reference(raw):
    return dict(id='synthetic-seed', imageBytes=raw, imageSha256=hashlib.sha256(raw).hexdigest(),
                points=copy.deepcopy(POINTS), rois=copy.deepcopy(ROIS))


class DevelopmentGripperTests(unittest.TestCase):
    def test_explicit_goal_view_has_separate_color_window_and_keeps_clipping_gate(self):
        im=image();im[165:206,150:175]=200
        cv2.rectangle(im,(150,60),(174,95),(120,40,240),-1)
        raw=encode(im);ref=reference(self.raw)
        goal=DevelopmentGripperObserver(CONTEXT,[ref],goal_view=True)
        self.assertNotEqual(goal.binding,self.observer.binding)
        self.assertEqual(self.observe(im)['status'],'partial_development')
        result=goal.observe(raw,'goal',CONTEXT,image_sha256=hashlib.sha256(raw).hexdigest())
        self.assertEqual(result['status'],'complete_development')
        self.assertFalse(result['controlEligible'])
        im[60:96,150:175]=200;cv2.rectangle(im,(150,50),(174,85),(120,40,240),-1)
        raw=encode(im)
        self.assertEqual(goal.observe(raw,'clipped',CONTEXT,image_sha256=hashlib.sha256(raw).hexdigest())['status'],
                         'partial_development')
        bad=default_parameters();bad['colorRows']=[50,240]
        with self.assertRaisesRegex(ValueError,'fixed_policy'):
            DevelopmentGripperObserver(CONTEXT,[ref],parameters=bad)
        bad['minimumNcc']=.89
        with self.assertRaisesRegex(ValueError,'fixed_policy'):
            DevelopmentGripperObserver(CONTEXT,[ref],parameters=bad,goal_view=True)

    def setUp(self):
        self.im = image()
        self.raw = encode(self.im)
        self.observer = DevelopmentGripperObserver(CONTEXT, [reference(self.raw)])

    def observe(self, im):
        raw = encode(im)
        return self.observer.observe(raw, 'synthetic-frame', CONTEXT, image_sha256=hashlib.sha256(raw).hexdigest())

    def test_frame_changes_are_observed_without_using_action_or_labels(self):
        moved = self.im.copy()
        moved[165:206, 150:175] = 200
        cv2.rectangle(moved, (154, 173), (178, 213), (120, 40, 240), -1)
        for name, (dx, dy) in dict(jawLeft=(-4, 3), jawRight=(5, 3)).items():
            x0, y0, x1, y1 = ROIS[name]
            moved[y0:y1, x0:x1] = 200
            moved[y0+dy:y1+dy, x0+dx:x1+dx] = self.im[y0:y1, x0:x1]
        r = self.observe(moved)
        self.assertEqual(r['status'], 'complete_development')
        self.assertEqual(r['object']['center'], [166.5, 193.5])
        self.assertEqual(r['jaws']['jawLeft']['point'], [102, 113])
        self.assertEqual(r['jaws']['jawRight']['point'], [215, 113])
        self.assertEqual(r['features']['jawHorizontalSpanPx'], 113)
        self.assertFalse(r['trainingEligible'])
        self.assertFalse(r['controlEligible'])
        self.assertFalse(r['liveAdmissionVerified'])

    def test_border_object_never_yields_complete_state(self):
        im = self.im.copy(); im[165:206, 150:175] = 200
        cv2.rectangle(im, (150, 220), (174, 239), (120, 40, 240), -1)
        r = self.observe(im)
        self.assertEqual(r['object']['reason'], 'color_at_observation_boundary')
        self.assertIsNone(r['object']['center'])
        self.assertIsNone(r['features'])
        self.assertIsNotNone(r['jaws']['jawLeft']['point'])

    def test_duplicate_color_is_not_resolved_by_largest_component(self):
        im = self.im.copy(); cv2.rectangle(im, (30, 165), (45, 185), (120, 40, 240), -1)
        r = self.observe(im)
        self.assertEqual(r['object']['reason'], 'color_ambiguous')
        self.assertIsNone(r['features'])

    def test_oversized_color_cannot_be_ignored_as_background(self):
        im = self.im.copy(); im[75:] = (120, 40, 240)
        self.assertEqual(self.observe(im)['object']['reason'], 'color_area_exceeded')

    def test_missing_object_keeps_jaws_partial(self):
        im = self.im.copy(); im[165:206, 150:175] = 200
        r = self.observe(im)
        self.assertEqual(r['object']['reason'], 'color_missing')
        self.assertIsNone(r['features'])
        self.assertIsNotNone(r['jaws']['jawRight']['point'])

    def test_one_hidden_corner_does_not_invent_symmetry(self):
        im = self.im.copy(); im[90:118, 90:118] = 200
        r = self.observe(im)
        self.assertIsNone(r['jaws']['jawLeft']['point'])
        self.assertEqual(r['jaws']['jawRight']['point'], POINTS['jawRight'])
        self.assertIsNone(r['features'])

    def test_nonpink_local_corner_occlusion_is_not_reconstructed_from_surroundings(self):
        for radius in (1, 3, 5):
            with self.subTest(radius=radius):
                im = self.im.copy()
                x, y = POINTS['jawLeft']
                im[y-radius:y+radius+1, x-radius:x+radius+1] = 200
                r = self.observe(im)
                self.assertIsNone(r['jaws']['jawLeft']['point'])
                self.assertIsNotNone(r['jaws']['jawRight']['point'])
                self.assertIsNone(r['features'])

    def test_color_at_corner_removes_that_corner(self):
        im = self.im.copy(); cv2.rectangle(im, (104, 108), (108, 112), (120, 40, 240), -1)
        r = self.observe(im)
        self.assertIsNone(r['jaws']['jawLeft']['point'])
        self.assertIsNotNone(r['jaws']['jawRight']['point'])

    def test_repeated_texture_is_rejected_not_chosen(self):
        im = self.im.copy(); im[90:118, 120:148] = self.im[90:118, 90:118]
        self.assertIsNone(self.observe(im)['jaws']['jawLeft']['point'])

    def test_disagreeing_reference_points_are_not_selected_by_score(self):
        a, b = reference(self.raw), reference(self.raw)
        b['id'] = 'other-seed'; b['points']['jawLeft'][0] -= 6
        obs = DevelopmentGripperObserver(CONTEXT, [a, b])
        r = obs.observe(self.raw, 'f', CONTEXT, image_sha256=hashlib.sha256(self.raw).hexdigest())
        self.assertEqual(r['jaws']['jawLeft']['reason'], 'corner_reference_disagreement')
        self.assertIsNone(r['features'])

    def test_hash_shape_context_and_policy_changes_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'hash'):
            self.observer.observe(self.raw, 'f', CONTEXT, image_sha256='0'*64)
        with self.assertRaisesRegex(ValueError, 'context_changed'):
            self.observer.observe(self.raw, 'f', dict(CONTEXT, viewId='other'), image_sha256=hashlib.sha256(self.raw).hexdigest())
        raw = encode(np.zeros((100, 100, 3), np.uint8))
        with self.assertRaisesRegex(ValueError, 'shape'):
            self.observer.observe(raw, 'f', CONTEXT, image_sha256=hashlib.sha256(raw).hexdigest())
        self.observer.parameters['minimumNcc'] = .1
        with self.assertRaisesRegex(ValueError, 'configuration_changed'):
            self.observe(self.im)
        p = default_parameters(); p['minimumNcc'] = .1
        with self.assertRaisesRegex(ValueError, 'policy_changed'):
            DevelopmentGripperObserver(CONTEXT, [reference(self.raw)], parameters=p)

    def test_reference_hash_is_verified(self):
        r = reference(self.raw); r['imageSha256'] = '0'*64
        with self.assertRaisesRegex(ValueError, 'reference_hash'):
            DevelopmentGripperObserver(CONTEXT, [r])

    def test_replay_audits_source_and_rejects_public_output_and_duplicate_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source = root/'.local/source'; source.mkdir(parents=True)
            source.joinpath('frame.png').write_bytes(self.raw)
            digest = hashlib.sha256(self.raw).hexdigest()
            label = dict(imageBinding=dict(path='frame.png', sha256=digest,
                decodedRgbSha256=hashlib.sha256(cv2.cvtColor(self.im, cv2.COLOR_BGR2RGB).tobytes()).hexdigest()),
                visiblePinkRegionBoxXYXY=[150, 165, 175, 206],
                jawLeftDistalInnerCorner=dict(xy=POINTS['jawLeft']), jawRightDistalInnerCorner=dict(xy=POINTS['jawRight']))
            labels = json.dumps(dict(frames=[label])).encode(); source.joinpath('labels.json').write_bytes(labels)
            spec = dict(schemaVersion='motionbrain.development_gripper_replay.v1', sourceRoot=str(source),
                context=CONTEXT, references=[dict(id='seed', path='frame.png', sha256=digest, points=POINTS, rois=ROIS)],
                frames=[dict(frameId='f', path='frame.png', sha256=digest)],
                annotations=dict(path='labels.json', sha256=hashlib.sha256(labels).hexdigest()))
            manifest = source/'manifest.json'; manifest.write_text(json.dumps(spec))
            report = run_replay(manifest, root/'.local/replay', root=root)
            self.assertEqual(report['completeFrames'], 1)
            self.assertEqual(report['referenceSelfReplayFrames'], 1)
            self.assertEqual(report['nonSeedLabelAgreement']['labeledFrames'], 0)
            self.assertIsNone(report['nonSeedLabelAgreement']['object']['meanEuclideanPx'])
            self.assertEqual(report['hardwareRequests'], 0)
            with self.assertRaisesRegex(ValueError, 'private_output'):
                run_replay(manifest, root/'public', root=root)
            spec['frames'] *= 2; manifest.write_text(json.dumps(spec))
            with self.assertRaisesRegex(ValueError, 'duplicate_frame'):
                run_replay(manifest, root/'.local/other', root=root)
            spec['frames'] = spec['frames'][:1]; manifest.write_text(json.dumps(spec))
            source.joinpath('frame.png').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'reference_hash'):
                run_replay(manifest, root/'.local/changed', root=root)

    def test_baseline_seed_label_is_verified_when_seed_is_not_replayed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source = root/'.local/source'; source.mkdir(parents=True)
            source.joinpath('seed.png').write_bytes(self.raw)
            source.joinpath('other.png').write_bytes(self.raw)
            digest = hashlib.sha256(self.raw).hexdigest()
            label = dict(imageBinding=dict(path='seed.png', sha256=digest,
                decodedRgbSha256=hashlib.sha256(self.im[:, :, ::-1].tobytes()).hexdigest()),
                visiblePinkRegionBoxXYXY=[150, 165, 175, 206],
                jawLeftDistalInnerCorner=dict(xy=POINTS['jawLeft']), jawRightDistalInnerCorner=dict(xy=POINTS['jawRight']))
            spec = dict(schemaVersion='motionbrain.development_gripper_replay.v1', sourceRoot=str(source),
                context=CONTEXT, references=[dict(id='seed', path='seed.png', sha256=digest, points=POINTS, rois=ROIS)],
                frames=[dict(frameId='nonseed', path='other.png', sha256=digest)], annotations={})
            manifest = source/'manifest.json'
            for key in ('sha256', 'decodedRgbSha256'):
                bad = copy.deepcopy(label); bad['imageBinding'][key] = '0'*64
                raw = json.dumps(dict(frames=[bad])).encode(); source.joinpath('labels.json').write_bytes(raw)
                spec['annotations'] = dict(path='labels.json', sha256=hashlib.sha256(raw).hexdigest())
                manifest.write_text(json.dumps(spec))
                with self.assertRaisesRegex(ValueError, 'baseline_seed_label_binding'):
                    run_replay(manifest, root/'.local'/key, root=root)


if __name__ == '__main__':
    unittest.main()
