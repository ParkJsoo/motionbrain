"""Synthetic saved-image diagnostics; no cameras or motor endpoints."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning import grasp_observation_diagnostic as m
from learning.gripper_camera_dataset import read_frame
from learning.relative_action_learning import fit


PARAMETERS = dict(imageShape=[240, 320], grayThreshold=90, objectRightArcRows=[60, 130],
                  objectRightArcColumns=[154, 195], jawRois=[[65, 55, 120, 140], [190, 55, 245, 140]],
                  jawDistalRows=8)
COMMAND = dict(joint='gripper', direction='close', percent=40, durationMs=350)


def image(left_occlusion=False, right_jaw=True):
    pixels = np.full((240, 320, 3), 230, np.uint8)
    cv2.circle(pixels, (154, 95), 38, (30, 30, 30), -1)
    cv2.rectangle(pixels, (90, 60), (100, 115), (20, 20, 20), -1)
    if right_jaw: cv2.rectangle(pixels, (210, 60), (220, 115), (20, 20, 20), -1)
    if left_occlusion: cv2.rectangle(pixels, (110, 82), (140, 128), (20, 20, 20), -1)
    ok, encoded = cv2.imencode('.jpg', pixels); assert ok
    return encoded.tobytes()


class GeometryTests(unittest.TestCase):
    def test_fixed_circle_and_jaws_are_diagnostic_only(self):
        r = m.extract_points(image(), PARAMETERS)
        np.testing.assert_allclose(r['points']['object'], [154, 95], atol=1.)
        self.assertAlmostEqual(r['relativeStatePx'][2], 120., delta=.2)
        self.assertFalse(r['trainingEligible']); self.assertFalse(r['controlEligible'])
        self.assertEqual(r['qualityFlags'], [])

    def test_left_occlusion_does_not_pull_right_arc_center(self):
        a = m.extract_points(image(), PARAMETERS); b = m.extract_points(image(left_occlusion=True), PARAMETERS)
        np.testing.assert_allclose(a['points']['object'], b['points']['object'], atol=.1)

    def test_missing_jaw_cannot_use_object_sliver_as_unflagged_jaw(self):
        r = m.extract_points(image(right_jaw=False), PARAMETERS)
        self.assertIn('jawRight_lateral_roi_clipped_or_wrong_component', r['qualityFlags'])

    def test_empty_corrupt_wrong_shape_and_bad_roi_fail(self):
        for raw in (b'not-jpeg', cv2.imencode('.jpg', np.full((240, 320, 3), 255, np.uint8))[1].tobytes()):
            with self.assertRaises(ValueError): m.extract_points(raw, PARAMETERS)
        changed = copy.deepcopy(PARAMETERS); changed['imageShape'] = [200, 320]
        with self.assertRaisesRegex(ValueError, 'shape_changed'): m.extract_points(image(), changed)
        changed = copy.deepcopy(PARAMETERS); changed['jawRois'][0] = [-1, 0, 10, 20]
        with self.assertRaisesRegex(ValueError, 'roi_invalid'): m.extract_points(image(), changed)

    def test_after_targets_cannot_expand_input_coverage(self):
        measurement = dict(relativeStatePx=[0., 0., 10.], nearestIntegerStatePx=[0., 0., 10.], qualityFlags=[])
        rows = [dict(role='training_before', command=COMMAND, measurement=measurement)]
        query = dict(role='rejected_query_input', measurement=dict(measurement, relativeStatePx=[0., 0., 11.], nearestIntegerStatePx=[0., 0., 11.]))
        r = m.input_coverage(rows, query)
        self.assertTrue(all(not x['insideObservedMinMax'] for x in r))
        rows.append(dict(role='training_after', command=COMMAND, measurement=query['measurement']))
        with self.assertRaisesRegex(ValueError, 'input_only'): m.input_coverage(rows, query)

    def test_rounding_is_reported_separately_without_tolerance_expansion(self):
        rows = [dict(role='training_before', command=COMMAND, measurement=dict(
            relativeStatePx=[x, 0., 10.], nearestIntegerStatePx=[rounded, 0., 10.], qualityFlags=[]))
            for x, rounded in ((-.4, 0.), (.4, 0.))]
        query = dict(role='rejected_query_input', measurement=dict(relativeStatePx=[.3, 0., 10.], nearestIntegerStatePx=[.5, 0., 10.], qualityFlags=[]))
        r = m.input_coverage(rows, query)
        self.assertTrue(r[0]['insideObservedMinMax']); self.assertFalse(r[1]['insideObservedMinMax'])
        self.assertEqual(r[1]['violations'][0]['outsideByPx'], .5)


class StudyTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve(); self.local = self.root/'.local'; self.local.mkdir()
        self.context = dict(physicalView='synthetic', object='cylinder', poseRegime='fixed', landmarks='analyst-three-points')
        self.records, self.rows = [], []
        for i, sid in enumerate(('train-a', 'train-b')):
            location = self.frames(sid, 100+i*200)
            annotation = dict(sessionId=sid, captureGroup='train', context=self.context)
            for label in ('before', 'after'): annotation[label] = self.annotation(location, sid, label)
            self.records.append(dict(sessionRoot=f'.local/{sid}', sessionSha256='source-audit', observationCamera='gripper', gripperObservationRoot=str(location), annotation=annotation))
            self.rows.append(dict(sessionId=sid, captureGroup='train', context=dict(self.context, cameraIdentity={'boot':'a'*32}, imageShape=[240,320], stateFields=m.STATE), command=COMMAND, before=[0.,0.,120.], after=[0.,0.,120.], episode=['boot', sid], frameIds=['a'*32+':'+str(100+i*200),'a'*32+':'+str(200+i*200)]))
        self.manifest = self.local/'manifest.json'; self.manifest.write_text(json.dumps(dict(records=self.records, evaluationGroups=[])))
        self.model = self.local/'model.json'; self.model.write_text(json.dumps(fit(self.rows)))
        self.query = self.frames('query', 600); self.query_ann = self.local/'query-annotation.json'; self.query_ann.write_text(json.dumps(self.annotation(self.query, 'query', 'before')))
        self.config = dict(schemaVersion='motionbrain.grasp_observation_study.v1', frozenModel=dict(path=str(self.model), sha256=m.SHA(self.model.read_bytes())), manifestPath=str(self.manifest), manifestSha256=m.SHA(self.manifest.read_bytes()), sceneContext=self.context, expectedCameraBootId='a'*32, parameters=PARAMETERS, rejectedQuery=dict(source=dict(root=str(self.query), sessionId='query', label='before'), annotationPath=str(self.query_ann), annotationSha256=m.SHA(self.query_ann.read_bytes())), stationarySources=[])
        self.config_path = self.local/'config.json'; self.write_config()

    def frames(self, sid, captured):
        location = self.local/sid; location.mkdir()
        body = image()
        for label, offset in (('before', 0), ('after', 100)):
            stamp = captured+offset
            metadata = dict(sessionId=sid, hostClockId=sid, clockDomain='python_monotonic', requestStartedAt=1., receivedAt=2., persistedAt=3., sha256=m.SHA(body), headers={'X-MotionBrain-Camera-Boot-Id':'a'*32, 'X-MotionBrain-Captured-Us':str(stamp), 'X-MotionBrain-Capture-Requested-Us':str(stamp-1), 'X-MotionBrain-Frame-Id':'a'*32+':'+str(stamp), 'X-MotionBrain-Capture-Mode':'after_request', 'X-MotionBrain-Clock-Domain':'camera_boot_us', 'X-MotionBrain-Timestamp-Semantics':'first_dma_buffer'})
            (location/f'esp32-{label}.jpg').write_bytes(body); (location/f'esp32-{label}.json').write_text(json.dumps(metadata))
        return location

    def annotation(self, location, sid, label):
        frame, _, _ = read_frame(location, label, sid)
        return dict(frameId=frame['frameId'], pixelSha256=frame['pixelSha256'], accepted=True, method='analyst_points', inputFrameOnly=True, points=dict(object=[154,95], jawLeft=[95,112], jawRight=[215,112]))

    def write_config(self): self.config_path.write_text(json.dumps(self.config))

    def run_study(self):
        with patch.object(m, 'load_manifest', return_value=(self.rows, [])):
            return m.run_study(self.config_path, m.SHA(self.config_path.read_bytes()), self.local/'output', root=self.root)

    def test_saved_sources_are_audited_and_model_annotations_unchanged(self):
        before = [p.read_bytes() for p in (self.model, self.manifest, self.query_ann)]
        r = self.run_study()
        self.assertEqual((r['listedFrames'], r['extractedFrames'], r['failures']), (5,5,0))
        self.assertFalse(r['modelRefit']); self.assertFalse(r['annotationsRewritten']); self.assertFalse(r['controlEligible'])
        self.assertEqual(before, [p.read_bytes() for p in (self.model, self.manifest, self.query_ann)])

    def test_tampered_query_is_failure_in_denominator_not_silently_removed(self):
        (self.query/'esp32-before.jpg').write_bytes(b'bad')
        r = self.run_study()
        self.assertEqual((r['listedFrames'], r['extractedFrames'], r['failures']), (5,4,1))
        self.assertFalse(any(c['queryCheckAvailable'] for c in r['inputCoverage']))

    def test_changed_bytes_after_frame_validation_are_rejected(self):
        original = m._read
        def changed(location, name):
            if Path(location) == self.query and name.endswith('.jpg'):
                return image(left_occlusion=True)
            return original(location, name)
        with patch.object(m, '_read', side_effect=changed):
            r = self.run_study()
        self.assertEqual(r['failures'], 1)
        self.assertEqual(r['observations'][-1]['error'], 'diagnostic_verified_image_changed')
        self.assertNotIn('measurement', r['observations'][-1])

    def test_source_audit_rejection_keeps_unknown_command_denominator(self):
        for accepted in (self.rows[:1], []):
            with self.subTest(accepted=len(accepted)), tempfile.TemporaryDirectory(dir=self.local) as out:
                with patch.object(m, 'load_manifest', return_value=(accepted, ['source-rejected'])):
                    r = m.run_study(self.config_path, m.SHA(self.config_path.read_bytes()), Path(out)/'result', root=self.root)
                for field in ('inputCoverage', 'pairedInputCoverage'):
                    for variant in ('relativeStatePx', 'nearestIntegerStatePx'):
                        entries = [c for c in r[field] if c['representation'] == variant]
                        self.assertEqual(sum(c['listedTrainingInputs'] for c in entries), 2)
                        unknown = next(c for c in entries if c['unknownCommand'])
                        self.assertEqual(unknown['listedTrainingInputs'], 2-len(accepted))
                        self.assertFalse(unknown['queryCheckAvailable'])
                self.assertEqual(r['failures'], 2*(2-len(accepted)))

    def test_flagged_after_excludes_pair_without_changing_before_denominator(self):
        original = m.extract_points
        calls = []
        def flag_second(*args):
            calls.append(None)
            result = original(*args)
            if len(calls) == 2: result['qualityFlags'] = ['synthetic_after_clipped']
            return result
        with patch.object(m, 'extract_points', side_effect=flag_second):
            r = self.run_study()
        self.assertEqual(r['completeUnflaggedPairs'], 1)
        self.assertEqual([c['extractedTrainingInputs'] for c in r['inputCoverage']], [2,2])
        self.assertEqual([c['extractedTrainingInputs'] for c in r['pairedInputCoverage']], [1,1])
        self.assertTrue(all(c['listedTrainingInputs'] == 2 for c in r['pairedInputCoverage']))

    def test_parameter_origin_cannot_be_silently_retuned(self):
        origin = self.local/'origin.json'
        parameters = {k:v for k,v in PARAMETERS.items() if k != 'imageShape'}
        origin.write_text(json.dumps(dict(parameters=parameters)))
        self.config['parameterOrigin'] = dict(path=str(origin), sha256=m.SHA(origin.read_bytes()))
        self.config['parameters'] = dict(PARAMETERS, grayThreshold=91); self.write_config()
        with self.assertRaisesRegex(ValueError, 'parameters_differ_from_origin'): self.run_study()

    def test_reboot_and_training_query_overlap_refuse_coverage(self):
        self.config['expectedCameraBootId'] = 'b'*32; self.write_config()
        r = self.run_study(); self.assertEqual(r['failures'],5)
        self.config['rejectedQuery']['source']['sessionId'] = 'train-a'; self.write_config()
        with self.assertRaisesRegex(ValueError, 'new_query_input'): self.run_study()

    def test_changed_config_and_public_output_are_refused(self):
        with self.assertRaisesRegex(ValueError, 'config_changed'):
            m.run_study(self.config_path, '0'*64, self.local/'output', root=self.root)
        with patch.object(m, 'load_manifest', return_value=(self.rows, [])):
            with self.assertRaisesRegex(ValueError, 'private_diagnostic'):
                m.run_study(self.config_path, m.SHA(self.config_path.read_bytes()), self.root/'public', root=self.root)


if __name__ == '__main__': unittest.main()
