"""Saved and fake-camera readiness diagnostics cannot dispatch a motor."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.artifacts import ArtifactStore
from learning.grasp_observation import GripperObservationSource
from learning.relative_action_learning import fit
from learning.relative_model_bundle import sha256
from tests import test_grasp_observation as fixtures
from tools import check_grasp_observation as cli


class CheckObservationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name); self.local = self.root/'.local'
        self.local.mkdir()
        self.clock = fixtures.Clock(); self.camera = fixtures.Camera(self.clock)
        store = ArtifactStore(self.local/'source')
        source = GripperObservationSource('http://camera.invalid', 'trial', fixtures.BOOT,
            fixtures.SCENE, store, opener=self.camera, clock=self.clock)
        source.start(deadline=60.)
        self.record = source.capture(deadline=60.)
        self.binding = store.root/'capture-0001/source-binding.json'
        self.binding_hash = sha256(self.binding.read_bytes())
        rows = [dict(sessionId=str(i), captureGroup='train', context=self.record['context'],
            command=fixtures.COMMAND, before=[0., -4., opening], after=[0., -4., opening-2.],
            episode=['boot', str(i)], frameIds=[f'{i}-pre', f'{i}-post']) for i, opening in enumerate((14., 18.))]
        self.model = fit(rows); self.model_path = self.local/'model.json'
        self.config_path = self.local/'config.json'
        self.config = dict(schemaVersion=cli.SCHEMA, sessionId='trial', cameraUrl='http://camera.invalid',
            expectedCameraBootId=fixtures.BOOT, sceneContext=fixtures.SCENE,
            captureTimeoutSeconds=2., modelBindings=[])
        self.save_model()
        frame = self.record['frame']
        self.annotation = dict(frameId=frame['frameId'], pixelSha256=frame['pixelSha256'],
            accepted=True, method='analyst_points', inputFrameOnly=True,
            points=dict(object=[16, 8], jawLeft=[8, 12], jawRight=[24, 12]))
        self.annotation_path = self.local/'annotation.json'
        self.annotation_path.write_text(json.dumps(self.annotation))

    def save_model(self):
        raw = json.dumps(self.model).encode(); self.model_path.write_bytes(raw)
        self.config['modelBindings'] = [dict(id='gripper', path=str(self.model_path), sha256=sha256(raw))]
        self.config_path.write_text(json.dumps(self.config))

    def replay(self, *, annotation=True):
        kwargs = dict(source_binding=self.binding, source_binding_sha256=self.binding_hash,
                      root=self.root, clock=self.clock)
        if annotation:
            kwargs.update(annotation_path=self.annotation_path,
                          annotation_sha256=sha256(self.annotation_path.read_bytes()))
        return cli.run(self.config_path, self.local/'out', **kwargs)

    def test_saved_source_query_is_diagnostic_and_keeps_original_times(self):
        self.clock.sleep(100.)
        report = self.replay()
        self.assertEqual(report['status'], 'checked')
        self.assertEqual(report['predictedCommands'], 1)
        self.assertEqual(report['state'], [0., -4., 16.])
        self.assertEqual(report['observation']['sourceEarliest'], self.record['sourceEarliest'])
        self.assertEqual(len(self.camera.calls), 1)
        self.assertEqual(report['newCameraCaptures'], 0)
        self.assertEqual(report['controllerWrites'], 0)
        self.assertFalse(report['liveControlReadinessVerified'])
        self.assertFalse(report['controlEligible'])

    def test_model_context_is_reported_instead_of_adapting_source(self):
        self.model['context']['poseRegime'] = 'different-pose'
        self.save_model()
        report = self.replay()
        self.assertEqual(report['models'][0]['status'], 'observation_context_changed')
        self.assertEqual(report['models'][0]['contextChangedFields'], ['poseRegime'])
        self.assertEqual(report['predictedCommands'], 0)
        self.assertEqual(report['observation']['context'], self.record['context'])

    def test_missing_or_unaccepted_points_never_generate_prediction(self):
        report = self.replay(annotation=False)
        self.assertEqual(report['models'][0]['status'], 'point_review_required')
        self.assertEqual(report['predictedCommands'], 0)
        self.assertIsNone(report['state'])
        self.annotation['accepted'] = False
        self.annotation_path.write_text(json.dumps(self.annotation))
        with tempfile.TemporaryDirectory() as another:
            kwargs = dict(source_binding=self.binding, source_binding_sha256=self.binding_hash,
                annotation_path=self.annotation_path, annotation_sha256=sha256(self.annotation_path.read_bytes()),
                root=self.root, clock=self.clock)
            report = cli.run(self.config_path, self.local/'unaccepted', **kwargs)
        self.assertEqual(report['predictedCommands'], 0)
        self.assertIn('unaccepted', report['stateStatus'])

    def test_original_jpeg_and_binding_tamper_are_rejected(self):
        (self.binding.parent/'esp32-frame.jpg').write_bytes(b'changed')
        report = self.replay()
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(report['errors'][0]['reason'], 'source_bytes_changed')
        self.assertEqual(len(self.camera.calls), 1)

    def test_annotation_hash_must_match_original_file(self):
        report = cli.run(self.config_path, self.local/'out', source_binding=self.binding,
            source_binding_sha256=self.binding_hash, annotation_path=self.annotation_path,
            annotation_sha256='0'*64, root=self.root, clock=self.clock)
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(report['errors'][0]['reason'], 'annotation_hash_changed')

    def test_bad_model_hash_is_refused_before_capture(self):
        self.config['modelBindings'][0]['sha256'] = '0'*64
        self.config_path.write_text(json.dumps(self.config))
        with patch.object(cli, 'GripperObservationSource', side_effect=AssertionError('must not capture')):
            with self.assertRaisesRegex(ValueError, 'model_hash_changed'):
                cli.run(self.config_path, self.local/'out', capture=True, root=self.root, clock=self.clock)
        self.assertFalse((self.local/'out').exists())

    def test_explicit_capture_makes_one_get_without_review_or_motor_access(self):
        def factory(url, sid, boot, scene, store, **kwargs):
            return GripperObservationSource(url, sid, boot, scene, store,
                opener=self.camera, **kwargs)
        count = len(self.camera.calls)
        with patch.object(cli, 'GripperObservationSource', side_effect=factory):
            report = cli.run(self.config_path, self.local/'capture-out', capture=True,
                             root=self.root, clock=self.clock)
        self.assertEqual(report['status'], 'checked')
        self.assertEqual(len(self.camera.calls), count+1)
        self.assertEqual(report['newCameraCaptures'], 1)
        self.assertEqual(report['predictedCommands'], 0)
        self.assertEqual(report['sourceFinalization']['status'], 'completed')

    def test_new_output_and_explicit_source_mode_are_required(self):
        self.replay()
        with self.assertRaises(Exception): self.replay()
        with self.assertRaisesRegex(ValueError, 'explicit_source_mode'):
            cli.run(self.config_path, self.local/'other', root=self.root)
        with self.assertRaisesRegex(ValueError, 'private_output'):
            cli.run(self.config_path, self.root/'public', capture=True, root=self.root)


if __name__ == '__main__':
    unittest.main()
