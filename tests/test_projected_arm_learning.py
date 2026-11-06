"""Synthetic contract checks; these do not establish physical motion accuracy."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning import projected_arm_learning as m
from learning.artifacts import StorageError
from learning.manual_collection_dataset import digest
from tools import replay_projected_arm_states as cli

C = dict(joint='elbow', direction='up', percent=40, durationMs=500)
CONTEXT = dict(physicalView='fixed-external', poseRegime='elbow-only',
    landmarks='distal-green-patch-center/proximal-green-patch-center/fixed-background',
    cameraIdentity=dict(deviceId='camera', viewId='fixed'), imageShape=[100, 100], stateFields=m.STATE)


def row(i, x, *, group='train', command=None, scene='object_present'):
    return dict(sessionId=str(i), captureGroup=group, context=copy.deepcopy(CONTEXT),
        command=copy.deepcopy(command or C), before=[x, 2., 10., 0.], after=[-x, 2., 10., 0.],
        episode=['boot', str(i)], frameIds=[f'{i}-pre', f'{i}-post'],
        commandAt=100.+i, sceneCondition=scene)


def annotation(frame='pre', pixel='pixels'):
    return dict(frameId=frame, pixelSha256=pixel, accepted=True, method='analyst_points',
        inputFrameOnly=True, points=dict(background=[10, 10], toolFront=[20, 20], toolRear=[30, 20]))


def source_and_annotation():
    frames = [dict(frameId=side, pixelSha256='pixels', width=100, height=100)
              for side in ('pre', 'post')]
    source = dict(sessionId='source', status='eligible', originalDatasetEligible=False,
        inputs=dict(command=C, image=frames[0]), targets=dict(image=frames[1]),
        evidence=dict(commandAt=123., controllerBootId='boot', commandId='command',
                      cameraIdentity=CONTEXT['cameraIdentity']))
    a = dict(sessionId='source', captureGroup='train', context={k: CONTEXT[k] for k in
        ('physicalView', 'poseRegime', 'landmarks')}, sceneCondition='object_present',
        before=annotation(), after=annotation('post'))
    return source, a


class ProjectedArmTests(unittest.TestCase):
    def setUp(self):
        self.rows = [row(0, -4.), row(1, 4.), row(2, 0.)]
        self.model = m.fit(self.rows)

    def test_state_can_outperform_command_baseline_without_using_after_or_scene_as_input(self):
        test = [row(3, 2., group='eval', scene='object_absent')]
        report = m.evaluate(self.model, test)
        self.assertLess(report['metrics']['stateRidge']['totalStateNormPx'],
                        report['metrics']['commandMean']['totalStateNormPx'])
        prediction = report['predictions'][0]['stateRidge']
        test[0]['after'] = [1000., -1000., 20., 0.]
        test[0]['sceneCondition'] = 'object_present'
        self.assertEqual(m.evaluate(self.model, test)['predictions'][0]['stateRidge'], prediction)
        self.assertEqual(report['sceneConditionShiftRows'], 1)
        self.assertEqual(report['sceneConditions']['object_absent'], 1)
        self.assertFalse(report['controlEligible'])

    def test_all_baselines_use_identical_supported_subset_and_preserve_rejection_count(self):
        test = [row(3, 2., group='eval'), row(4, 20., group='eval')]
        report = m.evaluate(self.model, test)
        self.assertEqual((report['listed'], report['evaluated'], len(report['rejected'])), (2, 1, 1))
        self.assertEqual(report['rejected'][0]['reason'], 'state_outside_training_range')
        self.assertEqual({metric['n'] for metric in report['metrics'].values()}, {1})
        self.assertEqual(report['metrics']['noChange']['linkVectorErrorPx'], 0.)

    def test_singleton_support_is_retained_but_never_promoted(self):
        unsupported = dict(C, durationMs=350)
        model = m.fit(self.rows + [row(9, 1., command=unsupported)])
        self.assertEqual(model['trainingTransitions'], 4)
        self.assertEqual(model['supportedTrainingTransitions'], 3)
        self.assertEqual(len(model['supportDeficits']), 1)
        with self.assertRaisesRegex(ValueError, 'without_two'):
            m.predict(model, [0., 2., 10., 0.], unsupported, CONTEXT)

    def test_frame_binding_manual_acceptance_and_marker_geometry(self):
        frame = dict(frameId='pre', pixelSha256='pixels', width=100, height=100)
        a = annotation()
        self.assertEqual(m.point_state(a, frame), [10., 10., 10., 0.])
        for change in (dict(accepted=False), dict(inputFrameOnly=False), dict(pixelSha256='other')):
            with self.subTest(change=change), self.assertRaises(ValueError):
                m.point_state(dict(a, **change), frame)
        for point in ([True, 1], [float('nan'), 1], [10**1000, 1], [100, 20]):
            bad = copy.deepcopy(a); bad['points']['toolFront'] = point
            with self.subTest(point=point), self.assertRaisesRegex(ValueError, 'outside_frame'):
                m.point_state(bad, frame)
        bad = copy.deepcopy(a); bad['points']['toolRear'] = bad['points']['toolFront']
        with self.assertRaisesRegex(ValueError, 'coincident'):
            m.point_state(bad, frame)

    def test_source_adapter_uses_audited_external_frame_and_no_sensor(self):
        source, a = source_and_annotation()
        source['targets']['sensorAfter'] = object()
        transition = m.annotated_transition(source, a)
        self.assertEqual(transition['before'], [10., 10., 10., 0.])
        self.assertEqual(transition['commandAt'], 123.)
        self.assertFalse(transition['originalDatasetEligible'])
        self.assertNotIn('sensorAfter', transition)
        source['targets']['image']['width'] = 200
        with self.assertRaisesRegex(ValueError, 'shape_changed'):
            m.annotated_transition(source, a)

    def test_duplicate_sources_episodes_frames_and_split_leakage_fail_closed(self):
        for field in ('sessionId', 'episode', 'frameIds'):
            bad = copy.deepcopy(self.rows); bad[1][field] = bad[0][field]
            with self.subTest(field=field), self.assertRaises(ValueError): m.fit(bad)
        for change in ('session', 'episode', 'frame', 'group'):
            test = row(3, 2., group='eval')
            if change == 'session': test['sessionId'] = self.rows[0]['sessionId']
            elif change == 'episode': test['episode'] = self.rows[0]['episode']
            elif change == 'frame': test['frameIds'][0] = self.rows[0]['frameIds'][0]
            else: test['captureGroup'] = 'train'
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, 'overlap'):
                m.evaluate(self.model, [test])

    def test_joint_camera_context_and_nonfinite_state_cannot_mix(self):
        for field in ('physicalView', 'cameraIdentity', 'imageShape', 'stateFields'):
            bad = copy.deepcopy(self.rows); bad[1]['context'][field] = 'changed'
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'context_changed'): m.fit(bad)
        bad = copy.deepcopy(self.rows); bad[1]['command']['joint'] = 'wrist'
        with self.assertRaisesRegex(ValueError, 'joint_contract_changed'): m.fit(bad)
        for state in ([True, 2., 10., 0.], [float('inf'), 2., 10., 0.], [0., 2., 0., 0.]):
            with self.subTest(state=state), self.assertRaises(ValueError):
                m.predict(self.model, state, C, CONTEXT)

    def test_changed_model_parameters_fail_validation(self):
        for key, value in [('alpha', True), ('count', 1), ('scale', [True, 1., 1., 1.]),
                           ('coefficient', [[float('nan')]*4]*4)]:
            bad = copy.deepcopy(self.model)
            if key == 'alpha': bad[key] = value
            else: bad['models'][0][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                m.predict(bad, [0., 2., 10., 0.], C, CONTEXT)

    def test_manifest_reaudits_sources_and_preserves_missing_or_wrong_frame_as_rejections(self):
        source, a = source_and_annotation()
        item = dict(sessionRoot='source', sessionSha256='source-hash', annotation=a)
        manifest = dict(schemaVersion=m.MANIFEST, records=[item], evaluationGroups=[])
        with patch.object(m, 'build_visual_session', return_value=source) as audit:
            rows, rejected = m.load_manifest(manifest, '/tmp')
        audit.assert_called_once_with(Path('/tmp/source').resolve(), 'source-hash')
        self.assertEqual((len(rows), rejected), (1, []))
        bad = copy.deepcopy(manifest); bad['records'][0]['annotation']['before']['frameId'] = 'old-snapshot'
        with patch.object(m, 'build_visual_session', return_value=source):
            rows, rejected = m.load_manifest(bad, '/tmp')
        self.assertFalse(rows); self.assertEqual(len(rejected), 1)
        with patch.object(m, 'build_visual_session', side_effect=StorageError('missing source')):
            rows, rejected = m.load_manifest(manifest, '/tmp')
        self.assertFalse(rows); self.assertEqual(len(rejected), 1)
        outside = copy.deepcopy(manifest); outside['records'][0]['sessionRoot'] = '../outside'
        with patch.object(m, 'build_visual_session') as audit:
            rows, rejected = m.load_manifest(outside, '/tmp')
        audit.assert_not_called(); self.assertIn('outside_manifest_root', rejected[0]['reason'])

    def test_legacy_object_manifest_is_not_silently_reinterpreted(self):
        with self.assertRaisesRegex(ValueError, 'manifest_required'):
            m.load_manifest(dict(schemaVersion='motionbrain.relative_action_manifest.v1', records=[]), '/tmp')

    def test_malformed_annotations_and_records_do_not_abort_valid_source_processing(self):
        source, a = source_and_annotation()
        valid = dict(sessionRoot='source', sessionSha256='source-hash', annotation=a)
        malformed = [dict(valid, annotation=value) for value in (None, [], 1, 'bad')]
        malformed += [None, [], 1, 'bad']
        manifest = dict(schemaVersion=m.MANIFEST, records=[*malformed[:4], valid, *malformed[4:]],
                        evaluationGroups=[])
        with patch.object(m, 'build_visual_session', return_value=source):
            rows, rejected = m.load_manifest(manifest, '/tmp')
        self.assertEqual([r['sessionId'] for r in rows], ['source'])
        self.assertEqual(len(rejected), len(malformed))
        self.assertTrue(all(r['sessionId'] is None and r['reason'] for r in rejected))

    def test_cli_report_preserves_bad_annotation_and_valid_training_source(self):
        source, a = source_and_annotation()
        valid = dict(sessionRoot='source', sessionSha256='source-hash', annotation=a)
        manifest = dict(schemaVersion=m.MANIFEST,
                        records=[dict(valid, annotation=None), valid], evaluationGroups=[])
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); private = root/'.local'; private.mkdir()
            manifest_path = private/'manifest.json'; manifest_path.write_text(json.dumps(manifest))
            with patch.object(m, 'build_visual_session', return_value=source):
                report = cli.run(manifest_path, private/'fit', root=root)
            model = json.loads((private/'fit/model.json').read_bytes())
            self.assertEqual(model['trainingSessionIds'], ['source'])
            self.assertEqual(len(report['sourceRejected']), 1)
            self.assertIsNone(report['sourceRejected'][0]['sessionId'])
            self.assertEqual(json.loads((private/'fit/report.json').read_bytes())['sourceRejected'],
                             report['sourceRejected'])

    def test_frozen_cli_checks_hash_and_never_fits_or_overwrites_model(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); private = root/'.local'; private.mkdir()
            model_path = private/'frozen.json'; raw = json.dumps(self.model).encode(); model_path.write_bytes(raw)
            manifest_path = private/'eval.json'; manifest_path.write_text(json.dumps(dict(
                schemaVersion=m.MANIFEST, records=[], evaluationGroups=['eval'])))
            test = [row(3, 2., group='eval', scene='object_absent')]
            with patch.object(m, 'load_manifest', return_value=(test, [])), \
                    patch.object(m, 'fit', side_effect=AssertionError('must not refit')):
                report = cli.run(manifest_path, private/'evaluation', frozen_model=model_path,
                                 model_sha256=digest(raw), root=root)
            self.assertFalse(report['modelRefit']); self.assertFalse(report['trainingPerformed'])
            self.assertEqual(model_path.read_bytes(), raw)
            self.assertFalse((private/'evaluation/model.json').exists())
            with patch.object(m, 'load_manifest', return_value=(test, [])), self.assertRaisesRegex(ValueError, 'hash_changed'):
                cli.run(manifest_path, private/'wrong-hash', frozen_model=model_path,
                        model_sha256='0'*64, root=root)
            self.assertFalse((private/'wrong-hash').exists())
            changed = copy.deepcopy(self.model); changed['codeSha256'] = '0'*64
            changed_raw = json.dumps(changed).encode(); model_path.write_bytes(changed_raw)
            with patch.object(m, 'load_manifest', return_value=(test, [])), \
                    self.assertRaisesRegex(ValueError, 'code_changed'):
                cli.run(manifest_path, private/'changed-code', frozen_model=model_path,
                        model_sha256=digest(changed_raw), root=root)
            self.assertFalse((private/'changed-code').exists())

    def test_cli_rejects_private_output_escape_and_training_rows_in_frozen_evaluation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); private = root/'.local'; private.mkdir()
            manifest = private/'manifest.json'; manifest.write_text(json.dumps(dict(evaluationGroups=['eval'])))
            with self.assertRaisesRegex(ValueError, 'private_output'):
                cli.run(manifest, root/'public-output', root=root)
            model = private/'model.json'; raw = json.dumps(self.model).encode(); model.write_bytes(raw)
            with patch.object(m, 'load_manifest', return_value=(self.rows, [])), \
                    self.assertRaisesRegex(ValueError, 'only_list_evaluation'):
                cli.run(manifest, private/'eval', frozen_model=model, model_sha256=digest(raw), root=root)

    def test_training_cli_persists_model_and_report_hash_without_consuming_evaluation_rows(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); private = root/'.local'; private.mkdir()
            manifest = private/'manifest.json'; manifest.write_text(json.dumps(dict(evaluationGroups=['eval'])))
            test = row(3, 2., group='eval', scene='object_absent')
            with patch.object(m, 'load_manifest', return_value=(self.rows + [test], [])):
                report = cli.run(manifest, private/'fit', root=root)
            saved = (private/'fit/model.json').read_bytes()
            self.assertEqual(report['modelSha256'], digest(saved))
            self.assertEqual(json.loads(saved)['trainingSessionIds'], ['0', '1', '2'])
            self.assertTrue(report['modelRefit']); self.assertTrue(report['trainingPerformed'])
            self.assertEqual(report['evaluated'], 1)
            self.assertEqual(json.loads((private/'fit/report.json').read_bytes())['modelSha256'], digest(saved))


if __name__ == '__main__':
    unittest.main()
