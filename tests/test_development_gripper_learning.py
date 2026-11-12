"""개발 관측의 명시적 오프라인 학습과 실물 모델 로더 차단."""
import copy
import json
import unittest
from unittest.mock import patch

from learning import relative_action_learning as relative
from learning import relative_model_bundle as bundle
from learning.pipeline_learning import run_learning
from learning.pipeline_observations import observe_transition
from tests.test_pipeline_learning import examples, manifest
import test_development_gripper_source as source_tests


def development_rows():
    rows = examples()
    for row in rows:
        row['context']['landmarks'] = 'development-gripper-visible-v1:'+'a'*64
        row['context']['stateFields'] = relative.state_fields(row['context'])
        row.update(developmentObservationOnly=True, recoveryDevelopmentOnly=True)
    return rows


def split(**kwargs):
    return manifest(allowDevelopmentGripperTraining=True,
                    allowRecoveredDevelopmentTraining=True, **kwargs)


class DevelopmentLearningTests(unittest.TestCase):
    def test_explicit_training_compares_baselines_but_never_loads_for_hardware(self):
        rows = development_rows()
        with self.assertRaisesRegex(ValueError, 'not_admitted'):
            run_learning(rows, manifest())
        result = run_learning(rows, split())
        partition = result['partitions'][0]; model = partition['model']
        self.assertTrue(result['trainingPerformed'])
        self.assertTrue(model['developmentObservationOnly'])
        self.assertEqual(partition['evaluation']['commonBaselineEvaluated'], 1)
        self.assertEqual(partition['evaluation']['kind'], 'developmentReplay')
        self.assertIsNone(partition['evaluation']['evaluationPassed'])
        self.assertFalse(model['controlEligible'])
        self.assertIn('corner_span', model['annotationSemantics'])
        self.assertEqual(bundle.validate_model(model, allow_development_observations=True), model)
        with self.assertRaisesRegex(ValueError, 'development_model_not_admitted'):
            bundle.validate_model(model)
        raw = json.dumps(model).encode()
        with patch('pathlib.Path.read_bytes', return_value=raw), self.assertRaisesRegex(ValueError, 'not_admitted'):
            bundle.load_frozen_model('synthetic', bundle.sha256(raw))

    def test_one_source_is_a_deficit_even_with_explicit_opt_in(self):
        rows = development_rows()[:1]
        result = run_learning(rows, split())
        self.assertEqual(result['counts']['training'], 1)
        self.assertFalse(result['trainingPerformed'])
        partition = result['partitions'][0]
        self.assertIsNone(partition['model'])
        self.assertEqual(partition['deficits'][0]['missing'], 1)
        empty = relative.fit(rows, allow_development_observations=True)
        self.assertEqual(empty['models'], [])
        with self.assertRaisesRegex(ValueError, 'development_model_not_admitted'):
            bundle.validate_model(empty)

    def test_opt_in_cannot_hide_missing_markers_or_overlap(self):
        for key in ('developmentObservationOnly', 'recoveryDevelopmentOnly'):
            rows = development_rows(); rows[0].pop(key)
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'marker_required'):
                run_learning(rows, split())
        with self.assertRaisesRegex(ValueError, 'boolean'):
            run_learning(development_rows(), manifest(allowDevelopmentGripperTraining=1))
        rows = development_rows(); rows[-1]['sourceHash'] = rows[0]['sourceHash']
        with self.assertRaisesRegex(ValueError, 'overlap_source'):
            run_learning(rows, split())
        result = run_learning(development_rows(), manifest(allowDevelopmentGripperTraining=True))
        self.assertEqual(result['counts']['training'], 0)
        self.assertEqual(len(result['excluded']), 2)

    def test_marker_mismatch_and_previous_model_default_loader_are_refused(self):
        rows = development_rows()
        model = relative.fit(rows[:2], allow_development_observations=True)
        model['developmentObservationOnly'] = False
        with self.assertRaisesRegex(ValueError, 'marker_mismatch'):
            bundle.validate_model(model, allow_development_observations=True)
        with self.assertRaisesRegex(ValueError, 'marker_mismatch'):
            relative.predict(model, rows[-1]['before'], rows[-1]['command'], rows[-1]['context'])
        with self.assertRaisesRegex(ValueError, 'not_admitted'):
            relative.fit(rows[:2])
        model = relative.fit(examples()[:2]); model['developmentObservationOnly'] = True
        with self.assertRaisesRegex(ValueError, 'marker_mismatch'):
            bundle.validate_model(model, allow_development_observations=True)

    def test_complete_pair_opt_in_preserves_prospective_command_and_originals(self):
        fixture = source_tests.DevelopmentSourceTests(); fixture.setUp(); self.addCleanup(fixture.doCleanups)
        fixture.write_frame('before', 18., 100)
        pair = fixture.root/'pair'; pair.mkdir()
        for side in ('before', 'after'):
            for suffix in ('json', 'jpg'):
                name = f'esp32-{side}.{suffix}'
                (pair/name).write_bytes((fixture.root/name).read_bytes())
        fixture.row['evidence'].update(controllerBootId='controller-boot', commandId='planned-one',
                                       dispatchHostBracket=[20.2, 20.3])
        spec = dict(kind='development_gripper', context=fixture.scene, captureGroup='train',
            gripperPairRoot='pair', observerBindingPath='binding.json',
            observerBindingSha256=fixture.binding_sha, cameraBootId=source_tests.BOOT)
        original = {f.name:f.read_bytes() for f in pair.iterdir()}
        self.assertIsNone(observe_transition(fixture.root, fixture.row, spec)['transition'])
        spec['learningUse'] = 'development_replay'
        result = observe_transition(fixture.root, fixture.row, spec)
        self.assertEqual(result['status'], 'eligible', result.get('reason'))
        row = result['transition']
        self.assertEqual(row['command'], fixture.command)
        self.assertEqual(row['episode'], ['controller-boot', 'planned-one'])
        self.assertTrue(row['developmentObservationOnly'])
        self.assertEqual({f.name:f.read_bytes() for f in pair.iterdir()}, original)
        fixture.row['evidence']['dispatchHostBracket'] = [24., 24.1]
        stale = observe_transition(fixture.root, fixture.row, spec)
        self.assertFalse(stale['trainingEligible'])
        self.assertEqual(stale['reason'], 'development_input_stale_at_dispatch')
        fixture.row['evidence']['dispatchHostBracket'] = [20.2, 20.3]
        with self.assertRaisesRegex(ValueError, 'not_admitted'):
            relative.fit([row])
        spec['learningUse'] = True
        self.assertEqual(observe_transition(fixture.root, fixture.row, spec)['status'], 'rejected')
        spec['learningUse'] = 'development_replay'
        with patch('learning.development_gripper_source.observe_saved_pair') as observe:
            observe.return_value = dict(status='partial_development', perFrame={}, reason='partial')
            result = observe_transition(fixture.root, fixture.row, spec)
        self.assertFalse(result['trainingEligible'])
        self.assertIsNone(result['transition'])


if __name__ == '__main__':
    unittest.main()
