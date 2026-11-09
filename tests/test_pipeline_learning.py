import copy
import hashlib
import unittest

from learning import relative_action_learning as relative
from learning.pipeline_learning import canonical_bytes, digest, run_learning


def row(name, group, x=0., delta=1., *, view='camera-a', joint='gripper'):
    command = dict(joint=joint, direction='close' if joint == 'gripper' else 'down',
                   percent=40 if joint == 'gripper' else 20, durationMs=350)
    context = dict(physicalView=view, object='test-object', poseRegime='fixed-other-joints',
                   landmarks='analyst_points_fixture', cameraIdentity=dict(viewId=view),
                   imageShape=[240, 320], stateFields=relative.STATE.copy())
    return dict(sessionId=name, captureGroup=group, context=context, command=command,
                before=[x, 0., 100.], after=[x+delta, 0., 100.],
                episode=['controller-boot', name+'-command'], frameIds=[name+'-before', name+'-after'],
                sourceHash=hashlib.sha256(name.encode()).hexdigest())


def manifest(**kwargs):
    return dict(trainGroups=['train'], evaluationGroups=['evaluation'], **kwargs)


def examples():
    return [row('train-a', 'train', 0.), row('train-b', 'train', 10.),
            row('evaluation-a', 'evaluation', 5.)]


class PipelineLearningTests(unittest.TestCase):
    def test_crosssection_range_deficit_names_shaft_separation(self):
        rows = examples()
        for item in rows:
            item['context']['landmarks'] = 'registered-gripper-crosssection-v1:'+'a'*64
            item['context']['stateFields'] = relative.state_fields(item['context'])
        rows[-1]['before'][2] = 101.
        result = run_learning(rows, manifest())['partitions'][0]['evaluation']
        self.assertEqual(result['inSupportEvaluated'], 0)
        self.assertEqual(result['strictRejected'][0]['rangeDeficits'][0]['field'], 'shaftSeparationPx')

    def test_roundtrip_is_deterministic_and_preserves_inputs(self):
        rows = examples(); original = copy.deepcopy(rows)
        a = run_learning(rows, manifest()); b = run_learning(rows, manifest())
        self.assertEqual(a, b); self.assertEqual(rows, original)
        self.assertEqual(a['counts'], dict(listed=3, training=2, evaluation=1, excluded=0))
        partition = a['partitions'][0]
        self.assertEqual(partition['modelSha256'], hashlib.sha256(canonical_bytes(partition['model'])).hexdigest())
        self.assertEqual(partition['evaluation']['inSupportEvaluated'], 1)
        self.assertFalse(partition['controlEligible'])
        self.assertEqual(partition['evaluation']['kind'], 'developmentReplay')
        self.assertFalse(partition['evaluation']['temporalOrderVerified'])
        self.assertIsNone(partition['evaluation']['evaluationPassed'])

    def test_contexts_and_joints_are_separate_models(self):
        rows = examples() + [row('b1', 'train', 0., view='camera-b'), row('b2', 'train', 10., view='camera-b')]
        rows += [row('s1', 'train', 0., joint='shoulder'), row('s2', 'train', 10., joint='shoulder')]
        report = run_learning(rows, manifest())
        self.assertEqual(len(report['partitions']), 3)
        self.assertTrue(all(len(p['model']['models']) == 1 for p in report['partitions']))

    def test_dual_view_same_training_episode_is_allowed_but_cross_split_is_not(self):
        a = row('shared', 'train', 0.); b = copy.deepcopy(a)
        b['context']['physicalView'] = 'camera-b'; b['context']['cameraIdentity'] = dict(viewId='camera-b')
        result = run_learning([a, b], manifest())
        self.assertEqual(len(result['partitions']), 2)
        b['captureGroup'] = 'evaluation'
        with self.assertRaisesRegex(ValueError, 'training_evaluation_overlap_source'):
            run_learning([a, b], manifest())

    def test_each_cross_split_identity_cannot_leak(self):
        for field, expected in [('sourceHash', 'source'), ('sessionId', 'session'),
                                ('episode', 'episode'), ('frameIds', 'frame')]:
            rows = examples(); rows[-1][field] = copy.deepcopy(rows[0][field])
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'training_evaluation_overlap_'+expected):
                run_learning(rows, manifest())

    def test_duplicate_within_one_context_is_rejected(self):
        a = row('duplicate', 'train')
        with self.assertRaisesRegex(ValueError, 'duplicate_or_missing_session'):
            run_learning([a, copy.deepcopy(a)], manifest())

    def test_ood_stays_rejected_with_optional_offline_prediction_and_baseline_denominators(self):
        rows = examples(); rows[-1]['before'][0] = 11.; rows[-1]['after'][0] = 12.
        standard = run_learning(rows, manifest())['partitions'][0]['evaluation']
        self.assertEqual(standard['inSupportEvaluated'], 0)
        self.assertEqual(standard['oodDiagnostics'], [])
        self.assertEqual(standard['baselinesAllRows']['commandMean']['n'], 1)
        self.assertEqual(standard['baselinesAllRows']['noChange']['n'], 1)
        report = run_learning(rows, manifest(allowOodDiagnostics=True))['partitions'][0]['evaluation']
        self.assertEqual(report['inSupportEvaluated'], 0)
        self.assertEqual(report['strictRejected'][0]['reason'], 'state_outside_training_range')
        self.assertEqual(report['strictRejected'][0]['rangeDeficits'][0]['above'], 1.)
        self.assertEqual(report['oodDiagnostics'][0]['predictedState'], [12., 0., 100.])
        self.assertFalse(report['oodDiagnostics'][0]['eligibleForEvaluationPass'])
        self.assertIsNone(report['evaluationPassed'])

    def test_insufficient_command_does_not_block_another_context(self):
        rows = examples() + [row('insufficient', 'train', view='camera-b')]
        report = run_learning(rows, manifest())
        self.assertEqual(sorted(p['status'] for p in report['partitions']), ['trained', 'unavailable'])
        failed = next(p for p in report['partitions'] if p['status'] == 'unavailable')
        self.assertEqual(failed['deficits'][0]['missing'], 1)

    def test_recovered_rows_need_training_opt_in_and_evaluation_stays_development_replay(self):
        rows = examples(); rows[0]['recoveryDevelopmentOnly'] = True; rows[-1]['recoveryDevelopmentOnly'] = True
        report = run_learning(rows, manifest())
        self.assertEqual(report['counts']['excluded'], 1)
        allowed = run_learning(rows, manifest(allowRecoveredDevelopmentTraining=True))
        self.assertEqual(allowed['counts'], dict(listed=3, training=2, evaluation=1, excluded=0))
        evaluation=allowed['partitions'][0]['evaluation']
        self.assertEqual(evaluation['posthocEvaluationRows'], 1)
        self.assertFalse(evaluation['temporalOrderVerified'])
        self.assertIsNone(evaluation['evaluationPassed'])

    def test_previous_comparison_uses_same_disjoint_evaluation_cases(self):
        previous = relative.fit([row('old-a', 'old-training', 0., 3.), row('old-b', 'old-training', 10., 3.)])
        report = run_learning(examples(), manifest(), [previous])
        comparison = report['partitions'][0]['previousComparison'][0]
        self.assertEqual(comparison['status'], 'improved')
        self.assertEqual(comparison['commonEvaluated'], 1)
        self.assertFalse(comparison['automaticPromotion'])
        self.assertEqual(comparison['metrics']['previous']['meanStateErrorPx'], 2.)

    def test_previous_model_training_overlap_cannot_be_called_improvement(self):
        previous = relative.fit([row('old-a', 'evaluation', 0.), row('old-b', 'old-training', 10.)])
        report = run_learning(examples(), manifest(), [previous])
        comparison = report['partitions'][0]['previousComparison'][0]
        self.assertEqual(comparison['status'], 'unavailable')
        self.assertEqual(comparison['reason'], 'previous_training_evaluation_overlap')

    def test_previous_model_canonical_binding_is_verified(self):
        previous = relative.fit([row('old-a', 'old-training'), row('old-b', 'old-training', 10.)])
        with self.assertRaisesRegex(ValueError, 'previous_model_canonical_hash_changed'):
            run_learning(examples(), manifest(), [dict(model=previous, modelSha256='0'*64)])
        accepted = run_learning(examples(), manifest(), {'old':dict(model=previous, modelSha256=digest(previous))})
        self.assertEqual(accepted['partitions'][0]['previousComparison'][0]['status'], 'equal')

    def test_split_unsupported_and_unassigned_are_explicit(self):
        with self.assertRaisesRegex(ValueError, 'training_evaluation_group_overlap'):
            run_learning(examples(), dict(trainGroups=['same'], evaluationGroups=['same']))
        rows = examples(); rows[-1]['command']['percent'] = 35
        report = run_learning(rows+[row('unselected', 'another')], manifest())
        self.assertEqual(report['excluded'][0]['reason'], 'group_not_selected')
        self.assertEqual(report['partitions'][0]['evaluation']['strictRejected'][0]['reason'],
                         'command_without_two_training_transitions')

    def test_invalid_values_do_not_generate_a_model(self):
        rows = examples(); rows[0]['before'][0] = float('nan')
        with self.assertRaisesRegex(ValueError, 'finite_relative_state_required'):
            run_learning(rows, manifest())


if __name__ == '__main__':
    unittest.main()
