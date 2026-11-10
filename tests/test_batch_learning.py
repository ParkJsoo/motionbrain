import copy
import unittest

from learning.batch_learning import evaluate, fit, predict, prospective_evidence, model_sha256, validate_model


def row(index, *, group='train', size=6):
    return dict(sessionId='s'+str(index), episodeId='boot:'+str(index),
        sourceHash=format(index, '064x'), frameIds=[f'f{index}-before', f'f{index}-after'],
        captureGroup=group, joint='wrist',
        context=dict(viewId='view', poseRegime='local', object='none', load='empty', observerSha256='a'*64),
        stateFields=['x'+str(i) for i in range(size)],
        before=[float(index)]*size, after=[float(index)+2]*size,
        command=dict(joint='wrist', direction='up', percent=30, durationMs=200),
        beforePersistedAt=10., commandAt=12.)


class BatchLearningTests(unittest.TestCase):
    def setUp(self):
        self.training = [row(1), row(3)]
        self.model = fit(self.training)
        self.evaluation = row(2, group='test')

    def test_arbitrary_state_dimension_and_frozen_prediction(self):
        self.assertEqual(len(self.model['stateFields']), 6)
        saved = copy.deepcopy(self.model)
        prediction = predict(self.model, self.evaluation['before'], self.evaluation['command'], self.evaluation['context'])
        self.assertEqual(prediction, [4.]*6)
        self.assertEqual(saved, self.model)
        self.assertEqual(model_sha256(saved), model_sha256(self.model))

    def test_same_case_baselines_and_development_label(self):
        result = evaluate(self.model, [self.evaluation])
        self.assertEqual(result['kind'], 'developmentReplay')
        self.assertFalse(result['prospectiveEvidenceVerified'])
        self.assertEqual(result['metrics']['stateRidge']['n'], 1)
        self.assertEqual(result['metrics']['commandMean']['meanStateError'], 0)
        self.assertGreater(result['metrics']['noChange']['meanStateError'], 0)
        self.assertFalse(result['controlEligible'])

    def test_prospective_frozen_evaluation_binds_prediction_and_timing(self):
        proof = prospective_evidence(self.model, self.evaluation, 11.)
        result = evaluate(self.model, [self.evaluation], {'s2':proof})
        self.assertEqual(result['kind'], 'prospectiveFrozenEvaluation')
        self.assertTrue(result['prospectiveEvidenceVerified'])
        self.assertEqual(result['evaluated'], 1)

    def test_target_assisted_or_tampered_prediction_is_not_verified(self):
        proof = prospective_evidence(self.model, self.evaluation, 11.)
        for field, value in [('preparedAt', 13.), ('prediction', [100.]*6),
                             ('modelSha256', 'bad'), ('beforeStateSha256', 'bad'),
                             ('commandSha256', 'bad')]:
            changed = copy.deepcopy(proof); changed[field] = value
            result = evaluate(self.model, [self.evaluation], {'s2':changed})
            with self.subTest(field=field):
                self.assertFalse(result['prospectiveEvidenceVerified'])
                self.assertEqual(result['evaluated'], 0)

    def test_all_identity_and_group_leakage_rejected(self):
        for key in ('sessionId', 'episodeId', 'sourceHash', 'frameIds', 'captureGroup'):
            changed = copy.deepcopy(self.evaluation); changed[key] = copy.deepcopy(self.training[0][key])
            with self.subTest(key=key):
                with self.assertRaisesRegex(ValueError, 'overlap'):
                    evaluate(self.model, [changed])

    def test_duplicate_raw_source_with_new_session_rejected(self):
        changed = row(5); changed['sourceHash'] = self.training[0]['sourceHash']
        with self.assertRaisesRegex(ValueError, 'duplicate_training_identity:sourceHash'):
            fit(self.training+[changed])

    def test_changed_context_or_state_semantics_not_merged(self):
        for key in ('context', 'stateFields'):
            changed = row(5)
            if key == 'context': changed[key]['load'] = 'held'
            else: changed[key][-1] = 'different'
            with self.subTest(key=key):
                with self.assertRaisesRegex(ValueError, 'learning_context_changed'):
                    fit(self.training+[changed])

    def test_input_range_rejection_not_clamped(self):
        changed = copy.deepcopy(self.evaluation); changed['before'][0] = 100
        result = evaluate(self.model, [changed])
        self.assertEqual(result['evaluated'], 0)
        self.assertEqual(result['rejected'][0]['reason'], 'state_outside_training_range')

    def test_missing_command_support_is_explicit_deficit(self):
        model = fit([row(1)])
        self.assertFalse(model['models'])
        self.assertEqual(len(model['deficits']), 1)
        self.assertEqual(evaluate(model, [self.evaluation])['rejected'][0]['reason'], 'unsupported_command')

    def test_different_output_dimension_rejected(self):
        changed = row(5); changed['after'] = [1.]
        with self.assertRaisesRegex(ValueError, 'finite_state_dimension_required'):
            fit(self.training+[changed])

    def test_malformed_frozen_arrays_never_broadcast(self):
        key = next(iter(self.model['models']))
        for field, value in [('coefficient', [[0.]*6]), ('coefficient', [[0.]]*6),
                             ('deltaMean', [1.]), ('minimum', [1.]),
                             ('scale', [0.]*6), ('mean', [float('nan')]*6),
                             ('maximum', [float('inf')]*6), ('minimum', [10.]*6)]:
            broken = copy.deepcopy(self.model); broken['models'][key][field] = value
            with self.subTest(field=field, value=value):
                with self.assertRaises(ValueError):
                    predict(broken, self.evaluation['before'], self.evaluation['command'], self.evaluation['context'])
                with self.assertRaises(ValueError):
                    validate_model(broken)

    def test_observer_contract_hash_required(self):
        broken = copy.deepcopy(self.training)
        for r in broken: del r['context']['observerSha256']
        with self.assertRaisesRegex(ValueError, 'learning_observer_context_required'):
            fit(broken)


if __name__ == '__main__':
    unittest.main()
