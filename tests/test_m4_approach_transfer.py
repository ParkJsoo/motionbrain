import copy
import json
from pathlib import Path
import tempfile
import unittest

from learning import m4_approach_transfer as m


CONTEXT = dict(view='new-registered-view', cameraBoot='current', otherJoints='fixed')
LIMITS = dict(sensorBandDeg=[218., 225.], maxObservedDeltaDeg=2.5, maxPredictionStepDeg=2.,
              maxStateStepPx=40., maxEvaluationErrorPx=6., minimumImprovementPx=3.)
PRIOR = dict(schemaVersion='motionbrain.manual_m4_precommitted_18.v1', controlEligible=False,
             models=dict(conditionMeans=[dict(command=c, count=3, meanDeg=d)
                                         for c, d in zip(m.COMMANDS, (-1., -1.5))]))


def row(index, before, angle, command_index=0, group='train'):
    delta = (-1., -1.5)[command_index]
    return dict(context=copy.deepcopy(CONTEXT), before=list(before),
                after=[before[0] - 2 * delta, before[1] - 4 * delta, before[2]],
                sensorBefore=angle, sensorAfter=angle + delta, command=copy.deepcopy(m.COMMANDS[command_index]),
                captureGroup=group, episode=['controller', str(index)],
                frameIds=[f'{index}-before', f'{index}-after'], sourceHash=f'{index+1:064x}')


class TransferTests(unittest.TestCase):
    def setUp(self):
        self.rows = [row(0, [10., 30., 100.], 224.), row(1, [12., 34., 100.], 223., 1)]
        self.model = m.fit(PRIOR, 'a' * 64, self.rows, CONTEXT, LIMITS)
        self.evaluation_rows = [row(2, [15., 40., 100.], 221.5, group='evaluation')]
        self.evaluation = m.evaluate(self.model, self.evaluation_rows)

    def test_transfer_reuses_prior_without_mutation_and_declares_extension(self):
        prior, rows = copy.deepcopy(PRIOR), copy.deepcopy(self.rows)
        model = m.fit(prior, 'a' * 64, rows, CONTEXT, LIMITS)
        self.assertEqual(prior, PRIOR)
        self.assertEqual(rows, self.rows)
        self.assertEqual(model['jacobian'], [-2., -4., 0.])
        self.assertTrue(model['rangeExtensionExperiment'])
        self.assertFalse(model['operatingLimitsAreTrainingSupport'])
        model['limits']['maxStateStepPx'] = 41
        with self.assertRaisesRegex(ValueError, 'model_changed'):
            m.predict(model, [17., 44., 100.], 220.5, m.COMMANDS[0], CONTEXT)

    def test_independent_evaluation_and_no_future_target_leakage(self):
        self.assertTrue(self.evaluation['evaluationPassed'])
        changed = copy.deepcopy(self.evaluation_rows); changed[0]['after'][0] += 100
        failed = m.evaluate(self.model, changed)
        self.assertFalse(failed['evaluationPassed'])
        self.assertEqual(failed['cases'][0]['prediction'], self.evaluation['cases'][0]['prediction'])
        for patch in [dict(captureGroup='train'), dict(frameIds=self.rows[0]['frameIds']),
                      dict(episode=self.rows[0]['episode']), dict(sourceHash=self.rows[0]['sourceHash'])]:
            bad = copy.deepcopy(self.evaluation_rows); bad[0].update(patch)
            with self.assertRaisesRegex(ValueError, 'overlap'):
                m.evaluate(self.model, bad)

    def test_selection_requires_exact_passed_evaluation_and_compares_hold(self):
        args = (self.model, [17., 44., 100.], 220.5, m.COMMANDS, [20., 50., 100.], CONTEXT)
        result = m.select(*args, self.evaluation, self.evaluation['evaluationSha256'])
        self.assertEqual(result['selected']['command'], m.COMMANDS[0])
        self.assertEqual(result['rejected'][0]['reason'], 'candidate_without_frozen_evaluation')
        self.assertFalse(result['selected']['baselines']['commandMean']['supported'])
        self.assertTrue(result['rangeExtensionExperiment'])
        both_rows = self.evaluation_rows + [row(3, [17., 44., 100.], 220.5, 1, group='evaluation')]
        both = m.evaluate(self.model, both_rows)
        self.assertEqual(m.select(*args, both, both['evaluationSha256'])['selected']['command'], m.COMMANDS[1])
        hold_args = (*args[:4], [17., 44., 100.], CONTEXT)
        self.assertEqual(m.select(*hold_args, self.evaluation, self.evaluation['evaluationSha256'])['selected']['kind'], 'hold')
        tampered = copy.deepcopy(self.evaluation); tampered['cases'][0]['absoluteErrorPx'][0] = 5
        with self.assertRaisesRegex(ValueError, 'passed_frozen'):
            m.select(*args, tampered, self.evaluation['evaluationSha256'])

    def test_context_limits_unsupported_candidates_and_unidentifiable_data(self):
        with self.assertRaisesRegex(ValueError, 'context_changed'):
            m.predict(self.model, [17., 44., 100.], 220.5, m.COMMANDS[0], {'view': 'other'})
        with self.assertRaisesRegex(ValueError, 'operating_band'):
            m.predict(self.model, [17., 44., 100.], 217., m.COMMANDS[0], CONTEXT)
        with self.assertRaisesRegex(ValueError, 'step_outside'):
            m.predict(self.model, [17., 44., 100.], 218.5, m.COMMANDS[0], CONTEXT)
        unsupported = dict(m.COMMANDS[0], percent=30)
        r = m.select(self.model, [17., 44., 100.], 220.5, [unsupported], [20., 50., 100.], CONTEXT,
                     self.evaluation, self.evaluation['evaluationSha256'])
        self.assertEqual(r['selected']['kind'], 'hold')
        self.assertEqual(r['rejected'][0]['reason'], 'unsupported_command')
        rows = copy.deepcopy(self.rows); rows[0]['sensorAfter'] = rows[0]['sensorBefore']
        with self.assertRaisesRegex(ValueError, 'not_identifiable'):
            m.fit(PRIOR, 'a' * 64, rows, CONTEXT, LIMITS)

    def test_prior_file_hash_and_calibration_consistency(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'prior.json'; path.write_text(json.dumps(PRIOR))
            with self.assertRaisesRegex(ValueError, 'hash_changed'):
                m.load_prior(path, 'b' * 64)
        bad = copy.deepcopy(self.rows); bad[1]['after'][0] = -10
        with self.assertRaisesRegex(ValueError, 'mapping_inconsistent'):
            m.fit(PRIOR, 'a' * 64, bad, CONTEXT, LIMITS)


if __name__ == '__main__':
    unittest.main()
