import copy
import unittest

from learning import m4_approach_transfer as transfer
from learning import m4_empirical_approach as m


CONTEXT = dict(view='registered', otherJoints='fixed')
LIMITS = dict(sensorBandDeg=[127.58, 296.12], maxObservedDeltaDeg=2.5,
              maxPredictionStepDeg=2., maxStateStepPx=40., maxEvaluationErrorPx=6., minimumImprovementPx=3.)
PRIOR = dict(schemaVersion='motionbrain.manual_m4_precommitted_18.v1', controlEligible=False,
             models=dict(conditionMeans=[dict(command=c, count=3, meanDeg=d)
                                         for c, d in zip(m.COMMANDS, (-1., -1.5))]))


def row(index, command=1, angle=-1.5, camera=-15., group='train'):
    return dict(context=copy.deepcopy(CONTEXT), command=copy.deepcopy(m.COMMANDS[command]),
        before=[0., 100., 51.], after=[0., 100. + camera, 51.], sensorBefore=224., sensorAfter=224. + angle,
        captureGroup=group, episode=['boot', str(index)], frameIds=[f'{index}-before', f'{index}-after'],
        sourceHash=f'{index + 1:064x}')


class EmpiricalTests(unittest.TestCase):
    def setUp(self):
        self.rows = [row(0, 0, -1., -10.), row(1), row(2, angle=-.8, camera=-8.)]
        self.parent = transfer.fit(PRIOR, 'a' * 64, self.rows[:2], CONTEXT, LIMITS)
        self.model = m.fit(self.rows, self.parent)
        self.eval_rows = [row(10, group='frozen-evaluation')]
        self.evaluation = m.evaluate(self.model, self.eval_rows)

    def select(self, state, target, model=None, evaluation=None):
        model, evaluation = model or self.model, evaluation or self.evaluation
        return m.select(model, state, 220., m.COMMANDS, target, CONTEXT, evaluation, evaluation['evaluationSha256'])

    def test_immutable_parent_paired_predictions_and_explicit_uncertainty(self):
        parent, rows = copy.deepcopy(self.parent), copy.deepcopy(self.rows)
        model = m.fit(rows, parent)
        self.assertEqual(parent, self.parent); self.assertEqual(rows, self.rows)
        p = m.predict(model, [0., 20., 51.], 220., m.COMMANDS[1], CONTEXT)
        self.assertEqual([(x['predictedSensorDeltaDeg'], x['predictedState'][1]) for x in p['observedOutcomes']],
                         [(-1.5, 5.), (self.rows[2]['sensorAfter'] - 224., 12.)])
        self.assertTrue(p['baselines']['parentTransferPrediction']['supported'])
        self.assertFalse(p['calibratedUncertainty']); self.assertFalse(p['futureCoverageGuaranteed'])
        self.assertEqual(model['commandSupport'][0]['count'], 1)
        model['outcomes'][0]['stateDeltaPx'][1] = 0
        with self.assertRaisesRegex(ValueError, 'empirical_model_changed'):
            m.predict(model, [0., 20., 51.], 220., m.COMMANDS[1], CONTEXT)

    def test_matching_requires_one_joint_camera_sensor_sample(self):
        # Separate camera and sensor extrema match, but no paired sample matches both.
        hybrid = row(11, angle=-.8, camera=-15., group='application')
        report = m.application_check(self.model, hybrid)
        self.assertFalse(report['passed']); self.assertEqual(report['matchedPairedSamples'], [])
        self.assertTrue(report['insideObservedRange']['camera']); self.assertTrue(report['insideObservedRange']['sensor'])
        good = m.application_check(self.model, row(12, angle=-.8, camera=-8., group='application'))
        self.assertTrue(good['passed']); self.assertFalse(good['addedToFrozenEvaluation'])
        wide = m.fit(self.rows + [row(3, angle=-2., camera=-30.)], self.parent)
        covered = m.application_check(wide, row(14, angle=-.8, camera=-8., group='application'))
        self.assertTrue(covered['passed']); self.assertGreater(max(covered['meanAbsoluteErrorPx']), 6.)
        outside = m.application_check(self.model, row(15, camera=-19., group='application'))
        self.assertTrue(outside['passed']); self.assertFalse(outside['insideObservedRange']['camera'])
        self.assertFalse(outside['futureCoverageGuaranteed'])
        unchanged = copy.deepcopy(self.evaluation)
        m.application_check(self.model, row(13, group='application'))
        self.assertEqual(self.evaluation, unchanged)

    def test_worst_outcome_controls_hold_and_unevaluated_commands_are_rejected(self):
        held = self.select([0., 20., 51.], [0., 12., 51.])
        self.assertEqual(held['selected']['kind'], 'hold')
        action = held['candidates'][1]
        self.assertGreaterEqual(held['candidates'][0]['errorPx'] - action['meanErrorPx'], 3.)
        self.assertLess(action['improvementPx'], 3.)
        chosen = self.select([0., 20., 51.], [0., 0., 51.])
        self.assertEqual(chosen['selected']['command'], m.COMMANDS[1])
        self.assertEqual(chosen['rejected'][0]['reason'], 'insufficient_empirical_command_support')
        model = m.fit(self.rows + [row(3, 0, -1.1, -11.)], self.parent)
        evaluation = m.evaluate(model, self.eval_rows)
        result = self.select([0., 20., 51.], [0., 0., 51.], model, evaluation)
        self.assertEqual(result['rejected'][0]['reason'], 'candidate_without_frozen_evaluation')

    def test_frozen_evaluation_rejects_overlap_tampering_and_future_leakage(self):
        self.assertTrue(self.evaluation['evaluationPassed'])
        for patch in (dict(captureGroup='train'), dict(episode=self.rows[0]['episode']),
                      dict(frameIds=self.rows[0]['frameIds']), dict(sourceHash=self.rows[0]['sourceHash'])):
            bad = copy.deepcopy(self.eval_rows); bad[0].update(patch)
            with self.assertRaisesRegex(ValueError, 'overlap'):
                m.evaluate(self.model, bad)
        changed = copy.deepcopy(self.eval_rows); changed[0]['after'][1] += 50
        failed = m.evaluate(self.model, changed)
        self.assertFalse(failed['evaluationPassed'])
        self.assertEqual(failed['cases'][0]['prediction'], self.evaluation['cases'][0]['prediction'])
        bad = copy.deepcopy(self.evaluation); bad['cases'][0]['passed'] = False
        with self.assertRaisesRegex(ValueError, 'passed_frozen_evaluation'):
            self.select([0., 20., 51.], [0., 0., 51.], evaluation=bad)

    def test_existing_effect_context_and_all_outcome_limits_remain(self):
        for patch in (dict(sensorAfter=224.), dict(after=[0., 100., 51.]), dict(context={'view': 'other'})):
            bad = copy.deepcopy(self.rows); bad[0].update(patch)
            with self.assertRaises(ValueError): m.fit(bad, self.parent)
        with self.assertRaisesRegex(ValueError, 'operating_band'):
            m.predict(self.model, [0., 20., 51.], 127., m.COMMANDS[1], CONTEXT)
        with self.assertRaisesRegex(ValueError, 'step_outside_limits'):
            m.predict(self.model, [0., 20., 51.], 128., m.COMMANDS[1], CONTEXT)
        bad = row(14, angle=.5, camera=-15., group='application')
        self.assertFalse(m.application_check(self.model, bad)['passed'])
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            m.fit(self.rows + [copy.deepcopy(self.rows[0])], self.parent)


if __name__ == '__main__':
    unittest.main()
