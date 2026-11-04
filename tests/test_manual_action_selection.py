import copy
import unittest

import numpy as np

from learning.manual_action_selection import rank_actions, rank_predictions
from learning.manual_collection_learning import fit, image_features
from learning.manual_multi_learning import as_multi


class ActionSelectionTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((8, 8, 4), dtype=np.uint8)
        self.a = dict(joint='shoulder', direction='down', percent=20, durationMs=350)
        self.b = dict(joint='shoulder', direction='down', percent=30, durationMs=500)
        x = np.array([image_features(self.image, self.a), image_features(self.image, self.b)])
        self.model = as_multi(fit(x, np.array([-1., -3.]), self.a, self.image.shape),
                              [{'inputs': {'command': c}} for c in [self.a, self.b]])

    def test_rank_and_original_model_unchanged(self):
        before = copy.deepcopy(self.model)
        result = rank_actions(self.model, self.image, -2., [self.a, self.b])
        self.assertEqual(len(result['candidates']), 3)
        self.assertFalse(result['controlEligible'])
        self.assertFalse(result['counterfactualOutcomesObserved'])
        self.assertEqual(self.model, before)
        self.assertEqual(result['selected'], result['candidates'][0])
        self.assertEqual(result['candidates'], sorted(result['candidates'], key=lambda x: abs(x['predictedResidualDeg'])))
        result['selected']['command']['percent'] = 99
        self.assertEqual(self.model, before)
        self.assertNotEqual(result['selected'], result['candidates'][0])

    def test_stop_tie_and_action_order(self):
        model = copy.deepcopy(self.model)
        model['coefficients'] = [0.] * len(model['coefficients'])
        model['intercept'] = 0.
        result = rank_actions(model, self.image, 0., [self.b, self.a])
        self.assertEqual([r['kind'] for r in result['candidates']], ['stop', 'action', 'action'])
        self.assertEqual([r['command'] for r in result['candidates'][1:]], [self.b, self.a])

    def test_frozen_baseline_predictions_and_rejections(self):
        values = [dict(command=self.a, predictedDeltaDeg=-1.),
                  dict(command=self.b, predictedDeltaDeg=-2.)]
        with self.assertRaises((ValueError, TypeError)):
            rank_predictions(values, 1.)
        before = copy.deepcopy(values)
        result = rank_predictions(values, -1.8)
        self.assertEqual(result['selected']['command'], self.b)
        self.assertEqual(values, before)
        for bad in ([values[0], values[0]],
                    [dict(command=self.a, predictedDeltaDeg=float('nan'))],
                    [dict(command=self.a, predictedDeltaDeg=True)],
                    [dict(command=self.a)],
                    [dict(command=dict(self.a, direction='up'), predictedDeltaDeg=-1.)]):
            with self.subTest(bad=bad), self.assertRaises((ValueError, TypeError)):
                rank_predictions(bad, -1.)

    def test_invalid_inputs_and_unsupported_action(self):
        for target in (float('nan'), float('inf'), 1., True, '1', None):
            with self.subTest(target=target), self.assertRaises((ValueError, TypeError)):
                rank_actions(self.model, self.image, target, [self.a])
        with self.assertRaises((ValueError, TypeError)):
            rank_actions(self.model, self.image, -1., [])
        with self.assertRaises((ValueError, TypeError)):
            rank_actions(self.model, self.image, -1., [dict(self.a, percent=21)])
        with self.assertRaises((ValueError, TypeError)):
            rank_actions(self.model, np.zeros((8, 8, 3), dtype=np.uint8), -1., [self.a])
        model = copy.deepcopy(self.model)
        model['controlEligible'] = True
        with self.assertRaises((ValueError, TypeError)):
            rank_actions(model, self.image, -1., [self.a])
        model = copy.deepcopy(self.model)
        model['intercept'] = float('nan')
        with self.assertRaises((ValueError, TypeError)):
            rank_actions(model, self.image, -1., [self.a])


if __name__ == '__main__':
    unittest.main()
