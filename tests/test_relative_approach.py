import unittest

from learning.relative_approach import RelativeApproachModel


SHOULDER = ('shoulder', 'down', 70, 420, 'finite_stop')
ELBOW = ('elbow', 'down', 50, 350, 'finite_stop')
GOAL = [(-.3, .3), (.12, .35), (-.07, .07), (.10, .35)]


def observation(frame, *, x=0, y=-50, near_y=55, center=686):
    return dict(frame=frame, state=[x, y, 0, near_y], gap=120, angle=206,
                box=[center-26, 655, 52, 175], upright=True)


def row(setup, command, *, side_delta=15, near_delta=-5, near_y=55):
    return dict(setup=setup, command=command, before=observation(setup+':1', near_y=near_y),
                after=[observation(f'{setup}:{i}', y=-50+side_delta, near_y=near_y+near_delta)
                       for i in (2, 3)])


class RelativeApproachTest(unittest.TestCase):
    def test_relocation_does_not_change_action_ranking(self):
        rows = [row(s, SHOULDER, near_delta=5) for s in ('a', 'b')]
        rows += [row(s, ELBOW) for s in ('c', 'd')]
        model = RelativeApproachModel(rows)
        first = model.select(observation('one'), GOAL, [SHOULDER, ELBOW])
        shifted = model.select(observation('two', center=784), GOAL, [SHOULDER, ELBOW])
        self.assertEqual(first, shifted)
        self.assertEqual(first['selected'], ELBOW)
        self.assertTrue(any(c['rejection'] == 'observed_response_worsens_goal_axis'
                            for c in first['candidates']))
        self.assertFalse(first['controlEligible'])

    def test_new_observation_changes_the_decision(self):
        model = RelativeApproachModel([row(s, ELBOW) for s in ('a', 'b')])
        result = model.select(observation('last'), GOAL, [ELBOW])
        self.assertEqual(result['selected'], ELBOW)
        result = model.select(observation('next', y=25, near_y=36), GOAL, [ELBOW])
        self.assertIsNone(result['selected'])
        self.assertEqual(result['reason'], 'within_goal_bands')

    def test_repetitions_and_other_stop_profile_cannot_supply_support(self):
        alternate = ELBOW[:-1]+('natural_expiry',)
        model = RelativeApproachModel([row('a', ELBOW)]*10 + [row('b', alternate)])
        result = model.select(observation('current'), GOAL, [ELBOW])
        self.assertIsNone(result['selected'])
        self.assertEqual(result['candidates'][0]['rejection'], 'insufficient_local_setup_support')
        self.assertEqual(model.replay()['matchedTransitions'], 0)

    def test_baseline_and_local_metrics_use_same_held_out_subset(self):
        model = RelativeApproachModel([row('a', ELBOW), row('b', ELBOW),
                                       row('c', ELBOW, near_y=-60, near_delta=5)])
        replay = model.replay()
        self.assertEqual(replay['matchedTransitions'], 2)
        metrics = replay['sameSubsetMeanAbsoluteError']
        self.assertAlmostEqual(metrics['predictedState'][3], 0)
        self.assertGreater(metrics['commandOnlyState'][3], metrics['predictedState'][3])
        self.assertIsNone(replay['decisions'][2]['predictedState'])

    def test_duplicate_frame_does_not_count_as_settled_pair(self):
        sample = row('a', ELBOW)
        sample['after'][1]['frame'] = sample['after'][0]['frame']
        with self.assertRaisesRegex(ValueError, 'distinct_before_and_two_settled'):
            RelativeApproachModel([sample])

    def test_shared_frame_cannot_leak_between_setup_groups(self):
        first = row('a', ELBOW)
        continuation = row('b', ELBOW)
        continuation['before'] = first['after'][-1]
        with self.assertRaisesRegex(ValueError, 'frame_shared_across_physical_setups'):
            RelativeApproachModel([first, continuation])
        continuation['setup'] = 'a'
        model = RelativeApproachModel([first, continuation])
        self.assertEqual(model.replay()['matchedTransitions'], 0)
