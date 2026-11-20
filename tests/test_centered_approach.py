import unittest

from learning.centered_approach import propose_centered_descent


def features(ny=-20):
    return dict(sy=-110, ny=ny, height=180, gap=120, leftMargin=30, rightMargin=34)


class CenteredApproachTests(unittest.TestCase):
    def test_relative_coordinates_and_independent_demonstrations(self):
        q = features()
        scaled = {k: v * 2 for k, v in q.items()}
        row = dict(episode=1, action='shoulder/down', features=q)
        def act(rows):
            return propose_centered_descent(scaled, rows)['action']
        self.assertEqual(act([row, row]), 'hold')
        other = dict(row, episode=2)
        self.assertEqual(act([row, other]), 'shoulder/down')
        self.assertEqual(act([row, other, dict(row, action='elbow/up')]), 'hold')
        self.assertEqual(act([row, other, dict(row, episode=3, action='gripper/close')]), 'hold')

    def test_requires_two_measurable_margins_and_supported_fore_aft(self):
        rows = [dict(episode=n, action='shoulder/down', features=features()) for n in (1, 2)]
        for q in (features(60), dict(features(), leftMargin=-1), dict(features(), rightMargin=None),
                  dict(features(), gap=0), dict(features(), height=float('nan'))):
            self.assertEqual(propose_centered_descent(q, rows)['action'], 'hold')

    def test_unidentified_episode_and_overflow_cannot_vote(self):
        row = dict(episode=1, action='shoulder/down', features=features())
        malformed = [None, {}, dict(row, episode=None), dict(row, episode=True)]
        self.assertEqual(propose_centered_descent(features(), [row, *malformed])['action'], 'hold')
        extreme = dict(features(), sy=1e308, height=1e-308)
        rows = [dict(row, features=extreme, episode=i) for i in (1, 2)]
        self.assertEqual(propose_centered_descent(extreme, rows)['action'], 'hold')
