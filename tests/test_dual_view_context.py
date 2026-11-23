import multiprocessing
import unittest
from unittest.mock import Mock

from learning.dual_view_capture import (_read_context, publish_active_context,
                                        publish_candidates, publish_close_context, response_vote)
from learning.dual_view_policy import VisualControl
from tests.test_dual_view_policy import command, pair


class DualViewContextTest(unittest.TestCase):
    def setUp(self):
        self.slot = multiprocessing.get_context('spawn').Array('c', 8192)
        self.boot = 'a'*32
        self.req = dict(command(1, -1), commandId='cid', bootId=self.boot)

    def test_active_observation_requires_both_sources_after_ack(self):
        vote = dict(action='gripper/close', decidedAt=1., features={'gap': 20})
        publish_active_context(self.slot, 's', self.boot, self.req, vote, 2.)
        value = _read_context(self.slot, 's', self.boot)
        with self.assertRaises(ValueError):
            _read_context(self.slot, 'other', self.boot)
        policy = Mock()
        policy.predict.side_effect = lambda *a, **kw: dict(action='hold', features={})
        for earliest, expected in ((1.9, None), (2., None), (2.01, 'cid')):
            row = response_vote(policy, None, None, earliest, value, None)
            self.assertEqual(row['activeCommandId'], expected)
            self.assertEqual(bool(policy.predict.call_args.kwargs['active_action']), expected is not None)
        publish_active_context(self.slot, 's', self.boot, None)
        self.assertIsNone(_read_context(self.slot, 's', self.boot))

    def test_model_gets_candidates_only_on_pair_after_verified_pose(self):
        publish_candidates(self.slot, 's', self.boot, ['shoulder/down'], 2.)
        candidates = _read_context(self.slot, 's', self.boot)
        model = Mock()
        self.assertEqual(response_vote(model, None, None, 2., None, None, candidates=candidates,
                                       require_candidates=True)['action'], 'hold')
        model.predict.assert_not_called()
        model.predict.return_value = dict(action='shoulder/down')
        vote = response_vote(model, None, None, 2.1, None, None, candidates=candidates, require_candidates=True)
        self.assertEqual(vote['action'], 'shoulder/down')
        self.assertEqual(model.predict.call_args.kwargs['eligible'], ['shoulder/down'])
        self.assertEqual(vote['candidateVerifiedOffAt'], 2.)
        model.predict.return_value = dict(action='wrist/down')
        active = dict(after=2.2, action={'label': 'shoulder/down'}, commandId='cid')
        running = response_vote(model, None, None, 2.3, active, None,
                                candidates=candidates, require_candidates=True)
        self.assertEqual(running['action'], 'wrist/down')
        self.assertNotIn('eligible', model.predict.call_args.kwargs)

    def test_candidate_epoch_cannot_mix_independent_pairs(self):
        old = pair('shoulder/down', 2.1, 1)
        new = dict(pair('shoulder/down', 2.2, 2), candidateVerifiedOffAt=2., previousPair=old)
        control = VisualControl(lambda: new, .6, .3, clock=lambda: 2.3)
        self.assertIsNone(control.admission(new, 2.))
        old['candidateVerifiedOffAt'] = 2.
        self.assertTrue(control.admission(new, 2.))

    def test_close_requires_real_negative_driver_write_and_post_off_pair(self):
        rec = {k:v for k,v in self.req.items() if k != 'pwm'}
        rec.update(ordinal=1, requestedPwm=-30, startedAtMs=1000, endedAtMs=1500, elapsedMs=500,
                   overrunMs=0, state='completed', reason='duration_elapsed', targetReached=False, outputOff=True)
        entries = [dict(motor=1, pwm=pwm, actionOrdinal=1, atMs=t) for pwm,t in [(-30,1010),(0,1500)]]
        log = dict(entries=entries, bootId=self.boot, evidence='driver_write', dropped=0)
        result = dict(executionState='known_terminal', commandId='cid', action=dict(request=self.req, record=rec,
                      outputOffHostBounds=dict(latest=3.), outputLog=log))
        publish_close_context(self.slot, 's', self.boot, result)
        value = _read_context(self.slot, 's', self.boot)
        model = Mock()
        self.assertEqual(response_vote(model, None, None, 3., None, value)['action'], 'hold')
        model.predict.assert_not_called()
        model.predict.return_value = dict(action='shoulder/up')
        row = response_vote(model, None, None, 3.1, None, value)
        self.assertEqual(row['closeCommandId'], 'cid')
        for pwm in (0, 1, -31):
            entries[0]['pwm'] = pwm
            with self.assertRaises(ValueError):
                publish_close_context(self.slot, 's', self.boot, result)
        publish_close_context(self.slot, 's', self.boot, None)
        self.assertIsNone(_read_context(self.slot, 's', self.boot))

    def test_admission_rejects_active_frames_and_cross_stage_pairs(self):
        sample = pair('shoulder/up', 2.2, 1)
        sample['previousPair'] = pair('shoulder/up', 2.1, 0)
        control = VisualControl(lambda: sample, .6, .3, clock=lambda: 2.3)
        self.assertTrue(control.admission(sample, 2.))
        sample['closeCommandId'] = 'cid'
        self.assertIsNone(control.admission(sample, 2.))
        sample['previousPair']['closeCommandId'] = 'cid'
        sample['previousPair']['activeCommandId'] = 'cid'
        self.assertIsNone(control.admission(sample, 2.))
