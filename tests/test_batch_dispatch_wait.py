import unittest
from unittest.mock import Mock
from tests.test_joint_batch import simulation


class DispatchWaitTests(unittest.TestCase):
    def runner(self):
        s=simulation();r=s.runner;r.scope['dispatchSettleWaitSeconds']=3
        r.active_port=Mock();return s,r

    def test_transient_settling_waits_without_command(self):
        s,r=self.runner();start=s.clock.now
        def fresh():
            if s.clock.now-start<.7:raise ValueError('invalid_or_unsettled_observation_window')
            return s.clock.now,{'angleDeg':5}
        r.fresh=fresh;at,state=r.dispatch_state()
        self.assertGreaterEqual(at-start,.7);self.assertLess(at-start,3)
        self.assertGreater(r.active_port.check_waiting.call_count,0);self.assertEqual(s.commands,[])
        self.assertEqual(r.active_port.check_waiting.call_args.args,(start+3,))
        self.assertEqual(len(s.store.data),1)

    def test_persistent_unsettled_has_fixed_deadline(self):
        s,r=self.runner();r.fresh=Mock(side_effect=ValueError('latest_observation_not_settled'));start=s.clock.now
        with self.assertRaisesRegex(ValueError,'dispatch_settle_deadline'):r.dispatch_state()
        self.assertLessEqual(s.clock.now-start,3.000001);self.assertEqual(s.commands,[])

    def test_fault_stale_off_and_queue_errors_not_retried(self):
        for reason in ('tracking_lost','latest_observation_stale','observation_precedes_off','batch_observation_outside_envelope'):
            s,r=self.runner();r.fresh=Mock(side_effect=ValueError(reason))
            with self.assertRaisesRegex(ValueError,reason):r.dispatch_state()
            r.active_port.check_waiting.assert_not_called()
        s,r=self.runner();r.fresh=Mock(side_effect=ValueError('invalid_or_unsettled_observation_window'))
        r.active_port.check_waiting.side_effect=ValueError('unexpected_output_before_dispatch')
        with self.assertRaisesRegex(ValueError,'unexpected_output'):r.dispatch_state()

    def test_slow_check_cannot_dispatch_after_deadline(self):
        s,r=self.runner();r.fresh=Mock(side_effect=ValueError('invalid_or_unsettled_observation_window'))
        r.active_port.check_waiting.side_effect=lambda deadline:setattr(s.clock,'now',deadline+.1)
        with self.assertRaisesRegex(ValueError,'dispatch_settle_deadline'):r.dispatch_state()
        self.assertEqual(s.commands,[])
