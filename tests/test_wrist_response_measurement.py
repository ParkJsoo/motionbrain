import unittest
from unittest.mock import Mock

from learning.dual_view_policy import VisualControl
from learning.open_side_approach import LEGACY_SIDE_VIEW
from learning.wrist_response_measurement import COMMAND, LABEL, measurement_vote, observe_after_off, profiles
from tests.test_action_execution import FakeClock
from tests.test_dual_view_policy import pair


class WristResponseMeasurementTest(unittest.TestCase):
    def test_command_is_a_single_shortened_version_of_the_learned_pulse(self):
        learned = dict(COMMAND, durationMs=800)
        plan = dict(wristResponseMeasurement=COMMAND, profiles={LABEL: {'command': learned}},
                    external={'viewConfigurationId': LEGACY_SIDE_VIEW}, maxActions=1, seconds=15,
                    useDevicePoseLimits=True, responsePolicy={}, ackAlignedGripperCapture=True,
                    asyncActionLookup=True, refreshExpiredDispatch=True)
        self.assertEqual(profiles(plan)[LABEL]['command'], COMMAND)
        learned['pwm'] = -180
        with self.assertRaisesRegex(ValueError, 'wrist_measurement_requires_matching_longer_response'):
            profiles(plan)

    def test_only_comparative_wrist_vote_and_visible_clearance_can_measure(self):
        features = dict(sy=-24, nx=2, gap=133, leftMargin=44, rightMargin=35,
                        objectWidth=40, objectHeight=60)
        for reason, sy, proposal, expected in (
            ('no_predicted_two_view_gain', -24, LABEL, LABEL),
            ('no_predicted_two_view_gain', -14, LABEL, 'hold'),
            ('no_predicted_two_view_gain', -24, 'elbow/down', 'hold'),
            ('outside_demonstrated_visual_extent', -24, LABEL, 'hold'),
        ):
            with self.subTest(reason=reason, sy=sy, proposal=proposal):
                policy = Mock()
                policy.decide.return_value = {'action': proposal}
                vote = dict(action='hold', reason=reason, features=dict(features, sy=sy))
                measurement_vote(vote, policy, [LABEL])
                self.assertEqual(vote['policyAction'], 'hold')
                self.assertEqual(vote['action'], expected)

    def test_expected_timer_stop_is_a_measurement_only_with_complete_fresh_features(self):
        for feature_loss, late_cancel, expected in (
            (False, False, 'wrist_response_measured'),
            (True, False, 'response_unverified'),
            (False, True, 'response_unverified'),
        ):
            with self.subTest(feature_loss=feature_loss, late_cancel=late_cancel):
                clock = FakeClock()
                result = dict(commandId='m2-test', cancelAttempted=late_cancel, cancelConfirmed=False, action=dict(
                    request=COMMAND, record=dict(state='timed_out', reason='deadline_exceeded',
                                                outputOff=True, startedAtMs=200),
                    outputOffHostBounds={'latest': 100.},
                    outputLog={'entries': [dict(motor=2, pwm=-80, atMs=250), dict(motor=2, pwm=0, atMs=601)]},
                ))

                def latest():
                    stamp = clock.monotonic()
                    f = None if feature_loss else dict(sx=20, sy=-19, nx=2, ny=43, gap=133)
                    return dict(pair('hold', stamp, stamp), features=f)

                control = VisualControl(latest, .69, .69, clock=clock.monotonic)
                report = observe_after_off(control, lambda:(213.92, clock.monotonic()), result, sleep=clock.sleep)
                self.assertTrue(report['outputVerified'])
                self.assertEqual(len(report['pairs']), 4)
                self.assertEqual(report['status'], expected)
