import unittest

from learning.dual_view_policy import VisualControl
from learning.shoulder_entry_measurement import (
    COMMAND, FOLLOWUP_COMMAND, FOLLOWUP_START_RANGE, LABEL, START_RANGE,
    measurement_vote, observe_entry, validate_plan,
)
from learning.open_side_approach import LEGACY_SIDE_VIEW
from tests.test_dual_view_policy import pair


class ShoulderEntryMeasurementTest(unittest.TestCase):
    def test_only_one_exact_short_step_from_measured_pose(self):
        profile = dict(command=dict(COMMAND), shoulderRangeDeg=list(START_RANGE))
        plan = dict(external=dict(viewConfigurationId=LEGACY_SIDE_VIEW), responsePolicy={},
                    shoulderEntryMeasurement=profile, maxActions=1, seconds=10, ackAlignedGripperCapture=True,
                    asyncActionLookup=True, refreshExpiredDispatch=True)
        validate_plan(plan)
        profile['command']['durationMs'] += 1
        with self.assertRaisesRegex(ValueError, 'invalid_shoulder_entry_measurement_plan'):
            validate_plan(plan)
        profile.update(command=dict(FOLLOWUP_COMMAND), shoulderRangeDeg=list(FOLLOWUP_START_RANGE))
        validate_plan(plan)
        profile['shoulderRangeDeg'] = list(START_RANGE)
        with self.assertRaisesRegex(ValueError, 'invalid_shoulder_entry_measurement_plan'):
            validate_plan(plan)

    def test_calibration_vote_keeps_original_and_stops_on_visual_loss(self):
        features = dict(sy=-40,nx=0,gap=130,leftMargin=25,rightMargin=23,objectWidth=30,objectHeight=40)
        for fields, reason, expected in ((features,'two_view_response_improvement',LABEL),
                                         ({},'two_view_landmarks_unavailable','hold'),
                                         (features,'learned_step_observed','hold'),
                                         (features,'outside_demonstrated_visual_extent','hold'),
                                         (dict(features, gap=151),'two_view_response_improvement','hold')):
            vote = dict(action='wrist/down', reason=reason, features=fields)
            measurement_vote(vote)
            self.assertEqual(vote['policyAction'], 'wrist/down')
            self.assertEqual(vote['action'], expected)

    def test_collects_four_new_pairs_and_requires_two_off_readings_in_range(self):
        for angle, command, close_votes, dissent, status in (
            (215.06, COMMAND, False, False, 'measured_in_close_range'),
            (215.86, COMMAND, False, False, 'outside_close_range'),
            (215.06, FOLLOWUP_COMMAND, False, False, 'close_visual_not_ready'),
            (215.06, FOLLOWUP_COMMAND, True, False, 'measured_in_close_range'),
            (215.06, FOLLOWUP_COMMAND, True, True, 'close_visual_not_ready'),
        ):
            with self.subTest(angle=angle, command=command, close_votes=close_votes, dissent=dissent):
                now = [2.0]
                last_box = [1]
                def sleep(dt):
                    now[0] += dt
                def off_pose():
                    return angle, now[0]
                def latest():
                    last_box[0] += 1
                    return dict(pair('hold',now[0]-.005,last_box[0]), features={'sy':-35},
                                policyAction='gripper/close' if close_votes else 'shoulder/down',
                                policyReason='preclose_dual_view_match' if close_votes else 'two_view_response_improvement',
                                closeDissentAt=2.63 if dissent and now[0] > 2.64 else 0.)
                control = VisualControl(latest, .69, .69, clock=lambda:now[0])
                zero_at = 100+command['durationMs']
                action = dict(request=dict(command),record=dict(
                              state='timed_out',reason='deadline_exceeded',startedAtMs=100,outputOff=True),
                              outputOffHostBounds={'latest':2.0},outputLog={'entries':[
                                  dict(motor=4,pwm=-75,atMs=101),dict(motor=4,pwm=0,atMs=zero_at)]})
                result = dict(executionState='known_terminal',commandId='measurement',action=action)
                report = observe_entry(control,off_pose,result,command,sleep=sleep)
                self.assertEqual(report['status'],status)
                self.assertEqual(len(report['pairs']),4)
                self.assertGreaterEqual(report['readings'][0]['offConfirmedAt'],2.6)
                late_cancel = dict(result, cancelAttempted=True, cancelConfirmed=False)
                self.assertEqual(observe_entry(control,off_pose,late_cancel,command,sleep=sleep)['status'],'unverified_output')
                action['outputLog']['entries'][1]['atMs'] = zero_at+2
                self.assertEqual(observe_entry(control,off_pose,result,command,sleep=sleep)['status'],'unverified_output')
