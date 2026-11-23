import unittest

from learning.dual_view_policy import VisualControl
from learning.shoulder_entry_measurement import measurement_vote, observe_entry, validate_plan, COMMAND, LABEL, START_RANGE
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
        for angle, status in ((215.06,'measured_in_close_range'), (215.86,'outside_close_range')):
            with self.subTest(angle=angle):
                now = [2.0]
                last_box = [1]
                def sleep(dt):
                    now[0] += dt
                def off_pose():
                    return angle, now[0]
                def latest():
                    last_box[0] += 1
                    return dict(pair('hold',now[0]-.005,last_box[0]), features={'sy':-35})
                control = VisualControl(latest, .69, .69, clock=lambda:now[0])
                action = dict(record=dict(state='timed_out',reason='deadline_exceeded',startedAtMs=100,outputOff=True),
                              outputOffHostBounds={'latest':2.0},outputLog={'entries':[
                                  dict(motor=4,pwm=-75,atMs=101),dict(motor=4,pwm=0,atMs=400)]})
                result = dict(executionState='known_terminal',commandId='measurement',action=action)
                report = observe_entry(control,off_pose,result,sleep=sleep)
                self.assertEqual(report['status'],status)
                self.assertEqual(len(report['pairs']),4)
                self.assertGreaterEqual(report['readings'][0]['offConfirmedAt'],2.6)
                late_cancel = dict(result, cancelAttempted=True, cancelConfirmed=False)
                self.assertEqual(observe_entry(control,off_pose,late_cancel,sleep=sleep)['status'],'unverified_output')
                action['outputLog']['entries'][1]['atMs'] = 402
                self.assertEqual(observe_entry(control,off_pose,result,sleep=sleep)['status'],'unverified_output')
