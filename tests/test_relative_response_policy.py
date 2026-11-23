import unittest

import cv2
import numpy as np

from learning.dual_view_policy import ACTIONS
from learning.near_gripper_tip_observer import shallow_color_core_height
from learning.relative_response_policy import SCHEMA, RelativeResponsePolicy


def features(sx=4., sy=-120., nx=0., ny=-18., gap=120.):
    return dict(sx=sx, sy=sy, nx=nx, ny=ny, gap=gap, width=60., height=180.,
                distalDepth=None, leftMargin=25, rightMargin=25, nearTop=8, objectWidth=45, objectHeight=45)


def artifact():
    responses = {a: dict(delta=[0]*5, stepFraction=1., progressChannels=[], sources=['demo']) for a in ACTIONS}
    responses['wrist/down'].update(delta=[0,24,0,10,0], stepFraction=.25, progressChannels=[1,3])
    responses['base/left'].update(delta=[0,0,7,0,0], progressChannels=[2])
    responses['base/right'].update(delta=[0,0,-8,0,0], progressChannels=[2])
    return dict(schema=SCHEMA, responses=responses, closeGoals=[
        dict(point=[.1,.20,0,.15], source='pose1', profile='observed_shallow')], heldGoals=[
        dict(point=[0,.3,0,.1], source='verified lift', gap=60)],
        observationBounds=[[-2,-3,-.5,-1],[2,1,.5,1]], centeringBandPx=4.)


def fixture():
    return RelativeResponsePolicy(artifact())


class RelativeResponseTest(unittest.TestCase):
    def test_goal_set_compares_best_cost_before_and_after(self):
        model = artifact()
        model['closeGoals'] = [
            dict(point=[.38,.17,-.017,.268], source='shallow', profile='observed_shallow'),
            dict(point=[.53,.27,0,.307], source='deep', profile='observed_deep'),
        ]
        model['responses']['wrist/down']['delta'] = [6,30,0,12,0]
        policy = RelativeResponsePolicy(model)
        f = dict(features(sx=30, sy=-24, nx=2, ny=40, gap=133), width=52, height=175)
        self.assertEqual(policy.decide(f)['action'], 'hold')
        self.assertEqual(policy.decide(f, compare_goal_set=True)['action'], 'wrist/down')
        self.assertEqual(policy.decide(f, eligible=['shoulder/up'], compare_goal_set=True)['action'], 'hold')

    def test_close_coordinate_approach_prefers_progress_to_both_view_windows(self):
        data = artifact()
        data['responses']['wrist/down']['delta'] = [6, 29, 0, 12, 0]
        data['responses']['elbow/down']['delta'] = [0, 22, 0, 1, 0]
        data['responses']['shoulder/down']['delta'] = [0, 120, 0, 0, 0]
        policy = RelativeResponsePolicy(data, approach_evaluation='close_coordinates')
        f = features(sx=20, sy=-18, nx=2, ny=42, gap=133)
        vote = policy.decide(f)
        self.assertEqual(vote['action'], 'elbow/down')
        self.assertEqual(vote['closeCoordinateCandidates']['elbow/down'],
                         {'profile': 'observed_shallow', 'side': 16., 'near': 0.})
        self.assertNotIn('shoulder/down', vote['closeCoordinateCandidates'])
        self.assertEqual(policy.decide(f, eligible=['wrist/down'])['action'], 'wrist/down')
        self.assertEqual(policy.decide(f, eligible=['shoulder/down'])['action'], 'hold')

    def test_predicted_close_coordinates_do_not_authorize_closing(self):
        policy = RelativeResponsePolicy(artifact(), approach_evaluation='close_coordinates')
        f = features(sx=6, sy=32, nx=0, ny=18)
        self.assertEqual(policy.decide(f)['action'], 'hold')
        f['distalDepth'] = [55, 55, 55]
        f['ny'] = 18.
        self.assertEqual(policy.decide(f)['action'], 'gripper/close')
        self.assertEqual(policy.decide(f, eligible=['wrist/down'])['reason'], 'no_executable_close')

    def test_shallow_color_core_checks_same_top_and_width_across_thresholds(self):
        field = np.zeros((240, 320, 3), np.uint8)
        field[91:134, 138:179] = [170, 145, 180]
        field[134:144, 138:179] = [170, 121, 170]
        image = cv2.cvtColor(field, cv2.COLOR_HSV2BGR)
        self.assertEqual(shallow_color_core_height(image, (138, 91, 41, 53)), 43)
        self.assertIsNone(shallow_color_core_height(image, (138, 87, 41, 53)))
        self.assertIsNone(shallow_color_core_height(image, (138, 91, 41, 120)))
        field[91:134, 185:226] = [170, 145, 180]
        image = cv2.cvtColor(field, cv2.COLOR_HSV2BGR)
        self.assertIsNone(shallow_color_core_height(image, (138, 91, 41, 53)))

    def test_color_core_is_opt_in_and_preserves_other_close_checks(self):
        data = artifact()
        baseline = RelativeResponsePolicy(data)
        verification = RelativeResponsePolicy(data, check_shallow_color_core=True)
        f = dict(features(sx=6, sy=32, nx=0, ny=18), distalDepth=[55, 55, 55],
                 objectHeight=53, shallowColorCoreHeight=43)
        self.assertNotEqual(baseline.decide(f)['action'], 'gripper/close')
        self.assertEqual(verification.decide(f)['action'], 'gripper/close')
        self.assertNotEqual(verification.decide(dict(f, objectHeight=120))['action'], 'gripper/close')
        self.assertNotEqual(verification.decide(dict(f, leftMargin=18))['action'], 'gripper/close')
        self.assertNotEqual(verification.decide(dict(f, distalDepth=None))['action'], 'gripper/close')

    def test_ranks_executable_approach_instead_of_replacing_the_model_choice(self):
        model = artifact()
        model['responses']['shoulder/down']['delta'] = [0, 10, 0, 4, 0]
        policy = RelativeResponsePolicy(model)
        self.assertEqual(policy.decide(features())['action'], 'wrist/down')
        self.assertEqual(policy.decide(features(), eligible=['shoulder/down'])['action'], 'shoulder/down')
        vote = policy.decide(features(), eligible=['shoulder/up'])
        self.assertEqual(vote['action'], 'hold')
        self.assertEqual(vote['reason'], 'no_executable_approach')

    def test_invalid_progress_and_goals_fail_before_observation(self):
        for channels in [[-1], [5], [1, 1], [0], [True]]:
            with self.subTest(channels=channels):
                data = artifact()
                data['responses']['wrist/down']['progressChannels'] = channels
                with self.assertRaises(ValueError):
                    RelativeResponsePolicy(data)
        for group, key, value in [('closeGoals', 'point', [1, 2, 3]), ('heldGoals', 'gap', float('nan')),
                                  ('heldGoals', 'point', [1, 2, 3, float('inf')]), ('closeGoals','profile','other')]:
            with self.subTest(group=group, key=key):
                data = artifact()
                data[group][0][key] = value
                with self.assertRaises(ValueError):
                    RelativeResponsePolicy(data)

    def test_fresh_response_in_both_views_changes_active_action(self):
        p=fixture()
        start=features()
        self.assertEqual(p.decide(start)['action'], 'wrist/down')
        ctx=dict(label='wrist/down',startFeatures=start)
        self.assertEqual(p.decide(features(sy=-113),active_action=ctx)['action'],'wrist/down')
        self.assertEqual(p.decide(features(sy=-113,ny=-15),active_action=ctx)['reason'],'learned_step_observed')

    def test_active_close_does_not_reapply_open_jaw_admission(self):
        policy = RelativeResponsePolicy(artifact(), check_shallow_color_core=True)
        start = dict(features(sx=6, sy=32, ny=18), distalDepth=[55, 55, 55],
                     sideBox=[783, 655, 60, 180])
        self.assertEqual(policy.decide(start)['action'], 'gripper/close')
        ctx = dict(label='gripper/close', startFeatures=start)
        closing = dict(start, ny=8, nearTop=-13, gap=90)
        self.assertEqual(policy.decide(closing)['reason'], 'unsupported_open_jaw_gap')
        self.assertEqual(policy.decide(closing, active_action=ctx)['action'], 'gripper/close')
        held = dict(start, sx=0, sy=54, ny=6, gap=60, leftMargin=0, rightMargin=0)
        cases = [
            (held, 'close_stop_held_geometry_observed'),
            (dict(closing, nx=9), 'close_stop_alignment_or_direction_changed'),
            (dict(closing, gap=126), 'close_stop_alignment_or_direction_changed'),
            (dict(closing, gap=54), 'close_stop_below_held_gap'),
            (dict(closing, sideBox=[783, 649, 60, 180]), 'close_stop_object_moved'),
            (dict(closing, sx=55), 'close_stop_side_overlap_lost'),
            (dict(closing, sy=-80), 'close_stop_side_overlap_lost'),
            (dict(closing, sx=1e4), 'outside_demonstrated_visual_extent'),
        ]
        for observed, reason in cases:
            with self.subTest(reason=reason):
                vote = policy.decide(observed, active_action=ctx)
                self.assertEqual((vote['action'],vote['reason']), ('hold', reason))

    def test_centering_is_signed_and_requires_visible_sweep_clearance(self):
        p=fixture()
        self.assertEqual(p.decide(features(nx=-10))['action'],'base/left')
        self.assertEqual(p.decide(features(nx=12))['action'],'base/right')
        self.assertEqual(p.decide(features(sy=10,nx=-10))['action'],'hold')

    def test_unseen_or_missing_contact_geometry_cannot_trigger_lift(self):
        p=fixture()
        self.assertEqual(p.decide(features(sx=800))['action'],'hold')
        self.assertEqual(p.decide(features(sy=54,gap=60,ny=6))['action'],'hold')
        self.assertEqual(p.decide(features(),close_receipt={})['action'],'hold')
        self.assertNotEqual(p.decide(features(sy=36,ny=18))['action'],'gripper/close')

    def test_only_matching_registered_view_can_select_action(self):
        from learning.open_side_approach import WIDE_SIDE_VIEW
        model = artifact()
        policy = RelativeResponsePolicy(model, view_id=WIDE_SIDE_VIEW)
        vote = policy.decide(features())
        self.assertEqual(vote['reason'], 'response_model_view_mismatch')
        self.assertEqual(vote['action'], 'hold')
        model['sideViewId'] = WIDE_SIDE_VIEW
        self.assertEqual(RelativeResponsePolicy(model, view_id=WIDE_SIDE_VIEW).decide(features())['reason'],
                         'wide_jaw_identity_observation_only')
        self.assertEqual(RelativeResponsePolicy(model).decide(features())['action'], 'hold')
        model['sideViewId'] = 'unregistered'
        unknown = RelativeResponsePolicy(model, view_id='unregistered')
        self.assertEqual(unknown.decide(features())['reason'], 'response_model_unregistered_view')
        self.assertEqual(unknown.decide(features())['action'], 'hold')
