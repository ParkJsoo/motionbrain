"""관측 상대 형상과 독립 종단점으로 새 자세 후보를 제한한다."""
import math
import unittest

from learning.pose_close import RESOLUTION, close_pose_margin, compare_grasp_geometry


class PoseCloseSupportTests(unittest.TestCase):
    def setUp(self):
        self.f={k:0. for k in RESOLUTION}
        self.refs=[dict(episode='a',features=self.f),
                   dict(episode='b',features={k:v for k,v in RESOLUTION.items()})]

    def test_geometry_uses_separate_episodes_and_refuses_unseen_tool_direction(self):
        self.assertTrue(compare_grasp_geometry(self.f,self.refs)['supported'])
        self.assertFalse(compare_grasp_geometry(self.f,[self.refs[0]]*3)['supported'])
        self.assertFalse(compare_grasp_geometry(dict(self.f,sideJawAngle=10.),self.refs)['supported'])
        self.assertFalse(compare_grasp_geometry(dict(self.f,depth=math.nan),self.refs)['supported'])

    def test_lift_leaves_space_for_the_measured_goal_and_stop(self):
        self.assertFalse(close_pose_margin(194.,[3.]))
        self.assertTrue(close_pose_margin(204.9,[3.]))
        self.assertFalse(close_pose_margin(205.,[3.]))
        self.assertFalse(close_pose_margin(206.,[float('inf')]))


if __name__=='__main__':
    unittest.main()
