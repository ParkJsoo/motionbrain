import unittest
from unittest.mock import Mock, patch

from PIL import Image
from learning.wide_pick import WidePickDevelopment, WIDE_SIDE_VIEW


class DevelopmentPickTest(unittest.TestCase):
    def setUp(self):
        proposer = Mock(labels=['gripper/close','elbow/down','shoulder/down','elbow/up'])
        self.policy = WidePickDevelopment(proposer, dict(external=dict(viewConfigurationId=WIDE_SIDE_VIEW),
            developmentPick='pink-prism-pick-and-lift-v1', scene=dict(object='pink-prism',physicalView=WIDE_SIDE_VIEW)))
        self.image = Image.new('RGB',(320,240))
        self.side = self.enterContext(patch('learning.wide_pick.observe_open_side',
            return_value=dict(box=[20,40,40,100],sideOffsetPx=[0,12])))
        self.near = self.enterContext(patch.object(self.policy.near,'observe',return_value=dict(
            state=[0,0,100],nearClosingVisibilityGate=dict(passed=True))))

    def vote(self, action, **kwargs):
        self.policy.proposer.predict.return_value = dict(action=action,meanSquaredError=10.)
        return self.policy.predict(self.image,self.image,**kwargs)

    def test_approach_stops_at_near_and_close_requires_both_views(self):
        self.assertEqual(self.vote('elbow/down')['action'],'hold')
        self.assertEqual(self.vote('gripper/close')['action'],'gripper/close')
        self.side.return_value['sideOffsetPx']=[0,-70]
        self.assertEqual(self.vote('shoulder/down')['action'],'shoulder/down')
        self.assertEqual(self.vote('gripper/close')['action'],'hold')
        self.side.return_value['sideOffsetPx']=[0,12]
        self.near.return_value['nearClosingVisibilityGate']['passed']=False
        self.assertEqual(self.vote('gripper/close')['action'],'hold')

    def test_lift_needs_close_and_repeat_close_is_held(self):
        self.assertEqual(self.vote('elbow/up')['action'],'hold')
        self.assertEqual(self.vote('gripper/close',close_receipt={})['action'],'hold')


    def test_verified_close_requires_narrow_visible_grasp_for_lift(self):
        receipt = dict(action=dict(record=dict(state='completed')))
        with patch('learning.wide_pick.closed_by_controller',return_value=True):
            self.near.return_value.update(components=dict(left=dict(bbox=[0,0,10,25]),
                right=dict(bbox=[40,0,10,25])),object=dict(bbox=[11,10,28,20]),state=[0,0,49])
            self.assertEqual(self.vote('elbow/up',close_receipt=receipt)['action'],'elbow/up')
            self.near.return_value['state'][2] = 130
            self.assertEqual(self.vote('elbow/up',close_receipt=receipt)['action'],'hold')
            self.near.return_value['state'][2] = 49
            receipt['action']['record']['state'] = 'cancelled'
            self.assertEqual(self.vote('elbow/up',close_receipt=receipt)['action'],'hold')
