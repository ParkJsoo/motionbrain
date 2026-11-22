import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
from PIL import Image

from learning.wide_pick import WidePickDevelopment
from learning.wide_wrist_height import WIDE_SIDE_VIEW, WideWristHeight


class WideWristHeightTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.labels = ['shoulder/down','elbow/down','gripper/close']
        self.rows = [dict(action=a,relativeHeight=h,frameId=f's:{i}',nearFrameId=f'n:{i}')
                     for i,(a,h) in enumerate(zip(self.labels,[-2,-1,0]))]
        self.rgb = self.ref('rgb', dict(exemplars=self.rows,trainingEpisode='s'))
        self.plan = dict(external=dict(viewConfigurationId=WIDE_SIDE_VIEW),
                         phaseObserver=self.rgb, developmentPick='pink-prism-pick-and-lift-v1',
                         scene=dict(object='pink-prism',physicalView=WIDE_SIDE_VIEW))

    def ref(self,name,data):
        p = self.root/name
        raw = json.dumps(data).encode()
        p.write_bytes(raw)
        return dict(artifact=str(p),sha256=hashlib.sha256(raw).hexdigest())

    def build(self):
        self.plan['wristHeightProposal'] = self.ref('height', dict(schema='wide_wrist_height_pick_v1',
            view=WIDE_SIDE_VIEW,trainingEpisode='s',sourceModelSha256=self.rgb['sha256'],examples=self.rows))
        return WideWristHeight(self.plan)

    def test_nested_heights_rejected(self):
        for row in self.rows:
            row['relativeHeight'] = [-2,-1]
        with self.assertRaises(ValueError):
            self.build()

    def test_relative_proposal_is_observation_only(self):
        self.build()
        proposer = Mock(labels=self.labels+['elbow/up'])
        proposer.predict.return_value = dict(action='gripper/close')
        policy = WidePickDevelopment(proposer,self.plan)
        im = Image.new('RGB',(1920,1080))
        match = (None,None,np.array([100,50]),None)
        with patch('learning.wide_wrist_height._marker_candidates',return_value=([match],[],[])), \
             patch('learning.wide_wrist_height.open_scene_prism_candidate',return_value=(10,200,30,100)), \
             patch('learning.wide_pick.observe_open_side',side_effect=ValueError('tip_missing')):
            vote = policy.predict(im,im)
            self.assertEqual(vote['wristHeightProposal']['action'],'elbow/down')
            self.assertEqual(vote['phaseProposal']['action'],'gripper/close')
            self.assertEqual(vote['action'],'hold')
