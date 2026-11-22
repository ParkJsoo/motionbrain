import contextlib
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image

from learning.dual_view_capture import load_policy
from learning.forearm_approach import WIDE_SIDE_VIEW, fit_response, project
from tools.run_dual_view_policy import main


class ForearmApproachTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.marks = np.array([[0, 0], [10, 0], [40, 0]])
        self.effect = fit_response(self.marks, self.marks+[0, 10])
        self.effect['trainingShoulderRangeDeg'] = [216, 216]
        model = dict(schema='wide_forearm_approach_v1', commandMode='manual_hold_70_percent_from_press',
                     controlEligible=False, configurationSha256='setup', view=WIDE_SIDE_VIEW,
                     effects={'0.55': {'elbow/down': self.effect}})
        artifact = self.root/'weights.json'
        artifact.write_text(json.dumps(model))
        self.ref = dict(artifact=str(artifact), sha256=hashlib.sha256(artifact.read_bytes()).hexdigest(),
                        shoulderDeg=214.71, horizonSeconds=.55, allowedActions=['elbow/down'])
        self.plan = dict(forearmApproachObserver=self.ref,
            scene=dict(physicalView=WIDE_SIDE_VIEW, object='pink-prism'),
            external=dict(configurationSha256='setup', viewConfigurationId=WIDE_SIDE_VIEW),
            profiles={label:dict(shoulderRangeDeg=[215, 217]) for label in ('elbow/down','gripper/close')})

    def test_transfers_rotation_and_translation_in_local_frame(self):
        # An independently placed link rotated 90 degrees moves left when the
        # source moved down. Applying another rigid rotation also turns its tip.
        quarter_turn = np.array([[0, 1], [-1, 0]])
        rotated = self.marks @ quarter_turn
        np.testing.assert_allclose(project(self.effect, rotated, rotated[2]), rotated[2]+[-10, 0])
        rotation = fit_response(self.marks, rotated)
        np.testing.assert_allclose(project(rotation, self.marks, [60, 0]), [0, 60])

    def test_scores_allowed_candidates_and_retains_pose_and_command_limits(self):
        im = Image.new('RGB', (320, 240))
        with patch('learning.forearm_approach.marks', return_value=self.marks), \
             patch('learning.forearm_approach.near_geometry', return_value={'insideJaws':True}), \
             patch('learning.forearm_approach.observe_open_side', return_value=dict(box=[0,50,10,20],trackingMode='wide_wrist_jaw_contour',sideOffsetPx=[0,-31])):
            vote = load_policy(self.plan).predict(im,im)
            self.assertEqual(vote['costs'], {'hold':31, 'elbow/down':21})
            self.assertEqual(vote['action'], 'elbow/down')
            self.assertFalse(vote['support']['elbow/down']['insideTrainingPose'])
            self.assertFalse(vote['support']['elbow/down']['catalogStartPoseInRange'])
            self.assertFalse(vote['controlEligible'])
            self.assertFalse(vote['support']['gripper/close']['fitted'])
            self.ref['allowedActions'] = []
            self.assertEqual(load_policy(self.plan).predict(im,im)['action'], 'hold')

    def test_native_timed_model_ranks_independently_of_manual_projection(self):
        from learning.timed_pick_evidence import SIDE_RESPONSE_SCHEMA
        im = Image.new('RGB', (320,240))
        self.plan['gripperBootId'] = 'camera'
        self.ref['shoulderDeg'] = 216
        command = dict(motor=3,kind='pulse',pwm=95,durationMs=550,percent=0,targetDegrees=0)
        self.plan['profiles']['elbow/down']['command'] = command
        row = dict(label='elbow/down',episode='fit',command=command,sourceRole='development',
                   terminal='timed_out',shoulderDeg=216,beforeOffset=[0,-41],
                   afterOffsets=[[0,-29],[0,-31]],trackingMode='wide_wrist_jaw_contour')
        artifact = dict(schema=SIDE_RESPONSE_SCHEMA,controlEligible=False,samples=[row],
                        **{k:self.plan[k] for k in ('external','scene','gripperBootId')})
        path = self.root/'native.json'
        path.write_text(json.dumps(artifact))
        self.ref['nativeTimedApproach'] = dict(artifact=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        with patch('learning.forearm_approach.marks',return_value=self.marks), \
             patch('learning.forearm_approach.near_geometry',return_value={'insideJaws':True}), \
             patch('learning.forearm_approach.observe_open_side',return_value=dict(
                 box=[0,50,10,20],trackingMode='wide_wrist_jaw_contour',sideOffsetPx=[0,-44])):
            policy=load_policy(self.plan)
            vote=policy.predict(im,im)
            self.assertEqual(vote['reason'],'native_timed_approach_cost')
            self.assertEqual(vote['action'],'elbow/down')
            self.assertEqual(vote['nativeTimedApproach']['costs']['elbow/down'],33)
            self.assertEqual(vote['costs']['elbow/down'],33)
            self.assertEqual(vote['manualPrediction']['costs']['elbow/down'],34)
            self.assertEqual(vote['commandMode'],'native_bounded_command_response')
            self.assertNotIn('projected',vote)
            self.assertNotIn('horizonSeconds',vote)
            self.assertFalse(vote['controlEligible'])
            self.assertEqual(policy.predict(im,im,close_receipt={})['action'],'hold')

    def test_projected_tip_on_or_past_stop_boundary_is_excluded_but_logged(self):
        im = Image.new('RGB',(320,240))
        with patch('learning.forearm_approach.marks',return_value=self.marks), \
             patch('learning.forearm_approach.near_geometry',return_value={'insideJaws':True}):
            for gap in (21,30):
                with self.subTest(gap=gap), patch('learning.forearm_approach.observe_open_side',
                        return_value=dict(box=[0,50,10,20],trackingMode='wide_wrist_jaw_contour',sideOffsetPx=[0,-gap])):
                    vote = load_policy(self.plan).predict(im,im)
                    self.assertEqual(vote['blockedByApproachBoundary'], ['elbow/down'])
                    self.assertEqual(vote['action'], 'hold')
                    self.assertLess(vote['costs']['elbow/down'], vote['costs']['hold'])
                    self.assertEqual(vote['projected']['elbow/down'], [5, 60-gap])

    def test_invalid_or_missing_scene_cannot_choose_approach(self):
        im = Image.new('RGB',(320,240))
        with patch('learning.forearm_approach.marks',return_value=self.marks), \
             patch('learning.forearm_approach.near_geometry',return_value=None):
            self.assertEqual(load_policy(self.plan).predict(im,im)['action'],'hold')
        self.plan['external']['configurationSha256'] = 'different'
        with self.assertRaisesRegex(ValueError,'unsupported_forearm'):
            load_policy(self.plan)

    def test_cli_refuses_execution_before_construction_or_device_access(self):
        plan = self.root/'plan.json'
        plan.write_text(json.dumps(self.plan))
        with patch('tools.run_dual_view_policy.load_policy', side_effect=AssertionError), \
             contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            main(['--plan', str(plan),'--execute'])
