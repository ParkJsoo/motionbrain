import hashlib
import json
import multiprocessing
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from PIL import Image
from learning.dual_view_capture import capture, effective_vote, load_policy


class DemonstrationBindingTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.side = self.root/'external-stream/host-ingress/0000.json'
        self.near = self.root/'gripper/capture-0001'
        self.side.parent.mkdir(parents=True)
        self.near.mkdir(parents=True)
        self.near_id = 'a'*32+':10'
        side_name = 'external-stream/frame-0000.bgra.lzbitmap'
        j = {'frameId': 'demo:0', 'imageSha256': 'side', 'imageFile': Path(side_name).name,
             'metadataFile': 'frame-0000.json', 'sourceEarliest': 1, 'sourceLatest': 1.02,
             'stdoutReadAt': 1.09}
        self.side.write_text(json.dumps(j))
        self.metadata = {'headers': {'X-MotionBrain-Frame-Id': self.near_id}, 'sha256': 'near',
                         'sessionId': 'demo', 'hostClockId': 'demo', 'requestStartedAt': 1.,
                         'receivedAt': 1.3, 'persistedAt': 1.35}
        self.scene = {'physicalView': 'side'}
        self.binding = {'imageSha256': 'near', 'record': {'frame': {'frameId': self.near_id},
                        'hostClockId': 'demo', 'context': self.scene, 'persistedAt': 1.35}}
        self.episode = dict(schemaVersion='motionbrain.manual_il_episode.v1', episode='demo', source='human_teleoperation',
            frames={'external': [dict(id='demo:0', file=side_name, sha256='side', sourceHostBounds=[1,1.02], availableHostTime=1.09)],
                    'gripper': [dict(id=self.near_id, file='gripper/capture-0001/esp32-frame.jpg', sha256='near',
                                     sourceHostBounds=[1.02,1.2], availableHostTime=1.35)]},
            actions=[dict(index=1,motor=2,command={'joint':'wrist','direction':'up'},press={
                'hostBounds':[1.6,1.7], 'availablePriorObservation':{'external':'demo:0','gripper':self.near_id}})])
        self.ref = {'episode':str(self.root/'episode.json'), 'actionIndex':1}
        self.plan = dict(examples=[self.ref],maxViewRmse=[.2,.2],scene=self.scene,external={})
        for name in ('side_image','gripper_image'):
            mock = patch('learning.dual_view_capture.'+name,return_value=Image.new('RGB',(32,24)))
            mock.start()
            self.addCleanup(mock.stop)

    def save(self):
        def put(path, payload):
            raw=json.dumps(payload).encode()
            path.write_bytes(raw)
            return hashlib.sha256(raw).hexdigest()
        self.binding['metadataSha256']=put(self.near/'esp32-frame.json',self.metadata)
        put(self.near/'source-binding.json',self.binding)
        self.ref['episodeSha256']=put(Path(self.ref['episode']),self.episode)

    def test_action_derived_from_press_and_bound_to_both_receipts(self):
        self.save()
        self.assertEqual(load_policy(self.plan).labels,['wrist/up'])
        self.plan['examples'].append(self.ref.copy())
        with self.assertRaisesRegex(ValueError,'duplicate_demonstration'):
            load_policy(self.plan)

    def test_reject_future_or_mismatched_frame_before_decoding(self):
        self.episode['actions'][0]['press']['hostBounds']=[1.32,1.34]
        self.save()
        with self.assertRaisesRegex(ValueError,'not_precommand'):
            load_policy(self.plan)
        self.episode['actions'][0]['press']['hostBounds']=[2.,2.01]
        self.metadata['sessionId']='other'
        self.save()
        with self.assertRaisesRegex(ValueError,'receipt_mismatch'):
            load_policy(self.plan)

    def test_dissent_survives_overwrite_and_stale_inputs_vote_hold(self):
        latch={}
        def vote(label, t, near):
            row=dict(action=label, decidedAt=t+.1,
                     extClock=dict(sourceEarliest=t,sourceLatest=t+.01,availableAt=t+.02),
                     nearClock=dict(sourceEarliest=near,sourceLatest=near+.01,availableAt=near+.02))
            effective_vote(row,latch,.5,.3)
            return row
        first=vote('wrist/up',1.,1.)
        hidden=vote('base/left',1.1,1.1)
        restored=vote('wrist/up',1.2,1.2)
        stale=vote('wrist/up',2.,1.3)
        self.assertNotIn('wrist/up',first['lastDissent'])
        self.assertEqual(hidden['lastDissent']['wrist/up'],1.1)
        self.assertEqual(restored['lastDissent']['wrist/up'],1.1)
        self.assertEqual(stale['action'],'hold')
        self.assertEqual(stale['modelAction'],'wrist/up')
        self.assertEqual(stale['lastDissent']['wrist/up'],1.3)

    def test_expired_warmup_and_failed_clock_close_are_not_successful_observations(self):
        helper = self.root/'helper'
        helper.write_bytes(b'helper')
        plan = dict(seconds=2, external={'deviceUniqueId':'id','viewConfigurationId':'view'},
                    helper=str(helper), helperSha256=hashlib.sha256(b'helper').hexdigest())
        for i, (closure, message) in enumerate([
                ({'status':'partial','clockBridge':{'verified':True}}, 'no_paired_observations'),
                ({'status':'failed','clockBridge':None}, 'closure_unverified')]):
            with self.subTest(closure=closure):
                run = self.root/str(i)
                run.mkdir()
                camera = Mock()
                camera.close.return_value = closure
                ctx = multiprocessing.get_context('spawn')
                mailbox = ctx.Array('c', 8192)
                with patch('learning.dual_view_capture.ActionRecorder',return_value=camera), \
                        patch('learning.dual_view_capture.load_policy'), \
                        patch('learning.dual_view_capture.time.monotonic',side_effect=[1.,4.]):
                    with self.assertRaisesRegex(ValueError,message):
                        capture(plan,run,'session',ctx.Event(),mailbox)
                self.assertTrue(json.loads(mailbox.value)['fault'])
