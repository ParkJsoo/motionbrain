import hashlib
import json
import multiprocessing
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from PIL import Image
from learning.dual_view_capture import _side_for_gripper, capture, effective_vote, load_policy, side_sample_fps


class DemonstrationBindingTest(unittest.TestCase):
    def test_post_ack_gripper_pair_can_wait_for_one_new_side_frame(self):
        from contextlib import nullcontext
        for delivered in (True, False):
            with self.subTest(delivered=delivered):
                before = {'sourceEarliest':9.9,'frameId':'side-old'}
                after = {'sourceEarliest':10.01,'frameId':'side-fresh'}
                camera = Mock(lock=nullcontext(), frames=[before])
                now = [10.02]
                stop = Mock()
                def wait(seconds):
                    now[0] += seconds
                    if delivered:
                        camera.frames.append(after)
                    return False
                stop.wait.side_effect = wait
                chosen = _side_for_gripper(camera,stop,10.,10.04,clock=lambda:now[0])
                self.assertEqual(chosen, after if delivered else before)
                self.assertLessEqual(now[0],10.04)

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

    def test_four_and_eight_fps_use_the_corresponding_recording_budget(self):
        self.assertEqual(side_sample_fps({}), 4)
        for invalid in (0, 5, True, '8', 8.0):
            with self.assertRaisesRegex(ValueError, 'invalid_side_sample_fps'):
                side_sample_fps({'sideSampleFps': invalid})
        for fps in (4, 8):
            run = self.root / str(fps)
            run.mkdir()
            helper = run / 'helper'
            helper.write_bytes(b'helper')
            plan = dict(seconds=2, sideSampleFps=fps, helper=str(helper),
                        helperSha256=hashlib.sha256(b'helper').hexdigest(),
                        external={'deviceUniqueId':'id', 'viewConfigurationId':'side'})
            ctx = multiprocessing.get_context('spawn')
            with patch('learning.dual_view_capture.ActionRecorder') as recorder, \
                    patch('learning.dual_view_capture.load_policy'), \
                    patch('learning.dual_view_capture.time.monotonic', side_effect=[1., 4.]):
                recorder.return_value.close.return_value = {'status':'partial', 'clockBridge':True}
                with self.assertRaisesRegex(ValueError, 'no_paired_observations'):
                    capture(plan, run, 'session', ctx.Event(), ctx.Array('c', 8192))
            config = recorder.call_args.args[2]
            self.assertEqual(config['sampleFps'], fps)
            self.assertEqual(config['maxFrames'], fps * 2 + 20)
            self.assertEqual(config['maxBytes'], config['maxFrames'] * (1920 * 1080 * 4 + 65536))

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

    def test_observe_only_stop_finishes_current_pair_without_masking_timeout(self):
        from contextlib import ExitStack, nullcontext
        import time
        from tools.run_dual_view_policy import main
        with self.assertRaises(SystemExit):
            main(['--plan',str(self.root/'absent'),'--execute','--allow-observation-stop'])
        for failure in (None,'body deadline','recording_source_stall','recording_ended','side observer failure'):
            run=self.root/(failure or 'ok')
            run.mkdir()
            helper=run/'helper'
            helper.write_bytes(b'camera')
            plan=dict(seconds=8,external={'deviceUniqueId':'id','viewConfigurationId':'wide'},
                      helper=str(helper),helperSha256=hashlib.sha256(b'camera').hexdigest(),
                      gripperUrl='http://unused',gripperBootId='b'*32,scene={},maxAge=.5,maxSkew=.5)
            camera=Mock()
            camera.lock=nullcontext()
            camera.close.return_value={'status':'partial','clockBridge':True}
            near=Mock()
            near.count=0
            near.max_captures=100
            near.store.root=run
            near._bindings={}
            def take(**_):
                near.count+=1
                i=near.count
                if i==4:
                    (run/'stop-observation').touch()
                    if failure=='body deadline':
                        raise TimeoutError('body deadline')
                if i>4:
                    self.fail('started another GET after stop marker')
                t=time.monotonic()
                key='near:'+str(i)
                camera.frames=[dict(frameIndex=i,frameId='side:'+str(i),
                                    imageSha256=f'image:{i}',sourceEarliest=t,sourceLatest=t,stdoutReadAt=t)]
                near._bindings[key]={'folder':str(i)}
                return dict(frame={'frameId':key},sourceEarliest=t,sourceLatest=t,persistedAt=t)
            def check_camera():
                if near.count==4 and failure in ('recording_source_stall','recording_ended'):
                    raise ValueError(failure)
            camera.check.side_effect=check_camera
            near.capture.side_effect=take
            ctx=multiprocessing.get_context('spawn')
            with ExitStack() as mocks:
                for name,obj in [('ActionRecorder',camera),('GripperObservationSource',near)]:
                    mocks.enter_context(patch('learning.dual_view_capture.'+name,return_value=obj))
                decode_side=mocks.enter_context(patch('learning.dual_view_capture.side_image'))
                mocks.enter_context(patch('learning.dual_view_capture.gripper_image'))
                policy=mocks.enter_context(patch('learning.dual_view_capture.load_policy'))
                policy.return_value.predict.side_effect=lambda *_: {'action':'hold'}
                mailbox=ctx.Array('c',8192)
                observed=[]

                def observe_side(image, pair):
                    self.assertIs(image, decode_side.return_value)
                    self.assertEqual(json.loads(mailbox.value)['frameId'], pair['frameId'])
                    observed.append((pair['frameId'],pair['imageSha256']))
                    pair['extClock'].clear()
                    if failure=='side observer failure' and pair['frameId']=='side:4':
                        raise ValueError(failure)

                expectation=nullcontext()
                if failure is not None:
                    error=TimeoutError if failure=='body deadline' else ValueError
                    expectation=self.assertRaisesRegex(error,failure)
                with expectation:
                    capture(plan,run,'test',ctx.Event(),mailbox,allow_observation_stop=True,on_decoded_side=observe_side)
                self.assertTrue(json.loads(mailbox.value)['fault'])
            rows=(run/'decisions.jsonl').read_text().splitlines()
            self.assertEqual(len(rows),3 if failure=='body deadline' else 4)
            self.assertEqual(decode_side.call_count,len(rows))
            self.assertEqual(observed,[(f'side:{i}',f'image:{i}') for i in range(1,len(rows)+1)])
            self.assertTrue(all(json.loads(row)['extClock'] for row in rows))
            camera.close.assert_called_once()
            near.close.assert_called_once()

    def test_decoded_side_callback_requires_read_only_capture(self):
        callback = Mock()
        for options in ({}, {'allow_observation_stop': True, 'active_mailbox': object()},
                        {'allow_observation_stop': True, 'gripper_gate': object()}):
            with self.subTest(options=options), self.assertRaisesRegex(ValueError, 'requires_observation_only'):
                capture({},self.root,'test',None,None,on_decoded_side=callback,**options)
        callback.assert_not_called()
