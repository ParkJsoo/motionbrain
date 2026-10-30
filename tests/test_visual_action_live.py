"""Deterministic camera/executor integration; never opens a physical port."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.action_execution import ActionLimits, BoundedActionExecutor
from learning.visual_action_collection import audit_collection
from learning.visual_action_live import LiveActionSession, RecordingPort, validate_plan, plan_digest
from learning.visual_camera_stream import FrameInbox
from tests.test_action_execution import FakePort, FakeClock


def plan():
    return dict(schemaVersion='motionbrain.visual_action_pilot.v1',
        request=dict(bootId='a'*32,commandId='m2-pilot-1',kind='pulse',motor=2,pwm=120,durationMs=200,targetDegrees=0,percent=0),
        camera=dict(deviceId='test-camera',sessionId='test-session',viewId='test-view',width=64,height=64,
                    videoMirrored=False,videoRotationAngle=0,maxFrames=60,seconds=4,sampleFps=5,maxBytes=64*64*4*60,maxFrameAgeSeconds=.5),
        points=[dict(id='tip',x=20,y=20,reference=False),dict(id='body',x=40,y=20,reference=False),dict(id='fixed',x=30,y=40,reference=True)],
        beforeSeconds=.2,afterSeconds=1,maxTravelPx=40,referenceLimitPx=3,maxDispatchLagSeconds=.25,
        helperSha256='b'*64,startPoseId='pose-1')


class PumpClock(FakeClock):
    camera=None
    def sleep(self,seconds):
        super().sleep(seconds)
        if self.camera: self.camera.pump()


class FakeCamera(FrameInbox):
    def __init__(self,clock,p):
        super().__init__(clock=clock,max_frames=60,max_age=.5)
        self.p=p; self.started=None; self.closed=False; self.inject=None
        self.raw=bytes([20,30,40,255])*(64*64)
    def start(self):
        self.started=self.clock.now;self.next=self.started+.1; self.clock.camera=self
    def pump(self):
        if self.started is None:return
        if self.clock.now-self.started>=4:self.done=True;return
        if self.clock.now+1e-8>=self.next:
            self.next+=.1
            self.accept(dict(frameId='frame-'+str(self.count),imageFile='fake.bgra',sha256=hashlib.sha256(self.raw).hexdigest(),
                             pts=dict(value=self.count+1,timescale=10,epoch=0),sourceEarliest=self.clock.now,
                             features=[dict(id='tip',x=20,y=20,visible=True,confidence=1)]))
            if self.inject:self.inject(self)
    def pixels(self,packet):return self.raw
    def close(self):self.closed=True


class LiveSessionTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.p=plan();self.clock=PumpClock()
        self.camera=FakeCamera(self.clock,self.p)
        self.fake=FakePort(self.clock);self.fake.running_reads=3
        old_preflight=self.fake.preflight
        def advancing_preflight(r):
            value=old_preflight(r);self.clock.now+=.001;return value
        self.fake.preflight=advancing_preflight
        self.port=RecordingPort(self.fake)
        self.executor=BoundedActionExecutor((self.root/'journal.sqlite').resolve(),self.port,
            ActionLimits(motor=2,max_duration_ms=1500,max_pwm=180,max_percent=0,min_degrees=0,max_degrees=0),clock=self.clock)
        self.session=LiveActionSession(self.root/'run',self.p,self.camera,self.executor,self.port,clock=self.clock)
    def run_session(self):
        with patch('socket.socket',side_effect=AssertionError('network forbidden')):
            r=self.session.run(confirmed_request=self.p['request'])
        audit=audit_collection(self.root/'run')
        self.assertTrue(audit['integrityOk'],audit)
        self.assertTrue(audit['sessionClosed'],audit)
        self.assertTrue(self.camera.closed)
        return r
    def test_complete_links_before_during_after_and_never_claims_settling(self):
        r=self.run_session();self.assertTrue(r['recordingComplete'],r)
        self.assertEqual(len(self.fake.posts),1)
        a=r['attempts'][0]
        self.assertTrue(all(a['windows'][k] for k in ('before','during','after')))
        self.assertEqual(a['settlingObservation'],'unverified')
        self.assertFalse(r['trainingEligible'])
        self.assertEqual(r['mode'],'live_observation')
        self.assertEqual(a['request']['motor'],2)
        self.assertTrue((self.root/'run/executor-result.json').exists())
        with self.assertRaises(ValueError):self.session.run(confirmed_request=self.p['request'])
    def test_disabled_controller_never_dispatches(self):
        self.fake.failure='preflight';r=self.run_session()
        self.assertEqual(self.fake.posts,[])
        self.assertIsNone(r['attempts'][0]['dispatchAt'])
        self.assertEqual(r['attempts'][0]['executionState'],'not_sent')
    def test_failure_before_ready_never_sends(self):
        self.camera.inject=lambda c:c.fail('camera_disconnected')
        r=self.run_session();self.assertFalse(r['recordingComplete']);self.assertEqual(self.fake.posts,[])
    def test_tracking_loss_during_action_cancels_once_and_keeps_unknown(self):
        self.camera.inject=lambda c:c.fail('tracking_lost') if c.count>=4 else None
        r=self.run_session();self.assertEqual(len(self.fake.posts),1);self.assertEqual(len(self.fake.cancels),1)
        self.assertEqual(r['attempts'][0]['status'],'outcome_unknown')
    def test_transport_loss_does_not_retry(self):
        self.fake.failure='submit';r=self.run_session()
        self.assertEqual(len(self.fake.posts),1);self.assertEqual(len(self.fake.cancels),1)
        self.assertFalse(r['recordingComplete'])
    def test_unknown_result_retains_later_frames_as_unresolved(self):
        self.fake.failure='lookup';r=self.run_session()
        self.assertTrue(r['attempts'][0]['windows']['unresolved'])
        self.assertFalse(r['attempts'][0]['windows']['after'])
    def test_camera_stale_after_preflight_never_submits(self):
        original=self.port.preflight
        def preflight(r):
            b=original(r);self.clock.now+=1;return b
        self.port.preflight=preflight
        r=self.run_session();self.assertEqual(self.fake.posts,[])
        self.assertFalse(r['recordingComplete'])
    def test_storage_delay_after_intent_blocks_final_submit(self):
        original=self.session._record
        def record(kind,*args,**kwargs):
            out=original(kind,*args,**kwargs)
            if kind=='dispatch':self.clock.now+=.3
            return out
        self.session._record=record
        r=self.run_session();self.assertEqual(self.fake.posts,[])
        self.assertEqual(r['attempts'][0]['executionState'],'outcome_unknown')
        exact=json.loads((self.root/'run/executor-result.json').read_text())
        self.assertEqual(exact['executionState'],'not_sent')
    def test_signal_during_submit_retains_unknown_and_cleanup(self):
        self.fake.failure='interrupt';r=self.run_session()
        self.assertTrue(r['operatorInterrupted']);self.assertEqual(len(self.fake.cancels),1)
        self.assertEqual(r['attempts'][0]['executionState'],'outcome_unknown')
    def test_dispatch_is_durable_before_port_submit(self):
        def check():
            inputs=[json.loads(p.read_text()) for p in (self.root/'run/raw').glob('*.json')]
            self.assertEqual(sum(e['type']=='dispatch' for e in inputs),1)
            self.assertTrue(any(e['type']=='frame' for e in inputs))
        self.fake.before_submit=check
        self.assertTrue(self.run_session()['recordingComplete'])


class PlanTest(unittest.TestCase):
    def test_envelope_and_capture_budget_rejections(self):
        for field,value in [('motor',1),('pwm',-120),('pwm',181),('durationMs',1501)]:
            p=plan();p['request'][field]=value
            with self.subTest(field=field,value=value),self.assertRaises(ValueError):validate_plan(p)
        for field,value in [('seconds',2),('maxFrames',2),('maxBytes',4),('maxFrameAgeSeconds',.6),('sampleFps',1),('sampleFps',2)]:
            p=plan();p['camera'][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):validate_plan(p)
    def test_confirmation_digest_covers_tracking_and_request(self):
        p=validate_plan(plan());original=plan_digest(p)
        p['points'][0]['x']+=1;self.assertNotEqual(original,plan_digest(p))
    def test_inbox_stale_eof_and_fault_are_fail_closed(self):
        c=FakeClock();calls=[];inbox=FrameInbox(clock=c,max_frames=1,on_fault=lambda:calls.append(1))
        with self.assertRaises(ValueError):inbox.check_fresh()
        inbox.accept({'sourceEarliest':c.now,'features':[{'visible':True}]});inbox.check_fresh()
        c.now+=.6
        with self.assertRaises(ValueError):inbox.check_fresh()
        inbox.accept({'sourceEarliest':c.now,'features':[]})
        self.assertEqual(inbox.count,1);self.assertEqual(len(calls),1)
