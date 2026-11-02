import copy
import tempfile
import unittest
from pathlib import Path
from learning.artifacts import ArtifactStore
from learning.supervised_collection import make_plan, validate_plan, plan_digest, SupervisedCollection


def plan(joint='wrist',direction='up',durations=(550,700,900),percent=63):
    return make_plan(session_id='session01',capture_group='group01',split='development',controller_url='http://127.0.0.1:9000',boot_id='a'*32,
        helper_sha256='b'*64,device_id='fake',view_id='side',pose_context='middle',joint=joint,direction=direction,percent=percent,durations=durations)

class Clock:
    def __init__(self):self.now=10.
    def monotonic(self):return self.now
    def sleep(self,s):self.now+=s

class Camera:
    def __init__(self,clock):self.clock=clock;self.frames=[];self.started=False;self.closed=False;self.fail_at=None;self.close_failed=False
    def start(self):self.started=True
    def wait_ready(self,timeout):pass
    def check(self):
        if self.fail_at is not None and self.clock.now>=self.fail_at:raise ValueError('recording_source_stall')
    def close(self):self.closed=True;return {'status':'failed' if self.close_failed else 'partial'}

class Port:
    def __init__(self,clock):self.clock=clock;self.executions=0;self.finish_calls=0;self.may_have_run=False;self.fail_run=None;self.fail_preflight=False;self.off=True
    def preflight(self):
        if self.fail_preflight:raise ValueError('not_idle')
    def execute(self,p,tick):
        self.may_have_run=False;tick();self.executions+=1;self.may_have_run=True
        start=self.clock.now;self.last_dispatch_before=start
        if self.executions==self.fail_run:raise TimeoutError('lost RUN')
        self.clock.sleep(.9)
        return dict(dispatchBefore=start,dispatchAfter=start+.05,offAt=self.clock.now,offVerified=True,outputTrace=[{'motor':2,'pwm':160},{'motor':2,'pwm':0}])
    def check_idle(self):
        if getattr(self,'idle_fault_after',None) is not None and self.clock.now>=self.idle_fault_after:raise ValueError('unexpected_quiet_output')
    def finish(self):self.finish_calls+=1;return self.off

class Tests(unittest.TestCase):
    def run_block(self,p=None,setup=lambda c,p:None):
        clock=Clock();camera=Camera(clock);port=Port(clock);setup(camera,port)
        with tempfile.TemporaryDirectory() as t:
            runner=SupervisedCollection(p or plan(),camera,port,ArtifactStore(Path(t)/'run'),clock=clock)
            result=runner.run(plan_digest(p or plan()))
            with self.assertRaises(ValueError):runner.run(plan_digest(p or plan()))
        return result,camera,port
    def test_success_three_no_live_measurement(self):
        r,c,p=self.run_block();self.assertEqual(r['status'],'completed');self.assertEqual(r['summary']['executed'],3)
        self.assertEqual(p.finish_calls,1);self.assertTrue(c.closed)
        for a,b in zip(r['trials'],r['trials'][1:]):self.assertGreaterEqual(b['dispatchBefore'],a['offAt']+3)
    def test_unknown_second_preserves_first_no_third(self):
        r,c,p=self.run_block(setup=lambda c,p:setattr(p,'fail_run',2))
        self.assertEqual([a['status'] for a in r['trials']],['executed','outcome_unknown','not_started'])
        self.assertEqual(p.executions,2);self.assertTrue(r['trials'][0]['measurementReady']);self.assertIn('dispatchBefore',r['trials'][1])
        self.assertTrue(r['finalIdleOffVerified'])
    def test_recording_stall_no_following_command(self):
        r,c,p=self.run_block(setup=lambda c,p:setattr(c,'fail_at',12))
        self.assertEqual(p.executions,1);self.assertEqual(r['trials'][0]['status'],'executed');self.assertNotIn('measurementReady',r['trials'][0])
    def test_post_output_fault_does_not_prove_quiet_window(self):
        r,c,p=self.run_block(setup=lambda c,p:setattr(p,'idle_fault_after',12))
        self.assertEqual(p.executions,1);self.assertNotIn('quietVerifiedUntil',r['trials'][0]);self.assertNotIn('measurementReady',r['trials'][0])
    def test_preflight_failure_does_not_start_camera_or_write(self):
        r,c,p=self.run_block(setup=lambda c,p:setattr(p,'fail_preflight',True));self.assertFalse(c.started);self.assertEqual(p.finish_calls,0);self.assertEqual(p.executions,0)
    def test_close_failure_not_reported_complete(self):
        r,c,p=self.run_block(setup=lambda c,p:setattr(c,'close_failed',True));self.assertEqual(r['status'],'failed');self.assertEqual(r['summary']['executed'],3)
    def test_bad_plan_boundaries(self):
        for key,value in [('postSeconds',0),('operatorSupervised',False),('controllerUrl','http://user:secret@localhost')]:
            v=plan();v[key]=value
            with self.assertRaises(ValueError):validate_plan(v)
        for key,value in [('expectedPwm',159),('percent',100),('durationMs',901)]:
            v=plan();v['pulses'][0][key]=value
            with self.assertRaises(ValueError):validate_plan(v)
        v=plan();v['pulses'][1]['direction']='down';v['pulses'][1]['expectedPwm']=-160
        with self.assertRaises(ValueError):validate_plan(v)
    def test_other_joint_profiles_not_m2_defaults(self):
        for args in [('gripper','open',(650,),60),('elbow','up',(1000,),75),('shoulder','down',(650,),35),('base','left',(700,),60)]:validate_plan(plan(*args))
        with self.assertRaises((ValueError,KeyError)):plan('shoulder','up',(650,),35)
        with self.assertRaises(ValueError):plan('shoulder','down',(650,650),35)
    def test_no_turnoff_claim_when_cleanup_fails(self):
        r,c,p=self.run_block(setup=lambda c,p:setattr(p,'off',False));self.assertFalse(r['finalIdleOffVerified']);self.assertEqual(r['status'],'failed')

if __name__=='__main__':unittest.main()
