import copy
import unittest
from collections import Counter
from unittest.mock import patch

from learning.joint_action_selection import canonical_sha256
from learning.joint_goal_evaluation import GoalEvaluation, make_scope, schedule, positioning, decide, summarize
from tests.test_joint_goal_trial import Clock, Camera, Store


class Selector:
    model_sha256='a'*64
    dataset_sha256='b'*64
    velocity_bounds=(-.02,.04)
    def select(self,state,goal,*,tolerance_deg=1.):
        candidates=[]
        for direction in ('up','down'):
            for duration in (700,900):
                end=state['angleDeg']+(1 if direction=='up' else -1)*(4 if duration==700 else 5)
                candidates.append(dict(kind='pulse',direction=direction,durationMs=duration,pwmMagnitude=160,
                                       eligible=True,predictedEndDeg=end,predictedGoalErrorDeg=abs(goal-end)))
        best=min(candidates,key=lambda c:c['predictedGoalErrorDeg'])
        return dict(status='selected',selected=copy.deepcopy(best),candidates=candidates,
                    noChangeGoalErrorDeg=abs(goal-state['angleDeg']),angleDeg=state['angleDeg'])


class Sim:
    def __init__(self):
        self.clock=Clock();self.angle=10.97;self.cameras=[];self.commands=[];self.ports=[]
        self.failure=None;self.bad_receipt=False;self.missing_guard=False;self.double_guard=False
        self.position_stuck=False;self.camera_failure=None;self.store=Store();self.selector=Selector()
        self.scope=make_scope(self.selector,'test-campaign')
        self.runner=GoalEvaluation(self.scope,self.selector,self.camera,self.port,self.store,clock=self.clock)
    def camera(self,index):
        c=Camera(self.clock);c.value=self.angle
        original=c.pump
        def pump():
            c.value=self.angle;original()
        c.pump=pump
        c.close=lambda:setattr(c,'closed',True)
        self.cameras.append(c);return c
    def port(self,label):
        sim=self
        class Port:
            def preflight(self):
                if sim.failure=='preflight':raise ValueError('preflight_failed')
            def execute(self,pulse,guard,tick):
                if sim.failure=='guard':raise ValueError('before_dispatch')
                at=sim.clock.monotonic() if sim.missing_guard else guard()
                if sim.double_guard:guard()
                sim.commands.append((label,copy.deepcopy(pulse)))
                if sim.failure=='response':raise TimeoutError('run_response_unknown')
                if not ('positioning' in label and sim.position_stuck):
                    sim.angle+=(1 if pulse['direction']=='up' else -1)*{550:2.,700:4.,900:5.}[pulse['durationMs']]
                sim.clock.sleep(pulse['durationMs']/1000);tick()
                if sim.camera_failure:sim.cameras[-1].fault=sim.camera_failure
                return dict(dispatchAt=at,offAt=sim.clock.monotonic(),offVerified=not sim.bad_receipt)
            def finalize(self):return sim.failure!='off'
        p=Port();sim.ports.append((label,p));return p
    def run(self):
        with patch('socket.socket',side_effect=AssertionError('network forbidden')):
            return self.runner.run(confirmed_scope_sha256=canonical_sha256(self.scope))


class EvaluationTests(unittest.TestCase):
    def test_balanced_prespecified_schedule(self):
        self.assertEqual(Counter((t['pose'],t['policy']) for t in schedule()),Counter({(p,q):2 for p in ('low','high') for q in ('model','fixed700','hold')}))
        self.assertEqual(Counter(t['session'] for t in schedule()),{0:6,1:6})
    def test_full_campaign_observes_hold_and_measures_all_denominators(self):
        s=Sim();r=s.run();self.assertEqual(r['status'],'completed',r['errors'])
        self.assertEqual(len(s.cameras),2);self.assertTrue(all(c.closed for c in s.cameras))
        self.assertEqual(r['counts']['target'],8);self.assertLessEqual(r['counts']['positioning'],36)
        for p in ('model','fixed700','hold'):
            self.assertEqual(r['summary'][p]['planned'],4);self.assertEqual(r['summary'][p]['measured'],4)
        self.assertEqual(r['summary']['hold']['executed'],0)
        self.assertTrue(all(abs(x)<1e-8 for x in r['summary']['hold']['measuredErrorReductionsDeg']))
        self.assertTrue(r['finalIdleOffVerified']);self.assertFalse(r['trainingPerformed'])
    def test_full_campaign_with_30hz_quantized_eight_fps_samples(self):
        s=Sim();factory=s.camera
        def camera(index):
            c=factory(index)
            def pump():
                if not c.started:return
                while c.first_source_at+len(c.history)*(4/30)<=s.clock.now:
                    at=c.first_source_at+len(c.history)*(4/30)
                    c.history.append(dict(sourceAt=at,receivedAt=at,angleDeg=s.angle,
                                          valid=True,stable=at-c.first_source_at>=.8))
            c.pump=pump;return c
        s.runner.camera_factory=camera;r=s.run()
        self.assertEqual(r['status'],'completed',r['errors'])
        self.assertEqual(sum(v['measured'] for v in r['summary'].values()),12)

    def test_scope_change_fails_before_factories(self):
        s=Sim()
        with self.assertRaises(ValueError):s.runner.run(confirmed_scope_sha256='wrong')
        self.assertEqual(s.ports,[]);self.assertEqual(s.cameras,[])
        for k,v in [('maxTargetCommands',9),('captureSeconds',151),('automaticRetries',1)]:
            scope=copy.deepcopy(s.scope);scope[k]=v
            with self.assertRaises(ValueError):GoalEvaluation(scope,s.selector,s.camera,s.port,s.store)
    def test_cannot_reuse_campaign(self):
        s=Sim();s.run()
        with self.assertRaises(ValueError):s.run()
    def test_lost_run_reply_stops_without_retry_and_retains_unstarted_trials(self):
        s=Sim();s.failure='response';r=s.run()
        self.assertEqual(r['status'],'failed');self.assertEqual(len(s.commands),1)
        self.assertEqual(len(s.cameras),1);self.assertEqual(sum(v['notStarted'] for v in r['summary'].values()),11)
    def test_target_reply_loss_is_unknown_not_zero_execution(self):
        s=Sim();s.angle=9.;s.failure='response';r=s.run()
        self.assertEqual(r['counts']['target'],1)
        self.assertIsNone(r['trials'][0]['executed'])
        self.assertEqual(r['trials'][0]['executionStatus'],'outcome_unknown')
        self.assertEqual(r['summary']['model']['executionUnknown'],1)
        self.assertEqual(r['commandAttempts'][0]['phase'],'target')
    def test_concurrent_new_sample_does_not_look_like_future_data(self):
        s=Sim();factory=s.camera
        def camera(index):
            c=factory(index);samples=c.samples
            def incoming():
                s.clock.now+=.001;c.pump();return samples()
            c.samples=incoming;return c
        s.runner.camera_factory=camera
        r=s.run();self.assertEqual(r['status'],'completed',r['errors'])

    def test_positioning_budget_is_explicit_and_stops_campaign(self):
        s=Sim();s.position_stuck=True;r=s.run()
        self.assertEqual(r['errors'][0]['reason'],'trial_positioning_budget')
        self.assertEqual(r['counts']['positioning'],3);self.assertEqual(r['counts']['target'],0)
    def test_invalid_off_receipt_never_reaches_measurement(self):
        s=Sim();s.bad_receipt=True;r=s.run()
        self.assertEqual(r['status'],'failed');self.assertEqual(len(s.commands),1)
        self.assertTrue(all(t['measurement'] is None for t in r['trials']))
    def test_missing_or_duplicate_guard_prevents_success(self):
        for flag in ('missing_guard','double_guard'):
            s=Sim();setattr(s,flag,True);r=s.run()
            self.assertEqual(r['status'],'failed');self.assertLessEqual(len(s.commands),1)
    def test_tracking_loss_stops_and_closes_camera(self):
        s=Sim();s.camera_failure='tracking_lost';r=s.run()
        self.assertEqual(r['status'],'failed');self.assertEqual(len(s.commands),1);self.assertTrue(s.cameras[0].closed)
    def test_cleanup_failure_never_claims_verified_off(self):
        s=Sim();s.failure='off';r=s.run()
        self.assertFalse(r['finalIdleOffVerified']);self.assertEqual(r['status'],'failed')
    def test_intent_storage_failure_never_sends_command(self):
        s=Sim()
        def hook(name):
            if '-positioning-' in name:raise OSError('disk full')
        s.store.hook=hook;r=s.run()
        self.assertEqual(r['status'],'failed');self.assertEqual(s.commands,[])
    def test_stale_or_unsettled_latest_before_dispatch_stops(self):
        for invalid in ('stable','sourceAt'):
            s=Sim()
            def hook(name):
                if '-position-' in name and '-dispatch' in name:
                    camera=s.cameras[-1]
                    camera.history[-1][invalid]=False if invalid=='stable' else 0.
            s.store.hook=hook;r=s.run()
            self.assertEqual(r['status'],'failed');self.assertEqual(s.commands,[])
    def test_fixed700_is_predeclared_not_model_argmin(self):
        state=dict(angleDeg=5.,velocityDegS=0.,valid=True,settled=True)
        t=dict(pose='low',policy='fixed700',goalDeg=10.)
        d=decide(Selector(),state,t);self.assertEqual(d['selected']['durationMs'],700)
        t['policy']='model';self.assertEqual(decide(Selector(),state,t)['selected']['durationMs'],900)
    def test_positioning_envelope_and_in_pose_no_command(self):
        self.assertIsNone(positioning({'angleDeg':5.},'low'))
        with self.assertRaises(ValueError):positioning({'angleDeg':25.},'low')
        self.assertEqual(positioning({'angleDeg':10.97},'high')['durationMs'],550)


if __name__=='__main__':unittest.main()
