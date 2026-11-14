"""실물 없이 유한 전체 과업과 실패 경계의 계약을 검증한다."""
import copy
import unittest

from learning.finite_task_cycle import FiniteTaskCycle, SCHEMA, STAGES, bounded_command
from learning.supervised_collection import plan_digest


class Clock:
    now = 100.
    def monotonic(self): return self.now


class Store:
    def __init__(self): self.files = {}
    def write_json(self, name, value):
        if name in self.files: raise FileExistsError(name)
        self.files[name] = copy.deepcopy(value)


def scope():
    cmds = [dict(joint=j, direction=d, percent=p, durationMs=ms) for j,d,p,ms in
            [('elbow','up',60,350), ('gripper','close',60,650), ('shoulder','up',50,650),
             ('shoulder','down',35,400), ('gripper','open',60,650)]]
    return dict(schemaVersion=SCHEMA, trialId='cycle-test', hostClockId='host', controllerBootId='b'*32,
                stages=[dict(kind=k,context={'object':'test', 'registration':k},modelSha256=str(i+1)*64,
                             maxCommands=3,commands=[c],target={'value':1}) for i,(k,c) in enumerate(zip(STAGES,cmds))],
                maxCommands=12,maxRequestedMs=7000,maxSeconds=200.,maxObservationAgeSeconds=3.,
                maxDecisionAgeSeconds=.25,postOffSeconds=3.,retentionSeconds=1.)


class Ports:
    def __init__(self, planned):
        self.scope=planned;self.clock=Clock();self.store=Store();self.n=0;self.count={k:0 for k in STAGES}
        self.required={k:1 for k in STAGES};self.sent=[];self.stops=0;self.idles=0
        self.fail=None;self.mutate=None;self.prepare_delay=0.;self.skip_prepare=False;self.skip_guard=False
        self.human=False;self.angle=196.;self.goal_bad=False;self.choice_changed=False;self.observation_audit=True

    def idle(self): self.idles+=1;return 'b'*32
    def stop(self): self.stops+=1
    def assets(self): pass
    def verify(self, stage, record): return self.observation_audit
    def observe(self, stage, *, after_off, deadline):
        self.clock.now=max(self.clock.now+.1, (after_off+4.2 if after_off else 0))
        self.n+=1
        frames=[dict(camera=k,frameId=f'{k}-{self.n}',pixelSha256='a'*64,
                     sourceEarliest=self.clock.now-.01,sourceLatest=self.clock.now-.005,persistedAt=self.clock.now)
                for k in ['external','gripper']]
        entry=dict(clearance=True,evidenceSha256='a'*64,noncontact=True,objectBetweenJaws=True,
                   depthVerified=True,bilateralContact=True,tableSupported=True,airborne=True)
        r=dict(context=stage['context'],hostClockId='host',controllerBootId='b'*32,stable=True,
               frames=frames,m4AngleDeg=self.angle,entry=entry,state=self.count[stage['kind']])
        if self.mutate:self.mutate(stage,r)
        return r
    def goal(self, stage, record):
        r=dict(reached=self.count[stage['kind']]>=self.required[stage['kind']],evidenceSha256='c'*64,
               observationSha256=plan_digest(record),humanAssisted=self.human,noncontact=True,clearance=True,
               bilateralContact=True,airborne=True,retained=True,tableSupported=True,bothJawsClear=True,
               observationTimes=[self.clock.now-1.,self.clock.now])
        if self.goal_bad:r['bothJawsClear']=False
        return r
    def choose(self,stage,record):
        c=copy.deepcopy(stage['commands'][0])
        if self.choice_changed and self.n%2==0:c['durationMs']-=1
        return dict(selected={'kind':'action','command':c},modelSha256=stage['modelSha256'])
    def execute(self, stage, command, *, prepare, before_run, decision, safety):
        if not self.skip_prepare:prepare()
        self.clock.now+=self.prepare_delay
        at=before_run() if not self.skip_guard else self.clock.now
        self.sent.append(command);self.count[stage['kind']]+=1
        if self.fail:raise self.fail
        self.clock.now+=command['durationMs']/1000+.05
        pulse=bounded_command(stage['kind'],command)
        return dict(requestedPulse=pulse,motor=pulse['motor'],offVerified=True,dispatchBefore=at,
                    dispatchAfter=at+.01,offAt=self.clock.now,
                    outputTrace=[dict(motor=pulse['motor'],pwm=pulse['expectedPwm']),
                                 dict(motor=pulse['motor'],pwm=0)])
    def cycle(self):
        return FiniteTaskCycle(self.scope,observe=self.observe,verify_observation=self.verify,choose=self.choose,
            goal=self.goal,verify_goal=lambda stage,record,proof:True,
            execute=self.execute,verify_assets=self.assets,check_idle=self.idle,
            stop=self.stop,store=self.store,clock=self.clock)


class FiniteCycleTests(unittest.TestCase):
    def run_ports(self, configure=None):
        p=Ports(scope())
        if configure: configure(p)
        c=p.cycle();return p,c,c.run()
    def test_complete_five_stage_model_cycle(self):
        p,c,r=self.run_ports()
        self.assertEqual(r['status'],'completed');self.assertEqual(r['confirmedCommands'],5)
        self.assertTrue(r['fullyModelSelectedCycle']);self.assertTrue(r['finalOffVerified'])
        self.assertEqual(p.stops,1)
        with self.assertRaisesRegex(ValueError,'single_use'):c.run()
    def test_reobserve_multiple_commands_not_fixed_playback(self):
        p,c,r=self.run_ports(lambda p:p.required.update(approach=3,lift=2,lower=2))
        self.assertEqual(r['status'],'completed');self.assertEqual(len(p.sent),9)
        self.assertGreater(p.n,18)
    def test_unknown_output_stops_without_retransmission(self):
        p,c,r=self.run_ports(lambda p:setattr(p,'fail',TimeoutError()))
        self.assertEqual(r['status'],'failed');self.assertEqual(len(p.sent),1)
        self.assertEqual(r['confirmedCommands'],0);self.assertEqual(p.stops,1)
    def test_no_prepare_means_no_dispatch(self):
        p,c,r=self.run_ports(lambda p:setattr(p,'skip_prepare',True))
        self.assertEqual(p.sent,[]);self.assertEqual(r['status'],'failed')
    def test_stale_late_decision_prevents_dispatch(self):
        p,c,r=self.run_ports(lambda p:setattr(p,'prepare_delay',.26))
        self.assertEqual(p.sent,[]);self.assertIn('cycle_dispatch_expired',str(r))
    def test_missing_post_camera_or_audit_prevents_run(self):
        for mode in ['audit','camera','stale','boot']:
            def config(p):
                if mode=='audit':p.observation_audit=False
                else:
                    def mutate(stage,r):
                        if mode=='camera':r['frames'].pop()
                        if mode=='stale':r['frames'][0]['sourceEarliest']-=4
                        if mode=='boot':r['controllerBootId']='x'*32
                    p.mutate=mutate
            p,c,r=self.run_ports(config)
            self.assertEqual(p.sent,[]);self.assertEqual(r['status'],'failed')
    def test_stage_and_total_budgets(self):
        for mode in ['stage','total','ms','deadline']:
            def config(p):
                if mode=='stage':p.required['approach']=4
                if mode=='total':p.scope['maxCommands']=1
                if mode=='ms':p.scope['maxRequestedMs']=350
                if mode=='deadline':p.scope['maxSeconds']=1
            p,c,r=self.run_ports(config)
            self.assertEqual(r['status'],'failed');self.assertLessEqual(len(p.sent),3)
    def test_scope_model_or_phase_goal_change_rejects(self):
        p=Ports(scope());c=p.cycle();c.scope['maxCommands']=20
        r=c.run();self.assertEqual(p.sent,[]);self.assertEqual(r['status'],'failed')
        p,c,r=self.run_ports(lambda p:setattr(p,'choice_changed',True))
        self.assertEqual(p.sent,[]);self.assertEqual(r['status'],'failed')
        p,c,r=self.run_ports(lambda p:setattr(p,'goal_bad',True))
        self.assertEqual(r['status'],'failed');self.assertEqual(len(p.sent),5)
    def test_zero_command_prepared_stages_not_full_model_success(self):
        p,c,r=self.run_ports(lambda p:p.required.update(close=0))
        self.assertEqual(r['status'],'completed');self.assertFalse(r['fullyModelSelectedCycle'])
    def test_human_contact_review_remains_explicit(self):
        p,c,r=self.run_ports(lambda p:setattr(p,'human',True))
        self.assertTrue(r['humanAssisted']);self.assertEqual(r['status'],'completed')
    def test_cumulative_M4_excursion_rejected(self):
        def config(p):
            def mutate(stage,r):
                if stage['kind']=='lift':r['m4AngleDeg']=199.1
            p.mutate=mutate
        p,c,r=self.run_ports(config)
        self.assertEqual(r['status'],'failed');self.assertEqual(len(p.sent),2)
    def test_contact_or_table_support_is_not_inferred_from_pixels(self):
        def config(p):
            def mutate(stage,r):
                if stage['kind']=='release':r['entry']['tableSupported']=False
            p.mutate=mutate
        p,c,r=self.run_ports(config)
        self.assertEqual(len(p.sent),4);self.assertEqual(r['status'],'failed')
    def test_goal_evidence_requires_independent_audit(self):
        p=Ports(scope());c=p.cycle();c.verify_goal=lambda *args:False
        r=c.run();self.assertEqual(r['status'],'failed');self.assertEqual(p.sent,[])
    def test_slow_observation_or_goal_audit_expires_frame(self):
        for mode in ['observation','goal']:
            p=Ports(scope());c=p.cycle()
            def slow(*args):p.clock.now+=3.1;return True
            if mode=='observation':c.verify_observation=slow
            else:c.verify_goal=slow
            r=c.run();self.assertEqual(r['status'],'failed');self.assertEqual(p.sent,[])
    def test_old_retention_before_last_lift_is_rejected(self):
        p=Ports(scope());c=p.cycle();original=p.goal
        def goal(stage,record):
            r=original(stage,record)
            if stage['kind']=='lift' and r['reached']:
                r['observationTimes']=[c.started,c.started+1]
            return r
        c.goal=goal;r=c.run()
        self.assertEqual(r['status'],'failed');self.assertEqual(len(p.sent),3)


if __name__ == '__main__':unittest.main()
