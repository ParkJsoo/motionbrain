"""전체 축 수집의 계속 진행·예산·재개·동시 실행 실패 회귀."""
import copy
import hashlib
from pathlib import Path
import tempfile
import unittest

from learning.batch_campaign import Campaign, SCHEMA, validate_plan


class Clock:
    def __init__(self): self.now = 100.
    def monotonic(self): return self.now
    def time(self): return self.now+1000


def plan():
    joints = [('gripper','close'),('wrist','up'),('elbow','up'),('shoulder','down'),('base','left')]
    return dict(schemaVersion=SCHEMA,campaignId='test',viewId='fixed',controllerBootId='boot',
                maxObservationAgeSeconds=2,nodes=['start','other'],
                budgets=dict(maxPhysicalCommands=200,maxPreparationCommands=40,maxAttempts=200,maxSeconds=3600),
                conditions=[dict(id=f'c{i}',command=dict(joint=j,direction=d,percent=20,durationMs=200),
                                 fromNodes=['start'],phase='training',captureGroup=f'train{i}',target=1,maxAttempts=1)
                            for i,(j,d) in enumerate(joints)])


def outcome(request, **changes):
    return dict(execution='completed',offVerified=True,journalClear=True,capture='recorded',
                observation='eligible',transitionId=hashlib.sha256(request.encode()).hexdigest(),
                reason='',dataIdentity=dict(sourceHash=hashlib.sha256(request.encode()).hexdigest(),
                    sessionId='session-'+request,episodeId='episode-'+request,
                    frameIds=[request+'-before',request+'-after']),**changes)


class Driver:
    def __init__(self, clock):
        self.clock,self.calls,self.saved = clock,[],{}
        self.actions = {};self.nodes=['start'];self.off=True;self.boot='boot';self.observations=0
    def observe(self):
        self.observations += 1
        return dict(nodes=self.nodes,bootId=self.boot,viewId='fixed',observedAt=self.clock.monotonic(),
                    offVerified=self.off,journalClear=self.off)
    def execute(self,c,r,*,deadline_wall):
        self.calls.append((c['id'],r));self.clock.now+=1
        result=outcome(r)
        result.update(self.actions.get(c['id'],{}))
        self.saved[r]=result
        return result
    def reconcile(self,c,r): return self.saved.get(r)


class BatchTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.clock=Clock();self.driver=Driver(self.clock);self.p=plan()
        self.instances=[]
        self.addCleanup(lambda:[x.close() for x in self.instances])
    def campaign(self,p=None,folder='run',driver=None):
        x=Campaign(p or self.p,self.root/folder,driver or self.driver,lock_path=self.root/'shared.lock',clock=self.clock)
        self.instances.append(x);return x

    def test_all_five_joints_and_180_transitions_without_device(self):
        for c in self.p['conditions']:c.update(target=36,maxAttempts=36)
        r=self.campaign().run()
        self.assertEqual((r['status'],r['validTransitions'],len(self.driver.calls)),('completed',180,180))
        self.assertEqual(len({x[1] for x in self.driver.calls}),180)
        self.assertEqual({c['command']['joint'] for c in self.p['conditions']},set(validate_plan(self.p)['supportedJoints']))

    def test_post_observation_failure_skips_condition_and_continues(self):
        self.driver.actions['c0']={'observation':'rejected','reason':'occluded'}
        r=self.campaign().run()
        self.assertEqual((r['status'],r['validTransitions'],r['completedExecutions']),('partial',4,5))
        self.assertEqual(r['conditions'][0]['blockedReason'],'occluded')
        self.assertEqual(len(self.driver.calls),5)

    def test_unknown_even_with_off_blocks_other_joints_and_never_replays(self):
        self.driver.actions['c0']={'execution':'unknown','observation':'rejected'}
        c=self.campaign();self.assertEqual(c.run()['status'],'halted')
        c.close();self.assertEqual(self.campaign().run()['status'],'halted')
        self.assertEqual(len(self.driver.calls),1)

    def test_unverified_off_blocks_all_joints(self):
        self.driver.actions['c0']={'offVerified':False}
        self.assertEqual(self.campaign().run()['status'],'halted')
        self.assertEqual(len(self.driver.calls),1)

    def test_crash_after_dispatch_reconciles_without_resend(self):
        c=self.campaign();original=c._finish
        def fail(*_):raise RuntimeError('power_loss_before_quota_commit')
        c._finish=fail
        with self.assertRaises(RuntimeError):c.run()
        self.assertEqual(c.report()['status'],'needs_reconciliation')
        c.close();r=self.campaign().run()
        self.assertEqual(r['validTransitions'],5)
        self.assertEqual([x[0] for x in self.driver.calls],['c0','c1','c2','c3','c4'])

    def test_crash_before_dispatch_no_result_cannot_be_replaced(self):
        def fail(c,r,**kwargs):raise RuntimeError('unknown_crash')
        self.driver.execute=fail;c=self.campaign()
        with self.assertRaises(RuntimeError):c.run()
        c.close();r=self.campaign().run()
        self.assertEqual(r['status'],'halted');self.assertEqual(self.driver.calls,[])

    def test_not_dispatched_is_not_valid_and_next_condition_runs(self):
        self.driver.actions['c0']={'execution':'not_dispatched','observation':'rejected'}
        r=self.campaign().run()
        self.assertEqual((r['notDispatched'],r['validTransitions']),(1,4))

    def test_preparation_is_charged_but_not_training(self):
        self.p['conditions'][0]['phase']='preparation'
        r=self.campaign().run()
        self.assertEqual((r['preparationCommands'],r['validTransitions']),(1,4))
        self.assertEqual(r['conditions'][0]['eligible'],1)

    def test_unsupported_m4_up_is_preflight_rejected_other_conditions_continue(self):
        self.p['conditions'][3]['command']['direction']='up'
        r=self.campaign().run()
        self.assertEqual(r['validTransitions'],4)
        self.assertEqual(r['rejectedConditions'][0]['conditionId'],'c3')

    def test_unreachable_node_never_creates_a_return_move(self):
        self.driver.nodes=['other']
        self.assertEqual(self.campaign().run()['status'],'no_reachable_conditions')
        self.assertEqual(self.driver.calls,[])

    def test_stale_or_changed_boot_or_off_state_never_dispatches(self):
        for field,value in [('boot','newboot'),('off',False)]:
            d=Driver(self.clock);setattr(d,field,value)
            r=self.campaign(folder=field,driver=d).run()
            self.assertEqual(r['status'],'awaiting_observation');self.assertEqual(d.calls,[])
        d=Driver(self.clock);original=d.observe
        def stale():
            result=original();result['observedAt']-=10;return result
        d.observe=stale
        self.assertEqual(self.campaign(folder='stale',driver=d).run()['status'],'awaiting_observation')
        self.assertEqual(d.calls,[])

    def test_budget_counts_failed_attempt_and_preparation(self):
        self.p['budgets'].update(maxPhysicalCommands=2,maxPreparationCommands=1)
        self.driver.actions['c0']={'observation':'rejected'}
        r=self.campaign().run();self.assertEqual(r['status'],'budget_exhausted')
        self.assertEqual(len(self.driver.calls),2)
        self.assertEqual(r['validTransitions'],1)

    def test_wall_deadline_survives_restart_and_observe_time_charged(self):
        self.p['budgets']['maxSeconds']=1
        c=self.campaign();original=self.driver.observe
        def slow():
            self.clock.now+=2;return original()
        self.driver.observe=slow
        self.assertEqual(c.run()['status'],'budget_exhausted');self.assertEqual(self.driver.calls,[])
        c.close();self.assertEqual(self.campaign().run()['status'],'budget_exhausted')

    def test_plan_mutation_and_group_overlap_rejected(self):
        c=self.campaign();c.close();p=copy.deepcopy(self.p);p['conditions'][0]['target']=2;p['conditions'][0]['maxAttempts']=2
        with self.assertRaisesRegex(ValueError,'saved_plan_changed'):self.campaign(p)
        p=copy.deepcopy(self.p);p['conditions'][1].update(phase='evaluation',captureGroup='train0')
        with self.assertRaisesRegex(ValueError,'group_overlap'):validate_plan(p)

    def test_duplicate_transition_cannot_inflate_count(self):
        for c in self.p['conditions']:self.driver.actions[c['id']]={'transitionId':'a'*64}
        r=self.campaign().run();self.assertEqual(r['validTransitions'],1)
        self.assertEqual(r['status'],'partial')

    def test_wrapped_same_source_does_not_inflate_quota(self):
        shared=dict(sourceHash='a'*64,sessionId='same',episodeId='same',frameIds=['before','after'])
        for c in self.p['conditions']:self.driver.actions[c['id']]={'dataIdentity':shared}
        r=self.campaign().run();self.assertEqual(r['validTransitions'],1)
        self.assertEqual(r['conditions'][1]['blockedReason'],'batch_reused_source_episode_or_frame')

    def test_evaluation_requires_prospective_evidence(self):
        self.p['conditions'][0].update(phase='evaluation',captureGroup='eval0')
        r=self.campaign().run();self.assertEqual(r['validTransitions'],4)
        self.assertEqual(r['conditions'][0]['eligible'],0)

    def test_two_dispatchers_share_resource_lock(self):
        c=self.campaign()
        with c._locked():
            with self.assertRaisesRegex(ValueError,'already_running'):self.campaign(folder='second')
        self.assertEqual(self.driver.calls,[])


if __name__=='__main__':unittest.main()
