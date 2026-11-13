"""M1 scalar 재관측 코어의 합성 회귀. 실제 카메라·포트·학습 결과를 사용하지 않는다."""
import copy
import unittest

from learning import m1_shaft_learning as learning
from learning.m1_shaft_trial import M1ShaftTrial
from tests.test_m1_shaft_learning import scalar_sample
from tests.test_supervised_transport import Clock, Store


def model(delta200=4.,delta350=7.):
    rows=[]
    for duration,delta in ((200,delta200),(350,delta350)):
        for value in (118.,155.5):
            row=scalar_sample(f'{duration}-{value}','synthetic-train',value)
            row.update(command=dict(joint='gripper',direction='close',percent=40,durationMs=duration),
                       afterShaftPx=value-delta)
            rows.append(row)
    return learning.fit(rows)


def scope(**updates):
    result=dict(targetShaftPx=137.,tolerancePx=1.5,maxCommands=4,totalRemaining=4,maxSeconds=45.,
        minimumImprovementPx=.25,maximumPredictionResidualPx=3.,minimumObservedProgressPx=.2,
        shaftMinimum=118.,shaftMaximum=155.5,maxObservationAgeSeconds=3.,maxDecisionAgeSeconds=.25)
    result.update(updates);return result


class Ports:
    def __init__(self,clock,fixed,states):
        self.clock,self.fixed,self.states=clock,fixed,list(states)
        self.records=[];self.attempts=[];self.sent=[];self.asset_checks=0;self.ticks=0
        self.fail=False;self.mutate=None;self.preparation_delay=.11;self.observation_delay=.1
    def tick(self):self.ticks+=1
    def verify(self):self.asset_checks+=1
    def observe(self,*,after_off,deadline,tick):
        if after_off is not None:self.clock.sleep(max(0.,after_off+3.1-self.clock.now))
        tick();self.clock.sleep(self.observation_delay)
        state=self.states.pop(0) if len(self.states)>1 else self.states[0]
        record=dict(shaftSeparationPx=state,context=copy.deepcopy(self.fixed['context']),
            sourceFreshMetadataVerified=True,sourceHostBracket=[self.clock.now-.02,self.clock.now-.01],
            persistedAt=self.clock.now,hostClockId='same-host-session',
            sourceFrame=dict(frameId=f'frame-{len(self.records)}',pixelSha256='a'*64,width=320,height=240))
        self.records.append(record);return record
    def execute(self,command,before_run,decision):
        self.attempts.append(copy.deepcopy(command));self.clock.sleep(self.preparation_delay)
        if self.mutate:self.mutate(command,decision)
        at=before_run()
        self.sent.append(copy.deepcopy(command))
        if self.fail:raise TimeoutError('unknown actor result')
        self.clock.sleep(command['durationMs']/1000)
        return dict(offVerified=True,offAt=self.clock.now,dispatchBefore=at,dispatchAfter=at+.01,
            requestedPulse=dict(command,motor=1,expectedPwm=-(command['percent']*255//100)))


class M1ShaftTrialTests(unittest.TestCase):
    def trial(self,states,*,fixed=None,planned=None):
        clock=Clock();fixed=fixed or model();ports=Ports(clock,fixed,states);store=Store()
        trial=M1ShaftTrial(fixed,planned or scope(),observe=ports.observe,execute=ports.execute,
                          tick=ports.tick,verify_assets=ports.verify,store=store,clock=clock)
        return trial,ports,store
    def test_two_real_observation_corrections_reach_goal_with_diagnostics(self):
        trial,ports,store=self.trial([150.,143.,137.])
        result=trial.run()
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual(result['confirmedCommands'],2);self.assertEqual(result['commandAttempts'],2)
        self.assertEqual(len(ports.records),3);self.assertEqual([c['durationMs'] for c in ports.sent],[350,350])
        self.assertTrue(result['shaftGoalReachedOnly']);self.assertFalse(result['graspVerified'])
        self.assertFalse(result['initialGoalAlreadySatisfied'])
        self.assertEqual(result['steps'][1]['actualShaftSeparationPx'],137.)
        self.assertEqual(result['steps'][1]['absolutePredictionResidualPx'],1.)
        self.assertIn('commandMeanPredictionErrorPx',result['steps'][0])
        self.assertIn('result.json',store.rows)
        with self.assertRaisesRegex(ValueError,'single_use'):trial.run()
        self.assertEqual(len(ports.sent),2)
    def test_initial_goal_and_global_remaining_budget_are_separate_from_model_success(self):
        trial,ports,_=self.trial([137.])
        result=trial.run();self.assertTrue(result['initialGoalAlreadySatisfied'])
        self.assertEqual(result['confirmedCommands'],0);self.assertEqual(ports.sent,[])
        trial,ports,_=self.trial([150.,143.,136.],planned=scope(totalRemaining=1))
        result=trial.run();self.assertEqual(result['status'],'budget_reached')
        self.assertEqual(result['confirmedCommands'],1);self.assertEqual(len(ports.sent),1)
    def test_OOD_candidate_is_not_replaced_or_reopened(self):
        fixed=model()
        for entry in fixed['models']:entry.update(minimum=135.,maximum=145.)
        trial,ports,_=self.trial([150.],fixed=fixed)
        result=trial.run();self.assertEqual(result['status'],'abstained',result)
        self.assertEqual(result['commandAttempts'],0);self.assertEqual(ports.sent,[])
    def test_safe_predicted_endpoint_can_choose_200_instead_of_out_of_fence_350(self):
        trial,ports,_=self.trial([123.,119.],planned=scope(targetShaftPx=118.))
        result=trial.run();self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual(ports.sent[0]['durationMs'],200)
        self.assertEqual(result['steps'][0]['decision']['rejected'][0]['reason'],'M1_predicted_shaft_outside_fence')
    def test_boundary_goal_is_scalar_only_and_actual_outside_fence_stops(self):
        trial,ports,_=self.trial([126.,119.5],planned=scope(targetShaftPx=118.))
        result=trial.run();self.assertTrue(result['goalBoundaryReached']);self.assertFalse(result['contactVerified'])
        self.assertEqual(result['confirmedCommands'],1)
        trial,ports,store=self.trial([126.,117.9],planned=scope(targetShaftPx=118.))
        result=trial.run();self.assertEqual(result['status'],'failed');self.assertEqual(result['confirmedCommands'],1)
        self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_observed_shaft_outside_fence')
        self.assertEqual(result['observations'][-1]['shaftSeparationPx'],117.9)
        self.assertIn('result.json',store.rows)
    def test_failed_actor_never_assumes_acknowledgment_or_retries(self):
        trial,ports,store=self.trial([150.,143.]);ports.fail=True
        result=trial.run();self.assertEqual(result['status'],'failed')
        self.assertEqual(result['commandAttempts'],1);self.assertEqual(result['confirmedCommands'],0)
        self.assertEqual(len(ports.sent),1);self.assertFalse(result['automaticRetry'])
        self.assertEqual(len(ports.records),1);self.assertIn('result.json',store.rows)
    def test_no_progress_and_excess_residual_stop_before_second_command(self):
        trial,ports,_=self.trial([150.,150.],fixed=model(.3,.3),planned=scope(targetShaftPx=140.))
        result=trial.run();self.assertEqual(result['status'],'observed_no_progress',result)
        self.assertEqual(len(ports.sent),1)
        trial,ports,_=self.trial([150.,147.])
        result=trial.run();self.assertEqual(result['status'],'prediction_residual_exceeded',result)
        self.assertEqual(len(ports.sent),1)
    def test_dispatch_guard_rejects_mutated_record_command_or_decision_without_RUN(self):
        for kind in ('record','command','decision'):
            trial,ports,store=self.trial([150.,143.])
            def mutate(command,decision,kind=kind):
                if kind=='record':ports.records[-1]['shaftSeparationPx']=149.
                elif kind=='command':command['durationMs']=200
                else:decision['selected']['predictedShaftSeparationPx']=140.
            ports.mutate=mutate
            result=trial.run()
            with self.subTest(kind=kind):
                self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
                self.assertEqual(result['commandAttempts'],1);self.assertEqual(result['confirmedCommands'],0)
                self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_dispatch_binding_changed')
                self.assertIn('result.json',store.rows)
    def test_clock_budget_and_source_freshness_are_not_reset_by_a_new_decision(self):
        trial,ports,_=self.trial([150.],planned=scope(maxSeconds=.05))
        result=trial.run();self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
        trial,ports,_=self.trial([150.]);ports.preparation_delay=3.1
        result=trial.run();self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
        self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_stale_or_noncausal_source')
        trial,ports,_=self.trial([150.,143.,137.]);ports.preparation_delay=.4
        result=trial.run();self.assertEqual(result['status'],'observed_target_reached',result)
    def test_reselection_work_over_quarter_second_is_rejected(self):
        trial,ports,_=self.trial([150.]);original=trial._choose;calls=[]
        def slow(value):
            result=original(value);calls.append(value)
            if len(calls)>1:ports.clock.sleep(.3)
            return result
        trial._choose=slow
        result=trial.run();self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
        self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_dispatch_expired_or_budget_exhausted')
    def test_source_that_expires_during_under_quarter_second_reselection_never_runs(self):
        trial,ports,_=self.trial([150.]);ports.preparation_delay=2.85
        original=trial._choose;calls=[]
        def delayed_reselection(value):
            result=original(value);calls.append(value)
            if len(calls)>1:ports.clock.sleep(.2)
            return result
        trial._choose=delayed_reselection
        result=trial.run()
        self.assertEqual(result['status'],'failed')
        self.assertEqual(result['commandAttempts'],1);self.assertEqual(result['confirmedCommands'],0)
        self.assertEqual(ports.sent,[])
        self.assertGreater(ports.clock.now-ports.records[0]['sourceHostBracket'][0],3.)
        self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_dispatch_expired_or_budget_exhausted')
    def test_failed_actor_cannot_reuse_a_callback_that_never_reached_RUN(self):
        trial,ports,_=self.trial([150.]);saved=[]
        def fail_before_guard(command,before_run,decision):
            saved.append(before_run);raise TimeoutError('before RUN')
        trial.execute=fail_before_guard
        result=trial.run();self.assertEqual(result['commandAttempts'],1);self.assertEqual(result['confirmedCommands'],0)
        with self.assertRaisesRegex(ValueError,'guard_closed_or_reused'):saved[0]()
        self.assertEqual(ports.sent,[])
    def test_reused_or_too_early_after_OFF_source_and_unverified_receipt_stop(self):
        for mode in ('reused','early','off'):
            trial,ports,store=self.trial([150.,143.]);observe=ports.observe;execute=ports.execute
            def observed(*,after_off,deadline,tick):
                result=observe(after_off=after_off,deadline=deadline,tick=tick)
                if after_off is not None:
                    if mode=='reused':result['sourceFrame']['frameId']='frame-0'
                    elif mode=='early':result['sourceHostBracket'][0]=after_off+2.9
                return result
            def executed(command,before_run,decision):
                result=execute(command,before_run,decision)
                if mode=='off':result['offVerified']=False
                return result
            trial.observe=observed;trial.execute=executed
            result=trial.run()
            with self.subTest(mode=mode):
                self.assertEqual(result['status'],'failed');self.assertEqual(len(ports.sent),1)
                self.assertEqual(result['confirmedCommands'],0 if mode=='off' else 1)
                self.assertIn('result.json',store.rows)

    def test_trial_uses_only_model_CLOSE_candidates_and_accepts_supported_60_650(self):
        rows=[]
        close=dict(joint='gripper',direction='close',percent=60,durationMs=650)
        for direction,delta in (('close',-10.),('open',10.)):
            for value in (118.,155.5):
                row=scalar_sample(f'{direction}-strong-{value}','strong-train',value)
                row.update(command=dict(close,direction=direction),afterShaftPx=value+delta)
                rows.append(row)
        fixed=learning.fit(rows)
        trial,ports,_=self.trial([150.,140.],fixed=fixed,planned=scope(targetShaftPx=140.))
        self.assertEqual(trial.commands,[close])
        result=trial.run()
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual(ports.sent,[close]);self.assertEqual(result['confirmedCommands'],1)
        self.assertTrue(all(item['command'] is None or item['command']==close
                            for item in result['steps'][0]['decision']['candidates']))
        self.assertEqual(result['steps'][0]['decision']['rejected'],[])
        self.assertFalse(result['graspVerified'])
    def test_trial_rejects_candidate_set_mutation_or_missing_CLOSE_support(self):
        trial,ports,_=self.trial([150.]);trial.commands[0]['durationMs']=650
        result=trial.run();self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
        fixed=model()
        for entry in fixed['models']:entry['command']['direction']='open'
        with self.assertRaisesRegex(ValueError,'supported_close_candidates_required'):
            self.trial([150.],fixed=fixed)

    def test_exact_scope_frozen_assets_and_frame_identity_remain_required(self):
        for changed in (scope(tolerancePx=2.),scope(maxCommands=5),scope(totalRemaining=True),
                        scope(shaftMinimum=117.),scope(targetShaftPx=[0.,0.,137.])):
            with self.subTest(changed=changed),self.assertRaises(ValueError):self.trial([150.],planned=changed)
        trial,ports,_=self.trial([150.]);trial.model['models'][0]['coefficient']+=1.
        result=trial.run();self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
        self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_frozen_contract_changed')


if __name__=='__main__':unittest.main()
