"""M1 scalar 재관측 코어의 합성 회귀. 실제 카메라·포트·학습 결과를 사용하지 않는다."""
import copy
import unittest

from learning import m1_shaft_learning as learning
from learning.m1_shaft_trial import (M1ShaftTrial, OBSERVED95_POLICY, OBSERVED95_LOWER_POLICY,
    OBSERVED95_REVIEWED_LOWER_POLICY, OBSERVED95_GRASP650_POLICY, OBSERVED95_GOAL, OBSERVED95_CONTACT_GOAL)
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


def observed95_model(*, minimum=92., maximum=130., delta=8., duration=450):
    rows=[]
    for value in (minimum,maximum):
        row=scalar_sample(f'observed95-{value}','observed95-train',value)
        row.update(command=dict(joint='gripper',direction='close',percent=60,durationMs=duration),
                   afterShaftPx=value-delta)
        rows.append(row)
    return learning.fit(rows)


def observed95_scope(fixed, **updates):
    result=scope(targetShaftPx=85.3,shaftMinimum=84.,shaftMaximum=139.5,
        geometryPolicy=OBSERVED95_POLICY,startIntervalPx=[100.,105.],
        observationRegistrationSha256=fixed['context']['landmarks'][len(learning.SHAFT_PREFIX):],
        goalSemantics=OBSERVED95_GOAL)
    result.update(updates);return result


class Observed95TrialTests(unittest.TestCase):
    trial=M1ShaftTrialTests.trial

    def test_supported_nonterminal_correction_and_terminal_endpoint_outside_training_support(self):
        fixed=observed95_model();planned=observed95_scope(fixed)
        trial,ports,store=self.trial([101.,93.,85.3],fixed=fixed,planned=planned)
        result=trial.run()
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual(result['confirmedCommands'],2)
        self.assertEqual([c['durationMs'] for c in ports.sent],[450,450])
        self.assertFalse(result['initialGoalAlreadySatisfied'])
        self.assertFalse(result['graspVerified']);self.assertIn('result.json',store.rows)

    def test_nonterminal_prediction_outside_next_support_abstains_without_command(self):
        fixed=observed95_model(minimum=100.,maximum=130.,delta=10.)
        trial,ports,_=self.trial([101.],fixed=fixed,planned=observed95_scope(fixed))
        result=trial.run()
        self.assertEqual(result['status'],'abstained',result);self.assertEqual(ports.sent,[])
        decision=trial._choose(101.)
        self.assertEqual(decision['rejected'][0]['reason'],'M1_predicted_nonterminal_shaft_outside_support')

    def test_explicit_one_command_trial_observes_unsupported_after_without_reusing_as_input(self):
        fixed=observed95_model(minimum=100.,maximum=130.,delta=10.)
        planned=observed95_scope(fixed,maxCommands=1)
        trial,ports,_=self.trial([101.,91.],fixed=fixed,planned=planned)
        result=trial.run()
        self.assertEqual(result['status'],'budget_reached',result)
        self.assertEqual(result['commandAttempts'],1);self.assertEqual(result['confirmedCommands'],1)
        self.assertEqual(len(ports.records),2);self.assertEqual(len(ports.sent),1)
        self.assertFalse(result['shaftGoalReachedOnly'])
        self.assertEqual(result['steps'][0]['actualShaftSeparationPx'],91.)
        # 잔여예산만 1인 다회 정책은 명시적인 단일명령 실험으로 바뀌지 않는다.
        trial,ports,_=self.trial([101.],fixed=fixed,planned=observed95_scope(fixed,totalRemaining=1))
        self.assertEqual(trial.run()['status'],'abstained');self.assertEqual(ports.sent,[])

    def test_one_command_policy_keeps_initial_support_fence_and_residual_checks(self):
        fixed=observed95_model(minimum=100.,maximum=130.,delta=10.)
        trial,ports,_=self.trial([99.9],fixed=fixed,planned=observed95_scope(fixed,maxCommands=1))
        self.assertEqual(trial.run()['status'],'failed');self.assertEqual(ports.sent,[])
        trial,ports,_=self.trial([101.,87.],fixed=fixed,planned=observed95_scope(fixed,maxCommands=1))
        result=trial.run()
        self.assertEqual(result['status'],'prediction_residual_exceeded');self.assertEqual(len(ports.sent),1)
        fixed=observed95_model(minimum=102.,maximum=130.,delta=10.)
        trial,ports,_=self.trial([101.],fixed=fixed,planned=observed95_scope(fixed,maxCommands=1))
        self.assertEqual(trial.run()['status'],'abstained');self.assertEqual(ports.sent,[])
        fixed=observed95_model(delta=20.)
        trial,ports,_=self.trial([101.],fixed=fixed,planned=observed95_scope(fixed,maxCommands=1))
        result=trial.run();self.assertEqual(result['status'],'abstained');self.assertEqual(ports.sent,[])
        self.assertEqual(trial._choose(101.)['rejected'][0]['reason'],'M1_predicted_shaft_outside_fence')

    def test_initial_goal_or_start_outside_declared_interval_never_counts_as_success(self):
        fixed=observed95_model()
        for value in (85.3,99.9,105.1):
            trial,ports,_=self.trial([value],fixed=fixed,planned=observed95_scope(fixed))
            result=trial.run()
            with self.subTest(value=value):
                self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
                self.assertFalse(result['shaftGoalReachedOnly'])
                self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_independent_start_required')

    def test_actual_next_state_outside_support_stops_without_automatic_replacement(self):
        fixed=observed95_model(minimum=93.,delta=8.)
        trial,ports,_=self.trial([101.,92.9],fixed=fixed,planned=observed95_scope(fixed))
        result=trial.run()
        self.assertEqual(result['status'],'abstained',result);self.assertEqual(len(ports.sent),1)
        self.assertEqual(result['confirmedCommands'],1)

    def test_new_scope_is_exact_and_registration_bound_without_legacy_relaxation(self):
        fixed=observed95_model();valid=observed95_scope(fixed)
        changes=[dict(geometryPolicy='unknown'),dict(goalSemantics='grasp'),
            dict(observationRegistrationSha256='f'*64),dict(startIntervalPx=[99.,105.]),
            dict(startIntervalPx=[105.,105.]),dict(startIntervalPx=[100.,138.]),
            dict(startIntervalPx=[True,105.]),dict(shaftMinimum=83.),dict(targetShaftPx=85.4),
            dict(maximumPredictionResidualPx=4.),dict(extra=True)]
        for change in changes:
            with self.subTest(change=change),self.assertRaises(ValueError):
                self.trial([101.],fixed=fixed,planned=dict(valid,**change))
        with self.assertRaises(ValueError):
            self.trial([101.],fixed=fixed,planned=scope(shaftMinimum=84.,shaftMaximum=139.5,targetShaftPx=85.3))
        self.assertEqual(valid,observed95_scope(fixed))

    def test_only_real_model_60_450_is_admitted_and_new_fence_still_stops(self):
        fixed=observed95_model(duration=650)
        with self.assertRaisesRegex(ValueError,'supported_close_candidates_required'):
            self.trial([101.],fixed=fixed,planned=observed95_scope(fixed))
        fixed=observed95_model()
        trial,ports,_=self.trial([101.,83.9],fixed=fixed,planned=observed95_scope(fixed))
        result=trial.run()
        self.assertEqual(result['status'],'failed');self.assertEqual(len(ports.sent),1)
        self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_observed_shaft_outside_fence')


def lower_model(*, short_delta=5., short_minimum=89., short_maximum=97., short_duration=200):
    rows=[]
    for duration,minimum,maximum,delta in ((350,100.,113.,10.),
                                          (short_duration,short_minimum,short_maximum,short_delta)):
        for value in (minimum,maximum):
            row=scalar_sample(f'lower-{duration}-{value}','lower-training',value)
            row.update(command=dict(joint='gripper',direction='close',percent=60,durationMs=duration),
                       afterShaftPx=value-delta)
            rows.append(row)
    return learning.fit(rows)


class LowerObserved95TrialTests(unittest.TestCase):
    trial=M1ShaftTrialTests.trial

    def planned(self,fixed,**changes):
        return observed95_scope(fixed,geometryPolicy=OBSERVED95_LOWER_POLICY,**changes)

    def test_250_bridge_requires_actual_training_and_300_remains_unsupported(self):
        fixed=observed95_model(minimum=100.,maximum=113.,delta=15.,duration=250)
        trial,ports,_=self.trial([101.,86.],fixed=fixed,planned=self.planned(fixed))
        result=trial.run()
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual([c['durationMs'] for c in ports.sent],[250])
        # 후보 기간을 추가해도 학습되지 않은 250 명령을 만들어내지 않는다.
        fixed=lower_model()
        trial,_,_=self.trial([105.],fixed=fixed,planned=self.planned(fixed))
        self.assertNotIn(250,[c['durationMs'] for c in trial.commands])
        for duration in (100,249,251,300):
            fixed=observed95_model(duration=duration)
            with self.subTest(duration=duration),self.assertRaisesRegex(ValueError,'supported_close_candidates_required'):
                self.trial([101.],fixed=fixed,planned=self.planned(fixed))

    def test_350_can_bridge_to_supported_250_after_fresh_observation(self):
        fixed=lower_model(short_duration=250)
        trial,ports,_=self.trial([105.,95.,90.,85.],fixed=fixed,planned=self.planned(fixed))
        result=trial.run()
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual([c['durationMs'] for c in ports.sent],[350,250,250])
        self.assertEqual(len(ports.records),4)

    def test_three_commands_bridge_from_350_to_supported_200_and_reobserve(self):
        fixed=lower_model();trial,ports,_=self.trial([105.,95.,90.,85.],fixed=fixed,planned=self.planned(fixed))
        result=trial.run()
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual([c['durationMs'] for c in ports.sent],[350,200,200])
        self.assertEqual(result['confirmedCommands'],3);self.assertEqual(len(ports.records),4)
        self.assertFalse(result['graspVerified'])

    def test_next_command_requires_own_support_safe_prediction_and_minimum_gain(self):
        for options in (dict(short_minimum=96.,short_maximum=99.),
                        dict(short_delta=20.),dict(short_delta=.25),dict(short_delta=0.)):
            fixed=lower_model(**options)
            trial,ports,_=self.trial([105.],fixed=fixed,planned=self.planned(fixed))
            result=trial.run()
            with self.subTest(options=options):
                self.assertEqual(result['status'],'abstained',result);self.assertEqual(ports.sent,[])
                self.assertIn('M1_predicted_nonterminal_shaft_outside_support',
                              [item['reason'] for item in trial._choose(105.)['rejected']])

    def test_actual_bridge_state_outside_next_support_stops_without_substitution(self):
        fixed=lower_model(short_minimum=95.,short_maximum=97.)
        trial,ports,_=self.trial([105.,94.9],fixed=fixed,planned=self.planned(fixed))
        result=trial.run()
        self.assertEqual(result['status'],'abstained',result);self.assertEqual(len(ports.sent),1)

    def test_lower_start_interval_remains_strictly_outside_goal_and_registration_bound(self):
        fixed=lower_model()
        trial,_,_=self.trial([90.],fixed=fixed,planned=self.planned(fixed,startIntervalPx=[86.8001,99.]))
        self.assertEqual(trial.scope['startIntervalPx'],[86.8001,99.])
        for change in (dict(startIntervalPx=[86.8,99.]),dict(startIntervalPx=[85.3,99.]),
                       dict(startIntervalPx=[99.,138.]),dict(observationRegistrationSha256='f'*64)):
            with self.subTest(change=change),self.assertRaises(ValueError):
                self.trial([90.],fixed=fixed,planned=self.planned(fixed,**change))
        with self.assertRaisesRegex(ValueError,'supported_close_candidates_required'):
            self.trial([105.],fixed=fixed,planned=observed95_scope(fixed))
        # 기존 450 후보는 새 정책에서도 실제 학습된 경우에만 사용할 수 있다.
        fixed=observed95_model()
        trial,_,_=self.trial([101.],fixed=fixed,planned=self.planned(fixed,maxCommands=1))
        self.assertEqual([c['durationMs'] for c in trial.commands],[450])


class ReviewedLowerTrialTests(unittest.TestCase):
    trial=M1ShaftTrialTests.trial

    def planned(self,fixed,**changes):
        planned=observed95_scope(fixed,geometryPolicy=OBSERVED95_REVIEWED_LOWER_POLICY,maxSeconds=120.)
        planned.update(changes);return planned

    def test_only_explicit_reviewed_policy_gets_120_second_budget(self):
        fixed=lower_model()
        trial,_,_=self.trial([105.],fixed=fixed,planned=self.planned(fixed))
        self.assertEqual(trial.scope['maxSeconds'],120.)
        for changed in (self.planned(fixed,maxSeconds=121.),
                        observed95_scope(fixed,geometryPolicy=OBSERVED95_LOWER_POLICY,maxSeconds=120.),
                        observed95_scope(fixed,maxSeconds=46.),scope(maxSeconds=46.),
                        self.planned(fixed,maxCommands=5),self.planned(fixed,totalRemaining=5)):
            with self.subTest(changed=changed),self.assertRaises(ValueError):
                self.trial([105.],fixed=fixed,planned=changed)

    def test_long_review_workflow_uses_new_fresh_frames_without_extending_original_deadline(self):
        fixed=lower_model()
        for reviewed in (False,True):
            planned=self.planned(fixed) if reviewed else observed95_scope(fixed,geometryPolicy=OBSERVED95_LOWER_POLICY)
            trial,ports,_=self.trial([105.,95.,90.,85.],fixed=fixed,planned=planned)
            ports.observation_delay=16.
            result=trial.run()
            with self.subTest(reviewed=reviewed):
                self.assertEqual(result['status'],'observed_target_reached' if reviewed else 'failed',result)
                self.assertEqual(trial.deadline-trial.started,120. if reviewed else 45.)
                if reviewed:
                    self.assertEqual([c['durationMs'] for c in ports.sent],[350,200,200])
                    self.assertGreater(ports.clock.now-trial.started,45.)
                else:self.assertIn('M1_scalar_trial_deadline',[e['reason'] for e in result['errors']])

    def test_long_total_budget_does_not_relax_dispatch_source_freshness_or_binding(self):
        fixed=lower_model()
        trial,ports,_=self.trial([105.],fixed=fixed,planned=self.planned(fixed))
        ports.preparation_delay=3.1
        result=trial.run()
        self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
        self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_stale_or_noncausal_source')
        trial,ports,_=self.trial([105.],fixed=fixed,planned=self.planned(fixed))
        def mutate(command,decision):decision['selected']['command']['durationMs']=450
        ports.mutate=mutate
        result=trial.run()
        self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
        self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_dispatch_binding_changed')


class Grasp650TrialTests(unittest.TestCase):
    trial=M1ShaftTrialTests.trial

    def fixed(self,duration=650):
        return observed95_model(minimum=80.,maximum=100.,delta=10.,duration=duration)

    def planned(self,fixed,**changes):
        planned=observed95_scope(fixed,geometryPolicy=OBSERVED95_GRASP650_POLICY,
            shaftMinimum=66.,targetShaftPx=75.3,startIntervalPx=[80.,90.],maxCommands=1,totalRemaining=1,
            goalSemantics=OBSERVED95_CONTACT_GOAL,goalReferenceSha256='c'*64)
        planned.update(changes);return planned

    def test_explicit_single_650_reproduction_does_not_infer_contact_or_grasp(self):
        fixed=self.fixed();trial,ports,store=self.trial([85.3,75.3],fixed=fixed,planned=self.planned(fixed))
        result=trial.run()
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual([c['durationMs'] for c in ports.sent],[650])
        self.assertEqual(result['confirmedCommands'],1);self.assertEqual(len(ports.records),2)
        self.assertFalse(result['contactVerified']);self.assertFalse(result['graspVerified'])
        self.assertEqual(store.rows['trial-intent.json']['scope']['goalReferenceSha256'],'c'*64)

    def test_separate_after_model_can_select_bounded_saturation_without_clamping_old_prediction(self):
        rows=[]
        for index,value in enumerate((80.,116.)):
            row=scalar_sample(f'clamped-trial-{index}','clamped-trial-training',value)
            row.update(command=dict(joint='gripper',direction='close',percent=60,durationMs=650),afterShaftPx=68.)
            rows.append(row)
        fixed=learning.fit_after(rows)
        planned=self.planned(fixed,targetShaftPx=68.,startIntervalPx=[79.,90.])
        trial,ports,_=self.trial([80.,68.],fixed=fixed,planned=planned)
        result=trial.run()
        self.assertEqual(result['status'],'observed_target_reached',result)
        self.assertEqual(result['steps'][0]['predictedShaftSeparationPx'],68.)
        self.assertEqual(result['steps'][0]['commandMeanPredictedShaftSeparationPx'],50.)
        self.assertEqual(len(ports.sent),1);self.assertFalse(result['graspVerified'])
        old=learning.fit(rows)
        trial,ports,_=self.trial([80.],fixed=old,planned=planned)
        self.assertEqual(trial.run()['status'],'abstained');self.assertEqual(ports.sent,[])

    def test_after_schema_requires_explicit_grasp_geometry_without_automatic_live_admission(self):
        rows=[]
        for index,value in enumerate((80.,116.)):
            row=scalar_sample(f'after-policy-{index}','after-policy-training',value)
            row.update(command=dict(joint='gripper',direction='close',percent=60,durationMs=650),afterShaftPx=68.)
            rows.append(row)
        fixed=learning.fit_after(rows)
        plans=[scope(),observed95_scope(fixed),
            observed95_scope(fixed,geometryPolicy=OBSERVED95_LOWER_POLICY),
            observed95_scope(fixed,geometryPolicy=OBSERVED95_REVIEWED_LOWER_POLICY,maxSeconds=120.)]
        for planned in plans:
            with self.subTest(planned=planned),self.assertRaisesRegex(ValueError,'requires_explicit_grasp650_policy'):
                self.trial([100.],fixed=fixed,planned=planned)

    def test_contact_scope_requires_reference_single_budget_45_seconds_and_start_outside_goal(self):
        fixed=self.fixed();valid=self.planned(fixed)
        changes=[dict(goalReferenceSha256='bad'),dict(goalSemantics=OBSERVED95_GOAL),
            dict(maxCommands=2),dict(totalRemaining=2),dict(maxSeconds=46.),dict(shaftMinimum=65.),
            dict(targetShaftPx=65.9),dict(targetShaftPx=139.6),dict(startIntervalPx=[76.8,90.]),
            dict(startIntervalPx=[80.,138.]),dict(observationRegistrationSha256='f'*64)]
        for change in changes:
            with self.subTest(change=change),self.assertRaises(ValueError):
                self.trial([85.3],fixed=fixed,planned=dict(valid,**change))
        missing=dict(valid);missing.pop('goalReferenceSha256')
        with self.assertRaisesRegex(ValueError,'scope_fields'):
            self.trial([85.3],fixed=fixed,planned=missing)
        for duration in (200,250,350,450):
            fixed=self.fixed(duration)
            with self.subTest(duration=duration),self.assertRaisesRegex(ValueError,'supported_close_candidates_required'):
                self.trial([85.3],fixed=fixed,planned=self.planned(fixed))

    def test_goal_reference_digest_is_frozen_and_external_confirmation_failure_blocks_all_commands(self):
        fixed=self.fixed();trial,ports,_=self.trial([85.3],fixed=fixed,planned=self.planned(fixed))
        trial.scope['goalReferenceSha256']='d'*64
        result=trial.run();self.assertEqual(result['status'],'failed');self.assertEqual(ports.sent,[])
        self.assertEqual(result['errors'][0]['reason'],'M1_scalar_trial_frozen_contract_changed')
        trial,ports,_=self.trial([85.3],fixed=fixed,planned=self.planned(fixed))
        def unconfirmed():raise ValueError('contact_reference_not_confirmed')
        trial.verify_assets=unconfirmed
        result=trial.run();self.assertEqual(result['status'],'failed')
        self.assertEqual(ports.records,[]);self.assertEqual(ports.sent,[])
        self.assertEqual(result['errors'][0]['reason'],'contact_reference_not_confirmed')

    def test_initial_support_new_floor_and_dispatch_freshness_remain_checked(self):
        fixed=self.fixed()
        for states,planned,delay,reason in (
            ([79.9],self.planned(fixed,startIntervalPx=[79.,90.]),.11,None),
            ([85.3,65.9],self.planned(fixed),.11,'M1_scalar_trial_observed_shaft_outside_fence'),
            ([85.3],self.planned(fixed),3.1,'M1_scalar_trial_stale_or_noncausal_source')):
            trial,ports,_=self.trial(states,fixed=fixed,planned=planned);ports.preparation_delay=delay
            result=trial.run()
            if reason is None:
                self.assertEqual(result['status'],'abstained');self.assertEqual(ports.sent,[])
            else:
                self.assertEqual(result['status'],'failed',result)
                self.assertEqual(result['errors'][0]['reason'],reason)


if __name__=='__main__':unittest.main()
