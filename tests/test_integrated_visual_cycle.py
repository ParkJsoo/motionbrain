import unittest

from learning.integrated_visual_cycle import IntegratedVisualCycle


class Clock:
    def __init__(self): self.now = 10.
    def monotonic(self): return self.now


def command(phase):
    joint, direction = dict(approach=('elbow', 'down'), close=('gripper', 'close'),
        lift=('elbow', 'up'), lower=('elbow', 'down'), release=('gripper', 'open'))[phase]
    return dict(joint=joint, direction=direction, percent=70, durationMs=1000)


class Hooks:
    def __init__(self):
        self.clock = Clock()
        self.calls = []
        self.serial = 0
        self.approaches = 0
        self.close_count = 3
        self.captured_permit = None

    def observe(self, phase):
        self.calls.append('observe:'+phase)
        self.serial += 1
        return dict(observationId=str(self.serial), observedAt=self.clock.now,
                    twoCameraRawAudit=True)

    def choose_approach(self, observation):
        if self.approaches:
            self.approaches -= 1
            return dict(selected=dict(kind='action', command=command('approach')))
        return dict(selected=dict(kind='stop'), stopReason='current_near_gripper_phase')

    def action(self, phase, permit, n=1):
        self.calls.append(phase)
        self.captured_permit = permit
        receipts = []
        for _ in range(n):
            cmd = command(phase)
            token = permit(cmd)
            self.clock.now += 2.
            receipt = dict(status='completed', offVerified=True, command=cmd, rawReceiptSha='bound-by-adapter')
            permit.complete(token, receipt)
            receipts.append(receipt)
        return dict(status='completed', modelSelectedCommands=n, commandReceipts=receipts)

    def execute_approach(self, decision, permit): return self.action('approach', permit)
    def grasp_ready(self, obs): return self.gate('grasp_ready')
    def close(self, obs, permit):
        result = self.action('close', permit, self.close_count)
        result.update(afterDecision=dict(selected=dict(kind='stop')), closeReady=self.gate('closeReady'))
        return result
    def lift(self, obs, permit): return self.action('lift', permit)
    def lower(self, obs, permit): return self.action('lower', permit)
    def release(self, obs, permit): return self.action('release', permit)
    def retained_lift(self, obs, result): return self.gate('retained_lift')
    def table_supported(self, obs, result): return self.gate('table_supported')
    def released(self, obs, result): return self.gate('released')
    def gate(self, name):
        self.calls.append(name)
        return dict(accepted=True, evidence=dict(rawAssessment=name))
    def cleanup(self): self.calls.append('cleanup')
    def callbacks(self):
        names = ('observe', 'choose_approach', 'execute_approach', 'grasp_ready', 'close', 'lift',
                 'retained_lift', 'lower', 'table_supported', 'release', 'released', 'cleanup')
        return {name: getattr(self, name) for name in names}


class IntegratedVisualCycleTests(unittest.TestCase):
    def setUp(self):
        self.h = Hooks()
        self.cb = self.h.callbacks()

    def run_cycle(self, **kwargs):
        return IntegratedVisualCycle(self.cb, clock=self.h.clock, **kwargs).run()

    def test_one_cycle_advances_only_after_independent_gates(self):
        report = self.run_cycle()
        self.assertEqual(report['status'], 'completed')
        self.assertEqual(report['commandAttempts'], 6)
        self.assertEqual(report['modelSelectedCommands'], 6)
        self.assertEqual(report['unconfirmedCommands'], 0)
        self.assertEqual(report['completedAttempts'], 1)
        self.assertEqual(report['failedAttempts'], 0)
        self.assertTrue(all(report[k] for k in ('graspImageAssessmentPassed', 'retainedLiftImageAssessmentPassed', 'tableReturnImageAssessmentPassed', 'releaseImageAssessmentPassed')))
        self.assertFalse(report['callbackEvidenceIndependentlyAudited'])
        self.assertEqual([(t['before'],t['after']) for t in report['phaseTransitions']],
                         [('approach','close'),('close','lift'),('lift','lower'),('lower','release')])
        self.assertLess(self.h.calls.index('retained_lift'), self.h.calls.index('lower'))
        self.assertLess(self.h.calls.index('table_supported'), self.h.calls.index('release'))
        self.assertEqual(self.h.calls[-1], 'cleanup')

    def test_approach_reobserves_and_accumulates_stage_limit(self):
        self.h.approaches = 9
        report = self.run_cycle()
        self.assertEqual(report['commandAttempts'], 4)
        self.assertEqual(report['modelSelectedCommands'], 4)
        self.assertIn('integrated_command_budget', report['errors'][0]['reason'])
        self.assertNotIn('close', self.h.calls)

    def test_model_selected_M1_OPEN_is_one_approach_action_with_fresh_reobservation(self):
        decisions=[]
        def choose(obs):
            decisions.append(obs['observationId'])
            return (dict(selected=dict(kind='action',command=command('release'))) if len(decisions)==1
                    else dict(selected=dict(kind='stop'),stopReason='current_near_gripper_phase'))
        self.cb['choose_approach']=choose
        self.cb['execute_approach']=lambda decision,permit:self.h.action('release',permit)
        report=self.run_cycle()
        self.assertEqual(report['status'],'completed')
        self.assertTrue(report['schemaVersion'].endswith('.v3'))
        self.assertEqual(report['commands'][0]['phase'],'approach')
        self.assertEqual(report['commands'][0]['command'],command('release'))
        self.assertEqual(len(decisions),2)
        self.assertNotEqual(decisions[0],decisions[1])
        self.assertEqual(report['modelSelectedCommands'],7)

    def test_approach_does_not_allow_M1_CLOSE(self):
        self.cb['choose_approach']=lambda obs:dict(selected=dict(kind='action',command=command('close')))
        report=self.run_cycle()
        self.assertEqual(report['commandAttempts'],0)
        self.assertEqual(report['status'],'failed')

    def test_release_connects_up_to_three_single_action_transactions(self):
        self.cb['release']=lambda obs,permit:self.h.action('release',permit,3)
        report=self.run_cycle()
        self.assertEqual(report['status'],'completed')
        self.assertEqual(report['modelSelectedCommands'],8)
        self.assertEqual(sum(row['phase']=='release' for row in report['commands']),3)
        policy=report['releaseMicrotransactionPolicy']
        self.assertEqual(policy['maximumMicrotransactions'],3)
        self.assertEqual(policy['commandsPerMicrotransaction'],1)
        self.assertFalse(policy['originalSingleActionModelPolicyChanged'])
        self.assertFalse(policy['microtransactionBudgetStopIsReleaseSuccess'])
        self.assertTrue(policy['finalReleasedImageAssessmentRequired'])

    def test_fourth_release_is_denied_before_RUN_and_not_success(self):
        self.cb['release']=lambda obs,permit:self.h.action('release',permit,4)
        report=self.run_cycle()
        self.assertEqual(report['status'],'failed')
        self.assertEqual(report['modelSelectedCommands'],8)
        self.assertEqual(report['unconfirmedCommands'],0)
        self.assertFalse(report['releaseImageAssessmentPassed'])
        self.assertNotIn('released',self.h.calls)

    def test_global_budget_still_stops_release_inside_three_transaction_allowance(self):
        self.cb['release']=lambda obs,permit:self.h.action('release',permit,3)
        report=self.run_cycle(max_commands=7)
        self.assertEqual(report['status'],'failed')
        self.assertEqual(report['commandAttempts'],7)
        self.assertEqual(report['modelSelectedCommands'],7)
        self.assertFalse(report['releaseImageAssessmentPassed'])

    def test_release_callback_failure_after_one_transaction_preserves_actual_count(self):
        def release(obs,permit):
            self.h.action('release',permit)
            raise ValueError('fresh_table_assessment_rejected')
        self.cb['release']=release
        report=self.run_cycle()
        self.assertEqual(report['status'],'failed')
        self.assertEqual(report['modelSelectedCommands'],6)
        self.assertFalse(report['releaseImageAssessmentPassed'])
        self.assertNotIn('released',self.h.calls)

    def test_three_release_transactions_do_not_replace_final_released_gate(self):
        self.cb['release']=lambda obs,permit:self.h.action('release',permit,3)
        self.cb['released']=lambda *args:dict(accepted=False,evidence={'stillBetweenJaws':True})
        report=self.run_cycle()
        self.assertEqual(report['modelSelectedCommands'],8)
        self.assertEqual(report['status'],'failed')
        self.assertFalse(report['releaseImageAssessmentPassed'])

    def test_lift_still_only_one_command_and_global_cap_is_twelve(self):
        self.cb['lift']=lambda obs,permit:self.h.action('lift',permit,2)
        report=self.run_cycle()
        self.assertEqual(report['modelSelectedCommands'],4)
        self.assertEqual(report['status'],'failed')
        self.assertNotIn('lower',self.h.calls)
        with self.assertRaises(ValueError):IntegratedVisualCycle(self.cb,max_commands=13)

    def test_global_budget_rejects_before_next_run_and_keeps_completed_count(self):
        report = self.run_cycle(max_commands=3)
        self.assertEqual(report['phase'], 'lift')
        self.assertEqual(report['commandAttempts'], 3)
        self.assertEqual(report['modelSelectedCommands'], 3)
        self.assertEqual(report['failedAttempts'], 1)
        self.assertFalse(report['graspImageAssessmentPassed'])
        self.assertNotIn('lower', self.h.calls)

    def test_unknown_run_never_retries_or_enters_lift(self):
        def close(obs, permit):
            permit(command('close'))
            raise ValueError('original_unknown_RUN')
        self.cb['close'] = close
        report = self.run_cycle()
        self.assertEqual(report['commandAttempts'], 1)
        self.assertEqual(report['unconfirmedCommands'], 1)
        self.assertEqual(report['commands'][0]['status'], 'unconfirmed')
        self.assertNotIn('lift', self.h.calls)
        self.assertFalse(report['automaticRetry'])

    def test_complete_before_later_callback_failure_preserves_denominator(self):
        def close(obs, permit):
            self.h.action('close', permit)
            raise ValueError('observation_failed')
        self.cb['close'] = close
        report = self.run_cycle()
        self.assertEqual(report['modelSelectedCommands'], 1)
        self.assertEqual(report['unconfirmedCommands'], 0)
        self.assertNotIn('lift', self.h.calls)

    def test_OFF_false_blocks_followup(self):
        def close(obs, permit):
            cmd = command('close')
            token = permit(cmd)
            permit.complete(token, dict(command=cmd,status='completed',offVerified=False))
        self.cb['close'] = close
        report = self.run_cycle()
        self.assertEqual(report['unconfirmedCommands'], 1)
        self.assertNotIn('lift', self.h.calls)

    def test_reservation_is_single_pending_and_completion_single_use(self):
        for duplicate in ('reserve', 'complete'):
            with self.subTest(duplicate=duplicate):
                h = Hooks()
                def close(obs, permit):
                    cmd = command('close'); token = permit(cmd)
                    if duplicate == 'reserve': permit(cmd)
                    receipt = dict(command=cmd,status='completed',offVerified=True)
                    permit.complete(token, receipt)
                    permit.complete(token, receipt)
                callbacks = h.callbacks(); callbacks['close'] = close
                report = IntegratedVisualCycle(callbacks, clock=h.clock).run()
                self.assertEqual(report['status'], 'failed')
                self.assertEqual(report['commandAttempts'], 1)
                self.assertNotIn('lift', h.calls)

    def test_permit_closed_after_callback(self):
        self.run_cycle()
        with self.assertRaisesRegex(ValueError, 'permit_closed'):
            self.h.captured_permit(command('release'))

    def test_stop_not_success_without_explicit_scene_reason_and_gate(self):
        for reason in (None, 'unsupported', 'budget_reached'):
            with self.subTest(reason=reason):
                self.cb['choose_approach'] = lambda obs: dict(selected=dict(kind='stop'), stopReason=reason)
                report = self.run_cycle()
                self.assertEqual(report['commandAttempts'], 0)
                self.assertNotIn('close', self.h.calls)

    def test_each_rejected_physical_gate_blocks_next_phase(self):
        for gate, forbidden in (('grasp_ready','close'),('retained_lift','lower'),
                                ('table_supported','release'),('released',None)):
            with self.subTest(gate=gate):
                h = Hooks(); callbacks = h.callbacks()
                callbacks[gate] = lambda *args: dict(accepted=False,evidence={'review':'rejected'})
                report = IntegratedVisualCycle(callbacks, clock=h.clock).run()
                self.assertEqual(report['status'], 'failed')
                if forbidden: self.assertNotIn(forbidden, h.calls)
                self.assertFalse(report['releaseImageAssessmentPassed'])

    def test_close_original_decision_and_closeReady_both_required(self):
        original = self.cb['close']
        for field in ('afterDecision','closeReady'):
            def close(obs, permit):
                result = original(obs, permit); result.pop(field)
                return result
            self.cb['close'] = close
            report = self.run_cycle()
            self.assertEqual(report['status'], 'failed')
            self.assertNotIn('lift', self.h.calls)

    def plateau_close(self, count=2, patch=None, ready=True):
        def close(obs, permit):
            result = self.h.action('close', permit, count)
            # 마지막 폐합 직전/직후 실제 관측을 어댑터가 결합하는 형태.
            evidence = dict(previousGapPx=64., currentGapPx=63.8,
                previousObservedAt=self.h.clock.now-1., currentObservedAt=self.h.clock.now,
                previousObservationId='before-last-close', currentObservationId='after-last-close')
            if patch: evidence.update(patch)
            result.update(afterDecision=dict(selected=dict(kind='action',command=command('close'))),
                closeTaskTermination=dict(reason='image_candidate_with_observed_plateau',evidence=evidence),
                closeReady=dict(accepted=ready,evidence={'fixedViewCandidate':True}))
            return result
        return close

    def test_image_plateau_after_two_close_advances_preserving_model_ACTION(self):
        self.cb['close'] = self.plateau_close()
        report = self.run_cycle()
        self.assertEqual(report['status'],'completed')
        self.assertEqual(report['modelSelectedCommands'],5)
        self.assertEqual(report['closeTermination']['owner'],'upper_image_plateau_rule')
        self.assertFalse(report['closeTermination']['modelSelectedStop'])
        result = next(row['result'] for row in report['stageResults'] if row['phase']=='close')
        self.assertEqual(result['afterDecision']['selected']['kind'],'action')
        self.assertTrue(report['graspImageAssessmentPassed'])

    def test_plateau_cannot_substitute_for_second_close_or_image_candidate(self):
        for count, ready in ((1,True),(2,False)):
            with self.subTest(count=count, ready=ready):
                h = Hooks(); self.h=h; self.cb=h.callbacks()
                self.cb['close'] = self.plateau_close(count=count,ready=ready)
                report = self.run_cycle()
                self.assertEqual(report['status'],'failed')
                self.assertNotIn('lift',h.calls)

    def test_plateau_rejects_progress_missing_reused_regressed_stale_nonfinite_evidence(self):
        patches = [dict(currentGapPx=61.9), dict(currentGapPx=-1), dict(currentGapPx=float('nan')),
            dict(previousGapPx=float('inf')), dict(previousObservationId='after-last-close'),
            dict(previousObservationId=''), dict(previousObservedAt=14., currentObservedAt=13.),
            dict(currentObservedAt=15.), dict(previousObservedAt=-50.), dict(currentObservedAt=True),
            dict(unboundExtra='not-in-contract')]
        for patch in patches:
            with self.subTest(patch=patch):
                h = Hooks(); self.h=h; self.cb=h.callbacks()
                self.cb['close'] = self.plateau_close(patch=patch)
                report = self.run_cycle()
                self.assertEqual(report['status'],'failed')
                self.assertNotIn('lift',h.calls)

    def test_plateau_exact_two_pixel_boundary_is_declared_rule(self):
        self.cb['close'] = self.plateau_close(patch=dict(currentGapPx=62.))
        report = self.run_cycle()
        self.assertEqual(report['status'],'completed')
        self.assertFalse(report['closeTermination']['modelSelectedStop'])

    def test_model_stop_remains_distinct_from_upper_rule(self):
        report = self.run_cycle()
        self.assertEqual(report['closeTermination'],dict(owner='model_stop',modelSelectedStop=True))

    def test_wrong_approach_command_rejected_before_reservation(self):
        self.h.approaches = 1
        def execute(decision, permit):
            cmd = command('approach'); cmd['direction'] = 'up'
            permit(cmd)
        self.cb['execute_approach'] = execute
        report = self.run_cycle()
        self.assertEqual(report['commandAttempts'], 0)
        self.assertIn('selection_changed', report['errors'][0]['reason'])

    def test_out_of_cohort_command_rejected(self):
        def close(obs, permit):
            cmd = command('close'); cmd['durationMs'] = 650
            permit(cmd)
        self.cb['close'] = close
        report = self.run_cycle()
        self.assertEqual(report['commandAttempts'], 0)

    def test_stale_future_reused_observations_rejected(self):
        for mode in ('stale','future','reused'):
            with self.subTest(mode=mode):
                h = Hooks(); cb = h.callbacks(); original = cb['observe']
                def observe(phase):
                    obs = original(phase)
                    if mode == 'stale': obs['observedAt'] -= 31.
                    if mode == 'future': obs['observedAt'] += .1
                    if mode == 'reused': obs['observationId'] = 'same'
                    return obs
                cb['observe'] = observe
                report = IntegratedVisualCycle(cb,clock=h.clock).run()
                self.assertEqual(report['status'],'failed')
                self.assertEqual(report['commandAttempts'],0)

    def test_deadline_after_verified_output_keeps_actual_OFF_count(self):
        def close(obs, permit):
            cmd = command('close'); token = permit(cmd)
            self.h.clock.now += 900.
            permit.complete(token,dict(command=cmd,status='completed',offVerified=True))
        self.cb['close'] = close
        report = self.run_cycle()
        self.assertEqual(report['modelSelectedCommands'],1)
        self.assertNotIn('lift',self.h.calls)

    def test_slow_choice_or_gate_cannot_advance_using_stale_scene(self):
        for callback in ('choose_approach','grasp_ready'):
            with self.subTest(callback=callback):
                h = Hooks(); cb = h.callbacks(); original = cb[callback]
                def slow(*args):
                    h.clock.now += 31.
                    return original(*args)
                cb[callback] = slow
                report = IntegratedVisualCycle(cb,clock=h.clock).run()
                self.assertEqual(report['status'],'failed')
                self.assertNotIn('close',h.calls)

    def test_stage_result_cannot_invent_or_drop_commands(self):
        def close(obs, permit):
            result = self.h.close(obs,permit)
            result['modelSelectedCommands'] = 99
            return result
        self.cb['close'] = close
        report = self.run_cycle()
        self.assertEqual(report['modelSelectedCommands'],3)
        self.assertNotIn('lift',self.h.calls)

    def test_cleanup_failure_and_interrupt_not_success(self):
        self.cb['cleanup'] = lambda: (_ for _ in ()).throw(ValueError('OFF_failed'))
        report = self.run_cycle()
        self.assertEqual(report['status'],'failed')
        self.assertEqual(report['completedAttempts'],0)
        self.assertEqual(report['failedAttempts'],1)
        self.cb['cleanup'] = self.h.cleanup
        self.cb['choose_approach'] = lambda obs: (_ for _ in ()).throw(KeyboardInterrupt())
        report = self.run_cycle()
        self.assertEqual(report['status'],'interrupted')
        self.assertEqual(report['failedAttempts'],1)

    def test_cycle_is_single_use_and_configuration_finite(self):
        cycle = IntegratedVisualCycle(self.cb,clock=self.h.clock)
        cycle.run()
        with self.assertRaisesRegex(ValueError,'single_use'): cycle.run()
        for value in (float('nan'),float('inf'),True,0,901):
            with self.assertRaises(ValueError): IntegratedVisualCycle(self.cb,max_seconds=value)


if __name__ == '__main__':
    unittest.main()
