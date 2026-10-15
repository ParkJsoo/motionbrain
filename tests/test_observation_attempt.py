"""Finite request boundaries and reused real probe failure paths (virtual clock)."""
import json
import unittest
from unittest.mock import patch

from learning.observation_attempt import ObservationAttempt, AttemptStopped
from learning.artifacts import ArtifactStore, read_events, verify_artifacts
from learning.fast_observation import _match_candidates
import test_causal_anchor_age as causal
from learning.dry_run import VirtualClock as Clock


class BudgetTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.a = ObservationAttempt('request-1', self.clock)
        self.a.anchor_ready()

    def test_exact_last_budget_success_and_no_further_capture(self):
        for _ in range(5): self.a.capture()
        self.clock.advance(3)
        self.assertTrue(self.a.finish_frame({}, {'storageStatus':'completed', 'consumed':True,
                                                'consumedMonotonic':self.clock.monotonic()}))
        self.assertEqual(self.a.snapshot()['operationElapsedMs'],3000)
        with self.assertRaises(AttemptStopped): self.a.capture()

    def test_fifth_rejection_or_sixth_capture_stops(self):
        for _ in range(5): self.a.capture()
        with self.assertRaisesRegex(AttemptStopped,'capture_budget_exhausted'):
            self.a.finish_frame({'reason':'source_age_exceeds_policy'},
                {'storageStatus':'completed','consumed':False,'reason':'measurement_rejected'})
        with self.assertRaises(AttemptStopped): self.a.capture()

    def test_budget_overrun_and_time_regression(self):
        self.clock.advance(3.001)
        with self.assertRaisesRegex(AttemptStopped,'operation_budget'): self.a.check()
        self.clock._seconds -= 1
        with self.assertRaisesRegex(AttemptStopped,'regressed'): self.a.check()


class AttemptProbeTest(unittest.TestCase):
    setUp = causal.CausalProbeTest.setUp
    configure = causal.CausalProbeTest.configure
    run_probe = causal.CausalProbeTest.run_probe
    run_causal = causal.CausalProbeTest.run_causal

    def run_attempt(self):
        report = self.run_causal(request_id='request-1')
        self.assertEqual(verify_artifacts(self.root,report['artifacts']),[])
        events, errors = read_events(self.root)
        self.assertEqual(errors,[])
        self.assertEqual(events[-1]['kind'],'finished')
        self.assertEqual(report['motorCommandsSent'],0)
        return report

    def test_first_success_ends_request(self):
        self.configure(); report=self.run_attempt()
        self.assertEqual(report['observationAttempt']['status'],'success',report)
        self.assertEqual(self.capture_count,2)
        self.assertEqual(report['observationAttempt']['currentCaptures'],1)

    def test_only_age_miss_retries_and_fifth_can_succeed(self):
        self.configure(); calls=[]
        def delay(*args):
            result=_match_candidates(*args); calls.append(1)
            if len(calls)<5: self.transport.clock.advance(.11)
            return result
        with patch('learning.fast_observation._match_candidates',side_effect=delay): report=self.run_attempt()
        self.assertEqual(report['observationAttempt']['status'],'success',report)
        self.assertEqual(self.capture_count,6)
        self.assertEqual(report['consumedObservations'],1)
        events,_=read_events(self.root)
        kinds=[e['kind'] for e in events]
        self.assertEqual(kinds.count('capture_requested'),5)
        self.assertEqual(kinds.count('measured'),5)

    def test_all_five_age_misses_are_failure(self):
        self.configure()
        def delay(*args):
            result=_match_candidates(*args); self.transport.clock.advance(.11); return result
        with patch('learning.fast_observation._match_candidates',side_effect=delay): report=self.run_attempt()
        self.assertEqual(report['status'],'failed')
        self.assertIn('capture_budget_exhausted',report['error'])
        self.assertEqual(self.capture_count,6)

    def test_acquisition_failure_in_denominator(self):
        self.configure(newer=False); report=self.run_attempt()
        self.assertEqual(report['observationAttempt']['status'],'acquisition_failed')
        self.assertEqual(report['observationAttempt']['currentCaptures'],0)
        self.assertGreater(report['observationAttempt']['requestToCoreEndMs'],7000)

    def test_http_failure_or_interruption_no_retry(self):
        for kind in ('http','interrupt'):
            self.setUp(); self.configure(); original=self.transport.fault
            def fail(device,path,r,body):
                body=original(device,path,r,body)
                if path=='/capture' and self.capture_count==2:
                    if kind=='interrupt': raise KeyboardInterrupt()
                    r['status']=503
                return body
            self.transport.fault=fail
            report=self.run_attempt()
            self.assertNotEqual(report['status'],'completed'); self.assertEqual(self.capture_count,2)

    def test_appearance_duplicate_regression_stop_first_current(self):
        for reason in ('appearance_mismatch_or_target_lost','frame_repeated_or_regressed','invalid_local_timing'):
            self.setUp(); self.configure()
            from learning.fast_observation import AnchoredRoiMeasurement
            original=AnchoredRoiMeasurement.measure
            def reject(tracker,*args,**kw):
                result=original(tracker,*args,**kw); result.update(accepted=False,reason=reason); return result
            with patch.object(AnchoredRoiMeasurement,'measure',reject): report=self.run_attempt()
            self.assertEqual(report['status'],'failed'); self.assertEqual(self.capture_count,2)
            self.assertIn(reason,report['error'])

    def test_capture_returns_after_budget_never_consumed(self):
        self.configure(); original=self.transport.fault
        def slow(device,path,r,body):
            body=original(device,path,r,body)
            if path=='/capture' and self.capture_count==2: self.transport.clock.advance(3.1)
            return body
        self.transport.fault=slow
        report=self.run_attempt()
        self.assertEqual(report['consumedObservations'],0); self.assertEqual(self.capture_count,2)
        self.assertTrue(report['observationAttempt']['operationBudgetExceeded'])
        self.assertTrue((self.root/'images/0/source.jpg').exists())

    def test_storage_failure_or_late_joins_and_stops(self):
        original=ArtifactStore.save_image
        for kind in ('late','error'):
            self.setUp(); self.configure()
            def fail(store,index,phase,body,digest):
                if index=='0':
                    if kind=='error': raise OSError('fixture storage error')
                    self.transport.clock.advance(.251)
                return original(store,index,phase,body,digest)
            with patch.object(ArtifactStore,'save_image',fail): report=self.run_attempt()
            self.assertEqual(report['status'],'failed'); self.assertEqual(self.capture_count,2)
            self.assertNotEqual(report['consumptions'][0]['storageStatus'],'completed')

    def test_status_io_budget_exhaustion_prevents_capture(self):
        self.configure(); original=self.transport.fault
        def slow(device,path,r,body):
            body=original(device,path,r,body)
            if device=='controller' and path=='/status' and self.capture_count==1:
                self.transport.clock.advance(3.1)
            return body
        self.transport.fault=slow; report=self.run_attempt()
        self.assertEqual(self.capture_count,1)
        self.assertEqual(report['observationAttempt']['currentCaptures'],0)

    def test_bad_configuration_no_get(self):
        with self.assertRaises(ValueError): self.run_probe(request_id='r')
        self.assertEqual(self.transport.calls,[])


class ConsumptionDeadlineTest(unittest.TestCase):
    def test_gate_uses_actual_consumption_time(self):
        f=causal.CausalAnchorTest(); f.setUp(); m,s,r=f.measurement()
        row=f.consumer().consume(m,s,r,f.body,deadline_monotonic=114.98)
        self.assertFalse(row['consumed']); self.assertTrue(row['haltAfterPersistence'])
        self.assertEqual(row['reason'],'operation_budget_exceeded_at_consumption')


class BatchAuditTest(unittest.TestCase):
    setUp = AttemptProbeTest.setUp
    configure = AttemptProbeTest.configure
    run_probe = AttemptProbeTest.run_probe
    run_causal = AttemptProbeTest.run_causal
    run_attempt = AttemptProbeTest.run_attempt
    def test_audit_original_terminal_and_all_request_denominators(self):
        from tools.evaluate_observation_attempts import audit, summarize
        self.configure(); report=self.run_attempt()
        self.assertEqual(audit(self.root),report)
        stats=summarize([report],[1000])
        self.assertEqual(stats['successRateAllPlanned'],.05)
        self.assertFalse(stats['gatePassed'])
        (self.root/'images/0/source.jpg').write_bytes(b'corrupt')
        with self.assertRaises(ValueError): audit(self.root)

    def test_batch_stops_on_failed_safety_check_or_transport(self):
        from tools.evaluate_observation_attempts import stop_batch
        for error in ('DeviceReadError: controller_not_idle', 'TimeoutError: timeout',
                      'DeviceReadError: controller_not_idle_at_end'):
            self.assertTrue(stop_batch(dict(error=error,outputLogUnchanged=True,status='failed',
                integrityErrors=[],consumptions=[])))
        self.assertFalse(stop_batch(dict(error='AttemptStopped: capture_budget_exhausted',
            outputLogUnchanged=True,status='failed',integrityErrors=[],consumptions=[])))

    def test_audit_failure_saved_with_fixed_denominator_before_stopping(self):
        import contextlib
        import io
        from tools.evaluate_observation_attempts import main
        self.configure(); report=self.run_attempt()
        output=self.root.parent/'batch'
        with patch('tools.evaluate_observation_attempts.run_fast_probe',return_value=report) as run, \
             patch('tools.evaluate_observation_attempts.audit',side_effect=ValueError('fixture audit failure')), \
             contextlib.redirect_stdout(io.StringIO()):
            code=main(['--output-dir',str(output),'--controller-url','http://controller',
                       '--camera-url','http://camera'])
        self.assertEqual(code,2); self.assertEqual(run.call_count,1)
        batch=json.loads((output/'batch.json').read_text())
        self.assertEqual(batch['completedRequests'],1)
        self.assertEqual(batch['unexecutedRequests'],19)
        self.assertEqual(batch['plannedRequests'],20)
        self.assertEqual(batch['successes'],0)
        self.assertIn('fixture audit failure',batch['auditError'])
