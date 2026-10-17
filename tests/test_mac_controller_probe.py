"""Finite observer tests with synthetic camera bytes and fake HTTP/processes."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
from unittest.mock import Mock, patch

from learning.artifacts import ArtifactStore, StorageError, describe_artifact
from learning.mac_controller_probe import audit_probe, read_controller, run_probe, stop_camera
from tests import test_mac_camera_probe as camera_fixture
from tests.test_action_execution import FakeClock
from tests.test_action_port import status


class ProbeTest(unittest.TestCase):
    def setUp(self):
        fixture = camera_fixture.CameraProbeTest()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.fixture = fixture
        self.root = fixture.root.parent / 'combined'
        self.clock = FakeClock()
        self.calls = []
        self.state = status()
        self.state.update(state='IDLE', motorEnabled=False)
        self.state['shoulderAngle'].update(correctionActive=False, ageMs=1)
        self.values = dict(status=self.state, sequence={'sequence':{'state':'IDLE','totalCount':0}},
                           actions={'bootId':'a'*32, 'active':False, 'enabled':False},
                           action_outputs={'bootId':'a'*32,'entries':[], 'dropped':100})
        self.mutate = lambda endpoint, value: None
        self.process = Mock(returncode=0, pid=999999)
        self.process.poll.return_value = None
        self.process.wait.side_effect = lambda **kw:self.clock.sleep(.1)
        self.stop = Mock()
        self.launch = Mock(side_effect=self.camera)

    def camera(self, helper, root, device, view):
        shutil.copytree(self.fixture.root, root/'camera')
        return self.process

    def read(self, url, endpoint, *, clock):
        self.calls.append(endpoint)
        value = copy.deepcopy(self.values[endpoint])
        self.mutate(endpoint, value)
        start = clock.monotonic()
        clock.sleep(.005)
        body = json.dumps(value)
        return dict(endpoint=endpoint, method='GET', clockDomain='python_monotonic',
                    requestStartedMonotonic=start, receivedMonotonic=clock.monotonic(),
                    elapsedMs=(clock.monotonic()-start)*1000, curlExit=0, httpStatus='200',
                    response=value, responseBody=body, bodySha256=hashlib.sha256(body.encode()).hexdigest())

    def run_probe(self, **kw):
        args = dict(controller_url='http://192.0.2.1', helper=sys.executable,
                    device_id='synthetic-device', view_id='synthetic-new-desk',
                    read=self.read, launch=self.launch, stop=self.stop, clock=self.clock)
        args.update(kw)
        return run_probe(self.root, **args)

    def test_fixed_reads_and_no_eligibility_or_clock_promotion(self):
        result = self.run_probe()
        self.assertEqual(result['status'], 'recorded', result)
        self.assertEqual(len(self.calls), 14)
        self.assertEqual(self.calls, ['status','sequence','actions','action_outputs']+['status']*6+['status','sequence','actions','action_outputs'])
        self.assertEqual(result['m4Within20ms'], 6)
        self.assertTrue(audit_probe(self.root)['ok'])
        self.stop.assert_called_once_with(self.process)
        for key in ('trainingEligible','controlEligible','clockDomainsMerged','framesPairedWithSensors','automaticRetry'):
            self.assertIs(result[key], False)
        self.assertEqual(result['motorRequests'], 0)

    def test_old_dropped_output_count_is_observed_not_cleared(self):
        result = self.run_probe()
        self.assertEqual(result['status'], 'recorded')
        self.assertEqual(self.values['action_outputs']['dropped'], 100)

    def test_invalid_arguments_before_device_access(self):
        for kw in ({'view_id':'../bad'},{'device_id':''},{'helper':'/nonexistent'},
                   {'controller_url':'http://host/status'},{'controller_url':'http://secret@host'}):
            with self.subTest(kw=kw), self.assertRaises((ValueError, OSError)):
                self.run_probe(**kw)
        self.assertFalse(self.calls)
        self.launch.assert_not_called()

    def test_existing_run_is_never_resumed(self):
        self.run_probe()
        with self.assertRaises(StorageError):self.run_probe()
        self.assertEqual(len(self.calls),14)
        self.launch.assert_called_once()

    def test_active_baseline_blocks_camera(self):
        self.state['motors']['M1']['speed'] = 10
        result=self.run_probe()
        self.assertEqual(result['status'],'rejected')
        self.assertEqual(self.calls,['status'])
        self.launch.assert_not_called()
        self.assertTrue(audit_probe(self.root)['ok'])

    def test_null_sensor_rejected_without_crash(self):
        self.state['sensor']=None
        result=self.run_probe()
        self.assertEqual(result['status'],'rejected')
        self.launch.assert_not_called()

    def test_non_numeric_m4_rejected(self):
        self.state['shoulderAngle']['ageMs']='1'
        self.assertEqual(self.run_probe()['failure'],'m4_measurement_invalid')

    def test_unrepresentable_m4_number_rejected_and_sealed(self):
        self.state['shoulderAngle']['ageMs']=10**400
        self.assertEqual(self.run_probe()['failure'],'m4_measurement_invalid')
        self.assertTrue(audit_probe(self.root)['ok'])

    def test_disabled_bounded_action_is_not_required_or_enabled(self):
        self.values['actions']['enabled']=True
        self.assertEqual(self.run_probe()['status'],'recorded')
        self.assertEqual(len(self.calls),14)

    def test_boot_change_rejected(self):
        def mutate(e,v):
            if e=='actions' and len(self.calls)>10:v['bootId']='b'*32
        self.mutate=mutate
        self.assertEqual(self.run_probe()['failure'],'controller_boot_changed')
        self.stop.assert_called_once()

    def test_any_output_change_rejected(self):
        def mutate(e,v):
            if e=='action_outputs' and len(self.calls)>10:v['dropped']+=1
        self.mutate=mutate
        self.assertEqual(self.run_probe()['failure'],'output_history_changed')

    def test_transport_latency_not_substituted_for_sensor_age(self):
        original=self.read
        def slow(*a,**kw):
            row=original(*a,**kw)
            self.clock.sleep(.04)
            row.update(receivedMonotonic=self.clock.monotonic(),elapsedMs=(self.clock.monotonic()-row['requestStartedMonotonic'])*1000)
            return row
        result=self.run_probe(read=slow)
        self.assertEqual(result['status'],'recorded')
        self.assertEqual(result['m4Within20ms'],0)

    def test_timeout_no_retry_and_camera_cleanup(self):
        original=self.read
        def timeout(*a,**kw):
            if len(self.calls)==4:raise subprocess.TimeoutExpired('curl',1.2)
            return original(*a,**kw)
        result=self.run_probe(read=timeout)
        self.assertEqual(result['requestCount'],5)
        self.assertEqual(result['status'],'rejected')
        self.stop.assert_called_once()

    def test_deadline_at_final_request_cannot_leave_success_status(self):
        original=self.read
        def final_deadline(*a,**kw):
            if len(self.calls)==13:self.clock.now=150.001
            return original(*a,**kw)
        result=self.run_probe(read=final_deadline)
        self.assertEqual(result['status'],'rejected')
        self.assertEqual(result['failure'],'session_deadline')
        self.assertEqual(result['requestCount'],14)

    def test_bad_body_binding_stops_before_camera(self):
        original=self.read
        def bad(*a,**kw):
            row=original(*a,**kw);row['responseBody']='{}';return row
        self.assertEqual(self.run_probe(read=bad)['failure'],'controller_body_binding_invalid')
        self.launch.assert_not_called()

    def test_camera_early_exit_does_not_retry(self):
        self.process.poll.return_value=3
        result=self.run_probe()
        self.assertEqual(result['failure'],'camera_process_ended_before_status_samples')
        self.assertEqual(len(self.calls),4)
        self.stop.assert_called_once()

    def test_camera_processing_failure_has_no_final_reads(self):
        self.fixture.frames[1]['convertedHostPtsSeconds']=None
        self.fixture.save_frame(1)
        self.assertEqual(self.run_probe()['failure'],'camera_processing_not_passed')
        self.assertEqual(len(self.calls),10)
        self.stop.assert_called_once()

    def test_storage_failure_stops_camera_and_leaves_unsealed(self):
        original=ArtifactStore.write_json
        def fail(store, path, value):
            if path.startswith('controller/05'):raise StorageError('disk_failure')
            return original(store,path,value)
        with patch.object(ArtifactStore,'write_json',fail), self.assertRaises(StorageError):self.run_probe()
        self.assertEqual(len(self.calls),5)
        self.stop.assert_called_once()
        self.assertFalse(audit_probe(self.root)['ok'])

    def test_slow_storage_stops_before_device_io(self):
        original=ArtifactStore.write_json
        def slow(store, path, value):
            ref=original(store,path,value);self.clock.sleep(.251);return ref
        with patch.object(ArtifactStore,'write_json',slow),self.assertRaises(StorageError):self.run_probe()
        self.assertFalse(self.calls)

    def test_interrupt_cleans_camera_without_seal(self):
        original=self.read
        def interrupt(*a,**kw):
            if len(self.calls)==4:raise KeyboardInterrupt()
            return original(*a,**kw)
        with self.assertRaises(KeyboardInterrupt):self.run_probe(read=interrupt)
        self.stop.assert_called_once()
        self.assertFalse(audit_probe(self.root)['ok'])

    def rewrite_summary(self, mutate):
        p=self.root/'result.json';v=json.loads(p.read_text());mutate(v);p.write_text(json.dumps(v))
        event=next((self.root/'events').iterdir());v=json.loads(event.read_text())
        v['payload']['summary']=describe_artifact(self.root,'result.json');event.write_text(json.dumps(v))

    def test_audit_recomputes_samples_instead_of_trusting_summary(self):
        self.run_probe()
        self.rewrite_summary(lambda v:v['samples'][0].update(angleDegrees=0))
        self.assertFalse(audit_probe(self.root)['ok'])

    def test_audit_rejects_receipts_outside_whole_observation_budget(self):
        self.run_probe()
        for p in sorted((self.root/'controller').iterdir())[10:]:
            v=json.loads(p.read_text())
            v['requestStartedMonotonic']+=60;v['receivedMonotonic']+=60
            p.write_text(json.dumps(v))
        def rebind(v):
            v['artifacts']=[describe_artifact(self.root,r['artifactRef']) for r in v['artifacts']]
            v['observationEndedMonotonic']+=60
            v['observationElapsedSeconds']+=60
            v['elapsedSeconds']+=60
        self.rewrite_summary(rebind)
        self.assertIn('observation_deadline_or_timing_invalid',audit_probe(self.root)['issues'])

    def test_cleanup_time_separate_from_observation_deadline(self):
        self.stop.side_effect=lambda process:self.clock.sleep(4)
        result=self.run_probe()
        self.assertEqual(result['status'],'recorded')
        self.assertAlmostEqual(result['elapsedSeconds']-result['observationElapsedSeconds'],4)
        self.assertTrue(audit_probe(self.root)['ok'])

    def test_audit_checks_camera_and_controller_identity(self):
        self.run_probe()
        p=self.root/'plan.json';v=json.loads(p.read_text());v['viewId']='different-view';p.write_text(json.dumps(v))
        def rebind(v):
            v['artifacts']=[describe_artifact(self.root,r['artifactRef']) for r in v['artifacts']]
        self.rewrite_summary(rebind)
        self.assertIn('camera_identity_mismatch',audit_probe(self.root)['issues'])

    def test_tampered_original_rejected(self):
        self.run_probe()
        (self.root/'controller/05-status.json').write_text('{}')
        self.assertFalse(audit_probe(self.root)['ok'])

    def test_missing_camera_evidence_rejected(self):
        self.run_probe()
        (self.root/'camera/frame-0000.bgra').unlink()
        self.assertFalse(audit_probe(self.root)['ok'])

    def test_malformed_saved_result_fails_closed(self):
        self.run_probe()
        self.rewrite_summary(lambda v:v.update(artifacts=None))
        self.assertFalse(audit_probe(self.root)['ok'])


class TransportTest(unittest.TestCase):
    def test_exact_read_only_command_and_bounded_process(self):
        response=Mock(stdout=b'{}\n200',returncode=0)
        with patch('learning.mac_controller_probe.subprocess.run',return_value=response) as run:
            result=read_controller('http://192.0.2.1','status')
        args=run.call_args.args[0]
        self.assertEqual(args[-1],'http://192.0.2.1/status')
        for forbidden in ('--location','-L','--retry','--data','--request'):
            self.assertNotIn(forbidden,args)
        self.assertEqual(run.call_args.kwargs['timeout'],1.2)
        self.assertEqual(result['method'],'GET')

    def test_nonwhitelisted_endpoint_rejected_before_subprocess(self):
        with patch('learning.mac_controller_probe.subprocess.run') as run:
            for endpoint in ('arm','command','capture','actions/cancel','status?arm=1'):
                with self.assertRaises(ValueError):read_controller('http://192.0.2.1',endpoint)
            run.assert_not_called()

    def test_cleanup_terminates_process_group_even_if_wrapper_exited(self):
        process=Mock(pid=999999,returncode=0)
        with patch('learning.mac_controller_probe.os.killpg') as kill:
            stop_camera(process)
        self.assertEqual(kill.call_count,2)
        self.assertEqual(process.wait.call_count,2)


if __name__=='__main__':unittest.main()
