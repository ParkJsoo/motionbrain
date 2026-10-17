"""Synthetic serial and HTTP transports; never opens hardware."""
import base64
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch,Mock

from learning.serial_m4_probe import collect,audit,parse_reply,validate_receipt,read_sample,open_usb,unexpected_tail
from tests import test_mac_controller_probe as fixture

BOOT='a'*32

def line(nonce='0123abcd',boot=BOOT,raw=2778,polled=1000,reported=1008,flags=31):
    return f'M4S1 {nonce} {boot} {raw:04x} {polled:08x} {reported:08x} {flags:02x}\n'.encode()

class FakeSerial:
    def __init__(self,clock):
        self.clock=clock;self.pending=bytearray();self.writes=[];self.closed=False;self.mutate=lambda b:b
    def write(self,command):
        assert command[:1]==b'\x1e' and command[-1:]==b'\n' and len(command)==10
        self.writes.append(command)
        polled=1000+len(self.writes)*30
        self.pending.extend(self.mutate(line(command[1:9].decode(),polled=polled,reported=polled+8)))
        return len(command)
    def read(self,n):
        self.clock.sleep(.0001)
        if not self.pending:return b''
        b=bytes(self.pending[:1]);del self.pending[:1];return b
    def close(self):self.closed=True

def fresh_line(nonce='0123abcd',boot=BOOT,polled=1005,reported=1008,received=1000):
    return f'M4S2 {nonce} {boot} 0ada {polled:08x} {reported:08x} {received:08x} 1f\n'.encode()

class FreshSerial(FakeSerial):
    def write(self,command):
        assert command[:2]==b'\x1e2' and command[-1:]==b'\n' and len(command)==11
        self.writes.append(command)
        received=1000+len(self.writes)*30
        self.pending.extend(self.mutate(fresh_line(command[2:10].decode(),polled=received+5,reported=received+8,received=received)))
        return len(command)

class SerialM4Test(unittest.TestCase):
    def setUp(self):
        self.f=fixture.ProbeTest();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.serial=FakeSerial(self.f.clock)
    def run_probe(self,**kwargs):
        return collect(self.f.root,'http://192.0.2.1','test-port','test-serial',
                       open_port=kwargs.pop("open_port",lambda *_:self.serial),http=self.f.read,clock=self.f.clock,**kwargs)
    def test_available_bytes_read_in_batch_without_losing_duplicates(self):
        class BatchSerial(FreshSerial):
            @property
            def in_waiting(self):return len(self.pending)
            def read(self,n):
                self.read_sizes.append(n)
                self.clock.sleep(.0001*max(1,min(n,len(self.pending))))
                b=bytes(self.pending[:n]);del self.pending[:n];return b
        self.serial=BatchSerial(self.f.clock);self.serial.read_sizes=[]
        r=self.run_probe(fresh_sample=True)
        self.assertEqual(r['status'],'recorded',r);self.assertTrue(audit(self.f.root)['ok'])
        self.assertEqual(self.serial.read_sizes.count(82),6)
    def test_batch_duplicate_or_partial_tail_cannot_disappear(self):
        for raw in (fresh_line()+fresh_line(),fresh_line()+b'M4S',fresh_line()+b'partial log',
                    fresh_line()+b'M4S2\n',fresh_line()+b'M4S\n',fresh_line()+b'M4S1x\n'):
            with self.assertRaises(ValueError):parse_reply(raw,'0123abcd',BOOT,fresh_sample=True)
    def test_batch_malformed_extra_magic_rejects_collector(self):
        class BatchSerial(FreshSerial):
            @property
            def in_waiting(self):return len(self.pending)
            def read(self,n):
                self.clock.sleep(.0001*max(1,min(n,len(self.pending))))
                b=bytes(self.pending[:n]);del self.pending[:n];return b
        self.serial=BatchSerial(self.f.clock)
        self.serial.mutate=lambda b:b+b'M4S2\n' if len(self.serial.writes)==6 else b
        r=self.run_probe(fresh_sample=True)
        self.assertEqual(r['status'],'rejected');self.assertEqual(r['serialCount'],6)
        row=json.loads((self.f.root/'serial/06.json').read_text())
        self.assertTrue(base64.b64decode(row['rawBase64']).endswith(b'M4S2\n'))
    def test_audit_recomputes_malformed_extra_magic_with_valid_hashes(self):
        from learning.artifacts import describe_artifact
        self.serial=FreshSerial(self.f.clock);self.run_probe(fresh_sample=True)
        path=self.f.root/'serial/06.json';row=json.loads(path.read_text())
        raw=base64.b64decode(row['rawBase64'])+b'M4S2\n'
        row.update(rawBase64=base64.b64encode(raw).decode(),rawSha256=hashlib.sha256(raw).hexdigest())
        path.write_text(json.dumps(row))
        result_path=self.f.root/'result.json';result=json.loads(result_path.read_text())
        result['artifacts']=[describe_artifact(self.f.root,x['artifactRef']) for x in result['artifacts']]
        result_path.write_text(json.dumps(result))
        event_path=self.f.root/'events/00000001.json';event=json.loads(event_path.read_text())
        event['payload']['summary']=describe_artifact(self.f.root,'result.json');event_path.write_text(json.dumps(event))
        result=audit(self.f.root)
        self.assertFalse(result['ok']);self.assertIn('serial_reply_count_invalid',result['issues'])
    def test_transport_buffer_count_failure_keeps_failure_receipt(self):
        class BrokenSerial(FreshSerial):
            @property
            def in_waiting(self):raise OSError('disconnected')
        self.serial=BrokenSerial(self.f.clock)
        r=self.run_probe(fresh_sample=True)
        self.assertEqual(r['status'],'rejected');self.assertEqual(r['serialCount'],1)
        self.assertTrue(self.serial.closed)
    def reseal(self,relative,mutate):
        from learning.artifacts import describe_artifact
        path=self.f.root/relative;value=json.loads(path.read_text());mutate(value);path.write_text(json.dumps(value))
        path=self.f.root/'result.json';result=json.loads(path.read_text())
        result['artifacts']=[describe_artifact(self.f.root,x['artifactRef']) for x in result['artifacts']]
        path.write_text(json.dumps(result))
        path=self.f.root/'events/00000001.json';event=json.loads(path.read_text())
        event['payload']['summary']=describe_artifact(self.f.root,'result.json');path.write_text(json.dumps(event))
    def test_opt_in_trace_records_boundaries_and_recomputes_audit(self):
        r=self.run_probe(transport_trace=True)
        self.assertEqual(r['status'],'recorded',r)
        a=audit(self.f.root);self.assertTrue(a['ok'],a)
        self.assertEqual(len(a['transportTraceSummaries']),6)
        for summary in a['transportTraceSummaries']:
            self.assertTrue(summary['hostObservationOnly']);self.assertFalse(summary['wireTimeVerified'])
            self.assertEqual((summary['readCalls'],summary['rawBytes']),(73,73))
        row=json.loads((self.f.root/'serial/01.json').read_text())
        self.assertEqual(row['transportTrace']['phases'][0]['kind'],'write')
        self.assertEqual(row['transportTrace']['phases'][-1]['kind'],'parse')
    def test_trace_cancellation_preserves_partial_chunks_and_failed_audit(self):
        original=self.serial.read
        def read(n):
            if self.serial.writes and len(self.serial.pending)==64:raise KeyboardInterrupt()
            return original(n)
        self.serial.read=read
        r=self.run_probe(transport_trace=True)
        self.assertEqual((r['failure'],r['serialCount']),('cancelled',1))
        a=audit(self.f.root);self.assertTrue(a['ok'],a)
        self.assertEqual(a['transportTraceSummaries'][0]['rawBytes'],9)
        self.reseal('serial/01.json',lambda v:v['transportTrace']['phases'][-1].update(rawOffset=0))
        self.assertFalse(audit(self.f.root)['ok'])
    def test_trace_end_clock_cancellation_preserves_observed_bytes(self):
        original=self.f.clock.monotonic;interrupted=[]
        def clock():
            if self.serial.writes and len(self.serial.pending)==64 and not interrupted:
                interrupted.append(True);raise KeyboardInterrupt()
            return original()
        self.f.clock.monotonic=clock
        r=self.run_probe(transport_trace=True)
        self.assertEqual((r['failure'],r['serialCount']),('cancelled',1))
        row=json.loads((self.f.root/'serial/01.json').read_text())
        self.assertEqual(len(base64.b64decode(row['rawBase64'])),9)
        self.assertEqual(row['transportTrace']['phases'][-1]['errorType'],'KeyboardInterrupt')
        self.assertTrue(self.serial.closed)
        a=audit(self.f.root);self.assertTrue(a['ok'],a)
    def test_trace_read_cap_is_a_sealed_failure_without_retry(self):
        self.serial.mutate=lambda b:b''
        r=self.run_probe(transport_trace=True)
        self.assertEqual(r['failure'],'serial_receipt_invalid');self.assertEqual(r['serialCount'],1)
        row=json.loads((self.f.root/'serial/01.json').read_text())
        self.assertEqual(row['error'],'transport_trace_read_limit')
        a=audit(self.f.root);self.assertTrue(a['ok'],a)
        self.assertEqual(a['transportTraceSummaries'][0]['readCalls'],256)
    def test_trace_mode_cannot_silently_lose_receipt_trace(self):
        self.run_probe(transport_trace=True)
        self.reseal('serial/01.json',lambda v:v.pop('transportTrace'))
        self.assertFalse(audit(self.f.root)['ok'])
    def test_failed_attempt_request_cannot_be_replaced_by_legacy_command(self):
        self.serial.mutate=lambda b:b'bad\n';self.run_probe(transport_trace=True)
        self.reseal('serial/01.json',lambda v:v.update(requestBase64=base64.b64encode(b'arm\n').decode()))
        self.assertFalse(audit(self.f.root)['ok'])
    def test_fresh_samples_audit_and_conservative_wait_subtraction(self):
        self.serial=FreshSerial(self.f.clock)
        r=self.run_probe(fresh_sample=True)
        self.assertEqual(r['status'],'recorded',r);self.assertEqual(r['serialCount'],6)
        self.assertTrue(audit(self.f.root)['ok'])
        for sample in r['samples']:
            self.assertEqual(sample['firmwareWaitMs'],8)
            self.assertEqual(sample['firmwareWaitLowerDiagnosticMs'],6)
            self.assertAlmostEqual(sample['ageUpperDiagnosticMs'],sample['transportMs']-6+3+2)
            self.assertFalse(sample['mcuClockRateCalibrated'])
            self.assertFalse(sample['trainingEligible'])
    def test_fresh_rejects_cached_protocol_and_stale_or_impossible_markers(self):
        for raw in (line(),fresh_line(polled=1000),fresh_line(polled=999),fresh_line(polled=1009),fresh_line(reported=1151)):
            with self.assertRaises(ValueError):parse_reply(raw,'0123abcd',BOOT,fresh_sample=True)
        sample=parse_reply(fresh_line(polled=2,reported=4,received=0xfffffff8),'0123abcd',BOOT,fresh_sample=True)
        self.assertEqual((sample['firmwareWaitMs'],sample['reportedAgeMs']),(12,2))
    def test_fresh_cannot_subtract_more_wait_than_host_elapsed(self):
        self.serial=FreshSerial(self.f.clock)
        self.serial.mutate=lambda b:fresh_line(b.split()[1].decode(),received=1000,polled=1090,reported=1100)
        r=self.run_probe(fresh_sample=True)
        self.assertEqual(r['failure'],'firmware_wait_exceeds_host_rtt');self.assertEqual(r['serialCount'],1)
    def test_fresh_timeout_and_final_duplicate_preserve_raw(self):
        self.serial=FreshSerial(self.f.clock)
        self.serial.mutate=lambda b:b+b if len(self.serial.writes)==6 else b
        r=self.run_probe(fresh_sample=True)
        self.assertEqual(r['failure'],'unexpected_serial_tail_reply')
        self.assertTrue(audit(self.f.root)['ok'])
        self.assertTrue(unexpected_tail(b'M4S2\n'))
    def test_fresh_wrong_wire_version_fails_first_request(self):
        self.serial=FreshSerial(self.f.clock);self.serial.mutate=lambda b:b.replace(b'M4S2',b'M4S1')
        r=self.run_probe(fresh_sample=True)
        self.assertEqual(r['status'],'rejected');self.assertEqual(r['serialCount'],1)
        self.assertEqual(len(self.serial.writes),1)
    def test_finite_six_samples_eight_http_and_audit(self):
        r=self.run_probe();self.assertEqual(r['status'],'recorded',r)
        self.assertEqual((r['httpCount'],r['serialCount'],r['within20ms']),(8,6,6))
        self.assertEqual(len(self.serial.writes),6);self.assertTrue(self.serial.closed)
        self.assertTrue(audit(self.f.root)['ok'])
        self.assertTrue(all(not x['sensorPhysicalSampleTimeVerified'] for x in r['samples']))
    def test_explicit_initialization_creates_new_baseline_after_single_open(self):
        new_boot='b'*32;opens=[]
        def opened(*args):
            opens.append(args)
            self.f.values['actions']['bootId']=new_boot
            self.f.values['action_outputs'].update(bootId=new_boot,dropped=0,entries=[dict(sequence=i,motor=i,pwm=0,actionOrdinal=0) for i in range(1,6)])
            self.serial.pending.extend(b'=== MotionBrain System Boot ===\n')
            self.serial.mutate=lambda b:b.replace(BOOT.encode(),new_boot.encode())
            return self.serial
        r=self.run_probe(initialize_port=True,open_port=opened)
        self.assertEqual(r['status'],'recorded',r);self.assertEqual(r['httpCount'],12)
        self.assertTrue(r['controllerBootChangedDuringInitialization']);self.assertEqual(len(opens),1)
        self.assertTrue(audit(self.f.root)['ok'])
        self.assertFalse(r['trainingEligible'])
    def test_initialization_without_reboot_must_preserve_outputs(self):
        def opened(*args):
            self.f.values['action_outputs']['entries']=[dict(motor=1,pwm=40),dict(motor=1,pwm=0)]
            return self.serial
        r=self.run_probe(initialize_port=True,open_port=opened)
        self.assertEqual(r['failure'],'outputs_changed_without_initialization_reboot')
        self.assertEqual(self.serial.writes,[])
    def test_new_boot_must_have_complete_all_off_output_history(self):
        from learning.serial_m4_probe import initialization_boot_changed
        before={'bootId':'a'}
        good=dict(bootId='b',dropped=0,entries=[dict(sequence=i,motor=i,pwm=0,actionOrdinal=0) for i in range(1,6)])
        self.assertTrue(initialization_boot_changed(before,good))
        for change in ({'dropped':1},{'entries':[]},{'entries':[dict(sequence=1,motor=1,pwm=40,actionOrdinal=0)]}):
            with self.assertRaises(ValueError):initialization_boot_changed(before,{**good,**change})
    def test_initialization_rejects_multiple_boot_logs_before_sensor_requests(self):
        self.serial.pending.extend(b'=== MotionBrain System Boot ===\n'*2)
        r=self.run_probe(initialize_port=True)
        self.assertEqual(r['failure'],'multiple_boots_during_initialization')
        self.assertEqual(self.serial.writes,[]);self.assertTrue(self.serial.closed)
    def test_initialization_still_rejects_boot_change_during_samples(self):
        self.serial.mutate=lambda b:b.replace(BOOT.encode(),b'b'*32)
        r=self.run_probe(initialize_port=True)
        self.assertEqual(r['status'],'rejected');self.assertEqual(r['serialCount'],1)
        self.assertFalse(r['controllerBootChangedDuringInitialization'])
    def test_wrong_boot_aborts_first_request_and_closes(self):
        self.serial.mutate=lambda b:b.replace(BOOT.encode(),b'b'*32)
        r=self.run_probe();self.assertEqual(r['status'],'rejected');self.assertEqual(r['serialCount'],1)
        self.assertEqual(r['httpCount'],4);self.assertTrue(self.serial.closed)
        self.assertTrue((self.f.root/'serial/01.json').exists())
    def test_duplicate_sample_is_rejected(self):
        self.serial.mutate=lambda b:line(b.split()[1].decode())
        r=self.run_probe();self.assertEqual(r['failure'],'serial_sample_not_new');self.assertEqual(r['serialCount'],2)
    def test_motion_flag_and_nonce_rejected(self):
        for transform in (lambda b:b[:-3]+b'0f\n',lambda b:b.replace(b.split()[1],b'ffffffff')):
            self.serial=FakeSerial(self.f.clock);self.serial.mutate=transform
            row=read_sample(self.serial,'0123abcd',BOOT,'clock',clock=self.f.clock)
            self.assertFalse(row['transportCompleted'])
            with self.assertRaises(ValueError):validate_receipt(row,'0123abcd',BOOT)
    def test_truncated_response_timeout_preserves_raw(self):
        self.serial.mutate=lambda b:b[:30]
        r=self.run_probe();self.assertEqual(r['status'],'rejected');self.assertEqual(len(self.serial.writes),1)
        row=json.loads((self.f.root/'serial/01.json').read_text())
        self.assertEqual(len(base64.b64decode(row['rawBase64'])),30)
    def test_raw_limit_and_corruption(self):
        for b in (b'x'*4097,line(raw=4096),line(flags=0),line(polled=0),line(reported=2000),line()+line()):
            with self.assertRaises(ValueError):parse_reply(b,'0123abcd',BOOT)
    def test_wraparound_age_is_unsigned_and_conservative(self):
        sample=parse_reply(line(polled=0xfffffff8,reported=2),'0123abcd',BOOT)
        self.assertEqual(sample['reportedAgeMs'],10)
    def test_changed_raw_file_fails_audit(self):
        self.run_probe();p=self.f.root/'serial/02.json';p.write_text('{}')
        self.assertFalse(audit(self.f.root)['ok'])
    def test_duplicate_final_reply_is_preserved_and_rejected(self):
        self.serial.mutate=lambda b:b+b if len(self.serial.writes)==6 else b
        r=self.run_probe();self.assertEqual(r['status'],'rejected')
        self.assertEqual(r['failure'],'unexpected_serial_tail_reply')
        tail=json.loads((self.f.root/'serial/tail.json').read_text())
        self.assertEqual(len(base64.b64decode(tail['rawBase64'])),73)
    def test_partial_final_reply_never_passes(self):
        self.serial.mutate=lambda b:b+b'M4S' if len(self.serial.writes)==6 else b
        r=self.run_probe();self.assertEqual(r['failure'],'unexpected_serial_tail_reply')
        for raw in (b'M',b'M4',b'M4S',b'M4S\n',b'M4S1',b'partial log'):
            self.assertTrue(unexpected_tail(raw))
        self.assertFalse(unexpected_tail(b'[INFO] complete log\n'))
    def test_nonobject_artifacts_return_failed_audit(self):
        from learning.artifacts import describe_artifact
        self.run_probe()
        (self.f.root/'plan.json').write_text('null')
        result_path=self.f.root/'result.json';result=json.loads(result_path.read_text())
        result['artifacts']=[describe_artifact(self.f.root,x['artifactRef']) for x in result['artifacts']]
        result_path.write_text(json.dumps(result))
        event_path=self.f.root/'events/00000001.json';event=json.loads(event_path.read_text())
        event['payload']['summary']=describe_artifact(self.f.root,'result.json');event_path.write_text(json.dumps(event))
        self.assertFalse(audit(self.f.root)['ok'])
        for row in (None,[],True):
            with self.assertRaises(ValueError):validate_receipt(row,'0123abcd',BOOT)
    def test_cancellation_closes_port_and_does_not_retry(self):
        original=self.serial.read
        def interrupt(n):
            if self.serial.writes and len(self.serial.pending)==64:raise KeyboardInterrupt()
            return original(n)
        self.serial.read=interrupt
        r=self.run_probe()
        self.assertTrue(self.serial.closed);self.assertEqual(len(self.serial.writes),1)
        self.assertEqual((r['status'],r['failure'],r['serialCount']),('rejected','cancelled',1))
        self.assertTrue(r['cancelled']);self.assertFalse(r['trainingEligible'])
        row=json.loads((self.f.root/'serial/01.json').read_text())
        self.assertTrue(row['cancelled']);self.assertFalse(row['transportCompleted'])
        raw=base64.b64decode(row['rawBase64']);self.assertEqual(len(raw),9)
        self.assertEqual(hashlib.sha256(raw).hexdigest(),row['rawSha256'])
        self.assertTrue((self.f.root/'result.json').exists());self.assertTrue(audit(self.f.root)['ok'])
    def test_cancel_before_first_serial_byte_preserves_attempt(self):
        self.serial.read=lambda n:(_ for _ in ()).throw(KeyboardInterrupt()) if self.serial.writes else b''
        # Startup needs the synthetic clock to advance independently.
        original=self.serial.read
        def read(n):self.f.clock.sleep(.005);return original(n)
        self.serial.read=read
        r=self.run_probe();self.assertEqual(r['failure'],'cancelled');self.assertEqual(r['serialCount'],1)
        row=json.loads((self.f.root/'serial/01.json').read_text());self.assertEqual(row['rawBase64'],'')
        self.assertTrue(self.serial.closed)
    def test_cli_reports_sealed_cancellation_with_exit_130(self):
        from tools import probe_serial_m4 as cli
        import io,signal
        previous=signal.getsignal(signal.SIGTERM)
        args=['probe','--output','unused','--controller-url','http://192.0.2.1','--port','fake','--usb-serial','fake']
        with patch('sys.argv',args),patch.object(cli,'collect',return_value=dict(status='rejected',cancelled=True)),patch('sys.stdout',new_callable=io.StringIO):
            self.assertEqual(cli.main(),130)
        self.assertIs(signal.getsignal(signal.SIGTERM),previous)
    def test_reported_time_must_advance_with_same_boot(self):
        def mutate(b):
            n=len(self.serial.writes);polled=1000+n*30
            return line(b.split()[1].decode(),polled=polled,reported=polled+(100 if n==1 else 8))
        self.serial.mutate=mutate
        r=self.run_probe();self.assertEqual(r['failure'],'serial_sample_not_new')
        self.assertEqual(r['serialCount'],2)
    def test_huge_or_nonfinite_clock_rejected_without_crashing(self):
        row=read_sample(self.serial,'0123abcd',BOOT,'clock',clock=self.f.clock)
        for v in (10**400,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):validate_receipt({**row,'receivedMonotonic':v},'0123abcd',BOOT)
    def test_existing_path_does_not_reopen_port(self):
        self.run_probe()
        with self.assertRaises(Exception):self.run_probe()
        self.assertEqual(len(self.serial.writes),6)
    def test_read_failure_keeps_partial_and_closes(self):
        original=self.serial.read
        def fail(n):
            if self.serial.writes and len(self.serial.pending)<60:raise OSError('disconnected')
            return original(n)
        self.serial.read=fail
        r=self.run_probe();self.assertEqual(r['status'],'rejected');self.assertTrue(self.serial.closed)
        row=json.loads((self.f.root/'serial/01.json').read_text());self.assertGreater(len(base64.b64decode(row['rawBase64'])),0)
    def test_unsafe_nonce_never_written(self):
        with self.assertRaises(ValueError):read_sample(self.serial,'arm\n',BOOT,'clock',clock=self.f.clock)
        self.assertEqual(self.serial.writes,[])
    def test_false_cached_sample_rejected(self):
        row=read_sample(self.serial,'0123abcd',BOOT,'clock',clock=self.f.clock);row['sample']['reportedAgeMs']=0
        with self.assertRaises(ValueError):validate_receipt(row,'0123abcd',BOOT)
    def test_output_mutation_rejects_whole_session(self):
        def mutate(e,v):
            if e=='action_outputs' and len(self.f.calls)>4:v['dropped']+=1
        self.f.mutate=mutate
        r=self.run_probe();self.assertEqual(r['status'],'rejected');self.assertEqual(r['failure'],'controller_boot_or_outputs_changed')

class USBIdentityTest(unittest.TestCase):
    def test_identity_checked_before_open_and_lines_set_before_open(self):
        import sys,types
        d=Mock();d.port=None
        fake=types.ModuleType('serial');fake.Serial=Mock(return_value=d)
        listing=types.ModuleType('serial.tools.list_ports');listing.comports=lambda:[types.SimpleNamespace(device='p',vid=0x10c4,pid=0xea60,serial_number='id')]
        package=types.ModuleType('serial.tools');package.list_ports=listing
        with patch.dict(sys.modules,{'serial':fake,'serial.tools':package,'serial.tools.list_ports':listing}):
            with self.assertRaises(ValueError):open_usb('p','wrong')
            fake.Serial.assert_not_called()
            def opened():self.assertIs(d.dtr,False);self.assertIs(d.rts,False);self.assertEqual(d.port,'p')
            d.open.side_effect=opened
            self.assertIs(open_usb('p','id'),d)
            self.assertTrue(fake.Serial.call_args.kwargs['exclusive'])

if __name__=='__main__':unittest.main()
