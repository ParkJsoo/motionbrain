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

class SerialM4Test(unittest.TestCase):
    def setUp(self):
        self.f=fixture.ProbeTest();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.serial=FakeSerial(self.f.clock)
    def run_probe(self,**kwargs):
        return collect(self.f.root,'http://192.0.2.1','test-port','test-serial',
                       open_port=kwargs.pop("open_port",lambda *_:self.serial),http=self.f.read,clock=self.f.clock,**kwargs)
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
            if self.serial.writes:raise KeyboardInterrupt()
            return original(n)
        self.serial.read=interrupt
        with self.assertRaises(KeyboardInterrupt):self.run_probe()
        self.assertTrue(self.serial.closed);self.assertEqual(len(self.serial.writes),1)
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
