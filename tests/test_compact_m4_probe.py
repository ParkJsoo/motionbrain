"""S3 fixed-width payload and collection regressions, entirely synthetic."""
import base64
import json
import struct
import unittest
import zlib

from learning.serial_m4_probe import collect,audit,parse_reply,unexpected_tail
from tests import test_mac_controller_probe as fixture
from tests.test_serial_m4_probe import FakeSerial,BOOT,fresh_line


def compact_line(nonce='0123abcd',boot=BOOT,raw=2778,polled=1005,reported=1008,received=1000,flags=31):
    payload=struct.pack('>4s16sHIIIB',bytes.fromhex(nonce),bytes.fromhex(boot),raw,polled,reported,received,flags)
    payload+=struct.pack('>I',zlib.crc32(payload))
    return b'M4S3 '+base64.b64encode(payload)+b'\n'


class CompactSerial(FakeSerial):
    def write(self,command):
        assert command[:2]==b'\x1e3' and command[-1:]==b'\n' and len(command)==11
        self.writes.append(command);self.clock.sleep(.005)
        received=1000+len(self.writes)*30
        self.pending.extend(self.mutate(compact_line(command[2:10].decode(),polled=received+5,reported=received+8,received=received)))
        return len(command)


class CompactM4Test(unittest.TestCase):
    def setUp(self):
        self.f=fixture.ProbeTest();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.serial=CompactSerial(self.f.clock)
    def run_probe(self,**kwargs):
        return collect(self.f.root,'http://192.0.2.1','fake-port','fake-id',open_port=lambda *_:self.serial,
                       http=self.f.read,clock=self.f.clock,compact_sample=True,**kwargs)
    def test_wire_fields_equal_s2_without_reconstructing_time_or_identity(self):
        actual=compact_line();self.assertEqual(len(actual),58)
        a=parse_reply(actual,'0123abcd',BOOT,compact_sample=True)
        b=parse_reply(fresh_line(),'0123abcd',BOOT,fresh_sample=True)
        self.assertEqual(a,b)
    def test_complete_six_samples_with_trace_and_audit(self):
        r=self.run_probe(transport_trace=True)
        self.assertEqual(r['status'],'recorded',r);self.assertEqual((r['serialCount'],r['httpCount']),(6,8))
        self.assertFalse(r['trainingEligible']);self.assertTrue(self.serial.closed)
        a=audit(self.f.root);self.assertTrue(a['ok'],a)
        self.assertEqual([x['rawBytes'] for x in a['transportTraceSummaries']],[58]*6)
        plan=json.loads((self.f.root/'plan.json').read_text())
        self.assertTrue(plan['policy']['freshSample']);self.assertTrue(plan['policy']['compactSample'])
    def test_crc_rejects_corruption_before_sample_can_be_used(self):
        raw=bytearray(base64.b64decode(compact_line()[5:-1]));raw[21]^=1
        with self.assertRaisesRegex(ValueError,'compact_crc_mismatch'):
            parse_reply(b'M4S3 '+base64.b64encode(raw)+b'\n','0123abcd',BOOT,compact_sample=True)
    def test_crc_rejects_wire_corruption_during_collection_and_stops(self):
        def corrupt(line):
            raw=bytearray(base64.b64decode(line[5:-1]));raw[22]^=1
            return b'M4S3 '+base64.b64encode(raw)+b'\n'
        self.serial.mutate=corrupt
        r=self.run_probe(transport_trace=True)
        self.assertEqual((r['status'],r['serialCount']),('rejected',1));self.assertEqual(len(self.serial.writes),1)
        self.assertTrue(audit(self.f.root)['ok'])
    def test_full_boot_nonce_sensor_flags_and_time_order_still_required(self):
        for kwargs in ({'nonce':'ffffffff'},{'boot':'b'*32},{'raw':4096},{'flags':15},
                       {'polled':1000},{'reported':1151},{'polled':1010}):
            with self.assertRaises(ValueError):parse_reply(compact_line(**kwargs),'0123abcd',BOOT,compact_sample=True)
        wrapped=parse_reply(compact_line(received=0xfffffff8,polled=2,reported=4),'0123abcd',BOOT,compact_sample=True)
        self.assertEqual((wrapped['firmwareWaitMs'],wrapped['reportedAgeMs']),(12,2))
    def test_exact_mode_length_base64_and_duplicates(self):
        for raw in (fresh_line(),compact_line()[:-1],compact_line()+b'M4S3\n',compact_line()+compact_line(),
                    b'M4S3 '+b'A'*51+b'\n',b'M4S3 '+b'A'*51+b'=\n',b'M4S3 '+b'!'*52+b'\n'):
            with self.assertRaises(ValueError):parse_reply(raw,'0123abcd',BOOT,compact_sample=True)
        with self.assertRaises(ValueError):parse_reply(compact_line(),'0123abcd',BOOT,fresh_sample=True)
        self.assertTrue(unexpected_tail(b'M4S3\n'))
    def test_duplicate_final_compact_response_is_preserved_and_rejected(self):
        self.serial.mutate=lambda b:b+b if len(self.serial.writes)==6 else b
        r=self.run_probe(transport_trace=True)
        self.assertEqual(r['status'],'rejected');self.assertEqual(r['failure'],'unexpected_serial_tail_reply')
        self.assertTrue(audit(self.f.root)['ok'])
    def test_compact_partial_cancellation_keeps_identity_and_bytes(self):
        original=self.serial.read
        def read(n):
            if self.serial.writes and len(self.serial.pending)==49:raise KeyboardInterrupt()
            return original(n)
        self.serial.read=read
        r=self.run_probe(transport_trace=True)
        self.assertEqual((r['failure'],r['serialCount']),('cancelled',1))
        row=json.loads((self.f.root/'serial/01.json').read_text())
        self.assertEqual(len(base64.b64decode(row['rawBase64'])),9)
        self.assertTrue(audit(self.f.root)['ok'])

if __name__=='__main__':unittest.main()
