"""Synthetic firmware observations and finite collector integration."""
import copy
import hashlib
import json
import unittest

from learning.m4_stage_trace import SCHEMA,check_diagnostics,validate_snapshot,link_stages
from learning.serial_m4_probe import collect,audit,parse_reply
from tests.test_compact_m4_probe import CompactSerial,compact_line,BOOT
from tests import test_mac_controller_probe as fixture


def stage_record(sample,sequence):
    request=sample['requestReceivedMs']*1000+100;poll=sample['polledMs']*1000+100
    reply=sample['reportedMs']*1000+10
    return dict(sequence=sequence,nonce=sample['nonce'],protocol=3,status='replied',
                requestReceivedMs=sample['requestReceivedMs'],requestParsedUs=request,
                pollAttempts=1,pollSucceeded=True,angleSucceeded=True,adcSampled=True,
                requestLoopStartedUs=request-100,requestLoopGapUs=1000,maxPendingLoopGapUs=poll-request,
                pollMarkerMs=sample['polledMs'],sensorUpdateEnteredUs=poll,afterAdcUs=poll+100,
                pollStartedUs=poll+110,angleReadStartedUs=poll+200,angleReadCompletedUs=poll+400,
                pollCompletedUs=poll+600,replyReportedMs=sample['reportedMs'],replyPollMarkerMs=sample['polledMs'],
                replyRaw=sample['raw'],replyEnqueueUs=reply,replyLoopStartedUs=poll-100,
                replyBytes=58,replyWriteReturnedUs=reply+10,writtenBytes=58)


def snapshot(records=()):
    return dict(schemaVersion=SCHEMA,bootId=BOOT,clockDomain='esp_timer_since_boot_us',
                clockRateCalibrated=False,physicalSampleTimeVerified=False,txCompleteVerified=False,
                capacity=8,dropped=max(0,len(records)-8),nextSequence=len(records)+1,active=False,records=list(records[-8:]))


def diagnostic_receipt(value,clock):
    start=clock.monotonic();clock.sleep(.005);body=json.dumps(value)
    return dict(endpoint='m4_diagnostics',method='GET',transport='python_http_client',transportCompleted=True,
                httpStatus='200',clockDomain='python_monotonic',requestStartedMonotonic=start,
                receivedMonotonic=clock.monotonic(),elapsedMs=5,response=value,responseBody=body,
                bodySha256=hashlib.sha256(body.encode()).hexdigest())


def samples_from_writes(writes):
    samples=[]
    for i,command in enumerate(writes,1):
        received=1000+i*30;nonce=command[2:10].decode()
        samples.append(parse_reply(compact_line(nonce,received=received,polled=received+5,reported=received+8),nonce,BOOT,compact_sample=True))
    return samples


class StageTraceTest(unittest.TestCase):
    def setUp(self):
        self.samples=samples_from_writes([b'\x1e3'+f'{i:08x}'.encode()+b'\n' for i in range(6)])
        self.before=snapshot();self.after=snapshot([stage_record(s,i) for i,s in enumerate(self.samples,1)])
    def test_six_exact_nonces_and_explicit_stage_semantics(self):
        r=link_stages(self.before,self.after,self.samples)
        self.assertEqual(len(r['records']),6);self.assertFalse(r['txCompleteVerified']);self.assertFalse(r['clockRateCalibrated'])
        self.assertTrue(all(row['latestPollOnly'] for row in r['records']))
    def test_known_previous_ring_overwrite_is_counted(self):
        prior=[stage_record(self.samples[0],i) for i in range(1,7)]
        new=[stage_record(s,i) for i,s in enumerate(self.samples,7)]
        r=link_stages(snapshot(prior),snapshot(prior+new),self.samples)
        self.assertEqual(r['overwrittenPriorRecords'],4)
    def test_mismatched_boot_nonce_marker_raw_or_partial_write_is_rejected(self):
        for field,value in [('nonce','ffffffff'),('pollMarkerMs',0),('replyRaw',0),('writtenBytes',57),
                            ('replyReportedMs',0),('protocol',2),('status','write_failed')]:
            altered=copy.deepcopy(self.after);altered['records'][0][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):link_stages(self.before,altered,self.samples)
        altered=copy.deepcopy(self.after);altered['bootId']='b'*32
        with self.assertRaises(ValueError):link_stages(self.before,altered,self.samples)
    def test_order_negative_bool_and_missing_stage_rejected(self):
        for field,value in [('afterAdcUs',0),('requestParsedUs',None),('pollAttempts',0),
                            ('sequence',True),('writtenBytes',True),('replyEnqueueUs',-1),('adcSampled',1)]:
            altered=copy.deepcopy(self.after);altered['records'][0][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):link_stages(self.before,altered,self.samples)
    def test_multiple_polls_are_explicit_latest_observation(self):
        self.after['records'][0]['pollAttempts']=2
        r=link_stages(self.before,self.after,self.samples);self.assertEqual(r['records'][0]['pollAttempts'],2)
    def test_missing_or_competing_record_cannot_relabel(self):
        self.after['records'].pop();self.after['nextSequence']-=1
        with self.assertRaises(ValueError):link_stages(self.before,self.after,self.samples)
    def test_receipt_body_and_time_are_bound(self):
        from tests.test_action_execution import FakeClock
        row=diagnostic_receipt(self.after,FakeClock());self.assertEqual(check_diagnostics(row),self.after)
        for field,value in [('method','POST'),('endpoint','arm'),('elapsedMs',801),('bodySha256','0'*64)]:
            with self.subTest(field=field),self.assertRaises(ValueError):check_diagnostics({**row,field:value})
    def test_snapshot_pending_and_ring_accounting(self):
        for field,value in [('active',True),('dropped',1),('nextSequence',True),('capacity',9),('txCompleteVerified',True)]:
            with self.subTest(field=field),self.assertRaises(ValueError):validate_snapshot({**self.after,field:value})


class StageCollectorTest(unittest.TestCase):
    def setUp(self):
        self.f=fixture.ProbeTest();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.serial=CompactSerial(self.f.clock);self.change=lambda value:None;self.diagnostic_calls=0
    def http(self,url,endpoint,*,clock):
        if endpoint!='m4_diagnostics':return self.f.read(url,endpoint,clock=clock)
        self.diagnostic_calls+=1
        value=snapshot([stage_record(s,i) for i,s in enumerate(samples_from_writes(self.serial.writes),1)])
        self.change(value)
        return diagnostic_receipt(value,clock)
    def run_probe(self):
        return collect(self.f.root,'http://192.0.2.1','fake','fake',http=self.http,clock=self.f.clock,
                       open_port=lambda *_:self.serial,compact_sample=True,transport_trace=True,firmware_timing=True)
    def test_two_readonly_snapshots_bound_six_samples_and_audit(self):
        r=self.run_probe();self.assertEqual(r['status'],'recorded',r)
        self.assertEqual((r['serialCount'],r['httpCount'],r['diagnosticCount']),(6,8,2))
        self.assertTrue(audit(self.f.root)['ok']);self.assertTrue(self.serial.closed)
    def test_bad_baseline_prevents_serial_requests(self):
        self.change=lambda value:value.update(bootId='b'*32)
        r=self.run_probe();self.assertEqual(r['serialCount'],0);self.assertTrue(self.serial.closed)
        self.assertEqual(r['diagnosticCount'],1);self.assertTrue(audit(self.f.root)['ok'])
    def test_after_mismatch_stops_and_preserves_diagnostic(self):
        def change(value):
            if value['records']:value['records'][0]['nonce']='ffffffff'
        self.change=change;r=self.run_probe()
        self.assertEqual(r['status'],'rejected');self.assertEqual(r['serialCount'],6)
        self.assertTrue((self.f.root/'firmware/after.json').exists());self.assertTrue(audit(self.f.root)['ok'])
    def test_recomputed_summary_rejects_tampering(self):
        from learning.artifacts import describe_artifact
        self.run_probe();p=self.f.root/'result.json';r=json.loads(p.read_text())
        r['firmwareStages']['records'][0]['parseToWriteReturnMs']=0;p.write_text(json.dumps(r))
        e=self.f.root/'events/00000001.json';v=json.loads(e.read_text());v['payload']['summary']=describe_artifact(self.f.root,'result.json');e.write_text(json.dumps(v))
        self.assertFalse(audit(self.f.root)['ok'])


if __name__=='__main__':unittest.main()
