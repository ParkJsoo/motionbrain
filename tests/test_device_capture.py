"""Commissioning collection with virtual devices only; never physical performance."""
import copy
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.action_execution import BoundedActionExecutor
from learning.artifacts import ArtifactStore, StorageError
from learning.device_capture import DeviceCaptureSession, audit_device_capture
from learning.dry_run import JPEG_BYTES
from tests.test_action_execution import FakeClock, FakePort, request, limits
from tests.test_action_port import status


class Transport:
    def __init__(self, clock):
        self.clock, self.host_clock_id, self.recorder = clock, 'collector-clock', None
        self.calls, self.receipts = [], []
        self.port = None
        self.fault = None
        self.frames = 0
        self.m4_reads = 0
        self.state = status()
        self.state['shoulderAngle'].update(correctionActive=False,ageMs=1)
        self.observation = None
    def get(self, device, path, query=None):
        start = self.clock.monotonic();self.clock.sleep(.001)
        receipt=dict(device=device,path=path,query=query or {},method='GET',status=200,
                     hostClockId=self.host_clock_id,requestStartedMonotonic=start,
                     receivedMonotonic=self.clock.monotonic(),headers={})
        body=b''
        if path=='/actions':
            receipt['json']=dict(schemaVersion='motionbrain.bounded_action.v1',bootId='a'*32,
                                clockDomain='controller_boot_millis',enabled=True,active=False,recordCount=0,recordCapacity=32)
            if query:receipt['json']=self.port.lookup(request(query['commandId']))
            if self.fault=='reboot' and self.port.posts:receipt['json']['bootId']='b'*32
        elif path=='/status':
            self.m4_reads+=1
            value=copy.deepcopy(self.state)
            if self.fault=='moving' and self.port.posts:value['shoulderAngle']['angleDeg']+=self.m4_reads
            if self.fault=='output' and self.port.posts:value['motors']['M2']['speed']=1
            if self.fault=='stale_m4':value['shoulderAngle']['ageMs']=21
            if self.fault=='stale_m4_transport':self.clock.sleep(.021);receipt['receivedMonotonic']=self.clock.monotonic()
            if self.fault=='correction':value['shoulderAngle']['correctionActive']=True
            receipt['json']=value
        elif path=='/action_outputs':
            receipt['json']=self.port.outputs() if self.port.last_record else self.port.log('a'*32,[])
            if self.fault=='reactivated':receipt['json']['entries'].append(dict(sequence=3,pwm=5))
            if self.fault=='gap':receipt['json']['entries'].append(dict(sequence=4,pwm=0))
            if self.fault=='dropped':receipt['json']['dropped']=1
        elif path=='/api/detection':
            self.frames+=1
            captured=round(start*1e6) if self.fault!='same_frame' else 123456
            fid='c'*32+':'+str(captured)
            o=dict(sourceFrameId=fid,cameraBootId='c'*32,capturedUs=captured,imageSha256=hashlib.sha256(JPEG_BYTES).hexdigest(),
                   hostClockId='remote-pi-clock',sourceAgeMs=.1,sourceTimeKnown=True,sourceMetadataValid=True,
                   sourceFresh=True,controlEligible=True,held=False,detectionSourceFrameId=fid,detected=True,liveDetected=True,
                   captureHostBoundsCalibrated=True,captureHostBoundsUncertaintySeconds=.0001,
                   sourceClockDomain='camera_boot_us',sourceTimestampSemantics='first_dma_buffer',
                   captureHostBoundsEvidenceRef='fixture:calibration',captureHostBounds=dict(clockId='remote-pi-clock',earliest=1,latest=1),
                   receivedMonotonic=1,inferenceCompletedMonotonic=1)
            if self.fault=='uncalibrated':o['captureHostBoundsCalibrated']=False
            if self.fault=='stale_frame':o['sourceAgeMs']=101
            if self.fault=='shadow':o['anchorAgeMs']=1
            if self.fault=='held':o['held']=True
            if self.fault=='synthetic':o['synthetic']=True
            if self.fault=='old_after' and self.frames>1:o['sourceAgeMs']=70
            self.observation=o;receipt['json']=copy.deepcopy(o)
        elif path=='/api/observation':
            receipt['json']={'observation':copy.deepcopy(self.observation)}
            if self.fault=='different_frame':receipt['json']['observation']['sourceFrameId']='bad'
        elif path=='/api/vision_frame':
            body=JPEG_BYTES if self.fault!='bad_image' else b'bad-image'
        else:raise AssertionError(path)
        receipt['sha256']=hashlib.sha256(body).hexdigest()
        self.calls.append((device,path));self.receipts.append(copy.deepcopy(receipt))
        if self.recorder:self.recorder(receipt)
        return receipt,body


class DeviceCaptureTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.clock=FakeClock();self.transport=Transport(self.clock)
        self.port=FakePort(self.clock);self.port.transport=self.transport;self.port.host_clock_id=self.transport.host_clock_id
        self.transport.port=self.port
        self.core=BoundedActionExecutor(self.root/'journal.sqlite',self.port,limits(),clock=self.clock)
        self.addCleanup(self.core.close)
        self.store=ArtifactStore(self.root/'run')
        self.session=DeviceCaptureSession(self.store,self.core,m4_tolerance_degrees=.2)
    def capture(self):return self.session.capture(request(),confirmed_request=request())
    def test_complete_records_raw_remote_clock_and_independent_m4_without_promotion(self):
        result=self.capture()
        self.assertEqual(result['status'],'recorded',result)
        self.assertEqual(len(self.port.posts),1)
        self.assertEqual(result['before']['observation']['hostClockId'],'remote-pi-clock')
        self.assertEqual(result['before']['m4Sample']['hostClockId'],'collector-clock')
        self.assertFalse(result['before']['clockDomainsMerged'])
        self.assertFalse(result['trainingEligible']);self.assertFalse(result['stationary']['physicalSettledVerified'])
        self.assertGreaterEqual(result['stationary']['spanLowerMs'],50)
        self.assertTrue(audit_device_capture(self.store.root)['ok'])
        self.assertIsNone(self.transport.recorder)
    def test_no_repeat_or_reconfirmation_even_after_failure(self):
        self.transport.fault='stale_frame';self.capture()
        with self.assertRaises(RuntimeError):self.capture()
        self.assertEqual(self.port.posts,[])
    def test_exact_confirmation_before_any_device_io(self):
        result=self.session.capture(request(),confirmed_request={**request(),'pwm':29})
        self.assertEqual(result['executionState'],'not_sent');self.assertFalse(self.transport.calls)
    def test_before_failures_block_all_action_writes(self):
        for fault in ['uncalibrated','stale_frame','stale_m4','stale_m4_transport','held','shadow','synthetic','correction','different_frame','bad_image']:
            with self.subTest(fault=fault):
                # Fresh artifact session and command journal per independent fault.
                with tempfile.TemporaryDirectory() as temp:
                    clock=FakeClock();transport=Transport(clock);port=FakePort(clock)
                    port.transport=transport;port.host_clock_id=transport.host_clock_id;transport.port=port;transport.fault=fault
                    with_core=BoundedActionExecutor(Path(temp)/'journal.sqlite',port,limits(),clock=clock)
                    try:
                        result=DeviceCaptureSession(ArtifactStore(Path(temp)/'run'),with_core,m4_tolerance_degrees=.2).capture(request(),confirmed_request=request())
                        self.assertEqual(result['executionState'],'not_sent',result);self.assertFalse(port.posts)
                    finally:with_core.close()
    def test_final_gate_rechecks_time_spent_in_executor_preflight(self):
        original=self.port.preflight
        def delayed(r):self.clock.sleep(.025);return original(r)
        self.port.preflight=delayed
        result=self.capture()
        self.assertEqual(result['executionState'],'not_sent');self.assertFalse(self.port.posts)
    def test_stationary_failures_preserve_known_terminal_but_reject_capture(self):
        for fault in ['moving','output','reboot','reactivated','gap','dropped','same_frame','old_after']:
            with self.subTest(fault=fault),tempfile.TemporaryDirectory() as temp:
                clock=FakeClock();transport=Transport(clock);port=FakePort(clock)
                port.transport=transport;port.host_clock_id=transport.host_clock_id;transport.port=port;transport.fault=fault
                core=BoundedActionExecutor(Path(temp)/'journal.sqlite',port,limits(),clock=clock)
                try:
                    result=DeviceCaptureSession(ArtifactStore(Path(temp)/'run'),core,m4_tolerance_degrees=.2).capture(request(),confirmed_request=request())
                    self.assertEqual(result['status'],'rejected',result)
                    self.assertEqual(result['executionState'],'known_terminal',result);self.assertEqual(len(port.posts),1)
                    self.assertFalse(result['trainingEligible'])
                finally:core.close()
    def test_unknown_action_never_reads_after_or_settling(self):
        self.port.failure='submit'
        result=self.capture();self.assertEqual(result['executionState'],'outcome_unknown')
        self.assertNotIn('after',result);self.assertNotIn('stationary',result);self.assertEqual(len(self.port.cancels),1)
    def test_persistence_250ms_gate_blocks_dispatch(self):
        original=self.store.save_image
        def slow(*args,**kwargs):value=original(*args,**kwargs);self.clock.sleep(.251);return value
        self.store.save_image=slow
        result=self.capture();self.assertEqual(result['reason'],'persistence_exceeds_250ms');self.assertFalse(self.port.posts)
    def test_storage_failure_before_intent_never_dispatches(self):
        with patch.object(self.store,'save_image',side_effect=StorageError('disk full')):
            with self.assertRaises(StorageError):self.capture()
        self.assertFalse(audit_device_capture(self.store.root)['ok']);self.assertFalse(self.port.posts)
    def test_missing_terminal_or_corrupt_image_audits_unknown(self):
        self.capture()
        (self.store.root/'images/capture/before.jpg').write_bytes(b'corrupt')
        self.assertFalse(audit_device_capture(self.store.root)['ok'])
        (self.store.root/'run.json').unlink()
        self.assertEqual(audit_device_capture(self.store.root)['executionState'],'outcome_unknown')
    def test_malformed_seal_payload_audits_invalid_instead_of_raising(self):
        import json
        self.capture()
        last=sorted((self.store.root/'events').glob('*.json'))[-1]
        event=json.loads(last.read_text());event['payload']=[];last.write_text(json.dumps(event))
        self.assertFalse(audit_device_capture(self.store.root)['ok'])
    def test_existing_terminal_id_never_becomes_a_new_transition(self):
        self.core.propose(request());self.core.confirm('first',confirmed_request=request(),dispatch_guard=lambda r:None)
        result=self.capture()
        self.assertEqual(result['reason'],'command_id_already_reserved_no_new_capture')
        self.assertEqual(self.transport.calls,[]);self.assertEqual(len(self.port.posts),1)
    def test_rejected_image_is_preserved(self):
        self.transport.fault='bad_image';self.capture()
        self.assertEqual((self.store.root/'images/capture/before-rejected-0.jpg').read_bytes(),b'bad-image')
    def test_late_terminal_persistence_has_no_success_seal(self):
        original=self.store.write_json
        def slow(path,value):
            result=original(path,value)
            if path=='run.json':self.clock.sleep(.251)
            return result
        self.store.write_json=slow
        with self.assertRaises(StorageError):self.capture()
        self.assertFalse(audit_device_capture(self.store.root)['ok'])
    def test_raw_write_failure_is_incomplete_and_keeps_unwritten_receipts(self):
        original=self.store.write_json
        def failed(path,value):
            if path.startswith('raw/'):raise StorageError('one failed receipt')
            return original(path,value)
        self.store.write_json=failed
        with self.assertRaises(StorageError):self.capture()
        self.assertEqual(len(self.session.pending),5)
        self.assertFalse(audit_device_capture(self.store.root)['ok']);self.assertFalse(self.port.posts)
    def test_storage_failure_inside_final_guard_is_not_retried_after_executor_returns(self):
        original_preflight=self.port.preflight
        def preflight(r):
            self.transport.get('controller','/status')
            return original_preflight(r)
        self.port.preflight=preflight
        original_write=self.store.write_json;failures=[]
        def failed(path,value):
            if path=='raw/http-0006.json':
                failures.append(path)
                raise StorageError('guard disk failure')
            return original_write(path,value)
        self.store.write_json=failed
        with self.assertRaises(StorageError):self.capture()
        self.assertEqual(len(failures),1);self.assertFalse(self.port.posts)
        self.assertFalse(audit_device_capture(self.store.root)['ok'])
    def test_outputs_and_m4_are_checked_again_after_final_image(self):
        for fault in ('reactivated','moving'):
            with self.subTest(fault=fault),tempfile.TemporaryDirectory() as temp:
                clock=FakeClock();transport=Transport(clock);port=FakePort(clock)
                port.transport=transport;port.host_clock_id=transport.host_clock_id;transport.port=port
                original=transport.get
                def mutate(device,path,query=None):
                    if path=='/api/detection' and transport.frames==1:transport.fault=fault
                    return original(device,path,query)
                transport.get=mutate
                core=BoundedActionExecutor(Path(temp)/'journal.sqlite',port,limits(),clock=clock)
                try:
                    result=DeviceCaptureSession(ArtifactStore(Path(temp)/'run'),core,m4_tolerance_degrees=.2).capture(request(),confirmed_request=request())
                    self.assertEqual(result['status'],'rejected',result)
                    self.assertIn(result['reason'],('m4_moved_after_stationary_window','output_reactivated_or_trace_gap_during_settling'))
                finally:core.close()
    def test_angle_wrap_is_small_measured_change(self):
        original=self.transport.get
        def wrapped(*args,**kwargs):
            self.transport.state['shoulderAngle']['angleDeg']=359.95 if self.transport.m4_reads%2 else .05
            return original(*args,**kwargs)
        self.transport.get=wrapped
        self.assertEqual(self.capture()['status'],'recorded')
    def test_stationary_budget_cannot_extend_on_slow_reads(self):
        original=self.transport.get
        def slow(*args,**kwargs):
            if args[1]=='/action_outputs':self.clock.sleep(.501)
            return original(*args,**kwargs)
        self.transport.get=slow
        result=self.capture()
        self.assertEqual(result['reason'],'stationary_budget_exceeded')
        self.assertEqual(len(self.port.posts),1)
    def test_interruption_never_resumes_and_restores_recorder(self):
        self.port.failure='interrupt'
        with self.assertRaises(KeyboardInterrupt):self.capture()
        self.assertFalse(audit_device_capture(self.store.root)['ok']);self.assertIsNone(self.transport.recorder)
        with self.assertRaises(RuntimeError):self.capture()



class DeviceCaptureHTTPTest(unittest.TestCase):
    def test_existing_http_executor_and_exact_observation_endpoints_end_to_end(self):
        import json
        import threading
        import urllib.parse
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        from learning.action_port import BoundedActionHTTP
        with tempfile.TemporaryDirectory() as temp:
            clock=FakeClock();upstream=Transport(clock);fake=FakePort(clock);upstream.port=fake
            posts=[]
            class Handler(BaseHTTPRequestHandler):
                def log_message(self,*args):pass
                def send(self,value,raw=False):
                    body=value if raw else json.dumps(value).encode()
                    self.send_response(200)
                    self.send_header('Content-Type','image/jpeg' if raw else 'application/json')
                    self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
                def do_GET(self):
                    parts=urllib.parse.urlsplit(self.path);query=dict(urllib.parse.parse_qsl(parts.query))
                    receipt,body=upstream.get('perception' if parts.path.startswith('/api/') else 'controller',parts.path,query)
                    self.send(receipt.get('json',body),raw='json' not in receipt)
                def do_POST(self):
                    posts.append(self.path)
                    if self.headers.get('X-MotionBrain-Token')!='fixture-only':
                        self.send_response(403);self.end_headers();return
                    fields=dict(urllib.parse.parse_qsl(self.rfile.read(int(self.headers['Content-Length'])).decode()))
                    if self.path=='/actions':
                        r={k:int(v) if k in ('motor','pwm','durationMs','percent') else float(v) if k=='targetDegrees' else v for k,v in fields.items()}
                        self.send(fake.submit(r))
                    elif self.path=='/actions/cancel':self.send(fake.cancel(request(fields['commandId'])))
                    else:raise AssertionError(self.path)
            server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                url=f'http://127.0.0.1:{server.server_port}'
                port=BoundedActionHTTP(url,camera_url=url,perception_url=url,clock=clock,
                                      writes_enabled=True,token_provider=lambda:'fixture-only')
                core=BoundedActionExecutor(Path(temp)/'journal.sqlite',port,limits(),clock=clock)
                try:
                    store=ArtifactStore(Path(temp)/'run')
                    result=DeviceCaptureSession(store,core,m4_tolerance_degrees=.2).capture(request(),confirmed_request=request())
                    self.assertEqual(result['status'],'recorded',result)
                    self.assertEqual(posts,['/actions'])
                    self.assertTrue(audit_device_capture(store.root)['ok'])
                    for path in store.root.rglob('*.json'):
                        self.assertNotIn('fixture-only',path.read_text())
                finally:core.close()
            finally:server.shutdown();thread.join(timeout=2);server.server_close()


if __name__=='__main__':unittest.main()
