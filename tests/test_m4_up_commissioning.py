"""Synthetic controller/camera integration; no real device or recorder access."""
import copy
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import urllib.parse
from types import SimpleNamespace

import cv2
import numpy as np

from learning import m4_up_commissioning as m
from learning.artifacts import ArtifactStore
from learning.supervised_collection import PROFILES, plan_digest
from learning.supervised_transport import SequencePort, validate_pulse
from tests.test_mac_clock_bridge import sample
from tests.test_supervised_transport import Clock, Controller, Response, pulse


class Device(Controller):
    def __init__(self, clock):
        super().__init__(clock); self.boot='a'*32
        self.after_request=None
    def open(self, request, timeout):
        path=urllib.parse.urlsplit(request.full_url).path
        if request.data is None and path in ('/collection_status','/action_outputs'):
            self.clock.sleep(.01);self.requests.append((path,None))
            if self.after_request:self.after_request(path)
            if path=='/collection_status':return Response(dict(self.status(),schemaVersion='collection.v1'))
            q=urllib.parse.parse_qs(urllib.parse.urlsplit(request.full_url).query)
            after=int(q['after'][0]);entries=[e for e in self.entries if e['sequence']>after][:8]
            return Response(dict(bootId=self.boot,evidence='driver_write',clockDomain='controller_boot_millis',
                oldestSequence=self.entries[0]['sequence'],latestSequence=self.entries[-1]['sequence'],gap=False,
                hasMore=bool(entries and entries[-1]['sequence']<self.entries[-1]['sequence']),entries=entries))
        return super().open(request,timeout)


def scope():
    return dict(schemaVersion=m.SCHEMA,trialId='m4-up-test',controllerUrl='http://controller.invalid',
        controllerBootId='a'*32,hostClockId='commissioning-test',implementationSha256=m.implementation_digest(),
        context=dict(physicalView='synthetic-fixed',poseRegime='contact-local',object='cylinder',
            supportMode='table_supported',externalDeviceId='camera-id',externalViewId='test-view',gripperCameraBootId='b'*32),
        pulse=copy.deepcopy(m.PULSE),defaultSpeed=255,maxCommands=1,expectedStartAngleDeg=220.,
        startAngleToleranceDeg=.5,maxObservedDeltaDeg=1.,maxSeconds=60.,maxObservationAgeSeconds=10.,postOffSeconds=3.)


class Observer:
    def __init__(self, root, clock, planned):
        self.root,self.clock,self.scope=root,clock,planned
        self.payloads={};self.started=False;self.closed=False;self.index=0;self.latest_record=None
        self.on_before=None;self.on_after=None;self.on_latest=None;self.on_close=None
    def save(self,name,value):
        path=self.root/name;path.parent.mkdir(parents=True,exist_ok=True)
        raw=value if isinstance(value,bytes) else json.dumps(value).encode()
        path.write_bytes(raw);return m.sha(raw)
    def start(self,**kwargs):
        self.started=True;self.bridge=sample(self.clock.now,self.scope['hostClockId'])
        self.clock_sha=self.save('external/host-ingress/clock-before.json',self.bridge)
    def check(self):pass
    def latest(self):
        if self.on_latest:self.on_latest()
        return copy.deepcopy(self.latest_record)
    def observe(self,*,after_off,deadline,tick):
        if after_off is not None:
            while self.clock.now<after_off+3.5:self.clock.sleep(.2);tick()
        self.clock.sleep(.3);tick()
        i=self.index;self.index+=1;t=self.clock.now-.2
        low,high=m.validate_sample(self.bridge)
        native=f'native-{i}'.encode();pixels=bytes([i+1])*8*8*4;pixel_sha=m.sha(pixels)
        self.payloads[native]=pixels;meta_name=f'frame-{i:04d}.json';image_name=f'frame-{i:04d}.bgra.lzbitmap'
        image_sha=self.save('external/'+image_name,native);config='{"format":"fixed"}'
        meta=dict(schemaVersion='motionbrain.mac_camera_frame.v2',sessionId=self.scope['hostClockId'],sourceFrameId=f"{self.scope['hostClockId']}:{i}",
            clockDomain='coremedia_host_clock',timestampSemantics='cmsamplebuffer_presentation_timestamp',storageEncoding=m.ENCODING,
            deviceUniqueId='camera-id',viewConfigurationId='test-view',imageFile=image_name,imageSha256=image_sha,pixelSha256=pixel_sha,
            rawByteLength=len(pixels),width=8,height=8,frameIndex=i,pixelFormat='BGRA8',rowBytes=32,
            storedByteLength=len(native),videoMirrored=False,videoRotationAngle=0,
            pts=dict(value=int(t*1000000),timescale=1000000,epoch=0,flags=1),callbackHostSeconds=t+.01,
            actualConfigurationJson=config,configurationSha256=m.sha(config.encode()),convertedHostPtsSeconds=t)
        meta_sha=self.save('external/'+meta_name,meta)
        frame=dict(frameIndex=i,frameId=meta['sourceFrameId'],qualityFlags=[],metadataFile=meta_name,imageFile=image_name,
            metadataSha256=meta_sha,imageSha256=image_sha,pixelSha256=pixel_sha,width=8,height=8,rawByteLength=len(pixels),storageEncoding=m.ENCODING,
            storedByteLength=len(native),convertedHostPtsSeconds=t,callbackHostSeconds=t+.01,
            clockOffsetBoundsSeconds=[low,high],sourceEarliest=t+low,sourceLatest=t+high,stdoutReadAt=t+.03,persistenceMs=10.)
        ingress=f'host-ingress/{i:04d}.json';ingress_sha=self.save('external/'+ingress,frame)
        label='before' if after_off is None else 'after';stamp=1000+i*100
        body=cv2.imencode('.jpg',np.full((32,32,3),100+i,np.uint8))[1].tobytes()
        receipt=dict(sessionId=self.scope['hostClockId'],hostClockId=self.scope['hostClockId'],clockDomain='python_monotonic',
            requestStartedAt=t,receivedAt=t+.03,persistedAt=t+.04,sha256=m.sha(body),headers={
            'X-MotionBrain-Camera-Boot-Id':'b'*32,'X-MotionBrain-Captured-Us':str(stamp),'X-MotionBrain-Capture-Requested-Us':str(stamp-1),
            'X-MotionBrain-Frame-Id':'b'*32+':'+str(stamp),'X-MotionBrain-Capture-Mode':'after_request','X-MotionBrain-Clock-Domain':'camera_boot_us',
            'X-MotionBrain-Timestamp-Semantics':'first_dma_buffer'})
        image_sha=self.save(f'gripper/esp32-{label}.jpg',body);meta_sha=self.save(f'gripper/esp32-{label}.json',receipt)
        record=dict(hostClockId=self.scope['hostClockId'],context=copy.deepcopy(self.scope['context']),stable=True,
            external=dict(root='external',ingressFile=ingress,ingressSha256=ingress_sha,clockSha256=self.clock_sha),
            gripper=dict(root='gripper',label=label,imageSha256=image_sha,metadataSha256=meta_sha),
            onsite=dict(operatorPresent=True,clearanceVerified=True,jawContactVerified=True,tableSupportVerified=after_off is None))
        review=f'review-{label}.json';review_sha=self.save(review,record)
        record.update(evidenceFile=review,evidenceSha256=review_sha)
        self.latest_record=copy.deepcopy(record)
        hook=self.on_before if after_off is None else self.on_after
        if hook:hook(record)
        return record
    def close(self):
        self.closed=True
        if self.on_close:self.on_close()
        return dict(status='completed')
    def rebind_external(self,record,*,native_changes=None,ingress_changes=None,remove_native=()):
        source=record['external'];prefix=source['root']+'/'
        frame=json.loads((self.root/prefix/source['ingressFile']).read_text())
        meta=json.loads((self.root/prefix/frame['metadataFile']).read_text())
        meta.update(native_changes or {})
        for key in remove_native:meta.pop(key,None)
        frame['metadataSha256']=self.save(prefix+frame['metadataFile'],meta)
        frame.update(ingress_changes or {})
        source['ingressSha256']=self.save(prefix+source['ingressFile'],frame)
        record['evidenceSha256']=self.save(record['evidenceFile'],{k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')})
        self.latest_record=copy.deepcopy(record)
    def decode(self,body,size,digest):
        raw=self.payloads[body]
        if len(raw)!=size or m.sha(raw)!=digest:raise ValueError('compressed_frame_pixels_invalid')
        return raw


class CommissioningTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.repo=Path(self.tmp.name).resolve();self.evidence=self.repo/'.local/evidence';self.evidence.mkdir(parents=True)
        self.root_patch=patch.object(m,'REPO',self.repo);self.root_patch.start();self.addCleanup(self.root_patch.stop)
        self.clock=Clock();self.device=Device(self.clock);self.scope=scope()
        self.observer=Observer(self.evidence,self.clock,self.scope)
        self.store=ArtifactStore(self.repo/'.local/run')
        self.trial=m.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.evidence,clock=self.clock)
        self.trial.port.opener=self.device
        self.decode_patch=patch.object(m,'decode',side_effect=self.observer.decode);self.decode_patch.start();self.addCleanup(self.decode_patch.stop)
    def run_trial(self):return self.trial.run(confirmed_scope_sha256=plan_digest(self.scope))
    def posts(self):return [(p,v) for p,v in self.device.requests if v is not None]
    def row(self):
        import sqlite3
        with sqlite3.connect(self.repo/'.local/visual-actions/controller-commands.sqlite') as db:
            return db.execute('SELECT phase FROM supervised_blocks WHERE session_id=?',(self.scope['trialId'],)).fetchone()[0]
    def test_construct_validate_and_general_rejection_make_no_requests(self):
        self.assertFalse(self.device.requests)
        self.assertEqual(PROFILES['shoulder']['directions'],{'down':(-1,35,650)})
        with self.assertRaisesRegex(ValueError,'m4_up'):validate_pulse(copy.deepcopy(m.PULSE))
        with self.assertRaisesRegex(ValueError,'scoped_commissioning'):self.trial.port.execute(m.PULSE,lambda:None)
        self.assertFalse(self.device.requests)
        self.assertFalse((self.repo/'.local/visual-actions').exists())
    def test_exact_scope_hash_required_before_journal_or_controller(self):
        with self.assertRaisesRegex(ValueError,'exact_unconsumed'):self.trial.run(confirmed_scope_sha256='0'*64)
        self.assertFalse((self.repo/'.local/visual-actions').exists());self.assertFalse(self.device.requests)
    def test_fixed_profile_normal_flow_uses_only_one_run_and_no_absolute_target(self):
        self.device.after_run=lambda d:setattr(d,'angle',220.25)
        r=self.run_trial();self.assertEqual(r['status'],'completed',r)
        self.assertTrue(r['finalIdleOffVerified']);self.assertTrue(r['mayHaveRun']);self.assertEqual(self.row(),'closed')
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)
        self.assertIn(('/sequence',dict(action='add',joint='shoulder',direction='up',speed='16',duration='200')),self.posts())
        self.assertEqual(self.device.default,100);self.assertFalse(self.device.queue);self.assertFalse(any(self.device.speeds.values()))
        self.assertTrue(self.observer.closed);self.assertFalse(r['modelSelected']);self.assertFalse(r['trainingEligible']);self.assertFalse(r['liftVerified'])
        self.assertFalse(any(p in ('/shoulder','/actions') for p,v in self.posts()))
        for p in self.store.root.glob('*.json'):self.assertNotIn('private-token',p.read_text())
    def test_same_instance_and_recreated_scope_never_dispatch_again(self):
        self.assertEqual(self.run_trial()['status'],'completed');count=len(self.device.requests)
        with self.assertRaisesRegex(ValueError,'exact_unconsumed'):self.run_trial()
        store=ArtifactStore(self.repo/'.local/second-run')
        other=m.M4UpCommissioning(self.scope,self.observer,'private-token',store,evidence_root=self.evidence,clock=self.clock);other.port.opener=self.device
        self.assertEqual(other.run(confirmed_scope_sha256=plan_digest(self.scope))['status'],'failed')
        self.assertEqual(len(self.device.requests),count)
    def test_scope_mutation_and_code_change_cannot_reach_device(self):
        self.trial.scope['pulse']['expectedPwm']=41
        self.assertEqual(self.run_trial()['status'],'failed');self.assertFalse(self.device.requests)
    def test_code_mutation_after_construction_consumes_but_never_dispatches(self):
        with patch.object(m,'implementation_digest',return_value='0'*64):r=self.run_trial()
        self.assertEqual(r['status'],'failed');self.assertFalse(self.device.requests);self.assertEqual(self.row(),'off_unverified')
    def test_busy_shared_owner_and_unresolved_other_block(self):
        path=self.repo/'.local/visual-actions/controller-commands.sqlite'
        j=m.BoundedActionJournal(str(path),10000)
        try:r=self.run_trial()
        finally:j.close()
        self.assertEqual(r['status'],'failed');self.assertFalse(self.device.requests)
    def test_unresolved_supervised_block_blocks_before_camera_or_reads(self):
        j=m.BoundedActionJournal(str(self.repo/'.local/visual-actions/controller-commands.sqlite'),10000)
        with j.connection:
            j.connection.execute('CREATE TABLE supervised_blocks (session_id TEXT PRIMARY KEY, plan_hash TEXT, output_path TEXT, phase TEXT, result_json TEXT)')
            j.connection.execute("INSERT INTO supervised_blocks VALUES ('other','hash','other','pending',NULL)")
        j.close();r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertFalse(self.device.requests);self.assertFalse(self.observer.started)
    def test_wrong_boot_stale_sensor_and_foreign_queue_never_send_writes(self):
        self.device.boot='c'*32
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertFalse(self.posts())
    def test_start_angle_outside_fixed_window_never_sends_writes(self):
        self.device.angle=220.6;r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertFalse(self.posts())
    def test_missing_onsite_or_changed_context_rejected_before_motor_writes(self):
        self.observer.on_before=lambda r:r['onsite'].update(clearanceVerified=False)
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertFalse(self.posts())
    def test_tampered_camera_source_cannot_reach_arm(self):
        self.observer.on_before=lambda r:(self.evidence/'gripper/esp32-before.jpg').write_bytes(b'changed')
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertFalse(self.posts())
    def test_negative_persistence_with_consistent_review_hash_never_reaches_motor_writes(self):
        self.observer.on_before=lambda record:self.observer.rebind_external(record,ingress_changes={'persistenceMs':-100.})
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertFalse(self.posts())
    def test_missing_native_pts_with_consistent_review_hash_never_reaches_motor_writes(self):
        self.observer.on_before=lambda record:self.observer.rebind_external(record,remove_native=('pts',))
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertFalse(self.posts())
    def test_hash_consistent_but_invalid_native_format_and_receipt_are_rejected(self):
        self.observer.start();record=self.observer.observe(after_off=None,deadline=60,tick=lambda:None)
        original=copy.deepcopy(record)
        meta_path=self.evidence/'external/frame-0000.json';meta_raw=meta_path.read_bytes()
        ingress_path=self.evidence/'external/host-ingress/0000.json';ingress_raw=ingress_path.read_bytes()
        cases=[dict(remove_native=(k,)) for k in ('pts','callbackHostSeconds','frameIndex','pixelFormat','rowBytes','storedByteLength','videoMirrored','videoRotationAngle')]
        cases += [dict(native_changes={'pts':dict(value=0,timescale=1,epoch=1,flags=1)}),
                  dict(native_changes={'pts':dict(value=0,timescale=1,epoch=0,flags=0)}),
                  dict(native_changes={'pts':dict(value=0,timescale=0,epoch=0,flags=1)}),
                  dict(native_changes={'callbackHostSeconds':-1}),dict(native_changes={'rowBytes':1}),
                  dict(ingress_changes={'persistenceMs':251}),dict(ingress_changes={'stdoutReadAt':4.}),
                  dict(ingress_changes={'persistenceMs':-100})]
        for change in cases:
            with self.subTest(change=change):
                meta_path.write_bytes(meta_raw);ingress_path.write_bytes(ingress_raw);current=copy.deepcopy(original)
                self.observer.rebind_external(current,**change)
                with self.assertRaises(ValueError):m._external_frame(self.evidence,current['external'],self.scope)
        self.assertFalse(self.device.requests)
    def test_post_frame_native_pts_regression_is_failed_after_single_command(self):
        self.observer.on_after=lambda record:self.observer.rebind_external(record,native_changes={'pts':dict(value=0,timescale=1,epoch=0,flags=1)})
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertTrue(r['finalIdleOffVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)
    def test_before_input_changed_at_run_guard_blocks_run_and_cleans_up(self):
        self.observer.on_latest=lambda:self.observer.latest_record['context'].update(poseRegime='changed')
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertFalse(r['mayHaveRun'])
        self.assertFalse(any(v=={'action':'run'} for _,v in self.posts()));self.assertTrue(r['finalIdleOffVerified']);self.assertEqual(self.row(),'closed')
    def test_pre_run_evidence_storage_delay_cannot_renew_input_or_dispatch(self):
        original=self.store.write_json
        def write(name,value):
            original(name,value)
            if name=='pre-run-guard.json':self.clock.sleep(.2)
        with patch.object(self.store,'write_json',side_effect=write):r=self.run_trial()
        self.assertEqual(r['status'],'failed');self.assertFalse(any(v=={'action':'run'} for _,v in self.posts()))
        self.assertTrue(r['finalIdleOffVerified'])
    def test_guard_storage_crossing_source_age_limit_blocks_run_below_guard_time_cap(self):
        original=self.store.write_json
        def write(name,value):
            original(name,value)
            if name.endswith('-intent.json') and value.get('payload')=={'action':'run'}:
                native=json.loads((self.evidence/'external/host-ingress/0000.json').read_text())
                grip=json.loads((self.evidence/'gripper/esp32-before.json').read_text())
                earliest=min(native['sourceEarliest'],grip['requestStartedAt'])
                self.clock.sleep(earliest+9.95-self.clock.now)
                # Isolate video ageing from the separate controller proof cap.
                self.trial.port.proof_started={'status':self.clock.now,'outputs':self.clock.now}
            if name=='pre-run-guard.json':self.clock.sleep(.09)
        with patch.object(self.store,'write_json',side_effect=write):r=self.run_trial()
        self.assertEqual(r['status'],'failed');self.assertFalse(r['mayHaveRun'])
        self.assertFalse(any(v=={'action':'run'} for _,v in self.posts()));self.assertTrue(r['finalIdleOffVerified'])
    def delayed_arm_proof(self,refresh_delay=.01,refresh_failure=False):
        original=self.device.open;reads=[]
        def delayed(request,timeout):
            armed=request.data is None and request.full_url.endswith('/collection_status') and self.device.state=='ARMED' and not self.device.running
            if armed:
                reads.append(self.clock.now)
                if len(reads)==2 and refresh_failure:raise TimeoutError('failed fresh read')
                self.clock.sleep(.24 if len(reads)==1 else refresh_delay)
            return original(request,timeout)
        self.device.open=delayed
        return reads
    def test_successful_slow_arm_proof_gets_one_fresh_read_before_run(self):
        reads=self.delayed_arm_proof();result=self.run_trial()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(len(reads),2);self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),1)
        proof=json.loads((self.store.root/'controller-proof-refresh.json').read_text())
        self.assertEqual(proof['refreshed'],'status');self.assertGreater(proof['after']['status'],proof['before']['status'])
        self.assertEqual(proof['after']['outputs'],proof['before']['outputs'])
        guard=json.loads((self.store.root/'pre-run-guard.json').read_text())
        self.assertTrue(all(0<=guard['validatedAt']-t<=.25 for t in guard['controllerProofStarted'].values()))
    def test_still_stale_after_bounded_successful_refreshes_never_sends_run(self):
        reads=self.delayed_arm_proof(refresh_delay=.30);result=self.run_trial()
        self.assertEqual(result['status'],'failed');self.assertEqual(len(reads),3)
        self.assertEqual(self.trial.port.proof_refresh_count,4)
        self.assertFalse(any(p=={'action':'run'} for _,p in self.posts()))
        self.assertFalse(result['mayHaveRun']);self.assertTrue(result['finalIdleOffVerified'])
    def test_old_output_proof_refresh_rejects_even_matching_nonzero_history(self):
        original=self.trial.port._refresh_run_proof
        def refresh(**kwargs):
            self.trial.port.proof_started={'status':self.clock.now-.01,'outputs':self.clock.now-.20}
            self.device.append(4,10);self.device.append(4,0)
            return original(**kwargs)
        with patch.object(self.trial.port,'_refresh_run_proof',side_effect=refresh):result=self.run_trial()
        self.assertEqual(result['status'],'failed');self.assertFalse(result['mayHaveRun'])
        self.assertFalse(any(p=={'action':'run'} for _,p in self.posts()));self.assertTrue(result['finalIdleOffVerified'])
    def test_refresh_storage_delay_cannot_extend_proof_age(self):
        self.delayed_arm_proof();original=self.store.write_json
        def slow(name,value):
            original(name,value)
            if name.startswith('controller-proof-refresh'):self.clock.sleep(.30)
        with patch.object(self.store,'write_json',side_effect=slow):result=self.run_trial()
        self.assertEqual(result['status'],'failed');self.assertFalse(result['mayHaveRun'])
        self.assertFalse(any(p=={'action':'run'} for _,p in self.posts()));self.assertTrue(result['finalIdleOffVerified'])

    def test_failed_fresh_read_is_not_retried_or_dispatched(self):
        reads=self.delayed_arm_proof(refresh_failure=True);result=self.run_trial()
        self.assertEqual(result['status'],'failed');self.assertEqual(len(reads),2)
        self.assertFalse(any(p=={'action':'run'} for _,p in self.posts()))
        self.assertFalse(result['mayHaveRun']);self.assertTrue(result['finalIdleOffVerified'])

    def test_successful_169ms_outputs_and_63ms_status_reserve_dispatch_headroom(self):
        original_open=self.device.open;counts={'status':0,'outputs':0}
        def delayed(request,timeout):
            path=urllib.parse.urlsplit(request.full_url).path
            if request.data is None and self.device.state=='ARMED' and not self.device.running:
                if path=='/collection_status':
                    counts['status']+=1
                    if counts['status']==2:self.clock.sleep(.053)
                elif path=='/action_outputs':
                    counts['outputs']+=1
                    if counts['outputs']==1:self.clock.sleep(.159)
            return original_open(request,timeout)
        self.device.open=delayed;original_write=self.store.write_json
        def write(name,value):
            original_write(name,value)
            if name=='controller-proof-refresh.json':self.clock.sleep(.012)
            if name.endswith('-intent.json') and value.get('payload')=={'action':'run'}:self.clock.sleep(.005)
            if name=='pre-run-guard.json':self.clock.sleep(.025)
        with patch.object(self.store,'write_json',side_effect=write):result=self.run_trial()
        self.assertEqual(result['status'],'completed',result)
        self.assertEqual(self.trial.port.proof_refresh_count,2)
        prepared=json.loads((self.store.root/'controller-pre-dispatch-admission.json').read_text())
        self.assertEqual(prepared['preparationCycles'],2)
        self.assertTrue(all(prepared['preparedAt']-at<=.20 for at in prepared['controllerProofStarted'].values()))
        self.assertEqual(sum(p=={'action':'run'} for _,p in self.posts()),1)

    def test_slow_admission_publication_cannot_consume_prepared_headroom(self):
        original=self.trial.port._refresh_run_proof
        def refresh(**kwargs):
            original(**kwargs)
            self.trial.port.proof_started={'status':self.clock.now-.19,'outputs':self.clock.now-.19}
        original_write=self.store.write_json
        def write(name,value):
            original_write(name,value)
            if name=='controller-pre-dispatch-admission.json':self.clock.sleep(.045)
        with patch.object(self.trial.port,'_refresh_run_proof',side_effect=refresh),patch.object(self.store,'write_json',side_effect=write):
            result=self.run_trial()
        self.assertEqual(result['status'],'failed');self.assertFalse(result['mayHaveRun'])
        self.assertFalse(any(p=={'action':'run'} for _,p in self.posts()))
        intents=[json.loads(path.read_text()) for path in self.store.root.glob('http-*-intent.json')]
        self.assertFalse(any(value.get('payload')=={'action':'run'} for value in intents))

    def test_previous_implementation_is_readonly_auditable_but_not_executable(self):
        recorded=copy.deepcopy(self.scope)
        recorded['implementationSha256']='4848a96eb9411bf486fa3bc662fafa3687e723910a59e1d6032a25e7c131f179'
        self.assertEqual(m.validate_recorded_scope(recorded),recorded)
        with self.assertRaisesRegex(ValueError,'implementation_changed'):m.validate_scope(recorded)
        noncontact=copy.deepcopy(recorded)
        noncontact.update(schemaVersion=m.NONCONTACT_SCHEMA,load='empty')
        noncontact['context'].update(supportMode='empty_no_object',object='none')
        self.assertEqual(m.validate_recorded_scope(noncontact),noncontact)
        with self.assertRaisesRegex(ValueError,'implementation_changed'):m.validate_scope(noncontact)
        noncontact['implementationSha256']='ed3333408c4662d442c7bd61bec579c1da16dd8e9eae79448f261f57d9b249ba'
        with self.assertRaisesRegex(ValueError,'recorded_implementation_unknown'):m.validate_recorded_scope(noncontact)

    def test_shared_preparation_can_be_checked_in_explicit_IDLE_readonly_port(self):
        class ReadOnly(m._CommissioningPort):
            read_only_proof=True
            def _call(self,path,payload=None,**kwargs):
                m.require(payload is None and not kwargs.get('run'),'readonly_write_forbidden')
                return super()._call(path,payload,**kwargs)
        owner=SimpleNamespace(scope=self.scope,deadline=self.clock.now+60.,angles=[],baseline_angle=220.,_guard=lambda:None)
        port=ReadOnly(self.scope,'test-token',self.store,owner,clock=self.clock);port.opener=self.device
        port.check_idle()
        m.prepare_controller_proof(port,clock=self.clock,guard=owner._guard,save=self.store.write_json,expected_state='IDLE')
        self.clock.sleep(.025)
        self.assertTrue(all(0<=self.clock.now-at<=.25 for at in port.proof_started.values()))
        self.assertFalse(any(payload is not None for _,payload in self.device.requests))
        for path,payload in (('/command',{'cmd':'arm'}),('/sequence',{'action':'run'}),('/command',{'cmd':'stop'})):
            with self.assertRaisesRegex(ValueError,'readonly_write_forbidden'):port._call(path,payload)
        self.assertFalse(any(payload is not None for _,payload in self.device.requests))

    def test_live_commissioning_port_cannot_replace_ARMED_proof_with_IDLE(self):
        with self.assertRaisesRegex(ValueError,'proof_expected_state'):
            self.trial.port._refresh_run_proof(require_ready=False,expected_state='IDLE')

    def test_slow_run_intent_cannot_reuse_old_angle_or_boot_proof(self):
        original=self.store.write_json
        def write(name,value):
            original(name,value)
            if name.endswith('-intent.json') and value.get('payload')=={'action':'run'}:
                self.clock.sleep(1.);self.device.angle=222.;self.device.boot='c'*32
        with patch.object(self.store,'write_json',side_effect=write):r=self.run_trial()
        self.assertEqual(r['status'],'failed');self.assertFalse(r['mayHaveRun'])
        self.assertFalse(any(v=={'action':'run'} for _,v in self.posts()))
    def test_final_guard_save_cannot_age_controller_proof_past_fixed_cap(self):
        original=self.store.write_json
        def write(name,value):
            original(name,value)
            if name.endswith('-intent.json') and value.get('payload')=={'action':'run'}:self.clock.sleep(.20)
            if name=='pre-run-guard.json':self.clock.sleep(.09)
        with patch.object(self.store,'write_json',side_effect=write):r=self.run_trial()
        self.assertTrue((self.store.root/'pre-run-guard.json').exists())
        self.assertEqual(r['status'],'failed');self.assertFalse(r['mayHaveRun'])
        self.assertFalse(any(v=={'action':'run'} for _,v in self.posts()));self.assertTrue(r['finalIdleOffVerified'])
    def test_run_reply_loss_is_unknown_no_run_resend(self):
        def lost(d):raise TimeoutError('lost RUN')
        self.device.after_run=lost;r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertTrue(r['mayHaveRun'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1);self.assertTrue(r['finalIdleOffVerified'])
        self.assertEqual(r['commandOutcome'],'unknown');self.assertTrue(r['requiresOperatorReview'])
        self.assertEqual(self.row(),'pending');count=len(self.device.requests)
        next_scope=copy.deepcopy(self.scope);next_scope['trialId']='replacement-not-allowed'
        store=ArtifactStore(self.repo/'.local/replacement')
        other=m.M4UpCommissioning(next_scope,self.observer,'private-token',store,evidence_root=self.evidence,clock=self.clock)
        other.port.opener=self.device
        self.assertEqual(other.run(confirmed_scope_sha256=plan_digest(next_scope))['status'],'failed')
        self.assertEqual(len(self.device.requests),count)
    def test_failed_stop_is_not_retried_even_in_outer_finalization(self):
        self.device.fail_stop=True;r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertFalse(r['finalIdleOffVerified'])
        self.assertEqual(sum(v=={'cmd':'stop'} for _,v in self.posts()),1);self.assertEqual(self.row(),'off_unverified')
    def test_excursion_is_failure_but_off_proof_stays_distinct(self):
        self.device.after_run=lambda d:setattr(d,'angle',221.1)
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertTrue(r['finalIdleOffVerified'])
        self.assertTrue(any(x['angleDeg']>221 for x in r['angleObservations']))
    def test_other_axis_and_negative_output_stop_without_second_run(self):
        self.device.after_run=lambda d:d.append(3,10)
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)
        self.assertFalse(any(self.device.speeds.values()))
    def test_output_during_camera_close_cannot_be_completed(self):
        self.observer.on_close=lambda:self.device.append(2,10)
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertTrue(r['finalIdleOffVerified'])
    def test_camera_close_failure_preserves_failed_session(self):
        def fail():raise OSError('capture close')
        self.observer.on_close=fail;r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertTrue(r['finalIdleOffVerified'])
    def test_scope_storage_failure_never_writes_to_controller(self):
        original=self.store.write_json
        def write(name,value):
            if name=='scope.json':raise OSError('disk full')
            return original(name,value)
        with patch.object(self.store,'write_json',side_effect=write):r=self.run_trial()
        self.assertEqual(r['status'],'failed');self.assertFalse(self.device.requests);self.assertEqual(self.row(),'off_unverified')

    def unconfirmed_contact(self, record):
        record['onsite']['jawContactVerified']=False
        raw={k:v for k,v in record.items() if k not in ('evidenceFile','evidenceSha256')}
        record['evidenceSha256']=self.observer.save(record['evidenceFile'],raw)
        self.observer.latest_record=copy.deepcopy(record)

    def contact_probe(self):
        self.scope.update(schemaVersion=m.CONTACT_PROBE_SCHEMA,contactState='unknown',expectedStartAngleDeg=199.95)
        self.device.angle=199.95
        self.observer.on_before=self.unconfirmed_contact
        self.trial=m.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.evidence,clock=self.clock)
        self.trial.port.opener=self.device

    def test_contact_unknown_probe_has_separate_outcome_and_one_fixed_pulse(self):
        self.contact_probe();self.device.after_run=lambda d:setattr(d,'angle',200.2)
        r=self.run_trial()
        self.assertEqual(r['status'],'probe_completed_contact_unverified',r)
        self.assertEqual(r['contactStateBefore'],'unknown');self.assertEqual(r['trialKind'],'contact_uncertain_probe')
        self.assertFalse(r['graspVerified']);self.assertFalse(r['liftVerified']);self.assertFalse(r['trainingEligible'])
        self.assertTrue(r['finalIdleOffVerified']);self.assertEqual(r['receipt']['requestedPulse'],m.PULSE)
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)

    def test_existing_commissioning_still_rejects_unconfirmed_contact(self):
        self.observer.on_before=self.unconfirmed_contact
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertTrue(r['finalIdleOffVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),0)

    def test_contact_unknown_probe_keeps_one_degree_guard_and_stop_off(self):
        self.contact_probe();self.device.after_run=lambda d:setattr(d,'angle',201.1)
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertTrue(r['finalIdleOffVerified'])
        self.assertFalse(r['graspVerified']);self.assertFalse(r['liftVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)

    def test_contact_response_350_is_explicit_preset_and_legacy_schema_rejects_it(self):
        self.scope['pulse']=copy.deepcopy(m.CONTACT_RESPONSE_PULSE);self.contact_probe()
        r=self.run_trial();self.assertEqual(r['status'],'probe_completed_contact_unverified',r)
        self.assertEqual(r['receipt']['requestedPulse'],m.CONTACT_RESPONSE_PULSE)
        self.assertTrue(r['finalIdleOffVerified']);self.assertFalse(r['modelSelected'])
        self.assertFalse(r['graspVerified']);self.assertFalse(r['liftVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)
        legacy=scope();legacy['pulse']=copy.deepcopy(m.CONTACT_RESPONSE_PULSE)
        with self.assertRaises(ValueError):m.validate_scope(legacy)
        for duration in (349,351,500):
            bad=copy.deepcopy(self.scope);bad['pulse']['durationMs']=duration
            with self.subTest(duration=duration),self.assertRaises(ValueError):m.validate_scope(bad)

    def test_contact_response_350_keeps_one_degree_abort_and_verified_off(self):
        self.scope['pulse']=copy.deepcopy(m.CONTACT_RESPONSE_PULSE);self.contact_probe()
        self.device.after_run=lambda d:setattr(d,'angle',201.1)
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertTrue(r['finalIdleOffVerified'])
        self.assertFalse(r['graspVerified']);self.assertFalse(r['liftVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)
        self.assertIn(('/command',{'cmd':'stop'}),self.posts())

    def lift_trial(self):
        self.scope.update(schemaVersion=m.LIFT_SCHEMA,pulse=copy.deepcopy(m.LIFT_PULSE))
        self.trial=m.M4UpCommissioning(self.scope,self.observer,'private-token',self.store,evidence_root=self.evidence,clock=self.clock)
        self.trial.port.opener=self.device

    def test_manual_lift_sends_exactly_one_fixed_750ms_pulse(self):
        self.lift_trial();r=self.run_trial()
        self.assertEqual(r['status'],'completed',r);self.assertEqual(r['schemaVersion'],m.LIFT_SCHEMA)
        self.assertEqual(r['receipt']['requestedPulse'],m.LIFT_PULSE)
        self.assertEqual(r['selectionKind'],'manual_lift_fixed_profile')
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1)
        self.assertIn(('/sequence',dict(action='add',joint='shoulder',direction='up',speed='50',duration='750')),self.posts())
        self.assertTrue(r['finalIdleOffVerified']);self.assertFalse(r['modelSelected'])

    def test_manual_lift_keeps_one_degree_abort_and_no_second_run(self):
        self.lift_trial();self.device.after_run=lambda d:setattr(d,'angle',221.1)
        r=self.run_trial();self.assertEqual(r['status'],'failed');self.assertTrue(r['finalIdleOffVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.posts()),1);self.assertEqual(self.row(),'pending')


class ScopeTests(unittest.TestCase):
    def test_contact_probe_450_preset_is_explicit_and_legacy_stays_restricted(self):
        s=scope();s.update(schemaVersion=m.CONTACT_PROBE_SCHEMA,contactState='unknown',expectedStartAngleDeg=200.03)
        s['pulse']=dict(m.CONTACT_RESPONSE_PULSE,durationMs=450)
        self.assertEqual(m.validate_scope(s)['pulse']['durationMs'],450)
        for duration in (449,451,500):
            bad=copy.deepcopy(s);bad['pulse']['durationMs']=duration
            with self.subTest(duration=duration),self.assertRaises(ValueError):m.validate_scope(bad)
        old=scope();old['pulse']=copy.deepcopy(s['pulse'])
        with self.assertRaises(ValueError):m.validate_scope(old)

    def test_contact_probe_requires_exact_unknown_scope_and_unchanged_limits(self):
        s=scope();s.update(schemaVersion=m.CONTACT_PROBE_SCHEMA,contactState='unknown',expectedStartAngleDeg=199.95)
        self.assertEqual(m.validate_scope(s)['pulse'],m.PULSE)
        for key,value in [('contactState','verified'),('maxObservedDeltaDeg',3.),('maxCommands',2),('extra',True)]:
            bad=copy.deepcopy(s);bad[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):m.validate_scope(bad)
        empty=copy.deepcopy(s);empty['pulse']={}
        with self.assertRaises(ValueError):m.validate_scope(empty)
        missing=copy.deepcopy(s);del missing['contactState']
        with self.assertRaises(ValueError):m.validate_scope(missing)
        for key,value in [('percent',17),('durationMs',201),('expectedPwm',41)]:
            bad=copy.deepcopy(s);bad['pulse'][key]=value
            with self.subTest(pulse=key),self.assertRaises(ValueError):m.validate_scope(bad)
        s['schemaVersion']=m.SCHEMA
        with self.assertRaises(ValueError):m.validate_scope(s)


    def test_manual_lift_is_separate_exact_profile_and_general_up_stays_rejected(self):
        s=scope();s.update(schemaVersion=m.LIFT_SCHEMA,pulse=copy.deepcopy(m.LIFT_PULSE))
        self.assertEqual(m.validate_scope(s)['pulse']['expectedPwm'],127)
        with self.assertRaisesRegex(ValueError,'m4_up'):validate_pulse(s['pulse'])
        for key,value in [('percent',51),('durationMs',800),('expectedPwm',128)]:
            bad=copy.deepcopy(s);bad['pulse'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):m.validate_scope(bad)
        s['schemaVersion']=m.SCHEMA
        with self.assertRaises(ValueError):m.validate_scope(s)
    def test_relaxing_any_fixed_limit_is_refused(self):
        cases=[('defaultSpeed',100),('maxCommands',2),('maxSeconds',61),('maxObservedDeltaDeg',2),
               ('maxObservationAgeSeconds',11),('postOffSeconds',2),('startAngleToleranceDeg',1),('expectedStartAngleDeg',300.)]
        for key,value in cases:
            s=scope();s[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):m.validate_scope(s)
    def test_pulse_direction_power_duration_and_extra_fields_refused(self):
        for key,value in [('direction','down'),('motor',3),('percent',17),('durationMs',201),('expectedPwm',41),('motor',4.)]:
            s=scope();s['pulse'][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):m.validate_scope(s)
        s=scope();s['allowM4Up']=True
        with self.assertRaises(ValueError):m.validate_scope(s)


class OfflineCliTests(unittest.TestCase):
    def test_validation_and_hash_modes_never_construct_execution_or_network(self):
        from tools import validate_m4_up_commissioning as cli
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp).resolve();private=root/'.local';private.mkdir()
            path=private/'scope.json';path.write_text(json.dumps(scope()))
            with (patch.object(cli,'ROOT',root), patch.object(m,'M4UpCommissioning',side_effect=AssertionError('execution')),
                 patch.object(m,'BoundedActionJournal',side_effect=AssertionError('journal')),
                 patch('urllib.request.build_opener',side_effect=AssertionError('network'))):
                output=io.StringIO()
                with contextlib.redirect_stdout(output):self.assertEqual(cli.main(['--scope',str(path)]),0)
                report=json.loads(output.getvalue());self.assertEqual(report['hardwareCalls'],0)
                self.assertFalse(report['generalM4UpEnabled']);self.assertFalse(report['liveReadinessVerified'])
                output=io.StringIO()
                with contextlib.redirect_stdout(output):self.assertEqual(cli.main(['--implementation-sha256']),0)
                self.assertEqual(output.getvalue().strip(),m.implementation_digest())
            self.assertEqual(sorted(p.name for p in private.iterdir()),['scope.json'])

    def test_public_symlink_malformed_and_extra_run_option_are_rejected(self):
        from tools import validate_m4_up_commissioning as cli
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp).resolve();private=root/'.local';private.mkdir()
            public=root/'public.json';public.write_text(json.dumps(scope()))
            link=private/'link.json';link.symlink_to(public)
            bad=private/'bad.json';bad.write_text('{"token":"private-value","token":null}')
            with patch.object(cli,'ROOT',root):
                for path in [public,link,bad]:
                    output=io.StringIO()
                    with contextlib.redirect_stdout(output):self.assertEqual(cli.main(['--scope',str(path)]),2)
                    self.assertNotIn('private-value',output.getvalue())
                    self.assertEqual(json.loads(output.getvalue())['hardwareCalls'],0)
                with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as error:
                    cli.main(['--scope',str(public),'--run'])
                self.assertEqual(error.exception.code,2)


if __name__=='__main__':unittest.main()
