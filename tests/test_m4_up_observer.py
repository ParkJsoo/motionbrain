"""Real adapter integration with synthetic persisted sources; never use devices."""
import copy
import json
from pathlib import Path
import shutil
import tempfile
import threading
import unittest
from unittest.mock import patch
import warnings

from learning import m4_up_commissioning as core
from learning import m4_up_observer as camera
from learning.artifacts import ArtifactStore, _read, _decode_json
from learning.gripper_camera_dataset import read_frame
from learning.supervised_collection import plan_digest
from tests.test_m4_up_commissioning import Clock, Device, Observer as Producer, scope
from tools import m4_up_trial as cli


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.repo=Path(self.tmp.name).resolve();self.clock=Clock();self.scope=scope()
        self.store=ArtifactStore(self.repo/'.local/run');self.device=Device(self.clock)
        helper=self.repo/'helper';helper.write_bytes(b'synthetic helper')
        self.config=dict(helperPath=str(helper),helperSha256=camera.sha(helper.read_bytes()),cameraUrl='http://camera.invalid')
        self.producer=Producer(self.repo/'producer',self.clock,self.scope)
        owner=self
        class Native:
            def __init__(self,root,helper,plan,**kwargs):
                self.root=root;self.lock=threading.RLock();self.frames=[]
                self.started=None;self.done=False;self.fault=None;self.closed=False;owner.native=self
            def emit(self):
                owner.producer.observe(after_off=None,deadline=100,tick=lambda:None)
                src=owner.producer.root/'external'
                for path in src.rglob('*'):
                    if path.is_file():
                        dest=self.root/path.relative_to(src);dest.parent.mkdir(parents=True,exist_ok=True)
                        shutil.copyfile(path,dest)
                i=owner.producer.index-1
                self.frames.append(_decode_json(_read(self.root,f'host-ingress/{i:04d}.json')))
            def start(self):
                self.started=owner.clock.monotonic();owner.producer.start();self.emit()
            def check(self):
                if self.fault:raise ValueError('native_fault')
            def close(self):
                self.closed=True
                return dict(status='partial',reason=self.fault,clockBridge={'bounded':True},terminalStatus='partial')
        class Gripper:
            def __init__(self,url,session,boot,scene,store,**kwargs):
                self.store=store;self._bindings={};self.count=0;self.closed=False;owner.gripper=self
            def start(self,**kwargs):pass
            def capture(self,**kwargs):
                self.count+=1;owner.native.emit()
                folder=f'capture-{self.count:04d}'
                meta=_read(owner.producer.root,'gripper/esp32-before.json')
                image=_read(owner.producer.root,'gripper/esp32-before.jpg')
                self.store._write(folder+'/esp32-frame.json',meta);self.store._write(folder+'/esp32-frame.jpg',image)
                frame,receipt,boot=read_frame(self.store.root/folder,'frame',owner.scope['hostClockId'])
                self._bindings[frame['frameId']]={'folder':folder}
                return dict(frame=frame,sourceEarliest=receipt['requestStartedAt'],persistedAt=receipt['persistedAt'])
            def verify(self,record):pass
            def check(self):pass
            def close(self):self.closed=True;return dict(status='completed')
        for target,value in [(camera,'ActionRecorder'),(camera,'GripperObservationSource')]:
            replacement=Native if value=='ActionRecorder' else Gripper
            p=patch.object(target,value,replacement);p.start();self.addCleanup(p.stop)
        for target in (camera,core):
            p=patch.object(target,'decode',side_effect=self.producer.decode);p.start();self.addCleanup(p.stop)
        p=patch.object(core,'REPO',self.repo);p.start();self.addCleanup(p.stop)
    def review(self,request,**kwargs):
        return dict(context=copy.deepcopy(self.scope['context']),stable=True,
                    onsite=dict(operatorPresent=True,clearanceVerified=True,jawContactVerified=True,
                                tableSupportVerified=request['label']=='before'))
    def adapter(self):return camera.M4DualCameraObserver(self.scope,self.config,self.store,self.review,clock=self.clock)
    def test_adapter_enters_existing_core_and_records_two_sources_around_one_command(self):
        observer=self.adapter();trial=core.M4UpCommissioning(self.scope,observer,'private-token',self.store,evidence_root=self.store.root,clock=self.clock)
        trial.port.opener=self.device
        result=trial.run(confirmed_scope_sha256=plan_digest(self.scope))
        self.assertEqual(result['status'],'completed',result);self.assertTrue(result['finalIdleOffVerified'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.device.requests),1)
        self.assertEqual(self.gripper.count,2);self.assertTrue(self.native.closed and self.gripper.closed)
        for label in ('before','after'):
            self.assertTrue((self.store.root/f'{label}-external.png').exists())
            self.assertTrue((self.store.root/f'{label}-gripper.jpg').exists())
        self.assertEqual(observer.close(),result['observerClose'])
    def test_capture_check_has_two_fresh_captures_and_only_controller_gets(self):
        real=cli.ReadOnlyPort
        def factory(*args,**kwargs):
            port=real(*args,**kwargs);port.opener=self.device;return port
        with patch.object(cli,'ReadOnlyPort',side_effect=factory):
            result=cli.capture_check(self.scope,self.config,self.store,clock=self.clock)
        self.assertEqual(result['status'],'completed',result);self.assertTrue(result['finalIdleOffVerified'])
        self.assertTrue(all(v is None for _,v in self.device.requests));self.assertEqual(self.gripper.count,2)
        self.assertFalse(result['liveMotionAuthorized'])
    def test_check_and_latest_do_not_capture_or_modify_pinned_review(self):
        observer=self.adapter();self.assertIsNone(self.native.started)
        observer.start(deadline=60)
        before=observer.observe(after_off=None,deadline=60,tick=lambda:None)
        count=self.gripper.count;requests=len(self.device.requests)
        observer.check();latest=observer.latest();latest['stable']=False
        self.assertTrue(observer.latest()['stable']);self.assertEqual(self.gripper.count,count)
        self.assertEqual(len(self.device.requests),requests);observer.close()
    def test_failed_label_is_not_recaptured_and_both_sources_close(self):
        observer=self.adapter();observer.start(deadline=60)
        def reject(*args,**kwargs):raise ValueError('review rejected')
        observer.reviewer=reject
        with self.assertRaises(ValueError):observer.observe(after_off=None,deadline=60,tick=lambda:None)
        with self.assertRaises(ValueError):observer.observe(after_off=None,deadline=60,tick=lambda:None)
        self.assertEqual(self.gripper.count,1);self.assertEqual(observer.close()['status'],'failed')
        self.assertTrue(self.native.closed and self.gripper.closed)
    def test_native_close_exception_still_closes_gripper(self):
        observer=self.adapter();observer.start(deadline=60)
        with patch.object(self.native,'close',side_effect=OSError('native close')):result=observer.close()
        self.assertEqual(result['status'],'failed');self.assertTrue(self.gripper.closed)
    def test_helper_changed_after_construction_blocks_start(self):
        observer=self.adapter();Path(self.config['helperPath']).write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'helper_changed'):observer.start(deadline=60)
        self.assertIsNone(self.native.started)
    def test_readonly_port_refuses_stop_arm_default_and_run_before_io(self):
        port=cli.ReadOnlyPort(self.scope['controllerUrl'],'unused',self.scope['controllerBootId'],self.store,clock=self.clock)
        port.opener=self.device
        for path,payload in [('/command',{'cmd':'stop'}),('/command',{'cmd':'arm'}),('/motor',{'speed':255}),('/sequence',{'action':'run'})]:
            with self.assertRaisesRegex(ValueError,'write_forbidden'):port._call(path,payload)
        self.assertFalse(self.device.requests)
    def test_getpass_fallback_warning_prevents_journal_or_camera_creation(self):
        config=self.repo/'.local/config.json';config.write_text(json.dumps(dict(scope=self.scope,camera=self.config)))
        def fallback(*args,**kwargs):warnings.warn('echo enabled',cli.getpass.GetPassWarning);return 'must-not-read'
        with (patch.object(cli,'ROOT',self.repo),patch.object(cli.sys.stdin,'isatty',return_value=True),
              patch.object(cli.getpass,'getpass',side_effect=fallback),patch.object(cli,'ArtifactStore',side_effect=AssertionError('store'))):
            with self.assertRaises(cli.getpass.GetPassWarning):
                cli.main(['run','--config',str(config),'--output-dir',str(self.repo/'.local/new'),
                          '--confirm-scope-sha256',plan_digest(self.scope)])
        self.assertFalse(self.device.requests)

    def test_file_review_requires_exact_request_hash_and_rejects_expiry(self):
        reviewer=cli.FileReviewer(self.store,clock=self.clock)
        request=dict(label='before',sourceEarliest=self.clock.now,previews={},sources={})
        count=0
        def wrong_decision():
            nonlocal count
            count+=1
            if count==1:self.store.write_json('before-review-decision.json',dict(requestSha256='0'*64,decision={}))
        with patch('builtins.print'),self.assertRaisesRegex(ValueError,'binding_changed'):
            reviewer(request,deadline=self.clock.now+2,tick=wrong_decision)
        # A fresh request location with no decision times out without another capture.
        other=ArtifactStore(self.repo/'.local/review-expired');reviewer=cli.FileReviewer(other,clock=self.clock)
        with patch('builtins.print'),self.assertRaisesRegex(ValueError,'expired_no_run'):
            reviewer(request,deadline=self.clock.now+.2,tick=lambda:None)

    def test_file_review_accepts_bound_explicit_decision(self):
        reviewer=cli.FileReviewer(self.store,clock=self.clock)
        request=dict(label='before',sourceEarliest=self.clock.now,previews={},sources={})
        expected=dict(context=self.scope['context'],stable=True,onsite={'operatorPresent':True})
        def decide():
            raw=_read(self.store.root,'before-review-request.json')
            self.store.write_json('before-review-decision.json',dict(requestSha256=camera.sha(raw),decision=expected))
        with patch('builtins.print'):result=reviewer(request,deadline=self.clock.now+2,tick=decide)
        self.assertEqual(result,expected)

    def test_start_wait_is_after_private_input_and_before_store_or_camera(self):
        config=self.repo/'.local/config.json';config.write_text(json.dumps(dict(scope=self.scope,camera=self.config)))
        events=[]
        def secret(*args,**kwargs):events.append('private_input');return 'private-token'
        def wait(*args,**kwargs):events.append('start_wait');raise EOFError('no start')
        with (patch.object(cli,'ROOT',self.repo),patch.object(cli.sys.stdin,'isatty',return_value=True),
              patch.object(cli.getpass,'getpass',side_effect=secret),patch('builtins.input',side_effect=wait),
              patch.object(cli,'ArtifactStore',side_effect=AssertionError('must not create store'))):
            with self.assertRaises(EOFError):
                cli.main(['run','--config',str(config),'--output-dir',str(self.repo/'.local/new'),
                          '--confirm-scope-sha256',plan_digest(self.scope),'--wait-for-start'])
        self.assertEqual(events,['private_input','start_wait']);self.assertFalse(self.device.requests)

    def attestation(self):
        source=dict(schemaVersion='motionbrain.attended_single_pulse_authorization.v1',pulse=self.scope['pulse'],
                    authorizedAt='2026-11-07T00:00:00+00:00',operatorOnSiteConfirmed=True,
                    clearanceUnchangedConfirmed=True,physicalPoseUnchangedConfirmed=True,userInstruction='one manual pulse')
        raw=json.dumps(source).encode();path=self.repo/'.local/operator-authorization.json';path.write_bytes(raw)
        return dict(scopeSha256=plan_digest(self.scope),trialId=self.scope['trialId'],context=self.scope['context'],
                    onsite=dict(operatorPresent=True,clearanceVerified=True,jawContactVerified=True,tableSupportVerified=True),
                    stable=True,confirmedAt=source['authorizedAt'],sourceAuthorizationFile=path.name,
                    sourceAuthorizationSha256=camera.sha(raw),expectedOutputSequence=self.device.entries[-1]['sequence'])

    def test_attested_manual_before_keeps_raw_guards_and_visual_after(self):
        att=self.attestation();observer=self.adapter();after=[]
        def review_after(request,**kwargs):after.append(request['label']);return self.review(request,**kwargs)
        with patch.object(cli,'ROOT',self.repo):
            reviewer=cli.AttestedBeforeReviewer(self.scope,att,camera.sha(json.dumps(att).encode()),self.store,
                                               lambda:trial.port.last_sequence,review_after)
        observer.reviewer=reviewer
        trial=core.M4UpCommissioning(self.scope,observer,'private-token',self.store,evidence_root=self.store.root,clock=self.clock)
        trial.port.opener=self.device;r=trial.run(confirmed_scope_sha256=plan_digest(self.scope))
        self.assertEqual(r['status'],'completed',r);self.assertEqual(after,['after'])
        evidence=_decode_json(_read(self.store.root,'before-operator-attestation-binding.json'))
        self.assertFalse(evidence['currentFramesReviewed']);self.assertIn('user_onsite',evidence['reviewBasis'])
        self.assertEqual(sum(v=={'action':'run'} for _,v in self.device.requests),1)
        with self.assertRaisesRegex(ValueError,'consumed'):reviewer({'label':'before'},deadline=60,tick=lambda:None)

    def test_attestation_scope_and_changed_output_block_before_decision(self):
        att=self.attestation()
        with patch.object(cli,'ROOT',self.repo):
            bad=copy.deepcopy(att);bad['scopeSha256']='0'*64
            with self.assertRaisesRegex(ValueError,'scope_changed'):
                cli.AttestedBeforeReviewer(self.scope,bad,'hash',self.store,lambda:5,self.review)
            reviewer=cli.AttestedBeforeReviewer(self.scope,att,'hash',self.store,lambda:999,self.review)
        with self.assertRaisesRegex(ValueError,'output_changed'):reviewer({'label':'before'},deadline=60,tick=lambda:None)
        self.assertFalse((self.store.root/'before-operator-attestation-binding.json').exists())


if __name__=='__main__':unittest.main()
