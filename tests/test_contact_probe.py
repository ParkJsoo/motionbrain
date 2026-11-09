"""접촉 탐색 driver의 재조회·검토 우선순위·수동 관측 실패를 가짜 장치로 확인한다."""
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock,patch
from tools import run_contact_probe as driver
from learning.artifacts import ArtifactStore


class ContactProbeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.private=self.root/'.local';self.base=self.private/'probe';self.base.mkdir(parents=True)
        self.config=self.base/'contact-plan.json';self.image=self.base/'review.png';self.image.write_bytes(b'review-original')
        self.plan=dict(schemaVersion='motionbrain.attended_contact_probe.v1',automaticRetry=False,automaticStageLoop=False,
            authorization=dict(additionalMotionAuthorized=True,latestUserInstruction='단발 진행',userInstruction='접촉 탐색'),
            maximumM1CloseCommands=6,M1CloseCommand=driver.COMMAND,geometryReviewPath='.local/probe/geometry.json',
            controllerBootId='boot',M4BaselineDeg=199.95,cameraUrl='http://camera.invalid',cameraBootId='camera',
            helperPath='.local/helper',controllerUrl='http://controller.invalid',deviceId='device',viewId='view',observerSpecPath='.local/spec.json',
            manualReview=dict(lastCompletedStep=1,decision='continue_single_close',evidencePath='.local/probe/review.png',evidenceSha256=driver.sha(self.image.read_bytes())))
        (self.base/'geometry.json').write_text('{}')
        self.write_plan()
        self.credential=patch.object(driver,'load_keychain_secret',return_value='test-only-private-secret').start()
        self.manager=patch.object(driver,'ManualCollection').start()
        patch.object(driver,'ROOT',self.root).start();patch.object(driver,'PRIVATE',self.private).start();self.addCleanup(patch.stopall)
    def write_plan(self):self.config.write_text(json.dumps(self.plan))
    def previous(self,status='recorded',off=True):
        (self.base/'response-01.json').write_text(json.dumps(dict(status=status,offVerified=off)))
    def call(self,step=2):return driver.run_probe(self.config,step,'synthetic-service','synthetic-account')
    def test_existing_request_returns_before_keychain_or_device_even_when_plan_was_updated(self):
        requests=self.base/'requests';requests.mkdir()
        saved=dict(status='reserved',request=dict(requestId='contact-close-02',command=driver.COMMAND))
        (requests/'contact-close-02.json').write_text(json.dumps(saved))
        self.assertEqual(self.call(),saved)
        self.credential.assert_not_called();self.manager.assert_not_called()
        self.assertFalse((self.base/'plan-02.json').exists())
    def test_failed_or_unverified_previous_prevents_credential_access(self):
        for status,off in (('failed',True),('recorded',False)):
            self.previous(status,off)
            with self.assertRaisesRegex(ValueError,'previous_step_incomplete'):self.call()
        self.credential.assert_not_called();self.manager.assert_not_called()
    def test_stale_or_tampered_manual_review_prevents_credential_access(self):
        self.previous();self.plan['manualReview']['lastCompletedStep']=0;self.write_plan()
        with self.assertRaisesRegex(ValueError,'current_manual_review'):self.call()
        self.plan['manualReview']['lastCompletedStep']=1;self.write_plan();self.image.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'manual_review_image_changed'):self.call()
        self.credential.assert_not_called();self.manager.assert_not_called()
    def test_exact_plan_snapshot_is_saved_before_credential_and_mismatch_is_rejected(self):
        self.previous()
        (self.private/'registration.json').write_text('{}');(self.private/'reference.jpg').write_bytes(b'reference')
        spec=dict(registrationPath='registration.json',registrationSha256=driver.sha(b'{}'),referenceImagePath='reference.jpg',referenceImageSha256=driver.sha(b'reference'),context={})
        (self.private/'spec.json').write_text(json.dumps(spec))
        original=self.config.read_bytes();snapshot=self.base/'plan-02.json'
        def stop_before_credential(*args):
            self.assertEqual(snapshot.read_bytes(),original)
            raise RuntimeError('synthetic_no_credential_or_device')
        self.credential.side_effect=stop_before_credential
        with patch.object(driver,'RegisteredGripperCrosssection'):
            for attempt in range(2):
                with self.assertRaisesRegex(RuntimeError,'synthetic_no_credential'):self.call()
        self.assertEqual(self.credential.call_count,2);self.manager.assert_not_called()
        self.config.write_bytes(original+b'\n')
        with patch.object(driver,'RegisteredGripperCrosssection'):
            with self.assertRaisesRegex(ValueError,'plan_snapshot_changed'):self.call()
        self.assertEqual(snapshot.read_bytes(),original)
        self.assertEqual(self.credential.call_count,2);self.manager.assert_not_called()

    def test_manual_probe_preserves_unobserved_shaft_and_dispatches_only_one_fixed_close(self):
        self.previous();(self.private/'registration.json').write_text('{}');(self.private/'reference.jpg').write_bytes(b'reference')
        spec=dict(registrationPath='registration.json',registrationSha256=driver.sha(b'{}'),referenceImagePath='reference.jpg',referenceImageSha256=driver.sha(b'reference'),context={})
        (self.private/'spec.json').write_text(json.dumps(spec))
        store=ArtifactStore(self.base/'synthetic-session');(store.root/'before.jpg').write_bytes(b'original')
        pair=store.root/'pair';pair.mkdir();(pair/'esp32-before.jpg').write_bytes(b'original')
        manager=SimpleNamespace(store=store,port=None,token='test-only-private-secret',close=Mock())
        self.manager.return_value=manager
        tracker=Mock();tracker.observe_shaft.side_effect=ValueError('crosssection_rows_insufficient')
        result=dict(status='recorded',offVerified=True)
        def collect(actual,request,command,context,**kwargs):
            self.assertIs(actual,manager);self.assertEqual(command,driver.COMMAND)
            kwargs['before_command'](manager,dict(path=str(pair),before=dict(frame=dict(frameId='frame',pixelSha256='pixel'))))
            return result
        with patch.object(driver,'RegisteredGripperCrosssection',return_value=tracker),patch.object(driver,'collect_once',side_effect=collect) as once:
            self.assertEqual(self.call(),result)
        self.assertEqual(once.call_count,1);self.credential.assert_called_once()
        evidence=json.loads((store.root/'prospective.json').read_bytes())
        self.assertIsNone(evidence['observation']);self.assertEqual(evidence['unsupportedReason'],'crosssection_rows_insufficient')
        self.assertFalse(evidence['geometryDepthVerified']);self.assertFalse(evidence['graspVerified'])
        self.assertEqual(json.loads((self.base/'response-02.json').read_bytes()),result)
        self.assertEqual(manager.token,'');manager.close.assert_called_once()
        for p in self.base.rglob('*.json'):self.assertNotIn('test-only-private-secret',p.read_text())

if __name__=='__main__':unittest.main()
