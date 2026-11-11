import contextlib
import copy
import io
import json
import shutil
import threading
import unittest
import urllib.error
import urllib.request
from types import SimpleNamespace

from learning import m4_up_commissioning as core
from learning.artifacts import ArtifactStore
from learning.file_review_console import FileReviewConsole
from learning.experiment_pipeline import encoded
from tools.file_review_server import ReviewServer
from tools.m4_up_trial import FileReviewer
from tests import test_batch_m4 as fixtures


class FileReviewConsoleTests(unittest.TestCase):
    def setUp(self):
        f=fixtures.SourceAuditTests();f.setUp();self.addCleanup(f.doCleanups);self.f=f
        self.repo=f.root.parent.resolve()
        private=self.repo/'.local';private.mkdir()
        self.root=private/'review';shutil.copytree(f.root,self.root)
        record=json.loads((self.root/'before-observation.json').read_bytes())
        frame=core._external_frame(self.root,record['external'],f.planned)
        grip=core._gripper_frame(self.root,record['gripper'],f.planned,'before')
        earliest=min(frame['sourceEarliest'],grip['sourceEarliest'])
        self.clock=SimpleNamespace(now=max(frame['persistedAt'],grip['persistedAt'])+.2)
        self.clock.monotonic=lambda:self.clock.now
        self.clock.sleep=lambda seconds:setattr(self.clock,'now',self.clock.now+seconds)
        self.request=dict(label='before',hostClockId=f.planned['hostClockId'],
            sources={k:record[k] for k in ('external','gripper')},
            previews=dict(external='before-external.png',gripper='before-gripper.jpg'),
            sourceEarliest=earliest,nonce='a'*32)
        (self.root/'before-gripper.jpg').write_bytes((self.root/'gripper/esp32-before.jpg').read_bytes())
        (self.root/'before-review-request.json').write_bytes(encoded(self.request))
        self.console=FileReviewConsole(self.repo,self.root/'scope.json',self.root,clock=self.clock)

    def payload(self):
        status=self.console.status()
        for name in ('external','gripper'):self.console.preview(name,status['requestSha256'])
        return dict(requestSha256=status['requestSha256'],framesReviewed=True,stable=False,
            onsite={k:False for k in ('operatorPresent','clearanceVerified','jawContactVerified','tableSupportVerified')},
            outcome='unknown')

    def test_original_previews_unknown_and_single_use(self):
        status=self.console.status()
        self.assertEqual(status['action'],{k:self.f.planned['pulse'][k] for k in ('joint','direction','percent','durationMs')})
        self.assertEqual(status['scene'],{k:self.f.planned['context'][k] for k in ('object','poseRegime')})
        payload=self.payload();result=self.console.submit(payload)
        self.assertEqual(result['outcome'],'unknown')
        response=json.loads((self.root/'before-review-decision.json').read_bytes())
        self.assertFalse(response['decision']['stable'])
        self.assertFalse(any(response['decision']['onsite'].values()))
        with self.assertRaises(FileExistsError):self.console.submit(payload)

    def test_no_pixel_delivery_or_explicit_acknowledgment_cannot_submit(self):
        payload=self.payload();self.console.delivered=set()
        with self.assertRaisesRegex(ValueError,'frames_not_reviewed'):self.console.submit(payload)
        payload=self.payload();payload['framesReviewed']=False
        with self.assertRaisesRegex(ValueError,'frames_not_reviewed'):self.console.submit(payload)

    def test_expiry_is_source_time_not_page_load_and_does_not_retry(self):
        payload=self.payload();self.clock.now=self.request['sourceEarliest']+9.
        self.assertEqual(self.console.status()['status'],'expired')
        with self.assertRaisesRegex(ValueError,'expired'):self.console.submit(payload)
        self.assertFalse((self.root/'before-review-decision.json').exists())

    def test_original_tamper_after_preview_rejected(self):
        payload=self.payload();(self.root/'before-gripper.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'original_changed'):self.console.submit(payload)

    def test_changed_request_requires_new_display_and_acknowledgment(self):
        payload=self.payload();self.request['nonce']='b'*32
        (self.root/'before-review-request.json').write_bytes(encoded(self.request))
        with self.assertRaisesRegex(ValueError,'frames_not_reviewed'):self.console.submit(payload)

    def test_unknown_cannot_hide_affirmative_fields(self):
        payload=self.payload();payload['onsite']['jawContactVerified']=True
        with self.assertRaisesRegex(ValueError,'unknown_not_affirmative'):self.console.submit(payload)

    def test_changed_preview_cannot_replace_original_pixels(self):
        (self.root/'before-external.png').write_bytes((self.root/'after-external.png').read_bytes())
        with self.assertRaisesRegex(ValueError,'preview_pixels_changed'):self.console.status()

    def test_loopback_http_to_actual_file_reviewer_consumption(self):
        server=ReviewServer(('127.0.0.1',0),self.console)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(server.server_close);self.addCleanup(server.shutdown)
        url='http://127.0.0.1:'+str(server.server_port)
        def get(path):
            with urllib.request.urlopen(url+path) as response:return response.read()
        def tick():
            state=json.loads(get('/api/state'))
            for path in state['previews'].values():get(path)
            payload=dict(requestSha256=state['requestSha256'],framesReviewed=True,stable=False,
                onsite={k:False for k in ('operatorPresent','clearanceVerified','jawContactVerified','tableSupportVerified')},outcome='unknown')
            request=urllib.request.Request(url+'/api/decision',data=encoded(payload),headers={
                'Content-Type':'application/json','X-Collection-Token':server.ui_token},method='POST')
            with urllib.request.urlopen(request) as response:self.assertEqual(response.status,200)
        self.assertIn('두 카메라 판독'.encode(),get('/'))
        # A foreign Origin cannot read the local pending review.
        foreign=urllib.request.Request(url+'/api/state',headers={'Origin':'https://foreign.invalid'})
        with self.assertRaises(urllib.error.HTTPError) as denied:urllib.request.urlopen(foreign)
        self.assertEqual(denied.exception.code,403);denied.exception.close()
        unknown=urllib.request.Request(url+'/api/decision',data=b'{}',headers={'Content-Type':'application/json'},method='POST')
        with self.assertRaises(urllib.error.HTTPError) as denied:urllib.request.urlopen(unknown)
        self.assertEqual(denied.exception.code,403);denied.exception.close()
        raw=copy.deepcopy(self.request);raw.pop('nonce')
        (self.root/'before-review-request.json').unlink()
        with contextlib.redirect_stdout(io.StringIO()):
            store=object.__new__(ArtifactStore);store.root=self.root
            decision=FileReviewer(store,clock=self.clock)(raw,deadline=self.clock.now+8,tick=tick)
        self.assertFalse(decision['stable']);self.assertFalse(any(decision['onsite'].values()))
        self.assertEqual(decision['context'],self.f.planned['context'])


if __name__=='__main__':unittest.main()
