"""순수 원본 수입: 단계 실패 분리와 재가공·무구동·중복 키 경계를 검증한다."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning import pipeline_evidence as m
from learning.artifacts import ArtifactStore
from learning.supervised_transport import SequencePort
from tests.test_supervised_transport import Clock, Controller, pulse


class PipelineEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'source'; self.store = ArtifactStore(self.root)
        self.clock = Clock(); self.controller = Controller(self.clock); self.controller.boot = 'a' * 32
        self.port = SequencePort('http://127.0.0.1:8181', 'unused', self.controller.boot,
                                 self.store, clock=self.clock)
        self.port.opener = self.controller; self.events = []
        original = self.port._call
        def observed(path, payload=None, **kwargs):
            start = self.clock.monotonic(); response = original(path, payload, **kwargs)
            self.events.append(dict(kind='http', path=path, payload=payload, response=response,
                requestStartedAt=start, responseReceivedAt=self.clock.monotonic()))
            return response
        self.port._call = observed

    def save(self, session):
        (self.root/'session.json').write_text(json.dumps(session))
        return hashlib.sha256((self.root/'session.json').read_bytes()).hexdigest()

    def fixture(self, *, failed=False, no_run=False):
        self.port.preflight()
        for _ in range(4): self.clock.sleep(.75); self.port._status()
        at = self.clock.monotonic()
        command = dict(kind='command', at=at, commandId='s-001', pulse=pulse('shoulder','down',25,350))
        if not no_run:
            self.events.append(command); self.store.write_json('command-001-intent.json', command)
            receipt = self.port.execute(command['pulse'], lambda: None)
            result = dict(kind='result', at=self.clock.monotonic(), commandId='s-001', receipt=receipt)
            self.events.append(result); self.store.write_json('command-001-result.json', result)
            for _ in range(7): self.clock.sleep(.8); self.port._status()
        else: receipt = dict(offAt=at)
        self.clock.sleep(1.); self.port.finish()
        session = dict(schemaVersion='motionbrain.manual_collection.v1',sessionId='s',joint='shoulder',
            datasetEligible=False,trainingPerformed=False,phase='failed' if failed else 'completed',
            error='operator_stopped' if failed else None,finalIdleOffVerified=True,
            outputBaselineSequence=5,events=self.events,endedAt=self.clock.monotonic(),frames=[],recording={})
        self.store.write_json('plan.json',dict(sessionId='s',joint='shoulder'))
        frames = []
        for lo,hi in [(at-3,at-.1),(receipt['offAt']+3,receipt['offAt']+6)]:
            for i in range(round((hi-lo)/.1)+1):
                t = lo+i*.1; n=len(frames)
                frames.append(dict(sourceEarliest=t,sourceLatest=t,stdoutReadAt=t+.001,persistenceMs=1,
                    qualityFlags=[],frameIndex=n,frameId='s:'+str(n),imageFile='i',imageSha256='b'*64,
                    metadataFile='m',metadataSha256='c'*64,pixelSha256='d'*64,width=32,height=32,
                    rawByteLength=4096,storageEncoding='test'))
        session['endedAt']=max(session['endedAt'],receipt['offAt']+6.1)
        return session,frames,self.save(session)

    def ingest(self, digest, frames, **kwargs):
        with patch.object(m.data,'audit_camera',return_value=(frames,{'viewId':'test'})):
            return m.ingest_session(self.root,digest,**kwargs)

    def test_complete_record_keeps_sensor_separate_and_verified(self):
        s,frames,h=self.fixture(); result=self.ingest(h,frames)
        self.assertEqual(result['execution']['status'],'verified')
        self.assertEqual(result['sensor']['status'],'verified')
        self.assertGreaterEqual(len(result['sensor']['before']['samples']),3)
        self.assertGreaterEqual(len(result['sensor']['after']['samples']),3)

    def test_explicit_failed_reprocessing_preserves_source_and_outcome(self):
        s,frames,h=self.fixture(failed=True); original=(self.root/'session.json').read_bytes()
        default=self.ingest(h,frames)
        self.assertEqual(default['execution']['status'],'verified')
        self.assertEqual(default['visual']['status'],'deferred')
        result=self.ingest(h,frames,allow_failed_development=True)
        self.assertEqual(result['visual']['status'],'verified')
        self.assertEqual(result['originalOutcome'],dict(phase='failed',error='operator_stopped'))
        self.assertTrue(result['visual']['row']['recoveryDevelopmentOnly'])
        self.assertFalse(result['trainingEligible']); self.assertFalse(result['controlEligible'])
        self.assertEqual(original,(self.root/'session.json').read_bytes())
        self.assertNotEqual(default['stageKey'],result['stageKey'])

    def test_deterministic_stage_key_and_bound_hash(self):
        s,frames,h=self.fixture(); a=self.ingest(h,frames); b=self.ingest(h,frames)
        self.assertEqual(a,b)
        self.assertEqual(a['visual']['row']['sourceHash'],h)
        with self.assertRaisesRegex(ValueError,'hash_mismatch'):self.ingest('0'*64,frames)
        (self.root/'session.json').write_text(json.dumps(dict(s,error='modified')))
        with self.assertRaisesRegex(ValueError,'hash_mismatch'):self.ingest(h,frames)

    def test_sensor_failure_does_not_discard_visual_and_execution(self):
        s,frames,h=self.fixture()
        with patch.object(m.data,'sensor_window',side_effect=ValueError('sensor_window_missing')):
            result=self.ingest(h,frames)
        self.assertEqual(result['execution']['status'],'verified')
        self.assertEqual(result['visual']['status'],'verified')
        self.assertEqual(result['sensor']['status'],'rejected')
        self.assertIn('before:sensor_window_missing',result['sensor']['reason'])
        self.assertIn('after:sensor_window_missing',result['sensor']['reason'])

    def test_recording_failure_does_not_hide_execution(self):
        s,frames,h=self.fixture()
        with patch.object(m.data,'audit_camera',side_effect=ValueError('image_hash_changed')):
            result=m.ingest_session(self.root,h)
        self.assertEqual(result['execution']['status'],'verified')
        self.assertEqual(result['recording']['status'],'rejected')
        self.assertEqual(result['visual']['reason'],'recording_not_verified')

    def test_missing_result_is_unknown_and_never_no_run(self):
        s,frames,h=self.fixture(); s['events']=[e for e in s['events'] if e['kind']!='result']
        (self.root/'command-001-result.json').unlink(); h=self.save(s)
        result=self.ingest(h,frames)
        self.assertEqual(result['execution']['status'],'unknown')
        self.assertFalse(result['execution']['offVerified'])
        self.assertIsNone(result['visual']['row'])

    def test_zero_command_requires_full_off_and_http_proof(self):
        s,frames,h=self.fixture(failed=True,no_run=True); result=self.ingest(h,frames)
        self.assertEqual(result['execution']['status'],'no_run')
        self.assertTrue(result['execution']['offVerified'])
        self.assertIsNone(result['visual']['row'])
        (self.root/'http-000001-response.json').unlink()
        result=self.ingest(h,frames)
        self.assertEqual(result['execution']['status'],'unknown')

    def test_no_run_cannot_be_claimed_by_removing_command_events(self):
        s,frames,h=self.fixture();s['events']=[e for e in s['events'] if e['kind'] not in ('command','result')]
        (self.root/'command-001-intent.json').unlink();(self.root/'command-001-result.json').unlink()
        result=self.ingest(self.save(s),frames)
        self.assertEqual(result['execution']['status'],'unknown')
        self.assertEqual(result['execution']['reason'],'no_run_unexpected_write')

    def test_no_run_baseline_cannot_hide_prior_outputs(self):
        s,frames,h=self.fixture(failed=True,no_run=True);s['outputBaselineSequence']+=100
        result=self.ingest(self.save(s),frames)
        self.assertEqual(result['execution']['status'],'unknown')
        self.assertEqual(result['execution']['reason'],'no_run_output_baseline_changed')

    def test_video_windows_stay_fixed_and_rejected_separately(self):
        s,frames,h=self.fixture(); result=self.ingest(h,frames[:3])
        self.assertEqual(result['execution']['status'],'verified')
        self.assertEqual(result['visual']['status'],'rejected')
        self.assertIn('video_window',result['visual']['reason'])

    def test_reprocessing_flag_is_not_truthy_string(self):
        s,frames,h=self.fixture()
        with self.assertRaisesRegex(ValueError,'boolean'):self.ingest(h,frames,allow_failed_development='yes')


if __name__=='__main__': unittest.main()
