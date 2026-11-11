"""실제 transport의 실패·회복 원본을 학습 원본 감사까지 연결한다."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from learning.artifacts import ArtifactStore
from learning.manual_collection import RecordingPort
from learning import manual_collection_dataset as data
from learning.pipeline_evidence import ingest_session
from tests.test_supervised_transport import Clock, Controller, pulse


class AdvancingClock(Clock):
    def monotonic(self):
        self.now += .000001
        return self.now


class QuietReadEvidenceTests(unittest.TestCase):
    def fixture(self, when='after'):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        root=Path(tmp.name)/'source';store=ArtifactStore(root);clock=AdvancingClock();device=Controller(clock);device.boot='a'*32
        events=[]
        port=RecordingPort('http://127.0.0.1:8181','unused',device.boot,store,clock=clock,observer=events.append)
        port.opener=device;port.preflight();clock.sleep(.1)
        def recover():
            original=device.open;failed=False
            def transient(request,timeout):
                nonlocal failed
                if '/action_outputs' in request.full_url and not failed:
                    failed=True;clock.sleep(1.5);raise TimeoutError()
                return original(request,timeout)
            device.open=transient
            port.check_idle(allow_quiet_disconnect_retry=True,allow_quiet_monitor_timeout=True,
                            allow_quiet_read_recovery=True)
            device.open=original
        if when=='before': recover()
        clock.sleep(3.5)
        command=dict(kind='command',at=clock.monotonic(),commandId='s-001',pulse=pulse('shoulder','down',25,350))
        events.append(command);store.write_json('command-001-intent.json',command)
        receipt=port.execute(command['pulse'],lambda:None)
        result=dict(kind='result',at=clock.monotonic(),commandId='s-001',receipt=receipt)
        events.append(result);store.write_json('command-001-result.json',result)
        clock.sleep(.1)
        if when=='after': recover()
        clock.sleep(6.5);port.finish()
        session=dict(schemaVersion='motionbrain.manual_collection.v1',sessionId='s',joint='shoulder',
            datasetEligible=False,trainingPerformed=False,phase='completed',error=None,finalIdleOffVerified=True,
            outputBaselineSequence=5,events=events,endedAt=clock.monotonic(),frames=[],recording={})
        store.write_json('plan.json',dict(sessionId='s',joint='shoulder'));store.write_json('session.json',session)
        return root,session,command,receipt

    def test_bounded_before_and_after_recovery_retains_failure_and_audits(self):
        for when in ('before','after'):
            with self.subTest(when=when):
                root,s,c,r=self.fixture(when)
                data.audit_http(data.Evidence(root),s,c,r)
                self.assertEqual(sum(e.get('kind')=='http_error' for e in s['events']),1)
                self.assertEqual(sum(e.get('payload')=={'action':'run'} for e in s['events']),1)
                self.assertEqual(len(list(root.glob('http-*-error.json'))),1)

    def test_recovery_is_usable_by_pipeline_without_hiding_error(self):
        root,s,c,r=self.fixture()
        frames=[]
        for lo,hi in ((c['at']-3,c['at']-.1),(r['offAt']+3,r['offAt']+6)):
            for i in range(round((hi-lo)/.1)+1):
                t=lo+i*.1;n=len(frames)
                frames.append(dict(sourceEarliest=t,sourceLatest=t,stdoutReadAt=t+.001,persistenceMs=1,
                    qualityFlags=[],frameIndex=n,frameId='s:'+str(n),imageFile='i',imageSha256='b'*64,
                    metadataFile='m',metadataSha256='c'*64,pixelSha256='d'*64,width=32,height=32,
                    rawByteLength=4096,storageEncoding='test'))
        original=(root/'session.json').read_bytes()
        with patch.object(data,'audit_camera',return_value=(frames,{'viewId':'test'})):
            report=ingest_session(root,data.digest(original))
        self.assertEqual(report['execution']['status'],'verified')
        self.assertEqual(report['visual']['status'],'verified')
        self.assertEqual((root/'session.json').read_bytes(),original)

    def test_missing_marker_extra_error_and_changed_failure_rejected(self):
        for variant in ('missing','extra','post','late','during','sequence','nonzero'):
            with self.subTest(variant=variant):
                root,s,c,r=self.fixture()
                marker=next(root.glob('quiet-monitor-*-recovery-intent.json'))
                recovery=json.loads(marker.read_text());serial=recovery['failedRequestSerial']
                failure=next(e for e in s['events'] if e.get('kind')=='http_error')
                retry=next(e for e in s['events'] if e.get('kind')=='http_retry')
                if variant=='missing': marker.unlink()
                elif variant=='extra': (root/'http-999999-error.json').write_text('{}')
                elif variant=='post':
                    p=root/f'http-{serial:06d}-intent.json';v=json.loads(p.read_text());v.update(method='POST',payload={'action':'run'});p.write_text(json.dumps(v))
                elif variant=='late':
                    recovery['deadline']=recovery['startedAt']+.01;retry['detail']=recovery;marker.write_text(json.dumps(recovery))
                elif variant=='during':
                    next(e for e in s['events'] if e.get('kind')=='result')['at']=recovery['deadline']+1
                elif variant=='sequence':
                    recovery['lastVerifiedSequence']+=1;retry['detail']=recovery;marker.write_text(json.dumps(recovery))
                else:
                    event=next(e for e in s['events'] if e.get('kind')=='http' and e.get('path','').startswith('/action_outputs') and e['requestStartedAt']>=recovery['startedAt'])
                    event['response']['entries']=[dict(sequence=999,motor=5,pwm=1,atMs=999)]
                with self.assertRaises(ValueError): data.audit_http(data.Evidence(root),s,c,r)

if __name__=='__main__':unittest.main()
