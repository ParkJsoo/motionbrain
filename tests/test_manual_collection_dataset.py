import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning import manual_collection_dataset as m
from learning.artifacts import ArtifactStore, StorageError
from learning.supervised_transport import SequencePort
from tests.test_supervised_transport import Clock, Controller, pulse


class DatasetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def http_fixture(self):
        root = self.root/'session'
        store = ArtifactStore(root)
        clock = Clock(); device = Controller(clock); device.boot = 'a'*32
        port = SequencePort('http://127.0.0.1:8181', 'unused', device.boot, store, clock=clock)
        port.opener = device
        events = []
        original = port._call
        def observed(path, payload=None, **kwargs):
            start = clock.monotonic()
            response = original(path, payload, **kwargs)
            events.append(dict(kind='http', path=path, payload=payload, response=response,
                               requestStartedAt=start, responseReceivedAt=clock.monotonic()))
            return response
        port._call = observed
        port.preflight(); clock.sleep(.01)
        command = dict(at=clock.monotonic(), pulse=pulse('shoulder','down',25,350))
        receipt = port.execute(command['pulse'], lambda: None)
        port.finish()
        return root, dict(events=events, outputBaselineSequence=5), command, receipt

    def test_http_real_transport_artifacts(self):
        root,s,c,r=self.http_fixture()
        _,entries,boot=m.audit_http(m.Evidence(root),s,c,r)
        self.assertEqual(boot,'a'*32)
        self.assertTrue(entries)

    def test_consistent_rejected_post_is_rejected(self):
        root,s,c,r=self.http_fixture()
        i=next(i for i,e in enumerate(s['events'],1) if e['payload'] is not None)
        event=s['events'][i-1];event['response']['success']=False
        p=root/f'http-{i:06d}-response.json';data=json.loads(p.read_text())
        data['parsed']=event['response'];data['rawUtf8']=json.dumps(data['parsed']);p.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError,'command_rejected'):m.audit_http(m.Evidence(root),s,c,r)

    def test_receipt_dispatch_and_off_are_bound(self):
        root,s,c,r=self.http_fixture()
        for key in ('dispatchBefore','offAt'):
            bad=copy.deepcopy(r);bad[key]-=1
            with self.subTest(key=key),self.assertRaisesRegex(ValueError,'receipt_bracket'):
                m.audit_http(m.Evidence(root),s,c,bad)

    def test_output_baseline_and_trace_cannot_be_trimmed(self):
        root,s,c,r=self.http_fixture()
        s['outputBaselineSequence']+=1;r['outputTrace']=r['outputTrace'][1:]
        with self.assertRaisesRegex(ValueError,'baseline_(gap|output_not_zero)'):m.audit_http(m.Evidence(root),s,c,r)

    def test_raw_http_tamper_and_missing_record(self):
        root,s,c,r=self.http_fixture()
        p=root/'http-000001-response.json';d=json.loads(p.read_text());d['rawUtf8']='{}';p.write_text(json.dumps(d))
        with self.assertRaisesRegex(ValueError,'response_changed'):m.audit_http(m.Evidence(root),s,c,r)
        p.unlink()
        with self.assertRaisesRegex(ValueError,'evidence_count'):m.audit_http(m.Evidence(root),s,c,r)

    def frames(self):
        return [dict(sourceEarliest=i/10,sourceLatest=i/10,stdoutReadAt=i/10+.01,
                     persistenceMs=1,qualityFlags=[],frameIndex=i) for i in range(11)]

    def test_pts_alone_does_not_make_input_available(self):
        frames=self.frames();frames[-1]['stdoutReadAt']=1.2
        selected=m.select_frames(frames,[0,1],1.1)
        self.assertEqual(selected[-1]['frameIndex'],9)
        frames[-2]['persistenceMs']=300
        self.assertEqual(m.select_frames(frames,[0,1],1.1)[-1]['frameIndex'],8)

    def test_video_gap_and_missing_edges_rejected(self):
        for frames in (self.frames()[:5],self.frames()[:3]+self.frames()[8:]):
            with self.assertRaisesRegex(ValueError,'video_window'):m.select_frames(frames,[0,1])

    def test_fixed_sensor_brackets_and_spread_preserved(self):
        statuses=[]
        for t,v in ((0,99),(.2,1),(.5,2),(.8,4),(1,99)):
            statuses.append(dict(requestStartedAt=t,responseReceivedAt=t+.01,response=dict(shoulderAngle=dict(
                available=True,sensorConnected=True,sensorFresh=True,sensorReady=True,ageMs=100,angleDeg=v))))
        result=m.sensor_window(statuses,[0,1])
        self.assertEqual([s['valueDeg'] for s in result['samples']],[1,2,4])
        self.assertEqual(result['rangeDeg'],3)
        self.assertEqual(result['medianDeg'],2)

    def index(self,paths):
        path=self.root/'index.json';path.write_text(json.dumps(dict(schemaVersion=m.INDEX,split='development',
            captureGroup='test',sessions=[dict(path=p,sessionSha256='a'*64) for p in paths],
            excluded=[dict(sessionId='failed',reason='capture_failure')])));return path

    def row(self):
        return dict(status='eligible',sessionId='one',inputs=dict(command=dict(joint='shoulder')),
                    evidence=dict(controllerBootId='a'*32,commandId='one-001',cameraIdentity={}))

    def test_duplicate_path_alias_and_episode(self):
        with self.assertRaises((ValueError,StorageError)):m.build_dataset(self.index(['one','./one']))
        with patch.object(m,'build_session',side_effect=[self.row(),self.row()]):
            result=m.build_dataset(self.index(['one','two']))
        self.assertEqual(result['counts'],dict(listed=2,eligible=1,rejected=1))
        self.assertEqual(result['rows'][1]['reason'],'duplicate_episode')

    def test_missing_artifact_preserves_denominator(self):
        result=m.build_dataset(self.index(['missing']))
        self.assertEqual(result['counts'],dict(listed=1,eligible=0,rejected=1))
        self.assertEqual(len(result['excluded']),1)
        self.assertFalse(result['trainingPerformed'])

    def test_camera_adapter_checks_all_persisted_bytes(self):
        from tests.test_action_recorder import RecorderTests, plan
        from learning.action_recorder import ActionRecorder
        helper=RecorderTests();helper.root=self.root/'native-camera';helper.root.mkdir()
        helper.rec=ActionRecorder(helper.root,'unused',plan())
        helper.rec.started=90.;helper.rec.offset_low=-.000001;helper.rec.offset_high=.000001
        helper.rec.store=ArtifactStore(helper.root/'host-ingress')
        receipt=helper.frame();helper.rec._packet(receipt,100.)
        terminal=helper.terminal(droppedFrames=0)
        (helper.root/'helper.stdout').write_text(json.dumps(dict(event='frame',receipt=receipt))+'\n'+json.dumps(terminal)+'\n')
        session=dict(frames=helper.rec.frames,recording=dict(reason=None,recordedFrames=1,
            terminalStatus='completed',clockBridge=dict(hostClockId='test-session')))
        clock=dict(basis='before_after_same_mach_clock',hostClockId='test-session',
                   validPythonBounds=[90.,101.],initialOffsets=[-.000001,.000001])
        with patch.object(m,'clock_evidence',return_value=clock):
            frames,_=m.audit_camera(m.Evidence(self.root),dict(camera=plan(),helperSha256='f'*64),session)
            self.assertEqual(len(frames),1)
            (helper.root/receipt['imageFile']).write_bytes(b'tampered')
            with self.assertRaisesRegex(ValueError,'image_hash'):
                m.audit_camera(m.Evidence(self.root),dict(camera=plan(),helperSha256='f'*64),session)

    def test_png_export_writes_real_immutable_store(self):
        import numpy as np
        import cv2
        raw=b'compressed';image=self.root/'one/native-camera/frame.lzbitmap';image.parent.mkdir(parents=True);image.write_bytes(raw)
        pixels=bytes(range(16));frame=dict(imageFile='frame.lzbitmap',imageSha256=m.digest(raw),rawByteLength=16,
                                          pixelSha256=m.digest(pixels),width=2,height=2)
        row=self.row();row['source']=dict(path='one');row['inputs']['image']=copy.deepcopy(frame)
        row['targets']=dict(image=copy.deepcopy(frame))
        with patch.object(m,'build_dataset',return_value=dict(rows=[row])),patch('learning.lossless_frame.decode',return_value=pixels):
            result=m.export_dataset(self.index(['one']),self.root/'output',export_png=True)
        ref=result['rows'][0]['inputs']['image']['png'];encoded=(self.root/'output'/ref['artifactRef']).read_bytes()
        self.assertEqual(m.digest(encoded),ref['sha256'])
        decoded=cv2.imdecode(np.frombuffer(encoded,dtype=np.uint8),cv2.IMREAD_UNCHANGED)
        self.assertEqual(decoded.tobytes(),pixels)
        with patch.object(m,'build_dataset',return_value=dict(rows=[])),self.assertRaises(StorageError):
            m.export_dataset(self.root/'index.json',self.root/'output')


if __name__=='__main__':unittest.main()
