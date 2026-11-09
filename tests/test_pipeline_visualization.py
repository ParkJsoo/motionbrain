import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location('pipeline_visualization',
    Path(__file__).resolve().parents[1]/'tools/visualization/pipeline.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


def sha(blob):
    return hashlib.sha256(blob).hexdigest()


class FakeRecording:
    def __init__(self, *args, **kwargs):
        self.logs = []; self.connections = []; self.saves = []; self.disconnected = False
    def save(self, path): self.saves.append(path)
    def set_sinks(self, *sinks): self.connections.append(sinks)
    def set_time(self, *args, **kwargs): pass
    def log(self, path, value): self.logs.append((path, value))
    def flush(self): pass
    def disconnect(self): self.disconnected = True


class PipelineVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name); self.output = self.root/'.local/pipeline'
        self.stages = self.output/'artifacts'; self.stages.mkdir(parents=True)
        self.source = self.root/'source'; (self.source/'native-camera').mkdir(parents=True)
        self.session = b'{"phase":"completed"}'; (self.source/'session.json').write_bytes(self.session)
        self.pixels = bytes([10,20,30,255])*64
        self.frames = {}
        for side in ('before','after'):
            data = ('native-'+side).encode(); (self.source/'native-camera'/side).write_bytes(data)
            self.frames[side] = dict(frameId=side, pixelSha256=sha(self.pixels), imageFile=side,
                imageSha256=sha(data), rawByteLength=len(self.pixels), width=8, height=8)
        self.evidence = dict(schemaVersion='motionbrain.pipeline_evidence.v1',
            source=dict(path=str(self.source), sha256=sha(self.session)),
            visual=dict(status='verified', row=dict(inputs=dict(image=self.frames['before']),
                targets=dict(image=self.frames['after']), evidence=dict(cameraIdentity={'camera':'external'}))),
            execution=dict(status='verified', offVerified=True))
        self.observation = dict(schemaVersion='motionbrain.pipeline_observations.v1',
            sourceHash=sha(self.session), status='partial', recoveryDevelopmentOnly=True, perFrame={})
        for side in ('before','after'):
            self.observation['perFrame'][side] = dict(frameId=side,pixelSha256=sha(self.pixels),
                status='partial',perPoint=dict(object=dict(status='valid',point=[2.,3.]),
                    jawLeft=dict(status='valid',point=[1.,4.]),jawRight=dict(status='unavailable')))
        self.report = dict(schemaVersion='motionbrain.experiment_pipeline.v1', sources=[],
            counts=dict(listed=1,eligible=0,unavailable=1), learning=dict(partitions=[],controlEligible=False),frozenModels=[])
        self.write_pipeline()
        self.decoder = patch.object(m,'decode',return_value=self.pixels);self.decoder.start();self.addCleanup(self.decoder.stop)

    def artifact(self,value):
        raw=json.dumps(value).encode(); ref=dict(path=sha(raw)+'.json',sha256=sha(raw))
        (self.stages/ref['path']).write_bytes(raw);return ref

    def write_pipeline(self):
        self.report['sources'] = [dict(id='source-one',status='partial',trainingEligible=False,
            evidence=self.artifact(self.evidence),observation=self.artifact(self.observation))]
        self.reference=self.artifact(self.report);self.latest=self.output/'latest.json'
        self.latest.write_text(json.dumps(self.reference))

    def test_partial_observation_shows_only_valid_points_on_bound_pixels(self):
        view=m.load_view(self.latest);record=view['records'][0]
        self.assertEqual(record['status'],'partial')
        self.assertEqual(record['frames']['before']['status'],'verified')
        self.assertEqual([p['name'] for p in record['frames']['after']['points']],['object','jawLeft'])
        self.assertEqual(record['frames']['before']['image'][0,0].tolist(),[30,20,10])
        self.assertIn('developmentReplay',view['label']);self.assertEqual(view['motorCommands'],0)

    def test_each_report_artifact_is_hash_verified(self):
        paths=[self.stages/self.reference['path']]
        paths.extend(self.stages/self.report['sources'][0][name]['path'] for name in ('evidence','observation'))
        for path in paths:
            original=path.read_bytes();path.write_bytes(original+b' ')
            with self.assertRaisesRegex(ValueError,'artifact_hash_changed'):m.load_view(self.latest)
            path.write_bytes(original)

    def test_native_corruption_becomes_frame_error_without_stale_landmarks(self):
        (self.source/'native-camera/after').write_bytes(b'corrupt')
        frame=m.load_view(self.latest)['records'][0]['frames']['after']
        self.assertEqual(frame['status'],'error');self.assertEqual(frame['points'],[])
        self.assertIsNone(frame['image'])

    def test_source_session_tampering_and_artifact_traversal_are_rejected(self):
        (self.source/'session.json').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'source_session_changed'):m.load_view(self.latest)
        self.latest.write_text(json.dumps(dict(path='../outside.json',sha256='0'*64)))
        with self.assertRaises((ValueError,OSError)):m.load_view(self.latest)

    def test_other_camera_coordinates_never_overlay_external_image(self):
        self.observation['perFrame']['after']['frameId']='gripper-camera-frame'
        self.write_pipeline();frame=m.load_view(self.latest)['records'][0]['frames']['after']
        self.assertEqual(frame['status'],'error');self.assertEqual(frame['points'],[])
        self.assertEqual(frame['reason'],'landmarks_image_identity_mismatch')

    def test_jpeg_binding_uses_its_own_pixels_and_identity(self):
        import cv2
        import numpy as np
        im=np.full((12,16,3),80,np.uint8);ok,encoded=cv2.imencode('.jpg',im);self.assertTrue(ok)
        body=encoded.tobytes();path=self.root/'gripper.jpg';path.write_bytes(body)
        pixels=cv2.cvtColor(cv2.imdecode(encoded,cv2.IMREAD_COLOR),cv2.COLOR_BGR2BGRA).tobytes()
        observed=self.observation['perFrame']['after'];observed['frameId']='gripper-frame';observed['pixelSha256']=sha(pixels)
        observed['imageBinding']=dict(path=str(path),encoding='jpeg',imageSha256=sha(body),
            pixelSha256=sha(pixels),width=16,height=12)
        self.write_pipeline();frame=m.load_view(self.latest)['records'][0]['frames']['after']
        self.assertEqual(frame['status'],'verified');self.assertEqual(frame['image'].shape,(12,16,3))
        self.assertEqual(frame['frameId'],'gripper-frame')
        path.write_bytes(body+b'changed');frame=m.load_view(self.latest)['records'][0]['frames']['after']
        self.assertEqual(frame['status'],'error');self.assertEqual(frame['points'],[])

    def fake_sdk(self):
        rec=FakeRecording()
        return rec,SimpleNamespace(RecordingStream=lambda *args,**kwargs:rec,
            Clear=lambda **kwargs:('clear',kwargs),Image=lambda im:('image',im),
            Points2D=lambda points,**kwargs:('points',points,kwargs),TextDocument=lambda text:('text',text),
            FileSink=lambda path:('file',path),GrpcSink=lambda url:('grpc',url))

    def test_default_rrd_export_never_connects_and_records_failure_status(self):
        rec,sdk=self.fake_sdk()
        with patch.object(m,'ROOT',self.root):
            result=m.export(self.latest,self.output/'view.rrd',rr_module=sdk)
        self.assertFalse(result['connected']);self.assertEqual(rec.connections,[]);self.assertEqual(len(rec.saves),1)
        self.assertTrue(rec.disconnected)
        self.assertTrue(any(path=='status' and 'partial' in val[1] for path,val in rec.logs))
        self.assertEqual(sum(val[0]=='points' for _,val in rec.logs),2)

    def test_explicit_local_connect_keeps_file_and_grpc_sinks(self):
        rec,sdk=self.fake_sdk()
        with patch.object(m,'ROOT',self.root):
            result=m.export(self.latest,self.output/'view.rrd',connect='rerun+http://127.0.0.1:9876/proxy',rr_module=sdk)
        self.assertTrue(result['connected']);self.assertEqual(rec.connections[0][0][0],'file')
        self.assertEqual(rec.connections[0][1][0],'grpc');self.assertEqual(rec.saves,[])

    def test_public_output_remote_connect_and_overwrite_are_rejected(self):
        _,sdk=self.fake_sdk()
        with patch.object(m,'ROOT',self.root):
            with self.assertRaisesRegex(ValueError,'private_rrd'):m.export(self.latest,self.root/'public.rrd',rr_module=sdk)
            with self.assertRaisesRegex(ValueError,'explicit_local'):m.export(self.latest,self.output/'x.rrd',
                connect='rerun+http://example.com:9876/proxy',rr_module=sdk)
            path=self.output/'exists.rrd';path.write_bytes(b'existing')
            with self.assertRaisesRegex(ValueError,'new_rrd'):m.export(self.latest,path,rr_module=sdk)

    @unittest.skipUnless(importlib.util.find_spec('rerun'), 'optional rerun SDK not installed')
    def test_real_optional_sdk_saves_a_readable_nonempty_recording_without_connect(self):
        path=self.output/'sdk-smoke.rrd'
        with patch.object(m,'ROOT',self.root):
            result=m.export(self.latest,path)
        self.assertFalse(result['connected'])
        self.assertTrue(path.is_file());self.assertGreater(path.stat().st_size,100)


if __name__=='__main__':unittest.main()
