import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

spec=importlib.util.spec_from_file_location('collection_visualization',
    Path(__file__).resolve().parents[1]/'tools/visualization/collection.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def sha(raw):return hashlib.sha256(raw).hexdigest()


class FakeRecording:
    def __init__(self):self.logs=[];self.sinks=None;self.saved=None;self.disconnected=False
    def save(self,path):self.saved=path
    def set_sinks(self,*sinks):self.sinks=sinks
    def set_time(self,*args,**kwargs):pass
    def log(self,path,value):self.logs.append((path,value))
    def flush(self):pass
    def disconnect(self):self.disconnected=True


class CollectionVisualizationTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.sessions=self.root/'.local/live/sessions'
        self.session=self.sessions/'manual-example';self.camera=self.session/'native-camera'
        self.camera.mkdir(parents=True)
        self.plan=dict(sessionId=self.session.name,joint='gripper',camera=dict(deviceId='camera-id',
            viewId='view-a',width=8,height=8))
        (self.session/'plan.json').write_text(json.dumps(self.plan))
        self.raw=b'compressed-frame';self.pixels=bytes([10,20,30,255])*64
        self.meta=dict(schemaVersion='motionbrain.mac_camera_frame.v2',sessionId=self.session.name,
            frameIndex=0,sourceFrameId=self.session.name+':0',deviceUniqueId='camera-id',
            viewConfigurationId='view-a',configurationSha256='a'*64,width=8,height=8,rawByteLength=256,
            storedByteLength=len(self.raw),imageFile='frame-0000.bgra.lzbitmap',imageSha256=sha(self.raw),
            pixelSha256=sha(self.pixels),storageEncoding='apple-lzbitmap-bgra8-v1',convertedHostPtsSeconds=12.)
        (self.camera/self.meta['imageFile']).write_bytes(self.raw)
        self.meta_path=self.camera/'frame-0000.json';self.meta_path.write_text(json.dumps(self.meta))
        self.now=self.meta_path.stat().st_mtime
        self.decode=patch('tools.visualization.pipeline.decode',return_value=self.pixels)
        self.decode.start();self.addCleanup(self.decode.stop)

    def snap(self,now=None):return m.snapshot(self.sessions,now=self.now if now is None else now)

    def test_explicit_directory_without_hardcoded_scope_follows_frame(self):
        view=self.snap();self.assertEqual(view['status']['followerPhase'],'updating')
        self.assertEqual(view['image'][0,0].tolist(),[30,20,10])
        self.assertEqual(view['status']['operationPath'],'manual_collection')
        self.assertFalse(view['status']['modelSelectionVerified'])
        self.assertEqual(view['status']['motorCommands'],0)

    def test_stale_recording_image_is_hidden_and_terminal_image_is_last_saved(self):
        view=self.snap(self.now+20);self.assertEqual(view['status']['followerPhase'],'stale')
        self.assertIsNone(view['image'])
        (self.session/'session.json').write_text(json.dumps(dict(sessionId=self.session.name,
            phase='completed',error=None,finalIdleOffVerified=True)))
        view=self.snap(self.now+20);self.assertEqual(view['status']['followerPhase'],'last_recorded')
        self.assertIsNotNone(view['image']);self.assertIn('현재 실시간 영상 아님',view['status']['label'])

    def test_failed_terminal_reason_is_visible(self):
        (self.session/'session.json').write_text(json.dumps(dict(sessionId=self.session.name,
            phase='failed',error='landmark_missing',finalIdleOffVerified=True)))
        self.assertEqual(self.snap()['status']['collectorError'],'landmark_missing')

    def test_corrupt_latest_frame_is_reported_without_falling_back_to_old_image(self):
        metadata=dict(self.meta,frameIndex=1,sourceFrameId=self.session.name+':1',imageSha256='0'*64)
        (self.camera/'frame-0001.json').write_text(json.dumps(metadata))
        view=self.snap();self.assertEqual(view['status']['followerPhase'],'invalid_frame')
        self.assertIsNone(view['image']);self.assertIn('changed',view['status']['frameError'])

    def test_context_and_source_identity_mismatches_are_not_displayed(self):
        for key,value in [('viewConfigurationId','wrong'),('sourceFrameId','other:0')]:
            meta=dict(self.meta);meta[key]=value;self.meta_path.write_text(json.dumps(meta))
            self.assertEqual(self.snap()['status']['followerPhase'],'invalid_frame')

    def test_missing_directory_or_frame_is_explicit(self):
        self.assertEqual(m.snapshot(self.root/'missing')['status']['followerPhase'],'waiting')
        self.meta_path.unlink();self.assertEqual(self.snap()['status']['followerPhase'],'missing')

    def test_sensor_and_command_are_only_last_recorded_files(self):
        (self.session/'http-000001-response.json').write_text(json.dumps(dict(receivedAt=13.,
            parsed=dict(shoulderAngle=dict(sensorFresh=True,sensorReady=True,angleDeg=200.3)))))
        (self.session/'command-001-intent.json').write_text(json.dumps(dict(pulse=dict(joint='gripper',
            direction='close',percent=40,durationMs=350))))
        view=self.snap();self.assertEqual(view['sensor'],200.3)
        self.assertIn('현재 센서 직접 조회 아님',view['status']['M4Display'])
        self.assertEqual(view['status']['commandIntents'],1);self.assertEqual(view['status']['commandResults'],0)

    def test_new_session_is_selected_only_from_explicit_parent(self):
        newer=self.sessions/'manual-new';newer.mkdir();plan=dict(self.plan,sessionId=newer.name)
        path=newer/'plan.json';path.write_text(json.dumps(plan));os.utime(path,(self.now+2,self.now+2))
        view=self.snap();self.assertEqual(view['status']['session'],newer.name)
        self.assertIsNone(view['image'])

    def test_follow_saves_stopped_status_and_never_connects_by_default(self):
        rec=FakeRecording();sdk=SimpleNamespace(RecordingStream=lambda *args,**kwargs:rec,
            Clear=lambda **kw:('clear',kw),Image=lambda x:SimpleNamespace(compress=lambda **kw:('image',x,kw)),Scalars=lambda x:('scalar',x),
            TextDocument=lambda text:('text',text),FileSink=lambda x:('file',x),GrpcSink=lambda x:('grpc',x))
        calls=iter([False,True]);output=self.root/'.local/viewer'
        with patch.object(m,'ROOT',self.root),patch.object(m.time,'sleep'):
            result=m.follow(self.sessions,output,rr_module=sdk,stop=lambda:next(calls))
        state=json.loads((output/'status.json').read_text())
        self.assertFalse(state['running']);self.assertEqual(state['followerPhase'],'stopped')
        self.assertIsNone(rec.sinks);self.assertTrue(rec.saved.endswith('collection.rrd'))
        self.assertTrue(rec.disconnected);self.assertEqual(result['motorCommands'],0)

    def test_public_paths_and_remote_rerun_are_rejected(self):
        with patch.object(m,'ROOT',self.root):
            with self.assertRaisesRegex(ValueError,'private_collection'):m.follow(self.root,self.root/'.local/out')
            with self.assertRaisesRegex(ValueError,'explicit_local'):m.follow(self.sessions,self.root/'.local/out',
                connect='rerun+http://example.com:9876/proxy')

    @unittest.skipUnless(importlib.util.find_spec('rerun'), 'optional rerun SDK not installed')
    def test_real_sdk_local_file_follower_exits_cleanly(self):
        calls=iter([False,True]);output=self.root/'.local/sdk-viewer'
        with patch.object(m,'ROOT',self.root),patch.object(m.time,'sleep'):
            result=m.follow(self.sessions,output,stop=lambda:next(calls))
        self.assertGreater(Path(result['output']).stat().st_size,100)
        self.assertFalse(json.loads((output/'status.json').read_text())['running'])


if __name__=='__main__':unittest.main()
