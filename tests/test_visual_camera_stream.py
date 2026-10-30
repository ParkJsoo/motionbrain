"""Native receipt contract tests using files and pipes only; no camera access."""
import hashlib
import io
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from learning.artifacts import ArtifactStore
from learning.visual_camera_stream import NativeCameraStream
from tests.test_visual_action_live import plan


class CameraStreamTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.p=plan()['camera']
        self.tracker=Mock(fault=None);self.tracker.update.return_value=[dict(id='tip',x=20,y=20,visible=True,confidence=1)]
        self.c=NativeCameraStream(self.root,Path(__file__),self.p,self.tracker,host_clock_id='host-test')
        self.c.offset_low=0;self.c.ingress_store=ArtifactStore(self.root/'ingress')
        self.calls=[];self.c.on_fault=lambda:self.calls.append(1)

    def frame(self,index=0,**changes):
        now=time.monotonic();name=f'frame-{index:04d}'
        pixels=b'\x00'*(64*64*4);digest=hashlib.sha256(pixels).hexdigest()
        m=dict(schemaVersion='motionbrain.mac_camera_frame.v1',sessionId=self.p['sessionId'],viewConfigurationId=self.p['viewId'],
               deviceUniqueId=self.p['deviceId'],sourceFrameId=f"{self.p['sessionId']}:{index}",frameIndex=index,width=64,height=64,rowBytes=256,
               pixelFormat='BGRA8',videoMirrored=False,videoRotationAngle=0,timestampSemantics='cmsamplebuffer_presentation_timestamp',
               clockDomain='coremedia_host_clock',imageFile=name+'.bgra',imageSha256=digest,actualConfigurationJson='{}',
               configurationSha256=hashlib.sha256(b'{}').hexdigest(),pts=dict(value=index+1,timescale=10,epoch=0,flags=1),
               callbackHostSeconds=now,convertedHostPtsSeconds=now-.01)
        m.update(changes);raw=json.dumps(m).encode()
        (self.root/(name+'.json')).write_bytes(raw);(self.root/(name+'.bgra')).write_bytes(pixels)
        return dict(frameIndex=index,metadataFile=name+'.json',imageFile=name+'.bgra',durable=True,
                    metadataSha256=hashlib.sha256(raw).hexdigest(),imageSha256=digest,persistenceMs=1)

    def reader(self,events,exit_code=0,**terminal_changes):
        terminal=dict(schemaVersion='motionbrain.mac_camera_stream_terminal.v1',sessionId=self.p['sessionId'],viewConfigurationId=self.p['viewId'],
                      recordedFrames=2,droppedFrames=0,status='completed',reason='duration_complete')
        terminal.update(terminal_changes);(self.root/'terminal.json').write_text(json.dumps(terminal))
        self.c.process=Mock(stdout=io.BytesIO(b''.join(json.dumps(e).encode()+b'\n' for e in events)))
        self.c.process.wait.return_value=exit_code;self.c.stdout_log=io.BytesIO()
        self.c._reader()

    def terminal(self):return dict(event='terminal',status='completed',reason='duration_complete')

    def frames(self):return [dict(event='frame',receipt=self.frame(i)) for i in range(2)]

    def test_ingress_is_before_tracking_and_persisted_with_source_clocks(self):
        receipt=self.frame();received=time.monotonic();self.c._packet(receipt,received,received+.001)
        saved=json.loads((self.root/'ingress/0000.json').read_text())
        self.assertEqual(saved['at'],received+.001);self.assertEqual(saved['stdoutReadAt'],received)
        self.assertIn('callbackHostSeconds',saved);self.assertIn('sourceEarliest',saved)
        self.assertEqual(self.c.drain()[0],saved)

    def test_success_requires_terminal_and_successful_child_exit(self):
        self.reader(self.frames()+[self.terminal()]);self.assertTrue(self.c.done);self.assertIsNone(self.c.fault)
        self.c.process.wait.assert_called_once_with(timeout=2)

    def test_eof_without_terminal_is_not_success(self):
        self.reader(self.frames());self.assertFalse(self.c.done);self.assertIsNotNone(self.c.fault)

    def test_nonzero_exit_after_successful_terminal_is_not_success(self):
        self.reader(self.frames()+[self.terminal()],exit_code=1);self.assertFalse(self.c.done);self.assertIsNotNone(self.c.fault)

    def test_dropped_frames_or_disagreeing_terminal_fail(self):
        self.reader(self.frames()+[self.terminal()],droppedFrames=1)
        self.assertFalse(self.c.done);self.assertEqual(self.c.fault,'camera_terminal_failed')

    def test_persisted_terminal_failure_overrides_stdout_success(self):
        self.reader(self.frames()+[self.terminal()],status='failed')
        self.assertFalse(self.c.done)

    def test_hash_corruption_fails_before_tracking(self):
        r=self.frame();(self.root/'frame-0000.bgra').write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError,'pixels_invalid'):self.c._packet(r,time.monotonic(),time.monotonic())
        self.tracker.update.assert_not_called();self.assertEqual(self.c.count,0)

    def test_stale_native_pts_fails_before_tracking(self):
        r=self.frame(convertedHostPtsSeconds=time.monotonic()-1)
        with self.assertRaisesRegex(ValueError,'camera_stale'):self.c._packet(r,time.monotonic(),time.monotonic())
        self.tracker.update.assert_not_called()

    def test_pts_regression_rejected(self):
        r=self.frame();self.c._packet(r,time.monotonic(),time.monotonic())
        r=self.frame(1,pts=dict(value=1,timescale=10,epoch=0,flags=1))
        with self.assertRaisesRegex(ValueError,'pts_regressed'):self.c._packet(r,time.monotonic(),time.monotonic())

    def test_storage_failure_triggers_stop_and_preserves_prefix(self):
        events=self.frames()
        with patch.object(self.c.ingress_store,'write_json',side_effect=OSError('disk failed')):
            self.reader(events)
        self.assertEqual(self.c.count,1);self.assertFalse(self.c.done);self.assertEqual(len(self.calls),1)
        self.assertTrue((self.root/'frame-0000.bgra').exists())
