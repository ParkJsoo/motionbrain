"""Recorder contracts use synthetic persisted receipts; never open a camera."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from learning.action_recorder import ActionRecorder
from learning.artifacts import ArtifactStore
from learning.lossless_frame import ENCODING


def plan():
    return dict(deviceId='test-device',sessionId='test-session',viewId='test-view',
                maxFrames=20,seconds=10,sampleFps=8,maxBytes=1000000,width=8,height=8)


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.clock=Mock();self.clock.monotonic.return_value=100.
        self.rec=ActionRecorder(self.root,Path(__file__),plan(),clock=self.clock)
        self.rec.started=90.;self.rec.offset_low=-.000001;self.rec.offset_high=.000001
        self.rec.store=ArtifactStore(self.root/'host-ingress')

    def frame(self,i=0,receipt_changes=None,**changes):
        name=f'frame-{i:04d}';stored=b'compressed'+bytes([i]);sha=hashlib.sha256(stored).hexdigest()
        m=dict(schemaVersion='motionbrain.mac_camera_frame.v2',sessionId='test-session',
               viewConfigurationId='test-view',deviceUniqueId='test-device',frameIndex=i,
               sourceFrameId=f'test-session:{i}',imageFile=name+'.bgra.lzbitmap',
               storageEncoding=ENCODING,pixelFormat='BGRA8',clockDomain='coremedia_host_clock',
               timestampSemantics='cmsamplebuffer_presentation_timestamp',width=8,height=8,rowBytes=32,
               rawByteLength=256,storedByteLength=len(stored),imageSha256=sha,pixelSha256='a'*64,
               actualConfigurationJson='{}',configurationSha256=hashlib.sha256(b'{}').hexdigest(),
               pts=dict(value=99800+i*50,timescale=1000,epoch=0,flags=1),
               convertedHostPtsSeconds=99.8+i*.05,callbackHostSeconds=99.81+i*.05,
               videoMirrored=False,videoRotationAngle=0)
        m.update(changes);raw=json.dumps(m).encode();(self.root/(name+'.json')).write_bytes(raw)
        (self.root/(name+'.bgra.lzbitmap')).write_bytes(stored)
        receipt=dict(frameIndex=i,metadataFile=name+'.json',imageFile=name+'.bgra.lzbitmap',durable=True,
                     metadataSha256=hashlib.sha256(raw).hexdigest(),imageSha256=sha,persistenceMs=1)
        receipt.update(receipt_changes or {});return receipt

    def test_constructor_never_starts_camera(self):
        with patch('subprocess.Popen') as spawn:
            ActionRecorder(self.root/'not-created','missing-helper',plan())
        spawn.assert_not_called();self.assertFalse((self.root/'not-created').exists())

    def test_slow_save_preserved_without_terminal_fault(self):
        r=self.frame(receipt_changes={'persistenceMs':700});self.rec._packet(r,100.)
        self.rec.check();f=self.rec.frames[0]
        self.assertEqual(f['qualityFlags'],['slow_save']);self.assertLess(f['sourceEarliest'],f['sourceLatest'])
        self.assertEqual(f['clockOffsetBoundsSeconds'],[-.000001,.000001])
        self.assertTrue((self.root/'host-ingress/0000.json').exists())

    def test_persistent_stall_is_latched(self):
        self.rec._packet(self.frame(),100.);self.clock.monotonic.return_value=102.1
        with self.assertRaisesRegex(ValueError,'source_stall'):self.rec.check()
        self.clock.monotonic.return_value=100.
        with self.assertRaisesRegex(ValueError,'source_stall'):self.rec.check()

    def test_image_corruption_rejects_and_keeps_prefix(self):
        self.rec._packet(self.frame(),100.);r=self.frame(1)
        (self.root/'frame-0001.bgra.lzbitmap').write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError,'image_hash'):self.rec._packet(r,100.)
        self.assertEqual(len(self.rec.frames),1)

    def test_pts_regression_rejects(self):
        self.rec._packet(self.frame(),100.)
        r=self.frame(1,convertedHostPtsSeconds=99.7)
        with self.assertRaisesRegex(ValueError,'pts_regressed'):self.rec._packet(r,100.)

    def test_configuration_change_rejects(self):
        self.rec._packet(self.frame(),100.)
        r=self.frame(1,videoMirrored=True)
        with self.assertRaisesRegex(ValueError,'configuration_changed'):self.rec._packet(r,100.)

    def terminal(self,status='completed',reason='duration_complete',**changes):
        t=dict(schemaVersion='motionbrain.mac_camera_stream_terminal.v1',sessionId='test-session',
               viewConfigurationId='test-view',deviceUniqueId='test-device',recordedFrames=len(self.rec.frames),
               status=status,reason=reason,droppedFrames=1)
        t.update(changes);(self.root/'terminal.json').write_text(json.dumps(t))
        return dict(event='terminal',status=status,reason=reason)

    def test_dropped_frame_does_not_fail_terminal(self):
        self.rec._packet(self.frame(),100.)
        self.rec._read_terminal(self.terminal());self.assertEqual(self.rec.terminal['droppedFrames'],1)

    def test_partial_accepted_only_for_requested_close(self):
        with self.assertRaisesRegex(ValueError,'terminal_operator_stop'):
            self.rec._read_terminal(self.terminal('partial','operator_stop'))
        self.rec.terminal=None;self.rec.closing=True
        self.rec._read_terminal(self.terminal('partial','operator_stop'))

    def test_budget_and_path_receipt_rejected(self):
        p=plan();p['seconds']=121
        with self.assertRaises(ValueError):ActionRecorder(self.root,'helper',p)
        r=self.frame(receipt_changes={'imageFile':'../frame.bgra'})
        with self.assertRaisesRegex(ValueError,'receipt_invalid'):self.rec._packet(r,100.)

    def test_close_preserves_prefix_and_is_idempotent(self):
        self.rec._packet(self.frame(),100.);self.rec._fail('test-fault')
        result=self.rec.close();self.assertEqual(result['recordedFrames'],1)
        self.assertEqual(result['status'],'failed');self.assertEqual(self.rec.close(),result)

if __name__=='__main__':unittest.main()
