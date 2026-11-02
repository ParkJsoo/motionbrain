"""Opt-in real native codec -> recorder -> endpoint measure; no camera or motor."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

@unittest.skipUnless(os.environ.get('MOTIONBRAIN_TEST_RECORDER'),'set native recorder path for offline codec integration')
class NativeEndpointIntegration(unittest.TestCase):
    def test_real_lossless_frames_through_recorder_and_measurement(self):
        import numpy as np
        from learning.action_recorder import ActionRecorder
        from learning.artifacts import ArtifactStore
        from learning.supervised_measurement import measure
        from tests.test_supervised_measurement import MeasurementTests
        fixture=MeasurementTests()
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=fixture.fixture(root);config=fixture.config()
            config.update(width=1920,height=1080);report['plan']['camera'].update(width=1920,height=1080)
            p=dict(report['plan']['camera'],maxFrames=20,seconds=20,sampleFps=8,maxBytes=100_000_000)
            rec=ActionRecorder(root,os.environ['MOTIONBRAIN_TEST_RECORDER'],p)
            rec.started=10;rec.offset_low=-1e-6;rec.offset_high=1e-6
            rec.store=ArtifactStore(root/'acceptance')
            encoded={}
            for f in report['frames']:
                ip=root/f['imageFile'];mp=root/f['metadataFile'];old=ip.read_bytes();meta=json.loads(mp.read_text())
                key=hashlib.sha256(old).hexdigest()
                if key not in encoded:
                    image=np.zeros((1080,1920,4),np.uint8);image[:,:,3]=255
                    image[:24,:32]=np.frombuffer(old,np.uint8).reshape(24,32,4)
                    raw=image.tobytes();src=Path(t)/(key+'.bgra');dst=Path(t)/(key+'.lzbitmap');src.write_bytes(raw)
                    subprocess.run([os.environ['MOTIONBRAIN_TEST_RECORDER'],'--encode-file',str(src),str(dst)],check=True,capture_output=True)
                    encoded[key]=(raw,dst.read_bytes())
                raw,packed=encoded[key];ip.write_bytes(packed);stamp=meta['convertedHostPtsSeconds'];i=f['frameIndex']
                meta.update(width=1920,height=1080,rowBytes=7680,rawByteLength=len(raw),storedByteLength=len(packed),
                    imageSha256=hashlib.sha256(packed).hexdigest(),pixelSha256=hashlib.sha256(raw).hexdigest(),
                    sourceFrameId=f'session:{i}',callbackHostSeconds=stamp+.01,pts=dict(value=round(stamp*1000),timescale=1000,epoch=0,flags=1))
                mb=json.dumps(meta).encode();mp.write_bytes(mb)
                receipt=dict(frameIndex=i,imageFile=f['imageFile'],metadataFile=f['metadataFile'],durable=True,
                    imageSha256=meta['imageSha256'],metadataSha256=hashlib.sha256(mb).hexdigest(),persistenceMs=10)
                rec._packet(receipt,stamp+.02)
            report['frames']=rec.frames
            for i,row in enumerate(report['trials']):
                if row['status']=='executed':(root.parent/f'trial-{i+1:03d}-result.json').write_text(json.dumps(row))
            result=measure(report,config,root)
            self.assertEqual(result['summary']['eligible'],1)
            self.assertAlmostEqual(result['rows'][0]['delta'],3.)
            self.assertEqual(len(rec.frames),8)
            self.assertEqual(result['rows'][1]['status'],'rejected')

if __name__=='__main__':unittest.main()
