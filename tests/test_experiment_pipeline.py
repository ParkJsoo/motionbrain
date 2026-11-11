"""합성 native 패킷·실제 HTTP 감사·주석·학습을 연결한다. 장치는 열지 않는다.

OS 전용 LZBITMAP과 Mach clock만 합성 어댑터로 대체한다. 감사기/관측 binding/
학습/분할/산출물 저장은 실제 구현을 호출한다. 실제 압축 자료는 별도 로컬 재생 검증.
"""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from learning.action_recorder import ActionRecorder
from learning.artifacts import ArtifactStore
from learning.experiment_pipeline import run_pipeline, SCHEMA
from learning.lossless_frame import ENCODING
from learning.pipeline_evidence import ingest_session
from learning.supervised_transport import SequencePort
from tests.test_supervised_transport import Clock, Controller, pulse


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def fixture(root, sid, x, group, *, failed=False, validation_scope=None):
    store = ArtifactStore(root); clock = Clock(); device = Controller(clock); device.boot = 'a'*32
    port = SequencePort('http://127.0.0.1:8181', 'unused-test-value', device.boot, store, clock=clock)
    port.opener = device; events = []; call = port._call
    def observed(path, payload=None, **kwargs):
        start = clock.monotonic(); response = call(path, payload, **kwargs)
        events.append(dict(kind='http', path=path, payload=payload, response=response,
                           requestStartedAt=start, responseReceivedAt=clock.monotonic()))
        return response
    port._call = observed; port.preflight()
    for _ in range(4): clock.sleep(.8); port._status()
    validation_sha = None
    if validation_scope is not None:
        from learning.batch_motion_admission import install_validation_scope
        manager=type('ScopeFixture',(),{})()
        manager.current=dict(sessionId=sid,phase='recording',events=events)
        manager.command_count=0;manager.clock=clock;manager.store=store
        manager._event=lambda event:events.append(event)
        install_validation_scope(manager,root.parent.parent,validation_scope)
        validation_sha=manager.current['validationScopeSha256']
        clock.sleep(.01)
    at = clock.monotonic()
    intent = dict(kind='command', at=at, commandId=sid+'-001', pulse=pulse('gripper','close',40,350))
    if validation_sha is not None: intent['validationScopeSha256']=validation_sha
    events.append(intent); store.write_json('command-001-intent.json', intent)
    receipt = port.execute(intent['pulse'], lambda: None)
    result = dict(kind='result', at=clock.monotonic(), commandId=sid+'-001', receipt=receipt)
    events.append(result); store.write_json('command-001-result.json', result)
    for _ in range(9): clock.sleep(.8); port._status()
    port.finish()
    camera = dict(deviceId='synthetic-device', sessionId=sid, viewId='fixed-view',
                  maxFrames=500, seconds=60, sampleFps=8, maxBytes=10_000_000, width=32, height=32)
    store.write_json('plan.json', dict(sessionId=sid, joint='gripper', camera=camera, helperSha256='f'*64))
    native = root/'native-camera'; native.mkdir()
    recorder = ActionRecorder(native, 'unused', camera); recorder.started=0.
    recorder.offset_low=-.000001; recorder.offset_high=.000001
    recorder.store=ArtifactStore(native/'host-ingress'); lines=[]
    times=np.arange(at-3.+.02, receipt['offAt']+6.-.02, .125)
    for index,t in enumerate(times):
        after=t>receipt['offAt']; pixels=np.zeros((32,32,4),np.uint8);pixels[:,:,3]=255
        ox=16+int(x)+(1 if after else 0); pixels[18:22,ox-1:ox+1,:3]=[0,0,255]
        pixels[19:22,7:9,:3]=255;pixels[19:22,23:25,:3]=255
        blob=pixels.tobytes();name=f'frame-{index:04d}';image_name=name+'.bgra.lzbitmap'
        (native/image_name).write_bytes(blob)
        metadata=dict(schemaVersion='motionbrain.mac_camera_frame.v2', sessionId=sid,
            viewConfigurationId='fixed-view', deviceUniqueId='synthetic-device', frameIndex=index,
            sourceFrameId=sid+':'+str(index),imageFile=image_name,storageEncoding=ENCODING,
            pixelFormat='BGRA8',clockDomain='coremedia_host_clock',
            timestampSemantics='cmsamplebuffer_presentation_timestamp',width=32,height=32,rowBytes=128,
            rawByteLength=len(blob),storedByteLength=len(blob),imageSha256=sha(blob),pixelSha256=sha(blob),
            actualConfigurationJson='{}',configurationSha256=sha(b'{}'),
            pts=dict(value=round(t*1_000_000),timescale=1_000_000,epoch=0,flags=1),
            convertedHostPtsSeconds=float(t),callbackHostSeconds=float(t+.001),
            videoMirrored=False,videoRotationAngle=0)
        meta=json.dumps(metadata).encode();(native/(name+'.json')).write_bytes(meta)
        packet=dict(frameIndex=index,metadataFile=name+'.json',imageFile=image_name,durable=True,
                    metadataSha256=sha(meta),imageSha256=sha(blob),persistenceMs=1)
        recorder._packet(packet,float(t+.005));lines.append(dict(event='frame',receipt=packet))
    terminal=dict(schemaVersion='motionbrain.mac_camera_stream_terminal.v1', sessionId=sid,
        viewConfigurationId='fixed-view',deviceUniqueId='synthetic-device',recordedFrames=len(times),
        status='completed',reason='duration_complete',droppedFrames=0)
    (native/'terminal.json').write_text(json.dumps(terminal))
    lines.append(dict(event='terminal',status='completed',reason='duration_complete'))
    (native/'helper.stdout').write_text('\n'.join(json.dumps(line) for line in lines)+'\n')
    session=dict(schemaVersion='motionbrain.manual_collection.v1',sessionId=sid,joint='gripper',
        phase='failed' if failed else 'completed',error='operator_stopped' if failed else None,
        datasetEligible=False,trainingPerformed=False,finalIdleOffVerified=True,outputBaselineSequence=5,
        events=events,endedAt=clock.monotonic(),frames=recorder.frames,
        recording=dict(reason=None,recordedFrames=len(times),terminalStatus='completed',clockBridge=dict(hostClockId=sid)))
    if validation_sha is not None: session['validationScopeSha256']=validation_sha
    store.write_json('session.json',session)
    return sha((root/'session.json').read_bytes())


class ExperimentPipelineTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        def clock(capture, hashes):
            sid=json.loads((capture.parent/'plan.json').read_text())['sessionId']
            return dict(basis='before_after_same_mach_clock',hostClockId=sid,
                        validPythonBounds=[0.,100.],initialOffsets=[-.000001,.000001])
        self.clock_patch=patch('learning.manual_collection_dataset.clock_evidence',side_effect=clock)
        self.clock_patch.start();self.addCleanup(self.clock_patch.stop)
        def decode(blob,size,digest):
            if len(blob)!=size or sha(blob)!=digest:raise ValueError('synthetic_pixels_invalid')
            return blob
        self.decoder=patch('learning.pipeline_observations.decode',side_effect=decode)
        self.decoder.start();self.addCleanup(self.decoder.stop)
        self.manifest=dict(schemaVersion=SCHEMA,artifactRoot='.',sources=[],split=dict(
            trainGroups=['train'],evaluationGroups=['eval'],allowRecoveredDevelopmentTraining=True))
        for sid,x,group in [('a',0,'train'),('b',10,'train'),('c',5,'eval')]:
            root=self.root/sid;digest=fixture(root,sid,x,group,failed=sid=='b')
            audited=ingest_session(root,digest,allow_failed_development=True)['visual']['row']
            annotations={}
            for side in ('before','after'):
                f=audited['inputs' if side=='before' else 'targets']['image']
                annotations[side]=dict(frameId=f['frameId'],pixelSha256=f['pixelSha256'],
                    sourceFrameIds=[f['frameId']],method='analyst_points',accepted=True,inputFrameOnly=True,
                    points=dict(object=[16.+x+(side=='after'),20.],jawLeft=[8.,20.],jawRight=[24.,20.]))
            spec=dict(kind='analyst_points',captureGroup=group,annotationProtocol='frame_only_analyst_points.v1',
                context=dict(physicalView='fixed-view',object='synthetic',poseRegime='M1-only',landmarks='manual-synthetic-v1'),
                annotations=annotations)
            self.manifest['sources'].append(dict(id=sid,path=sid,sha256=digest,
                allowFailedDevelopment=True,observer=spec))
        self.path=self.root/'manifest.json';self.save()

    def save(self):self.path.write_text(json.dumps(self.manifest))

    def test_full_audit_observe_train_evaluate_and_repeat_without_duplicate_artifacts(self):
        output=self.root/'out';first=run_pipeline(self.path,output)
        self.assertEqual(first['counts'],dict(listed=3,eligible=3,unavailable=0))
        partition=first['learning']['partitions'][0]
        self.assertEqual(partition['counts'],dict(training=2,evaluation=1))
        self.assertEqual(partition['evaluation']['inSupportEvaluated'],1)
        self.assertFalse(first['controlEligible']);self.assertFalse(first['physicalGraspVerified'])
        paths=list((output/'artifacts').iterdir())
        self.assertEqual(run_pipeline(self.path,output),first)
        self.assertEqual(set((output/'artifacts').iterdir()),set(paths))
        model=first['frozenModels'][0];self.assertEqual(sha((output/model['path']).read_bytes()),model['sha256'])
        self.assertEqual(json.loads((self.root/'b/session.json').read_text())['phase'],'failed')

    def test_partial_observation_and_changed_source_stay_visible_without_aborting_other_sources(self):
        del self.manifest['sources'][0]['observer']['annotations']['after']['points']['jawRight']
        self.manifest['sources'][1]['sha256']='0'*64;self.save()
        report=run_pipeline(self.path,self.root/'out')
        self.assertEqual(report['counts']['listed'],3);self.assertEqual(report['counts']['eligible'],1)
        self.assertEqual(report['sources'][0]['status'],'partial')
        self.assertEqual(report['sources'][1]['reason'],'session_hash_mismatch')

    def test_cross_split_leak_blocks_learning_but_preserves_evidence_report(self):
        duplicate=copy.deepcopy(self.manifest['sources'][0]);duplicate['id']='leak'
        duplicate['observer']['captureGroup']='eval';self.manifest['sources'].append(duplicate);self.save()
        report=run_pipeline(self.path,self.root/'out')
        self.assertEqual(report['learning']['status'],'rejected')
        self.assertIn('overlap',report['learning']['reason'])
        self.assertEqual(len(report['sources']),4)

    def test_corrupt_existing_model_is_not_silently_overwritten(self):
        out=self.root/'out';report=run_pipeline(self.path,out)
        (out/report['frozenModels'][0]['path']).write_bytes(b'corrupt')
        with self.assertRaisesRegex(ValueError,'frozen_model_corrupt'):run_pipeline(self.path,out)

    def test_incomplete_source_directory_does_not_abort_other_sources(self):
        (self.root/'incomplete').mkdir()
        self.manifest['sources'].append(dict(id='incomplete',path='incomplete',sha256='a'*64))
        self.save();report=run_pipeline(self.path,self.root/'out')
        self.assertEqual(report['counts'],dict(listed=4,eligible=3,unavailable=1))
        self.assertEqual(report['sources'][-1]['errorType'],'StorageError')
        self.assertEqual(report['sources'][-1]['status'],'rejected')


if __name__=='__main__':unittest.main()
