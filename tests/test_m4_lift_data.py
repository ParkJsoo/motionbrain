"""Synthetic raw receipt replay only: no real camera, network, or motor."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning import m4_lift_data as data, m4_lift_learning as learning
from learning import m4_up_commissioning as core
from learning.action_recorder import ActionRecorder, recording_bridge
from learning.artifacts import ArtifactStore
from learning.lossless_frame import ENCODING, SUFFIX
from learning.mac_clock_bridge import validate_sample
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort
from tests.test_mac_clock_bridge import sample
from tests.test_m4_up_commissioning import Device, scope
from tests.test_supervised_transport import Clock


class LiftDataTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)/'trial'; self.store = ArtifactStore(self.root)
        self.scope = scope()
        requested=copy.deepcopy(getattr(self,'requestedPulse',core.PULSE))
        self.scope.update(schemaVersion=core.LEARNING_SCHEMA, phase='training', modelPath=None, modelSha256=None,
                          observationContractSha256='0'*64, maxObservedDeltaDeg=3.,pulse=requested,expectedStartAngleDeg=195.)
        if requested == core.SHORT_PULSE:self.scope['schemaVersion']=core.LEARNING_SHORT_SCHEMA
        clock = Clock(); device = Device(clock)
        device.angle=195.
        port = SequencePort('http://controller.invalid', 'unused-private-test', device.boot, self.store, clock=clock, compact_reads=True)
        port.opener = device
        port.preflight()
        self.receipt = port._execute_validated(requested, lambda: None)
        self.assertTrue(port.finish())
        self.images = {}
        native, chosen = self.native_fixture()
        features = {}
        for label, frame in zip(('before', 'after'), chosen):
            grip = self.gripper_fixture(label, frame['sourceLatest']+.01)
            external = dict(root='external', ingressFile=f"host-ingress/{frame['frameIndex']:04d}.json",
                            ingressSha256=core.sha((self.root/'external'/f"host-ingress/{frame['frameIndex']:04d}.json").read_bytes()),
                            clockSha256=core.sha((self.root/'external/host-ingress/clock-before.json').read_bytes()))
            record = dict(hostClockId=self.scope['hostClockId'], context=copy.deepcopy(self.scope['context']),
                          external=external, gripper=grip, stable=True,
                          onsite=dict(operatorPresent=True, clearanceVerified=True, jawContactVerified=True,
                                      tableSupportVerified=label == 'before'))
            review = label+'-review.json'; self.write(review, record)
            record.update(evidenceFile=review, evidenceSha256=core.sha((self.root/review).read_bytes()))
            self.write(label+'-observation.json', record)
            pixels = np.frombuffer(self.images[(self.root/'external'/frame['imageFile']).read_bytes()], np.uint8).reshape(1080,1920,4)
            png = cv2.imencode('.png', pixels)[1].tobytes(); self.write(label+'-external.png', png)
            source = dict(id=self.scope['hostClockId']+':'+label, context=copy.deepcopy(self.scope['context']),
                          external=dict(path=label+'-external.png', sha256=core.sha(png)),
                          gripper=dict(path='gripper/esp32-'+label+'.jpg', sha256=grip['imageSha256']))
            self.write(label+'-measurement-source.json', source)
        self.registration = learning.register(self.root/'before-external.png', self.root/'gripper/esp32-before.jpg', self.scope['context'])
        self.registration['cameraBinding'] = dict(external=dict(configurationSha256=core.sha(b'{}'),width=1920,height=1080,
                                                              videoMirrored=False,videoRotationAngle=0),
                                                  gripper=dict(width=240,height=200))
        self.scope['observationContractSha256'] = learning.contract_sha(self.registration)
        self.write('scope.json', self.scope)
        for label in ('before','after'):
            features[label] = learning.measure(self.root, self.read(label+'-measurement-source.json'), self.registration)
            self.write(label+'-measurement.json', features[label])
        result = dict(schemaVersion=self.scope['schemaVersion'], scopeSha256=plan_digest(self.scope), status='completed',
                      errors=[], finalIdleOffVerified=True, trainingEligible=False, mayHaveRun=True,
                      commandOutcome='receipt_recorded', modelSelected=False, selectedPulse=requested,
                      receipt=self.receipt, beforeFeatures=features['before'], afterFeatures=features['after'],
                      observerClose=dict(status='completed', errors=[], native=native, gripper=dict(status='completed')),
                      angleObservations=[dict(at=self.receipt['dispatchBefore'], angleDeg=195., state='ARMED')])
        self.write('result.json', result)
        self.decoder = patch.object(core, 'decode', side_effect=lambda raw,size,sha: self.decode(raw,size,sha))
        self.decoder.start(); self.addCleanup(self.decoder.stop)

    def write(self, name, value):
        path = self.root/name; path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value if isinstance(value, bytes) else json.dumps(value).encode())

    def read(self, name):
        return json.loads((self.root/name).read_bytes())

    def decode(self, raw, size, sha):
        pixels = self.images[raw]
        if len(pixels) != size or core.sha(pixels) != sha:
            raise ValueError('fixture_decode_changed')
        return pixels

    def native_fixture(self):
        start = self.receipt['dispatchBefore']-3.4
        end = self.receipt['offAt']+6.5
        before = sample(start-.1, self.scope['hostClockId']); after = sample(end+.1, self.scope['hostClockId'])
        bridge = recording_bridge(before, after)
        plan = dict(deviceId='camera-id',viewId='test-view',sessionId=self.scope['hostClockId'],
                    maxFrames=500,seconds=90,sampleFps=4,maxBytes=4200000000)
        rec = ActionRecorder(self.root/'external', 'unused', plan)
        rec.started=before['pairs'][-1]['after']; rec.offset_low,rec.offset_high=validate_sample(before)
        rec.store=ArtifactStore(self.root/'external/host-ingress')
        lines=[]
        for i,t in enumerate(np.arange(start,end,.25)):
            rise = 20 if t >= self.receipt['offAt']+3 else 0
            image=np.full((1080,1920,4),255,np.uint8)
            image[730-rise:810-rise,710:770,:3]=40
            image[620-rise:630-rise,890:900,:3]=[255,0,255]
            pixels=image.tobytes(); native=('synthetic-compressed-'+str(i)).encode(); self.images[native]=pixels
            name=f'frame-{i:04d}'; self.write('external/'+name+SUFFIX,native)
            meta=dict(schemaVersion='motionbrain.mac_camera_frame.v2',sessionId=plan['sessionId'],
                      sourceFrameId=f"{plan['sessionId']}:{i}",frameIndex=i,deviceUniqueId='camera-id',
                      viewConfigurationId='test-view',storageEncoding=ENCODING,pixelFormat='BGRA8',
                      clockDomain='coremedia_host_clock',timestampSemantics='cmsamplebuffer_presentation_timestamp',
                      width=1920,height=1080,rowBytes=7680,rawByteLength=len(pixels),storedByteLength=len(native),
                      imageFile=name+SUFFIX,imageSha256=core.sha(native),pixelSha256=core.sha(pixels),
                      actualConfigurationJson='{}',configurationSha256=core.sha(b'{}'),videoMirrored=False,
                      videoRotationAngle=0,pts=dict(value=int(t*1e6),timescale=1000000,epoch=0,flags=1),
                      convertedHostPtsSeconds=float(t),callbackHostSeconds=float(t+.01))
            self.write('external/'+name+'.json',meta)
            packet=dict(frameIndex=i,metadataFile=name+'.json',imageFile=name+SUFFIX,durable=True,
                        metadataSha256=core.sha((self.root/'external'/ (name+'.json')).read_bytes()),
                        imageSha256=core.sha(native),persistenceMs=1)
            rec._packet(packet,float(t+.02)); lines.append(dict(event='frame',receipt=packet))
        terminal=dict(schemaVersion='motionbrain.mac_camera_stream_terminal.v1',sessionId=plan['sessionId'],
                      viewConfigurationId='test-view',deviceUniqueId='camera-id',recordedFrames=len(rec.frames),
                      status='partial',reason='operator_stop',droppedFrames=0)
        self.write('external/terminal.json',terminal)
        lines.append(dict(event='terminal',status='partial',reason='operator_stop'))
        self.write('external/helper.stdout',b''.join(json.dumps(line).encode()+b'\n' for line in lines))
        for name,value in [('clock-before',before),('clock-after',after),('clock-bridge',bridge)]:
            self.write('external/host-ingress/'+name+'.json',value)
        summary=dict(status='partial',reason=None,recordedFrames=len(rec.frames),savedBytes=rec.total_bytes,
                     terminalStatus='partial',clockBridge=bridge,frames=rec.frames)
        self.write('external/host-ingress/recording-summary.json',summary)
        chosen=[max((f for f in rec.frames if f['sourceLatest'] < self.receipt['dispatchBefore']-.2),key=lambda f:f['frameIndex']),
                min((f for f in rec.frames if f['sourceEarliest'] > self.receipt['offAt']+3.1),key=lambda f:f['frameIndex'])]
        return summary,chosen

    def gripper_fixture(self,label,t):
        image=np.full((200,240,3),220,np.uint8)
        image[100:145,130:185]=np.random.default_rng(7).integers(0,255,(45,55,3),dtype=np.uint8)
        raw=cv2.imencode('.jpg',image)[1].tobytes(); stamp=10000 if label=='before' else 20000; boot='b'*32
        meta=dict(sessionId=self.scope['hostClockId'],hostClockId=self.scope['hostClockId'],clockDomain='python_monotonic',
                  requestStartedAt=t,receivedAt=t+.01,persistedAt=t+.02,sha256=core.sha(raw),headers={
                  'X-MotionBrain-Camera-Boot-Id':boot,'X-MotionBrain-Captured-Us':str(stamp),
                  'X-MotionBrain-Capture-Requested-Us':str(stamp-1),'X-MotionBrain-Frame-Id':boot+':'+str(stamp),
                  'X-MotionBrain-Capture-Mode':'after_request','X-MotionBrain-Clock-Domain':'camera_boot_us',
                  'X-MotionBrain-Timestamp-Semantics':'first_dma_buffer'})
        self.write(f'gripper/esp32-{label}.jpg',raw); self.write(f'gripper/esp32-{label}.json',meta)
        return dict(root='gripper',label=label,imageSha256=core.sha(raw),
                    metadataSha256=core.sha((self.root/f'gripper/esp32-{label}.json').read_bytes()))

    def test_complete_new_trial_raw_replay(self):
        row=data.build_row(self.root,self.registration)
        self.assertEqual(row['before']['state'],[0,0,0]); self.assertEqual(row['after']['state'],[20,20,0])
        self.assertEqual(row['episodeIdentity'],['a'*32,6]); self.assertEqual(len(row['sourceCaptureIds']),4)
        self.assertFalse(self.read('result.json')['trainingEligible'])
        self.assertEqual(set(row['pulse']),{'joint','direction','percent','durationMs'})

    def test_failed_old_and_changed_features_reject(self):
        original=self.read('result.json')
        for changes in [dict(status='failed'),dict(errors=[{}]),dict(trainingEligible=True),dict(finalIdleOffVerified=False)]:
            self.write('result.json',{**original,**changes})
            with self.assertRaisesRegex(ValueError,'lift_completed_raw_trial_required'):
                data.build_row(self.root,self.registration)
        self.write('result.json',original)
        measured=self.read('after-measurement.json'); measured['state'][0]+=1;self.write('after-measurement.json',measured)
        with self.assertRaisesRegex(ValueError,'lift_saved_measurement_changed'):
            data.build_row(self.root,self.registration)

    def test_missing_frame_window_and_preview_change_reject(self):
        png=cv2.imdecode(np.frombuffer((self.root/'before-external.png').read_bytes(),np.uint8),cv2.IMREAD_UNCHANGED)
        png[0,0]=0; self.write('before-external.png',cv2.imencode('.png',png)[1].tobytes())
        with self.assertRaisesRegex(ValueError,'lift_preview_pixels_changed'):
            data.build_row(self.root,self.registration)
        frames=[dict(sourceEarliest=t,sourceLatest=t+.01,qualityFlags=[],stdoutReadAt=t+.02,persistenceMs=1)
                for t in (0.,.2,.8,1.)]
        with self.assertRaisesRegex(ValueError,'video_window_gap'):
            data.select_frames(frames,[0.,1.01])

    def test_angle_evidence_preserves_failed_absolute_excursion(self):
        result=self.read('result.json'); result['status']='failed'
        result['errors']=[dict(phase='after_observation',type='ValueError')]
        self.write('result.json',result)
        for path in self.root.glob('http-*-intent.json'):
            intent=self.read(path.name)
            if intent['path'].split('?')[0] in ('/status','/collection_status') and intent['startedAt']>self.receipt['dispatchBefore']:
                name=path.name.replace('-intent.json','-response.json'); response=self.read(name)
                response['parsed']['shoulderAngle']['angleDeg']=198.08
                response['rawUtf8']=json.dumps(response['parsed']);self.write(name,response)
        report=data.build_angle_evidence([self.root],self.registration)
        response=report['responses'][0]
        self.assertEqual(response['status'],'failed');self.assertTrue(response['failedOutcomePreserved'])
        self.assertAlmostEqual(response['maximumObservedAbsDeltaDeg'],3.08)
        self.assertFalse(learning.candidate_angle_screen(report,response['pulse'])['admissible'])
        del result['receipt'];self.write('result.json',result)
        with self.assertRaisesRegex(ValueError,'angle_incomplete_execution_evidence'):
            data.build_angle_evidence([self.root],self.registration)


class ShortLiftDataTests(LiftDataTests):
    requestedPulse=core.SHORT_PULSE

    def test_new_short_raw_pulse_does_not_pass_legacy_scope(self):
        row=data.build_row(self.root,self.registration)
        self.assertEqual(row['pulse']['durationMs'],500)
        scope=self.read('scope.json');scope['schemaVersion']=core.LEARNING_SCHEMA
        result=self.read('result.json');result['schemaVersion']=core.LEARNING_SCHEMA;result['scopeSha256']=plan_digest(scope)
        self.write('scope.json',scope);self.write('result.json',result)
        with self.assertRaisesRegex(ValueError,'lift_learning_scope_invalid'):
            data.build_row(self.root,self.registration)


if __name__=='__main__': unittest.main()
