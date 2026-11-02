import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from learning import supervised_measurement as m
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None


@unittest.skipIf(cv2 is None, 'OpenCV required')
class MeasurementTests(unittest.TestCase):
    def config(self):
        return dict(schemaVersion=m.SCHEMA,width=32,height=24,stabilityThreshold=.5,
                    points=[dict(id='fixed',roi=[0,0,10,20],hsvLower=[0,200,200],hsvUpper=[5,255,255],minAreaPx=5,maxAreaPx=20),
                            dict(id='moving',roi=[10,0,20,20],hsvLower=[0,200,200],hsvUpper=[5,255,255],minAreaPx=5,maxAreaPx=20)],
                    representation=dict(kind='distance',points=['fixed','moving']))
    def fixture(self, root):
        root.mkdir(exist_ok=True)
        from learning.mac_clock_bridge import SCHEMA, MARGIN_SECONDS
        snapshot=dict(schemaVersion=SCHEMA,hostClockId='session',hostBootId='boot',sourceClock='coremedia_host_clock',
                      targetClock='python_monotonic',sharedTimebase='mach_absolute_time',marginSeconds=MARGIN_SECONDS,
                      opticalExposureVerified=False,trainingEligible=False,controlEligible=False,
                      pythonClock=dict(implementation='mach_absolute_time()',monotonic=True,adjustable=False,resolution=1e-9),
                      pairs=[dict(before=10+i*.01,after=10+i*.01,value=1000+i,timescale=100,flags=1,epoch=0) for i in range(8)])
        (root/'host-ingress').mkdir();(root/'host-ingress/clock-before.json').write_text(json.dumps(snapshot))
        frames=[]
        for i,t in enumerate([19.3,19.5,19.7,19.85,23.3,23.5,23.7,23.9]):
            image=np.zeros((24,32,4),np.uint8);image[:,:,3]=255
            image[7:10,4:7,:3]=[0,0,255];x=15 if t<20 else 18
            image[7:10,x-1:x+2,:3]=[0,0,255];pixels=image.tobytes()
            image_name=f'frame-{i:04d}.bgra.lzbitmap';meta_name=f'frame-{i:04d}.json'
            (root/image_name).write_bytes(pixels)
            meta=dict(schemaVersion='motionbrain.mac_camera_frame.v2',storageEncoding='apple-lzbitmap-bgra8-v1',
                      frameIndex=i,imageFile=image_name,width=32,height=24,rowBytes=128,pixelFormat='BGRA8',rawByteLength=len(pixels),
                      imageSha256=m.sha(pixels),pixelSha256=m.sha(pixels),storedByteLength=len(pixels),convertedHostPtsSeconds=t,
                      clockDomain='coremedia_host_clock',timestampSemantics='cmsamplebuffer_presentation_timestamp',actualConfigurationJson='{}',configurationSha256=m.sha(b'{}'),sessionId='session',viewConfigurationId='view',
                      deviceUniqueId='camera',videoMirrored=False,videoRotationAngle=0)
            raw=json.dumps(meta).encode();(root/meta_name).write_bytes(raw)
            frames.append(dict(frameIndex=i,imageFile=image_name,imageSha256=m.sha(pixels),metadataFile=meta_name,
                               metadataSha256=m.sha(raw),sourceEarliest=t-1e-6,sourceLatest=t+1e-6,clockOffsetBoundsSeconds=[-1e-6,1e-6]))
        good=dict(id='one',pulse=dict(joint='wrist',motor=2,direction='up',percent=60,durationMs=700,expectedPwm=153),status='executed',
                  dispatchBefore=20.,dispatchAfter=20.1,offAt=21.,postEnd=24.,quietVerifiedUntil=24.,offVerified=True,outputTrace=[{'motor':2,'pwm':153,'atMs':100,'sequence':1},{'motor':2,'pwm':0,'atMs':800,'sequence':2}])
        good.update(requestedPulse=good['pulse'],motor=2)
        bad=dict(id='two',pulse=good['pulse'],status='outcome_unknown',dispatchBefore=26.,offVerified=False)
        return dict(schemaVersion='motionbrain.supervised_collection.v1',plan=dict(captureGroup='capture-a',split='train',pulses=[good['pulse'],bad['pulse']],camera=dict(sessionId='session',viewId='view',deviceId='camera',width=32,height=24)),
                    frames=frames,trials=[good,bad],recording=dict(status='failed'))
    def run_measure(self,report,root,config=None):
        for i,trial in enumerate(report['trials']):
            if trial['status']=='executed':(root.parent/f'trial-{i+1:03d}-result.json').write_text(json.dumps(trial))
        with patch('learning.lossless_frame.decode',side_effect=lambda stored,size,digest: stored):
            return m.measure(report,config or self.config(),root)
    def test_failed_tail_does_not_discard_earlier_trial(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';result=self.run_measure(self.fixture(root),root)
            self.assertEqual(result['summary'],dict(attempts=2,eligible=1,rejected=1))
            self.assertAlmostEqual(result['rows'][0]['delta'],3)
            self.assertAlmostEqual(result['rows'][0]['before']['velocityPerSecond'],0)
            self.assertEqual(len(result['rows'][0]['before']['frameIds']),4)
            self.assertEqual(result['schemaVersion'],m.DATA_SCHEMA)
    def test_corrupt_frame_is_rejected_without_crashing(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root)
            (root/report['frames'][0]['imageFile']).write_bytes(b'corrupt')
            result=self.run_measure(report,root)
            self.assertEqual(result['summary']['eligible'],0)
            self.assertIn('hash_mismatch',result['rows'][0]['reason'])
    def test_intersecting_unknown_command_rejects_window(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root);report['trials'][1]['dispatchBefore']=23.4
            result=self.run_measure(report,root)
            self.assertIn('intersects_other_command',result['rows'][0]['reason'])
    def test_uncertainty_is_not_hidden_by_midpoint(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root)
            for frame in report['frames'][:4]:frame['sourceEarliest']-=1
            result=self.run_measure(report,root)
            self.assertIn('window_missing',result['rows'][0]['reason'])
    def test_recorded_time_must_match_native_metadata(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root);report['frames'][0]['clockOffsetBoundsSeconds']=[.01,.01]
            self.assertIn('clock_offsets_unproven',self.run_measure(report,root)['rows'][0]['reason'])
    def test_marker_ambiguity_and_roi_boundary(self):
        config=self.config();image=np.zeros((24,32,4),np.uint8)
        image[4:7,2:5,:3]=[0,0,255];image[12:15,5:8,:3]=[0,0,255]
        with self.assertRaisesRegex(ValueError,'ambiguous'):m.detect_points(image.tobytes(),config)
        image[:,:,:]=0;image[7:10,0:3,:3]=[0,0,255]
        with self.assertRaisesRegex(ValueError,'touches_roi'):m.detect_points(image.tobytes(),config)
    def test_angular_wrap_and_position(self):
        cfg=dict(representation=dict(kind='relative_angle',parent=['a','b'],child=['c','d']),minLinkLengthPx=1)
        self.assertEqual(m.representation({'a':(0,0),'b':(10,0),'c':(0,0),'d':(0,10)},cfg),90)
        self.assertEqual(m.wrap(-358),2)
        cfg=dict(representation=dict(kind='projected_position',moving='a',reference='b',axis='x'))
        self.assertEqual(m.representation({'a':(20,7),'b':(3,19)},cfg),17)
    def test_no_capture_count_or_exact_120_gate(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root);report['trials']=report['trials'][:1];report['plan']['split']='development'
            result=self.run_measure(report,root)
            self.assertEqual(result['summary']['eligible'],1)
            self.assertFalse(result['trainingPerformed'])
    def test_full_saved_windows_reject_missing_post_end(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root);report['trials'][0].pop('postEnd')
            self.assertEqual(self.run_measure(report,root)['rows'][0]['reason'],'post_window_quiet_not_verified')
    def test_unexpected_post_output_rejects_current_keeps_prior_trial(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root)
            # The first trial finished and proved quiet through its endpoint.
            # A subsequent executed pulse encounters unrelated output while
            # waiting. A full camera window cannot replace quiet evidence.
            second=copy.deepcopy(report['trials'][0]);second.update(id='two',dispatchBefore=26.,dispatchAfter=26.1,offAt=27.)
            second.pop('postEnd');second.pop('quietVerifiedUntil');report['trials'][1]=second
            report['errors']=[dict(type='ValueError',reason='unexpected_quiet_output')]
            result=self.run_measure(report,root)
            self.assertEqual(result['rows'][0]['status'],'eligible')
            self.assertEqual(result['rows'][1]['reason'],'post_window_quiet_not_verified')
    def test_partial_or_missing_quiet_coverage_is_rejected(self):
        for value in (None,23.99,float('nan')):
            with self.subTest(value=value),tempfile.TemporaryDirectory() as t:
                root=Path(t)/'native-camera';report=self.fixture(root);report['trials'][0]['quietVerifiedUntil']=value
                row=self.run_measure(report,root)['rows'][0]
                self.assertEqual(row['status'],'rejected')
                self.assertIn(row['reason'],('post_window_quiet_not_verified','execution_receipt_report_mismatch'))
    def test_invalid_after_clock_is_not_silently_ignored(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root)
            (root/'host-ingress/clock-after.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'clock_sample'):self.run_measure(report,root)
    def test_endpoint_window_must_be_covered(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root);report['frames']=[f for f in report['frames'] if f['frameIndex']!=7]
            result=self.run_measure(report,root)
            self.assertIn('window_missing',result['rows'][0]['reason'])
    def test_wrong_motor_output_rejects_only_trial(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root);report['trials'][0]['outputTrace'][0]['motor']=3
            self.assertIn('unexpected_output',self.run_measure(report,root)['rows'][0]['reason'])
    def test_output_trace_gaps_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root);report['trials'][0]['outputTrace'][-1]['sequence']=3
            self.assertIn('trace_gap',self.run_measure(report,root)['rows'][0]['reason'])
    def test_plan_camera_mismatch_not_accepted(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root);report['plan']['camera']['deviceId']='another-camera'
            self.assertIn('plan_identity_mismatch',self.run_measure(report,root)['rows'][0]['reason'])
    def test_clock_identity_is_bound_to_plan(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/'native-camera';report=self.fixture(root);report['plan']['camera']['sessionId']='another-session'
            with self.assertRaisesRegex(ValueError,'clock_identity'):self.run_measure(report,root)
    def test_invalid_config_rejected(self):
        config=self.config();config['points'][1]['roi']=[100,0,20,20]
        with self.assertRaisesRegex(ValueError,'roi'):m.validate_config(config)

if __name__=='__main__':unittest.main()
