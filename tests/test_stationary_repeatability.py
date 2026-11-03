import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning import stationary_repeatability as m
from learning import supervised_measurement as shared

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None


def plan():
    return dict(clips=['clip-01'], anchorsSeconds=[4,16,28], beforeWindow=[-.75,-.1],
                afterWindow=[6.366753167,7.116753167], candidateBeforeWindow=[-2.1,-.1],
                candidateAfterWindow=[5.116753167,7.116753167], stabilityThreshold=.5, maxGapSeconds=.3)


class WindowTests(unittest.TestCase):
    def test_wrap_uses_short_arc(self):
        rows = [dict(frameIndex=i, sourceEarliest=t, sourceLatest=t,
                     status='valid', angleDeg=a, identity=['camera'])
                for i,(t,a) in enumerate([(0.1,179.9),(.3,-179.9),(.5,179.8)])]
        result = m.evaluate_window(rows,0,.6,plan())
        self.assertEqual(result['status'],'eligible')
        self.assertAlmostEqual(result['spread'],.3)
        self.assertAlmostEqual(result['value'],179.9)

    def test_gap_or_invalid_frame_is_not_silently_removed(self):
        rows = [dict(frameIndex=i,sourceEarliest=t,sourceLatest=t,status='valid',
                     angleDeg=0,identity=['camera']) for i,t in enumerate([.1,.2,.7])]
        self.assertIn('window_missing_or_gapped',m.evaluate_window(rows,0,.8,plan())['reasons'])
        rows[-1].update(sourceEarliest=.3,sourceLatest=.3,status='invalid')
        result=m.evaluate_window(rows,0,.4,plan())
        self.assertEqual(result['count'],3)
        self.assertEqual(result['failedFrameIds'],[2])
        self.assertIn('window_contains_invalid_frame',result['reasons'])

    def test_candidate_must_keep_endpoint_and_two_second_duration(self):
        p=plan();p['candidateAfterWindow'][1]+=.1
        with self.assertRaisesRegex(ValueError,'candidate_endpoints'):
            m.validate_plan(p)


@unittest.skipIf(cv2 is None,'OpenCV required')
class SourceTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.clip=self.root/'clip-01';self.capture=self.clip/'native-camera'
        (self.capture/'host-ingress').mkdir(parents=True)
        points=[]
        for name,roi in [('p1',[0,0,15,18]),('p2',[16,0,16,18]),('c1',[0,20,15,20]),('c2',[16,20,16,20])]:
            points.append(dict(id=name,roi=roi,hsvLower=[0,200,200],hsvUpper=[5,255,255],minAreaPx=5,maxAreaPx=20))
        self.config=dict(schemaVersion=shared.SCHEMA,width=64,height=48,stabilityThreshold=.5,
            minLinkLengthPx=5,points=points,representation=dict(kind='relative_angle',parent=['p1','p2'],child=['c1','c2']))
        from learning.mac_clock_bridge import SCHEMA,MARGIN_SECONDS
        snapshot=dict(schemaVersion=SCHEMA,hostClockId='session',hostBootId='boot',sourceClock='coremedia_host_clock',
            targetClock='python_monotonic',sharedTimebase='mach_absolute_time',marginSeconds=MARGIN_SECONDS,
            opticalExposureVerified=False,trainingEligible=False,controlEligible=False,
            pythonClock=dict(implementation='mach_absolute_time()',monotonic=True,adjustable=False,resolution=1e-9),
            pairs=[dict(before=10+i*.01,after=10+i*.01,value=1000+i,timescale=100,flags=1,epoch=0) for i in range(8)])
        (self.capture/'host-ingress/clock-before.json').write_text(json.dumps(snapshot))
        image=np.zeros((48,64,4),np.uint8);image[:,:,3]=255
        for x,y in [(5,7),(22,7),(5,27),(22,27)]:image[y-1:y+2,x-1:x+2,:3]=[0,0,255]
        pixels=image.tobytes();frames=[]
        for i in range(401):
            t=20+i*.1;image_name=f'frame-{i}.bgra';meta_name=f'frame-{i}.json'
            (self.capture/image_name).write_bytes(pixels)
            meta=dict(schemaVersion='motionbrain.mac_camera_frame.v2',storageEncoding='apple-lzbitmap-bgra8-v1',
                frameIndex=i,imageFile=image_name,width=64,height=48,rowBytes=256,pixelFormat='BGRA8',
                rawByteLength=len(pixels),imageSha256=shared.sha(pixels),pixelSha256=shared.sha(pixels),storedByteLength=len(pixels),
                convertedHostPtsSeconds=t,clockDomain='coremedia_host_clock',timestampSemantics='cmsamplebuffer_presentation_timestamp',
                actualConfigurationJson='{}',configurationSha256=shared.sha(b'{}'),sessionId='session',viewConfigurationId='view',
                deviceUniqueId='camera',videoMirrored=False,videoRotationAngle=0)
            raw=json.dumps(meta).encode();(self.capture/meta_name).write_bytes(raw)
            frames.append(dict(frameIndex=i,imageFile=image_name,imageSha256=shared.sha(pixels),metadataFile=meta_name,
                metadataSha256=shared.sha(raw),sourceEarliest=t-1e-6,sourceLatest=t+1e-6,clockOffsetBoundsSeconds=[-1e-6,1e-6]))
        self.report=dict(schemaVersion='motionbrain.stationary_capture.v1',motorCommands=0,
                         camera=dict(sessionId='session',hostClockId='session',viewId='view',deviceId='camera',width=64,height=48),
                         frames=frames,recording=dict(status='completed'))
        self.save()
        decoder=patch('learning.lossless_frame.decode',side_effect=lambda stored,size,digest:stored)
        decoder.start();self.addCleanup(decoder.stop)

    def save(self):
        summary=dict(status=self.report['recording']['status'],reason=None,terminalStatus='completed',
                     recordedFrames=len(self.report['frames']),frames=copy.deepcopy(self.report['frames']))
        self.report['recording']={k:v for k,v in summary.items() if k!='frames'}
        (self.capture/'host-ingress/recording-summary.json').write_text(json.dumps(summary))
        terminal=dict(schemaVersion='motionbrain.mac_camera_stream_terminal.v1',sessionId='session',
                      viewConfigurationId='view',deviceUniqueId='camera',status='completed',reason='duration_complete',
                      recordedFrames=len(self.report['frames']),frames=[dict(durable=True,**{k:f[k] for k in
                          ('frameIndex','metadataFile','metadataSha256','imageFile','imageSha256')}) for f in self.report['frames']])
        (self.capture/'terminal.json').write_text(json.dumps(terminal))
        (self.clip/'report.json').write_text(json.dumps(self.report))

    def run_analysis(self,p=None):return m.analyze(self.root,self.config,p or plan())

    def test_valid_fixed_windows_all_attempts_and_diagnostics(self):
        result=self.run_analysis()
        self.assertEqual(result['plannedPhysicalPairs'],3)
        for method in ('A','B'):
            self.assertEqual(result['methodSummary'][method]['eligible'],3)
            self.assertEqual(result['methodSummary'][method]['rmsDelta'],0)
        self.assertEqual(len(result['clips'][0]['frames']),401)
        point=result['clips'][0]['frames'][0]['diagnostics']['points']['p1']
        self.assertEqual(point['areaPx'],9);self.assertEqual(point['brightnessMean'],255)
        self.assertFalse(result['trainingPerformed']);self.assertFalse(result['independentTestSetAvailable'])

    def test_source_hash_mutation_rejects_affected_pairs_and_retains_denominator(self):
        frame=self.report['frames'][35];(self.capture/frame['imageFile']).write_bytes(b'changed')
        result=self.run_analysis()
        for method in ('A','B'):
            self.assertEqual(result['methodSummary'][method]['attempts'],3)
            self.assertEqual(result['methodSummary'][method]['eligible'],2)
            self.assertEqual(result['methodSummary'][method]['rejected'],1)
        self.assertIn('hash_mismatch',result['clips'][0]['frames'][35]['reason'])
        self.assertEqual(result['clips'][0]['rawSummary']['failedFrames'],1)

    def test_actual_missing_marker_is_retained(self):
        frame=self.report['frames'][35];path=self.capture/frame['metadataFile'];meta=json.loads(path.read_text())
        pixels=bytearray((self.capture/frame['imageFile']).read_bytes())
        image=np.frombuffer(pixels,np.uint8).reshape(48,64,4);image[6:9,4:7,:3]=0
        pixels=bytes(pixels);(self.capture/frame['imageFile']).write_bytes(pixels)
        meta['pixelSha256']=meta['imageSha256']=frame['imageSha256']=shared.sha(pixels)
        raw=json.dumps(meta).encode();path.write_bytes(raw);frame['metadataSha256']=shared.sha(raw);self.save()
        result=self.run_analysis()
        self.assertIn('marker_missing',result['clips'][0]['frames'][35]['reason'])
        self.assertEqual(result['methodSummary']['A']['rejected'],1)

    def test_missing_clip_retains_all_planned_pairs(self):
        p=plan();p['clips'].append('clip-02')
        result=self.run_analysis(p)
        self.assertEqual(result['plannedPhysicalPairs'],6)
        self.assertEqual(result['methodSummary']['A']['attempts'],6)
        self.assertEqual(result['laterClipsMethodSummary']['B']['rejected'],3)
        self.assertEqual(len(result['clips'][1]['pairs']),6)

    def test_anchor_cannot_use_unverified_first_frame_time(self):
        self.report['frames'][0]['sourceEarliest'] -= 1.
        self.save()
        result=self.run_analysis()
        self.assertEqual(result['methodSummary']['A']['rejected'],3)
        self.assertEqual(result['methodSummary']['B']['rejected'],3)
        self.assertIn('first_anchor_frame_invalid',result['clips'][0]['reasons'])

    def test_failed_recording_and_configuration_change_cannot_pass(self):
        self.report['recording']['status']='failed';self.save()
        self.assertEqual(self.run_analysis()['methodSummary']['A']['rejected'],3)
        self.report['recording']['status']='completed'
        frame=self.report['frames'][100];path=self.capture/frame['metadataFile'];meta=json.loads(path.read_text());meta['videoMirrored']=True
        raw=json.dumps(meta).encode();path.write_bytes(raw);frame['metadataSha256']=shared.sha(raw);self.save()
        self.assertEqual(self.run_analysis()['methodSummary']['B']['rejected'],3)

    def test_other_capture_contract_or_motor_command_is_rejected(self):
        for schema, count in [('motionbrain.finite_profile_capture.v1',0),('motionbrain.stationary_capture.v1',1)]:
            self.report.update(schemaVersion=schema,motorCommands=count);self.save()
            result=self.run_analysis()
            self.assertEqual(result['methodSummary']['A']['rejected'],3)
            self.assertIn('stationary_capture_contract',result['clips'][0]['reasons'])

    def test_missing_or_failed_terminal_cannot_use_report_completed_claim(self):
        path=self.capture/'terminal.json';original=path.read_text();path.unlink()
        self.assertEqual(self.run_analysis()['methodSummary']['A']['rejected'],3)
        terminal=json.loads(original);terminal.update(status='failed',reason='source_stall');path.write_text(json.dumps(terminal))
        result=self.run_analysis()
        self.assertEqual(result['methodSummary']['B']['rejected'],3)
        self.assertIn('recording_terminal_not_completed_or_identity',result['clips'][0]['reasons'])

    def test_terminal_count_and_summary_frame_mismatch_are_rejected(self):
        path=self.capture/'terminal.json';terminal=json.loads(path.read_text());terminal['recordedFrames']-=1;path.write_text(json.dumps(terminal))
        self.assertEqual(self.run_analysis()['methodSummary']['A']['rejected'],3)
        self.save()
        path=self.capture/'host-ingress/recording-summary.json';summary=json.loads(path.read_text());summary['frames'][0]['sourceEarliest']-=1;path.write_text(json.dumps(summary))
        result=self.run_analysis()
        self.assertEqual(result['methodSummary']['B']['rejected'],3)
        self.assertIn('recording_frame_count_or_summary_mismatch',result['clips'][0]['reasons'])


if __name__=='__main__':unittest.main()
