"""Synthetic diagnostic evidence; no physical-camera performance claims."""
import copy
import hashlib
import json
import unittest

import test_mac_camera_probe as fixtures
from test_mac_camera_probe import write_json


class InstrumentationTest(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.CameraProbeTest()
        self.f.setUp()
        self.addCleanup(self.f.doCleanups)
        f = self.f
        f.plan['requiredInstrumentationVersion'] = 1
        write_json(f.root/'plan.json', f.plan)
        duration = dict(value=1, timescale=30, flags=1, epoch=0)
        self.config = dict(sessionPreset='640x480', outputVideoSettings={'PixelFormatType': 1111970369},
                           activeFormat=dict(width=2, height=1, mediaSubtype=1111970369,
                                             videoMinFrameDuration=duration, videoMaxFrameDuration=duration))
        f.session.update(instrumentationVersion=1, launchHostSeconds=99,
                         actualConfigurationBeforeStart=copy.deepcopy(self.config))
        write_json(f.root/'session.json', f.session)
        f.terminal.update(instrumentationVersion=1, droppedTrace=[], droppedTraceTruncatedCount=0,
                          lifecycle=dict(startRunningBeginHostSeconds=99.1, startRunningEndHostSeconds=99.2,
                                         stopRunningBeginHostSeconds=105, stopRunningEndHostSeconds=105.1,
                                         callbackQueueDrainedHostSeconds=105.2),
                          warmupTrace=[dict(callbackIndex=i, pts=dict(value=99200+i*10, timescale=1000, flags=1, epoch=0),
                                            convertedHostPtsSeconds=99.2+i*.01, callbackHostSeconds=99.201+i*.01,
                                            callbackReturnPreparedHostSeconds=99.202+i*.01) for i in range(30)])
        for i, frame in enumerate(f.frames):
            config = copy.deepcopy(self.config)
            config.update(sampleFormat=dict(width=2,height=1,mediaSubtype=1111970369),
                          deliveredPixelBuffer=dict(width=2,height=1,pixelFormat=1111970369),
                          videoMirrored=False,videoRotationAngle=0)
            frame.update(instrumentationVersion=1,actualConfiguration=config)
            f.terminal['frames'][i]['callbackReturnPreparedHostSeconds'] = 100+i*.2+.015
            self.bind(i)

    def bind(self, i):
        frame = self.f.frames[i]
        serialized = json.dumps(frame['actualConfiguration'], sort_keys=True)
        frame.update(actualConfigurationJson=serialized,configurationSha256=hashlib.sha256(serialized.encode()).hexdigest())
        self.f.save_frame(i)

    def test_complete_diagnostics_remain_ineligible(self):
        result = self.f.result()
        self.assertTrue(result['captureProcessingPassed'],result)
        self.assertTrue(result['instrumentation']['available'])
        self.assertEqual(len(result['instrumentation']['frames']),21)
        self.assertFalse(result['trainingEligible'])
        self.assertFalse(result['overallPassed'])

    def test_required_version_cannot_downgrade(self):
        del self.f.session['instrumentationVersion']
        write_json(self.f.root/'session.json',self.f.session)
        self.f.rejected('instrumentation_version_missing')

    def test_actual_format_change_rejected_even_with_valid_hash(self):
        self.f.frames[1]['actualConfiguration']['activeFormat']['width']=4
        self.bind(1)
        self.f.rejected('instrumentation_configuration_changed')

    def test_delivered_dimensions_bound_to_pixels(self):
        self.f.frames[1]['actualConfiguration']['deliveredPixelBuffer']['width']=4
        self.bind(1)
        self.f.rejected('instrumentation_sample_format_mismatch')

    def test_configuration_hash_and_object_binding(self):
        self.f.frames[1]['actualConfiguration']['sessionPreset']='changed'
        self.f.save_frame(1)
        self.f.rejected('instrumentation_configuration_hash')

    def test_callback_prepare_must_follow_durable_write(self):
        self.f.terminal['frames'][1]['callbackReturnPreparedHostSeconds']=100.205
        self.f.save_terminal()
        self.f.rejected('instrumentation_callback_order')

    def test_callback_cannot_overlap_previous_prepared_marker(self):
        self.f.terminal['frames'][0]['callbackReturnPreparedHostSeconds']=100.3
        self.f.save_terminal()
        self.f.rejected('instrumentation_callback_order')

    def test_trace_count_bound_to_terminal(self):
        self.f.terminal['warmupTrace'].pop()
        self.f.save_terminal()
        self.f.rejected('instrumentation_trace_count')

    def test_lifecycle_order(self):
        self.f.terminal['lifecycle']['stopRunningEndHostSeconds']=99
        self.f.save_terminal()
        self.f.rejected('instrumentation_lifecycle_order')

    def test_warmup_not_hidden_by_negative_age(self):
        self.f.terminal['warmupTrace'][0]['convertedHostPtsSeconds']=1000
        self.f.save_terminal()
        self.f.rejected('instrumentation_warmup_pts')



    def test_unknown_warmup_timing_preserved_without_promoting_eligibility(self):
        self.f.terminal['warmupTrace'][0].update(pts=dict(value=0,timescale=0,flags=0,epoch=0), convertedHostPtsSeconds=None)
        self.f.save_terminal()
        result=self.f.result()
        self.assertTrue(result['captureProcessingPassed'],result)
        self.assertIsNone(result['instrumentation']['warmupTrace'][0]['convertedHostPtsSeconds'])
        self.assertFalse(result['trainingEligible'])

    def test_drop_drained_after_stop_and_unknown_reason(self):
        self.f.terminal.update(droppedFrames=1, droppedTrace=[dict(dropIndex=0, reason=None,
            pts=dict(value=0,timescale=0,flags=0,epoch=0), convertedHostPtsSeconds=None,
            callbackHostSeconds=105.11, callbackReturnPreparedHostSeconds=105.12)])
        self.f.save_terminal()
        self.assertTrue(self.f.result()['captureProcessingPassed'],self.f.result())
        self.f.terminal['droppedTrace'][0]['callbackReturnPreparedHostSeconds']=106
        self.f.save_terminal()
        self.f.rejected('instrumentation_drop_time_order')

    def test_unset_device_duration_is_raw_diagnostic(self):
        self.f.session['actualConfigurationBeforeStart']['activeFormat']['videoMinFrameDuration']=dict(value=0,timescale=0,flags=0,epoch=0)
        write_json(self.f.root/'session.json',self.f.session)
        self.assertTrue(self.f.result()['captureProcessingPassed'],self.f.result())

    def test_cmtime_outside_native_range_rejected(self):
        self.f.terminal['warmupTrace'][0]['pts']['value']=10**400
        self.f.save_terminal()
        self.f.rejected('instrumentation_cmtime_out_of_range')

    def test_bool_dimension_rejected(self):
        self.f.frames[0]['actualConfiguration']['sampleFormat']['height']=True
        self.bind(0)
        self.f.rejected('instrumentation_sample_format_mismatch')

    def test_duplicate_configuration_key_rejected(self):
        frame=self.f.frames[0]
        raw=frame['actualConfigurationJson']
        raw='{"sessionPreset":"hidden",'+raw[1:]
        frame.update(actualConfigurationJson=raw,configurationSha256=hashlib.sha256(raw.encode()).hexdigest())
        self.f.save_frame(0)
        self.f.rejected('instrumentation_duplicate_configuration_key')
