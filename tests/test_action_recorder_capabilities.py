"""Reject the legacy raw helper before camera start; no device is opened."""
import copy
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from learning.action_recorder import ActionRecorder, stream_helper_capabilities
from learning.lossless_frame import ENCODING


def capabilities():
    return dict(schemaVersion='motionbrain.mac_camera_capabilities.v1',stream=dict(
        frameSchemaVersion='motionbrain.mac_camera_frame.v2',storageEncoding=ENCODING,
        processingMethod='lossless_record_only_v1',gracefulStop=True,maxFrames=1000,
        maxSeconds=120,maxSampleFps=30,maxBytes=8400000000,overallDeadlineSeconds=170,
        droppedFramePolicy='fail_close',storageBudgetMs=250))


def plan():
    return dict(deviceId='synthetic',sessionId='synthetic',viewId='synthetic',
                maxFrames=500,seconds=60,sampleFps=8,maxBytes=4000000000)


class StreamHelperCapabilitiesTest(unittest.TestCase):
    def response(self,value):return Mock(returncode=0,stdout=json.dumps(value).encode())

    def test_legacy_helper_is_rejected_before_clock_directory_or_capture(self):
        with tempfile.TemporaryDirectory() as temp:
            output=Path(temp)/'not-created'
            r=ActionRecorder(output,Path(__file__),plan())
            with patch('learning.action_recorder.subprocess.run',return_value=self.response(
                    {'schemaVersion':'motionbrain.mac_camera_capabilities.v1','instrumentationVersion':1})), \
                 patch('learning.action_recorder.subprocess.Popen') as spawn, \
                 patch('learning.action_recorder.MacHostClock') as clock:
                with self.assertRaisesRegex(ValueError,'protocol_incompatible'):r.start()
                spawn.assert_not_called();clock.assert_not_called()
            self.assertFalse(output.exists())
            self.assertEqual(r.close()['status'],'failed')

    def test_supported_contract_and_budgets_are_checked_without_capture(self):
        value=capabilities()
        with patch('learning.action_recorder.subprocess.run',return_value=self.response(value)) as call:
            self.assertEqual(stream_helper_capabilities(__file__,plan()),value)
            self.assertEqual(call.call_args.args[0][-1],'--describe-capabilities')
        for key,new in [('storageEncoding','raw-bgra8-v1'),('gracefulStop',False),
                        ('maxSeconds',15),('maxBytes',True),('overallDeadlineSeconds',190),
                        ('overallDeadlineSeconds',60),('droppedFramePolicy','record_only'),
                        ('storageBudgetMs',5000),('storageBudgetMs',250.0)]:
            bad=copy.deepcopy(value);bad['stream'][key]=new
            with patch('learning.action_recorder.subprocess.run',return_value=self.response(bad)):
                with self.assertRaises(ValueError):stream_helper_capabilities(__file__,plan())

    def test_query_failure_is_bounded_and_does_not_fall_back_to_legacy_capture(self):
        with patch('learning.action_recorder.subprocess.run',side_effect=subprocess.TimeoutExpired('helper',3)):
            with self.assertRaisesRegex(ValueError,'query_failed'):stream_helper_capabilities(__file__,plan())
        with patch('learning.action_recorder.subprocess.run',return_value=Mock(returncode=1,stdout=b'')):
            with self.assertRaisesRegex(ValueError,'query_failed'):stream_helper_capabilities(__file__,plan())


if __name__=='__main__':unittest.main()
