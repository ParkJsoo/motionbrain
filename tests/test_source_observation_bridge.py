"""Execute pure production bridge boundary functions without importing ROS.

AST selects complete function definitions, not reimplementations. This verifies
field/age conversion only; generated messages and ROS transport need ROS tests.
"""
import ast
import math
from pathlib import Path
import unittest

PATH = Path(__file__).resolve().parents[1] / 'ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/motionbrain_status_node.py'
NAMES = {'source_observation_fields', 'add_perception_transport_age'}
MODULE = ast.parse(PATH.read_text())
ENV = {'Any': object, 'math': math, 'as_str': lambda v: '' if v is None else str(v)}
exec(compile(ast.Module(body=[node for node in MODULE.body if isinstance(node, ast.FunctionDef) and node.name in NAMES], type_ignores=[]), str(PATH), 'exec'), ENV)


class SourceObservationBridgeTest(unittest.TestCase):
    def payload(self):
        return dict(sourceFrameId='aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa:10', cameraBootId='aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa', capturedUs=10,
                    sourceClockDomain='camera_boot_us', sourceTimestampSemantics='first_dma_buffer',
                    sourceTimeKnown=True, sourceAgeMs=20.0, sourceFresh=True, held=False,
                    detectionSourceFrameId='aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa:10', controlEligible=True)

    def test_fields_survive_boundary(self):
        fields = ENV['source_observation_fields'](self.payload())
        self.assertEqual(fields['source_frame_id'], 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa:10')
        self.assertEqual(fields['captured_us'], 10)
        self.assertEqual(fields['source_age_ms'], 20)
        self.assertTrue(fields['source_time_known'])
        self.assertFalse(fields['held'])

    def test_legacy_and_invalid_metadata_fail_closed(self):
        fields = ENV['source_observation_fields']({})
        self.assertFalse(fields['source_time_known'])
        self.assertFalse(fields['control_eligible'])
        self.assertTrue(fields['held'])
        for value in (float('nan'), float('inf'), -1, '20', True, 10**1000):
            payload = self.payload(); payload['sourceAgeMs'] = value
            self.assertFalse(ENV['source_observation_fields'](payload)['source_time_known'])
        for value in (0, -1, 2**64, '10', True):
            payload = self.payload(); payload['capturedUs'] = value
            self.assertFalse(ENV['source_observation_fields'](payload)['source_time_known'])

    def test_malformed_camera_boot_fails_closed(self):
        for boot in ('boot', 'A' * 32, 'z' * 32, '', None):
            payload = self.payload(); payload['cameraBootId'] = boot
            self.assertFalse(ENV['source_observation_fields'](payload)['source_time_known'])

    def test_poll_latency_adds_to_original_age(self):
        payload = self.payload()
        ENV['add_perception_transport_age'](payload, 0.125)
        self.assertEqual(payload['sourceAgeMs'], 145)
        self.assertEqual(payload['sourceFrameId'], 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa:10')
        for delay in (-1, float('nan'), float('inf')):
            payload = self.payload()
            ENV['add_perception_transport_age'](payload, delay)
            self.assertFalse(payload['controlEligible'])
            self.assertFalse(payload['sourceTimeKnown'])
