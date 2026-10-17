"""Offline production ring/JSON tests; no device or network operations."""
from pathlib import Path
import json
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class M4StageTraceNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if not compiler:
            raise unittest.SkipTest('native compiler unavailable')
        tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(tmp.cleanup)
        binary = Path(tmp.name) / 'm4-stage-trace'
        flags = []
        if sys.platform == 'darwin':
            sdk = subprocess.check_output(['xcrun', '--sdk', 'macosx', '--show-sdk-path'], text=True).strip()
            flags = ['-isysroot', sdk]
        subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror', *flags,
                        '-I', str(ROOT / 'src'), str(ROOT / 'tests/native/test_m4_stage_trace.cpp'),
                        '-o', str(binary)], check=True, capture_output=True, timeout=60)
        result = subprocess.run([str(binary)], check=True, capture_output=True, text=True, timeout=10)
        cls.snapshots = [json.loads(line) for line in result.stdout.splitlines()]

    def test_empty_success_failed_overwritten_and_wrapped_snapshots(self):
        empty, success, failed, overwritten, wrapped = self.snapshots
        self.assertEqual(empty['records'], [])
        self.assertEqual(empty['nextSequence'], 1)
        self.assertEqual(success['schemaVersion'], 'motionbrain.m4_stage_trace.v1')
        self.assertEqual(success['clockDomain'], 'esp_timer_since_boot_us')
        self.assertFalse(success['active'])
        for key in ('clockRateCalibrated', 'physicalSampleTimeVerified', 'txCompleteVerified'):
            self.assertIs(success[key], False)
        r = success['records'][0]
        self.assertEqual(r['nonce'], '0123abcd')
        self.assertEqual(r['status'], 'replied')
        self.assertEqual(r['protocol'], 3)
        self.assertTrue(r['adcSampled'])
        self.assertEqual(r['replyBytes'], 58)
        self.assertEqual(r['writtenBytes'], 58)
        self.assertEqual(r['pollMarkerMs'], r['replyPollMarkerMs'])
        stages = ['requestParsedUs', 'sensorUpdateEnteredUs', 'afterAdcUs', 'pollStartedUs',
                  'angleReadStartedUs', 'angleReadCompletedUs', 'pollCompletedUs',
                  'replyEnqueueUs', 'replyWriteReturnedUs']
        times = [r[k] for k in stages]
        self.assertEqual(times, sorted(times))
        self.assertEqual(r['maxPendingLoopGapUs'], 3000)
        rejected = failed['records'][1]
        self.assertEqual(rejected['status'], 'expired')
        self.assertEqual(rejected['pollAttempts'], 2)
        self.assertFalse(rejected['pollSucceeded'])
        self.assertIsNone(rejected['replyEnqueueUs'])
        self.assertIsNone(rejected['writtenBytes'])
        self.assertEqual(overwritten['dropped'], 12)
        self.assertEqual([r['sequence'] for r in overwritten['records']], list(range(13, 21)))
        self.assertEqual(wrapped['records'][0]['requestParsedUs'], 4294967296001)
        self.assertIsNone(wrapped['records'][0]['requestLoopGapUs'])

    def test_get_route_is_passive_and_control_loop_order_unchanged(self):
        source = (ROOT / 'src/network/web_server.cpp').read_text()
        self.assertIn('server_.on("/m4_diagnostics", HTTP_GET', source)
        handler = source.split('void MotionBrainWebServer::handleM4Diagnostics()', 1)[1]
        for mutation in ('DebugLog', 'Serial.', '.clear(', '.begin(', 'sample(', 'setSpeed', 'dispatch'):
            self.assertNotIn(mutation, handler)
        main = (ROOT / 'src/main.cpp').read_text().split('void loop()', 1)[1]
        self.assertLess(main.index('shoulderAngleSensor.update'), main.index('serialCommand.update'))
        sensor = (ROOT / 'src/peripheral/shoulder_angle_sensor.cpp').read_text()
        poll = sensor.split('void ShoulderAngleSensor::pollI2c(', 1)[1].split('bool ShoulderAngleSensor::readI2c', 1)[0]
        self.assertLess(poll.index('angleStarted('), poll.index('readI2c(AS5600_REG_RAW_ANGLE'))
        self.assertGreater(poll.index('angleCompleted('), poll.index('readI2c(AS5600_REG_RAW_ANGLE'))

    def test_host_accepts_native_serialized_snapshots_including_failures(self):
        from learning.m4_stage_trace import validate_snapshot
        for value in self.snapshots:
            self.assertEqual(validate_snapshot(value), value)


if __name__ == '__main__':
    unittest.main()
