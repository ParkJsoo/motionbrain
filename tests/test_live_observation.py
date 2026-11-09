"""최신 값의 노후화·연결 실패와 프레임 경계가 거짓 실시간 표시를 막는지 검사한다."""
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from learning.live_observation import LatestFeed, read_preview, shoulder_sample


class LiveObservationTests(unittest.TestCase):
    def test_old_frame_is_hidden_and_new_frame_can_replace_it(self):
        feed = LatestFeed()
        self.assertEqual(feed.snapshot(10., 1.)[0]['phase'], 'waiting')
        feed.publish(dict(sourceAt=10., image='first'))
        self.assertEqual(feed.snapshot(10.5, 1.)[1]['image'], 'first')
        self.assertIsNone(feed.snapshot(11.1, 1.)[1])
        feed.publish(dict(sourceAt=12., image='second'))
        self.assertEqual(feed.snapshot(12.1, 1.)[1]['image'], 'second')
        self.assertIsNone(feed.snapshot(11., 1.)[1])

    def test_disconnected_source_cannot_resume_silently(self):
        feed = LatestFeed()
        feed.publish(dict(sourceAt=10.))
        feed.fail('camera_boot_changed')
        self.assertEqual(feed.snapshot(10.1, 1.)[0]['phase'], 'failed')
        self.assertIsNone(feed.snapshot(10.1, 1.)[1])
        with self.assertRaisesRegex(ValueError, 'cannot_resume'):
            feed.publish(dict(sourceAt=10.2))

    def test_rejected_observation_keeps_raw_image_but_marks_rejection(self):
        feed = LatestFeed()
        feed.publish(dict(sourceAt=10., image='raw', observation=None, observationError='scene_match_ambiguous'))
        status, item = feed.snapshot(10.1, 1.)
        self.assertEqual(status['phase'], 'rejected')
        self.assertEqual(item['image'], 'raw')
        self.assertIsNone(item['observation'])

    def frame(self, **changes):
        payload = b'compressed'
        expected = dict(deviceUniqueId='exact-camera', configurationSha256='config',
                        width=2, height=2, videoMirrored=False, videoRotationAngle=0)
        h = dict(expected, event='preview_frame', sequence=0, sourceHostSeconds=10.,
                 callbackHostSeconds=10.1, rawByteLength=16, storedByteLength=len(payload),
                 pixelSha256='a'*64, imageSha256=hashlib.sha256(payload).hexdigest(),
                 trainingEligible=False, controlEligible=False)
        h.update(changes)
        return expected, io.BytesIO(json.dumps(h).encode()+b'\n'+payload)

    def test_frame_identity_hash_and_current_source_time_are_required(self):
        expected, stream = self.frame()
        with patch('learning.live_observation.decode', return_value=b'pixels'):
            h, pixels, at = read_preview(stream, expected, -1, lambda: 10.2)
        self.assertEqual((h['sequence'], pixels, at), (0, b'pixels', 10.))
        for change, reason in [({'deviceUniqueId': 'other'}, 'identity_changed'),
                               ({'sequence': 2}, 'sequence_changed'),
                               ({'storedByteLength': 100000000}, 'size_invalid'),
                               ({'pixelSha256': None}, 'hash_required'),
                               ({'imageSha256': 'b'*64}, 'hash_changed')]:
            expected, stream = self.frame(**change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, reason):
                read_preview(stream, expected, -1, lambda: 10.2)

    def test_truncated_payload_and_stale_frames_never_publish(self):
        expected, stream = self.frame(storedByteLength=20)
        with self.assertRaisesRegex(ValueError, 'truncated'):
            read_preview(stream, expected, -1, lambda: 10.2)
        expected, stream = self.frame()
        with patch('learning.live_observation.decode', return_value=b'pixels'), self.assertRaisesRegex(ValueError, 'stale'):
            read_preview(stream, expected, -1, lambda: 12.)

    def test_sensor_missing_or_stale_is_not_a_measured_angle(self):
        status = dict(uptimeMs=1000, shoulderAngle=dict(available=True, sensorConnected=True,
            sensorFresh=True, sensorReady=True, ageMs=20, angleDeg=208.65))
        self.assertEqual(shoulder_sample(status), 208.65)
        for key, value in [('sensorFresh', False), ('ageMs', 1000), ('angleDeg', float('nan'))]:
            altered = dict(status, shoulderAngle=dict(status['shoulderAngle'], **{key: value}))
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'not_fresh'):
                shoulder_sample(altered)


@unittest.skipUnless(sys.platform == 'darwin', 'Apple SDK required')
class NativePreviewTests(unittest.TestCase):
    def test_native_preview_build_and_no_capture_capabilities(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as folder:
            binary = str(Path(folder)/'preview')
            build = subprocess.run(['bash', str(root/'tools/macos/build_camera_probe.sh'), binary, '--preview'],
                                   capture_output=True, text=True, timeout=60)
            self.assertEqual(build.returncode, 0, build.stderr)
            cap = json.loads(subprocess.check_output([binary, '--describe-capabilities'], timeout=3))
            self.assertFalse(cap['diskRecording'])
            self.assertFalse(cap['controlEligible'])
            self.assertFalse(cap['trainingEligible'])
            for fps in ['0', '6', 'nan', '2extra']:
                result = subprocess.run([binary, 'no-camera', fps], capture_output=True, timeout=3)
                self.assertEqual(result.returncode, 2)


if __name__ == '__main__':
    unittest.main()
