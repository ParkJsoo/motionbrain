import copy
import json
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from learning import gripper_camera_dataset as m


class GripperPairTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.row = dict(sessionId='session', inputs={'image': {'frameId': 'external-before'}},
                        targets={'image': {'frameId': 'external-after'}},
                        evidence=dict(cameraIdentity={'camera': 'external'}, commandAt=20., offReceiptAt=22., cameraHostBounds=[0., 60.]))
        _, encoded = cv2.imencode('.jpg', np.zeros((24, 32, 3), np.uint8))
        body = encoded.tobytes(); boot = 'a'*32
        for name, at, captured in [('before', 10., 100), ('after', 26., 200)]:
            meta = dict(sessionId='session', hostClockId='session', clockDomain='python_monotonic',
                        requestStartedAt=at, receivedAt=at+1, persistedAt=at+2, sha256=m.digest(body),
                        headers={'X-MotionBrain-Camera-Boot-Id': boot,
                                 'X-MotionBrain-Captured-Us': str(captured),
                                 'X-MotionBrain-Capture-Requested-Us': str(captured-1),
                                 'X-MotionBrain-Frame-Id': f'{boot}:{captured}',
                                 'X-MotionBrain-Capture-Mode': 'after_request',
                                 'X-MotionBrain-Clock-Domain': 'camera_boot_us',
                                 'X-MotionBrain-Timestamp-Semantics': 'first_dma_buffer'})
            (self.root/f'esp32-{name}.jpg').write_bytes(body)
            (self.root/f'esp32-{name}.json').write_text(json.dumps(meta))

    def mutate(self, name, key, value):
        p = self.root/f'esp32-{name}.json'; data = json.loads(p.read_text()); data[key] = value
        p.write_text(json.dumps(data))

    def test_camera_coordinates_stay_separate_and_original_row_unchanged(self):
        original = copy.deepcopy(self.row); out = m.bind_gripper_pair(self.row, self.root)
        self.assertEqual(original, self.row)
        self.assertEqual(out['inputs']['image']['width'], 32)
        self.assertEqual(out['evidence']['externalFrameIds'], ['external-before', 'external-after'])
        self.assertEqual(out['evidence']['cameraIdentity']['camera'], 'gripper')

    def test_received_after_command_and_early_post_are_refused(self):
        self.mutate('before', 'persistedAt', 21.)
        with self.assertRaisesRegex(ValueError, 'input_outside_window'): m.bind_gripper_pair(self.row, self.root)
        self.mutate('before', 'persistedAt', 12.)
        self.mutate('after', 'requestStartedAt', 24.)
        with self.assertRaisesRegex(ValueError, 'target_outside_window'): m.bind_gripper_pair(self.row, self.root)

    def test_host_identity_image_tamper_and_no_fresh_semantics(self):
        self.mutate('before', 'hostClockId', 'other')
        with self.assertRaisesRegex(ValueError, 'host_clock'): m.bind_gripper_pair(self.row, self.root)
        self.mutate('before', 'hostClockId', 'session')
        p = self.root/'esp32-before.json'; data = json.loads(p.read_text())
        data['headers']['X-MotionBrain-Capture-Mode'] = 'buffered'; p.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'fresh_source'): m.bind_gripper_pair(self.row, self.root)
        (self.root/'esp32-before.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'image_hash'): m.bind_gripper_pair(self.row, self.root)

    def test_old_input_late_target_and_session_escape_are_refused(self):
        self.row['evidence']['commandAt'] = 55.
        with self.assertRaisesRegex(ValueError, 'input_outside_window'): m.bind_gripper_pair(self.row, self.root)
        self.row['evidence']['commandAt'] = 20.
        self.mutate('after', 'persistedAt', 53.)
        with self.assertRaisesRegex(ValueError, 'target_outside_window'): m.bind_gripper_pair(self.row, self.root)
        self.row['evidence']['cameraHostBounds'] = [15., 60.]
        with self.assertRaisesRegex(ValueError, 'outside_audited_session'): m.bind_gripper_pair(self.row, self.root)

    def test_source_time_reversal_is_refused(self):
        p = self.root/'esp32-after.json'; data = json.loads(p.read_text())
        data['headers']['X-MotionBrain-Captured-Us'] = '50'
        data['headers']['X-MotionBrain-Capture-Requested-Us'] = '49'
        data['headers']['X-MotionBrain-Frame-Id'] = 'a'*32+':50'; p.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'source_time_reversal'): m.bind_gripper_pair(self.row, self.root)

    def test_reboot_with_valid_frame_headers_is_refused(self):
        p = self.root/'esp32-after.json'; data = json.loads(p.read_text())
        data['headers']['X-MotionBrain-Camera-Boot-Id'] = 'b'*32
        data['headers']['X-MotionBrain-Frame-Id'] = 'b'*32+':200'; p.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, 'camera_reboot'): m.bind_gripper_pair(self.row, self.root)


if __name__ == '__main__': unittest.main()
