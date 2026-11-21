import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from tools.visualization import collector_live as live


class CollectorLiveTest(unittest.TestCase):
    def test_latest_committed_block_skips_incomplete_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for block, index, committed in [('gripper', 100, True), ('gripper-02', 1, True),
                                             ('gripper-02', 2, False)]:
                folder = root / block / f'capture-{index:04d}'
                folder.mkdir(parents=True)
                if committed:
                    (folder / 'source-binding.json').touch()
            self.assertEqual(live.latest(root, 'gripper'),
                             root / 'gripper-02/capture-0001/source-binding.json')

    def test_gripper_identity_and_stale_cutoff(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            path = folder / 'source-binding.json'
            raw = b'collector-jpeg'
            path.write_text(json.dumps(dict(record=dict(sourceEarliest=10, frame=dict(frameId='boot:4')),
                                            imageSha256=hashlib.sha256(raw).hexdigest())))
            (folder / 'esp32-frame.jpg').write_bytes(raw)
            self.assertEqual(live.read_frame(path, 'gripper', lambda: 11), (10, 'boot:4', raw))
            self.assertIsNone(live.read_frame(path, 'gripper', lambda: 12))
            (folder / 'esp32-frame.jpg').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'hash'):
                live.read_frame(path, 'gripper', lambda: 11)

    def test_stalled_reader_cannot_delay_clear_or_publish_expired_frame(self):
        stop = threading.Event()
        now = [10.]
        clear_count = []
        def reader(root, camera, single_command, slots, lock, stop):
            if camera == 'external':
                with lock:
                    slots[camera] = (Path('first'), (10., 'session:1', b'jpeg'))
                stop.wait(2)
            else:
                with lock:
                    slots[camera] = (Path('old'), (1., 'boot:1', b'jpeg'))

        rec, rr = Mock(), Mock()
        def log(entity, value):
            if value == rr.EncodedImage.return_value:
                now[0] = 12.
            elif entity == 'cameras/external':
                clear_count.append(now[0])
                stop.set()
        rec.log.side_effect = log
        timer = threading.Timer(1, stop.set)
        timer.start()
        try:
            with patch.object(live, 'watch', reader), patch.object(live.time, 'monotonic', side_effect=lambda: now[0]):
                live.monitor(Path('.'), rec, rr, stop)
        finally:
            timer.cancel()
        rr.EncodedImage.assert_called_once()
        self.assertEqual(clear_count, [12., 12.])
