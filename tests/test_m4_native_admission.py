"""Replay cached receipt timing; no device, file, network or capture access."""
import copy
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch

from learning import m4_up_observer as observer
from learning.manual_collection_dataset import select_frames
from tests.test_supervised_transport import Clock


class NativeAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.clock.now = 1282279.643389
        self.adapter = object.__new__(observer.M4DualCameraObserver)
        self.adapter.started, self.adapter.closed, self.adapter.failed = True, None, False
        self.adapter.clock = self.clock
        self.adapter.recorder = SimpleNamespace(lock=threading.RLock(), frames=[], fault=None, done=False, check=Mock())
        self.adapter.observe = Mock(side_effect=AssertionError('must not capture'))

    def frame(self, at, read=None, persistence=1.):
        return dict(sourceEarliest=at, sourceLatest=at+.000002, stdoutReadAt=at+.02 if read is None else read,
                    persistenceMs=persistence, qualityFlags=[])

    def logged_margin_frames(self):
        # The next frame was 1.252 ms short of the conservative persistence
        # boundary. The prior frame left an unmodified 0.300490 s coverage gap.
        last = self.clock.now-.400490
        frames = [self.frame(last-.25*i) for i in reversed(range(12))]
        frames.append(self.frame(1282279.509558, read=1282279.637162, persistence=7.479))
        return frames

    def test_logged_receipt_margin_refuses_then_admits_without_new_capture(self):
        self.adapter.recorder.frames = self.logged_margin_frames()
        original = copy.deepcopy(self.adapter.recorder.frames)
        self.assertFalse(self.adapter.native_pre_window_ready())
        self.clock.now += .002
        self.assertTrue(self.adapter.native_pre_window_ready())
        self.assertEqual(self.adapter.recorder.frames, original)
        self.adapter.observe.assert_not_called()

    def test_future_horizon_cannot_borrow_future_receipts(self):
        self.adapter.recorder.frames = self.logged_margin_frames()
        now = self.clock.now
        self.adapter.recorder.frames.append(self.frame(now+.01, read=now+.03))
        self.assertFalse(self.adapter.native_pre_window_ready())
        self.adapter.recorder.frames.pop()  # A future source never publishes an admissible cache.
        with patch.object(observer, 'select_frames', wraps=select_frames) as policy:
            self.clock.now += .002
            self.assertTrue(self.adapter.native_pre_window_ready())
        self.assertEqual(len(policy.call_args_list), 2)
        self.assertTrue(all(call.kwargs['command_at'] == self.clock.now for call in policy.call_args_list))

    def test_quality_gap_and_only_current_window_coverage_are_refused(self):
        now = self.clock.now
        # Current tail gap .29 s passes; the future tail gap .31 s fails.
        frames = [self.frame(now-.39-.25*i) for i in reversed(range(12))]
        self.adapter.recorder.frames = frames
        self.assertTrue(select_frames(frames, [now-3, now-.1], command_at=now))
        self.assertFalse(self.adapter.native_pre_window_ready())
        self.adapter.recorder.frames = self.logged_margin_frames()
        self.clock.now += .002
        self.adapter.recorder.frames[5]['qualityFlags'] = ['slow_save']
        self.assertFalse(self.adapter.native_pre_window_ready())

    def test_only_expected_policy_failures_are_waitable(self):
        self.assertFalse(self.adapter.native_pre_window_ready())
        self.adapter.recorder.frames = self.logged_margin_frames()
        with patch.object(observer, 'select_frames', side_effect=ValueError('bad_metadata')):
            with self.assertRaisesRegex(ValueError, 'bad_metadata'):
                self.adapter.native_pre_window_ready()
        self.adapter.recorder.check.side_effect = ValueError('recording_source_stall')
        with self.assertRaisesRegex(ValueError, 'recording_source_stall'):
            self.adapter.native_pre_window_ready()

    def test_busy_writer_lock_returns_false_without_waiting_or_reading(self):
        ordinary = threading.Lock()
        ordinary.acquire()
        self.adapter.recorder.lock = ordinary
        try:
            self.assertFalse(self.adapter.native_pre_window_ready())
            self.adapter.recorder.check.assert_not_called()
        finally:
            ordinary.release()
        lock = threading.RLock(); acquired = threading.Event(); release = threading.Event()
        self.adapter.recorder.lock = lock
        def writer():
            with lock:
                acquired.set(); release.wait(1)
        thread = threading.Thread(target=writer)
        thread.start()
        try:
            self.assertTrue(acquired.wait(1))
            self.assertFalse(self.adapter.native_pre_window_ready())
            self.adapter.recorder.check.assert_not_called()
        finally:
            release.set(); thread.join(1)
        self.assertFalse(thread.is_alive())
        # Exceptions must release the lock as well.
        lock = threading.Lock(); self.adapter.recorder.lock = lock
        self.adapter.recorder.frames = self.logged_margin_frames()
        self.adapter.recorder.check.side_effect = ValueError('recording_source_stall')
        with self.assertRaisesRegex(ValueError, 'recording_source_stall'):
            self.adapter.native_pre_window_ready()
        self.assertTrue(lock.acquire(blocking=False)); lock.release()

    def test_published_immutable_snapshot_remains_usable_while_writer_is_busy(self):
        self.adapter.recorder.frames = self.logged_margin_frames()
        self.clock.now += .002
        self.assertTrue(self.adapter.native_pre_window_ready())
        snapshot = self.adapter._native_admission_snapshot
        self.assertIsInstance(snapshot, tuple)
        with self.assertRaises(TypeError):
            snapshot[-1]['sourceEarliest'] = 0
        self.assertIsInstance(snapshot[-1]['qualityFlags'], tuple)
        lock = threading.Lock(); self.adapter.recorder.lock = lock; lock.acquire()
        try:
            self.adapter.recorder.frames[-1]['qualityFlags'].append('writer_mutation')
            self.assertTrue(self.adapter.native_pre_window_ready())
            self.assertEqual(snapshot[-1]['qualityFlags'], ())
            self.clock.now += 2.1
            self.assertFalse(self.adapter.native_pre_window_ready())
        finally:
            lock.release()

    def test_busy_snapshot_never_borrows_late_receipt_and_faults_raise_immediately(self):
        self.adapter.recorder.frames = self.logged_margin_frames()
        self.assertFalse(self.adapter.native_pre_window_ready())  # Publishes, but receipt is not ready yet.
        lock = threading.Lock(); self.adapter.recorder.lock = lock; lock.acquire()
        try:
            self.assertFalse(self.adapter.native_pre_window_ready())
            self.clock.now += .002
            self.assertTrue(self.adapter.native_pre_window_ready())
            for flag in ('fault', 'done', 'closing'):
                setattr(self.adapter.recorder, flag, 'fault' if flag == 'fault' else True)
                with self.assertRaisesRegex(ValueError, 'm4_native_stream_failed'):
                    self.adapter.native_pre_window_ready()
                setattr(self.adapter.recorder, flag, None if flag == 'fault' else False)
        finally:
            lock.release()

    def test_wait_uses_cached_frames_until_available_and_honors_deadline(self):
        self.adapter.recorder.frames = self.logged_margin_frames()
        tick = Mock()
        self.assertTrue(self.adapter.wait_pre_dispatch_window(deadline=self.clock.now+.2, tick=tick))
        self.assertGreaterEqual(tick.call_count, 2)
        self.adapter.observe.assert_not_called()
        self.adapter.recorder.frames = []
        with self.assertRaisesRegex(ValueError, 'm4_camera_wait_expired'):
            self.adapter.wait_pre_dispatch_window(deadline=self.clock.now+.1, tick=lambda:None)


if __name__ == '__main__':
    unittest.main()
