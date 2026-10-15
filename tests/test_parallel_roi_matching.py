"""Exact scale-map equivalence and bounded worker lifetime; synthetic only."""
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from learning.dry_run import VirtualClock
from learning.fast_observation import AnchoredRoiMeasurement, _match_candidates, _decode
from test_fast_observation import anchor, jpeg, metadata


class ParallelRoiTest(unittest.TestCase):
    def test_full_score_maps_and_input_order_identical(self):
        rng = np.random.default_rng(19)
        for shape in ((64, 96), (240, 320)):
            frame = rng.integers(0, 256, shape, dtype=np.uint8)
            templates = [(scale, rng.integers(0, 256, size, dtype=np.uint8)) for scale, size in
                         ((.8, (13, 17)), (1., (20, 21)), (1.2, (25, 30)))]
            serial = _match_candidates(frame, templates, 1)
            for workers in (2, 3):
                parallel = _match_candidates(frame, templates, workers)
                for a, b in zip(serial, parallel):
                    self.assertEqual(a[:6], b[:6])
                    np.testing.assert_array_equal(a[6], b[6])

    def test_gate_coordinates_and_state_transitions_identical(self):
        original = jpeg()
        for body in (jpeg(13, -7), jpeg(50), jpeg(lost=True), jpeg(duplicate=True), jpeg(scale=.8)):
            results = []
            for workers in (1, 3):
                clock = VirtualClock()
                tracker = AnchoredRoiMeasurement(anchor(original), original, clock=clock,
                                                 match_workers=workers)
                sequence = []
                for image, captured in ((body, 2_000_000), (original, 3_000_000)):
                    now = clock.monotonic()
                    sequence.append(tracker.measure(image, metadata(image, captured),
                        request_started=now-.02, received=now, host_clock_id="synthetic-clock"))
                results.append(sequence)
            self.assertEqual(*results)

    def test_worker_options_are_bounded_and_not_boolean(self):
        body = jpeg()
        for workers in (True, 0, 4, 1.5, None):
            with self.assertRaises(ValueError):
                AnchoredRoiMeasurement(anchor(body), body, match_workers=workers)

    def test_completion_order_does_not_reorder_equal_score_scales(self):
        def fake(frame, item):
            time.sleep((3-item[0])*.003)
            return item[0]
        with patch("learning.fast_observation._match_one", side_effect=fake):
            self.assertEqual(_match_candidates(None, [(1,None), (2,None), (3,None)], 3), [1,2,3])

    def test_error_and_interrupt_join_remaining_workers(self):
        for exception in (RuntimeError("match failed"), KeyboardInterrupt()):
            barrier = threading.Barrier(3, timeout=3)
            finished = []
            def fake(frame, item):
                barrier.wait()
                if item[0] == 0:
                    raise exception
                time.sleep(.02)
                finished.append(item[0])
            with patch("learning.fast_observation._match_one", side_effect=fake):
                with self.assertRaises(type(exception)):
                    _match_candidates(None, [(0,None), (1,None), (2,None)], 3)
            self.assertEqual(sorted(finished), [1,2])
            self.assertFalse(any(t.name.startswith("roi-scale") for t in threading.enumerate()))

    def test_no_worker_survives_successful_frames(self):
        body = jpeg()
        tracker = AnchoredRoiMeasurement(anchor(body), body)
        frame = _decode(body, metadata(body)["imageSha256"])
        for _ in range(4):
            _match_candidates(frame, tracker.templates, 3)
        self.assertFalse(any(t.name.startswith("roi-scale") for t in threading.enumerate()))
