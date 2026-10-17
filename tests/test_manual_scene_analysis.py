"""Synthetic checks for the offline diagnostic; no real images or devices."""
import unittest

from learning.dry_run import VirtualClock
from learning.fast_observation import AnchoredRoiMeasurement, MeasurementError
from test_fast_observation import anchor, jpeg, metadata

from learning import manual_scene_analysis as ANALYSIS


class ManualSceneAnalysisTest(unittest.TestCase):
    def test_image_gates_match_live_algorithm_on_fresh_synthetic_frames(self):
        for body in (jpeg(13, -7), jpeg(50), jpeg(lost=True), jpeg(duplicate=True), jpeg(scale=.8)):
            with self.subTest(digest=metadata(body)["imageSha256"]):
                clock = VirtualClock()
                original = jpeg()
                tracker = AnchoredRoiMeasurement(anchor(original), original, clock=clock)
                source = metadata(body, 2_000_000)
                diagnostic = ANALYSIS.diagnose(tracker, body, source)
                now = clock.monotonic()
                live = tracker.measure(body, source, request_started=now-.02, received=now,
                                       host_clock_id="synthetic-clock")
                expected = "image_gates_pass_only" if live["accepted"] else live["reason"]
                self.assertEqual(diagnostic["imageGateReason"], expected)
                for key in ("score", "peakMargin", "bestScaleProbe"):
                    self.assertEqual(diagnostic[key], live[key])
                self.assertFalse(diagnostic["liveAcceptanceEvaluated"])
                self.assertFalse(diagnostic["controlEligible"])
                self.assertFalse(diagnostic["trainingEligible"])

    def test_expired_anchor_stays_expired_even_when_image_gates_pass(self):
        body = jpeg()
        tracker = AnchoredRoiMeasurement(anchor(body), body)
        source = metadata(body, 30_000_000)
        result = ANALYSIS.diagnose(tracker, body, source)
        self.assertTrue(result["anchorExpired"])
        self.assertEqual(result["anchorAgeMs"], 29000)
        self.assertEqual(source["capturedUs"], 30_000_000)
        self.assertEqual(tracker.last_us, tracker.anchor_us)
        self.assertNotIn("accepted", result)
        self.assertNotIn("targetBox", result)

    def test_corrupt_image_identity_and_camera_reboot_cannot_be_diagnosed(self):
        body = jpeg()
        tracker = AnchoredRoiMeasurement(anchor(body), body)
        for source in (metadata(body, 2_000_000, boot="b"*32), metadata(body, 1_000_000),
                       {**metadata(body, 2_000_000), "imageSha256": "0"*64}):
            with self.assertRaises((ValueError, MeasurementError)):
                ANALYSIS.diagnose(tracker, body, source)
