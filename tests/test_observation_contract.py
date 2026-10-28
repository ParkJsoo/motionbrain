"""Synthetic camera responses and monotonic clocks only; no device claims."""
import threading
import unittest
from unittest.mock import patch

from tools import motionbrain_perception_service as service
from motionbrain_ros_bridge.observation import parse_capture_headers
from motionbrain_ros_bridge.vision_detection import DetectionConfig
from tests.test_perception_service import capture_headers


class ObservationContractTest(unittest.TestCase):
    def setUp(self):
        self.state = service.PerceptionState(
            "http://camera.test", DetectionConfig(), stale_seconds=1,
            display_hold_seconds=5, observation_cache_size=2, max_frame_bytes=16)
        self.now = 10.0
        self.live = {"available": True, "detected": True, "label": "cup",
                     "targetType": "object", "targetBox": {"x": 1},
                     "detector": {"name": "fake"}}

    def capture(self, captured=1000, *, frame=b"jpeg", age=0, boot="a" * 32,
                headers=None, detected=True, request=0.02, inference=0.03):
        start = self.now
        self.now += request + inference
        result = dict(self.live, detected=detected)
        with patch.object(service, "fetch_capture", return_value=(
                frame, "image/jpeg", capture_headers(captured, age, boot) if headers is None else headers)), \
             patch.object(service, "detect_frame", return_value=result), \
             patch.object(service.time, "monotonic", side_effect=[start, start + request, self.now]):
            self.state.run_once()
        return self.payload()

    def payload(self):
        with patch.object(service.time, "monotonic", return_value=self.now):
            return self.state.detection_payload()

    def test_source_identity_hash_and_capture_bounds(self):
        value = self.capture(age=100000)
        self.assertEqual(value["sourceFrameId"], "a" * 32 + ":1000")
        self.assertEqual(value["detectionSourceFrameId"], value["sourceFrameId"])
        self.assertEqual(len(value["imageSha256"]), 64)
        self.assertEqual(value["sourceTimestampSemantics"], "first_dma_buffer")
        self.assertAlmostEqual(value["sourceAgeMs"], 150)
        self.assertAlmostEqual(value["captureHostBounds"]["earliest"], 9.9)
        self.assertAlmostEqual(value["captureHostBounds"]["latest"], 9.92)
        self.assertTrue(value["controlEligible"])
        self.assertFalse(value["captureHostBoundsCalibrated"])
        self.assertIsNone(value["captureHostBoundsUncertaintySeconds"])
        self.assertIsNone(value["captureHostBoundsEvidenceRef"])
        self.assertIn("assumption", value["captureHostBoundsBasis"])

    def test_receive_latency_and_inference_can_expire_source(self):
        value = self.capture(age=700000, request=0.2, inference=0.2)
        self.assertTrue(value["fresh"])
        self.assertFalse(value["sourceFresh"])
        self.assertFalse(value["controlEligible"])
        self.assertAlmostEqual(value["sourceAgeMs"], 1100)

    def test_missing_metadata_is_display_only(self):
        value = self.capture(headers={})
        self.assertTrue(value["detected"])
        self.assertTrue(value["fresh"])
        self.assertFalse(value["sourceTimeKnown"])
        self.assertFalse(value["sourceFresh"])
        self.assertFalse(value["controlEligible"])
        self.assertEqual(value["stableFrames"], 0)
        self.assertIsNone(value["sourceFrameId"])

    def test_malformed_metadata_rejected(self):
        for field, value in [("Captured-Us", "0"), ("Captured-Us", "-1"), ("Captured-Us", "1.0"),
                             ("Captured-Us", "01000"), ("Source-Age-Us", "nan"),
                             ("Camera-Boot-Id", "no-boot"), ("Frame-Id", "wrong"),
                             ("Timestamp-Semantics", "response_time"), ("Clock-Domain", "utc")]:
            with self.subTest(field=field):
                headers = capture_headers()
                headers["X-MotionBrain-" + field] = value
                self.assertFalse(parse_capture_headers(headers)["sourceMetadataValid"])

    def test_headers_case_insensitive(self):
        headers = {k.lower(): v for k, v in capture_headers().items()}
        self.assertTrue(parse_capture_headers(headers)["sourceMetadataValid"])

    def test_repeated_frame_keeps_first_age_and_observation(self):
        first = self.capture()
        self.now += 2
        second = self.capture()
        self.assertEqual(first["observationId"], second["observationId"])
        self.assertEqual(first["receivedMonotonic"], second["receivedMonotonic"])
        self.assertEqual(second["stableFrames"], 1)
        self.assertFalse(second["sourceFresh"])
        self.assertEqual(self.state.detect_total, 1)

    def test_unique_frames_increment_stability(self):
        self.capture()
        self.assertEqual(self.capture(2000)["stableFrames"], 2)

    def test_same_id_different_bytes_quarantines_original(self):
        first = self.capture()
        second = self.capture(frame=b"other")
        self.assertFalse(second["controlEligible"])
        self.assertEqual(second["sourceMetadataError"], "source_frame_content_conflict")
        self.assertEqual(second["imageSha256"], first["imageSha256"])
        self.assertEqual(self.state.frame_payload(first["sourceFrameId"])[0], b"jpeg")
        self.assertFalse(self.capture()["controlEligible"])

    def test_eviction_cannot_rejuvenate_duplicate(self):
        first = self.capture()
        self.capture(2000)
        self.capture(3000)
        self.assertIsNone(self.state.observation_payload(first["sourceFrameId"]))
        self.assertIsNone(self.state.frame_payload(first["sourceFrameId"]))
        repeated = self.capture()
        self.assertFalse(repeated["controlEligible"])
        self.assertEqual(repeated["sourceMetadataError"], "retired_or_out_of_order_source_frame")
        self.assertEqual(len(self.state.observations), 2)

    def test_out_of_order_capture_is_ineligible(self):
        self.capture(2000)
        value = self.capture(1500)
        self.assertFalse(value["controlEligible"])

    def test_cached_out_of_order_replay_is_not_latest_control_evidence(self):
        first = self.capture()
        self.capture(2000)
        repeated = self.capture()
        self.assertFalse(repeated["controlEligible"])
        self.assertEqual(repeated["sourceMetadataError"], "out_of_order_cached_source_frame")
        with patch.object(service.time, "monotonic", return_value=self.now):
            historical = self.state.observation_payload(first["sourceFrameId"])
        self.assertTrue(historical["sourceMetadataValid"])
        self.assertEqual(historical["observationId"], first["observationId"])

    def test_camera_restart_retires_old_boot_and_resets_stability(self):
        first = self.capture()
        new = self.capture(1, boot="b" * 32)
        self.assertTrue(new["controlEligible"])
        self.assertEqual(new["stableFrames"], 1)
        old = self.capture()
        self.assertFalse(old["controlEligible"])
        self.assertEqual(old["sourceMetadataError"], "retired_camera_boot")
        self.assertEqual(old["imageSha256"], first["imageSha256"])

    def test_bounded_boot_registry_fails_closed(self):
        self.state.max_camera_boots = 1
        self.capture()
        self.assertEqual(self.capture(1, boot="b" * 32)["sourceMetadataError"], "camera_boot_registry_exhausted")

    def test_hold_retains_geometry_source_and_never_controls(self):
        first = self.capture()
        held = self.capture(2000, detected=False)
        self.assertTrue(held["held"])
        self.assertTrue(held["sourceFresh"])
        self.assertFalse(held["controlEligible"])
        self.assertNotEqual(first["sourceFrameId"], held["sourceFrameId"])
        self.assertEqual(held["detectionSourceFrameId"], first["sourceFrameId"])

    def test_wall_clock_jumps_do_not_change_freshness(self):
        self.capture()
        with patch.object(service.time, "time", return_value=-1e10):
            self.assertTrue(self.payload()["controlEligible"])
        self.now += 2
        with patch.object(service.time, "time", return_value=1e10):
            self.assertFalse(self.payload()["sourceFresh"])

    def test_monotonic_regression_fails_closed(self):
        self.capture()
        self.now = 0
        self.assertFalse(self.payload()["sourceFresh"])

    def test_payload_mutation_does_not_corrupt_cached_evidence(self):
        first = self.capture()
        first["targetBox"]["x"] = 100
        cached = self.state.observation_payload(first["sourceFrameId"])
        self.assertEqual(cached["targetBox"]["x"], 1)
        cached["targetBox"]["x"] = 200
        self.assertEqual(self.payload()["targetBox"]["x"], 1)
        health = self.state.health_payload()
        health["detector"]["name"] = "evil"
        self.assertEqual(self.state.health_payload()["detector"]["name"], "fake")

    def test_exact_image_does_not_follow_latest(self):
        first = self.capture(frame=b"first")
        self.capture(2000, frame=b"second")
        self.assertEqual(self.state.frame_payload(first["sourceFrameId"])[0], b"first")
        with patch.object(service.time, "monotonic", return_value=self.now):
            self.assertEqual(self.state.frame_payload()[0], b"second")
        self.assertIsNone(self.state.frame_payload("missing"))

    def test_oversize_frame_rejected_before_inference(self):
        with self.assertRaisesRegex(ValueError, "frame_exceeds_byte_limit"):
            self.capture(frame=b"x" * 17)
        self.assertIsNone(self.state.frame_payload())

    def test_fetch_reads_headers_and_bounds_bytes(self):
        response = type("Response", (), {
            "headers": capture_headers() | {"Content-Type": "image/jpeg"},
            "read": lambda self, size: b"x" * min(size, 4),
            "__enter__": lambda self: self,
            "__exit__": lambda *args: None,
        })()
        with patch.object(service.urllib.request, "urlopen", return_value=response):
            frame, content, headers = service.fetch_capture("http://fake", 1, 4)
            self.assertEqual(frame, b"xxxx")
            self.assertEqual(content, "image/jpeg")
            self.assertTrue(parse_capture_headers(headers)["sourceTimeKnown"])
            with self.assertRaisesRegex(ValueError, "frame_exceeds_byte_limit"):
                service.fetch_capture("http://fake", 1, 3)

    def test_atomic_read_during_inference_sees_previous_complete_record(self):
        first = self.capture(frame=b"first")
        entered, release = threading.Event(), threading.Event()
        def detector(*args):
            entered.set()
            self.assertTrue(release.wait(2))
            return dict(self.live)
        with patch.object(service, "fetch_capture", return_value=(b"second", "image/jpeg", capture_headers(2000))), \
             patch.object(service, "detect_frame", side_effect=detector), \
             patch.object(service.time, "monotonic", return_value=self.now):
            worker = threading.Thread(target=self.state.run_once)
            worker.start()
            try:
                self.assertTrue(entered.wait(2))
                self.assertEqual(self.state.detection_payload()["sourceFrameId"], first["sourceFrameId"])
                self.assertEqual(self.state.frame_payload()[0], b"first")
            finally:
                release.set()
                worker.join(2)
        with patch.object(service.time, "monotonic", return_value=self.now):
            self.assertEqual(self.state.frame_payload()[0], b"second")

    def test_exact_http_routes_and_missing_id_do_not_fallback(self):
        first = self.capture()
        handler = service.PerceptionHandler.__new__(service.PerceptionHandler)
        handler.server = type("Server", (), {"state": self.state, "allowed_origins": {"*"}})()
        handler.send_json = lambda payload, *a, **kw: setattr(handler, "result", payload)
        handler.send_error_json = lambda status, error: setattr(handler, "result", (status, error))
        handler.path = "/api/observation?frameId=" + first["sourceFrameId"]
        handler.do_GET()
        self.assertEqual(handler.result["observation"]["observationId"], first["observationId"])
        handler.path = "/api/observation?frameId=missing"
        handler.do_GET()
        self.assertEqual(handler.result[0], 404)
        handler.path = "/api/observation"
        handler.do_GET()
        self.assertEqual(handler.result[0], 400)
        handler.path = "/api/vision_frame?frameId=missing"
        handler.do_GET()
        self.assertEqual(handler.result[0], 404)
        handler.path = "/api/vision_frame?frameId="
        handler.do_GET()
        self.assertEqual(handler.result[0], 400)


if __name__ == "__main__":
    unittest.main()
