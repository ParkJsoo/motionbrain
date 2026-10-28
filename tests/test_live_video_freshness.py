"""Live display expiry/recovery with synthetic bytes, clocks and loopback HTTP."""
import threading
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch

from tools import motionbrain_dashboard as dashboard
from tools import motionbrain_perception_service as service
from motionbrain_ros_bridge.vision_detection import DetectionConfig
from tests.test_perception_service import capture_headers


class LiveVideoFreshnessTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.clock = patch.object(service.time, "monotonic", side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.state = service.PerceptionState("http://unused.test", DetectionConfig(), stale_seconds=2)

    def capture(self, captured=1000, *, metadata=True, inference_delay=0):
        def detect(*args):
            self.now += inference_delay
            return {"available": True, "detected": False, "reason": "no_objects"}

        with patch.object(service, "fetch_capture", return_value=(
                b"synthetic-jpeg", "image/jpeg", capture_headers(captured) if metadata else {})), \
             patch.object(service, "detect_frame", side_effect=detect):
            self.state.run_once()

    def test_live_frame_expires_at_original_receive_time_and_recovers(self):
        self.capture()
        self.state.mark_error(TimeoutError("camera disconnected"))
        self.now = 102.0
        self.assertIsNotNone(self.state.frame_payload())
        self.assertTrue(self.state.health_payload()["fresh"])
        self.now += 0.001
        self.assertIsNone(self.state.frame_payload())
        self.assertFalse(self.state.health_payload()["fresh"])
        self.capture(2000)
        self.assertIsNotNone(self.state.frame_payload())

    def test_duplicate_responses_do_not_rejuvenate_live_image(self):
        self.capture()
        self.now += 3
        self.capture()
        self.assertIsNone(self.state.frame_payload())
        self.assertEqual(self.state.detect_total, 1)
        self.assertIsNotNone(self.state.frame_payload("a" * 32 + ":1000"))

    def test_slow_inference_does_not_extend_live_image_lifetime(self):
        self.capture(inference_delay=3)
        self.assertIsNone(self.state.frame_payload())

    def test_no_target_or_missing_source_metadata_can_still_display_fresh_image(self):
        self.capture(metadata=False)
        self.assertFalse(self.state.detection_payload()["controlEligible"])
        self.assertIsNotNone(self.state.frame_payload())
        self.now += 3
        self.assertIsNone(self.state.frame_payload())

    def test_wall_clock_jumps_do_not_affect_live_expiry(self):
        self.capture()
        with patch.object(service.time, "time", return_value=-1e10):
            self.assertIsNotNone(self.state.frame_payload())
        self.now += 3
        with patch.object(service.time, "time", return_value=1e10):
            self.assertIsNone(self.state.frame_payload())

    def test_missing_or_invalid_monotonic_time_fails_closed(self):
        self.assertIsNone(self.state.frame_payload())
        self.capture()
        for received in (None, float("nan"), float("inf"), 101.0):
            with self.subTest(received=received):
                self.state.latest_detection["receivedMonotonic"] = received
                self.assertIsNone(self.state.frame_payload())

    def start_server(self, server):
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        def close():
            server.shutdown()
            server.server_close()
            worker.join(2)
        self.addCleanup(close)
        return f"http://127.0.0.1:{server.server_port}"

    def test_http_proxy_rejects_expired_live_frame_but_preserves_history(self):
        perception = self.start_server(service.PerceptionServer(
            ("127.0.0.1", 0), service.PerceptionHandler, self.state, {"*"}))
        proxy = self.start_server(dashboard.DashboardServer(
            ("127.0.0.1", 0), dashboard.DashboardHandler, "", "", perception,
            "red", 1.0, 10, "", "", set(), 100, 20, "cup", 0.5))

        def check(url, status, image=False):
            try:
                response = urllib.request.urlopen(url, timeout=2)
            except urllib.error.HTTPError as exc:
                response = exc
            with response:
                self.assertEqual(response.status, status)
                self.assertEqual(response.headers["Cache-Control"], "no-store")
                body = response.read()
                if image:
                    self.assertEqual(body, b"synthetic-jpeg")
                else:
                    self.assertNotIn(b"synthetic-jpeg", body)

        check(perception + "/api/vision_frame", 503)
        self.capture()
        for path in ("/api/capture", "/api/vision_frame"):
            check(proxy + path, 200, image=True)
        self.now += 3
        check(perception + "/api/vision_frame", 503)
        for path in ("/api/capture", "/api/vision_frame"):
            check(proxy + path, 502)
        check(perception + "/api/vision_frame?frameId=" + "a" * 32 + ":1000", 200, image=True)
        check(perception + "/api/vision_frame?frameId=missing", 404)
        self.capture(2000)
        check(proxy + "/api/vision_frame", 200, image=True)


if __name__ == "__main__":
    unittest.main()
