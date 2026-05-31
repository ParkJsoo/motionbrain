import unittest
from unittest.mock import patch

import cv2
import numpy as np

from tools import motionbrain_dashboard as dashboard
from tools import motionbrain_perception_service as service
from tools.motionbrain_dashboard import DashboardServer
from tools.motionbrain_perception_service import PerceptionHandler
from tools.motionbrain_perception_service import PerceptionState
from motionbrain_ros_bridge.vision_detection import DetectionConfig


def make_jpeg_with_red_target() -> bytes:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[:] = (20, 20, 20)
    cv2.rectangle(image, (64, 42), (96, 78), (0, 0, 255), -1)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


class PerceptionServiceTest(unittest.TestCase):
    def make_state(self) -> PerceptionState:
        return PerceptionState(
            "http://camera.local",
            DetectionConfig(mode="color", color="red"),
            timeout=0.2,
            interval=0.1,
            stale_seconds=10.0,
        )

    def test_run_once_updates_detection_and_health(self) -> None:
        state = self.make_state()
        frame = make_jpeg_with_red_target()

        with patch.object(service, "fetch_bytes", return_value=(frame, "image/jpeg")):
            state.run_once()

        detection = state.detection_payload()
        self.assertTrue(detection["available"])
        self.assertTrue(detection["detected"])
        self.assertTrue(detection["fresh"])
        self.assertEqual(detection["cameraUrl"], "http://camera.local")
        self.assertEqual(detection["contentType"], "image/jpeg")
        self.assertEqual(detection["targetType"], "color")
        self.assertEqual(detection["label"], "red")

        health = state.health_payload()
        self.assertTrue(health["ok"])
        self.assertEqual(health["framesTotal"], 1)
        self.assertEqual(health["detectTotal"], 1)
        self.assertEqual(health["errorTotal"], 0)

    def test_detection_payload_reports_no_frame_before_first_capture(self) -> None:
        state = self.make_state()

        detection = state.detection_payload()

        self.assertFalse(detection["available"])
        self.assertFalse(detection["detected"])
        self.assertEqual(detection["reason"], "no_frame")
        self.assertEqual(detection["alignment"], "LOST")

    def test_handler_routes_detection_health_and_frame_paths(self) -> None:
        state = self.make_state()
        frame = make_jpeg_with_red_target()
        with patch.object(service, "fetch_bytes", return_value=(frame, "image/jpeg")):
            state.run_once()

        class FakeServer:
            allowed_origins = {"*"}

            def __init__(self, state: PerceptionState) -> None:
                self.state = state

            def config_payload(self) -> dict:
                return {"ok": True}

        class RecordingHandler(PerceptionHandler):
            def send_json(self, payload, status=200, *, allow_cross_origin=False):  # type: ignore[no-untyped-def]
                self.recorded = ("json", status, payload, allow_cross_origin)

            def handle_vision_frame(self) -> None:
                self.recorded = ("frame", self.server.state.frame_payload())

            def send_error_json(self, status, error):  # type: ignore[no-untyped-def]
                self.recorded = ("error", status, error)

        handler = RecordingHandler.__new__(RecordingHandler)
        handler.server = FakeServer(state)

        handler.path = "/api/detection"
        handler.do_GET()
        self.assertEqual(handler.recorded[0], "json")
        self.assertTrue(handler.recorded[2]["detected"])

        handler.path = "/health"
        handler.do_GET()
        self.assertTrue(handler.recorded[2]["ok"])

        handler.path = "/api/vision_frame?t=1"
        handler.do_GET()
        self.assertEqual(handler.recorded, ("frame", (frame, "image/jpeg")))

    def test_cors_reflects_wildcard_origin_for_read_only_endpoints(self) -> None:
        handler = PerceptionHandler.__new__(PerceptionHandler)
        handler.server = type("FakeServer", (), {"allowed_origins": {"*"}})()
        handler.headers = {"Origin": "http://motionbrain.local"}
        sent: list[tuple[str, str]] = []
        handler.send_header = lambda key, value: sent.append((key, value))  # type: ignore[method-assign]

        handler.send_cross_origin_headers()

        self.assertIn(("Access-Control-Allow-Origin", "*"), sent)


class DashboardPerceptionProxyTest(unittest.TestCase):
    def make_server(self) -> DashboardServer:
        server = DashboardServer.__new__(DashboardServer)
        server.perception_url = "http://perception.local:8766"
        server.camera_url = ""
        server.detect_color = "red"
        server.timeout = 1.0
        return server

    def test_get_detection_uses_perception_service_when_configured(self) -> None:
        server = self.make_server()
        payload = {"available": True, "detected": True, "label": "cup"}

        with patch.object(dashboard, "fetch_json", return_value=payload) as fetch_json:
            detection = server.get_detection()

        self.assertEqual(detection, payload)
        fetch_json.assert_called_once_with("http://perception.local:8766/api/detection", 1.0)

    def test_get_camera_frame_uses_perception_service_when_configured(self) -> None:
        server = self.make_server()
        frame = b"jpeg"

        with patch.object(dashboard, "fetch_bytes", return_value=(frame, "image/jpeg")) as fetch_bytes:
            result = server.get_camera_frame()

        self.assertEqual(result, (frame, "image/jpeg"))
        fetch_bytes.assert_called_once_with("http://perception.local:8766/api/vision_frame", 1.0)

    def test_capture_handler_allows_perception_only_mode(self) -> None:
        server = self.make_server()
        frame = b"jpeg"

        class RecordingHandler(dashboard.DashboardHandler):
            def send_response(self, status):  # type: ignore[no-untyped-def]
                self.status = status

            def send_header(self, key, value):  # type: ignore[no-untyped-def]
                self.headers_sent.append((key, value))

            def end_headers(self) -> None:
                self.headers_done = True

            def send_error_json(self, status, error):  # type: ignore[no-untyped-def]
                self.error = (status, error)

        handler = RecordingHandler.__new__(RecordingHandler)
        handler.server = server
        handler.headers_sent = []
        handler.wfile = type("Writer", (), {"write": lambda self, value: setattr(self, "body", value)})()

        with patch.object(server, "get_camera_frame", return_value=(frame, "image/jpeg")):
            handler.handle_capture()

        self.assertEqual(handler.status, 200)
        self.assertNotIn("error", handler.__dict__)
        self.assertEqual(handler.wfile.body, frame)
        self.assertIn(("Content-Type", "image/jpeg"), handler.headers_sent)


if __name__ == "__main__":
    unittest.main()
