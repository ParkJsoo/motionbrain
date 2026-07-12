import contextlib
import io
import json
import os
import sys
import threading
import unittest
import urllib.error
import urllib.request
from types import SimpleNamespace
from unittest.mock import patch

from tools import motionbrain_dashboard as dashboard


class DashboardSecurityTest(unittest.TestCase):
    @contextlib.contextmanager
    def running_server(
        self,
        *,
        dashboard_token: str = "dashboard-secret-123456",
        http_token: str = "controller-secret",
        cors_origins: set[str] | None = None,
    ):
        server = dashboard.DashboardServer(
            ("127.0.0.1", 0),
            dashboard.DashboardHandler,
            "http://controller.local",
            "",
            "",
            "red",
            0.5,
            12,
            http_token,
            dashboard_token,
            cors_origins or set(),
            250,
            25,
            "cup",
            0.5,
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            yield server
        finally:
            server.shutdown()
            thread.join(timeout=2.0)
            server.server_close()

    def request_json(
        self,
        server: dashboard.DashboardServer,
        path: str,
        *,
        method: str = "GET",
        headers: dict[str, str] | None = None,
        payload: dict | None = None,
    ) -> tuple[int, dict[str, str], dict]:
        data = None
        request_headers = dict(headers or {})
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            request_headers["Content-Type"] = "application/json"
        url = f"http://127.0.0.1:{server.server_address[1]}{path}"
        request = urllib.request.Request(url, method=method, headers=request_headers, data=data)
        try:
            with urllib.request.urlopen(request, timeout=2.0) as response:
                return response.status, dict(response.headers), json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, dict(exc.headers), json.loads(exc.read().decode("utf-8"))

    def test_config_never_exposes_dashboard_token_and_only_allows_configured_cors(self) -> None:
        with self.running_server(cors_origins={"http://allowed.example"}) as server:
            status, headers, payload = self.request_json(server, "/api/config")
            self.assertEqual(status, 200)
            self.assertNotIn("dashboardToken", payload)
            self.assertTrue(payload["dashboardAuthConfigured"])
            self.assertTrue(payload["dashboardAuthRequired"])
            self.assertNotIn("Access-Control-Allow-Origin", headers)

            status, headers, payload = self.request_json(
                server,
                "/api/config",
                headers={"Origin": "http://allowed.example"},
            )
            self.assertEqual(status, 200)
            self.assertNotIn("dashboardToken", payload)
            self.assertEqual(headers["Access-Control-Allow-Origin"], "http://allowed.example")

            status, headers, payload = self.request_json(
                server,
                "/api/config",
                headers={"Origin": "http://not-allowed.example"},
            )
            self.assertEqual(status, 200)
            self.assertNotIn("dashboardToken", payload)
            self.assertNotIn("Access-Control-Allow-Origin", headers)

    def test_post_requires_dashboard_token_before_proxying(self) -> None:
        with self.running_server() as server:
            with patch.object(dashboard, "post_motionbrain") as post_motionbrain:
                status, _headers, payload = self.request_json(
                    server,
                    "/api/light",
                    method="POST",
                    payload={"action": "toggle"},
                )

        self.assertEqual(status, 403)
        self.assertEqual(payload["error"], "dashboard_auth_required")
        post_motionbrain.assert_not_called()

    def test_policy_proposal_get_is_read_only_and_never_proxies_post(self) -> None:
        status_payload = {
            "state": "IDLE",
            "sensor": {"blocked": False, "faultLatched": False},
            "baseAngle": {"active": False},
            "motors": {},
        }
        detection_payload = {
            "available": True,
            "detected": True,
            "fresh": True,
            "held": False,
            "label": "cup",
            "confidence": 0.8,
            "alignment": "CENTER",
        }
        with self.running_server() as server:
            server.get_detection = lambda: detection_payload  # type: ignore[method-assign]
            with (
                patch.object(dashboard, "fetch_json", return_value=status_payload),
                patch.object(dashboard, "post_motionbrain") as post_motionbrain,
            ):
                status, _headers, payload = self.request_json(
                    server,
                    "/api/policy_proposal?instruction=center%20cup",
                )

        self.assertEqual(status, 200)
        self.assertEqual(payload["action"], "hold")
        self.assertFalse(payload["executionAvailable"])
        post_motionbrain.assert_not_called()

    def test_policy_endpoint_does_not_reuse_lower_grasp_threshold(self) -> None:
        status_payload = {
            "state": "ARMED",
            "sensor": {"blocked": False, "faultLatched": False},
            "baseAngle": {"active": False},
            "motors": {},
        }
        detection_payload = {
            "available": True,
            "detected": True,
            "fresh": True,
            "held": False,
            "label": "cup",
            "confidence": 0.4,
            "alignment": "RIGHT",
        }
        with self.running_server() as server:
            server.grasp_min_confidence = 0.25
            server.get_detection = lambda: detection_payload  # type: ignore[method-assign]
            with patch.object(dashboard, "fetch_json", return_value=status_payload):
                status, _headers, payload = self.request_json(
                    server,
                    "/api/policy_proposal?instruction=center%20cup",
                )

        self.assertEqual(status, 200)
        self.assertEqual(payload["action"], "ask_operator")
        self.assertEqual(payload["reason"], "confidence_below_threshold")

    def test_policy_proposal_rejects_oversized_instruction_without_fetching(self) -> None:
        with self.running_server() as server:
            with patch.object(dashboard, "fetch_json") as fetch:
                status, _headers, payload = self.request_json(
                    server,
                    f"/api/policy_proposal?instruction={'x' * 241}",
                )

        self.assertEqual(status, 400)
        self.assertEqual(payload["error"], "instruction_too_long")
        fetch.assert_not_called()

    def test_empty_dashboard_token_rejects_post_fail_closed(self) -> None:
        with self.running_server(dashboard_token="") as server:
            with patch.object(dashboard, "post_motionbrain") as post_motionbrain:
                status, _headers, payload = self.request_json(
                    server,
                    "/api/light",
                    method="POST",
                    headers={"X-Dashboard-Token": "anything"},
                    payload={"action": "toggle"},
                )

        self.assertEqual(status, 403)
        self.assertEqual(payload["error"], "dashboard_auth_required")
        post_motionbrain.assert_not_called()

    def test_controller_token_is_required_for_light_proxy(self) -> None:
        with self.running_server(http_token="") as server:
            with patch.object(dashboard, "post_motionbrain") as post_motionbrain:
                status, _headers, payload = self.request_json(
                    server,
                    "/api/light",
                    method="POST",
                    headers={"X-Dashboard-Token": "dashboard-secret-123456"},
                    payload={"action": "toggle"},
                )

        self.assertEqual(status, 403)
        self.assertEqual(payload["error"], "http_token_required")
        post_motionbrain.assert_not_called()

    def test_valid_tokens_allow_light_proxy_without_exposing_dashboard_token(self) -> None:
        with self.running_server() as server:
            with patch.object(dashboard, "post_motionbrain", return_value={"success": True, "message": "light toggle"}) as post:
                status, _headers, payload = self.request_json(
                    server,
                    "/api/light",
                    method="POST",
                    headers={"X-Dashboard-Token": "dashboard-secret-123456"},
                    payload={"action": "toggle"},
                )

        self.assertEqual(status, 200)
        self.assertTrue(payload["success"])
        self.assertEqual(payload["requestedAction"], "toggle")
        post.assert_called_once_with("http://controller.local", "/light?action=toggle", 0.5, "controller-secret")

    def test_log_message_redacts_query_parameters(self) -> None:
        handler = dashboard.DashboardHandler.__new__(dashboard.DashboardHandler)
        handler.path = "/api/events?dashboardToken=secret&limit=12"
        handler.command = "POST"
        handler.address_string = lambda: "127.0.0.1"  # type: ignore[method-assign]

        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            handler.log_message('"%s" %s %s', "POST /api/events?dashboardToken=secret HTTP/1.1", "403", "-")

        output = stderr.getvalue()
        self.assertIn("/api/events?<redacted>", output)
        self.assertNotIn("secret", output)

    def test_parse_args_keeps_loopback_default_and_rejects_unsafe_public_bind(self) -> None:
        with patch.dict(os.environ, {}, clear=True), patch.object(sys, "argv", ["dashboard"]):
            args = dashboard.parse_args()

        self.assertEqual(args.host, "127.0.0.1")
        self.assertEqual(args.dashboard_token, "")
        self.assertEqual(args.cors_origins, set())

        with patch.dict(os.environ, {}, clear=True), patch.object(sys, "argv", ["dashboard", "--host", "0.0.0.0"]):
            with self.assertRaises(SystemExit):
                dashboard.parse_args()

        with patch.dict(os.environ, {}, clear=True), patch.object(
            sys,
            "argv",
            ["dashboard", "--host", "0.0.0.0", "--dashboard-token", "dashboard-secret-123456"],
        ):
            args = dashboard.parse_args()

        self.assertEqual(args.host, "0.0.0.0")
        self.assertEqual(args.dashboard_token, "dashboard-secret-123456")

    def test_dashboard_cors_origins_keep_legacy_controller_origin(self) -> None:
        origins = dashboard.dashboard_cors_origins(
            "http://motionbrain.local:80",
            {"http://allowed.example/"},
        )

        self.assertIn("http://motionbrain.local", origins)
        self.assertIn("http://motionbrain.local:80", origins)
        self.assertIn("http://allowed.example", origins)

    def test_run_logs_auth_state_without_token_value(self) -> None:
        args = SimpleNamespace(
            host="127.0.0.1",
            port=8765,
            motion_host="192.168.4.1",
            motion_port=80,
            camera_url="",
            perception_url="",
            detect_color="red",
            timeout=2.0,
            events_limit=12,
            http_token="controller-secret",
            dashboard_token="dashboard-secret-123456",
            cors_origins=set(),
            align_nudge_ms=250,
            align_percent=25,
            grasp_target_label="cup",
            grasp_min_confidence=0.5,
        )

        class FakeServer:
            def __init__(self, *args, **kwargs):  # type: ignore[no-untyped-def]
                pass

            def serve_forever(self):  # type: ignore[no-untyped-def]
                raise KeyboardInterrupt

            def server_close(self):  # type: ignore[no-untyped-def]
                pass

        stdout = io.StringIO()
        with patch.object(dashboard, "DashboardServer", FakeServer), contextlib.redirect_stdout(stdout):
            self.assertEqual(dashboard.run(args), 0)

        output = stdout.getvalue()
        self.assertIn("dashboard_auth=enabled", output)
        self.assertNotIn("dashboard_token=", output)
        self.assertNotIn("dashboard-secret-123456", output)


if __name__ == "__main__":
    unittest.main()
