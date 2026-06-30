import json
import threading
import unittest
import urllib.error
import urllib.request

from motionbrain_ros_bridge.fake_motionbrain_endpoint import make_server
from motionbrain_ros_bridge.fake_motionbrain_endpoint import strip_ros_args


class FakeEndpointServer:
    def __init__(self, scenario: str, *, delay_sec: float = 0.1) -> None:
        self.server = make_server(
            "127.0.0.1",
            0,
            scenario=scenario,
            delay_sec=delay_sec,
            quiet=True,
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def __enter__(self) -> "FakeEndpointServer":
        self.thread.start()
        return self

    def __exit__(self, *_exc: object) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    @property
    def base_url(self) -> str:
        host, port = self.server.server_address
        return f"http://{host}:{port}"

    def get_json(self, path: str, *, timeout: float = 1.0) -> dict:
        with urllib.request.urlopen(f"{self.base_url}{path}", timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))

    def post_json(self, path: str, *, timeout: float = 1.0) -> dict:
        request = urllib.request.Request(f"{self.base_url}{path}", method="POST")
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))


class FakeMotionBrainEndpointTest(unittest.TestCase):
    def test_ready_scenario_serves_status_routine_events_and_detection(self):
        with FakeEndpointServer("ready") as server:
            status = server.get_json("/status")
            routine = server.get_json("/routine")
            events = server.get_json("/events?limit=1")
            detection = server.get_json("/api/detection")

        self.assertEqual("IDLE", status["state"])
        self.assertFalse(status["motorEnabled"])
        self.assertTrue(status["shoulderAngle"]["sensorReady"])
        self.assertTrue(routine["dryRunOnly"])
        self.assertFalse(routine["feedback"]["physicalRoutineExecutionAllowed"])
        self.assertEqual(1, len(events["events"]))
        self.assertTrue(detection["available"])

    def test_stale_shoulder_scenario_marks_sensor_unready(self):
        with FakeEndpointServer("stale_shoulder") as server:
            shoulder = server.get_json("/status")["shoulderAngle"]

        self.assertTrue(shoulder["available"])
        self.assertFalse(shoulder["sensorFresh"])
        self.assertFalse(shoulder["sensorReady"])
        self.assertEqual("SENSOR_STALE", shoulder["lastStopReason"])

    def test_controller_fault_scenario_latches_fault_without_motion(self):
        with FakeEndpointServer("controller_fault") as server:
            status = server.get_json("/status")

        self.assertEqual("FAULT", status["state"])
        self.assertFalse(status["motorEnabled"])
        self.assertTrue(status["sensor"]["faultLatched"])
        self.assertEqual("FAKE_CONTROLLER_FAULT", status["sensor"]["faultReason"])
        self.assertEqual("clear_fault", status["recovery"]["action"])

    def test_policy_mismatch_scenario_exposes_unsafe_routine_state(self):
        with FakeEndpointServer("policy_mismatch") as server:
            routine = server.get_json("/routine")

        self.assertTrue(routine["feedback"]["physicalRoutineExecutionAllowed"])
        self.assertFalse(routine["feedback"]["readyForRoutineExecution"])
        self.assertTrue(routine["executor"]["queueApplyAllowed"])

    def test_stale_detection_scenario_marks_detection_unavailable(self):
        with FakeEndpointServer("stale_detection") as server:
            detection = server.get_json("/api/detection")

        self.assertFalse(detection["available"])
        self.assertFalse(detection["detected"])
        self.assertEqual("LOST", detection["alignment"])
        self.assertEqual("fault injection: stale detection", detection["reason"])
        self.assertGreaterEqual(detection["ageMs"], 60000)

    def test_malformed_status_scenario_returns_invalid_json_only_for_status(self):
        with FakeEndpointServer("malformed_status") as server:
            with self.assertRaises(json.JSONDecodeError):
                server.get_json("/status")
            routine = server.get_json("/routine")

        self.assertTrue(routine["dryRunOnly"])

    def test_timeout_status_scenario_delays_only_status(self):
        with FakeEndpointServer("timeout_status", delay_sec=0.2) as server:
            with self.assertRaises((TimeoutError, urllib.error.URLError)):
                server.get_json("/status", timeout=0.05)
            routine = server.get_json("/routine")

        self.assertTrue(routine["dryRunOnly"])

    def test_routine_post_is_read_only_and_never_forwarded(self):
        with FakeEndpointServer("ready") as server:
            result = server.post_json("/routine?action=run&name=inspect")

        self.assertFalse(result["success"])
        self.assertFalse(result["forwarded"])
        self.assertEqual("fake_endpoint_read_only", result["error"])

    def test_unknown_path_returns_404_json(self):
        with FakeEndpointServer("ready") as server:
            with self.assertRaises(urllib.error.HTTPError) as raised:
                server.get_json("/missing")

        self.assertEqual(404, raised.exception.code)

    def test_cli_ignores_ros_launch_arguments(self):
        self.assertEqual(
            ["--host", "127.0.0.1", "--port", "8767"],
            strip_ros_args(
                [
                    "--host",
                    "127.0.0.1",
                    "--port",
                    "8767",
                    "--ros-args",
                    "-r",
                    "__node:=motionbrain_fake_endpoint",
                ]
            ),
        )


if __name__ == "__main__":
    unittest.main()
