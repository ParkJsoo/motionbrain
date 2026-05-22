import unittest
from unittest.mock import patch

from tools import motionbrain_dashboard as dashboard
from tools.motionbrain_dashboard import DashboardServer


class DashboardAlignmentTest(unittest.TestCase):
    def make_server(self) -> DashboardServer:
        return DashboardServer.__new__(DashboardServer)

    def test_status_allows_align_nudge_only_when_armed_clear_and_base_idle(self) -> None:
        server = self.make_server()
        clear = {"state": "ARMED", "sensor": {"blocked": False, "faultLatched": False}, "baseAngle": {"active": False}}
        self.assertEqual(server.status_allows_align_nudge(clear), (True, "ok"))

        blocked = {"state": "ARMED", "sensor": {"blocked": True, "blockReason": "SENSOR_STALE"}, "baseAngle": {}}
        self.assertEqual(server.status_allows_align_nudge(blocked), (False, "SENSOR_STALE"))

        fault = {"state": "ARMED", "sensor": {"faultLatched": True, "faultReason": "VIBRATION"}, "baseAngle": {}}
        self.assertEqual(server.status_allows_align_nudge(fault), (False, "VIBRATION"))

        idle = {"state": "IDLE", "sensor": {"blocked": False}, "baseAngle": {}}
        self.assertEqual(server.status_allows_align_nudge(idle), (False, "state_IDLE"))

        busy = {"state": "ARMED", "sensor": {"blocked": False}, "baseAngle": {"active": True}}
        self.assertEqual(server.status_allows_align_nudge(busy), (False, "base_busy"))

    def test_execute_base_nudge_stops_after_successful_start(self) -> None:
        calls: list[str] = []

        def fake_post(_base_url: str, path: str, _timeout: float, _token: str) -> dict:
            calls.append(path)
            if "action=stop" in path:
                return {"success": True, "message": "base stop"}
            return {"success": True, "message": "base right at 25%"}

        with patch.object(dashboard, "post_motionbrain", side_effect=fake_post), patch.object(dashboard.time, "sleep"):
            result = dashboard.execute_base_nudge("http://motionbrain", "right", 25, 250, 2.0, "token")

        self.assertTrue(result["success"])
        self.assertTrue(result["stopped"])
        self.assertEqual(
            calls,
            ["/joint?joint=base&action=right&percent=25", "/joint?joint=base&action=stop"],
        )

    def test_execute_base_nudge_attempts_stop_if_start_response_fails(self) -> None:
        calls: list[str] = []

        def fake_post(_base_url: str, path: str, _timeout: float, _token: str) -> dict:
            calls.append(path)
            if "action=stop" in path:
                return {"success": True, "message": "base stop"}
            raise OSError("connection reset")

        with patch.object(dashboard, "post_motionbrain", side_effect=fake_post):
            with self.assertRaises(OSError):
                dashboard.execute_base_nudge("http://motionbrain", "left", 25, 250, 2.0, "token")

        self.assertEqual(
            calls,
            ["/joint?joint=base&action=left&percent=25", "/joint?joint=base&action=stop"],
        )


if __name__ == "__main__":
    unittest.main()
