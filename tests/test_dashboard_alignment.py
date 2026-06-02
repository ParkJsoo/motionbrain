import unittest
from unittest.mock import patch

from tools import motionbrain_dashboard as dashboard
from tools.motionbrain_dashboard import DashboardServer
from tools.motionbrain_dashboard import build_grasp_dry_run_plan


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
        self.assertTrue(result["startSuccess"])
        self.assertTrue(result["stopSuccess"])
        self.assertEqual(
            calls,
            ["/joint?joint=base&action=right&percent=25", "/joint?joint=base&action=stop"],
        )

    def test_execute_base_nudge_reports_failed_start_even_if_stop_succeeds(self) -> None:
        calls: list[str] = []

        def fake_post(_base_url: str, path: str, _timeout: float, _token: str) -> dict:
            calls.append(path)
            if "action=stop" in path:
                return {"success": True, "message": "base stop"}
            return {"success": False, "message": "Blocked by safety"}

        with patch.object(dashboard, "post_motionbrain", side_effect=fake_post), patch.object(dashboard.time, "sleep") as sleep:
            result = dashboard.execute_base_nudge("http://motionbrain", "left", 25, 250, 2.0, "token")

        sleep.assert_not_called()
        self.assertFalse(result["ok"])
        self.assertFalse(result["success"])
        self.assertTrue(result["stopped"])
        self.assertFalse(result["startSuccess"])
        self.assertTrue(result["stopSuccess"])
        self.assertEqual(
            calls,
            ["/joint?joint=base&action=left&percent=25", "/joint?joint=base&action=stop"],
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

    def test_grasp_dry_run_plan_requires_centered_cup(self) -> None:
        detection = {
            "detected": True,
            "targetType": "object",
            "label": "cup",
            "confidence": 0.82,
            "alignment": "CENTER",
        }

        plan = build_grasp_dry_run_plan(detection, target_label="cup", min_confidence=0.5)

        self.assertTrue(plan["ok"])
        self.assertTrue(plan["dryRun"])
        self.assertEqual(plan["target"], "cup")
        self.assertEqual(plan["alignment"], "CENTER")
        self.assertGreaterEqual(len(plan["plannedSequence"]), 4)
        self.assertEqual(plan["plannedSequence"][0]["joint"], "gripper")

    def test_grasp_dry_run_plan_blocks_non_cup_targets(self) -> None:
        detection = {
            "detected": True,
            "targetType": "object",
            "label": "cell phone",
            "confidence": 0.9,
            "alignment": "CENTER",
        }

        plan = build_grasp_dry_run_plan(detection, target_label="cup", min_confidence=0.5)

        self.assertFalse(plan["ok"])
        self.assertEqual(plan["error"], "target_mismatch:cell phone")

    def test_grasp_dry_run_plan_blocks_low_confidence_or_off_center(self) -> None:
        low_confidence = {
            "detected": True,
            "targetType": "object",
            "label": "cup",
            "confidence": 0.3,
            "alignment": "CENTER",
        }
        off_center = {
            "detected": True,
            "targetType": "object",
            "label": "cup",
            "confidence": 0.8,
            "alignment": "LEFT",
        }

        self.assertEqual(
            build_grasp_dry_run_plan(low_confidence, target_label="cup", min_confidence=0.5)["error"],
            "confidence_below_threshold",
        )
        self.assertEqual(
            build_grasp_dry_run_plan(off_center, target_label="cup", min_confidence=0.5)["error"],
            "alignment_not_center:LEFT",
        )


if __name__ == "__main__":
    unittest.main()
