import unittest
import urllib.error
from types import SimpleNamespace
from unittest.mock import patch

from tools import motionbrain_dashboard as dashboard
from tools.motionbrain_dashboard import DashboardServer
from tools.motionbrain_dashboard import build_dashboard_policy_proposal
from tools.motionbrain_dashboard import build_grasp_dry_run_plan
from tools.motionbrain_dashboard import dependency_error_payload


SOURCE_OBSERVATION = {
    "sourceFrameId": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa:10",
    "cameraBootId": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
    "capturedUs": 10,
    "detectionSourceFrameId": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa:10",
    "sourceClockDomain": "camera_boot_us",
    "sourceTimestampSemantics": "first_dma_buffer",
    "sourceTimeKnown": True,
    "sourceMetadataValid": True,
    "sourceAgeMs": 20.0,
    "sourceFresh": True,
    "controlEligible": True,
    "liveDetected": True,
}


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

    def test_dashboard_policy_proposal_holds_while_idle_without_execution(self) -> None:
        proposal = build_dashboard_policy_proposal(
            {
                "state": "IDLE",
                "motorEnabled": False,
                "sensor": {"blocked": False, "faultLatched": False},
                "baseAngle": {"active": False},
                "motors": {},
            },
            {
                "available": True,
                "detected": True,
                "fresh": True,
                **SOURCE_OBSERVATION,
                "held": False,
                "label": "cup",
                "confidence": 0.8,
                "alignment": "CENTER",
            },
            instruction="center cup",
        )

        self.assertEqual(proposal["action"], "hold")
        self.assertEqual(proposal["reason"], "state_not_armed")
        self.assertFalse(proposal["physicalMotionCandidate"])
        self.assertFalse(proposal["executionAvailable"])

    def test_dashboard_policy_proposal_exposes_confirmed_dry_run_candidate(self) -> None:
        proposal = build_dashboard_policy_proposal(
            {
                "state": "ARMED",
                "sensor": {"blocked": False, "faultLatched": False},
                "baseAngle": {"active": False},
                "motors": {},
            },
            {
                "available": True,
                "detected": True,
                "fresh": True,
                **SOURCE_OBSERVATION,
                "held": False,
                "label": "cup",
                "confidence": 0.8,
                "alignment": "CENTER",
            },
            instruction="center cup",
        )

        self.assertEqual(proposal["action"], "cup_grasp_plan")
        self.assertTrue(proposal["requiresOperatorConfirm"])
        self.assertFalse(proposal["physicalMotionCandidate"])
        self.assertFalse(proposal["executionAvailable"])

    def test_dashboard_policy_uses_policy_threshold_for_low_confidence_cup(self) -> None:
        proposal = build_dashboard_policy_proposal(
            {
                "state": "ARMED",
                "sensor": {"blocked": False, "faultLatched": False},
                "baseAngle": {"active": False},
                "motors": {},
            },
            {
                "available": True,
                "detected": True,
                "fresh": True,
                **SOURCE_OBSERVATION,
                "held": False,
                "label": "cup",
                "confidence": 0.4,
                "alignment": "RIGHT",
            },
            instruction="center cup",
            min_confidence=dashboard.POLICY_MIN_CONFIDENCE,
        )

        self.assertEqual(proposal["action"], "ask_operator")
        self.assertEqual(proposal["reason"], "confidence_below_threshold")
        self.assertFalse(proposal["physicalMotionCandidate"])

    def fresh_source_detection(self):
        return {**SOURCE_OBSERVATION, "available": True, "detected": True, "fresh": True,
                "held": False, "label": "cup", "confidence": 0.8, "alignment": "LEFT"}

    def test_policy_rejects_held_unknown_and_stale_original_even_when_display_fresh(self):
        status = {"state": "ARMED", "sensor": {}, "baseAngle": {}, "motors": {}}
        valid = self.fresh_source_detection()
        self.assertEqual(build_dashboard_policy_proposal(status, valid, instruction="align target")["action"], "align_left")
        invalid = [
            {**valid, "held": True}, {**valid, "sourceTimeKnown": False},
            {**valid, "sourceFresh": False}, {**valid, "controlEligible": False},
            {**valid, "sourceAgeMs": 2001}, {**valid, "sourceAgeMs": float("nan")},
            {**valid, "detectionSourceFrameId": "other"},
            {key: value for key, value in valid.items() if key not in SOURCE_OBSERVATION},
        ]
        for detection in invalid:
            with self.subTest(detection=detection):
                proposal = build_dashboard_policy_proposal(status, detection, instruction="align target")
                self.assertEqual(proposal["action"], "hold")
                self.assertFalse(proposal["physicalMotionCandidate"])

    def align_handler(self, detection):
        handler = dashboard.DashboardHandler.__new__(dashboard.DashboardHandler)
        handler.server = SimpleNamespace(
            http_token="token", camera_url="", perception_url="http://perception",
            get_detection=lambda: detection, motion_base_url="http://controller", timeout=1,
            status_allows_align_nudge=lambda status: (True, "ok"), align_percent=25, align_nudge_ms=100)
        handler.read_json_body = lambda: {"alignment": "LEFT"}
        handler.responses = []
        handler.send_error_json = lambda status, error: handler.responses.append((int(status), error))
        handler.send_json = lambda result: handler.responses.append((200, result))
        return handler

    def test_align_endpoint_never_executes_on_missing_held_or_stale_source(self):
        valid = self.fresh_source_detection()
        for detection in ({}, {**valid, "held": True}, {**valid, "sourceFresh": False},
                          {**valid, "sourceTimeKnown": False}):
            handler = self.align_handler(detection)
            with patch.object(dashboard, "execute_base_nudge") as execute:
                handler.handle_align_nudge()
            execute.assert_not_called()
            self.assertEqual(handler.responses, [(409, "alignment_source_not_eligible")])

    def test_align_endpoint_rechecks_source_age_after_status_request(self):
        handler = self.align_handler(self.fresh_source_detection())
        with (patch.object(dashboard.time, "monotonic", side_effect=[10.0, 12.1]),
              patch.object(dashboard, "fetch_json", return_value={}),
              patch.object(dashboard, "execute_base_nudge") as execute):
            handler.handle_align_nudge()
        execute.assert_not_called()
        self.assertEqual(handler.responses, [(409, "alignment_source_expired")])

    def test_align_endpoint_accepts_valid_original_source(self):
        handler = self.align_handler(self.fresh_source_detection())
        with (patch.object(dashboard, "fetch_json", return_value={}),
              patch.object(dashboard, "execute_base_nudge", return_value={"success": True}) as execute):
            handler.handle_align_nudge()
        execute.assert_called_once()
        self.assertEqual(handler.responses[0][0], 200)

    def test_dashboard_perception_transport_age_is_counted(self):
        server = self.make_server(); server.perception_url = "http://perception"; server.timeout = 1
        with (patch.object(dashboard, "fetch_json", return_value=self.fresh_source_detection()),
              patch.object(dashboard.time, "monotonic", side_effect=[10.0, 10.1])):
            detection = server.get_detection()
        self.assertAlmostEqual(detection["sourceAgeMs"], 120.0)
        self.assertTrue(detection["controlEligible"])
        with (patch.object(dashboard, "fetch_json", return_value=self.fresh_source_detection()),
              patch.object(dashboard.time, "monotonic", side_effect=[10.0, 12.1])):
            detection = server.get_detection()
        self.assertFalse(detection["controlEligible"])

    def test_dependency_error_payload_marks_controller_unavailable_as_degraded(self) -> None:
        payload = dependency_error_payload(
            "controller",
            "http://motionbrain.local/status",
            urllib.error.URLError("Temporary failure in name resolution"),
            last_success_at=123.5,
        )

        self.assertFalse(payload["ok"])
        self.assertTrue(payload["degraded"])
        self.assertTrue(payload["serviceReady"])
        self.assertFalse(payload["motionReady"])
        self.assertEqual(payload["dependency"], "controller")
        self.assertEqual(payload["dependencyUrl"], "http://motionbrain.local/status")
        self.assertEqual(payload["error"], "controller_unavailable")
        self.assertEqual(payload["errorClass"], "name_resolution_failed")
        self.assertEqual(payload["lastSuccessfulAt"], 123.5)

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

    def test_grasp_dry_run_plan_blocks_display_held_detection(self) -> None:
        held = {
            "detected": True,
            "held": True,
            "targetType": "object",
            "label": "cup",
            "confidence": 0.8,
            "alignment": "CENTER",
        }

        plan = build_grasp_dry_run_plan(held, target_label="cup", min_confidence=0.5)

        self.assertFalse(plan["ok"])
        self.assertEqual(plan["error"], "held_detection")


if __name__ == "__main__":
    unittest.main()
