import json
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "motionbrain_mission"))

from motionbrain_mission.mission_flow import MissionFlow, MissionState, parse_command  # noqa: E402


READY_GUARD = json.dumps(
    {
        "ready": True,
        "reason": "ready",
        "suggestedAction": "none",
        "statusFresh": True,
        "detectionFresh": True,
    }
)


class MissionFlowTest(unittest.TestCase):
    def test_start_waits_for_detection(self):
        flow = MissionFlow()
        decision = flow.update_guard_json(READY_GUARD)
        self.assertEqual(MissionState.IDLE, decision.state)

        decision = flow.handle_command("start")
        self.assertEqual(MissionState.WAIT_DETECTION, decision.state)
        self.assertEqual("wait_for_detection", decision.next_step)

    def test_alignment_required_before_confirm(self):
        flow = MissionFlow()
        flow.update_guard_json(READY_GUARD)
        flow.handle_command("start")

        decision = flow.update_detection(
            available=True,
            detected=True,
            alignment="LEFT",
            command_suggestion="base_left",
            area_ratio=0.05,
        )

        self.assertEqual(MissionState.ALIGN, decision.state)
        self.assertEqual("base_left", decision.next_step)

    def test_center_detection_waits_for_operator_confirm(self):
        flow = MissionFlow()
        flow.update_guard_json(READY_GUARD)
        flow.handle_command("start")

        decision = flow.update_detection(
            available=True,
            detected=True,
            alignment="CENTER",
            command_suggestion="hold",
            area_ratio=0.08,
        )

        self.assertEqual(MissionState.WAIT_CONFIRM, decision.state)
        self.assertEqual("operator_confirm", decision.next_step)

    def test_confirm_requests_action_once_ready(self):
        flow = MissionFlow()
        flow.update_guard_json(READY_GUARD)
        flow.handle_command("start")
        flow.update_detection(
            available=True,
            detected=True,
            alignment="CENTER",
            command_suggestion="hold",
            area_ratio=0.08,
        )

        decision = flow.handle_command("confirm")

        self.assertEqual(MissionState.COMPLETE, decision.state)
        self.assertEqual("toggle", decision.act_request)

    def test_guard_blocks_mission(self):
        flow = MissionFlow()
        flow.update_guard_json('{"ready":false,"reason":"faulted"}')

        decision = flow.handle_command("start")

        self.assertEqual(MissionState.BLOCKED, decision.state)
        self.assertEqual("guard_faulted", decision.reason)

    def test_parse_json_command(self):
        self.assertEqual("start", parse_command('{"command":"start"}'))
        self.assertEqual("confirm", parse_command('{"action":"confirm"}'))


if __name__ == "__main__":
    unittest.main()
