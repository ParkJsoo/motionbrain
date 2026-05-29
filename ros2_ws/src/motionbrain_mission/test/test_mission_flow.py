import json
import unittest

from motionbrain_mission.mission_flow import MissionFlow
from motionbrain_mission.mission_flow import MissionState
from motionbrain_mission.mission_flow import parse_command


READY_GUARD = json.dumps(
    {
        "ready": True,
        "reason": "ready",
        "suggestedAction": "none",
        "statusFresh": True,
        "detectionFresh": True,
    }
)


class MissionFlowPackageTest(unittest.TestCase):
    def test_start_waits_for_detection(self):
        flow = MissionFlow()
        self.assertEqual(MissionState.IDLE, flow.update_guard_json(READY_GUARD).state)

        decision = flow.handle_command("start")

        self.assertEqual(MissionState.WAIT_DETECTION, decision.state)
        self.assertEqual("wait_for_detection", decision.next_step)

    def test_alignment_blocks_confirm_until_centered(self):
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

        ignored = flow.handle_command("confirm")
        self.assertEqual(MissionState.ALIGN, ignored.state)
        self.assertIsNone(ignored.act_request)

    def test_center_detection_requires_operator_confirm_before_action(self):
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
        self.assertIsNone(decision.act_request)

        confirmed = flow.handle_command("confirm")
        self.assertEqual(MissionState.COMPLETE, confirmed.state)
        self.assertEqual("toggle", confirmed.act_request)

    def test_guard_fault_blocks_start(self):
        flow = MissionFlow()
        flow.update_guard_json('{"ready":false,"reason":"faulted"}')

        decision = flow.handle_command("start")

        self.assertEqual(MissionState.BLOCKED, decision.state)
        self.assertEqual("guard_faulted", decision.reason)
        self.assertEqual("operator_check", decision.next_step)

    def test_parse_command_supports_raw_and_json_commands(self):
        self.assertEqual("start", parse_command("start"))
        self.assertEqual("confirm", parse_command('{"command":"confirm"}'))
        self.assertEqual("reset", parse_command('{"action":"reset"}'))
        self.assertEqual("", parse_command('{"command":'))


if __name__ == "__main__":
    unittest.main()
