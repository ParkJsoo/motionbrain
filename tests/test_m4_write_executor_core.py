import unittest

from tests.test_m4_write_contract import armed_status
from motionbrain_ros_bridge.m4_write_contract import ros_rad_from_sensor_deg
from motionbrain_ros_bridge.m4_write_executor_core import M4WriteExecutorCore


def proposal(command_id="proposal-1", target=250.0):
    return {
        "commandId": command_id,
        "joint": "shoulder_pitch_joint",
        "targetPositionRad": ros_rad_from_sensor_deg(target),
        "timeoutMs": 10000,
        "forwarded": False,
        "operatorConfirmationRequired": True,
    }


class M4WriteExecutorCoreTest(unittest.TestCase):
    def test_confirm_forwards_exactly_once(self):
        core = M4WriteExecutorCore()
        core.accept_proposal(proposal())
        paths = []
        result = core.confirm("proposal-1", armed_status, lambda path: paths.append(path) or {"success": True})
        self.assertTrue(result["success"])
        self.assertTrue(result["forwarded"])
        self.assertEqual(1, len(paths))
        self.assertIn("degrees=250.000000", paths[0])
        duplicate = core.confirm("proposal-1", armed_status, lambda path: paths.append(path) or {"success": True})
        self.assertEqual("proposal_already_consumed", duplicate["reason"])
        self.assertEqual(1, len(paths))

    def test_idle_and_stale_fail_closed_without_post(self):
        for mutation, reason in ((lambda s: s.update(state="IDLE"), "state_not_armed"),
                                 (lambda s: s["shoulderAngle"].update(sensorFresh=False), "sensor_stale")):
            core = M4WriteExecutorCore()
            core.accept_proposal(proposal())
            status = armed_status()
            mutation(status)
            paths = []
            result = core.confirm("proposal-1", lambda: status, lambda path: paths.append(path) or {})
            self.assertEqual(reason, result["reason"])
            self.assertFalse(result["forwarded"])
            self.assertEqual([], paths)

    def test_invalid_boundary_is_rejected(self):
        core = M4WriteExecutorCore()
        bad = proposal()
        bad["forwarded"] = True
        with self.assertRaisesRegex(ValueError, "invalid_proposal_boundary"):
            core.accept_proposal(bad)


if __name__ == "__main__":
    unittest.main()
