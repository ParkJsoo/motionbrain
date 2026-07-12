import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BRIDGE_SRC = ROOT / "ros2_ws" / "src" / "motionbrain_ros_bridge"
sys.path.insert(0, str(BRIDGE_SRC))

from motionbrain_ros_bridge.fake_motionbrain_endpoint import base_status_payload  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import M4ContractError  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import M4ConfirmationStore  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import M4WriteConfig  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import ros_rad_from_sensor_deg  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import sensor_deg_from_ros_rad  # noqa: E402
from motionbrain_ros_bridge.m4_write_contract import validate_m4_request  # noqa: E402


def armed_status():
    status = base_status_payload()
    status["state"] = "ARMED"
    return status


def request_for_sensor_deg(sensor_deg: float):
    return {
        "commandId": "cmd-1",
        "joint": "shoulder_pitch_joint",
        "targetPositionRad": ros_rad_from_sensor_deg(sensor_deg),
        "timeoutMs": 5000,
        "confirmId": "confirm-1",
        "mode": "shadow",
    }


class M4WriteContractTest(unittest.TestCase):
    def test_zero_and_round_trip_mapping(self):
        config = M4WriteConfig()
        self.assertAlmostEqual(222.8, sensor_deg_from_ros_rad(0.0, config), places=6)
        for sensor_deg in (122.08, 222.8, 252.15, 301.02):
            self.assertAlmostEqual(
                sensor_deg,
                sensor_deg_from_ros_rad(ros_rad_from_sensor_deg(sensor_deg, config), config),
                places=6,
            )

    def test_non_finite_and_live_soft_limit_targets_are_rejected(self):
        with self.assertRaisesRegex(M4ContractError, "non_finite_target"):
            sensor_deg_from_ros_rad(math.nan, M4WriteConfig())
        with self.assertRaisesRegex(M4ContractError, "target_out_of_range"):
            validate_m4_request(request_for_sensor_deg(122.07), armed_status())
        with self.assertRaisesRegex(M4ContractError, "target_out_of_range"):
            validate_m4_request(request_for_sensor_deg(301.03), armed_status())

    def test_controller_reported_soft_limit_boundaries_are_accepted(self):
        low = validate_m4_request(request_for_sensor_deg(122.08), armed_status())
        high = validate_m4_request(request_for_sensor_deg(301.02), armed_status())
        self.assertAlmostEqual(122.08, low["requestedSensorDeg"], places=6)
        self.assertAlmostEqual(301.02, high["requestedSensorDeg"], places=6)
        self.assertEqual((122.08, 301.02), (low["allowedMinDeg"], low["allowedMaxDeg"]))

    def test_missing_or_invalid_live_soft_limits_fail_closed(self):
        missing = armed_status()
        del missing["shoulderAngle"]["softMinDeg"]
        with self.assertRaisesRegex(M4ContractError, "soft_limits_unavailable"):
            validate_m4_request(request_for_sensor_deg(252.0), missing)
        invalid = armed_status()
        invalid["shoulderAngle"]["softMinDeg"] = 302.0
        with self.assertRaisesRegex(M4ContractError, "soft_limits_invalid"):
            validate_m4_request(request_for_sensor_deg(252.0), invalid)

    def test_state_sensor_and_other_motor_guards(self):
        request = request_for_sensor_deg(237.0)
        with self.assertRaisesRegex(M4ContractError, "state_not_armed"):
            validate_m4_request(request, base_status_payload())
        stale = armed_status()
        stale["shoulderAngle"]["sensorFresh"] = False
        with self.assertRaisesRegex(M4ContractError, "sensor_stale"):
            validate_m4_request(request, stale)
        other = armed_status()
        other["motors"]["M3"]["enabled"] = True
        with self.assertRaisesRegex(M4ContractError, "other_motor_active"):
            validate_m4_request(request, other)

    def test_confirmation_joint_and_timeout_are_required(self):
        request = request_for_sensor_deg(237.0)
        request["confirmId"] = ""
        with self.assertRaisesRegex(M4ContractError, "operator_confirmation_required"):
            validate_m4_request(request, armed_status())
        request = request_for_sensor_deg(237.0)
        request["joint"] = "base_yaw_joint"
        with self.assertRaisesRegex(M4ContractError, "unsupported_joint"):
            validate_m4_request(request, armed_status())
        request = request_for_sensor_deg(237.0)
        request["timeoutMs"] = 100
        with self.assertRaisesRegex(M4ContractError, "invalid_timeout"):
            validate_m4_request(request, armed_status())

    def test_confirmation_is_one_shot_and_bound_to_target(self):
        store = M4ConfirmationStore()
        request = request_for_sensor_deg(237.0)
        confirmation = store.issue(request)
        request["confirmId"] = confirmation["confirmId"]
        store.consume(confirmation["confirmId"], request)
        with self.assertRaisesRegex(M4ContractError, "confirmation_already_consumed"):
            store.consume(confirmation["confirmId"], request)

        changed = request_for_sensor_deg(238.0)
        confirmation = store.issue(changed)
        changed["confirmId"] = confirmation["confirmId"]
        changed["targetPositionRad"] = ros_rad_from_sensor_deg(239.0)
        with self.assertRaisesRegex(M4ContractError, "confirmation_command_mismatch"):
            store.consume(confirmation["confirmId"], changed)

    def test_expired_confirmation_is_rejected(self):
        store = M4ConfirmationStore()
        request = request_for_sensor_deg(237.0)
        confirmation = store.issue(request)
        store.pending[confirmation["confirmId"]]["expiresAt"] = 0.0
        with self.assertRaisesRegex(M4ContractError, "confirmation_expired"):
            store.consume(confirmation["confirmId"], request)


if __name__ == "__main__":
    unittest.main()
