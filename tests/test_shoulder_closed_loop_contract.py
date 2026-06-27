import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class ShoulderClosedLoopContractTest(unittest.TestCase):
    def test_sensor_contract_uses_i2c_and_rejects_bad_magnet_state(self):
        header = (ROOT / "src" / "peripheral" / "shoulder_angle_sensor.h").read_text()
        source = (ROOT / "src" / "peripheral" / "shoulder_angle_sensor.cpp").read_text()

        for fragment in {
            "I2C_SDA_PIN = 0",
            "I2C_SCL_PIN = 15",
            "I2C_ADDRESS = 0x36",
            "I2C_POLL_INTERVAL_MS = 20",
            "isI2cFresh",
            "isMagnetDetected",
            "isMagnetTooWeak",
            "isMagnetTooStrong",
        }:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, header + source)

        self.assertIn("isI2cFresh(maxAgeMs) && isMagnetDetected()", source)
        self.assertIn("!isMagnetTooWeak() && !isMagnetTooStrong()", source)

    def test_controller_has_bounded_motion_and_stop_reasons(self):
        header = (ROOT / "src" / "control" / "shoulder_angle_controller.h").read_text()
        source = (ROOT / "src" / "control" / "shoulder_angle_controller.cpp").read_text()

        expected = {
            "SOFT_MIN_DEGREES = 230.0f",
            "SOFT_MAX_DEGREES = 245.0f",
            "SENSOR_STALE_MS = 150",
            "COMMAND_TIMEOUT_MS = 5000",
            "PROGRESS_TIMEOUT_MS = 900",
            "UP_STOP_LEAD_DEGREES = 0.90f",
            "DOWN_STOP_LEAD_DEGREES = 1.50f",
            "TARGET_REACHED",
            "SENSOR_FAULT",
            "SAFETY_BLOCK",
            "NO_PROGRESS",
            "SOFT_LIMIT",
        }
        for fragment in expected:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, header + source)

        self.assertIn("hardStop(MotorControl::MOTOR_4)", source)
        self.assertIn("sensor_->isReadyForMotion(SENSOR_STALE_MS)", source)

    def test_all_manual_shoulder_paths_cancel_or_block_conflicts(self):
        dispatcher = (ROOT / "src" / "control" / "dispatcher.cpp").read_text()
        teleop = (ROOT / "src" / "input" / "teleop_adapter.cpp").read_text()

        self.assertIn("cancelShoulderAngleIfNeeded", dispatcher)
        self.assertIn('"shoulder motor override"', dispatcher)
        self.assertIn('"shoulder joint override"', dispatcher)
        self.assertIn('"Stop active sequence before shoulder angle control"', dispatcher)
        self.assertIn('"teleop shoulder"', teleop)
        self.assertIn("shoulderAngleController_->cancel", teleop)

    def test_serial_command_is_safety_gated(self):
        serial = (ROOT / "src" / "input" / "serial_command.cpp").read_text()
        safety = (ROOT / "src" / "control" / "safety_gate.cpp").read_text()
        command = (ROOT / "src" / "control" / "command.h").read_text()

        self.assertIn('strcasecmp(cmdName, "shoulder")', serial)
        self.assertIn("shoulder angle <deg> [%%]", serial)
        self.assertIn("CommandType::SHOULDER_ANGLE_RUN", serial)
        self.assertIn("SHOULDER_ANGLE_RUN", command)
        self.assertGreaterEqual(safety.count("CommandType::SHOULDER_ANGLE_RUN"), 2)


if __name__ == "__main__":
    unittest.main()
