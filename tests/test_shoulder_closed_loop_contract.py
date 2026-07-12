import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class ShoulderClosedLoopContractTest(unittest.TestCase):
    def test_sensor_contract_uses_i2c_and_rejects_bad_magnet_state(self):
        header = (ROOT / "src" / "peripheral" / "shoulder_angle_sensor.h").read_text()
        source = (ROOT / "src" / "peripheral" / "shoulder_angle_sensor.cpp").read_text()

        for fragment in {
            "MOTIONBRAIN_SHOULDER_I2C_SDA_PIN 0",
            "MOTIONBRAIN_SHOULDER_I2C_SCL_PIN 15",
            "I2C_SDA_PIN = MOTIONBRAIN_SHOULDER_I2C_SDA_PIN",
            "I2C_SCL_PIN = MOTIONBRAIN_SHOULDER_I2C_SCL_PIN",
            "I2C_ADDRESS = 0x36",
            "MOUNT_OFFSET_DEGREES = -24.35f",
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
        self.assertIn("getI2cRawDegrees() + MOUNT_OFFSET_DEGREES", source)

    def test_controller_has_bounded_motion_and_stop_reasons(self):
        header = (ROOT / "src" / "control" / "shoulder_angle_controller.h").read_text()
        source = (ROOT / "src" / "control" / "shoulder_angle_controller.cpp").read_text()

        expected = {
            "SOFT_MIN_DEGREES = 122.08f",
            "SOFT_MAX_DEGREES = 301.02f",
            "TARGET_TOLERANCE_DEGREES = 0.50f",
            "SETTLED_SUCCESS_TOLERANCE_DEGREES = 0.40f",
            "STOP_THRESHOLD_WINDOW_DEGREES = 0.35f",
            "SENSOR_STALE_MS = 150",
            "COMMAND_TIMEOUT_MS = 10000",
            "PROGRESS_TIMEOUT_MS = 900",
            "UP_STOP_LEAD_DEGREES = 0.90f",
            "DOWN_STOP_LEAD_DEGREES = 1.50f",
            "TARGET_REACHED",
            "SENSOR_FAULT",
            "SAFETY_BLOCK",
            "NO_PROGRESS",
            "SOFT_LIMIT",
            "TARGET_MISSED",
            "UP_CORRECTION_PERCENT = 75",
            "DOWN_CORRECTION_PERCENT = 35",
            "UP_CORRECTION_PULSE_MS = 500",
            "DOWN_CORRECTION_PULSE_MS = 250",
            "MAX_CORRECTION_ATTEMPTS = 6",
        }
        for fragment in expected:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, header + source)

        self.assertIn("hardStop(MotorControl::MOTOR_4)", source)
        self.assertIn("sensor_->isReadyForMotion(SENSOR_STALE_MS)", source)
        self.assertIn("manualDirectionAllowed", source)
        self.assertIn("enforceManualDriveLimits", source)
        self.assertIn("appendShoulderStatusJson", header + source)
        self.assertIn(
            "fabsf(finalErrorDegrees_) <= SETTLED_SUCCESS_TOLERANCE_DEGREES",
            source,
        )
        self.assertIn("beginCorrection(now)", source)
        self.assertIn("SHOULDER_ANGLE_CORRECTION", source)
        self.assertIn("correcting_ ? correctionPercent() : requestedPercent_", source)
        self.assertIn("correctionPulseExpired", source)
        for field in {
            "shoulderAngle",
            "sensorReady",
            "magnetDetected",
            "mountOffsetDeg",
            "manualGuardBlocked",
            "correctionActive",
            "correctionAttempts",
            "maxCorrectionAttempts",
            "targetToleranceDeg",
            "settledSuccessToleranceDeg",
            "stopThresholdWindowDeg",
            "upCorrectionPulseMs",
            "downCorrectionPulseMs",
            "lastStopReason",
        }:
            with self.subTest(status_field=field):
                self.assertIn(field, source)

    def test_http_dashboard_exposes_read_only_shoulder_feedback(self):
        web = (ROOT / "src" / "network" / "web_server.cpp").read_text()
        dashboard = (ROOT / "tools" / "motionbrain_dashboard.py").read_text()

        self.assertIn("extern ShoulderAngleController shoulderAngleController", web)
        self.assertGreaterEqual(web.count("appendShoulderStatusJson"), 2)
        for fragment in {
            "M4 Shoulder Feedback",
            'id="m4Angle"',
            'id="m4Sensor"',
            "status.shoulderAngle",
            "shoulder.magnetDetected",
            "shoulder.manualGuardBlocked",
        }:
            with self.subTest(dashboard_fragment=fragment):
                self.assertIn(fragment, dashboard)

    def test_all_manual_shoulder_paths_cancel_or_block_conflicts(self):
        dispatcher = (ROOT / "src" / "control" / "dispatcher.cpp").read_text()
        teleop = (ROOT / "src" / "input" / "teleop_adapter.cpp").read_text()

        self.assertIn("cancelShoulderAngleIfNeeded", dispatcher)
        self.assertIn('"shoulder motor override"', dispatcher)
        self.assertIn('"shoulder joint override"', dispatcher)
        self.assertIn('"Stop active sequence before shoulder angle control"', dispatcher)
        self.assertIn('"teleop shoulder"', teleop)
        self.assertIn("shoulderAngleController_->cancel", teleop)
        self.assertIn("shoulderAngleController_->manualDirectionAllowed", teleop)
        self.assertIn("SHOULDER_GUARD", teleop)

        sequence = (ROOT / "src" / "motion" / "motion_sequence.cpp").read_text()
        self.assertIn("shoulderAngleController_->manualDirectionAllowed", sequence)

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
