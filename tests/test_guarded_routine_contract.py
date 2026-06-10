import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class GuardedRoutineContractTest(unittest.TestCase):
    def test_firmware_defines_expected_routine_names(self):
        source = (ROOT / "src" / "control" / "guarded_routine.cpp").read_text()

        for name in {
            "inspect",
            "open_gripper_check",
            "stow",
            "center_target_dry_run",
            "soft_home_reference",
        }:
            with self.subTest(name=name):
                self.assertIn(f'"{name}"', source)
        self.assertIn("isSoftHomeAlias", source)
        self.assertIn("confirm-soft-home-reference", source)
        self.assertIn("software reference", source)
        self.assertIn("no encoder-grade homing", source)

    def test_routine_execute_is_explicitly_dry_run_only(self):
        dispatcher = (ROOT / "src" / "control" / "dispatcher.cpp").read_text()
        executor_header = (ROOT / "src" / "control" / "guarded_routine_executor.h").read_text()
        executor_source = (ROOT / "src" / "control" / "guarded_routine_executor.cpp").read_text()

        self.assertIn("ROUTINE_DRY_RUN", dispatcher)
        self.assertIn("ROUTINE_CONFIRM_REQ", dispatcher)
        self.assertIn("ROUTINE_PREFLIGHT_BLOCK", dispatcher)
        self.assertIn("ROUTINE_EXECUTE_BLOCKED", dispatcher)
        self.assertIn("prepare=%s applied=%u started=%u motion=%u", dispatcher)
        self.assertIn("requires operator confirmation code", dispatcher)
        self.assertIn("preflight blocked", dispatcher)
        self.assertIn("GuardedRoutineExecutor::begin", dispatcher)
        self.assertIn("GuardedRoutineExecutor::abort", dispatcher)
        self.assertIn("Routine executor blocked", dispatcher)
        self.assertIn("ROUTINE_ABORT", dispatcher)
        self.assertIn("ROUTINE_ABORT_IDLE", dispatcher)
        self.assertIn("#define MOTIONBRAIN_ROUTINE_EXECUTOR_ENABLED 0", executor_header)
        self.assertIn("routine executor disabled by firmware policy", executor_source)
        self.assertIn("GuardedRoutineExecutorState::RUNNING", executor_source)
        self.assertIn("enum class GuardedRoutineStepResult", executor_header)
        self.assertIn("PENDING = 0", executor_header)
        self.assertIn("SKIPPED", executor_header)
        self.assertIn("BLOCKED", executor_header)
        self.assertIn("buildStepJournal", executor_source)
        self.assertIn("buildPreparedSequence", executor_source)
        self.assertIn("sequence candidate ready; not applied to MotionSequence", executor_source)
        self.assertIn("AngleController::MIN_TARGET_DEGREES", executor_source)
        self.assertIn("PREPARE_READY", executor_header)
        self.assertIn("PREPARE_TOO_MANY_STEPS", executor_header)
        self.assertIn("PREPARE_INVALID_STEP", executor_header)
        self.assertIn("motion step blocked by executor policy", executor_source)
        self.assertIn("motion step skipped before executor", executor_source)
        self.assertIn("routine executor timeout", executor_source)
        self.assertIn("ROUTINE_EXECUTOR_TIMEOUT", executor_source)
        self.assertIn("no active routine executor to abort", executor_source)
        self.assertIn("sequenceStarted(false)", executor_source)
        self.assertIn("executeImplemented() {\n  return false;", executor_source)

    def test_http_and_serial_expose_same_routine_boundary(self):
        web_server = (ROOT / "src" / "network" / "web_server.cpp").read_text()
        serial = (ROOT / "src" / "input" / "serial_command.cpp").read_text()

        self.assertIn('server_.on("/routine", HTTP_POST', web_server)
        self.assertIn('server_.on("/routine", HTTP_GET', web_server)
        self.assertIn("CommandType::ROUTINE_DRY_RUN", web_server)
        self.assertIn("routineConfirmCode", web_server)
        self.assertIn("GuardedRoutine::evaluateExecutePreflight", web_server)
        self.assertIn("GuardedRoutineExecutor::appendReportJson", web_server)
        self.assertIn("GuardedRoutineExecutor::appendPolicyJson", web_server)
        self.assertIn("CommandType::ROUTINE_ABORT", web_server)
        self.assertIn("noActiveSequence", web_server)
        self.assertIn('strcasecmp(cmdName, "routine")', serial)
        self.assertIn("routine dry-run <name>", serial)
        self.assertIn("routine status", serial)
        self.assertIn("routine abort", serial)
        self.assertIn("confirm=confirm-inspect", serial)
        self.assertIn("soft_home_reference", serial)

    def test_message_interface_documents_dry_run_contract(self):
        doc = (ROOT / "MESSAGE_INTERFACE.md").read_text()

        self.assertIn("POST /routine?action=dry_run&name=inspect", doc)
        self.assertIn("POST /routine?action=run&name=inspect&confirm=confirm-inspect", doc)
        self.assertIn("executeImplemented", doc)
        self.assertIn("operatorConfirmation", doc)
        self.assertIn("executionPolicy", doc)
        self.assertIn('"executor"', doc)
        self.assertIn('"skeleton_disabled_by_default"', doc)
        self.assertIn('"abortSupported": true', doc)
        self.assertIn('"timeoutSupported": true', doc)
        self.assertIn('"stepJournal"', doc)
        self.assertIn('"preparedSequence"', doc)
        self.assertIn('"appliedToMotionSequence": false', doc)
        self.assertIn('"preparedStepCount"', doc)
        self.assertIn('"preparedMotionCount"', doc)
        self.assertIn('"prepareResult": "ready"', doc)
        self.assertIn('"result": "skipped"', doc)
        self.assertIn("pending|skipped|blocked", doc)
        self.assertIn('"sequenceStarted": false', doc)
        self.assertIn("executor.result=disabled", doc)
        self.assertIn("POST /routine?action=abort", doc)
        self.assertIn("ROUTINE_ABORT_IDLE", doc)
        self.assertIn("state_not_armed", doc)
        self.assertIn("motion_blocked", doc)
        self.assertIn("sequence_active", doc)
        self.assertIn("ROUTINE_CONFIRM_REQ", doc)
        self.assertIn("ROUTINE_PREFLIGHT_BLOCK", doc)
        self.assertIn("ROUTINE_EXECUTE_BLOCKED", doc)
        self.assertIn("ROUTINE_EXECUTOR_TIMEOUT", doc)
        self.assertIn("prepare=<prepareResult>", doc)
        self.assertIn("system|safety|base_angle|teleop|routine", doc)
        self.assertIn("soft_home_reference", doc)
        self.assertIn("motion step", doc)
        self.assertIn("hard-stop seeking", doc)
        self.assertIn("dryRunOnly", doc)

    def test_homing_feedback_plan_documents_current_boundary(self):
        doc = (ROOT / "docs" / "HOMING_FEEDBACK_PLAN.md").read_text()
        pin_map = (ROOT / "PIN_MAP.md").read_text()
        message = (ROOT / "MESSAGE_INTERFACE.md").read_text()

        self.assertIn("does not currently support true automatic homing", doc)
        self.assertIn("soft_home_reference", doc)
        self.assertIn("zero motion", doc)
        self.assertIn("Limit switch", doc)
        self.assertIn("Hall sensor", doc)
        self.assertIn("Absolute magnetic encoder", doc)
        self.assertIn("Read-only telemetry", doc)
        self.assertIn("STM32 sensor/teleop node", doc)
        self.assertIn("I2C GPIO expander", doc)
        self.assertIn("I2C mux", doc)
        self.assertIn("HOMING_FEEDBACK_PLAN.md", pin_map)
        self.assertIn("HOMING_FEEDBACK_PLAN.md", message)


if __name__ == "__main__":
    unittest.main()
