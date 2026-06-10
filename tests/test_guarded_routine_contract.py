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
        }:
            with self.subTest(name=name):
                self.assertIn(f'"{name}"', source)

    def test_routine_execute_is_explicitly_dry_run_only(self):
        dispatcher = (ROOT / "src" / "control" / "dispatcher.cpp").read_text()
        executor_header = (ROOT / "src" / "control" / "guarded_routine_executor.h").read_text()
        executor_source = (ROOT / "src" / "control" / "guarded_routine_executor.cpp").read_text()

        self.assertIn("ROUTINE_DRY_RUN", dispatcher)
        self.assertIn("ROUTINE_CONFIRM_REQ", dispatcher)
        self.assertIn("ROUTINE_PREFLIGHT_BLOCK", dispatcher)
        self.assertIn("ROUTINE_EXECUTE_BLOCKED", dispatcher)
        self.assertIn("requires operator confirmation code", dispatcher)
        self.assertIn("preflight blocked", dispatcher)
        self.assertIn("GuardedRoutineExecutor::begin", dispatcher)
        self.assertIn("Routine executor blocked", dispatcher)
        self.assertIn("#define MOTIONBRAIN_ROUTINE_EXECUTOR_ENABLED 0", executor_header)
        self.assertIn("routine executor disabled by firmware policy", executor_source)
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
        self.assertIn("noActiveSequence", web_server)
        self.assertIn('strcasecmp(cmdName, "routine")', serial)
        self.assertIn("routine dry-run <name>", serial)
        self.assertIn("confirm=confirm-inspect", serial)

    def test_message_interface_documents_dry_run_contract(self):
        doc = (ROOT / "MESSAGE_INTERFACE.md").read_text()

        self.assertIn("POST /routine?action=dry_run&name=inspect", doc)
        self.assertIn("POST /routine?action=run&name=inspect&confirm=confirm-inspect", doc)
        self.assertIn("executeImplemented", doc)
        self.assertIn("operatorConfirmation", doc)
        self.assertIn("executionPolicy", doc)
        self.assertIn('"executor"', doc)
        self.assertIn('"skeleton_disabled_by_default"', doc)
        self.assertIn('"sequenceStarted": false', doc)
        self.assertIn("executor.result=disabled", doc)
        self.assertIn("state_not_armed", doc)
        self.assertIn("motion_blocked", doc)
        self.assertIn("sequence_active", doc)
        self.assertIn("ROUTINE_CONFIRM_REQ", doc)
        self.assertIn("ROUTINE_PREFLIGHT_BLOCK", doc)
        self.assertIn("ROUTINE_EXECUTE_BLOCKED", doc)
        self.assertIn("dryRunOnly", doc)


if __name__ == "__main__":
    unittest.main()
