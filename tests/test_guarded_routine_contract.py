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

        self.assertIn("ROUTINE_DRY_RUN", dispatcher)
        self.assertIn("ROUTINE_CONFIRM_REQ", dispatcher)
        self.assertIn("ROUTINE_EXECUTE_BLOCKED", dispatcher)
        self.assertIn("requires operator confirmation code", dispatcher)
        self.assertIn("Routine execute is not implemented in guarded routine v0", dispatcher)

    def test_http_and_serial_expose_same_routine_boundary(self):
        web_server = (ROOT / "src" / "network" / "web_server.cpp").read_text()
        serial = (ROOT / "src" / "input" / "serial_command.cpp").read_text()

        self.assertIn('server_.on("/routine", HTTP_POST', web_server)
        self.assertIn('server_.on("/routine", HTTP_GET', web_server)
        self.assertIn("CommandType::ROUTINE_DRY_RUN", web_server)
        self.assertIn("routineConfirmCode", web_server)
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
        self.assertIn("ROUTINE_CONFIRM_REQ", doc)
        self.assertIn("ROUTINE_EXECUTE_BLOCKED", doc)
        self.assertIn("dryRunOnly", doc)


if __name__ == "__main__":
    unittest.main()
