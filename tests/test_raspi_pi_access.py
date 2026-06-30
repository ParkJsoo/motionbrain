from __future__ import annotations

import importlib.util
import os
import unittest
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "tools" / "raspi" / "check_pi_ssh_target.py"
SPEC = importlib.util.spec_from_file_location("check_pi_ssh_target", SCRIPT_PATH)
assert SPEC is not None
check_pi_ssh_target = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(check_pi_ssh_target)


class RaspiPiAccessTest(unittest.TestCase):
    def test_ssh_config_parser_extracts_key_fields(self) -> None:
        parsed = check_pi_ssh_target.parse_ssh_config(
            """
            user motionbrain
            hostname motionbrain-pi.davolink
            hostkeyalias motionbrain-pi.local
            addressfamily inet
            """
        )

        self.assertEqual(parsed["user"], "motionbrain")
        self.assertEqual(parsed["hostname"], "motionbrain-pi.davolink")
        self.assertEqual(parsed["hostkeyalias"], "motionbrain-pi.local")
        self.assertEqual(parsed["addressfamily"], "inet")

    def test_static_ip_detection_flags_dhcp_literals(self) -> None:
        self.assertTrue(check_pi_ssh_target.is_ip_literal("192.168.219.110"))
        self.assertFalse(check_pi_ssh_target.is_ip_literal("motionbrain-pi.davolink"))
        self.assertFalse(check_pi_ssh_target.is_ip_literal("motionbrain-pi.local"))

    def test_tcp_connect_treats_netcat_timeout_as_unreachable(self) -> None:
        with (
            mock.patch.object(
                check_pi_ssh_target.shutil,
                "which",
                return_value="/usr/bin/nc",
            ),
            mock.patch.object(
                check_pi_ssh_target,
                "run_command",
                side_effect=check_pi_ssh_target.subprocess.TimeoutExpired(
                    ["nc"],
                    1.0,
                ),
            ),
        ):
            self.assertFalse(
                check_pi_ssh_target.tcp_connect("motionbrain-pi.local", 22, 1.0)
            )

    def test_script_defaults_to_pi_hostnames(self) -> None:
        self.assertEqual(
            ("motionbrain-pi.local", "motionbrain-pi.davolink"),
            check_pi_ssh_target.DEFAULT_CANDIDATES,
        )
        self.assertTrue(os.access(SCRIPT_PATH, os.R_OK))

    def test_operations_docs_have_pi_ssh_recovery_contract(self) -> None:
        operations_text = (REPO_ROOT / "OPERATIONS.md").read_text()

        required_fragments = [
            "Pi Access / SSH",
            "check_pi_ssh_target.py",
            "HostName motionbrain-pi.local",
            "stale `192.168.219.110`",
            "motionbrain.local` is the ESP32 motion controller",
            "not a Pi SSH discovery tool",
            "ssh.socket",
            "SSH alias reaches old IP",
            "router-DNS fallback",
        ]
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, operations_text)


if __name__ == "__main__":
    unittest.main()
