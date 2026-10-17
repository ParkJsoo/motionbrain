"""Exercise the installer in temporary directories without sudo or systemd."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest

REPO_ROOT = Path(__file__).resolve().parents[1]
INSTALLER = REPO_ROOT / "tools/raspi/install_systemd_units.sh"
SERVICES = ("perception", "dashboard", "ros-bridge")

# Intercept privileged operations; real install/chmod affect only the sandbox.
FAKE_SUDO = r'''
import json, os, pathlib, subprocess, sys
root = pathlib.Path(os.environ["INSTALL_TEST_ROOT"]).resolve()
cmd, *args = sys.argv[1:]
with (root / "calls.jsonl").open("a") as log:
    log.write(json.dumps([cmd, *args]) + "\n")
if cmd == "systemctl":
    assert args == ["daemon-reload"], args
elif cmd in {"install", "chmod", "chown"}:
    pathlib.Path(args[-1]).resolve().relative_to(root)
    if cmd == "install":
        filtered = []
        i = 0
        while i < len(args):
            if args[i] in {"-o", "-g"}:
                i += 2
            else:
                filtered.append(args[i])
                i += 1
        subprocess.run([os.environ["INSTALL_TEST_BINARY"], *filtered], check=True)
    elif cmd == "chmod":
        os.chmod(args[-1], int(args[0], 8))
else:
    raise AssertionError("Unexpected privileged operation: " + cmd)
'''


class RaspiSystemdInstallTest(unittest.TestCase):
    def setUp(self) -> None:
        temp = tempfile.TemporaryDirectory(prefix="motionbrain-install-")
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.env_dir = self.root / "environment"
        self.units = self.root / "units"
        self.bin_dir = self.root / "bin"
        for path in (self.env_dir, self.units, self.bin_dir):
            path.mkdir()
        sudo = self.bin_dir / "sudo"
        sudo.write_text(f"#!{sys.executable}\n" + FAKE_SUDO)
        sudo.chmod(0o755)
        self.env = dict(os.environ)
        self.env.update(
            PATH=f"{self.bin_dir}{os.pathsep}{os.environ['PATH']}",
            INSTALL_TEST_ROOT=str(self.root),
            INSTALL_TEST_BINARY=shutil.which("install") or "/usr/bin/install",
            MOTIONBRAIN_REPO=str(REPO_ROOT),
            MOTIONBRAIN_SERVICE_USER="install-test",
            MOTIONBRAIN_SERVICE_GROUP="install-test",
            MOTIONBRAIN_SERVICE_HOME=str(self.root / "home"),
            MOTIONBRAIN_ENV_DIR=str(self.env_dir),
            MOTIONBRAIN_SYSTEMD_DIR=str(self.units),
            MOTIONBRAIN_LIBEXEC_DIR=str(self.root / "libexec"),
        )

    def install(self) -> list[list[str]]:
        result = subprocess.run(
            ["bash", str(INSTALLER)], env=self.env, text=True,
            capture_output=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        calls = [json.loads(line) for line in (self.root / "calls.jsonl").read_text().splitlines()]
        self.assertTrue(all(call == ["systemctl", "daemon-reload"]
                            for call in calls if call[0] == "systemctl"))
        return calls

    def test_fresh_install_creates_files_referenced_by_services(self) -> None:
        calls = self.install()
        for service in SERVICES:
            with self.subTest(service=service):
                unit = (self.units / f"motionbrain-{service}.service").read_text()
                declared = next(line.split("=", 1)[1] for line in unit.splitlines()
                                if line.startswith("EnvironmentFile="))
                self.assertEqual(declared, f"/etc/motionbrain/{service}.env")
                target = self.env_dir / Path(declared).name
                self.assertTrue(target.is_file(), str(target))
                self.assertEqual(target.read_bytes(), target.with_suffix(".env.example").read_bytes())
                self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o640)
                self.assertIn(str(REPO_ROOT), target.read_text())
                self.assertFalse((self.env_dir / f"motionbrain-{service}.env").exists())
                self.assertIn(["install", "-o", "root", "-g", "install-test", "-m", "0640",
                               next(c[-2] for c in calls if c[0] == "install" and c[-1] == str(target)),
                               str(target)], calls)
        self.assertEqual(stat.S_IMODE(self.env_dir.stat().st_mode), 0o750)

    def test_existing_canonical_settings_win_over_legacy_and_template(self) -> None:
        for service in SERVICES:
            (self.env_dir / f"{service}.env").write_text("CUSTOM_VALUE=keep-current\n")
            (self.env_dir / f"motionbrain-{service}.env").write_text("CUSTOM_VALUE=keep-legacy\n")
            (self.env_dir / f"{service}.env.example").write_text("old template\n")
        self.install()
        for service in SERVICES:
            target = self.env_dir / f"{service}.env"
            self.assertEqual(target.read_text(), "CUSTOM_VALUE=keep-current\n")
            self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o640)
            self.assertEqual((self.env_dir / f"motionbrain-{service}.env").read_text(),
                             "CUSTOM_VALUE=keep-legacy\n")
            self.assertNotEqual(target.with_suffix(".env.example").read_text(), "old template\n")

    def test_legacy_settings_are_copied_unchanged_and_retained(self) -> None:
        original = b"CUSTOM_VALUE=local-setting\n# Preserve formatting and values.\n"
        for service in SERVICES:
            (self.env_dir / f"motionbrain-{service}.env").write_bytes(original)
        self.install()
        for service in SERVICES:
            self.assertEqual((self.env_dir / f"{service}.env").read_bytes(), original)
            self.assertEqual((self.env_dir / f"motionbrain-{service}.env").read_bytes(), original)
        # A later legacy edit must not replace the already-active configuration.
        for service in SERVICES:
            (self.env_dir / f"motionbrain-{service}.env").write_text("CHANGED=legacy\n")
        self.install()
        for service in SERVICES:
            self.assertEqual((self.env_dir / f"{service}.env").read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
