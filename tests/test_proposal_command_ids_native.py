"""Execute the production ID helper as standard C++; this does not build ROS."""

from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "ros2_ws/src/motionbrain_hardware_interface"


class ProposalCommandIdsNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next(
            (path for name in ("clang++", "g++", "c++") if (path := shutil.which(name))),
            None,
        )
        if compiler is None:
            raise unittest.SkipTest("Native C++ compiler unavailable; ID helper was not executed")
        cls.temp_dir = tempfile.TemporaryDirectory(prefix="motionbrain-proposal-ids-")
        cls.addClassCleanup(cls.temp_dir.cleanup)
        cls.executable = Path(cls.temp_dir.name) / "proposal_ids_test"
        sdk_flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            # Match the selected Xcode SDK; the global CommandLineTools SDK may
            # belong to a different toolchain and be unreadable by its linker.
            sdk = subprocess.run(
                ["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                check=True, capture_output=True, text=True, timeout=30,
            ).stdout.strip()
            sdk_flags = ["-isysroot", sdk]
        compiled = subprocess.run(
            [
                compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                *sdk_flags,
                "-I", str(PACKAGE / "include"),
                str(PACKAGE / "test/test_proposal_command_ids.cpp"),
                "-o", str(cls.executable),
            ],
            capture_output=True, text=True, timeout=60,
        )
        if compiled.returncode:
            raise AssertionError(f"Native C++ compilation failed:\n{compiled.stderr}")

    def test_sequence_session_and_entropy_failure_contract(self):
        subprocess.run([str(self.executable)], check=True, capture_output=True, timeout=10)

    def test_independent_instances_and_processes_receive_distinct_sessions(self):
        ids = []
        for _ in range(2):
            result = subprocess.run(
                [str(self.executable), "random"],
                check=True, capture_output=True, text=True, timeout=10,
            )
            ids.extend(result.stdout.splitlines())
        self.assertEqual(len(ids), 4)
        self.assertEqual(len(set(ids)), 4)
        for command_id in ids:
            self.assertRegex(command_id, r"^ros2-control-[0-9a-f]{32}-1$")


if __name__ == "__main__":
    unittest.main()
