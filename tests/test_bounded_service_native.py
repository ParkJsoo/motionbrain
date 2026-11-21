"""Actual MCU service/executor/driver integration, with fake peripheral adapters.

Compiles both default-disabled and opt-in variants; no firmware or device runs.
"""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class BoundedServiceNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = next((p for n in ("clang++", "g++", "c++") if (p := shutil.which(n))), None)
        if compiler is None:
            raise unittest.SkipTest("C++ compiler unavailable; MCU service not executed")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-bounded-service-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.binaries = {}
        sdk_flags = []
        if sys.platform == "darwin" and shutil.which("xcrun"):
            sdk = subprocess.run(["xcrun", "--sdk", "macosx", "--show-sdk-path"],
                                 check=True, capture_output=True, text=True, timeout=30).stdout.strip()
            sdk_flags = ["-isysroot", sdk]
        for enabled in (False, True, "m2", "dual"):
            binary = Path(cls.temp.name) / (str(enabled) if enabled else "default")
            flags = ["-DMOTIONBRAIN_ENABLE_BOUNDED_ACTIONS=1"] if enabled else []
            if enabled == "m2": flags += ["-DMOTIONBRAIN_M2_VISUAL_PILOT=1"]
            if enabled == "dual": flags += ["-DMOTIONBRAIN_DUAL_VIEW_TIMER=1"]
            command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk_flags, *flags]
            for path in ("tests/native/service_stubs", "tests/native/stubs", "src"):
                command += ["-I", str(ROOT / path)]
            if enabled == "dual":
                command += ["-I", str(ROOT/"tests/native/whole_arm_stubs"),
                            str(ROOT/"src/motor/bounded_pwm_deadline.cpp")]
            command += [str(ROOT / path) for path in (
                "tests/native/test_bounded_service.cpp", "src/control/bounded_action_service.cpp",
                "src/control/bounded_action_executor.cpp", "src/motor/motor_driver.cpp")]
            result = subprocess.run([*command, "-o", str(binary)], capture_output=True, text=True, timeout=60)
            if result.returncode:
                raise AssertionError(f"MCU service integration compilation failed:\n{result.stderr}")
            cls.binaries[enabled] = binary

    def run_case(self, case, enabled=True):
        result = subprocess.run([str(self.binaries[enabled]), case],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return [json.loads(line) for line in result.stdout.splitlines()]

    def test_dual_timer_cuts_pwm_without_control_loop_and_owns_stop_reason(self):
        for motor in range(1, 6):
            for cause in ("deadline", "stalled", "manual", "cancel"):
                with self.subTest(motor=motor, cause=cause):
                    status, record, output = self.run_case(f"dual-{motor}-{cause}", enabled="dual")
                    self.assertEqual(status["profile"], "dual_view_timer_v1")
                    self.assertEqual(record["reason"], {
                        "deadline": "deadline_exceeded", "stalled": "state_changed",
                        "manual": "operator_override", "cancel": "cancelled"}[cause])
                    self.assertTrue(record["outputOff"])
                    if cause in ("deadline", "stalled"):
                        edges = [e for e in output["entries"] if e["motor"] == motor and
                                 e["actionOrdinal"] == record["ordinal"]]
                        self.assertTrue(any(e["pwm"] != 0 for e in edges))
                        zero = next(i for i, e in enumerate(edges) if e["pwm"] == 0)
                        self.assertEqual(edges[zero]["atMs"], 100 if cause == "deadline" else 150)
                        self.assertTrue(all(e["pwm"] == 0 for e in edges[zero:]))

    def test_m2_profile_refuses_other_axes_direction_and_excess_budget(self):
        status, record, output = self.run_case("pilot", enabled="m2")
        self.assertEqual(status["profile"], "m2_up_pwm180_ms1500")
        self.assertEqual(record["motor"], 2)
        self.assertEqual(record["requestedPwm"], 180)
        self.assertTrue(record["outputOff"])
        self.assertEqual(record["elapsedMs"], 1500)

    def test_default_build_blocks_before_any_output_or_lease_mutation(self):
        status, = self.run_case("disabled", enabled=False)
        self.assertFalse(status["enabled"])
        self.assertEqual(status["recordCount"], 0)

    def test_real_driver_pulse_deadline_counts_stop_latency_and_resets_context(self):
        status, record, output = self.run_case("pulse")
        self.assertTrue(status["enabled"])
        self.assertFalse(status["active"])
        self.assertEqual(record["state"], "timed_out")
        self.assertEqual(record["overrunMs"], 15)
        self.assertEqual(output["evidence"], "driver_write")
        self.assertEqual(output["clockDomain"], "controller_boot_millis")
        self.assertEqual(output["entries"][5]["pwm"], 10)
        self.assertEqual(output["entries"][6]["pwm"], 0)
        self.assertEqual(output["entries"][6]["actionOrdinal"], record["ordinal"])
        self.assertEqual(output["entries"][11]["actionOrdinal"], 0)
        self.assertIsNone(record["taskSuccess"])
        self.assertIsNone(record["visuallySettled"])

    def test_existing_controller_and_teleop_ownership_blocks_then_override_cancels(self):
        _, record, _ = self.run_case("guards")
        self.assertEqual(record["reason"], "operator_override")
        self.assertTrue(record["outputOff"])

    def test_shoulder_delegates_to_controller_and_reports_target_separately(self):
        _, record, output = self.run_case("shoulder")
        self.assertEqual(record["targetDegrees"], 45)
        self.assertTrue(record["targetReached"])
        self.assertTrue(record["outputOff"])
        self.assertIsNone(record["taskSuccess"])
        self.assertEqual(output["entries"][-1]["pwm"], 0)

    def test_unknown_latch_keeps_emergency_stop_available_and_rejects_motor_run(self):
        status, record, _ = self.run_case("unknown")
        self.assertTrue(status["active"])
        self.assertEqual(record["state"], "outcome_unknown")
        self.assertTrue(record["outputOff"])


if __name__ == "__main__":
    unittest.main()
