"""Integration boundaries: fresh safety before dispatch, no UART log flood."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class FiniteLoopContractTest(unittest.TestCase):
    def test_main_delivers_only_after_fresh_input_and_safety(self):
        loop = (ROOT / "src/main.cpp").read_text().split("void loop() {", 1)[1]
        tick = "finite_profile::runtime::update();"
        self.assertEqual(loop.count(tick), 1)
        order = [loop.index(s) for s in (
            "stm32Bridge.update(finiteWindow ? 128 : 0)",
            "teleopAdapter.processInput(finiteWindow ? 128 : 0)",
            "safetyMonitor.update(safetySnapshot)", tick, "webServer.update()")]
        self.assertEqual(order, sorted(order))
        for call in ("serialCommand.update()", "wifiAP.update()", "webServer.update()"):
            self.assertIn("if (!finite_profile::runtime::motionPending()) " + call, loop)
        self.assertIn("if (!finiteWindow) shoulderAngleSensor.update", loop)

    def test_real_debug_log_does_not_write_uart_during_output(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        with tempfile.TemporaryDirectory(prefix="motionbrain-finite-log-") as name:
            folder = Path(name)
            (folder / "control").mkdir()
            (folder / "Arduino.h").write_text(r'''
#pragma once
#include <stdint.h>
#include <cstdarg>
#include <cstdio>
struct FakeSerial {
  unsigned writes = 0;
  void begin(uint32_t) {}
  void println(const char*) { ++writes; }
  void printf(const char*, ...) { ++writes; }
};
extern FakeSerial Serial;
inline void delay(uint32_t) {}
''')
            (folder / "control/finite_profile_runtime.h").write_text(r'''
#pragma once
#define MOTIONBRAIN_ENABLE_FINITE_PROFILE_RUNTIME 1
namespace finite_profile { namespace runtime { bool motionPending(); } }
''')
            (folder / "test.cpp").write_text(r'''
#include <cassert>
#include "debug/debug_log.h"
FakeSerial Serial;
bool pending = false;
namespace finite_profile { namespace runtime { bool motionPending() { return pending; } } }
int main() {
  DebugLog::init();
  const auto before = Serial.writes;
  pending = true;
  for (unsigned i = 0; i < 64; ++i) DebugLog::warn("bad frame %u", i);
  DebugLog::log(LogLevel::ERROR, "error");
  DebugLog::stateChange("ARMED", "IDLE", "stop");
  DebugLog::command("stop", true, "done");
  DebugLog::safety("stop", "done");
  DebugLog::motor("stop", "%s", "done");
  assert(Serial.writes == before);
  assert(DebugLog::suppressedDuringFiniteOutput() == 69);
  pending = false;
  DebugLog::warn("logging restored");
  assert(Serial.writes == before + 1);
}
''')
            sdk = []
            if sys.platform == "darwin":
                path = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
                sdk = ["-isysroot", path]
            result = subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk,
                "-I", str(folder), "-I", str(ROOT / "src"), str(folder / "test.cpp"),
                str(ROOT / "src/debug/debug_log.cpp"), "-o", str(folder / "test")],
                capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stderr)
            subprocess.run([str(folder / "test")], check=True, timeout=10)

if __name__ == "__main__":
    unittest.main()
