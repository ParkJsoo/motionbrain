"""Serialize requested schedule and observed writes without merging their clocks."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FiniteProfileJsonTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            raise unittest.SkipTest("C++ compiler unavailable")
        cls.temp = tempfile.TemporaryDirectory(prefix="motionbrain-finite-json-")
        cls.addClassCleanup(cls.temp.cleanup)
        folder = Path(cls.temp.name)
        (folder / "Arduino.h").write_text(r'''
#pragma once
#include <stdint.h>
#include <string>
#include <type_traits>
class String {
public:
  std::string value;
  String& operator+=(const char* s) { value += s; return *this; }
  String& operator+=(char c) { value += c; return *this; }
  template<class T> typename std::enable_if<std::is_integral<T>::value, String&>::type
  operator+=(T n) { value += std::to_string(n); return *this; }
};
uint32_t millis();
''')
        (folder / "test.cpp").write_text(r'''
#include <iostream>
#include <cstring>
#include "control/finite_profile_json.h"
#include "control/finite_profile_trial_catalog.h"
int main() {
  auto p = finite_profile::trialProfile();
  finite_profile::Record r;
  strcpy(r.request.bootId, "boot1"); strcpy(r.request.commandId, "trial1");
  r.request.expectedEpoch = 1; r.request.profile = p;
  r.state = finite_profile::State::Completed;
  r.reason = finite_profile::Reason::NominalEnd;
  r.plannedStartAt = 1000; r.plannedOffAt = 1350;
  r.firstNonzeroWriteAt = 1003; r.offWriteAt = 1352;
  r.writes[0].present = true; r.writes[0].atMs = 1003; r.writes[0].pwm = 10;
  finite_profile::DriverOutputEvidence e;
  strcpy(e.bootId, "boot1"); strcpy(e.commandId, "trial1");
  e.count = 2; e.totalWrites = 2; e.captureComplete = true; e.closed = true;
  e.entries[0] = {1, 1003, 10, 2, bounded::OutputReason::DriverWrite};
  e.entries[1] = {2, 1004, 10, 2, bounded::OutputReason::DriverWrite};
  String j; finite_profile::appendRecordJson(j, r); std::cout << j.value << "\n";
  String raw; finite_profile::appendEvidenceJson(raw, &e); std::cout << raw.value << "\n";
  String missing; finite_profile::appendEvidenceJson(missing, nullptr); std::cout << missing.value << "\n";
  std::cout << finite_profile::canonicalProfile(p);
}
''')
        sdk = []
        if sys.platform == "darwin":
            path = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
            sdk = ["-isysroot", path]
        command = [compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror", *sdk,
                   "-I", str(folder), "-I", str(ROOT / "tests/native/stubs"), "-I", str(ROOT / "src")]
        command += ["-DMOTIONBRAIN_ENABLE_FINITE_PROFILE_" + flag + "=1"
                    for flag in ("CORE", "DRIVER", "SCHEDULER", "RUNTIME")]
        command += [str(folder / "test.cpp"), *[str(ROOT / "src/control" / name) for name in
                    ("finite_profile_json.cpp", "finite_profile_contract.cpp", "finite_profile_executor.cpp")],
                    "-o", str(folder / "test")]
        result = subprocess.run(command, capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise AssertionError(result.stderr)
        output = subprocess.check_output([str(folder / "test")], text=True).splitlines()
        cls.record, cls.evidence, cls.missing = map(json.loads, output[:3])
        cls.canonical = "\n".join(output[3:]) + "\n"

    def test_requested_schedule_and_actual_writes_remain_distinct(self):
        r = self.record
        self.assertEqual(r["plannedStartAt"], 1000)
        self.assertEqual(r["firstNonzeroWriteAt"], 1003)
        self.assertEqual(r["plannedOffAt"], 1350)
        self.assertEqual(r["offWriteAt"], 1352)
        self.assertEqual(r["plannedEventReceipts"][0]["atMs"], 1003)
        self.assertEqual(r["requestedProfile"]["points"][0], {"offsetMs": 0, "pwm": 10})
        self.assertIsNone(self.missing)
        self.assertNotIn("token", r)

    def test_raw_repeated_writes_keep_sequence_and_identity(self):
        e = self.evidence
        self.assertEqual(e["bootId"], self.record["bootId"])
        self.assertEqual(e["commandId"], self.record["commandId"])
        self.assertEqual(e["evidence"], "driver_write")
        self.assertEqual([w["sequence"] for w in e["entries"]], [1, 2])
        self.assertEqual([w["pwm"] for w in e["entries"]], [10, 10])
        self.assertTrue(e["closed"] and e["captureComplete"])

    def test_fixed_catalog_digest_and_commissioning_limits(self):
        p = self.record["requestedProfile"]
        source = "".join(line + "\n" for line in self.canonical.splitlines()
                         if not line.startswith("catalogHash="))
        self.assertEqual(hashlib.sha256(source.encode()).hexdigest(), p["catalogHash"])
        self.assertEqual(p["motor"], 2)
        self.assertEqual(p["direction"], 1)
        self.assertEqual(p["startLeadMs"], 300)
        self.assertEqual(p["maxLatenessMs"], 20)
        self.assertEqual(p["points"], [{"offsetMs": i * 50, "pwm": pwm}
                         for i, pwm in enumerate([10, 20, 30, 40, 30, 20, 10, 0])])


if __name__ == "__main__":
    unittest.main()
