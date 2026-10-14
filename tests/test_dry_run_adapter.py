"""Synthetic adapter tests; neither image decoding libraries nor devices required."""
import hashlib
import unittest
from unittest.mock import patch

from learning.dry_run import JPEG_BYTES, SCENARIOS, SyntheticAdapter, VirtualClock
from learning.schema import SCHEMA_VERSION, validate_transition


def collect_synthetic_transition(adapter):
    before = adapter.observe("before")
    action = adapter.execute(adapter.request("synthetic-command:1"))
    adapter.settle(50)
    after = adapter.observe("after")
    for phase, observation in (("before", before), ("after", after)):
        image = adapter.fetch_frame(observation["sourceFrameId"])
        observation["image"] = {"artifactRef": f"images/{phase}.jpg",
                                "sha256": hashlib.sha256(image).hexdigest()}
    return {
        "schemaVersion": SCHEMA_VERSION, "transitionId": "synthetic-transition",
        "sessionId": "synthetic-session", "episodeId": "synthetic-episode",
        "synthetic": True, "collectionMode": "dry_run",
        "metadata": {"robotId": "synthetic-robot", "configHash": "a" * 64,
                     "calibrationHash": "b" * 64, "firmwareVersion": "synthetic-firmware",
                     "hostVersion": "synthetic-host", "detectorVersion": "synthetic-detector"},
        "qualityPolicy": {"maxSourceAgeMs": 100, "maxM4AgeMs": 20, "minimumSettlingMs": 50},
        "before": before, "action": action, "after": after, "outcome": {"taskSuccess": None},
    }


class DryRunAdapterTest(unittest.TestCase):
    def test_complete_contract_uses_no_network_or_real_sleep(self):
        with patch("socket.create_connection", side_effect=AssertionError("network forbidden")), \
             patch("urllib.request.urlopen", side_effect=AssertionError("network forbidden")), \
             patch("time.sleep", side_effect=AssertionError("real sleep forbidden")):
            adapter = SyntheticAdapter()
            value = collect_synthetic_transition(adapter)
        report = validate_transition(value)
        self.assertTrue(report["valid"], report)
        for reason in report["exclusionReasons"]:
            self.assertTrue("synthetic" in reason or "dry_run" in reason, report)
        self.assertEqual(adapter.execute_calls, 1)
        self.assertEqual(adapter.mode, "dry_run")
        self.assertAlmostEqual(adapter.clock.monotonic(), 100.192)
        self.assertTrue(value["before"]["synthetic"])
        self.assertTrue(value["before"]["m4"]["synthetic"])
        self.assertTrue(value["action"]["outputLog"]["synthetic"])

    def test_original_jpeg_is_stable_and_exact_frame_lookup_never_falls_back(self):
        self.assertEqual(len(JPEG_BYTES), 286)
        self.assertEqual(hashlib.sha256(JPEG_BYTES).hexdigest(),
                         "c944873d37463dd4fedec83fb0f81b630359cb08814c254427a03589ceea56be")
        self.assertTrue(JPEG_BYTES.startswith(b"\xff\xd8\xff\xe0"))
        self.assertTrue(JPEG_BYTES.endswith(b"\xff\xd9"))
        # Baseline SOF0 encodes 8-bit precision, 2px high, 2px wide, 3 components.
        self.assertIn(b"\xff\xc0\x00\x11\x08\x00\x02\x00\x02\x03", JPEG_BYTES)
        adapter = SyntheticAdapter()
        first, second = adapter.observe("before"), adapter.observe("after")
        self.assertNotEqual(first["sourceFrameId"], second["sourceFrameId"])
        self.assertEqual(adapter.fetch_frame(first["sourceFrameId"]), JPEG_BYTES)
        self.assertEqual(first["imageSha256"], second["imageSha256"])
        with self.assertRaises(FileNotFoundError):
            adapter.fetch_frame("unknown-frame")
        # Caller mutation cannot replace stored source bytes.
        first["imageSha256"] = "corrupted"
        self.assertEqual(adapter.fetch_frame(first["sourceFrameId"]), JPEG_BYTES)

    def test_missing_observations_and_after_image_are_explicit(self):
        for phase in ("before", "after"):
            with self.subTest(phase=phase):
                adapter = SyntheticAdapter(f"{phase}_missing")
                with self.assertRaises(FileNotFoundError):
                    adapter.observe(phase)
                self.assertEqual(adapter.execute_calls, 0)
        adapter = SyntheticAdapter("after_image_missing")
        before, after = adapter.observe("before"), adapter.observe("after")
        self.assertEqual(adapter.fetch_frame(before["sourceFrameId"]), JPEG_BYTES)
        with self.assertRaises(FileNotFoundError):
            adapter.fetch_frame(after["sourceFrameId"])

    def test_hash_faults_preserve_original_bytes(self):
        for phase in ("before", "after"):
            with self.subTest(phase=phase):
                adapter = SyntheticAdapter(f"{phase}_hash_mismatch")
                observation = adapter.observe(phase)
                pixels = adapter.fetch_frame(observation["sourceFrameId"])
                self.assertEqual(pixels, JPEG_BYTES)
                self.assertNotEqual(observation["imageSha256"], hashlib.sha256(pixels).hexdigest())

    def test_unknown_and_interruption_do_not_retry_or_claim_terminal_output(self):
        for scenario, exception in (("unknown_result", TimeoutError),
                                    ("interrupt_during_action", KeyboardInterrupt)):
            with self.subTest(scenario=scenario):
                adapter = SyntheticAdapter(scenario)
                with self.assertRaises(exception):
                    adapter.execute(adapter.request("synthetic-command:1"))
                self.assertEqual(adapter.execute_calls, 1)
                self.assertAlmostEqual(adapter.clock.monotonic(), 100.05)

    def test_dropped_trace_is_preserved_and_excluded(self):
        value = collect_synthetic_transition(SyntheticAdapter("dropped_outputs"))
        self.assertEqual(value["action"]["outputLog"]["dropped"], 1)
        report = validate_transition(value)
        self.assertTrue(report["valid"], report)
        self.assertIn("output_history_dropped", report["exclusionReasons"])

    def test_stale_before_and_boot_sessions_are_explicit(self):
        adapter = SyntheticAdapter("stale_before")
        before = adapter.observe("before")
        self.assertFalse(before["sourceFresh"])
        self.assertFalse(before["controlEligible"])
        self.assertEqual(before["sourceAgeMs"], 5000)
        other = SyntheticAdapter()
        self.assertNotEqual(adapter.host_clock_id, other.host_clock_id)
        self.assertNotEqual(adapter.camera_boot_id, other.camera_boot_id)
        self.assertNotEqual(adapter.mcu_boot_id, other.mcu_boot_id)

    def test_output_ordinals_sequences_and_request_copy(self):
        adapter = SyntheticAdapter()
        request = adapter.request("first")
        first = adapter.execute(request)
        request["pwm"] = 99
        self.assertEqual(first["request"]["pwm"], 30)
        second = adapter.execute(adapter.request("second"))
        self.assertEqual(first["record"]["ordinal"], 1)
        self.assertEqual(second["record"]["ordinal"], 2)
        self.assertEqual([x["sequence"] for x in second["outputLog"]["entries"]], [3, 4])
        self.assertEqual(second["outputLog"]["entries"][-1]["pwm"], 0)

    def test_scenarios_and_virtual_clock_inputs_are_validated(self):
        self.assertEqual(len(SCENARIOS), 10)
        with self.assertRaises(ValueError):
            SyntheticAdapter("unsupported")
        clock = VirtualClock()
        self.assertEqual(clock.monotonic(), 100)
        for bad in (-1, float("inf"), float("nan"), True, "1"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                clock.advance(bad)
        adapter = SyntheticAdapter()
        for bad in (-1, float("inf"), float("nan"), True, "1"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                adapter.settle(bad)
        with self.assertRaises(ValueError):
            adapter.observe("latest")


if __name__ == "__main__":
    unittest.main()
