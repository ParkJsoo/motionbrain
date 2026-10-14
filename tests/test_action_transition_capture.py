"""Real local files + synthetic time/output. Never touches a device."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from learning.artifacts import ArtifactStore, StorageError, read_events, verify_artifacts
from learning.dry_run import JPEG_BYTES, SCENARIOS, SyntheticAdapter
from learning.runner import CapturePersistenceError, CaptureRunner, audit_run

ROOT = Path(__file__).resolve().parents[1]


class TransitionCaptureTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)

    def runner(self, scenario="complete", checkpoint=None):
        adapter = SyntheticAdapter(scenario)
        store = ArtifactStore(self.base / scenario)
        return CaptureRunner(store, adapter, checkpoint=checkpoint)

    def test_two_steps_save_original_jpeg_and_immutable_raw_evidence_without_network(self):
        runner = self.runner()
        with patch("socket.socket", side_effect=AssertionError("network forbidden")):
            first = runner.capture(transition_id="first")
            second = runner.capture(transition_id="second")
        self.assertEqual(runner.adapter.execute_calls, 2)
        for result in (first, second):
            self.assertEqual(result["status"], "completed", result)
            self.assertTrue(result["validation"]["valid"])
            self.assertFalse(result["validation"]["trainingEligible"])
            self.assertEqual(result["validation"]["exclusionReasons"], ["synthetic_dry_run"])
            self.assertEqual(result["missing"], [])
            self.assertFalse(result["trainingEligible"])
            self.assertEqual(verify_artifacts(runner.store.root, result["artifacts"]), [])
            for phase in ("before", "after"):
                obs = result["transition"][phase]
                content = (runner.store.root / obs["image"]["artifactRef"]).read_bytes()
                self.assertEqual(content, JPEG_BYTES)
                self.assertEqual(hashlib.sha256(content).hexdigest(), obs["imageSha256"])
                raw = json.loads((runner.store.root / f"raw/{result['transitionId']}/{phase}.json").read_text())
                self.assertNotIn("image", raw)
                self.assertEqual(raw["sourceFrameId"], obs["sourceFrameId"])
        self.assertNotEqual(first["transition"]["before"]["sourceFrameId"], second["transition"]["before"]["sourceFrameId"])
        events, errors = read_events(runner.store.root)
        self.assertEqual(errors, [])
        kinds = [event["kind"] for event in events]
        self.assertLess(kinds.index("action_intent"), kinds.index("action_received"))
        runner.close()
        self.assertTrue(audit_run(runner.store.root)["ok"])

    def test_fault_scenarios_preserve_partial_data_and_halt_without_retry(self):
        cases = {
            "before_missing": (0, "aborted", "not_sent", "before.observation"),
            "after_missing": (1, "aborted", "known_terminal", "after.observation"),
            "after_image_missing": (1, "aborted", "known_terminal", "after.image"),
            "before_hash_mismatch": (0, "aborted", "not_sent", "action.record"),
            "after_hash_mismatch": (1, "aborted", "known_terminal", None),
            "unknown_result": (1, "aborted", "outcome_unknown", "action.record"),
            "dropped_outputs": (1, "excluded", "known_terminal", None),
            "stale_before": (0, "aborted", "not_sent", "action.record"),
            "interrupt_during_action": (1, "interrupted", "outcome_unknown", "action.record"),
        }
        for scenario, (calls, status, execution, missing) in cases.items():
            with self.subTest(scenario=scenario):
                runner = self.runner(scenario)
                result = runner.capture(transition_id="case")
                self.assertEqual(runner.adapter.execute_calls, calls)
                self.assertEqual(result["status"], status, result)
                self.assertEqual(result["executionState"], execution)
                if missing:
                    self.assertIn(missing, result["missing"])
                self.assertFalse(result["trainingEligible"])
                self.assertTrue((runner.store.root / "attempts/case.json").exists())
                with self.assertRaises(RuntimeError):
                    runner.capture()
                self.assertEqual(runner.adapter.execute_calls, calls)

    def test_hash_mismatch_keeps_received_bytes_and_both_hashes(self):
        runner = self.runner("before_hash_mismatch")
        result = runner.capture(transition_id="mismatch")
        image = next(item for item in result["artifacts"] if "hashMatches" in item)
        self.assertFalse(image["hashMatches"])
        self.assertEqual(image["expectedSha256"], "0" * 64)
        self.assertEqual((runner.store.root / image["artifactRef"]).read_bytes(), JPEG_BYTES)
        self.assertIn("image_hash_mismatch", result["errors"][0]["message"])

    def test_operator_interrupt_at_durable_boundaries_keeps_execution_distinction(self):
        for index, (stage, calls, execution) in enumerate((
                ("before_action", 0, "not_sent"), ("action_intent", 0, "outcome_unknown"),
                ("action_received", 1, "known_terminal"), ("settled", 1, "known_terminal"))):
            def interrupt(current):
                if current == stage:
                    raise KeyboardInterrupt("operator stop")
            runner = CaptureRunner(ArtifactStore(self.base / str(index)), SyntheticAdapter(), checkpoint=interrupt)
            result = runner.capture()
            self.assertEqual(result["status"], "interrupted")
            self.assertEqual(result["executionState"], execution)
            self.assertEqual(runner.adapter.execute_calls, calls)
            self.assertIn("after.image", result["missing"])

    def test_exact_frame_mismatch_never_falls_back_to_latest(self):
        runner = self.runner()
        with patch.object(runner.adapter, "fetch_frame", side_effect=FileNotFoundError("evicted")) as fetch:
            result = runner.capture(transition_id="missing")
        fetch.assert_called_once_with(result["transition"]["before"]["sourceFrameId"])
        self.assertEqual(runner.adapter.execute_calls, 0)
        self.assertIn("before.image", result["missing"])

    def test_pre_action_virtual_delay_expires_before_snapshot(self):
        runner = self.runner()
        runner.checkpoint = lambda stage: runner.adapter.clock.advance(1) if stage == "before_action" else None
        result = runner.capture()
        self.assertEqual(runner.adapter.execute_calls, 0)
        self.assertIn("before:source_expired", result["errors"][0]["message"])

    def test_durable_intent_delay_rechecks_freshness_before_adapter_call(self):
        runner = self.runner()
        runner.checkpoint = lambda stage: runner.adapter.clock.advance(5) if stage == "action_intent" else None
        result = runner.capture()
        self.assertEqual(runner.adapter.execute_calls, 0)
        self.assertEqual(result["status"], "aborted")
        self.assertEqual(result["executionState"], "outcome_unknown")

    def test_clock_uncertainty_expired_before_never_executes(self):
        adapter = SyntheticAdapter()
        runner = CaptureRunner(ArtifactStore(self.base / "uncertainty"), adapter,
                               quality_policy={"maxSourceAgeMs": 20.5, "maxM4AgeMs": 20,
                                               "minimumSettlingMs": 50})
        result = runner.capture(transition_id="expired")
        before = result["transition"]["before"]
        self.assertLess(before["sourceAgeMs"], runner.policy["maxSourceAgeMs"])
        self.assertGreater(before["sourceAgeMs"] + before["captureHostBoundsUncertaintySeconds"] * 1000,
                           runner.policy["maxSourceAgeMs"])
        self.assertEqual(adapter.execute_calls, 0)
        self.assertEqual(result["status"], "aborted")
        self.assertEqual(result["executionState"], "not_sent")
        self.assertIn("before:source_expired", result["errors"][0]["message"])
        self.assertEqual((runner.store.root / before["image"]["artifactRef"]).read_bytes(), JPEG_BYTES)
        raw = json.loads((runner.store.root / "raw/expired/before.json").read_text())
        self.assertEqual(raw["captureHostBoundsUncertaintySeconds"], .001)
        events, errors = read_events(runner.store.root)
        self.assertEqual(errors, [])
        self.assertNotIn("action_intent", [event["kind"] for event in events])
        runner.close()
        audit = audit_run(runner.store.root)
        self.assertTrue(audit["ok"], audit)
        self.assertEqual(audit["attempts"][0]["executionState"], "not_sent")

    def test_intent_delay_and_clock_uncertainty_together_block_execution(self):
        adapter = SyntheticAdapter()
        def delay(stage):
            if stage == "action_intent":
                adapter.clock.advance(.002)
        runner = CaptureRunner(ArtifactStore(self.base / "uncertainty-delay"), adapter,
                               checkpoint=delay,
                               quality_policy={"maxSourceAgeMs": 22.5, "maxM4AgeMs": 20,
                                               "minimumSettlingMs": 50})
        result = runner.capture(transition_id="expired")
        self.assertEqual(adapter.execute_calls, 0)
        self.assertEqual(result["status"], "aborted")
        self.assertEqual(result["executionState"], "outcome_unknown")
        self.assertIn("before:source_expired", result["errors"][0]["message"])
        events, errors = read_events(runner.store.root)
        self.assertEqual(errors, [])
        kinds = [event["kind"] for event in events]
        self.assertIn("action_intent", kinds)
        self.assertNotIn("action_received", kinds)
        runner.close()
        self.assertEqual(audit_run(runner.store.root)["attempts"][0]["executionState"], "outcome_unknown")

    def test_finite_uncertainty_at_freshness_limit_allows_one_execution(self):
        for index, uncertainty in enumerate((0, .001)):
            with self.subTest(uncertainty=uncertainty):
                adapter = SyntheticAdapter()
                original = adapter.observe
                def observe(phase):
                    observation = original(phase)
                    observation["captureHostBoundsUncertaintySeconds"] = uncertainty
                    return observation
                runner = CaptureRunner(ArtifactStore(self.base / f"boundary-{index}"), adapter,
                                       quality_policy={"maxSourceAgeMs": 20.000001 + uncertainty * 1000,
                                                       "maxM4AgeMs": 20, "minimumSettlingMs": 50})
                with patch.object(adapter, "observe", side_effect=observe):
                    result = runner.capture()
                self.assertEqual(adapter.execute_calls, 1)
                self.assertEqual(result["status"], "completed", result["errors"])
                self.assertEqual(result["validation"]["exclusionReasons"], ["synthetic_dry_run"])

    def test_missing_or_invalid_clock_uncertainty_cannot_relax_dispatch_gate(self):
        changes = [{"captureHostBoundsUncertaintySeconds": value} for value in (None, -.001, True, "0.001")]
        changes += [{"captureHostBoundsCalibrated": False}]
        for index, change in enumerate(changes):
            with self.subTest(change=change):
                adapter = SyntheticAdapter()
                original = adapter.observe
                def observe(phase):
                    observation = original(phase)
                    observation.update(change)
                    return observation
                runner = CaptureRunner(ArtifactStore(self.base / f"invalid-uncertainty-{index}"), adapter)
                with patch.object(adapter, "observe", side_effect=observe):
                    result = runner.capture()
                self.assertEqual(adapter.execute_calls, 0)
                self.assertEqual(result["status"], "aborted")
                self.assertEqual(result["executionState"], "not_sent")
                self.assertIn("before:source_clock_uncertainty_invalid", result["errors"][0]["message"])

    def test_wrong_command_result_never_establishes_intended_outcome(self):
        runner = self.runner()
        original = runner.adapter.execute
        def wrong(request):
            result = original(request)
            result["record"]["commandId"] = "different-command"
            return result
        with patch.object(runner.adapter, "execute", side_effect=wrong):
            result = runner.capture()
        runner.close()
        self.assertEqual(result["executionState"], "outcome_unknown")
        report = audit_run(runner.store.root)
        self.assertFalse(report["ok"])
        self.assertEqual(report["attempts"][0]["executionState"], "outcome_unknown")
        self.assertIn("action_identity_mismatch_or_intent_missing", report["attempts"][0]["issues"])

    def test_missing_session_terminal_and_partial_image_damage_are_visible(self):
        runner = self.runner()
        result = runner.capture()
        image = runner.store.root / result["transition"]["before"]["image"]["artifactRef"]
        image.unlink()
        report = audit_run(runner.store.root)
        self.assertFalse(report["ok"])
        self.assertIn("session_terminal_missing", report["issues"])
        self.assertTrue(any("before.jpg" in issue for issue in report["attempts"][0]["issues"]))

    def test_write_failure_before_dispatch_never_calls_action(self):
        runner = self.runner()
        with patch.object(runner.store, "save_image", side_effect=StorageError("disk full")):
            result = runner.capture()
        self.assertEqual(runner.adapter.execute_calls, 0)
        self.assertEqual(result["status"], "aborted")
        self.assertEqual(result["errors"][0]["type"], "StorageError")

    def test_failed_intent_write_blocks_dispatch(self):
        runner = self.runner()
        original = runner.store.append
        def append(kind, tid, payload):
            if kind == "action_intent":
                raise StorageError("intent disk failure")
            return original(kind, tid, payload)
        with patch.object(runner.store, "append", side_effect=append):
            result = runner.capture()
        self.assertEqual(runner.adapter.execute_calls, 0)
        self.assertEqual(result["executionState"], "not_sent")

    def test_lost_summary_after_execution_is_reported_and_auditable(self):
        runner = self.runner()
        original = runner.store.write_json
        def write(relative, data):
            if relative.startswith("attempts/"):
                raise StorageError("summary disk failure")
            return original(relative, data)
        with patch.object(runner.store, "write_json", side_effect=write):
            with self.assertRaises(CapturePersistenceError):
                runner.capture(transition_id="partial")
        self.assertEqual(runner.adapter.execute_calls, 1)
        report = audit_run(runner.store.root)
        self.assertFalse(report["ok"])
        self.assertEqual(report["attempts"][0]["executionState"], "known_terminal")
        self.assertIn("attempt_incomplete", report["attempts"][0]["issues"])
        self.assertTrue(runner.halted)

    def test_uncatchable_exit_leaves_durable_intent_and_audit_never_replays(self):
        def crash(stage):
            if stage == "action_intent":
                raise SystemExit(99)
        runner = self.runner(checkpoint=crash)
        with self.assertRaises(SystemExit):
            runner.capture(transition_id="crash")
        with patch.object(SyntheticAdapter, "execute", side_effect=AssertionError("must not replay")):
            report = audit_run(runner.store.root)
        self.assertEqual(report["attempts"][0]["executionState"], "outcome_unknown")
        self.assertFalse(report["ok"])
        self.assertEqual(runner.adapter.execute_calls, 0)

    def test_tampered_or_deleted_images_fail_read_only_audit(self):
        for scenario, tamper in (("modified", True), ("deleted", False)):
            runner = CaptureRunner(ArtifactStore(self.base / scenario), SyntheticAdapter())
            result = runner.capture()
            runner.close()
            image = runner.store.root / result["transition"]["after"]["image"]["artifactRef"]
            image.write_bytes(b"damaged") if tamper else image.unlink()
            report = audit_run(runner.store.root)
            self.assertFalse(report["ok"])
            self.assertTrue(report["attempts"][0]["issues"])

    def test_corrupt_or_missing_journal_does_not_claim_nonexecution(self):
        runner = self.runner()
        runner.capture()
        runner.close()
        (runner.store.root / "events/00000004.json").unlink()
        report = audit_run(runner.store.root)
        self.assertFalse(report["ok"])
        self.assertEqual(report["executionState"], "outcome_unknown")
        self.assertEqual(report["attempts"][0]["executionState"], "outcome_unknown")

    def test_reused_id_and_non_dry_adapter_are_rejected(self):
        runner = self.runner()
        runner.capture(transition_id="once")
        with self.assertRaises(ValueError):
            runner.capture(transition_id="once")
        self.assertEqual(runner.adapter.execute_calls, 1)
        adapter = SyntheticAdapter(); adapter.mode = "physical"
        with self.assertRaises(ValueError):
            CaptureRunner(ArtifactStore(self.base / "physical"), adapter)

    def test_cli_requires_explicit_dry_run_and_reports_fault_exit_codes(self):
        tool = str(ROOT / "tools/capture_action_transitions.py")
        no_mode = subprocess.run([sys.executable, tool], capture_output=True, text=True)
        self.assertEqual(no_mode.returncode, 2)
        for scenario, expected in (("complete", 0), ("after_image_missing", 2), ("interrupt_during_action", 130)):
            run = subprocess.run([sys.executable, tool, "--dry-run", "--output-dir", str(self.base / "cli"),
                                  "--scenario", scenario, "--steps", "2"], capture_output=True, text=True)
            self.assertEqual(run.returncode, expected, run.stderr)
            result = json.loads(run.stdout)
            self.assertFalse(result["trainingEligible"])
            self.assertEqual(len(result["attempts"]), 2 if scenario == "complete" else 1)
            audit = subprocess.run([sys.executable, tool, "--audit-run", result["runDir"]], capture_output=True, text=True)
            self.assertEqual(audit.returncode, 0, audit.stdout + audit.stderr)
            self.assertTrue(json.loads(audit.stdout)["readOnly"])


if __name__ == "__main__":
    unittest.main()
