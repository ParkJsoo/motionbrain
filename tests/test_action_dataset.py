"""Real saved synthetic runs exercise dataset gates without physical relabeling."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from learning.artifacts import ArtifactStore
from learning.dataset import build_manifest, validate_manifest
from learning.dry_run import SyntheticAdapter
from learning.runner import CaptureRunner


ROOT = Path(__file__).resolve().parents[1]


class ActionDatasetTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)

    def saved_run(self, name, *, steps=1, scenario="complete"):
        directory = self.base / name
        runner = CaptureRunner(ArtifactStore(directory), SyntheticAdapter(scenario))
        for index in range(steps):
            runner.capture(transition_id=f"transition-{index}", episode_id=f"episode-{index // 2}")
            if runner.halted:
                break
        runner.close()
        return directory

    @staticmethod
    def write_json(root, relative, value):
        """Deliberately simulate consistent file edits, not storage API writes."""
        raw = (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode()
        (root / relative).write_bytes(raw)
        return {"artifactRef": relative, "sha256": hashlib.sha256(raw).hexdigest(), "bytes": len(raw)}

    def edit_summary(self, root, mutate):
        """Update descriptor chain so semantic mismatch, not stale hash, is tested."""
        relative = "attempts/transition-0.json"
        summary = json.loads((root / relative).read_text())
        mutate(summary)
        descriptor = self.write_json(root, relative, summary)
        run = json.loads((root / "run.json").read_text())
        run["attempts"] = [descriptor if item["artifactRef"] == relative else item for item in run["attempts"]]
        run_descriptor = self.write_json(root, "run.json", run)
        for path in sorted((root / "events").glob("*.json")):
            event = json.loads(path.read_text())
            if event["kind"] == "attempt_finished" and event["transitionId"] == "transition-0":
                event["payload"]["summary"] = descriptor
            elif event["kind"] == "session_finished":
                event["payload"]["summary"] = run_descriptor
            else:
                continue
            self.write_json(root, str(path.relative_to(root)), event)

    def test_snapshot_policy_disagreement_is_source_integrity_failure(self):
        for index, policy in enumerate((
            {"maxSourceAgeMs": 1, "maxM4AgeMs": 1, "minimumSettlingMs": 1000},
            None,
        )):
            with self.subTest(policy=policy):
                root = self.base / f"policy-mismatch-{index}"
                runner = CaptureRunner(ArtifactStore(root), SyntheticAdapter(),
                                       config={"mode": "dry_run", "qualityPolicy": policy})
                runner.capture(transition_id="transition-0", episode_id="episode-0")
                runner.close()
                manifest = build_manifest([root])
                self.assertIn("snapshot_quality_policy_mismatch_or_missing", manifest["sources"][0]["issues"])
                self.assertIn("source_run_integrity_invalid", manifest["rows"][0]["exclusionReasons"])
                self.assertFalse(validate_manifest(manifest)["valid"])

    def test_inventory_is_deterministic_and_keeps_all_synthetic_attempts_excluded(self):
        one = self.saved_run("one", steps=3)
        two = self.saved_run("two")
        with patch("socket.socket", side_effect=AssertionError("network forbidden")):
            first = build_manifest([one, two], seed="fixed")
            second = build_manifest([two, one], seed="fixed")
            report = validate_manifest(first)
        self.assertEqual(first, second)
        self.assertEqual(len(first["rows"]), 4)
        self.assertEqual(first["eligibleCount"], 0)
        self.assertEqual(first["excludedCount"], 4)
        self.assertEqual(first["assignments"], {})
        self.assertFalse(first["readyForTraining"])
        self.assertTrue(report["valid"], report)
        self.assertFalse(report["readyForTraining"])
        for row in first["rows"]:
            self.assertFalse(row["eligible"])
            self.assertEqual(row["provenance"], "synthetic")
            self.assertIn("synthetic_dry_run", row["exclusionReasons"])
            self.assertIn("image_seen_in_synthetic_source", row["exclusionReasons"])
            self.assertEqual(len(row["images"]), 2)
            for image in row["images"]:
                raw = (Path(row["sourceRoot"]) / image["artifactRef"]).read_bytes()
                self.assertEqual(image["sha256"], hashlib.sha256(raw).hexdigest())
                self.assertEqual(image["bytes"], len(raw))

    def test_cached_eligible_spoof_cannot_promote_synthetic_capture(self):
        root = self.saved_run("cached")
        def spoof(summary):
            summary["trainingEligible"] = True
            summary["validation"] = {"valid": True, "trainingEligible": True,
                                     "schemaErrors": [], "exclusionReasons": []}
        self.edit_summary(root, spoof)
        manifest = build_manifest([root])
        self.assertEqual(manifest["inputIssues"], [])
        self.assertFalse(manifest["rows"][0]["validation"]["trainingEligible"])
        self.assertIn("synthetic_dry_run", manifest["rows"][0]["validation"]["exclusionReasons"])
        self.assertFalse(manifest["rows"][0]["eligible"])
        self.assertEqual(manifest["assignments"], {})

    def test_tampered_and_missing_original_images_invalidate_integrity(self):
        for name in ("tampered", "missing"):
            with self.subTest(name=name):
                root = self.saved_run(name)
                original = build_manifest([root])
                image = root / original["rows"][0]["images"][0]["artifactRef"]
                image.write_bytes(b"changed original") if name == "tampered" else image.unlink()
                rebuilt = build_manifest([root])
                self.assertTrue(rebuilt["inputIssues"])
                self.assertFalse(rebuilt["rows"][0]["eligible"])
                self.assertFalse(validate_manifest(original)["valid"])
                self.assertFalse(validate_manifest(rebuilt)["valid"])

    def test_summary_cannot_substitute_observation_or_action_for_original_raw(self):
        for name, mutate, expected in (
            ("observation", lambda summary: summary["transition"]["before"]["m4"].update(angleDegrees=9),
             "before:raw_summary_mismatch"),
            ("action", lambda summary: summary["transition"]["action"]["record"].update(reason="changed"),
             "action:raw_summary_mismatch"),
        ):
            with self.subTest(name=name):
                root = self.saved_run(name)
                self.edit_summary(root, mutate)
                manifest = build_manifest([root])
                self.assertIn(expected, manifest["rows"][0]["integrityIssues"])
                self.assertFalse(validate_manifest(manifest)["valid"])

    def test_duplicate_roots_and_copied_sessions_and_identities_are_rejected(self):
        root = self.saved_run("source")
        duplicate = build_manifest([root, root])
        self.assertIn("duplicate_source_root", duplicate["inputIssues"])
        self.assertIn("duplicate_sample_identity", duplicate["inputIssues"])
        self.assertIn("duplicate_action_identity", duplicate["inputIssues"])
        self.assertFalse(validate_manifest(duplicate)["valid"])
        copied = self.base / "copy"
        shutil.copytree(root, copied)
        manifest = build_manifest([root, copied])
        self.assertTrue(any(issue.startswith("duplicate_session_source:") for issue in manifest["inputIssues"]))
        self.assertIn("duplicate_sample_identity", manifest["inputIssues"])
        self.assertIn("duplicate_action_identity", manifest["inputIssues"])
        self.assertFalse(validate_manifest(manifest)["valid"])

    def test_failure_and_interruption_remain_visible_without_split_assignment(self):
        for scenario in ("before_missing", "after_image_missing", "unknown_result", "interrupt_during_action"):
            with self.subTest(scenario=scenario):
                root = self.saved_run(scenario, scenario=scenario)
                manifest = build_manifest([root])
                self.assertEqual(len(manifest["rows"]), 1)
                self.assertFalse(manifest["rows"][0]["eligible"])
                self.assertIn("collection_not_complete_known_terminal", manifest["rows"][0]["exclusionReasons"])
                self.assertEqual(manifest["assignments"], {})
                self.assertEqual(manifest["excludedCount"], 1)

    def test_missing_terminal_and_run_symlink_fail_closed(self):
        root = self.saved_run("truncated")
        sorted((root / "events").glob("*.json"))[-1].unlink()
        manifest = build_manifest([root])
        self.assertTrue(manifest["inputIssues"])
        self.assertFalse(manifest["readyForTraining"])
        self.assertFalse(validate_manifest(manifest)["valid"])
        symlink = self.base / "linked"
        symlink.symlink_to(root, target_is_directory=True)
        linked = build_manifest([symlink])
        self.assertTrue(any("run_root_symlink" in issue for issue in linked["inputIssues"]))
        self.assertEqual(linked["rows"], [])

    def test_manifest_assignment_and_image_hash_edits_fail_revalidation(self):
        root = self.saved_run("manifest")
        original = build_manifest([root])
        for name in ("assignment", "image_hash", "eligible", "ready", "policy"):
            with self.subTest(name=name):
                manifest = copy.deepcopy(original)
                if name == "assignment":
                    manifest["assignments"][manifest["rows"][0]["sampleId"]] = "train"
                elif name == "image_hash":
                    manifest["rows"][0]["images"][0]["sha256"] = "0" * 64
                elif name == "eligible":
                    manifest["rows"][0]["eligible"] = True
                elif name == "ready":
                    manifest["readyForTraining"] = True
                else:
                    manifest["splitPolicy"]["ratios"] = [100, 0, 0]
                report = validate_manifest(manifest)
                self.assertFalse(report["valid"], report)
                self.assertFalse(report["readyForTraining"])

    def test_malformed_artifact_reference_or_image_digest_is_quarantined_without_crash(self):
        for name in ("artifact_ref", "image_digest"):
            with self.subTest(name=name):
                root = self.saved_run(name)
                if name == "artifact_ref":
                    self.edit_summary(root, lambda summary: summary["artifacts"][0].update(artifactRef=[]))
                else:
                    for path in sorted((root / "events").glob("*.json")):
                        event = json.loads(path.read_text())
                        if event["kind"] == "image_saved":
                            event["payload"]["image"]["sha256"] = []
                            self.write_json(root, str(path.relative_to(root)), event)
                            break
                manifest = build_manifest([root])
                self.assertTrue(manifest["inputIssues"])
                self.assertFalse(manifest["rows"][0]["eligible"])
                self.assertFalse(validate_manifest(manifest)["valid"])

    def test_returned_manifest_cannot_mutate_global_split_policy(self):
        root = self.saved_run("policy-alias")
        manifest = build_manifest([root])
        manifest["splitPolicy"]["ratios"][0] = 100
        self.assertFalse(validate_manifest(manifest)["valid"])
        self.assertEqual(build_manifest([root])["splitPolicy"]["ratios"], [80, 10, 10])

    def test_bad_terminal_summary_reference_is_reported_without_type_error(self):
        root = self.saved_run("bad-summary-ref")
        for path in sorted((root / "events").glob("*.json")):
            event = json.loads(path.read_text())
            if event["kind"] == "attempt_finished":
                event["payload"]["summary"]["artifactRef"] = []
                self.write_json(root, str(path.relative_to(root)), event)
        manifest = build_manifest([root])
        self.assertIn("summary_reference_invalid", manifest["rows"][0]["integrityIssues"])
        self.assertFalse(validate_manifest(manifest)["valid"])

    def test_inconsistent_expected_hash_is_excluded_even_with_hash_matches_true(self):
        root = self.saved_run("expected-hash")
        relative = "images/transition-0/before.jpg"
        def mutate(summary):
            for reference in summary["artifacts"]:
                if reference["artifactRef"] == relative:
                    reference["expectedSha256"] = "0" * 64
        self.edit_summary(root, mutate)
        for path in sorted((root / "events").glob("*.json")):
            event = json.loads(path.read_text())
            if event["kind"] == "image_saved" and event["payload"]["phase"] == "before":
                event["payload"]["image"]["expectedSha256"] = "0" * 64
                self.write_json(root, str(path.relative_to(root)), event)
        manifest = build_manifest([root])
        self.assertIn("before:image_hash_disagreement", manifest["rows"][0]["exclusionReasons"])
        self.assertFalse(manifest["rows"][0]["eligible"])

    def test_reordered_journal_cannot_claim_before_image_preceded_intent(self):
        root = self.saved_run("order")
        paths = sorted((root / "events").glob("*.json"))
        events = [json.loads(path.read_text()) for path in paths]
        image_index = next(index for index, event in enumerate(events)
                           if event["kind"] == "image_saved" and event["payload"]["phase"] == "before")
        intent_index = next(index for index, event in enumerate(events) if event["kind"] == "action_intent")
        events[image_index], events[intent_index] = events[intent_index], events[image_index]
        for index, (path, event) in enumerate(zip(paths, events), 1):
            event["sequence"] = index
            self.write_json(root, str(path.relative_to(root)), event)
        manifest = build_manifest([root])
        self.assertIn("attempt_event_order_invalid", manifest["rows"][0]["integrityIssues"])
        self.assertFalse(validate_manifest(manifest)["valid"])

    def test_cli_writes_immutable_inventory_and_require_ready_rejects_synthetic(self):
        root = self.saved_run("cli-source")
        tool = str(ROOT / "tools/build_action_dataset.py")
        destination = self.base / "manifest-output"
        args = [sys.executable, tool, "--runs", str(root), "--output-dir", str(destination)]
        created = subprocess.run(args, capture_output=True, text=True)
        self.assertEqual(created.returncode, 0, created.stdout + created.stderr)
        report = json.loads(created.stdout)
        self.assertTrue(report["valid"])
        self.assertFalse(report["readyForTraining"])
        manifest_path = destination / "manifest.json"
        original_bytes = manifest_path.read_bytes()
        duplicate = subprocess.run(args, capture_output=True, text=True)
        self.assertEqual(duplicate.returncode, 2)
        self.assertEqual(manifest_path.read_bytes(), original_bytes)
        for ready, expected in ((False, 0), (True, 2)):
            command = [sys.executable, tool, "--validate", str(manifest_path)]
            if ready:
                command.append("--require-ready")
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
            self.assertEqual(manifest_path.read_bytes(), original_bytes)
        required_output = self.base / "required-output"
        required = subprocess.run([sys.executable, tool, "--runs", str(root), "--output-dir",
                                   str(required_output), "--require-ready"], capture_output=True, text=True)
        self.assertEqual(required.returncode, 2, required.stdout + required.stderr)
        self.assertTrue((required_output / "manifest.json").exists())


if __name__ == "__main__":
    unittest.main()
