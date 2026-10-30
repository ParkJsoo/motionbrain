"""Device-free recording tests: real local files, synthetic frames and output logs."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from learning.artifacts import StorageError, read_events
from learning.visual_action_collection import VisualActionCollector, audit_collection, replay_bundle
from learning.visual_action_examples import synthetic_bundle, run_synthetic, write_example

ROOT = Path(__file__).resolve().parents[1]


class VisualActionCollectionTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def collect(self, session=None, items=None, name="run"):
        if session is None:
            session, items = synthetic_bundle()
        collector = VisualActionCollector(self.root / name, session)
        for event, pixels in items:
            if not collector.record(event, pixels):
                break
        summary = collector.close()
        audit = audit_collection(self.root / name)
        self.assertTrue(audit["integrityOk"], audit)
        self.assertTrue(audit["sessionClosed"], audit)
        self.assertEqual(summary, audit["summary"])
        return collector, summary

    def test_complete_keeps_originals_request_output_and_three_windows_without_network(self):
        with patch("socket.socket", side_effect=AssertionError("network forbidden")):
            collector, result = self.collect()
        self.assertTrue(result["recordingComplete"])
        self.assertFalse(result["trainingEligible"])
        self.assertEqual(result["physicalLearningEligibleRows"], 0)
        a = result["attempts"][0]
        self.assertEqual(a["request"]["pwm"], 30)
        self.assertEqual(a["record"]["elapsedMs"], 100)
        self.assertEqual(a["outputTrace"]["entries"][-1]["pwm"], 0)
        self.assertEqual(a["settlingObservation"], "unverified")
        self.assertEqual(a["leakageGroup"], "synthetic-session")
        self.assertNotIn("m4", a)
        for phase in ("before", "during", "after"):
            frames = a["windows"][phase]
            self.assertEqual(len(frames), 1)
            raw = (collector.store.root / frames[0]["image"]["artifactRef"]).read_bytes()
            self.assertEqual(hashlib.sha256(raw).hexdigest(), frames[0]["image"]["sha256"])
        self.assertEqual(a["windows"]["before"][0]["features"][0]["x"], 0)
        self.assertEqual(a["windows"]["after"][0]["features"][0]["x"], 1)

    def test_no_visible_motion_is_retained_not_filtered_as_failure(self):
        session, items = synthetic_bundle("no_motion")
        _, result = self.collect(session, items)
        self.assertTrue(result["recordingComplete"])
        a = result["attempts"][0]
        self.assertEqual(a["motionObservation"], "no_visible_motion")
        self.assertEqual(a["windows"]["before"][0]["image"]["sha256"], a["windows"]["after"][0]["image"]["sha256"])
        self.assertFalse(a["trainingEligible"])

    def test_all_fault_scenarios_keep_attempt_and_auditable_partial_data(self):
        expected = {"missing_after": "incomplete", "unknown_result": "outcome_unknown",
                    "controller_failed": "failed", "missing_frame": "incomplete",
                    "corrupt_frame": "incomplete", "interrupted": "outcome_unknown"}
        for scenario, status in expected.items():
            with self.subTest(scenario=scenario):
                session, items = synthetic_bundle(scenario)
                c, result = self.collect(session, items, scenario)
                self.assertFalse(result["recordingComplete"])
                self.assertEqual(result["attemptCount"], 1)
                self.assertEqual(result["attempts"][0]["status"], status)
                with self.assertRaises(RuntimeError):
                    c.record(items[0][0])
                if scenario == "corrupt_frame":
                    events, _ = read_events(c.store.root)
                    image = events[2]["payload"]["image"]
                    self.assertTrue((c.store.root / image["artifactRef"]).exists())
                    self.assertEqual(result["errors"][0]["reason"], "frame_hash_mismatch")

    def test_before_failure_never_claims_dispatch_and_unknown_never_claims_off(self):
        for scenario, state in (("missing_frame", "not_sent"), ("unknown_result", "outcome_unknown")):
            session, items = synthetic_bundle(scenario)
            _, result = self.collect(session, items, scenario)
            a = result["attempts"][0]
            self.assertEqual(a["executionState"], state)
            self.assertIsNone(a["record"])
            self.assertIsNone(a["outputTrace"])

    def test_identity_clock_order_and_shape_errors_are_durable_and_halt(self):
        cases = [
            (1, "hostClockId", "other", "host_clock_mismatch"),
            (1, "viewId", "other", "frame_viewId_mismatch"),
            (1, "streamId", "other", "frame_streamId_mismatch"),
            (1, "sourceClockId", "other", "frame_sourceClockId_mismatch"),
            (1, "actionId", "other", "active_action_mismatch"),
            (1, "at", 99, "host_time_regressed"),
            (1, "pts", {"value": True, "timescale": 1, "epoch": 0}, "invalid_pts"),
            (3, "pts", {"value": 10, "timescale": 1000, "epoch": 0}, "pts_reset_repeated_or_regressed"),
            (3, "pts", {"value": 20, "timescale": 1000, "epoch": 1}, "pts_reset_repeated_or_regressed"),
            (3, "frameId", "frame-1", "duplicate_frame_id"),
            (2, "at", 100.1, "ambiguous_before_dispatch_order"),
            (5, "at", 100.4, "ambiguous_after_result_order"),
            (5, "at", 200, "recording_time_budget_exceeded"),
        ]
        for i, (index, key, value, error) in enumerate(cases):
            with self.subTest(error=error):
                session, items = synthetic_bundle()
                items[index][0][key] = value
                c, result = self.collect(session, items, str(i))
                self.assertFalse(result["recordingComplete"])
                self.assertEqual(result["errors"][0]["reason"], error)
                raw = json.loads((c.store.root / f"raw/{index + 1:08d}.json").read_text())
                self.assertEqual(raw[key], value)

    def test_duplicate_action_and_dispatch_are_rejected(self):
        session, items = synthetic_bundle()
        session["maxAttempts"] = 2
        duplicate = copy.deepcopy(items[0])
        duplicate[0]["at"] = 101
        _, result = self.collect(session, items + [duplicate])
        self.assertEqual(result["attemptCount"], 1)
        self.assertEqual(result["errors"][0]["reason"], "duplicate_action_id")
        session, items = synthetic_bundle()
        items.insert(3, copy.deepcopy(items[2]))
        _, result = self.collect(session, items, "dispatch")
        self.assertEqual(result["errors"][0]["reason"], "duplicate_or_late_dispatch")

    def two_actions(self):
        session, first = synthetic_bundle()
        _, second = synthetic_bundle()
        session.update(maxAttempts=2, maxFrames=6)
        for e, _ in second:
            e["actionId"] = "action-2"
            e["at"] += 1
            if e["type"] == "begin":
                e["request"]["commandId"] = "action-2"
                e["episodeId"] = "episode-2"
            elif e["type"] == "frame":
                e["frameId"] += "-next"
                e["pts"]["value"] += 100
            elif e["type"] == "result":
                e["record"].update(commandId="action-2", ordinal=2, startedAtMs=2000, endedAtMs=2100)
                e["baselineOutputs"]["entries"] = copy.deepcopy(first[4][0]["outputLog"]["entries"])
                for output in e["outputLog"]["entries"]:
                    output["sequence"] += 2
                    output["actionOrdinal"] = 2
                    output["atMs"] += 1000
        return session, first + second

    def test_two_actions_preserve_order_and_same_session_leakage_group(self):
        session, items = self.two_actions()
        _, result = self.collect(session, items)
        self.assertTrue(result["recordingComplete"])
        self.assertEqual(result["attemptCount"], 2)
        self.assertEqual({a["leakageGroup"] for a in result["attempts"]}, {session["sessionId"]})

    def test_controller_evidence_cannot_be_reused_between_actions(self):
        for kind in ("ordinal", "sequence", "time"):
            session, items = self.two_actions()
            second = items[11][0]
            if kind == "ordinal":
                second["record"]["ordinal"] = 1
                for row in second["outputLog"]["entries"]:
                    row["actionOrdinal"] = 1
                expected = "controller_ordinal_reused_or_regressed"
            elif kind == "sequence":
                second["baselineOutputs"]["entries"] = []
                for row in second["outputLog"]["entries"]:
                    row["sequence"] -= 2
                expected = "controller_output_sequence_reused_or_regressed"
            else:
                second["record"].update(startedAtMs=900, endedAtMs=1000)
                for row in second["outputLog"]["entries"]:
                    row["atMs"] -= 1100
                expected = "controller_time_regressed"
            _, result = self.collect(session, items, kind)
            self.assertEqual(result["errors"][0]["reason"], expected)
            self.assertFalse(result["recordingComplete"])

    def test_new_controller_boot_can_restart_sequence_and_clock(self):
        session, items = self.two_actions()
        items[7][0]["request"]["bootId"] = "b" * 32
        result = items[11][0]
        result["record"].update(bootId="b" * 32, ordinal=1, startedAtMs=1, endedAtMs=101)
        for key in ("baselineOutputs", "outputLog"):
            result[key]["bootId"] = "b" * 32
        result["baselineOutputs"]["entries"] = []
        for row in result["outputLog"]["entries"]:
            row["sequence"] -= 2
            row["actionOrdinal"] = 1
            row["atMs"] -= 1999
        _, report = self.collect(session, items)
        self.assertTrue(report["recordingComplete"])

    def test_controller_millisecond_wrap_is_not_a_clock_reset(self):
        session, items = self.two_actions()
        first = items[4][0]
        first["record"].update(startedAtMs=2**32 - 110, endedAtMs=2**32 - 10)
        for row in first["outputLog"]["entries"]:
            row["atMs"] += 2**32 - 1110
        second = items[11][0]
        second["baselineOutputs"]["entries"] = copy.deepcopy(first["outputLog"]["entries"])
        second["record"].update(startedAtMs=10, endedAtMs=110)
        for row in second["outputLog"]["entries"]:
            row["atMs"] -= 1990
        _, report = self.collect(session, items)
        self.assertTrue(report["recordingComplete"])

    def test_failed_and_unknown_outcomes_keep_passive_observations_but_block_next_action(self):
        for scenario, phase in (("controller_failed", "after"), ("unknown_result", "unresolved")):
            session, items = synthetic_bundle(scenario)
            c, report = self.collect(session, items, scenario)
            a = report["attempts"][0]
            self.assertEqual(len(a["windows"][phase]), 1)
            self.assertTrue(a["ended"])
            self.assertTrue(report["newActionsBlocked"])
            with self.assertRaises(RuntimeError):
                c.record(items[0][0])

    def test_camera_failure_preserves_current_observation_and_prevents_dispatch(self):
        session, items = synthetic_bundle()
        failure = {"type": "failure", "at": 100.15, "hostClockId": session["hostClockId"],
                   "actionId": "action-1", "reason": "camera_disconnected"}
        _, report = self.collect(session, items[:2] + [(failure, None)] + items[2:])
        self.assertEqual(report["errors"][0]["reason"], "dispatch_blocked_after_failure")
        self.assertEqual(report["attempts"][0]["executionState"], "not_sent")

    def test_terminal_receipt_mismatch_stays_unknown_and_no_after_window(self):
        session, items = synthetic_bundle()
        items[4][0]["record"]["commandId"] = "other"
        _, result = self.collect(session, items)
        a = result["attempts"][0]
        self.assertEqual(a["executionState"], "outcome_unknown")
        self.assertIsNone(a["record"])
        self.assertEqual(a["windows"]["after"], [])

    def test_dropped_output_trace_retains_terminal_but_halts(self):
        session, items = synthetic_bundle()
        items[4][0]["outputLog"]["dropped"] = 1
        _, result = self.collect(session, items)
        a = result["attempts"][0]
        self.assertEqual(a["executionState"], "known_terminal")
        self.assertTrue(a["record"]["outputOff"])
        self.assertIn("output_coverage_unverified", a["issues"])
        self.assertEqual(result["frameCount"], 3)
        self.assertFalse(result["recordingComplete"])

    def test_m4_uses_target_angle_request_without_requiring_m4_camera_measurements(self):
        session, items = synthetic_bundle()
        request = items[0][0]["request"]
        request.update(kind="shoulder", motor=4, pwm=0, targetDegrees=230, percent=80)
        record = items[4][0]["record"]
        record.update(kind="shoulder", motor=4, requestedPwm=0, targetDegrees=230, percent=80,
                      reason="target_reached", targetReached=True)
        for e in items[4][0]["outputLog"]["entries"]:
            e["motor"] = 4
        _, result = self.collect(session, items)
        self.assertTrue(result["recordingComplete"])
        self.assertEqual(result["attempts"][0]["request"]["targetDegrees"], 230)

    def test_invisible_features_are_explicit_and_invalid_coordinates_rejected(self):
        session, items = synthetic_bundle()
        items[5][0]["features"] = [{"id": "tip", "visible": False, "x": None, "y": None, "confidence": 0}]
        _, result = self.collect(session, items)
        self.assertTrue(result["recordingComplete"])
        items[5][0]["features"][0]["x"] = 1
        _, result = self.collect(session, items, "invalid")
        self.assertEqual(result["errors"][0]["reason"], "invalid_feature_coordinates")

    def test_budget_exhaustion_and_missing_during_window(self):
        session, items = synthetic_bundle()
        session["maxFrames"] = 2
        _, result = self.collect(session, items)
        self.assertEqual(result["errors"][0]["reason"], "frame_budget_exceeded")
        session, items = synthetic_bundle()
        del items[3]
        _, result = self.collect(session, items, "missing")
        self.assertIn("missing_during_frames", result["attempts"][0]["issues"])

    def test_recorded_bytes_and_summary_tampering_fail_audit(self):
        c, _ = self.collect()
        frame = next((c.store.root / "frames").glob("*.bgra"))
        frame.write_bytes(b"tampered")
        self.assertFalse(audit_collection(c.store.root)["integrityOk"])
        c, _ = self.collect(name="summary")
        (c.store.root / "summary.json").write_text('{"trainingEligible":true}')
        self.assertFalse(audit_collection(c.store.root)["integrityOk"])

    def test_process_death_after_dispatch_keeps_unknown_not_success_or_retry(self):
        session, items = synthetic_bundle()
        c = VisualActionCollector(self.root / "run", session)
        for e, pixels in items[:3]:
            c.record(e, pixels)
        audit = audit_collection(c.store.root)
        self.assertFalse(audit["sessionClosed"])
        self.assertFalse(audit["recordingComplete"])
        self.assertEqual(audit["summary"]["attempts"][0]["executionState"], "outcome_unknown")
        with self.assertRaises(StorageError):
            VisualActionCollector(c.store.root, session)

    def test_storage_failure_blocks_further_input_and_preserves_dispatch_prefix(self):
        session, items = synthetic_bundle()
        c = VisualActionCollector(self.root / "run", session)
        for e, pixels in items[:3]:
            c.record(e, pixels)
        with patch.object(c.store, "_write", side_effect=StorageError("disk full")):
            with self.assertRaises(StorageError):
                c.record(*items[3])
        self.assertTrue(c.halted)
        with self.assertRaises(RuntimeError):
            c.record(*items[4])
        audit = audit_collection(c.store.root)
        self.assertEqual(audit["summary"]["attempts"][0]["executionState"], "outcome_unknown")
        self.assertFalse(audit["recordingComplete"])

    def test_replay_rejects_unsafe_image_paths_and_never_substitutes_a_frame(self):
        bundle_path = write_example(self.root / "example")
        original = json.loads(bundle_path.read_text())
        for i, path in enumerate(("../secret.bgra", "/tmp/secret.bgra", "missing.bgra", "link.bgra")):
            bundle = copy.deepcopy(original)
            bundle["events"][1]["imagePath"] = path
            if path == "link.bgra":
                (bundle_path.parent / path).symlink_to(bundle_path.parent / "frames/0001.bgra")
            bundle_path.write_text(json.dumps(bundle))
            result = replay_bundle(bundle_path, self.root / f"import-{i}")
            self.assertFalse(result["recordingComplete"])
            self.assertEqual(result["attempts"][0]["executionState"], "not_sent")
            self.assertEqual(result["errors"][0]["reason"], "frame_bytes_missing_or_shape_mismatch")

    def test_replay_example_and_cli_status(self):
        bundle = write_example(self.root / "example")
        result = replay_bundle(bundle, self.root / "import")
        self.assertTrue(result["recordingComplete"])
        for scenario, code in (("complete", 0), ("unknown_result", 2)):
            run = self.root / scenario
            p = subprocess.run([sys.executable, str(ROOT / "tools/collect_visual_actions.py"),
                                "dry-run", "--scenario", scenario, "--output-dir", str(run)],
                               capture_output=True, text=True, timeout=10)
            self.assertEqual(p.returncode, code, p.stderr)
            self.assertFalse(json.loads(p.stdout)["trainingEligible"])
            p = subprocess.run([sys.executable, str(ROOT / "tools/collect_visual_actions.py"), "audit", str(run)],
                               capture_output=True, text=True, timeout=10)
            self.assertEqual(p.returncode, code, p.stderr)

    def test_interrupt_during_replay_persistence_remains_keyboard_interrupt(self):
        from learning.artifacts import ArtifactStore
        bundle = write_example(self.root / "example")
        original = ArtifactStore.append
        def interrupt(store, kind, transition_id, payload):
            if kind == "input":
                raise KeyboardInterrupt("signal during persistence")
            return original(store, kind, transition_id, payload)
        with patch.object(ArtifactStore, "append", interrupt):
            with self.assertRaises(KeyboardInterrupt):
                replay_bundle(bundle, self.root / "interrupted")
        report = audit_collection(self.root / "interrupted")
        self.assertFalse(report["sessionClosed"])
        self.assertFalse(report["recordingComplete"])

    def test_invalid_session_does_not_create_output_directory(self):
        session, _ = synthetic_bundle()
        session["mode"] = "physical"
        with self.assertRaises(ValueError):
            VisualActionCollector(self.root / "no", session)
        self.assertFalse((self.root / "no").exists())


if __name__ == "__main__":
    unittest.main()
