import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.capture_policy_episodes import capture_policy_episodes  # noqa: E402
from tools.evaluate_policy_replay import PolicyConfig  # noqa: E402
from tools.evaluate_policy_replay import evaluate_policy_replay  # noqa: E402


READY_STATUS = {
    "available": True,
    "state": "ARMED",
    "moving": False,
    "faulted": False,
    "sensor": {"blocked": False, "faultLatched": False},
    "baseAngle": {"active": False},
}

READY_GUARD = {
    "ready": True,
    "reason": "ready",
    "statusFresh": True,
    "detectionFresh": True,
}


class PolicyEpisodeToolsTest(unittest.TestCase):
    def test_capture_policy_episodes_writes_manifest_frames_and_json_samples(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            frames = [b"jpeg-a", b"jpeg-b"]
            json_payloads = {
                "http://motion/status": READY_STATUS,
                "http://pi/detection": {
                    "available": True,
                    "detected": True,
                    "fresh": True,
                    "alignment": "LEFT",
                    "label": "cup",
                    "confidence": 0.9,
                },
                "http://ros/guard": READY_GUARD,
                "http://ros/mission": {"state": "ALIGN"},
                "http://dashboard/policy": {
                    "action": "align_left",
                    "reason": "alignment_nudge_candidate",
                    "executionAvailable": False,
                },
            }

            def fake_fetch_bytes(_url: str, _timeout: float) -> tuple[bytes, str]:
                return frames.pop(0), "image/jpeg"

            def fake_fetch_json(url: str, _timeout: float) -> dict:
                return dict(json_payloads[url])

            dataset_dir = capture_policy_episodes(
                output_root=Path(tmpdir),
                session_name="policy_session",
                label="align",
                frame_url="http://camera/frame",
                status_url="http://motion/status",
                detection_url="http://pi/detection",
                guard_url="http://ros/guard",
                mission_url="http://ros/mission",
                policy_url="http://dashboard/policy",
                instruction="align target",
                operator_action="align_left",
                count=2,
                interval=0,
                fetch_bytes_func=fake_fetch_bytes,
                fetch_json_func=fake_fetch_json,
            )

            manifest = json.loads((dataset_dir / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual("motionbrain.policy_episode.v2", manifest["schemaVersion"])
            self.assertEqual(2, manifest["capturedSamples"])
            self.assertTrue((dataset_dir / "frames" / "000000.jpg").exists())
            entries = (dataset_dir / "episodes.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual(2, len(entries))
            first = json.loads(entries[0])
            self.assertEqual("align_left", first["operatorAction"])
            self.assertEqual("LEFT", first["detection"]["alignment"])
            self.assertEqual("ARMED", first["status"]["state"])
            self.assertEqual("align_left", first["policyProposal"]["action"])
            self.assertFalse(first["policyProposal"]["executionAvailable"])
            self.assertTrue(first["controlGuard"]["ready"])

    def test_capture_marks_required_source_failure_without_counting_sample(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            dataset_dir = capture_policy_episodes(
                output_root=Path(tmpdir),
                session_name="missing_policy",
                frame_url="http://camera/frame",
                status_url="http://motion/status",
                detection_url="http://pi/detection",
                count=1,
                interval=0,
                fetch_bytes_func=lambda _url, _timeout: (b"jpeg", "image/jpeg"),
                fetch_json_func=lambda url, _timeout: READY_STATUS if url.endswith("status") else READY_GUARD,
            )

            manifest = json.loads((dataset_dir / "manifest.json").read_text(encoding="utf-8"))
            entry = json.loads((dataset_dir / "episodes.jsonl").read_text(encoding="utf-8"))
            self.assertEqual(0, manifest["capturedSamples"])
            self.assertFalse(entry["ok"])
            self.assertIn(
                {"source": "policyProposal", "error": "required_source_missing"},
                entry["errors"],
            )
            self.assertTrue(entry["controlGuard"]["derived"])
            self.assertEqual(
                "episode_recorder_http_snapshot",
                entry["controlGuard"]["provenance"],
            )

    def test_policy_replay_reports_agreement_and_zero_unsafe_rate(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            dataset_dir = Path(tmpdir) / "policy_session"
            dataset_dir.mkdir()
            entries = [
                {
                    "index": 0,
                    "ok": True,
                    "instruction": "align target",
                    "operatorAction": "align_left",
                    "status": READY_STATUS,
                    "controlGuard": READY_GUARD,
                    "detection": {
                        "available": True,
                        "detected": True,
                        "fresh": True,
                        "alignment": "LEFT",
                        "label": "cup",
                        "confidence": 0.9,
                    },
                },
                {
                    "index": 1,
                    "ok": True,
                    "instruction": "plan cup grasp",
                    "operatorAction": "cup_grasp_plan",
                    "status": READY_STATUS,
                    "controlGuard": READY_GUARD,
                    "detection": {
                        "available": True,
                        "detected": True,
                        "fresh": True,
                        "alignment": "CENTER",
                        "label": "cup",
                        "confidence": 0.9,
                    },
                },
                {
                    "index": 2,
                    "ok": True,
                    "instruction": "align target",
                    "operatorAction": "hold",
                    "status": READY_STATUS,
                    "controlGuard": {
                        "ready": True,
                        "reason": "ready",
                        "statusFresh": True,
                        "detectionFresh": False,
                    },
                    "detection": {
                        "available": True,
                        "detected": True,
                        "fresh": False,
                        "alignment": "LEFT",
                        "label": "cup",
                        "confidence": 0.9,
                    },
                },
            ]
            (dataset_dir / "episodes.jsonl").write_text(
                "".join(json.dumps(entry) + "\n" for entry in entries),
                encoding="utf-8",
            )

            summary = evaluate_policy_replay(
                dataset_dir=dataset_dir,
                output_path=dataset_dir / "policy_replay.json",
                results_path=dataset_dir / "policy_replay.jsonl",
                config=PolicyConfig(),
            )

            metrics = summary["metrics"]
            self.assertEqual(3, metrics["total"])
            self.assertEqual(0, metrics["unsafeProposals"])
            self.assertEqual(0.0, metrics["unsafeProposalRate"])
            self.assertEqual(1, metrics["physicalMotionCandidates"])
            self.assertEqual(1, metrics["staleRejections"])
            self.assertEqual(3, metrics["operatorAgreements"])
            self.assertEqual(1.0, metrics["operatorAgreementRate"])
            self.assertEqual(3, len((dataset_dir / "policy_replay.jsonl").read_text(encoding="utf-8").splitlines()))


if __name__ == "__main__":
    unittest.main()
