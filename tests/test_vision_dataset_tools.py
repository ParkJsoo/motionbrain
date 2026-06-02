import json
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.capture_vision_dataset import capture_dataset  # noqa: E402
from tools.evaluate_object_detector import DetectionConfig  # noqa: E402
from tools.evaluate_object_detector import evaluate_dataset  # noqa: E402


def make_jpeg(red: bool) -> bytes:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[:] = (20, 20, 20)
    if red:
        cv2.rectangle(image, (64, 42), (96, 78), (0, 0, 255), -1)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


class VisionDatasetToolsTest(unittest.TestCase):
    def test_capture_dataset_writes_manifest_frames_and_labels(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            frames = [make_jpeg(True), make_jpeg(False)]

            def fake_fetch_bytes(_url: str, _timeout: float) -> tuple[bytes, str]:
                return frames.pop(0), "image/jpeg"

            def fake_fetch_json(_url: str, _timeout: float) -> dict:
                return {"available": True, "detected": True, "label": "red"}

            dataset_dir = capture_dataset(
                output_root=Path(tmpdir),
                session_name="session",
                label="red",
                frame_url="http://camera/capture",
                detection_url="http://pi/api/detection",
                count=2,
                interval=0,
                fetch_bytes_func=fake_fetch_bytes,
                fetch_json_func=fake_fetch_json,
            )

            manifest = json.loads((dataset_dir / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["schemaVersion"], "motionbrain.vision_dataset.v1")
            self.assertEqual(manifest["capturedFrames"], 2)
            self.assertTrue((dataset_dir / "frames" / "000000.jpg").exists())
            lines = (dataset_dir / "labels.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 2)
            first = json.loads(lines[0])
            self.assertEqual(first["label"], "red")
            self.assertEqual(first["detection"]["label"], "red")

    def test_evaluate_dataset_reports_color_detector_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            dataset_dir = Path(tmpdir) / "dataset"
            frames_dir = dataset_dir / "frames"
            frames_dir.mkdir(parents=True)
            (frames_dir / "000000.jpg").write_bytes(make_jpeg(True))
            (frames_dir / "000001.jpg").write_bytes(make_jpeg(False))
            (dataset_dir / "labels.jsonl").write_text(
                json.dumps({"index": 0, "ok": True, "frame": "frames/000000.jpg", "label": "red"}) + "\n"
                + json.dumps({"index": 1, "ok": True, "frame": "frames/000001.jpg", "label": "background"})
                + "\n",
                encoding="utf-8",
            )

            summary = evaluate_dataset(
                dataset_dir=dataset_dir,
                output_path=dataset_dir / "evaluation.json",
                results_path=dataset_dir / "evaluation.jsonl",
                base_config=DetectionConfig(mode="color", color="red"),
                detector=None,
            )

            metrics = summary["metrics"]
            self.assertEqual(metrics["total"], 2)
            self.assertEqual(metrics["targetMatches"], 1)
            self.assertEqual(metrics["trueNegatives"], 1)
            self.assertEqual(metrics["targetFoundRate"], 1.0)
            self.assertEqual(metrics["falsePositiveRate"], 0.0)
            self.assertTrue((dataset_dir / "evaluation.json").exists())
            self.assertEqual(len((dataset_dir / "evaluation.jsonl").read_text(encoding="utf-8").splitlines()), 2)


if __name__ == "__main__":
    unittest.main()
