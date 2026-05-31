import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "motionbrain_ros_bridge"))

from motionbrain_ros_bridge.vision_detection import DetectionConfig  # noqa: E402
from motionbrain_ros_bridge.vision_detection import OpenCvDnnObjectDetector  # noqa: E402
from motionbrain_ros_bridge.vision_detection import decode_dnn_detections  # noqa: E402
from motionbrain_ros_bridge.vision_detection import detect_frame  # noqa: E402
from motionbrain_ros_bridge.vision_detection import load_labels  # noqa: E402
from tools.motionbrain_perception_service import build_detector  # noqa: E402


def make_jpeg() -> bytes:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[:] = (20, 20, 20)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


class FakeNet:
    def __init__(self, output: np.ndarray) -> None:
        self.output = output
        self.input_shape: tuple[int, ...] | None = None

    def setInput(self, blob: np.ndarray) -> None:  # noqa: N802
        self.input_shape = tuple(blob.shape)

    def forward(self) -> np.ndarray:
        return self.output


class VisionObjectBackendTest(unittest.TestCase):
    def test_repo_coco_label_file_matches_yolo_class_count(self) -> None:
        labels = load_labels(str(ROOT / "config" / "coco80.labels"))

        self.assertEqual(len(labels), 80)
        self.assertEqual(labels[0], "person")
        self.assertEqual(labels[39], "bottle")
        self.assertEqual(labels[41], "cup")
        self.assertEqual(labels[67], "cell phone")

    def test_label_loader_supports_plain_and_indexed_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            labels_path = Path(tmpdir) / "labels.txt"
            labels_path.write_text(
                "# comment\n"
                "background\n"
                "2 cup\n"
                "bottle\n",
                encoding="utf-8",
            )

            self.assertEqual(load_labels(str(labels_path)), ["background", "class_1", "cup", "bottle"])

    def test_decode_ssd_output_maps_normalized_box_to_candidates(self) -> None:
        output = np.array(
            [[[[0, 2, 0.91, 0.25, 0.20, 0.75, 0.80], [0, 1, 0.20, 0.1, 0.1, 0.2, 0.2]]]],
            dtype=np.float32,
        )

        candidates = decode_dnn_detections(
            output,
            frame_width=160,
            frame_height=120,
            labels=["background", "person", "cup"],
            min_confidence=0.5,
            nms_threshold=0.45,
        )

        self.assertEqual(len(candidates), 1)
        candidate = candidates[0]
        self.assertEqual(candidate.label, "cup")
        self.assertEqual(candidate.class_id, 2)
        self.assertAlmostEqual(candidate.confidence or 0.0, 0.91, places=5)
        self.assertAlmostEqual(candidate.x, 40.0, places=5)
        self.assertAlmostEqual(candidate.y, 24.0, places=5)
        self.assertAlmostEqual(candidate.width, 80.0, places=5)
        self.assertAlmostEqual(candidate.height, 72.0, places=5)

    def test_decode_yolo_output_maps_center_box_to_candidates(self) -> None:
        output = np.zeros((1, 7, 2), dtype=np.float32)
        output[0, :, 0] = [320, 320, 160, 200, 0.20, 0.10, 0.86]
        output[0, :, 1] = [80, 160, 80, 120, 0.12, 0.88, 0.20]

        candidates = decode_dnn_detections(
            output,
            frame_width=320,
            frame_height=240,
            labels=["person", "bottle", "cup"],
            min_confidence=0.5,
            nms_threshold=0.45,
            input_width=640,
            input_height=640,
        )

        self.assertEqual(len(candidates), 2)
        candidate = candidates[0]
        self.assertEqual(candidate.label, "bottle")
        self.assertEqual(candidate.class_id, 1)
        self.assertAlmostEqual(candidate.confidence or 0.0, 0.88, places=5)
        self.assertEqual((candidate.x, candidate.y, candidate.width, candidate.height), (20.0, 37.5, 40.0, 45.0))
        candidate = candidates[1]
        self.assertEqual(candidate.label, "cup")
        self.assertEqual(candidate.class_id, 2)
        self.assertAlmostEqual(candidate.confidence or 0.0, 0.86, places=5)
        self.assertEqual((candidate.x, candidate.y, candidate.width, candidate.height), (120.0, 82.5, 80.0, 75.0))

    def test_decode_nx6_output_still_handles_batched_shape(self) -> None:
        output = np.array([[[0.25, 0.20, 0.75, 0.80, 0.78, 2]]], dtype=np.float32)

        candidates = decode_dnn_detections(
            output,
            frame_width=160,
            frame_height=120,
            labels=["person", "bottle", "cup"],
            min_confidence=0.5,
            nms_threshold=0.45,
        )

        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].label, "cup")
        self.assertAlmostEqual(candidates[0].x, 40.0, places=5)

    def test_opencv_dnn_detector_feeds_net_and_returns_object_payload(self) -> None:
        output = np.array(
            [[[[0, 1, 0.82, 0.55, 0.25, 0.90, 0.85], [0, 2, 0.73, 0.05, 0.20, 0.25, 0.70]]]],
            dtype=np.float32,
        )
        net = FakeNet(output)
        detector = OpenCvDnnObjectDetector(net=net, labels=["background", "cup", "bottle"], input_size=96)
        config = DetectionConfig(
            mode="object",
            object_backend="opencv-dnn",
            object_target="cup",
            object_min_confidence=0.5,
            object_input_size=96,
        )

        payload = detect_frame(make_jpeg(), config, detector)

        self.assertEqual(net.input_shape, (1, 3, 96, 96))
        self.assertTrue(payload["detected"])
        self.assertEqual(payload["label"], "cup")
        self.assertEqual(payload["detector"]["name"], "opencv-dnn")
        self.assertEqual(payload["targetBox"], {"x": 88, "y": 30, "width": 56, "height": 72})
        self.assertEqual(len(payload["objects"]), 2)

    def test_service_build_detector_keeps_color_path_and_requires_object_model(self) -> None:
        self.assertIsNone(build_detector(DetectionConfig(mode="color")))

        with self.assertRaisesRegex(ValueError, "--object-model is required"):
            build_detector(DetectionConfig(mode="object", object_backend="opencv-dnn"))

        with self.assertRaisesRegex(ValueError, "not implemented"):
            build_detector(DetectionConfig(mode="object", object_backend="tflite"))


if __name__ == "__main__":
    unittest.main()
