import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "motionbrain_ros_bridge"))

from motionbrain_ros_bridge.vision_detection import DetectionCandidate  # noqa: E402
from motionbrain_ros_bridge.vision_detection import DetectionConfig  # noqa: E402
from motionbrain_ros_bridge.vision_detection import detect_colored_target  # noqa: E402
from motionbrain_ros_bridge.vision_detection import detect_frame  # noqa: E402


def make_jpeg_with_red_target(center_x: int | None) -> bytes:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[:] = (20, 20, 20)
    if center_x is not None:
        cv2.rectangle(image, (center_x - 16, 42), (center_x + 16, 78), (0, 0, 255), -1)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


class FakeObjectDetector:
    name = "fake-object"

    def __init__(self, candidates: list[DetectionCandidate]) -> None:
        self.candidates = candidates

    def detect(self, _frame: bytes, _config: DetectionConfig) -> list[DetectionCandidate]:
        return self.candidates


class VisionDetectionContractTest(unittest.TestCase):
    def test_color_payload_keeps_existing_fields_and_adds_contract_fields(self) -> None:
        payload = detect_colored_target(make_jpeg_with_red_target(80), "red", 0.15)

        self.assertTrue(payload["detected"])
        self.assertTrue(payload["available"])
        self.assertEqual(payload["color"], "red")
        self.assertEqual(payload["targetType"], "color")
        self.assertEqual(payload["label"], "red")
        self.assertEqual(payload["alignment"], "CENTER")
        self.assertEqual(payload["commandSuggestion"], "hold")
        self.assertIsInstance(payload["targetBox"], dict)
        self.assertIsInstance(payload["centerX"], float)
        self.assertEqual(payload["centerX"], payload["centroidX"])
        self.assertIn("detector", payload)
        self.assertEqual(payload["detector"]["mode"], "color")
        self.assertEqual(payload["stableFrames"], 0)
        self.assertEqual(payload["target"]["targetType"], "color")
        self.assertEqual(payload["objects"][0]["targetType"], "color")

    def test_color_payload_still_omits_reason_for_normal_lost_target(self) -> None:
        payload = detect_colored_target(make_jpeg_with_red_target(None), "red", 0.15)

        self.assertFalse(payload["detected"])
        self.assertEqual(payload["alignment"], "LOST")
        self.assertEqual(payload["commandSuggestion"], "none")
        self.assertNotIn("reason", payload)
        self.assertIsNone(payload["targetBox"])
        self.assertIsNone(payload["centerX"])

    def test_object_mode_selects_target_and_maps_to_alignment_payload(self) -> None:
        candidates = [
            DetectionCandidate(
                target_type="object",
                label="cup",
                class_id=47,
                confidence=0.71,
                x=92,
                y=50,
                width=34,
                height=42,
                frame_width=160,
                frame_height=120,
            ),
            DetectionCandidate(
                target_type="object",
                label="bottle",
                class_id=39,
                confidence=0.95,
                x=8,
                y=35,
                width=24,
                height=48,
                frame_width=160,
                frame_height=120,
            ),
        ]
        config = DetectionConfig(
            mode="object",
            object_backend="fake",
            object_target="cup",
            object_min_confidence=0.5,
            target_policy="highest-confidence",
        )

        payload = detect_frame(b"fake-jpeg", config, FakeObjectDetector(candidates))

        self.assertTrue(payload["detected"])
        self.assertEqual(payload["targetType"], "object")
        self.assertEqual(payload["label"], "cup")
        self.assertEqual(payload["classId"], 47)
        self.assertAlmostEqual(payload["confidence"], 0.71)
        self.assertEqual(payload["targetBox"], {"x": 92, "y": 50, "width": 34, "height": 42})
        self.assertEqual(payload["pixels"], 34 * 42)
        self.assertEqual(payload["alignment"], "RIGHT")
        self.assertEqual(payload["commandSuggestion"], "base_right")
        self.assertEqual(payload["detector"]["name"], "fake-object")
        self.assertEqual(len(payload["objects"]), 2)
        self.assertEqual(payload["target"]["label"], "cup")

    def test_object_mode_reports_unconfigured_backend_without_model_dependency(self) -> None:
        config = DetectionConfig(mode="object", object_target="cup")

        payload = detect_frame(b"fake-jpeg", config)

        self.assertFalse(payload["detected"])
        self.assertFalse(payload["available"])
        self.assertEqual(payload["reason"], "object_detector_unconfigured")
        self.assertEqual(payload["targetType"], "object")
        self.assertEqual(payload["label"], "cup")


if __name__ == "__main__":
    unittest.main()
