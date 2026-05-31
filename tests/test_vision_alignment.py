import unittest
from pathlib import Path
import sys

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import vision_host_mvp as vision  # noqa: E402


def make_jpeg_with_target(center_x: int | None, color_bgr: tuple[int, int, int] = (0, 0, 255)) -> bytes:
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    image[:] = (20, 20, 20)
    if center_x is not None:
        cv2.rectangle(image, (center_x - 16, 42), (center_x + 16, 78), color_bgr, -1)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    return encoded.tobytes()


class VisionAlignmentTest(unittest.TestCase):
    def test_classify_alignment_deadband(self) -> None:
        self.assertEqual(vision.classify_alignment(None, 0.15), "LOST")
        self.assertEqual(vision.classify_alignment(-0.16, 0.15), "LEFT")
        self.assertEqual(vision.classify_alignment(-0.15, 0.15), "CENTER")
        self.assertEqual(vision.classify_alignment(0.15, 0.15), "CENTER")
        self.assertEqual(vision.classify_alignment(0.16, 0.15), "RIGHT")

    def test_command_suggestion_for_alignment(self) -> None:
        self.assertEqual(vision.command_suggestion_for_alignment("LEFT"), "base_left")
        self.assertEqual(vision.command_suggestion_for_alignment("CENTER"), "hold")
        self.assertEqual(vision.command_suggestion_for_alignment("RIGHT"), "base_right")
        self.assertEqual(vision.command_suggestion_for_alignment("LOST"), "none")

    def test_red_target_left_center_right_and_lost(self) -> None:
        cases = [
            (32, "LEFT", "base_left"),
            (80, "CENTER", "hold"),
            (128, "RIGHT", "base_right"),
        ]
        for center_x, alignment, suggestion in cases:
            with self.subTest(alignment=alignment):
                detection = vision.detect_colored_target(make_jpeg_with_target(center_x), "red", 0.15)
                self.assertTrue(detection["detected"])
                self.assertEqual(detection["alignment"], alignment)
                self.assertEqual(detection["commandSuggestion"], suggestion)
                self.assertGreaterEqual(detection["areaRatio"], vision.TARGET_RATIO_THRESHOLD)
                self.assertIsInstance(detection["centerX"], float)
                self.assertIsInstance(detection["centerY"], float)
                self.assertEqual(detection["centroidX"], detection["centerX"])
                self.assertEqual(detection["centroidY"], detection["centerY"])
                self.assertIsInstance(detection["targetBox"], dict)
                self.assertGreater(detection["targetBox"]["width"], 0)
                self.assertGreater(detection["targetBox"]["height"], 0)
                self.assertIsInstance(detection["offsetX"], float)

        lost = vision.detect_colored_target(make_jpeg_with_target(None), "red", 0.15)
        self.assertFalse(lost["detected"])
        self.assertEqual(lost["alignment"], "LOST")
        self.assertEqual(lost["commandSuggestion"], "none")
        self.assertIsNone(lost["centerX"])
        self.assertIsNone(lost["targetBox"])
        self.assertIsNone(lost["offsetX"])

    def test_status_gates_alignment_actions(self) -> None:
        clear_armed = {"state": "ARMED", "sensor": {"blocked": False}, "baseAngle": {"active": False}}
        self.assertTrue(vision.status_allows_base_alignment(clear_armed))

        self.assertFalse(vision.status_allows_base_alignment({"state": "IDLE", "sensor": {"blocked": False}}))
        self.assertFalse(
            vision.status_allows_base_alignment(
                {"state": "ARMED", "sensor": {"blocked": True}, "baseAngle": {"active": False}}
            )
        )
        self.assertFalse(
            vision.status_allows_base_alignment(
                {"state": "ARMED", "sensor": {"blocked": False}, "baseAngle": {"active": True}}
            )
        )


if __name__ == "__main__":
    unittest.main()
