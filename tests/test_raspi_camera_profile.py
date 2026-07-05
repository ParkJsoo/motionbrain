from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "tools" / "raspi" / "apply_camera_profile.py"
SPEC = importlib.util.spec_from_file_location("apply_camera_profile", SCRIPT_PATH)
assert SPEC is not None
apply_camera_profile = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(apply_camera_profile)


class RaspiCameraProfileTest(unittest.TestCase):
    def test_service_scripts_default_to_stable_live_quality(self) -> None:
        repo_root = Path(__file__).resolve().parents[1]
        expected_fragment = 'MOTIONBRAIN_CAMERA_QUALITY:-10'
        for relative_path in (
            "tools/raspi/start_perception_service.sh",
            "tools/raspi/start_dashboard_service.sh",
            "tools/raspi/reconcile_dashboard_services.sh",
        ):
            with self.subTest(path=relative_path):
                script_text = (repo_root / relative_path).read_text()
                self.assertIn(expected_fragment, script_text)
                self.assertIn("MOTIONBRAIN_CAMERA_MIN_STABLE_QUALITY", script_text)
                self.assertIn("MOTIONBRAIN_ALLOW_UNSTABLE_CAMERA_QUALITY", script_text)
                self.assertIn("--quality \"${CAMERA_QUALITY}\"", script_text)

        perception_script = (repo_root / "tools/raspi/start_perception_service.sh").read_text()
        self.assertIn("MOTIONBRAIN_PERCEPTION_MAX_CAMERA_TIMEOUT", perception_script)
        self.assertIn("MOTIONBRAIN_ALLOW_LONG_CAMERA_TIMEOUT", perception_script)
        self.assertIn("--timeout \"${PERCEPTION_TIMEOUT}\"", perception_script)

        self.assertIn(
            'parser.add_argument("--quality", type=int, default=10',
            SCRIPT_PATH.read_text(),
        )

    def test_normalize_base_url_strips_path_and_adds_scheme(self) -> None:
        self.assertEqual(
            apply_camera_profile.normalize_base_url("motionbrain-cam.local/status"),
            "http://motionbrain-cam.local",
        )
        self.assertEqual(
            apply_camera_profile.normalize_base_url("http://192.168.219.111/camera?quality=4"),
            "http://192.168.219.111",
        )

    def test_camera_profile_url_encodes_expected_query(self) -> None:
        self.assertEqual(
            apply_camera_profile.camera_profile_url("http://motionbrain-cam.local/", "qvga", 4),
            "http://motionbrain-cam.local/camera?framesize=qvga&quality=4",
        )

    def test_needs_profile_update_compares_framesize_and_quality(self) -> None:
        self.assertFalse(
            apply_camera_profile.needs_profile_update(
                {"frameSize": "QVGA", "jpegQuality": 4},
                "qvga",
                4,
            )
        )
        self.assertTrue(
            apply_camera_profile.needs_profile_update(
                {"frameSize": "qvga", "jpegQuality": 15},
                "qvga",
                4,
            )
        )
        self.assertTrue(
            apply_camera_profile.needs_profile_update(
                {"frameSize": "vga", "jpegQuality": 4},
                "qvga",
                4,
            )
        )

    def test_missing_quality_requires_update(self) -> None:
        self.assertTrue(apply_camera_profile.needs_profile_update({"frameSize": "qvga"}, "qvga", 4))


if __name__ == "__main__":
    unittest.main()
