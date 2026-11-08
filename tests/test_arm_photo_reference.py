"""기존 관측 근거를 검증하며 알려진 합성 자세와 길이 불일치를 구분한다."""
import copy
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/motionbrain_ros_bridge"))
from motionbrain_ros_bridge.arm_geometry import load_geometry_profile, profile_digest  # noqa: E402
from learning.arm_photo_reference import infer_reference  # noqa: E402


def png(width, height):
    def chunk(tag, value):
        return struct.pack(">I", len(value)) + tag + value + struct.pack(">I", zlib.crc32(tag + value))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress((b"\0" * (width + 1)) * height)) + chunk(b"IEND", b""))


class ArmPhotoReferenceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        image = png(1000, 800)
        (self.root / "image.png").write_bytes(image)
        profile = load_geometry_profile()
        for name, value in dict(base_height_m=.07, shoulder_offset_m=0, upper_arm_m=.09,
                                forearm_m=.11, wrist_m=.045, tool_m=.065).items():
            profile["parameters"][name]["value_m"] = value
        (self.root / "geometry.json").write_text(json.dumps(profile))
        self.manifest = dict(imagePath="image.png", imageSha256=hashlib.sha256(image).hexdigest(),
                             imageWidth=1000, imageHeight=800, points={}, forwardImageSign=-1,
                             horizontalRollDeg=0, geometryProfilePath="geometry.json",
                             geometryProfileSha256=profile_digest(profile))
        self.pose()

    def pose(self, sign=-1, roll=0, scales=(1500, 1500, 1500), elevations=(60, -40, -10)):
        self.manifest.update(forwardImageSign=sign, horizontalRollDeg=roll)
        x, y = 500., 500.
        self.manifest["points"] = {"shoulder": [x, y]}
        r = math.radians(roll)
        for name, length, degrees, scale in zip(("elbow", "wrist", "tool_tip"), (.09, .11, .11),
                                               elevations, scales):
            a = math.radians(degrees)
            dx, up = sign * length * scale * math.cos(a), length * scale * math.sin(a)
            x += math.cos(r) * dx - math.sin(r) * up
            y -= math.sin(r) * dx + math.cos(r) * up
            self.manifest["points"][name] = [x, y]

    def test_known_pose_both_forward_directions_and_roll(self):
        for sign, roll in [(-1, 0), (1, 0), (-1, 12), (1, -8)]:
            with self.subTest(sign=sign, roll=roll):
                self.pose(sign, roll)
                result = infer_reference(self.manifest, self.root)
                for name, expected in [("shoulder_pitch", 60), ("elbow_pitch", -100), ("wrist_pitch", 30)]:
                    self.assertAlmostEqual(result["jointCandidateDeg"][name], expected)
                self.assertAlmostEqual(result["scaleFit"]["pixelsPerMetre"], 1500)
                self.assertLess(result["scaleFit"]["rmsLengthResidualMm"], 1e-10)
                self.assertLess(max(result["scaleFit"]["landmarkResidualMm"].values()), 1e-10)
                self.assertEqual(result["status"], "coarse_photo_reference_only")
                self.assertFalse(result["controlEligible"])
                self.assertFalse(result["m4ZeroEstablished"])
                self.assertFalse(result["directionSignEstablished"])
                self.assertFalse(result["cameraExtrinsicsCalibrated"])
                self.assertEqual(result["unmeasuredJoints"], ["base_yaw", "gripper"])

    def test_inconsistent_tip_length_is_preserved_as_residual_not_hidden(self):
        self.pose(scales=(1500, 1500, 1000))
        fit = infer_reference(self.manifest, self.root)["scaleFit"]
        self.assertAlmostEqual(fit["independentPixelsPerMetre"]["wrist_to_observed_tip"], 1000)
        self.assertGreater(fit["maxRelativeLengthResidual"], .1)
        self.assertGreater(fit["rmsLengthResidualMm"], 10)
        self.assertGreater(fit["landmarkResidualMm"]["tool_tip"], 1)

    def test_angle_branch_is_reported_as_direct_difference_without_silent_wrapping(self):
        self.pose(elevations=(170, -170, -150))
        result = infer_reference(self.manifest, self.root)
        self.assertAlmostEqual(result["jointCandidateDeg"]["elbow_pitch"], -340)
        self.assertIn("direct_principal_elevation_differences_not_joint_winding", result["assumptions"])

    def test_source_hashes_dimensions_and_points_are_required(self):
        cases = [
            lambda m: m.update(imageSha256="0" * 64),
            lambda m: m.update(geometryProfileSha256="0" * 64),
            lambda m: m.update(imageWidth=999),
            lambda m: m.update(imageWidth=True),
            lambda m: m.update(forwardImageSign=True),
            lambda m: m.update(horizontalRollDeg=float("nan")),
            lambda m: m["points"].update(elbow=[1000, 200]),
            lambda m: m["points"].update(elbow=[float("inf"), 200]),
            lambda m: m["points"].update(elbow=[True, 200]),
            lambda m: m["points"].update(elbow=m["points"]["shoulder"]),
            lambda m: m["points"].pop("wrist"),
        ]
        for index, change in enumerate(cases):
            with self.subTest(case=index):
                manifest = copy.deepcopy(self.manifest)
                change(manifest)
                with self.assertRaises(ValueError):
                    infer_reference(manifest, self.root)

    def test_cli_refuses_outside_local_and_overwrite_and_keeps_source_photo_private(self):
        spec = importlib.util.spec_from_file_location("photo_reference_cli", ROOT / "tools/estimate_arm_photo_reference.py")
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        source = self.root / "input.json"
        raw = json.dumps(self.manifest).encode()
        source.write_bytes(raw)
        output = self.root / ".local/candidate"
        with patch.object(cli, "ROOT", self.root), redirect_stderr(io.StringIO()), redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit):
                cli.main(["--input", str(source), "--output", str(self.root / "public")])
            self.assertEqual(cli.main(["--input", str(source), "--output", str(output)]), 0)
            self.assertEqual([p.name for p in output.iterdir()], ["reference.json"])
            result = json.loads((output / "reference.json").read_text())
            self.assertEqual(result["evidence"]["inputFileSha256"], hashlib.sha256(raw).hexdigest())
            with self.assertRaises(SystemExit):
                cli.main(["--input", str(source), "--output", str(output)])


if __name__ == "__main__":
    unittest.main()
