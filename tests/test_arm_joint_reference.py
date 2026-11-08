"""수직 높이 기준의 단일 근사 anchor와 실패 경계를 합성 자료로 검증한다."""
import copy
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/motionbrain_ros_bridge"))
from motionbrain_ros_bridge.arm_geometry import load_geometry_profile, profile_digest
from motionbrain_ros_bridge.m4_sensor_mapping import SensorJointCalibration, sensor_degrees_to_ros_radians
from learning.arm_joint_reference import infer_joint_reference
from tests.test_arm_photo_reference import png


class JointReferenceTest(unittest.TestCase):
    def write(self, name, value):
        raw = json.dumps(value).encode()
        (self.root / name).write_bytes(raw)
        return hashlib.sha256(raw).hexdigest()

    def snapshot(self, start, uptime):
        return dict(schema="motionbrain.arm_anchor_snapshot.v1", hostClockId="synthetic-static-clock",
            requestStartedAt=start, receivedAt=start+1,
            status=dict(state="IDLE", motorEnabled=False, uptimeMs=uptime,
                motors={f"M{i}": dict(speed=0, enabled=False) for i in range(1, 6)},
                shoulderAngle=dict(available=True, sensorConnected=True, sensorFresh=True, sensorReady=True,
                    magnetDetected=True, magnetTooWeak=False, magnetTooStrong=False, active=False,
                    correctionActive=False, manualCalibrationUnlock=False, angleDeg=192.83,
                    rawDeg=217.18, mountOffsetDeg=-24.35, ageMs=10)),
            actions=dict(bootId="b"*32, active=False),
            action_outputs=dict(bootId="b"*32, evidence="driver_write", clockDomain="controller_boot_millis",
                entries=[dict(sequence=i, motor=i, pwm=0, atMs=100+i) for i in range(1, 6)]),
            sequence=dict(sequence=dict(state="IDLE", totalCount=0)))

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        profile = load_geometry_profile()
        for key, value in dict(base_height_m=.07, upper_arm_m=.09, forearm_m=.11, wrist_m=.045, tool_m=.065).items():
            profile["parameters"][key]["value_m"] = value
        self.write("geometry.json", profile)
        image = png(1000, 800)
        (self.root / "image.png").write_bytes(image)
        self.height = dict(schema="motionbrain.elbow_height_measurement.v1", unit="mm",
            elbowHeightRangeMm=[145, 145], method="synthetic_vertical_height", uncertaintyQuantified=False)
        self.before, self.after = self.snapshot(10., 1000), self.snapshot(15., 6000)
        self.manifest = dict(schema="motionbrain.arm_joint_reference_input.v1",
            geometryProfilePath="geometry.json", geometryProfileSha256=profile_digest(profile),
            heightMeasurementPath="height.json", heightMeasurementSha256=self.write("height.json", self.height),
            beforeSnapshotPath="before.json", beforeSnapshotSha256=self.write("before.json", self.before),
            afterSnapshotPath="after.json", afterSnapshotSha256=self.write("after.json", self.after),
            image=dict(imagePath="image.png", imageSha256=hashlib.sha256(image).hexdigest(),
                imageWidth=1000, imageHeight=800, hostClockId="synthetic-static-clock",
                sourceEarliest=12., sourceLatest=12.01,
                points=dict(shoulder=[700, 500], elbow=[600, 310], wrist=[420, 490], tool_tip=[220, 540]),
                forwardImageSign=-1, horizontalRollDeg=0), directionSign=1,
            directionEvidence=dict(status="provisional_relative_correspondence", path="direction.json",
                sha256=self.write("direction.json", dict(sensorDeltaDeg=-1.23, projectedDeltaDeg=-1.27))))

    def test_known_height_yields_anchor_pair_without_double_mount_offset(self):
        result = infer_joint_reference(self.manifest, self.root)
        expected = math.asin(75/90)
        anchor = result["m4Reference"]
        self.assertAlmostEqual(anchor["jointReferenceRad"], expected)
        self.assertEqual(anchor["sensorReferenceDeg"], 192.83)
        calibration = SensorJointCalibration(**anchor["rosMappingCandidate"])
        self.assertAlmostEqual(sensor_degrees_to_ros_radians(192.83, calibration), expected)
        self.assertAlmostEqual(sensor_degrees_to_ros_radians(193.83, calibration), expected+math.radians(1))
        self.assertFalse(anchor["enabled"])
        self.assertFalse(anchor["derivedHorizontalValueWasMeasured"])
        self.assertFalse(result["precisionZeroCalibrated"])
        self.assertFalse(result["absoluteDirectionValidated"])
        self.assertFalse(result["controlEligible"])
        self.assertFalse(result["staticPhotoReference"]["m2M3SensorZerosEstablished"])
        self.assertEqual(result["unmeasuredJoints"], ["base_yaw", "gripper"])
        height = result["heightReference"]
        self.assertFalse(height["equalHeightBoundsMeanZeroError"])
        self.assertFalse(height["uncertaintyQuantified"])
        self.assertIn("not_confidence_interval", height["rangeMeaning"])
        self.assertGreater(abs(height["heightMinusPhotoShoulderDeg"]), 1)

    def test_reported_height_range_is_not_a_precision_interval(self):
        self.height["elbowHeightRangeMm"] = [145, 150]
        self.manifest["heightMeasurementSha256"] = self.write("height.json", self.height)
        result = infer_joint_reference(self.manifest, self.root)
        height = result["heightReference"]
        self.assertEqual(height["selectedNominalHeightMm"], 147.5)
        self.assertAlmostEqual(height["heightInputAngleRangeDeg"][0], math.degrees(math.asin(75/90)))
        self.assertAlmostEqual(height["heightInputAngleRangeDeg"][1], math.degrees(math.asin(80/90)))
        self.assertFalse(result["feedbackCalibrated"])

    def test_boot_off_queue_output_freshness_and_sensor_space_fail_closed(self):
        cases = [
            lambda s: s["actions"].update(bootId="c"*32),
            lambda s: s["status"]["motors"]["M4"].update(speed=10),
            lambda s: s["status"].update(state="ARMED"),
            lambda s: s["sequence"]["sequence"].update(totalCount=1),
            lambda s: s["status"]["shoulderAngle"].update(sensorFresh=False),
            lambda s: s["status"]["shoulderAngle"].update(ageMs=151),
            lambda s: s["status"]["shoulderAngle"].update(angleDeg=168.48),
            lambda s: s["status"]["shoulderAngle"].update(angleDeg=193.23, rawDeg=217.58),
            lambda s: s["action_outputs"]["entries"].extend([
                dict(sequence=6,motor=4,pwm=-10,atMs=600),dict(sequence=7,motor=4,pwm=0,atMs=700)]),
            lambda s: s.update(hostClockId="different-clock"),
        ]
        for index, mutate in enumerate(cases):
            with self.subTest(case=index):
                snapshot = copy.deepcopy(self.after)
                mutate(snapshot)
                manifest = copy.deepcopy(self.manifest)
                manifest["afterSnapshotSha256"] = self.write("after.json", snapshot)
                with self.assertRaises(ValueError):
                    infer_joint_reference(manifest, self.root)
        self.write("after.json", self.after)

    def test_domain_units_hashes_branch_and_timing_are_checked(self):
        for change in [dict(unit="cm"), dict(elbowHeightRangeMm=[200, 200]),
                       dict(elbowHeightRangeMm=[True, 145]), dict(elbowHeightRangeMm=[float("nan"), 145])]:
            height = dict(self.height, **change)
            manifest = copy.deepcopy(self.manifest)
            manifest["heightMeasurementSha256"] = self.write("height.json", height)
            with self.assertRaises(ValueError):
                infer_joint_reference(manifest, self.root)
        self.write("height.json", self.height)
        for change in [lambda m: m.update(beforeSnapshotSha256="0"*64),
                       lambda m: m["image"].update(sourceLatest=17),
                       lambda m: m["image"]["points"].update(elbow=[800, 310]),
                       lambda m: m.update(directionSign=-1)]:
            manifest = copy.deepcopy(self.manifest)
            change(manifest)
            with self.assertRaises(ValueError):
                infer_joint_reference(manifest, self.root)

    def test_cli_writes_only_new_local_candidate_and_never_modifies_sources(self):
        spec = importlib.util.spec_from_file_location("joint_reference_cli", ROOT / "tools/calibrate_arm_joint_reference.py")
        cli = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli)
        source = self.root / "manifest.json"
        self.write(source.name, self.manifest)
        originals = {p.name:p.read_bytes() for p in self.root.iterdir() if p.is_file()}
        output = self.root / ".local/candidate"
        with patch.object(cli, "ROOT", self.root), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                cli.main(["--input", str(source), "--output", str(self.root / "public")])
            self.assertEqual(cli.main(["--input", str(source), "--output", str(output)]), 0)
            self.assertEqual([p.name for p in output.iterdir()], ["joint-reference.json"])
            with self.assertRaises(SystemExit):
                cli.main(["--input", str(source), "--output", str(output)])
        self.assertEqual(originals, {p.name:p.read_bytes() for p in self.root.iterdir() if p.is_file()})


if __name__ == "__main__":
    unittest.main()
