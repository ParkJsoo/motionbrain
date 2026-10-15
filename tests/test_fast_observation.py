"""Synthetic image translations and GET-only fake devices; no physical motion."""
import copy
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from learning.artifacts import ArtifactStore, read_events, verify_artifacts
from learning.dataset import build_manifest
from learning.dry_run import VirtualClock
from learning.fast_observation import AnchoredRoiMeasurement, MeasurementError
from learning.fast_probe import run_fast_probe
from test_device_collection_probe import FakeDevice

BOOT = "a" * 32
TEXTURE = np.random.default_rng(34).integers(20, 235, (40, 48), dtype=np.uint8)


def jpeg(dx=0, dy=0, *, lost=False, duplicate=False, scale=1):
    image = np.full((160, 240), 50, dtype=np.uint8)
    if not lost:
        target = cv2.resize(TEXTURE, (round(48*scale), round(40*scale)))
        h, w = target.shape
        image[60+dy:60+dy+h, 64+dx:64+dx+w] = target
        if duplicate:
            image[60:100, 170:218] = TEXTURE
    return cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()


def metadata(body, captured=1_000_000, boot=BOOT):
    return {"sourceFrameId": f"{boot}:{captured}", "cameraBootId": boot, "capturedUs": captured,
            "sourceMetadataValid": True, "sourceClockDomain": "camera_boot_us",
            "sourceTimestampSemantics": "first_dma_buffer", "reportedSourceAgeMs": 10,
            "imageSha256": hashlib.sha256(body).hexdigest()}


def anchor(body):
    source = metadata(body)
    return {**source, "detected": True, "liveDetected": True, "held": False,
            "detectionSourceFrameId": source["sourceFrameId"], "confidence": .9,
            "width": 240, "height": 160, "label": "synthetic_fixture",
            "hostClockId": "producer-clock", "sourceAgeMs": 3000,
            "targetBox": {"x": 64, "y": 60, "width": 48, "height": 40}}


class MeasurementTest(unittest.TestCase):
    def setUp(self):
        self.clock = VirtualClock()
        self.body = jpeg()
        self.tracker = AnchoredRoiMeasurement(anchor(self.body), self.body, clock=self.clock)

    def measure(self, body=None, **changes):
        body = self.body if body is None else body
        source = {**metadata(body, 2_000_000), **changes}
        now = self.clock.monotonic()
        return self.tracker.measure(body, source, request_started=now-.02, received=now,
                                    host_clock_id="local-clock")

    def rejected(self, result, reason):
        self.assertFalse(result["accepted"], result)
        self.assertEqual(result["reason"], reason)
        self.assertNotIn("targetBox", result)
        self.assertNotIn("deltaX", result)
        self.assertFalse(result["controlEligible"])
        self.assertFalse(result["trainingEligible"])

    def test_translation_measured_from_current_original_not_old_coordinates(self):
        result = self.measure(jpeg(13, -7))
        self.assertTrue(result["accepted"], result)
        self.assertEqual((result["deltaX"], result["deltaY"]), (13, -7))
        self.assertNotEqual(result["sourceFrameId"], result["semanticAnchor"]["sourceFrameId"])
        self.assertNotEqual(result["imageSha256"], result["semanticAnchor"]["imageSha256"])
        self.assertFalse(result["controlEligible"])

    def test_identical_pixels_new_capture_are_not_repeated_identity(self):
        self.assertTrue(self.measure()["accepted"])
        self.rejected(self.measure(), "frame_repeated_or_regressed")

    def test_reboot_rejected(self):
        self.rejected(self.measure(cameraBootId="b"*32, sourceFrameId="b"*32+":2000000"), "camera_rebooted")

    def test_hash_mismatch_rejected(self):
        self.rejected(self.measure(imageSha256="0"*64), "image_hash_mismatch")

    def test_source_mismatch_rejected(self):
        self.rejected(self.measure(capturedUs=42), "invalid_source_identity")

    def test_target_loss_rejected_without_held_coordinates(self):
        self.rejected(self.measure(jpeg(lost=True)), "appearance_mismatch_or_target_lost")

    def test_duplicate_target_rejected(self):
        self.rejected(self.measure(jpeg(duplicate=True)), "ambiguous_match")

    def test_target_loss_cannot_silently_reacquire_identity(self):
        self.rejected(self.measure(jpeg(lost=True)), "appearance_mismatch_or_target_lost")
        self.rejected(self.measure(capturedUs=3_000_000, sourceFrameId=BOOT+":3000000"), "new_anchor_required")

    def test_timing_miss_can_recover_only_on_new_source_frame(self):
        self.rejected(self.measure(reportedSourceAgeMs=81), "source_age_exceeds_policy")
        self.rejected(self.measure(), "frame_repeated_or_regressed")
        self.assertTrue(self.measure(capturedUs=3_000_000, sourceFrameId=BOOT+":3000000")["accepted"])

    def test_large_scale_change_rejected(self):
        for scale in (.8, 1.2):
            with self.subTest(scale=scale):
                self.setUp()
                self.rejected(self.measure(jpeg(scale=scale)), "scale_change_requires_new_anchor")

    def test_large_translation_rejected(self):
        self.rejected(self.measure(jpeg(50)), "translation_out_of_range")

    def test_anchor_expiry_rejected(self):
        self.rejected(self.measure(capturedUs=17_000_000, sourceFrameId=BOOT+":17000000"), "anchor_expired")

    def test_reported_source_age_plus_network_rejected(self):
        self.rejected(self.measure(reportedSourceAgeMs=81), "source_age_exceeds_policy")

    def test_processing_time_also_counts_toward_age(self):
        original = cv2.matchTemplate
        def delayed(*args):
            self.clock.advance(.03)
            return original(*args)
        with patch("learning.fast_observation.cv2.matchTemplate", side_effect=delayed):
            self.rejected(self.measure(), "source_age_exceeds_policy")

    def test_bad_age_and_local_clock_are_rejected(self):
        for age in (None, True, float("nan"), -1):
            with self.subTest(age=age):
                self.setUp()
                self.rejected(self.measure(reportedSourceAgeMs=age), "invalid_local_timing")
        self.setUp()
        result = self.tracker.measure(self.body, metadata(self.body, 2_000_000),
                                      request_started=200, received=199, host_clock_id="host")
        self.rejected(result, "invalid_local_timing")

    def test_held_bad_roi_low_texture_anchor_refused(self):
        for changes in ({"held": True}, {"detectionSourceFrameId": "wrong"},
                        {"confidence": float("nan")}, {"targetBox": {"x": 220, "y": 0, "width": 40, "height": 40}}):
            with self.subTest(changes=changes), self.assertRaises(MeasurementError):
                AnchoredRoiMeasurement({**anchor(self.body), **changes}, self.body)
        body = jpeg(lost=True)
        with self.assertRaisesRegex(MeasurementError, "texture_insufficient"):
            AnchoredRoiMeasurement(anchor(body), body)

    def test_changed_image_dimensions_refused(self):
        body = cv2.imencode(".jpg", np.zeros((80, 80), np.uint8))[1].tobytes()
        self.rejected(self.measure(body), "frame_dimensions_changed")


class FastProbeTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name) / "run"
        self.transport = FakeDevice()
        self.body = jpeg()
        self.transport.observation = anchor(self.body)
        self.capture_count = 0
        def image_fault(device, path, receipt, body):
            if path in ("/api/vision_frame", "/capture"):
                body = self.body
                receipt["sha256"] = hashlib.sha256(body).hexdigest()
            if path == "/capture":
                self.capture_count += 1
                captured = 1_000_000 + self.capture_count * 100_000
                receipt["headers"].update({"x-motionbrain-captured-us": str(captured),
                                          "x-motionbrain-frame-id": f"{BOOT}:{captured}"})
            return body
        self.transport.fault = image_fault

    def run_probe(self, **kwargs):
        return run_fast_probe(ArtifactStore(self.root), self.transport, samples=2, interval=0, **kwargs)

    def test_originals_hashes_results_and_dataset_exclusion(self):
        report = self.run_probe()
        self.assertEqual(report["status"], "completed", report)
        self.assertEqual(report["acceptedMeasurements"], 2)
        self.assertTrue(report["outputLogUnchanged"])
        self.assertEqual(verify_artifacts(self.root, report["artifacts"]), [])
        self.assertEqual((self.root / "images/0/source.jpg").read_bytes(), self.body)
        self.assertFalse(build_manifest([self.root])["readyForTraining"])
        self.assertEqual(report["motorCommandsSent"], 0)
        self.assertTrue(all(path in {"/actions", "/action_outputs", "/status", "/capture",
                                    "/api/detection", "/api/observation", "/api/vision_frame"}
                            for _, path, _ in self.transport.calls))

    def test_interruption_keeps_completed_sample_and_terminal(self):
        def stop(_):
            raise KeyboardInterrupt()
        report = self.run_probe(sleep=stop)
        self.assertEqual(report["status"], "interrupted")
        self.assertEqual(report["completedSamples"], 1)
        events, errors = read_events(self.root)
        self.assertEqual(errors, [])
        self.assertEqual(events[-1]["kind"], "finished")

    def test_missing_and_modified_original_detected(self):
        report = self.run_probe()
        (self.root / "images/0/source.jpg").unlink()
        (self.root / "images/1/source.jpg").write_bytes(b"changed")
        errors = verify_artifacts(self.root, report["artifacts"])
        self.assertTrue(any("cannot read" in e for e in errors))
        self.assertTrue(any("sha256_mismatch" in e for e in errors))

    def test_nonidle_preflight_never_captures(self):
        self.transport.controller["motorEnabled"] = True
        report = self.run_probe()
        self.assertEqual(report["status"], "failed")
        self.assertEqual(self.capture_count, 0)

    def test_camera_transport_failure_recorded_without_retry(self):
        original = self.transport.fault
        def fail(device, path, receipt, body):
            if path == "/capture":
                raise TimeoutError("fixture")
            return original(device, path, receipt, body)
        self.transport.fault = fail
        report = self.run_probe()
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["completedSamples"], 0)
        self.assertEqual(sum(path == "/capture" for _, path, _ in self.transport.calls), 1)

    def test_changed_output_log_fails_even_when_status_remains_idle(self):
        original = self.transport.fault
        def changed(device, path, receipt, body):
            body = original(device, path, receipt, body)
            if path == "/capture":
                self.transport.output["entries"].append({"sequence": 2+self.capture_count, "pwm": 0})
            return body
        self.transport.fault = changed
        report = self.run_probe()
        self.assertEqual(report["status"], "failed")
        self.assertFalse(report["outputLogUnchanged"])

    def test_controller_reboot_during_probe_is_failure(self):
        original = self.transport.fault
        def reboot(device, path, receipt, body):
            body = original(device, path, receipt, body)
            if path == "/capture":
                self.transport.cap["bootId"] = "b"*32
            return body
        self.transport.fault = reboot
        report = self.run_probe()
        self.assertEqual(report["status"], "failed")
        self.assertIn("controller_rebooted", report["error"])

    def test_disk_failure_cannot_report_completed_measurement(self):
        original = ArtifactStore.save_image
        def fail(store, index, phase, body, digest):
            if index == "0":
                raise OSError("disk full fixture")
            return original(store, index, phase, body, digest)
        with patch.object(ArtifactStore, "save_image", fail):
            report = self.run_probe()
        self.assertEqual(report["status"], "failed")
        self.assertEqual(report["completedSamples"], 0)
        self.assertNotIn("measured", [event["kind"] for event in read_events(self.root)[0]])
