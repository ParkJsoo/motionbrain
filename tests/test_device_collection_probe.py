"""Read-only commissioning: fake devices and local HTTP only, never real motors."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

from learning.artifacts import ArtifactStore, StorageError
from learning.dataset import build_manifest
from learning.device_adapter import DeviceReadAdapter, DeviceReadError, ReadOnlyHTTP
from learning.device_probe import audit_device_probe, observation_blockers, run_device_probe
from learning.dry_run import JPEG_BYTES, VirtualClock
from learning.runner import CaptureRunner

BOOT = "a" * 32
DIGEST = hashlib.sha256(JPEG_BYTES).hexdigest()


class FakeDevice:
    def __init__(self):
        self.clock = VirtualClock()
        self.host_clock_id = "client-clock"
        self.bases = {"controller": "http://controller", "camera": "http://camera", "perception": "http://perception"}
        self.recorder = None
        self.calls = []
        self.cap = {"schemaVersion": "motionbrain.bounded_action.v1", "bootId": BOOT,
                    "clockDomain": "controller_boot_millis", "enabled": False, "active": False,
                    "recordCount": 0, "recordCapacity": 32}
        self.output = {"bootId": BOOT, "evidence": "driver_write", "clockDomain": "controller_boot_millis",
                       "dropped": 0, "entries": [{"sequence": 1, "pwm": 100}]}
        # An old nonzero output is not current movement. The current status is off.
        self.controller = {"state": "IDLE", "motorEnabled": False,
                           "motors": {f"M{i}": {"speed": 0, "enabled": False} for i in range(1, 6)},
                           "sensor": {"connected": True, "imuOk": True, "simulated": False, "faultLatched": False},
                           "shoulderAngle": {"sensorConnected": True, "sensorFresh": True, "sensorReady": True,
                                             "angleDeg": 200, "ageMs": 5}}
        self.observation = {"sourceFrameId": BOOT + ":1000000", "cameraBootId": BOOT, "capturedUs": 1000000,
                            "imageSha256": DIGEST, "hostClockId": "pi-clock", "receivedMonotonic": 7000,
                            "sourceAgeMs": 3100, "captureHostBoundsCalibrated": False,
                            "sourceFresh": True, "sourceMetadataValid": True, "sourceTimeKnown": True,
                            "controlEligible": True, "held": False,
                            "detectionSourceFrameId": BOOT + ":1000000", "detector": {"latencyMs": 2900}}
        self.fault = None
        self.lookup_record = None

    def get(self, device, path, query=None):
        self.calls.append((device, path, query))
        start = self.clock.monotonic()
        self.clock.advance(.02)
        receipt = {"device": device, "path": path, "query": query or {}, "method": "GET", "status": 200,
                   "hostClockId": self.host_clock_id, "requestStartedMonotonic": start,
                   "receivedMonotonic": self.clock.monotonic(), "headers": {}}
        body = b""
        if path == "/actions":
            if query:
                receipt.update(status=404, json={"state": "outcome_unknown", "code": "not_found"})
                if self.lookup_record is not None:
                    receipt.update(status=200, json=copy.deepcopy(self.lookup_record))
            else:
                receipt["json"] = copy.deepcopy(self.cap)
        elif path == "/action_outputs":
            receipt["json"] = copy.deepcopy(self.output)
        elif device == "controller":
            receipt["json"] = copy.deepcopy(self.controller)
        elif path == "/api/detection":
            receipt["json"] = copy.deepcopy(self.observation)
        elif path == "/api/observation":
            receipt["json"] = {"observation": copy.deepcopy(self.observation)}
        elif path == "/api/vision_frame":
            body = JPEG_BYTES
        elif path == "/capture":
            body = JPEG_BYTES
            now_us = round((start + .01) * 1e6)
            receipt["headers"] = {"x-motionbrain-camera-boot-id": BOOT,
                "x-motionbrain-frame-id": f"{BOOT}:{now_us-1000}",
                "x-motionbrain-captured-us": str(now_us-1000), "x-motionbrain-source-age-us": "1000",
                "x-motionbrain-clock-domain": "camera_boot_us", "x-motionbrain-timestamp-semantics": "first_dma_buffer"}
        receipt["sha256"] = hashlib.sha256(body).hexdigest()
        if self.fault:
            body = self.fault(device, path, receipt, body)
        if self.recorder:
            self.recorder(receipt)
        return receipt, body


class DeviceProbeTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name) / "probe"
        self.transport = FakeDevice()

    def run_probe(self, **kwargs):
        return run_device_probe(ArtifactStore(self.root), self.transport, samples=2,
                                interval=0, sleep=lambda _: None, **kwargs)

    def test_real_adapter_contract_saves_images_and_never_becomes_training_data(self):
        result = self.run_probe()
        self.assertEqual(result["status"], "completed", result)
        self.assertTrue(result["outputLogUnchanged"])
        self.assertEqual(result["motorCommandsSent"], 0)
        self.assertFalse(result["trainingEligible"])
        self.assertFalse(result["readyForPhysicalCollection"])
        self.assertIn("source_age_exceeds_collection_policy", result["blockers"])
        self.assertTrue(result["clock"]["constantOffsetConsistent"])
        self.assertFalse(result["clock"]["captureHostBoundsCalibrated"])
        self.assertTrue(audit_device_probe(self.root)["ok"])
        saved = json.loads((self.root / "observations/0000.json").read_text())
        self.assertEqual(saved["observation"]["receivedMonotonic"], 7000)
        self.assertEqual(saved["observation"]["hostClockId"], "pi-clock")
        self.assertEqual(saved["retrievalHostClockId"], "client-clock")
        self.assertFalse(saved["clockDomainsMerged"])
        self.assertGreater(saved["sourceAgeUpperBoundAtRetrievalMs"], 3139)
        self.assertLess(saved["sourceAgeUpperBoundAtRetrievalMs"], 3141)
        self.assertEqual((self.root / "images/0/perception.jpg").read_bytes(), JPEG_BYTES)
        manifest = build_manifest([self.root])
        self.assertFalse(manifest["readyForTraining"])
        self.assertEqual(manifest["eligibleCount"], 0)
        self.assertTrue(manifest["inputIssues"])

    def test_actual_adapter_cannot_be_passed_to_synthetic_runner(self):
        with self.assertRaisesRegex(ValueError, "only dry_run"):
            CaptureRunner(ArtifactStore(self.root), DeviceReadAdapter(self.transport))

    def test_execute_always_rejects_without_network(self):
        with self.assertRaisesRegex(DeviceReadError, "physical_execution"):
            DeviceReadAdapter(self.transport).execute({})
        self.assertEqual(self.transport.calls, [])

    def test_failed_missing_action_lookup_cannot_pass_preflight(self):
        for fault_kind in ("timeout", "wrong_404", "server_error", "unexpected_record"):
            with self.subTest(fault_kind=fault_kind):
                self.root = self.root.parent / fault_kind
                self.transport = FakeDevice()
                def fault(device, path, r, body):
                    if path == "/actions" and r["query"]:
                        if fault_kind == "timeout":
                            raise TimeoutError()
                        if fault_kind == "wrong_404":
                            r["json"]["code"] = "route_not_found"
                        elif fault_kind == "server_error":
                            r["status"] = 500
                        elif fault_kind == "unexpected_record":
                            r.update(status=200, json={"state": "running"})
                    return body
                self.transport.fault = fault
                result = self.run_probe()
                self.assertEqual(result["status"], "aborted")
                self.assertEqual(result["errors"][0]["reason"], "missing_action_lookup_not_verified")
                self.assertEqual(result["motorCommandsSent"], 0)
                self.assertFalse(any(c[0] == "camera" for c in self.transport.calls))

    def test_404_exact_image_never_falls_back_to_latest(self):
        def fault(device, path, r, body):
            if path == "/api/vision_frame":
                r["status"] = 404
            return body
        self.transport.fault = fault
        result = self.run_probe()
        self.assertEqual(result["status"], "aborted")
        image_calls = [c for c in self.transport.calls if c[1] == "/api/vision_frame"]
        self.assertEqual(len(image_calls), 1)
        self.assertEqual(image_calls[0][2], {"frameId": BOOT + ":1000000"})
        self.assertTrue(audit_device_probe(self.root)["ok"])

    def test_hash_mismatch_retains_actual_bytes_and_halts(self):
        self.transport.fault = lambda device, path, r, body: body + b"bad" if path == "/api/vision_frame" else body
        result = self.run_probe()
        self.assertEqual(result["errors"][0]["reason"], "exact_image_hash_mismatch")
        self.assertEqual((self.root / "images/0/perception.jpg").read_bytes(), JPEG_BYTES + b"bad")
        self.assertEqual(result["completedSamples"], 0)

    def test_observation_identity_mismatch_is_rejected(self):
        def fault(device, path, r, body):
            if path == "/api/observation":
                r["json"]["observation"]["sourceFrameId"] = BOOT + ":2"
            return body
        self.transport.fault = fault
        self.assertEqual(self.run_probe()["errors"][0]["reason"], "exact_observation_mismatch")

    def test_restart_and_new_output_are_not_silent_success(self):
        for field in ("controller_boot", "perception_boot", "camera_boot", "new_output"):
            with self.subTest(field=field):
                self.root = self.root.parent / field
                self.transport = FakeDevice()
                counts = {}
                def fault(device, path, r, body):
                    counts[path] = counts.get(path, 0) + 1
                    if field == "controller_boot" and path == "/actions" and not r["query"] and counts[path] > 1:
                        r["json"]["bootId"] = "b" * 32
                    if field == "perception_boot" and path == "/api/detection" and counts[path] > 1:
                        self.transport.observation["hostClockId"] = "new-pi-clock"
                        r["json"]["hostClockId"] = "new-pi-clock"
                    if field == "camera_boot" and path == "/capture" and counts[path] > 1:
                        r["headers"]["x-motionbrain-camera-boot-id"] = "b" * 32
                        r["headers"]["x-motionbrain-frame-id"] = "b" * 32 + ":" + r["headers"]["x-motionbrain-captured-us"]
                    if field == "new_output" and path == "/action_outputs" and counts[path] > 1:
                        r["json"]["entries"].append({"sequence": 2, "pwm": 0})
                    return body
                self.transport.fault = fault
                self.assertEqual(self.run_probe()["status"], "aborted")

    def test_output_drop_or_boot_mismatch_is_not_complete_evidence(self):
        self.transport.output["bootId"] = "b" * 32
        self.assertEqual(self.run_probe()["errors"][0]["reason"], "output_contract_mismatch")

    def test_m4_display_ready_does_not_override_collection_age_limit(self):
        self.transport.controller["shoulderAngle"]["ageMs"] = 25
        result = self.run_probe()
        self.assertIn("m4_status_age_exceeds_collection_policy", result["blockers"])
        self.assertFalse(result["readyForPhysicalCollection"])

    def test_current_nonzero_output_aborts_without_image_requests(self):
        self.transport.controller["motors"]["M4"]["speed"] = 10
        self.assertEqual(self.run_probe()["status"], "aborted")
        self.assertFalse(any(c[0] == "perception" for c in self.transport.calls))

    def test_interrupt_is_durable_and_not_retried(self):
        def fault(device, path, r, body):
            if path == "/capture":
                raise KeyboardInterrupt()
            return body
        self.transport.fault = fault
        result = self.run_probe()
        self.assertEqual(result["status"], "interrupted")
        self.assertTrue(audit_device_probe(self.root)["ok"])
        self.assertEqual(sum(c[1] == "/capture" for c in self.transport.calls), 1)

    def test_storage_failure_leaves_auditable_incomplete_prefix(self):
        store = ArtifactStore(self.root)
        with patch.object(store, "save_image", side_effect=StorageError("disk full")):
            with self.assertRaises(StorageError):
                run_device_probe(store, self.transport, samples=2, interval=0)
        self.assertFalse(audit_device_probe(self.root)["ok"])
        self.assertFalse((self.root / "run.json").exists())

    def test_tampered_image_and_missing_terminal_are_detected(self):
        self.run_probe()
        (self.root / "images/0/perception.jpg").write_bytes(b"corrupt")
        self.assertFalse(audit_device_probe(self.root)["ok"])
        sorted((self.root / "events").glob("*.json"))[-1].unlink()
        self.assertIn("probe_terminal_missing", audit_device_probe(self.root)["issues"])

    def test_lookup_does_not_infer_nonexecution_or_retry(self):
        adapter = DeviceReadAdapter(self.transport)
        request = {"bootId": BOOT, "commandId": "test", "kind": "shoulder", "motor": 4,
                   "durationMs": 100, "targetDegrees": 200, "percent": 30, "pwm": 0}
        self.assertEqual(adapter.lookup_action(request)["executionState"], "outcome_unknown")
        record = {**request, "requestedPwm": 0, "state": "completed", "outputOff": True}
        self.transport.lookup_record = record
        result = adapter.lookup_action(request)
        self.assertEqual(result["executionState"], "known_terminal")
        self.assertFalse(result["physicalArrivalVerified"])
        self.transport.lookup_record["targetDegrees"] = 201
        self.assertEqual(adapter.lookup_action(request)["executionState"], "outcome_unknown")
        self.transport.lookup_record = record.copy()
        self.transport.lookup_record["outputOff"] = False
        self.assertEqual(adapter.lookup_action(request)["executionState"], "outcome_unknown")
        self.assertEqual(len(self.transport.calls), 4)

    def test_policy_does_not_accept_three_second_old_display_fresh_frame(self):
        self.assertIn("source_age_exceeds_collection_policy", observation_blockers(self.transport.observation))

    def test_clock_uncertainty_is_included_in_freshness_blocker(self):
        observation = {**self.transport.observation, "sourceAgeMs": 90,
                       "captureHostBoundsCalibrated": True, "captureHostBoundsUncertaintySeconds": .02}
        self.assertIn("source_age_with_clock_uncertainty_exceeds_collection_policy", observation_blockers(observation))
        self.assertEqual(self.transport.calls, [])

    def test_same_frame_id_changed_bytes_is_detected(self):
        adapter = DeviceReadAdapter(self.transport)
        adapter._frame_identity(BOOT + ":1", "a")
        with self.assertRaisesRegex(DeviceReadError, "different_bytes"):
            adapter._frame_identity(BOOT + ":1", "b")

    def test_nonoverlapping_clock_bounds_never_become_calibration(self):
        adapter = DeviceReadAdapter(self.transport)
        adapter.camera_samples = [
            {"offsetIntervalSeconds": [1.0, 1.1], "sourceFrameId": "a"},
            {"offsetIntervalSeconds": [1.2, 1.3], "sourceFrameId": "b"},
        ]
        report = adapter.clock_report()
        self.assertFalse(report["constantOffsetConsistent"])
        self.assertIsNone(report["offsetIntersectionSeconds"])
        self.assertFalse(report["physicalCalibration"])
        self.assertFalse(report["usableForControl"])

    def test_invalid_camera_timestamp_semantics_abort(self):
        def fault(device, path, r, body):
            if path == "/capture":
                r["headers"]["x-motionbrain-timestamp-semantics"] = "response_time"
            return body
        self.transport.fault = fault
        self.assertEqual(self.run_probe()["errors"][0]["reason"], "camera_source_metadata_invalid")

    def test_missing_observation_and_request_timeout_do_not_retry(self):
        def fault(device, path, r, body):
            if path == "/api/observation":
                raise TimeoutError()
            return body
        self.transport.fault = fault
        self.assertEqual(self.run_probe()["status"], "aborted")
        self.assertEqual(sum(c[1] == "/api/observation" for c in self.transport.calls), 1)
        self.assertTrue(audit_device_probe(self.root)["ok"])


class ReadOnlyTransportTest(unittest.TestCase):
    def setUp(self):
        self.paths = []
        paths = self.paths
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                paths.append(("GET", self.path))
                if self.path == "/status":
                    self.send_response(302)
                    self.send_header("Location", "/motor")
                    self.end_headers()
                else:
                    self.send_response(404)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"state":"outcome_unknown"}')
            def log_message(self, *args):
                pass
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        origin = f"http://127.0.0.1:{self.server.server_port}"
        self.http = ReadOnlyHTTP(controller_url=origin, camera_url=origin, perception_url=origin)

    def test_redirect_never_follows_to_control_endpoint(self):
        with self.assertRaisesRegex(DeviceReadError, "redirect_refused"):
            self.http.get("controller", "/status")
        self.assertEqual(self.paths, [("GET", "/status")])

    def test_missing_action_returns_404_receipt_and_only_one_get(self):
        receipt, _ = self.http.get("controller", "/actions", {"bootId": BOOT, "commandId": "a&b"})
        self.assertEqual(receipt["status"], 404)
        self.assertEqual(receipt["json"]["state"], "outcome_unknown")
        self.assertEqual(len(self.paths), 1)
        self.assertIn("commandId=a%26b", self.paths[0][1])

    def test_unknown_endpoint_and_control_query_never_reach_network(self):
        for path, query in [("/motor", None), ("/actions", {"kind": "pulse"}), ("/actions/cancel", None)]:
            with self.assertRaises(DeviceReadError):
                self.http.get("controller", path, query)
        self.assertEqual(self.paths, [])

    def test_url_credentials_and_query_rejected(self):
        for url in ("http://user:password@localhost", "http://localhost?token=secret", "http://localhost/motor"):
            with self.assertRaises(ValueError):
                ReadOnlyHTTP(controller_url=url, camera_url="http://camera", perception_url="http://pi")

    def test_oversized_response_aborts_and_records_failure(self):
        records = []
        self.http.recorder = records.append
        with patch("learning.device_adapter.MAX_IMAGE_BYTES", 4):
            with self.assertRaisesRegex(DeviceReadError, "response_too_large"):
                self.http.get("controller", "/actions")
        self.assertEqual(len(self.paths), 1)
        self.assertEqual(records[0]["error"], "DeviceReadError")

    def test_interval_overrun_is_rejected_even_when_http_completes(self):
        with patch.object(self.http, "clock") as clock:
            clock.monotonic.side_effect = [100, 110, 110, 110, 110]
            with self.assertRaisesRegex(DeviceReadError, "over_budget"):
                self.http.get("controller", "/actions")
        self.assertEqual(len(self.paths), 1)

    def test_header_body_and_receipt_times_are_separate_local_stamps(self):
        with patch.object(self.http, "clock") as clock:
            clock.monotonic.side_effect = [100, 100.02, 100.05, 100.055, 100.055]
            receipt, _ = self.http.get("controller", "/actions")
        self.assertEqual(receipt["headersAvailableMonotonic"], 100.02)
        self.assertEqual(receipt["bodyReadCompletedMonotonic"], 100.05)
        self.assertEqual(receipt["receivedMonotonic"], 100.055)

    def test_regressed_intermediate_stamp_is_rejected(self):
        with patch.object(self.http, "clock") as clock:
            clock.monotonic.side_effect = [100, 100.05, 100.02, 100.06, 100.06]
            with self.assertRaisesRegex(DeviceReadError, "interval_invalid"):
                self.http.get("controller", "/actions")


if __name__ == "__main__":
    unittest.main()
