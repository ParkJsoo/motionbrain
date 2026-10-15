"""Read-only device adapter for commissioning the real collection path.

No credentials, POST, motor execution, retry, or redirect support. Remote
perception timestamps retain their own hostClockId; they are never subtracted
from this client's monotonic clock. This is not a physical action collector.
"""
from __future__ import annotations

import hashlib
import math
import re
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

from learning.artifacts import MAX_IMAGE_BYTES, _decode_json
from learning.runner import _action_matches
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


class DeviceReadError(RuntimeError):
    pass


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise DeviceReadError("redirect_refused")


class ReadOnlyHTTP:
    """Only explicitly listed GET endpoints; bounded response body, no retries."""
    PATHS = {
        "controller": {"/status", "/actions", "/action_outputs"},
        "camera": {"/status", "/capture"},
        "perception": {"/health", "/api/detection", "/api/observation", "/api/vision_frame"},
    }

    def __init__(self, *, controller_url, camera_url, perception_url, timeout=4.0,
                 clock=time, recorder=None):
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= 30:
            raise ValueError("timeout must be in (0, 30] seconds")
        self.bases = {}
        for name, url in (("controller", controller_url), ("camera", camera_url), ("perception", perception_url)):
            parsed = urllib.parse.urlsplit(url)
            if (parsed.scheme not in ("http", "https") or not parsed.hostname or parsed.username
                    or parsed.password or parsed.path not in ("", "/") or parsed.query or parsed.fragment):
                raise ValueError("device URL must be an HTTP(S) origin without credentials")
            self.bases[name] = url.rstrip("/")
        self.timeout, self.clock, self.recorder = timeout, clock, recorder
        self.host_clock_id = uuid.uuid4().hex
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
        self.get_calls = 0

    def get(self, device, path, query=None):
        if path not in self.PATHS.get(device, set()):
            raise DeviceReadError("endpoint_not_read_only")
        allowed = {"frameId"} if path in ("/api/observation", "/api/vision_frame") else (
            {"bootId", "commandId"} if path == "/actions" else set())
        if query and not set(query).issubset(allowed):
            raise DeviceReadError("unsupported_query")
        url = self.bases[device] + path
        if query:
            url += "?" + urllib.parse.urlencode(query)
        started = self.clock.monotonic()
        receipt = {"device": device, "path": path, "query": query or {}, "method": "GET",
                   "hostClockId": self.host_clock_id, "requestStartedMonotonic": started}
        self.get_calls += 1
        try:
            try:
                response = self.opener.open(urllib.request.Request(url, method="GET"), timeout=self.timeout)
            except urllib.error.HTTPError as exc:
                response = exc  # 404/409 are evidence, never a reason to retry.
            with response:
                body = response.read(MAX_IMAGE_BYTES + 1)
                receipt.update(status=response.code, headers={k.lower(): v for k, v in response.headers.items()
                    if k.lower().startswith("x-motionbrain-") or k.lower() in ("content-type", "content-length")})
            if len(body) > MAX_IMAGE_BYTES:
                raise DeviceReadError("response_too_large")
            receipt.update(bytes=len(body), sha256=hashlib.sha256(body).hexdigest())
            if "json" in receipt["headers"].get("content-type", ""):
                receipt["json"] = _decode_json(body)
            ended = self.clock.monotonic()
            receipt["receivedMonotonic"] = ended
            if ended < started or ended - started > self.timeout:
                # Socket timeout is per blocking operation, not an end-to-end
                # guarantee. Reject overruns even if a slow peer completed.
                raise DeviceReadError("request_interval_invalid_or_over_budget")
            return receipt, body
        except Exception as exc:
            receipt["error"] = type(exc).__name__
            raise
        finally:
            receipt.setdefault("receivedMonotonic", self.clock.monotonic())
            if self.recorder:
                self.recorder(receipt)


def _object(receipt, status=200):
    if receipt.get("status") != status or not isinstance(receipt.get("json"), dict):
        raise DeviceReadError("unexpected_http_status_or_json")
    return receipt["json"]


def _hex_id(value):
    return isinstance(value, str) and re.fullmatch(r"[0-9a-f]{32}", value) is not None


class DeviceReadAdapter:
    mode = "device_read_only"

    def __init__(self, transport):
        self.transport = transport
        self.host_clock_id = transport.host_clock_id
        self.clock = transport.clock
        self.camera_samples = []
        self.frame_hashes = {}
        self.camera_boot_id = None
        self.perception_clock_id = None
        self.controller_boot_id = None

    def execute(self, request):
        raise DeviceReadError("physical_execution_not_implemented_in_read_only_adapter")

    def capability(self):
        receipt, _ = self.transport.get("controller", "/actions")
        value = _object(receipt)
        if (value.get("schemaVersion") != "motionbrain.bounded_action.v1"
                or value.get("clockDomain") != "controller_boot_millis" or not _hex_id(value.get("bootId"))
                or type(value.get("enabled")) is not bool or type(value.get("active")) is not bool):
            raise DeviceReadError("unsupported_controller_capability")
        if self.controller_boot_id is not None and value["bootId"] != self.controller_boot_id:
            raise DeviceReadError("controller_rebooted")
        self.controller_boot_id = value["bootId"]
        return value

    def status(self):
        receipt, _ = self.transport.get("controller", "/status")
        return _object(receipt)

    def outputs(self):
        receipt, _ = self.transport.get("controller", "/action_outputs")
        value = _object(receipt)
        if (self.controller_boot_id is None or value.get("bootId") != self.controller_boot_id
                or value.get("evidence") != "driver_write"
                or value.get("clockDomain") != "controller_boot_millis"
                or type(value.get("dropped")) is not int or value["dropped"] < 0
                or not isinstance(value.get("entries"), list)):
            raise DeviceReadError("output_contract_mismatch")
        previous = 0
        for entry in value["entries"]:
            if (not isinstance(entry, dict) or type(entry.get("sequence")) is not int
                    or entry["sequence"] <= previous or type(entry.get("pwm")) is not int
                    or not -255 <= entry["pwm"] <= 255):
                raise DeviceReadError("output_entry_invalid")
            previous = entry["sequence"]
        return value

    def lookup_action(self, request):
        """Read-only reconciliation evidence. Never updates a host journal.

        Missing/boot mismatch/transport failure cannot prove non-execution.
        A matching terminal MCU record still does not prove physical arrival.
        """
        if (not _hex_id(request.get("bootId")) or not isinstance(request.get("commandId"), str)
                or not 1 <= len(request["commandId"]) <= 128):
            raise ValueError("action identity required")
        try:
            receipt, _ = self.transport.get("controller", "/actions", {
                "bootId": request["bootId"], "commandId": request["commandId"]})
            payload = receipt.get("json")
            if (receipt.get("status") == 404 and isinstance(payload, dict)
                    and payload.get("code") == "not_found" and payload.get("state") == "outcome_unknown"):
                return {"executionState": "outcome_unknown", "lookupStatus": "not_found",
                        "automaticRetry": False, "physicalArrivalVerified": False}
            record = _object(receipt)
            action = {"request": request, "record": record}
            if not _action_matches(action, request):
                raise DeviceReadError("action_identity_or_payload_mismatch")
            if record.get("state") not in ("running", "completed", "timed_out", "cancelled", "failed"):
                raise DeviceReadError("action_state_unknown")
            terminal = record["state"] != "running" and record.get("outputOff") is True
            return {"executionState": "known_terminal" if terminal else "outcome_unknown",
                    "lookupStatus": "matched_record", "record": record,
                    "automaticRetry": False, "physicalArrivalVerified": False}
        except (DeviceReadError, OSError, ValueError):
            return {"executionState": "outcome_unknown", "lookupStatus": "unverified", "automaticRetry": False,
                    "physicalArrivalVerified": False}

    def _frame_identity(self, frame_id, digest):
        previous = self.frame_hashes.get(frame_id)
        if previous is not None and previous != digest:
            raise DeviceReadError("same_frame_id_different_bytes")
        self.frame_hashes[frame_id] = digest

    def observe(self, save_image):
        latest, _ = self.transport.get("perception", "/api/detection")
        latest = _object(latest)
        frame_id = latest.get("sourceFrameId")
        if not isinstance(frame_id, str) or not re.fullmatch(r"[0-9a-f]{32}:[1-9][0-9]{0,18}", frame_id):
            raise DeviceReadError("observation_source_identity_missing")
        exact, _ = self.transport.get("perception", "/api/observation", {"frameId": frame_id})
        observation = _object(exact).get("observation")
        if (not isinstance(observation, dict) or observation.get("sourceFrameId") != frame_id
                or observation.get("cameraBootId") != frame_id.split(":")[0]
                or type(observation.get("capturedUs")) is not int
                or str(observation["capturedUs"]) != frame_id.split(":")[1]
                or observation.get("imageSha256") != latest.get("imageSha256")
                or observation.get("hostClockId") != latest.get("hostClockId")
                or not isinstance(observation.get("hostClockId"), str) or not observation["hostClockId"]):
            raise DeviceReadError("exact_observation_mismatch")
        if self.perception_clock_id is not None and observation["hostClockId"] != self.perception_clock_id:
            raise DeviceReadError("perception_restarted")
        self.perception_clock_id = observation["hostClockId"]
        frame, body = self.transport.get("perception", "/api/vision_frame", {"frameId": frame_id})
        if frame.get("status") != 200:
            raise DeviceReadError("exact_image_missing")
        image = save_image(body, observation.get("imageSha256"))
        if not image["hashMatches"]:
            raise DeviceReadError("exact_image_hash_mismatch")
        if not body.startswith(b"\xff\xd8") or not body.endswith(b"\xff\xd9"):
            raise DeviceReadError("jpeg_markers_invalid")
        self._frame_identity(frame_id, image["sha256"])
        age = observation.get("sourceAgeMs")
        # The producer computed age during the exact-observation response.
        # Add the entire request interval plus subsequent image-fetch time.
        # Only local durations are used; remote absolute timestamps stay opaque.
        age_bound = (age + (self.clock.monotonic() - exact["requestStartedMonotonic"]) * 1000
                     if type(age) in (int, float) and math.isfinite(age) and age >= 0 else None)
        # Keep producer timing unchanged. Do not label the later M4/status
        # request as a simultaneous angle measurement for this old image.
        return {"observation": observation, "image": image,
                "retrievalHostClockId": self.host_clock_id,
                "retrievedMonotonic": frame["receivedMonotonic"],
                "sourceAgeUpperBoundAtRetrievalMs": age_bound,
                "clockDomainsMerged": False, "trainingEligible": False}

    def sample_camera_clock(self, save_image):
        receipt, body = self.transport.get("camera", "/capture")
        if receipt.get("status") != 200:
            raise DeviceReadError("camera_capture_failed")
        metadata = parse_capture_headers(receipt["headers"])
        if not metadata["sourceMetadataValid"]:
            raise DeviceReadError("camera_source_metadata_invalid")
        image = save_image(body, receipt["sha256"])
        if not body.startswith(b"\xff\xd8") or not body.endswith(b"\xff\xd9"):
            raise DeviceReadError("jpeg_markers_invalid")
        boot = metadata["cameraBootId"]
        if self.camera_boot_id is not None and self.camera_boot_id != boot:
            raise DeviceReadError("camera_rebooted")
        self.camera_boot_id = boot
        self._frame_identity(metadata["sourceFrameId"], image["sha256"])
        # source age is evaluated when the CAM response metadata is assembled.
        camera_now_us = metadata["capturedUs"] + int(receipt["headers"]["x-motionbrain-source-age-us"])
        start, end = receipt["requestStartedMonotonic"], receipt["receivedMonotonic"]
        if self.camera_samples and (camera_now_us <= self.camera_samples[-1]["cameraNowUs"]
                                    or start < self.camera_samples[-1]["receivedMonotonic"]):
            raise DeviceReadError("clock_sample_regressed")
        sample = {**metadata, "cameraNowUs": camera_now_us, "hostClockId": self.host_clock_id,
                  "requestStartedMonotonic": start, "receivedMonotonic": end,
                  "rttMs": (end - start) * 1000,
                  "offsetIntervalSeconds": [start - camera_now_us / 1e6, end - camera_now_us / 1e6],
                  "captureHostBoundsCalibrated": False, "image": image}
        self.camera_samples.append(sample)
        return sample

    def clock_report(self):
        samples = self.camera_samples
        low = max((s["offsetIntervalSeconds"][0] for s in samples), default=None)
        high = min((s["offsetIntervalSeconds"][1] for s in samples), default=None)
        return {"schemaVersion": "motionbrain.clock_probe.v1", "hostClockId": self.host_clock_id,
                "cameraBootId": self.camera_boot_id, "samples": len(samples),
                "uniqueFrames": len({s["sourceFrameId"] for s in samples}),
                "constantOffsetConsistent": len(samples) >= 2 and low <= high,
                "offsetIntersectionSeconds": [low, high] if samples and low <= high else None,
                "physicalCalibration": False, "captureHostBoundsCalibrated": False,
                "usableForControl": False,
                "limitations": ["equal_clock_rate_assumption_not_calibrated",
                                "first_dma_timestamp_not_optically_verified",
                                "no_cross_host_monotonic_mapping", "no_future_drift_bound"]}
