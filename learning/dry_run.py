"""Device-free synthetic collection adapter with virtual time and original JPEGs.

No network, device access, real sleeps, or runtime image library is used. All
sensor/output/calibration claims here are explicitly synthetic test evidence.
"""
from __future__ import annotations

import base64
import copy
import hashlib
import math
import uuid


SCENARIOS = (
    "complete", "before_missing", "after_missing", "after_image_missing",
    "before_hash_mismatch", "after_hash_mismatch", "unknown_result",
    "dropped_outputs", "stale_before", "interrupt_during_action",
)

# A deterministic real 2x2 RGB JPEG, generated once for this fixture. Repeated
# pixels are intentional: different capture IDs may legitimately hash identically.
JPEG_BYTES = base64.b64decode(
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0aHBwg"
    "JC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/2wBDAQkJCQwLDBgNDRgyIRwhMjIyMjIyMjIyMjIy"
    "MjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjIyMjL/wAARCAACAAIDASIAAhEBAxEB/8QA"
    "FQABAQAAAAAAAAAAAAAAAAAAAAb/xAAUEAEAAAAAAAAAAAAAAAAAAAAA/8QAFAEBAAAAAAAAAAAAAAAA"
    "AAAABf/EABQRAQAAAAAAAAAAAAAAAAAAAAD/2gAMAwEAAhEDEQA/AI4AOff/2Q=="
)


class VirtualClock:
    def __init__(self):
        self._seconds = 100.0

    def monotonic(self):
        return self._seconds

    def advance(self, seconds):
        if type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds < 0:
            raise ValueError("virtual clock advance must be finite and nonnegative")
        self._seconds += seconds


class SyntheticAdapter:
    mode = "dry_run"

    def __init__(self, scenario="complete"):
        if scenario not in SCENARIOS:
            raise ValueError(f"unsupported dry-run scenario: {scenario}")
        self.scenario = scenario
        self.clock = VirtualClock()
        self.host_clock_id = f"synthetic-host-clock-{uuid.uuid4().hex}"
        self.camera_boot_id = uuid.uuid4().hex
        self.mcu_boot_id = uuid.uuid4().hex
        self.execute_calls = 0
        self._frames = {}
        self._sequence = 1

    def observe(self, phase):
        if phase not in ("before", "after"):
            raise ValueError("observation phase must be before or after")
        if self.scenario == f"{phase}_missing":
            raise FileNotFoundError(f"synthetic {phase} observation unavailable")
        captured = self.clock.monotonic()
        self.clock.advance(.02)
        captured_us = round(captured * 1_000_000)
        frame_id = f"{self.camera_boot_id}:{captured_us}"
        self._frames[frame_id] = (JPEG_BYTES, phase)
        digest = hashlib.sha256(JPEG_BYTES).hexdigest()
        if self.scenario == f"{phase}_hash_mismatch":
            digest = "0" * 64
        stale = phase == "before" and self.scenario == "stale_before"
        bounds = {"clockId": self.host_clock_id, "earliest": captured, "latest": captured + .001}
        return {
            "synthetic": True, "provenance": "dry_run_virtual_camera_and_AS5600",
            "observationId": f"synthetic-observation:{frame_id}", "sourceFrameId": frame_id,
            "cameraBootId": self.camera_boot_id, "capturedUs": captured_us,
            "sourceClockDomain": "camera_boot_us", "sourceTimestampSemantics": "first_dma_buffer",
            "sourceTimeKnown": True, "sourceMetadataValid": True, "sourceMetadataError": None,
            "reportedSourceAgeMs": 0.0, "sourceAgeAtReceiveMs": 10.0,
            "sourceAgeMs": 5000.0 if stale else 20.000001,
            "sourceFresh": not stale, "available": True, "detected": True,
            "liveDetected": True, "controlEligible": not stale, "held": False,
            "fresh": not stale, "label": "synthetic_target", "confidence": 1.0,
            "detectionSourceFrameId": frame_id, "imageSha256": digest,
            "hostClockId": self.host_clock_id, "requestStartedMonotonic": captured,
            "receivedMonotonic": captured + .01,
            "inferenceCompletedMonotonic": self.clock.monotonic(),
            "captureHostBounds": bounds,
            "captureHostBoundsBasis": "synthetic_virtual_clock_mapping",
            "captureHostBoundsCalibrated": True,
            "captureHostBoundsUncertaintySeconds": .001,
            "captureHostBoundsEvidenceRef": "synthetic://virtual-clock/calibration",
            "m4": {
                "motor": 4, "sensor": "AS5600", "measured": True,
                "synthetic": True, "angleDegrees": 4.5, "sampleAgeMs": 5.000001,
                "evidenceRef": f"synthetic://AS5600/{phase}/{captured_us}",
                "sampledHostBounds": {
                    "clockId": self.host_clock_id, "earliest": captured + .005,
                    "latest": captured + .005, "method": "synthetic_virtual_clock",
                    "evidenceRef": "synthetic://virtual-clock/calibration",
                },
            },
        }

    def fetch_frame(self, source_frame_id):
        if source_frame_id not in self._frames:
            raise FileNotFoundError("synthetic exact source frame unavailable")
        pixels, phase = self._frames[source_frame_id]
        if self.scenario == "after_image_missing" and phase == "after":
            raise FileNotFoundError("synthetic after image unavailable")
        return pixels

    def request(self, command_id):
        return {"bootId": self.mcu_boot_id, "commandId": command_id, "kind": "pulse",
                "motor": 1, "pwm": 30, "durationMs": 100, "targetDegrees": 0, "percent": 0}

    def execute(self, request):
        self.execute_calls += 1
        if request != self.request(request.get("commandId")):
            raise ValueError("dry-run adapter supports only its canonical pulse request")
        started = self.clock.monotonic()
        if self.scenario in ("unknown_result", "interrupt_during_action"):
            self.clock.advance(.05)
            if self.scenario == "unknown_result":
                raise TimeoutError("synthetic command outcome unknown; do not retry")
            raise KeyboardInterrupt("synthetic interruption during action; outcome unknown")
        self.clock.advance(.1)
        ended = self.clock.monotonic()
        started_ms = round(started * 1000) % (2**32)
        ended_ms = (started_ms + 100) % (2**32)
        record = copy.deepcopy(request)
        record.pop("pwm")
        record.update(requestedPwm=30, ordinal=self.execute_calls, state="completed",
                      reason="duration_elapsed", startedAtMs=started_ms, endedAtMs=ended_ms,
                      elapsedMs=100, overrunMs=0, outputOff=True, targetReached=False)

        def bounds(at):
            return {"clockId": self.host_clock_id, "earliest": at, "latest": at,
                    "method": "synthetic_virtual_clock",
                    "evidenceRef": "synthetic://virtual-clock/action"}

        first = self._sequence
        self._sequence += 2
        return {
            "synthetic": True, "request": copy.deepcopy(request), "record": record,
            "startHostBounds": bounds(started), "outputOffHostBounds": bounds(ended),
            "outputSequenceBounds": {"first": first, "last": first + 1,
                                     "evidenceRef": "synthetic://complete-output-trace"},
            "outputLog": {
                "bootId": self.mcu_boot_id, "clockDomain": "controller_boot_millis",
                "evidence": "driver_write", "synthetic": True,
                "dropped": 1 if self.scenario == "dropped_outputs" else 0,
                "entries": [
                    {"sequence": first, "atMs": started_ms, "actionOrdinal": self.execute_calls,
                     "motor": 1, "pwm": 30, "reason": "driver_write"},
                    {"sequence": first + 1, "atMs": ended_ms, "actionOrdinal": self.execute_calls,
                     "motor": 1, "pwm": 0, "reason": "forced_off"},
                ],
            },
        }

    def settle(self, milliseconds):
        if type(milliseconds) not in (int, float) or not math.isfinite(milliseconds) or milliseconds < 0:
            raise ValueError("settling duration must be finite and nonnegative")
        # One millisecond covers capture uncertainty and another ensures the
        # strict 'after settling' inequality. This advances only virtual time.
        self.clock.advance(milliseconds / 1000.0 + .002)
