"""Camera source identity and conservative host-monotonic freshness contract.

The camera timestamp is first DMA buffer capture, not HTTP response generation.
Host capture bounds include request/response latency; they are only comparable
inside the same hostClockId. No camera/host clock synchronization is assumed.
"""
from __future__ import annotations

import copy
import math
import re
from typing import Any, Mapping


def parse_capture_headers(headers: Mapping[str, str]) -> dict[str, Any]:
    values = {str(k).lower(): str(v) for k, v in headers.items()}
    prefix = "x-motionbrain-"
    boot = values.get(prefix + "camera-boot-id", "")
    frame_id = values.get(prefix + "frame-id", "")
    captured = values.get(prefix + "captured-us", "")
    age = values.get(prefix + "source-age-us", "")
    clock = values.get(prefix + "clock-domain", "")
    semantics = values.get(prefix + "timestamp-semantics", "")
    valid = bool(
        re.fullmatch(r"[0-9a-f]{32}", boot)
        and re.fullmatch(r"[1-9][0-9]{0,18}", captured)
        and re.fullmatch(r"0|[1-9][0-9]{0,18}", age)
        and frame_id == f"{boot}:{captured}"
        and clock == "camera_boot_us"
        and semantics == "first_dma_buffer"
    )
    return {
        "sourceFrameId": frame_id if valid else None,
        "cameraBootId": boot if valid else None,
        "capturedUs": int(captured) if valid else None,
        "sourceClockDomain": clock if valid else None,
        "sourceTimestampSemantics": semantics if valid else None,
        "sourceTimeKnown": valid,
        "sourceMetadataValid": valid,
        "sourceMetadataError": None if valid else "missing_or_invalid_camera_metadata",
        "reportedSourceAgeMs": int(age) / 1000.0 if valid else None,
    }


def refresh_observation(payload: dict[str, Any], now_monotonic: float, stale_seconds: float) -> dict[str, Any]:
    """Copy an observation and derive freshness without mutating frozen evidence."""
    result = copy.deepcopy(payload)
    received = result.get("receivedMonotonic")
    age_at_receive = result.get("sourceAgeAtReceiveMs")
    elapsed = now_monotonic - received if isinstance(received, (int, float)) else float("nan")
    elapsed_valid = math.isfinite(elapsed) and elapsed >= 0
    result["ageMs"] = elapsed * 1000.0 if elapsed_valid else None
    result["fresh"] = elapsed_valid and elapsed <= stale_seconds  # display freshness only
    age = age_at_receive + elapsed * 1000.0 if elapsed_valid and isinstance(age_at_receive, (int, float)) else None
    result["sourceAgeMs"] = age
    result["sourceFresh"] = bool(
        result.get("sourceMetadataValid") is True
        and result.get("sourceTimeKnown") is True
        and age is not None and math.isfinite(age) and 0 <= age <= stale_seconds * 1000.0
    )
    result["controlEligible"] = bool(
        result["sourceFresh"] and result.get("available") is True
        and result.get("detected") is True and result.get("liveDetected") is True
        and result.get("held") is False and result.get("sourceFrameId")
        and result.get("detectionSourceFrameId") == result.get("sourceFrameId")
    )
    return result
