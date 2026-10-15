"""Causal semantic-anchor lifetime, using ordering only on the CAM clock.

A fresh checkpoint precedes the exact detection frame on the same camera boot.
Its first GET send therefore lower-bounds both captures on this host clock.
No camera duration, rate bound, offset or timestamp margin is converted.
"""
import math
import re

from learning.source_age import source_age_basis


def semantic_anchor(observation):
    return {k: observation.get(k) for k in (
        "sourceFrameId", "imageSha256", "targetBox", "label", "classId")}


def causal_anchor_basis(evidence, host_clock_id):
    if not isinstance(evidence, dict) or evidence.get("hostClockId") != host_clock_id:
        raise ValueError("anchor_host_mismatch")
    observation, receipt, source = (evidence[k] for k in (
        "anchorObservation", "checkpointReceipt", "checkpointSource"))
    if not all(isinstance(v, dict) for v in (observation, receipt, source)):
        raise ValueError("invalid_anchor_evidence")
    basis = source_age_basis(receipt, source, request_started=receipt["requestStartedMonotonic"],
        received=receipt["receivedMonotonic"], host_clock_id=host_clock_id, fresh_age_mode="send_start")
    captured = observation.get("capturedUs")
    boot = source["cameraBootId"]
    if (type(captured) is not int or not source["capturedUs"] < captured < 10**19
            or observation.get("cameraBootId") != boot
            or observation.get("sourceFrameId") != f"{boot}:{captured}"
            or observation.get("sourceMetadataValid") is not True
            or observation.get("sourceClockDomain") != "camera_boot_us"
            or observation.get("sourceTimestampSemantics") != "first_dma_buffer"
            or not isinstance(observation.get("imageSha256"), str)
            or not re.fullmatch(r"[0-9a-f]{64}", observation["imageSha256"])
            or observation.get("detected") is not True or observation.get("liveDetected") is not True
            or observation.get("held") is not False
            or observation.get("detectionSourceFrameId") != observation["sourceFrameId"]):
        raise ValueError("anchor_not_after_checkpoint")
    retrieved = evidence.get("anchorRetrievedMonotonic")
    if (type(retrieved) not in (int, float) or not math.isfinite(retrieved)
            or retrieved < receipt["receivedMonotonic"]):
        raise ValueError("invalid_anchor_retrieval_clock")
    return {"anchorAgeMode": "causal_send_start",
            "anchorCaptureLowerBoundMonotonic": basis["sourceCaptureLowerBoundMonotonic"],
            "anchorRetrievedMonotonic": retrieved,
            "anchorAgeClockRateAssumptionUsed": False, "anchorAgeTimestampMarginUsed": False}


def anchor_age_upper_ms(basis, now, host_clock_id, expected_host):
    if (host_clock_id != expected_host or type(now) not in (int, float) or not math.isfinite(now)
            or now < basis["anchorRetrievedMonotonic"]):
        raise ValueError("invalid_anchor_host_clock")
    return (now - basis["anchorCaptureLowerBoundMonotonic"]) * 1000
