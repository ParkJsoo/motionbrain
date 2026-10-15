"""Causal camera age upper bound on one host; not physical clock calibration.

The direct, no-proxy/no-redirect GET handler timestamps *before* the first
socket send. Firmware samples age while serving that request, before headers.
Thus send_start <= server_age_event <= parsed_headers. Send completion does
NOT have this property. Equal clock rates within the request are still assumed.
"""
import math

from learning.fresh_capture import validate_fresh_capture
from learning.http_timing import TRACE_STAMPS, SEND_START_SEMANTICS
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def source_age_basis(receipt, source, *, request_started, received, host_clock_id):
    """Bind an optional receipt to the exact measurement; fail closed if malformed.

    Old callers/receipts without certified send semantics retain the full request
    bound. The semantics tag is local producer provenance, not authentication.
    """
    fallback = {"sourceAgeBoundBasis": "request_start",
                "sourceAgeBoundStartedMonotonic": request_started}
    if receipt is None:
        return fallback
    if not isinstance(receipt, dict):
        raise ValueError("invalid_capture_receipt")
    if (receipt.get("device"), receipt.get("path"), receipt.get("method"), receipt.get("status")) != (
            "camera", "/capture", "GET", 200) or "error" in receipt:
        raise ValueError("invalid_capture_receipt")
    if (receipt.get("hostClockId") != host_clock_id or not host_clock_id
            or receipt.get("requestStartedMonotonic") != request_started
            or receipt.get("receivedMonotonic") != received
            or receipt.get("sha256") != source.get("imageSha256")):
        raise ValueError("capture_receipt_mismatch")
    headers = receipt.get("headers")
    if not isinstance(headers, dict):
        raise ValueError("invalid_capture_receipt")
    parsed = parse_capture_headers(headers)
    if not parsed["sourceMetadataValid"] or any(source.get(k) != parsed[k] for k in (
            "sourceFrameId", "cameraBootId", "capturedUs", "reportedSourceAgeMs",
            "sourceClockDomain", "sourceTimestampSemantics", "sourceMetadataValid")):
        raise ValueError("capture_receipt_mismatch")
    validate_fresh_capture(receipt, parsed)
    trace = receipt.get("connectionTiming")
    if trace is None and "connectionTiming" not in receipt:
        return fallback
    if not isinstance(trace, dict):
        raise ValueError("invalid_connection_timing")
    stamps = [request_started, *(trace.get(k) for k in TRACE_STAMPS),
              receipt.get("headersAvailableMonotonic"), receipt.get("bodyReadCompletedMonotonic"), received]
    if (not all(type(v) in (int, float) and math.isfinite(v) for v in stamps)
            or stamps != sorted(stamps) or type(trace.get("addressAttempts")) is not int
            or trace["addressAttempts"] < 1):
        raise ValueError("invalid_connection_timing")
    if trace.get("sendStartSemantics") != SEND_START_SEMANTICS:
        return fallback
    return {"sourceAgeBoundBasis": "first_get_send_start",
            "sourceAgeBoundStartedMonotonic": trace["requestSendStartedMonotonic"]}
