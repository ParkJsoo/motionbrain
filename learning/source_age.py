"""Causal camera age upper bound on one host; not physical clock calibration.

The direct, no-proxy/no-redirect GET handler timestamps *before* the first
socket send. Firmware samples age while serving that request, before headers.
Thus send_start <= server_age_event <= parsed_headers. Send completion does
NOT have this property. Equal clock rates within the request are still assumed.

Fresh evidence additionally places capture AFTER camera request handling. For
camera rate <= host rate * (1 + rho), capture_host >= send_host + delta_lower.
This is a shadow measurement assumption, NOT a measured hardware drift bound.
"""
import math

from learning.fresh_capture import validate_fresh_capture
from learning.http_timing import TRACE_STAMPS, SEND_START_SEMANTICS
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers

FRESH_CLOCK_RATE_ERROR_PPM = 1000
FRESH_TIMESTAMP_ERROR_MS = 0.002


def source_age_upper_ms(basis, reported_age, now):
    """Re-evaluate the same bound at calculation, persistence or consumption."""
    if "sourceCaptureLowerBoundMonotonic" in basis:
        return (now - basis["sourceCaptureLowerBoundMonotonic"]) * 1000
    return reported_age + (now - basis["sourceAgeBoundStartedMonotonic"]) * 1000


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
    proof = validate_fresh_capture(receipt, parsed)
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
    send = trace["requestSendStartedMonotonic"]
    result = {"sourceAgeBoundBasis": "first_get_send_start",
              "sourceAgeBoundStartedMonotonic": send}
    if proof is None:
        return result
    if receipt.get("query") != {"fresh": "1"}:
        raise ValueError("fresh_capture_contract_mismatch")
    rate_error, timestamp_error = FRESH_CLOCK_RATE_ERROR_PPM, FRESH_TIMESTAMP_ERROR_MS
    if (type(rate_error) not in (int, float) or not math.isfinite(rate_error) or rate_error < 0
            or type(timestamp_error) not in (int, float) or not math.isfinite(timestamp_error)
            or timestamp_error < 0):
        raise ValueError("invalid_fresh_clock_error")
    rate = 1 + rate_error / 1e6
    delta_ms = (parsed["capturedUs"] - proof["requestedUs"]) / 1000
    event_delta_ms = delta_ms + parsed["reportedSourceAgeMs"]
    # Selection is capped at 200ms by this firmware contract. Also ensure the
    # camera's request->age-event interval fits inside send->parsed-headers.
    if (delta_ms >= 200 or event_delta_ms / rate - timestamp_error >
            (receipt["headersAvailableMonotonic"] - send) * 1000):
        raise ValueError("fresh_capture_timing_inconsistent")
    lower_ms = max(0.0, delta_ms / rate - timestamp_error)
    result.update(sourceAgeBoundBasis="fresh_capture_lower_bound",
                  sourceCaptureLowerBoundMonotonic=send + lower_ms / 1000,
                  sourceAgeBoundReductionMs=parsed["reportedSourceAgeMs"] + lower_ms,
                  freshCaptureDelayMs=delta_ms, freshCaptureDelayLowerMs=lower_ms,
                  freshClockRateErrorPpm=rate_error, freshTimestampErrorMs=timestamp_error,
                  freshClockErrorCalibrated=False)
    return result
