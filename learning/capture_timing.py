"""Diagnostic causal intervals, not physical calibration or control eligibility.

The firmware samples source age after fb_get and before writing HTTP headers.
Client open() returns after parsing headers, not necessarily at first byte arrival.
These intervals cannot split exposure/readout from frame-buffer residence.
"""
import math
import statistics

from learning.http_timing import TRACE_STAMPS, connection_metrics
from learning.source_age import source_age_basis

from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def capture_timing(receipt, before, after):
    result = {"schemaVersion": "motionbrain.capture_timing.v1", "valid": False,
              "hostClockId": receipt.get("hostClockId"), "controlEligible": False,
              "physicalCalibration": False, "exposureReadoutMs": None, "bufferResidenceMs": None,
              "serverStatsAssociated": False}
    source = parse_capture_headers(receipt.get("headers", {}))
    result.update(sourceFrameId=source["sourceFrameId"], cameraBootId=source["cameraBootId"],
                  capturedUs=source["capturedUs"], imageSha256=receipt.get("sha256"))
    stamps = [receipt.get(k) for k in ("requestStartedMonotonic", "headersAvailableMonotonic",
                                      "bodyReadCompletedMonotonic", "receivedMonotonic")]
    if (receipt.get("status") != 200 or "error" in receipt or not source["sourceMetadataValid"]
            or (receipt.get("device"), receipt.get("path"), receipt.get("method")) != ("camera", "/capture", "GET")
            or not isinstance(receipt.get("hostClockId"), str) or not receipt["hostClockId"]
            or not all(_number(v) for v in stamps) or stamps != sorted(stamps)):
        result["reason"] = "invalid_source_or_request_timing"
        return result
    start, headers, body, end = stamps
    if "connectionTiming" in receipt:
        trace = receipt["connectionTiming"]
        detail = [trace.get(k) for k in TRACE_STAMPS] if isinstance(trace, dict) else []
        order = [start, *detail, headers]
        if (len(detail) != len(TRACE_STAMPS) or not all(_number(v) for v in order)
                or order != sorted(order) or type(trace.get("addressAttempts")) is not int
                or trace["addressAttempts"] < 1):
            result["reason"] = "invalid_connection_timing"
            return result
        result.update(connection_metrics(receipt))
        result["connectionTiming"] = trace
        result["responseWaitMeaning"] = "request_send_complete_to_parsed_headers_includes_server_network_and_scheduling"
    age = source["reportedSourceAgeMs"]
    headers_map = {str(k).lower(): str(v) for k,v in receipt["headers"].items()}
    camera_now_us = source["capturedUs"] + int(headers_map["x-motionbrain-source-age-us"])
    result.update(valid=True, reportedSourceAgeMs=age, cameraAgeSampledUs=camera_now_us,
                  requestStartedMonotonic=start, headersAvailableMonotonic=headers,
                  bodyReadCompletedMonotonic=body, receivedMonotonic=end,
                  requestToHeadersMs=(headers-start)*1000,
                  headersToBodyReadCompleteMs=(body-headers)*1000,
                  bodyReadToReceiptMs=(end-body)*1000,
                  requestTotalMs=(end-start)*1000,
                  reportedAgeAtReceiptIntervalMs=[age+(end-headers)*1000, age+(end-start)*1000],
                  cameraToHostOffsetIntervalSeconds=[start-camera_now_us/1e6, headers-camera_now_us/1e6],
                  intervalAssumption="equal_clock_rate_within_request_and_firmware_age_sample_before_headers")
    try:
        basis = source_age_basis(receipt, {**source, "imageSha256": receipt.get("sha256")},
                                 request_started=start, received=end, host_clock_id=receipt["hostClockId"])
    except ValueError:
        result.update(valid=False, reason="invalid_age_bound")
        return result
    bound_start = basis["sourceAgeBoundStartedMonotonic"]
    result.update(basis)
    # Keep legacy full-request intervals for direct, same-frame comparison.
    result.update(causalAgeAtReceiptUpperMs=age+(end-bound_start)*1000,
                  causalBoundReductionMs=(bound_start-start)*1000)
    counters = ("captureRequests", "captures", "captureFailures", "cameraRecoveries")
    if not all(isinstance(s, dict) and all(type(s.get(k)) is int and s[k] >= 0 for k in counters)
               for s in (before, after)):
        result["serverStatsReason"] = "status_counters_missing"
    elif not before.get("cameraBootId") == after.get("cameraBootId") == source["cameraBootId"]:
        result["serverStatsReason"] = "camera_boot_mismatch"
    elif (after["captureRequests"] != before["captureRequests"] + 1
          or after["captures"] != before["captures"] + 1
          or after["captureFailures"] != before["captureFailures"]
          or after["cameraRecoveries"] != before["cameraRecoveries"]):
        result["serverStatsReason"] = "interleaved_capture_failure_or_recovery"
    elif (after.get("lastFrameBytes") != receipt.get("bytes")
          or any(type(after.get(k)) is not int or after[k] < 0 for k in ("lastCaptureMs", "lastWriteMs"))):
        result["serverStatsReason"] = "frame_size_or_server_duration_invalid"
    else:
        fresh = receipt.get("query", {}).get("fresh") == "1"
        result.update(serverStatsAssociated=True, serverBodyWriteMs=after["lastWriteMs"],
                      serverTimingMeaning=("fresh_selection_including_fb_get_return_yield_and_socket_write"
                                           if fresh else "fb_get_wait_and_socket_write_not_sensor_exposure_or_network_delivery"))
        result["serverFreshFrameSelectionMs" if fresh else "serverFrameAcquireWaitMs"] = after["lastCaptureMs"]
    return result


def summarize_capture_timing(rows):
    """Report a single boot/host series only; never merge absolute clocks."""
    report = {"schemaVersion": "motionbrain.capture_timing_summary.v1", "samples": len(rows),
              "physicalCalibration": False, "controlEligible": False, "validSeries": False,
              "sensorFrameRateHz": None, "limitations": ["no_optical_timestamp_verification",
              "equal_clock_rate_assumption", "no_future_drift_bound", "client_scheduling_included",
              "source_frame_deltas_are_sampled_not_sensor_frame_periods"]}
    if not rows or any(not r.get("valid") for r in rows):
        report["reason"] = "missing_or_invalid_sample"
        return report
    if len({r["hostClockId"] for r in rows}) != 1 or len({r["cameraBootId"] for r in rows}) != 1:
        report["reason"] = "mixed_host_or_camera_boot"
        return report
    if (len({r["sourceFrameId"] for r in rows}) != len(rows)
            or any(b["capturedUs"] <= a["capturedUs"] or b["requestStartedMonotonic"] < a["receivedMonotonic"]
                   for a,b in zip(rows,rows[1:]))):
        report["reason"] = "repeated_or_regressed_frame_or_request"
        return report
    low = max(r["cameraToHostOffsetIntervalSeconds"][0] for r in rows)
    high = min(r["cameraToHostOffsetIntervalSeconds"][1] for r in rows)
    consistent = len(rows) >= 2 and low <= high
    report.update(validSeries=True, hostClockId=rows[0]["hostClockId"], cameraBootId=rows[0]["cameraBootId"],
                  serverStatsAssociatedSamples=sum(r["serverStatsAssociated"] for r in rows),
                  constantOffsetConsistent=consistent,
                  offsetIntersectionSeconds=[low,high] if consistent else None,
                  offsetIntersectionWidthMs=(high-low)*1000 if consistent else None)
    metrics = ("reportedSourceAgeMs", "requestToHeadersMs", "headersToBodyReadCompleteMs",
               "bodyReadToReceiptMs", "requestTotalMs", "serverFrameAcquireWaitMs", "serverFreshFrameSelectionMs", "serverBodyWriteMs",
               "clientSetupMs", "addressResolutionMs", "tcpConnectMs", "connectedToSendMs",
               "requestSendMs", "requestSentToHeadersMs", "causalBoundReductionMs", "causalAgeAtReceiptUpperMs")
    report["statistics"] = {}
    for key in metrics:
        values = [r[key] for r in rows if key in r]
        if values:
            report["statistics"][key] = {"n": len(values), "min": min(values),
                                         "median": statistics.median(values), "max": max(values)}
    return report
