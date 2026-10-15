"""Diagnostic interval and status-race tests; no real camera or motor requests."""
import copy
import unittest

from learning.capture_timing import capture_timing, summarize_capture_timing

BOOT = "a"*32


def receipt(index=0):
    captured = 5_000_000 + index*1_000_000
    return {"device": "camera", "path": "/capture", "method": "GET", "status": 200,
            "hostClockId": "host-A", "requestStartedMonotonic": 100+index,
            "headersAvailableMonotonic": 100.04+index, "bodyReadCompletedMonotonic": 100.06+index,
            "receivedMonotonic": 100.061+index, "bytes": 3000, "sha256": "c"*64,
            "headers": {"x-motionbrain-camera-boot-id": BOOT,
                        "x-motionbrain-captured-us": str(captured),
                        "x-motionbrain-frame-id": f"{BOOT}:{captured}",
                        "x-motionbrain-source-age-us": "30000",
                        "x-motionbrain-clock-domain": "camera_boot_us",
                        "x-motionbrain-timestamp-semantics": "first_dma_buffer"}}


def status(count):
    return {"cameraBootId": BOOT, "captureRequests": count, "captures": count,
            "captureFailures": 0, "cameraRecoveries": 0, "lastFrameBytes": 3000,
            "lastCaptureMs": 2, "lastWriteMs": 3}


class CaptureTimingTest(unittest.TestCase):
    def test_causal_intervals_do_not_assume_symmetric_network(self):
        value = capture_timing(receipt(), status(5), status(6))
        self.assertTrue(value["valid"])
        self.assertAlmostEqual(value["requestToHeadersMs"], 40)
        self.assertAlmostEqual(value["headersToBodyReadCompleteMs"], 20)
        self.assertAlmostEqual(value["bodyReadToReceiptMs"], 1)
        self.assertAlmostEqual(value["reportedAgeAtReceiptIntervalMs"][0], 51)
        self.assertAlmostEqual(value["reportedAgeAtReceiptIntervalMs"][1], 91)
        self.assertAlmostEqual(value["cameraToHostOffsetIntervalSeconds"][0], 94.97)
        self.assertAlmostEqual(value["cameraToHostOffsetIntervalSeconds"][1], 95.01)
        self.assertTrue(value["serverStatsAssociated"])
        self.assertEqual(value["serverFrameAcquireWaitMs"], 2)
        self.assertIsNone(value["exposureReadoutMs"])
        self.assertIsNone(value["bufferResidenceMs"])
        self.assertFalse(value["controlEligible"])
        self.assertFalse(value["physicalCalibration"])

    def test_other_client_capture_prevents_status_attribution(self):
        value = capture_timing(receipt(),status(5),status(7))
        self.assertTrue(value["valid"])
        self.assertFalse(value["serverStatsAssociated"])
        self.assertNotIn("serverFrameAcquireWaitMs",value)

    def test_failed_capture_or_recovery_prevents_status_attribution(self):
        for field in ("captureRequests", "captureFailures", "cameraRecoveries"):
            with self.subTest(field=field):
                after = status(6)
                after[field] += 1
                self.assertFalse(capture_timing(receipt(),status(5),after)["serverStatsAssociated"])

    def test_boot_size_and_missing_counters_cannot_match(self):
        for changes in ({"cameraBootId":"b"*32}, {"lastFrameBytes":20}, {"captures":None},
                        {"lastCaptureMs":True}, {"lastWriteMs":-1}):
            with self.subTest(changes=changes):
                self.assertFalse(capture_timing(receipt(),status(5),{**status(6),**changes})["serverStatsAssociated"])

    def test_missing_reversed_or_nonfinite_client_timing_invalid(self):
        for changes in ({"headersAvailableMonotonic":None}, {"headersAvailableMonotonic":100.07},
                        {"bodyReadCompletedMonotonic":99}, {"receivedMonotonic":float('nan')},
                        {"hostClockId":""}, {"error":"TimeoutError"}, {"path":"/status"}):
            with self.subTest(changes=changes):
                self.assertFalse(capture_timing({**receipt(),**changes},status(5),status(6))["valid"])

    def test_source_semantics_and_identity_required(self):
        for field,value in (("x-motionbrain-frame-id","wrong"),
                            ("x-motionbrain-timestamp-semantics","response_time")):
            r=receipt()
            r["headers"][field]=value
            self.assertFalse(capture_timing(r,status(5),status(6))["valid"])

    def test_header_names_case_insensitive(self):
        r=receipt()
        r["headers"]={k.upper():v for k,v in r["headers"].items()}
        self.assertTrue(capture_timing(r,status(5),status(6))["valid"])

    def test_consistent_offset_is_not_physical_calibration(self):
        rows=[capture_timing(receipt(i),status(i),status(i+1)) for i in range(3)]
        summary=summarize_capture_timing(rows)
        self.assertTrue(summary["validSeries"])
        self.assertTrue(summary["constantOffsetConsistent"])
        self.assertAlmostEqual(summary["offsetIntersectionWidthMs"],40)
        self.assertFalse(summary["physicalCalibration"])
        self.assertFalse(summary["controlEligible"])
        self.assertIsNone(summary["sensorFrameRateHz"])

    def test_mixed_boot_host_or_duplicate_frames_do_not_merge(self):
        rows=[capture_timing(receipt(i),status(i),status(i+1)) for i in range(2)]
        for field in ("hostClockId","cameraBootId","sourceFrameId","capturedUs"):
            changed=copy.deepcopy(rows)
            changed[1][field]=rows[0][field] if field in ("sourceFrameId","capturedUs") else "other"
            self.assertFalse(summarize_capture_timing(changed)["validSeries"])

    def test_disjoint_offset_and_bad_sample_are_not_calibrated(self):
        rows=[capture_timing(receipt(i),status(i),status(i+1)) for i in range(2)]
        rows[1]["cameraToHostOffsetIntervalSeconds"]=[96,96.1]
        summary=summarize_capture_timing(rows)
        self.assertFalse(summary["constantOffsetConsistent"])
        self.assertIsNone(summary["offsetIntersectionSeconds"])
        rows[1]["valid"]=False
        self.assertFalse(summarize_capture_timing(rows)["validSeries"])
