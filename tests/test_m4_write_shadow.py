import unittest

from tests.test_m4_write_contract import armed_status
from tools.run_m4_write_shadow import evaluate_shadow_request


class M4WriteShadowTest(unittest.TestCase):
    def test_valid_shadow_records_would_execute_without_post(self):
        evidence = evaluate_shadow_request(
            status_url="http://controller/status",
            target_sensor_deg=237.0,
            command_id="shadow-valid",
            fetch_json_func=lambda _url, _timeout: armed_status(),
        )

        self.assertTrue(evidence["accepted"])
        self.assertTrue(evidence["wouldExecute"])
        self.assertFalse(evidence["transport"]["postAttempted"])
        self.assertFalse(evidence["transport"]["forwarded"])
        self.assertAlmostEqual(237.0, evidence["validatedRequest"]["requestedSensorDeg"], places=6)

    def test_idle_status_is_recorded_as_rejection(self):
        status = armed_status()
        status["state"] = "IDLE"
        evidence = evaluate_shadow_request(
            status_url="http://controller/status",
            target_sensor_deg=237.0,
            command_id="shadow-idle",
            fetch_json_func=lambda _url, _timeout: status,
        )

        self.assertFalse(evidence["accepted"])
        self.assertFalse(evidence["wouldExecute"])
        self.assertEqual("state_not_armed", evidence["reason"])
        self.assertFalse(evidence["transport"]["postAttempted"])

    def test_out_of_range_is_rejected_before_status_fetch(self):
        calls = []
        evidence = evaluate_shadow_request(
            status_url="http://controller/status",
            target_sensor_deg=250.0,
            command_id="shadow-range",
            fetch_json_func=lambda url, timeout: calls.append((url, timeout)) or armed_status(),
        )

        self.assertEqual([], calls)
        self.assertEqual("target_out_of_range", evidence["reason"])
        self.assertFalse(evidence["transport"]["postAttempted"])

    def test_status_failure_is_fail_closed(self):
        evidence = evaluate_shadow_request(
            status_url="http://controller/status",
            target_sensor_deg=237.0,
            command_id="shadow-offline",
            fetch_json_func=lambda _url, _timeout: (_ for _ in ()).throw(OSError("offline")),
        )

        self.assertEqual("status_unavailable", evidence["reason"])
        self.assertFalse(evidence["accepted"])
        self.assertFalse(evidence["transport"]["postAttempted"])


if __name__ == "__main__":
    unittest.main()
