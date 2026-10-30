"""Adversarial synthetic pixels and time bounds; no physical calibration fixtures."""
import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
import unittest

from learning.optical_challenge import evaluate_case, inspect_marker, render_marker, synthetic_case
from tools.check_optical_challenge import main


class OpticalChallengeTest(unittest.TestCase):
    def setUp(self):
        self.case = synthetic_case()

    def report(self):
        return evaluate_case(self.case)

    def stamp(self, name, value):
        self.case["times"][name] = dict(clockId="synthetic-host", ns=value)

    def test_consistent_model_never_produces_calibration_or_eligibility(self):
        old = copy.deepcopy(self.case)
        report = self.report()
        self.assertTrue(report["modelConsistent"])
        self.assertEqual(report["modeledBoundsNs"]["earliest"], 979_000_000)
        self.assertEqual(report["modeledBoundsNs"]["latest"], 1_041_000_000)
        for key in ("opticalExposureVerified", "trainingEligible", "controlEligible", "executionAllowed"):
            self.assertFalse(report[key])
        self.assertTrue(report["missingPhysicalEvidence"])
        self.assertNotIn("method", report)
        self.assertEqual(self.case, old)

    def test_rejects_physical_or_unknown_provenance(self):
        for origin in ("physical", "measured", None):
            self.case["caseOrigin"] = origin
            self.assertEqual(self.report()["blockers"], ["synthetic_case_required"])

    def test_stale_nonce_is_not_a_matching_frame(self):
        self.case["pixels"] = render_marker("fedcba9876543210")
        self.assertEqual(self.report()["marker"]["matchedRows"], [])
        self.assertFalse(self.report()["modelConsistent"])

    def test_one_stale_row_remains_in_denominator(self):
        self.case["pixels"][7] = render_marker("fedcba9876543210", rows=1)[0]
        report = self.report()
        self.assertEqual(report["marker"]["totalRows"], 16)
        self.assertEqual(report["marker"]["rejectedRows"], [7])
        self.assertIn("every_source_row_must_match", report["blockers"])

    def test_blur_and_partial_cell_fail_closed(self):
        for pixel in (65, 128, 191):
            self.case = synthetic_case()
            self.case["pixels"][0][0] = pixel
            self.assertFalse(self.report()["modelConsistent"])

    def test_crc_and_framing_pixels_are_checked(self):
        for column in (0, 150, 300, 415):
            self.case = synthetic_case()
            self.case["pixels"][2][column] ^= 255
            self.assertEqual(self.report()["marker"]["rejectedRows"], [2])

    def test_crop_shift_and_invalid_pixel_types_are_not_corrected(self):
        original = self.case["pixels"][0]
        for row in (original[:-1], original+[0], [True]+original[1:], [-1]+original[1:], [256]+original[1:]):
            self.case["pixels"][0] = row
            self.assertTrue(self.report()["blockers"][0].startswith("marker_invalid:"))

    def test_processed_output_rows_do_not_prove_sensor_row_coverage(self):
        self.case["imageDomain"] = "synthetic_processed_rows"
        self.assertIn("output_rows_do_not_prove_source_rows", self.report()["blockers"])
        self.assertIsNone(self.report()["modeledBoundsNs"])

    def test_whole_frame_support_allows_partial_row_match_only_conditionally(self):
        self.case["imageDomain"] = "synthetic_processed_rows"
        self.case["assumptions"]["supportScope"] = "whole_frame"
        self.case["assumptions"]["supportUpperNs"] = 30_000_000
        self.case["pixels"][0] = render_marker("fedcba9876543210", rows=1)[0]
        report = self.report()
        self.assertTrue(report["modelConsistent"])
        self.assertEqual(report["modeledBoundsNs"]["earliest"], 969_000_000)
        self.assertFalse(report["marker"]["sourceRowCoverageVerified"])

    def test_missing_upper_bound_or_uncertainty_never_invents_zero(self):
        for key in ("supportUpperNs", "uncertaintyNs"):
            self.case = synthetic_case()
            del self.case["assumptions"][key]
            self.assertIsNone(self.report()["modeledBoundsNs"])

    def test_invalid_numeric_bounds_rejected_without_coercion(self):
        for value in (None, True, -1, 1.5, float("nan"), float("inf"), "1000", 2**63):
            self.case["assumptions"]["supportUpperNs"] = value
            self.assertIn("invalid_nanoseconds:supportUpperNs", self.report()["blockers"])

    def test_temporal_mixing_nonce_reuse_and_precomputation_block_model(self):
        for key in ("singleFrame", "notReused", "freshAfterChallenge"):
            self.case = synthetic_case()
            self.case["assumptions"][key] = False
            self.assertIsNone(self.report()["modeledBoundsNs"])

    def test_sum_of_split_exposures_is_not_total_support_span(self):
        self.case["assumptions"]["supportMeaning"] = "sum_of_integration_durations"
        self.assertIn("full_optical_support_span_required", self.report()["blockers"])

    def test_dark_integration_before_marker_still_belongs_to_exposure(self):
        # A 200 ms shutter with photons only during its last 1 ms is not a 1 ms exposure.
        self.case["assumptions"]["supportMeaning"] = "first_to_last_contributing_photon"
        self.case["assumptions"]["supportUpperNs"] = 1_000_000
        self.assertIn("full_optical_support_span_required", self.report()["blockers"])

    def test_storage_cannot_start_inside_expanded_exposure_bounds(self):
        self.stamp("persistenceStarted", 1_040_000_000)
        self.assertIn("persistence_before_exposure_upper_bound", self.report()["blockers"])

    def test_precomputed_nonce_time_and_receipt_order_rejected(self):
        for field, value in (("nonceGenerated", 999_000_000), ("received", 1_000_000_000),
                             ("consumed", 1_044_000_000)):
            self.case = synthetic_case()
            self.stamp(field, value)
            self.assertIn("event_order_invalid", self.report()["blockers"])

    def test_mixed_clock_and_boolean_timestamp_rejected(self):
        self.case["times"]["received"]["clockId"] = "other"
        self.assertIn("clock_mismatch:received", self.report()["blockers"])
        self.case = synthetic_case()
        self.stamp("received", True)
        self.assertIn("invalid_nanoseconds:received", self.report()["blockers"])

    def test_exact_100ms_boundary_and_one_nanosecond_over(self):
        self.case["usage"] = "observation"
        self.stamp("consumed", 1_079_000_000)
        self.assertTrue(self.report()["modelConsistent"])
        self.stamp("consumed", 1_079_000_001)
        self.assertIn("source_age_over_100ms", self.report()["blockers"])

    def test_uncertainty_can_destroy_100ms_margin(self):
        self.case["usage"] = "observation"
        self.stamp("consumed", 1_079_000_000)
        self.case["assumptions"]["uncertaintyNs"] += 1
        self.assertIn("source_age_over_100ms", self.report()["blockers"])

    def test_dispatch_expiration_distinct_from_consumption(self):
        self.stamp("dispatchLatest", 1_079_000_001)
        self.assertIn("dispatch_age_over_100ms", self.report()["blockers"])
        self.assertNotIn("source_age_over_100ms", self.report()["blockers"])

    def test_after_uses_earliest_photon_not_challenge_time(self):
        self.case["usage"] = "after"
        self.stamp("offLatest", 930_000_000)
        self.assertIn("exposure_not_after_OFF_plus_50ms", self.report()["blockers"])
        self.stamp("offLatest", 929_000_000)
        self.assertFalse(self.report()["modelConsistent"])
        self.stamp("offLatest", 928_999_999)
        self.assertTrue(self.report()["modelConsistent"])

    def test_storage_250ms_boundary_and_excess_preserved_even_when_age_fails(self):
        self.case["usage"] = "observation"
        self.stamp("persistenceCompleted", 1_292_000_000)
        self.stamp("consumed", 1_292_000_001)
        self.assertNotIn("persistence_over_250ms", self.report()["blockers"])
        self.stamp("persistenceCompleted", 1_292_000_001)
        self.assertIn("persistence_over_250ms", self.report()["blockers"])

    def test_geometry_does_not_claim_qvga_feasibility(self):
        self.assertEqual(len(render_marker(self.case["nonce"])[0]), 416)
        with self.assertRaises(ValueError):
            inspect_marker([[0]*320], self.case["nonce"])

    def test_cli_preserves_packet_recomputes_and_detects_tampering(self):
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            first, second, third = [Path(temp)/name for name in ("first", "second", "third")]
            self.assertEqual(main(["--synthetic-example", "--output-dir", str(first)]), 0)
            self.assertEqual(main(["--root", str(first), "--output-dir", str(second)]), 0)
            self.assertEqual(json.loads((first/"report.json").read_text()), json.loads((second/"report.json").read_text()))
            (first/"case.json").write_text("{}")
            self.assertEqual(main(["--root", str(first), "--output-dir", str(third)]), 2)
            self.assertFalse(third.exists())

    def test_cli_does_not_overwrite_or_hide_a_failed_case(self):
        from learning.artifacts import ArtifactStore
        with tempfile.TemporaryDirectory() as temp, contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            store = ArtifactStore(Path(temp)/"input")
            self.case["assumptions"]["notReused"] = False
            ref = store.write_json("case.json", self.case)
            store.write_json("manifest.json", dict(case=ref))
            output = Path(temp)/"result"
            args = ["--root", str(store.root), "--output-dir", str(output)]
            self.assertEqual(main(args), 2)
            self.assertFalse(json.loads((output/"report.json").read_text())["modelConsistent"])
            self.assertTrue((output/"case.json").exists())
            before = (output/"report.json").read_bytes()
            self.assertEqual(main(args), 2)
            self.assertEqual(before, (output/"report.json").read_bytes())


if __name__ == "__main__":
    unittest.main()
