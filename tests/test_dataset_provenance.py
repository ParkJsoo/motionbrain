"""Synthetic test dictionaries exercise provenance policy, never real evidence."""
import copy
import hashlib
import unittest

from learning.provenance import (KNOWN_SYNTHETIC_IMAGE_HASHES,
                                 physical_provenance_reasons, synthetic_reasons)


def declared_physical_contract():
    """In-memory declaration fixture only; never exported as physical records."""
    return [
        {"mode": "physical", "evidence": "physical"},
        {"mode": "physical", "evidence": "physical"},
        {"mode": "physical", "evidence": "physical"},
        {"collection": {"mode": "physical", "evidence": "physical"}},
        {"mode": "physical"},
        {"physicalCalibration": True},
    ]


class DatasetProvenanceTest(unittest.TestCase):
    def test_nested_sensor_output_and_array_markers_are_detected(self):
        document = {"before": {"m4": {"synthetic": True}},
                    "action": {"outputLog": {"synthetic": True}},
                    "items": [{"synthetic": True}, {"synthetic": False}]}
        reasons = synthetic_reasons(document)
        self.assertEqual(len(reasons), 3)
        self.assertTrue(any("before.m4.synthetic" in reason for reason in reasons))
        self.assertTrue(any("action.outputLog.synthetic" in reason for reason in reasons))
        self.assertTrue(any("items[0].synthetic" in reason for reason in reasons))

    def test_semantic_modes_versions_adapters_and_references(self):
        for key, value in (
            ("mode", "dry_run"), ("mode", "dry-run"), ("mode", "synthetic"),
            ("mode", "simulation"), ("evidence", "synthetic"),
            ("firmwareVersion", "synthetic-fw-v1"), ("host_version", "dry_run-v1"),
            ("adapter", "SyntheticAdapter"), ("evidenceRef", "synthetic://clock"),
            ("captureHostBoundsBasis", "synthetic_virtual_clock_mapping"),
            ("provenance", "dry_run_virtual_camera_and_AS5600"),
            ("method", "synthetic"), ("robotId", "synthetic-motionbrain"),
        ):
            with self.subTest(key=key, value=value):
                self.assertTrue(synthetic_reasons({"nested": {key: value}}))
        self.assertTrue(synthetic_reasons({"evidenceRefs": ["physical/path", "synthetic/clock"]}))

    def test_free_text_and_unrelated_values_do_not_imply_synthetic(self):
        self.assertEqual(synthetic_reasons({
            "notes": "synthetic fixtures are excluded from this physical run",
            "description": "simulation is useful for testing",
            "evidenceRef": "records/synthetic-comparison/clock.json",
            "mode": "physical", "synthetic": False,
        }), [])

    def test_marker_absence_never_establishes_physical_provenance(self):
        reasons = physical_provenance_reasons({}, {}, {}, {}, {}, {})
        self.assertEqual(len(reasons), 10)
        self.assertTrue(all(reason.startswith("physical_provenance_required:") for reason in reasons))
        self.assertEqual(len(physical_provenance_reasons(None, [], {}, None, 1, "unknown")), 10)

    def test_each_required_declaration_is_independently_required(self):
        for index, key in ((0, "mode"), (0, "evidence"), (1, "mode"), (1, "evidence"),
                           (2, "mode"), (2, "evidence"), (4, "mode"), (5, "physicalCalibration")):
            with self.subTest(index=index, key=key):
                docs = declared_physical_contract()
                del docs[index][key]
                self.assertTrue(physical_provenance_reasons(*docs))
        for key in ("mode", "evidence"):
            docs = declared_physical_contract()
            del docs[3]["collection"][key]
            self.assertTrue(physical_provenance_reasons(*docs))

    def test_consistent_declarations_pass_without_claiming_measurement_truth(self):
        docs = declared_physical_contract()
        original = copy.deepcopy(docs)
        self.assertEqual(physical_provenance_reasons(*docs), [])
        self.assertEqual(docs, original)

    def test_nested_marker_overrides_all_positive_physical_declarations(self):
        for index, marker in ((0, {"adapter": "SyntheticAdapter"}),
                              (1, {"evidenceRef": "synthetic://run"}),
                              (2, {"provenance": "dry_run_virtual"}),
                              (3, {"before": {"m4": {"synthetic": True}}}),
                              (3, {"action": {"outputLog": {"synthetic": True}}}),
                              (4, {"firmwareVersion": "synthetic-fw"}),
                              (5, {"method": "synthetic"})):
            with self.subTest(index=index, marker=marker):
                docs = declared_physical_contract()
                docs[index].update(marker)
                reasons = physical_provenance_reasons(*docs)
                self.assertTrue(any(reason.startswith("synthetic_marker:") for reason in reasons))

    def test_physical_calibration_requires_boolean_true(self):
        for value in (False, None, 1, "true"):
            docs = declared_physical_contract()
            docs[5]["physicalCalibration"] = value
            self.assertIn("physical_provenance_required:calibration.physicalCalibration",
                          physical_provenance_reasons(*docs))

    def test_known_image_hash_matches_repository_fixture_only(self):
        from learning.dry_run import JPEG_BYTES
        self.assertIn(hashlib.sha256(JPEG_BYTES).hexdigest(), KNOWN_SYNTHETIC_IMAGE_HASHES)
        self.assertNotIn(hashlib.sha256(b"unrecognized image").hexdigest(), KNOWN_SYNTHETIC_IMAGE_HASHES)
        self.assertTrue(physical_provenance_reasons({}, {}, {}, {}, {}, {}))


if __name__ == "__main__":
    unittest.main()
