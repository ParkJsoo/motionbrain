"""Structural split fixtures only; no fixture claims physical eligibility."""
import copy
import hashlib
import unittest

from learning.splits import assign_splits, validate_splits


def row(name, session=None, episode="episode-1"):
    return {"sampleId": name, "sessionId": session or f"session-{name}",
            "episodeId": episode, "transitionId": f"transition-{name}",
            "actionIdentity": {"bootId": f"boot-{name}", "commandId": f"command-{name}"},
            "images": [{"sourceFrameId": f"frame-{name}-{phase}",
                        "sha256": hashlib.sha256(f"{name}-{phase}".encode()).hexdigest(),
                        "artifactRef": f"images/{name}/{phase}.jpg"} for phase in ("before", "after")]}


class DatasetSplitTests(unittest.TestCase):
    def test_deterministic_order_independent_and_no_mutation(self):
        rows = [row(str(i)) for i in range(200)]
        original = copy.deepcopy(rows)
        expected = assign_splits(rows)
        self.assertEqual(expected, assign_splits(rows[::-1]))
        self.assertEqual(rows, original)
        self.assertEqual(validate_splits(rows, expected), [])
        self.assertEqual(set(expected.values()), {"train", "validation", "test"})
        self.assertNotEqual(expected, assign_splits(rows, "different-seed"))

    def test_session_and_episode_are_never_split(self):
        rows = [row("a", "session"), row("b", "session"), row("c", "session", "episode-2")]
        assigned = assign_splits(rows)
        self.assertEqual(len(set(assigned.values())), 1)
        issues = validate_splits(rows, {"a": "train", "b": "test", "c": "validation"})
        self.assertTrue(any("session_leakage" in issue for issue in issues))
        self.assertTrue(any("episode_leakage" in issue for issue in issues))

    def test_transitive_cross_phase_image_and_placement_sharing(self):
        rows = [row(x) for x in "abcd"]
        rows[1]["images"][0]["sha256"] = rows[0]["images"][1]["sha256"]
        rows[2]["images"][1]["sourceFrameId"] = rows[1]["images"][0]["sourceFrameId"]
        rows[2]["splitGroupId"] = rows[3]["splitGroupId"] = "same-placement"
        self.assertEqual(len(set(assign_splits(rows).values())), 1)
        self.assertEqual(validate_splits(rows, assign_splits(rows)), [])

    def test_each_shared_identity_rejects_manually_separated_splits(self):
        for kind in ("sha256", "sourceFrameId", "splitGroupId", "actionIdentity"):
            with self.subTest(kind=kind):
                a, b = row("a"), row("b")
                if kind in ("sha256", "sourceFrameId"):
                    b["images"][0][kind] = a["images"][1][kind]
                elif kind == "splitGroupId":
                    a[kind] = b[kind] = "placement"
                else:
                    b[kind] = copy.deepcopy(a[kind])
                self.assertEqual(len(set(assign_splits([a, b]).values())), 1)
                issues = validate_splits([a, b], {"a": "train", "b": "test"})
                self.assertTrue(any("leakage" in issue for issue in issues), issues)

    def test_same_pixels_within_split_are_allowed(self):
        a, b = row("a"), row("b")
        b["images"][0]["sha256"] = a["images"][1]["sha256"]
        a["images"][0] = copy.deepcopy(a["images"][1])
        self.assertEqual(validate_splits([a, b], {"a": "train", "b": "train"}), [])

    def test_renamed_artifact_paths_do_not_change_assignment(self):
        rows = [row(str(i)) for i in range(20)]
        expected = assign_splits(rows)
        for item in rows:
            for image in item["images"]:
                image["artifactRef"] = "moved/somewhere.jpg"
        self.assertEqual(expected, assign_splits(rows))

    def test_episode_names_are_scoped_to_session(self):
        rows = [row("a", episode="episode-1"), row("b", episode="episode-1")]
        self.assertEqual(validate_splits(rows, {"a": "train", "b": "test"}), [])

    def test_same_command_name_on_different_boots_is_distinct(self):
        rows = [row("a"), row("b")]
        rows[1]["actionIdentity"]["commandId"] = rows[0]["actionIdentity"]["commandId"]
        self.assertEqual(validate_splits(rows, {"a": "train", "b": "test"}), [])

    def test_duplicate_action_and_session_transition_are_invalid_even_in_one_split(self):
        for kind in ("action", "transition"):
            rows = [row("a"), row("b")]
            if kind == "action":
                rows[1]["actionIdentity"] = copy.deepcopy(rows[0]["actionIdentity"])
            else:
                rows[1]["sessionId"] = rows[0]["sessionId"]
                rows[1]["transitionId"] = rows[0]["transitionId"]
            issues = validate_splits(rows, {"a": "train", "b": "train"})
            self.assertTrue(any("duplicate_" in issue for issue in issues), issues)

    def test_duplicate_sample_is_not_collapsed(self):
        rows = [row("a"), row("a")]
        with self.assertRaises(ValueError):
            assign_splits(rows)
        self.assertTrue(any("duplicate_sample" in x for x in validate_splits(rows, {"a": "train"})))

    def test_assignment_coverage_and_label_validation(self):
        rows = [row("a"), row("b")]
        issues = validate_splits(rows, {"a": "training", "extra": "train"})
        self.assertTrue(any("missing_assignment:b" in x for x in issues))
        self.assertTrue(any("unexpected_assignment" in x for x in issues))
        self.assertTrue(any("invalid_split:a" in x for x in issues))
        for assignments in (None, [], {"a": []}, {"a": True}, {1: "train"}):
            self.assertTrue(validate_splits(rows, assignments))

    def test_malformed_rows_fail_closed_without_validator_crash(self):
        invalid = [None, {}, [None]]
        for field, value in (("sampleId", ""), ("sessionId", None), ("episodeId", False),
                             ("transitionId", []), ("splitGroupId", " "),
                             ("actionIdentity", None), ("images", []), ("images", [None])):
            item = row("a"); item[field] = value; invalid.append([item])
        for field, value in (("sha256", "A" * 64), ("sha256", "bad"), ("sourceFrameId", "")):
            item = row("a"); item["images"][0][field] = value; invalid.append([item])
        item = row("a"); item["actionIdentity"]["bootId"] = None; invalid.append([item])
        for rows in invalid:
            with self.subTest(rows=rows):
                with self.assertRaises(ValueError):
                    assign_splits(rows)
                self.assertTrue(validate_splits(rows, {}))
        for seed in (None, "", " ", 1, []):
            with self.assertRaises(ValueError):
                assign_splits([], seed)

    def test_empty_and_tiny_dataset_never_force_balance(self):
        self.assertEqual(assign_splits([]), {})
        self.assertEqual(validate_splits([], {}), [])
        rows = [row("a", "session"), row("b", "session")]
        self.assertEqual(len(set(assign_splits(rows).values())), 1)


if __name__ == "__main__":
    unittest.main()
