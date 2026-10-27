import contextlib
import copy
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from learning.cv_dataset import CLASSES, COORDINATES, CVDataset, load_candidate_frames
from learning.cv_display import render_frame, render_frames
from learning.cv_evaluation import direction, evaluate, reference_vector
from tools.offline_cv import main


class OfflineCVTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.manifest = self.root / "manifest.json"
        self.doc = {"schema": "motionbrain.cv_dataset.v1",
                    "classIds": {"background_reserved_not_annotation": 0, "lip_balm": 1, "gripper": 2},
                    "records": [], "splits": {"train": [], "validation": [], "test": []}}
        self.add_frame("pair", [(30, 110, 60, 150), (100, 60, 130, 100)])
        self.add_frame("negative", [None, (100, 60, 130, 100)])
        self.save()

    def add_frame(self, name, boxes, *, width=200, height=200):
        image_id = len(self.doc["records"]) + 1
        image = np.full((height, width, 3), image_id * 20, np.uint8)
        for b in boxes:
            if b is not None:
                cv2.rectangle(image, b[:2], b[2:], (100, 150, 220), -1)
        path = self.root / f"{name}.png"
        self.assertTrue(cv2.imwrite(str(path), image))
        labels = [{"class": cls, "presence": "present" if b else "absent", "bbox_xyxy": list(b) if b else None,
                   "bbox_valid": b is not None, "occlusion": "none" if b else "unknown",
                   "truncated": False if b else "unknown", "humanApproved": True, "reviewStatus": "human_approved"}
                  for cls, b in zip(CLASSES, boxes)]
        self.doc["records"].append({"id": name, "imageId": image_id, "image": path.name,
                                    "sourceSha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                    "width": width, "height": height, "sessionId": "session", "leakageGroup": "group",
                                    "split": "train", "coordinates": COORDINATES, "labels": labels})
        self.doc["splits"]["train"].append(name)

    def save(self):
        self.manifest.write_text(json.dumps(self.doc))
        return self.manifest

    def predictions(self):
        return {"schema": "motionbrain.cv_predictions.v1", "coordinates": COORDINATES,
                "evidenceKind": "synthetic", "frames": [
                    {"id": row["id"], "sourceSha256": row["sourceSha256"], "status": "ok",
                     "detections": [{"class": label["class"], "bbox_xyxy": label["bbox_xyxy"], "score": 0.9}
                                    for label in row["labels"] if label["presence"] == "present"]}
                    for row in self.doc["records"]]}

    def evaluate(self, predictions=None, **kwargs):
        return evaluate(CVDataset(self.save()), predictions if predictions is not None else self.predictions(),
                        score_threshold=0.5, nms_threshold=kwargs.pop("nms_threshold", 0.5), **kwargs)

    def test_loader_rgb_targets_metadata_and_empty_frame(self):
        self.add_frame("empty", [None, None])
        dataset = CVDataset(self.save())
        image, target = dataset[0]
        self.assertEqual(image.shape, (200, 200, 3))
        self.assertEqual(image[120, 40].tolist(), [220, 150, 100])
        self.assertEqual(target["labels"], [1, 2])
        self.assertEqual(target["area"], [1200, 1200])
        self.assertEqual(dataset[1][1]["labels"], [2])
        self.assertEqual(dataset[2][1]["boxes"], [])
        target["boxes"][0][0] = 0
        self.assertEqual(dataset[0][1]["boxes"][0][0], 30)

    def test_unapproved_unknown_and_invalid_labels_fail_closed(self):
        original = copy.deepcopy(self.doc)
        changes = [{"humanApproved": False}, {"presence": "unknown"}, {"bbox_valid": False},
                   {"occlusion": "unknown"}, {"truncated": "unknown"}, {"presence": "absent"},
                   {"bbox_xyxy": [0, 0, 0, 10]}, {"bbox_xyxy": [0, 0, 201, 10]},
                   {"bbox_xyxy": [0, 0, float("nan"), 10]}, {"bbox_xyxy": [True, 0, 10, 10]}]
        for change in changes:
            with self.subTest(change=change):
                self.doc = copy.deepcopy(original)
                self.doc["records"][0]["labels"][0].update(change)
                with self.assertRaises(ValueError):
                    CVDataset(self.save())

    def test_duplicate_class_and_missing_class_rejected(self):
        self.doc["records"][0]["labels"][1]["class"] = "lip_balm"
        with self.assertRaises(ValueError):
            CVDataset(self.save())
        self.doc["records"][0]["labels"].pop()
        with self.assertRaises(ValueError):
            CVDataset(self.save())

    def test_path_traversal_and_escaping_symlink_rejected(self):
        self.doc["records"][0]["image"] = "../escape.png"
        with self.assertRaises(ValueError):
            CVDataset(self.save())
        with tempfile.TemporaryDirectory() as outside:
            (self.root / "escape.png").symlink_to(Path(outside) / "image.png")
            self.doc["records"][0]["image"] = "escape.png"
            with self.assertRaises(ValueError):
                CVDataset(self.save())

    def test_duplicate_identity_and_wrong_class_mapping(self):
        original = copy.deepcopy(self.doc)
        for key in ("id", "imageId", "sourceSha256"):
            self.doc = copy.deepcopy(original)
            self.doc["records"][1][key] = self.doc["records"][0][key]
            with self.subTest(key=key), self.assertRaises(ValueError):
                CVDataset(self.save())
        self.doc = original
        self.doc["classIds"]["lip_balm"] = 2
        with self.assertRaises(ValueError):
            CVDataset(self.save())

    def test_cross_split_session_or_group_rejected_before_filtering(self):
        original = copy.deepcopy(self.doc)
        for changed in ("sessionId", "leakageGroup"):
            self.doc = copy.deepcopy(original)
            self.doc["records"][1]["split"] = "test"
            self.doc["records"][1][changed] = "different"
            self.doc["splits"] = {"train": ["pair"], "test": ["negative"]}
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                CVDataset(self.save(), split="train")

    def test_split_inventory_must_cover_exactly_once(self):
        self.doc["splits"]["train"].append("pair")
        with self.assertRaises(ValueError):
            CVDataset(self.save())
        self.doc["splits"]["train"] = ["pair"]
        with self.assertRaises(ValueError):
            CVDataset(self.save())

    def test_source_bytes_revalidated_at_each_access(self):
        dataset = CVDataset(self.save())
        dataset[0]
        (self.root / "pair.png").write_bytes(b"changed")
        with self.assertRaises(ValueError):
            dataset[0]

    def test_perfect_synthetic_predictions_are_not_model_accuracy(self):
        report = self.evaluate()
        self.assertFalse(report["modelAccuracyMeasured"])
        self.assertEqual(report["metrics"]["pairCoverage"], {"count": 1, "denominator": 1, "rate": 1.0})
        self.assertEqual(report["metrics"]["classes"]["gripper"]["absentFalsePositive"]["rate"], None)
        self.assertEqual(report["metrics"]["classes"]["lip_balm"]["absentClearFrames"], 1)
        self.assertEqual(report["sessions"]["session"]["frames"], 2)

    def test_missing_prediction_and_negative_failure_stay_in_denominators(self):
        predictions = self.predictions()
        predictions["frames"] = []
        report = self.evaluate(predictions)
        self.assertEqual(report["metrics"]["processingFailures"], 2)
        self.assertEqual(report["metrics"]["positionSuccess"]["denominator"], 1)
        self.assertEqual(report["metrics"]["positionSuccess"]["count"], 0)
        self.assertEqual(report["metrics"]["classes"]["gripper"]["fn"], 2)
        negative = report["metrics"]["classes"]["lip_balm"]
        self.assertEqual(negative["absentProcessingFailures"], 1)
        self.assertEqual(negative["absentClearFrames"], 0)

    def test_decode_hash_size_and_prediction_failures_count(self):
        for mode in ("missing", "hash", "decode", "size", "status", "prediction_hash", "invalid_prediction"):
            with self.subTest(mode=mode):
                predictions = self.predictions()
                raw = (self.root / "pair.png").read_bytes()
                original = copy.deepcopy(self.doc)
                if mode == "missing":
                    (self.root / "pair.png").unlink()
                elif mode == "hash":
                    (self.root / "pair.png").write_bytes(b"changed")
                elif mode == "decode":
                    (self.root / "pair.png").write_bytes(b"not a png")
                    self.doc["records"][0]["sourceSha256"] = hashlib.sha256(b"not a png").hexdigest()
                elif mode == "size":
                    self.doc["records"][0]["width"] = 201
                elif mode == "status":
                    predictions["frames"][0]["status"] = "error"
                elif mode == "prediction_hash":
                    predictions["frames"][0]["sourceSha256"] = "0" * 64
                else:
                    predictions["frames"][0]["detections"][0]["score"] = float("nan")
                report = self.evaluate(predictions)
                self.assertEqual(report["metrics"]["processingFailures"], 1)
                self.assertFalse(report["frames"][0]["positionSuccess"])
                self.doc = original
                (self.root / "pair.png").write_bytes(raw)

    def test_highest_score_selected_without_gt_rescue(self):
        p = self.predictions()
        p["frames"][0]["detections"].append({"class": "lip_balm", "score": 0.99, "bbox_xyxy": [1, 1, 20, 20]})
        report = self.evaluate(p)
        self.assertFalse(report["frames"][0]["pairMatched"])
        self.assertEqual(report["metrics"]["classes"]["lip_balm"]["tp"], 1)
        self.assertEqual(report["metrics"]["classes"]["lip_balm"]["fp"], 1)
        self.assertIsNone(report["frames"][0]["errorReferencePx"])

    def test_nms_duplicates_and_score_ties(self):
        p = self.predictions()
        p["frames"][0]["detections"].append(copy.deepcopy(p["frames"][0]["detections"][0]))
        self.assertEqual(self.evaluate(p)["metrics"]["classes"]["lip_balm"]["fp"], 0)
        self.assertEqual(self.evaluate(p, nms_threshold=1)["metrics"]["classes"]["lip_balm"]["fp"], 1)
        p["frames"][0]["detections"].append({"class": "lip_balm", "score": 0.9, "bbox_xyxy": [1, 1, 20, 20]})
        first = self.evaluate(p)["frames"][0]["selectedBoxes"]
        p["frames"][0]["detections"].reverse()
        self.assertEqual(first, self.evaluate(p)["frames"][0]["selectedBoxes"])
        self.assertEqual(first[0], [1, 1, 20, 20])

    def test_absent_false_positive_and_low_score_missing(self):
        p = self.predictions()
        p["frames"][0]["detections"][0]["score"] = 0.49
        p["frames"][1]["detections"].append({"class": "lip_balm", "score": 0.9, "bbox_xyxy": [30, 110, 60, 150]})
        report = self.evaluate(p)
        self.assertEqual(report["metrics"]["processingFailures"], 0)
        self.assertEqual(report["metrics"]["classes"]["lip_balm"]["fn"], 1)
        self.assertEqual(report["metrics"]["classes"]["lip_balm"]["absentFalsePositive"]["rate"], 1)
        self.assertEqual(report["metrics"]["classes"]["lip_balm"]["absentClearFrames"], 0)

    def test_small_error_can_change_direction_and_boundary_is_retained(self):
        self.doc["records"], self.doc["splits"]["train"] = [], []
        self.add_frame("boundary", [(130, 130, 230, 230), (121, 100, 221, 200)], width=960, height=540)
        p = self.predictions()
        p["frames"][0]["detections"][0]["bbox_xyxy"] = [132, 130, 232, 230]
        report = self.evaluate(p)
        r = report["frames"][0]
        self.assertTrue(r["positionSuccess"])
        self.assertFalse(r["directionSuccess"])
        self.assertEqual(r["errorReferencePx"], 2)
        self.assertEqual(report["boundaryStrata"]["near"]["pairCoverage"]["denominator"], 1)

    def test_vector_match_does_not_replace_object_matching(self):
        p = self.predictions()
        for d in p["frames"][0]["detections"]:
            d["bbox_xyxy"] = [x + 20 if i % 2 == 0 else x for i, x in enumerate(d["bbox_xyxy"])]
        r = self.evaluate(p)["frames"][0]
        self.assertFalse(r["pairMatched"])
        self.assertFalse(r["positionSuccess"])

    def test_invalid_envelope_thresholds_and_duplicate_predictions(self):
        for key, value in (("evidenceKind", "unknown"), ("coordinates", "normalized")):
            p = self.predictions()
            p[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.evaluate(p)
        p = self.predictions()
        p["frames"].append(copy.deepcopy(p["frames"][0]))
        with self.assertRaises(ValueError):
            self.evaluate(p)
        p["frames"][-1]["id"] = "outside"
        with self.assertRaises(ValueError):
            self.evaluate(p)
        for value in (float("nan"), -1, 1.1, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.evaluate(nms_threshold=value)

    def test_direction_exact_boundaries_scaling_and_invalid_numbers(self):
        self.assertEqual(direction([-10, 10]), [0, 0])
        self.assertEqual(direction([-10.01, 10.01]), [-1, 1])
        self.assertEqual(reference_vector([20, -20], 1920, 1080), [10, -10])
        self.assertEqual(reference_vector([40, -40], 3840, 2160), [10, -10])
        self.assertEqual(reference_vector([20, -20], 1920, 1200), [9, -9])
        with self.assertRaises(ValueError):
            direction([float("nan"), 0])

    def candidate_files(self):
        labels = {"schema": "motionbrain-p1-reviewed-working-v1", "sessionId": "session", "records": copy.deepcopy(self.doc["records"])}
        captures = {"sessionId": "session", "records": []}
        for row in labels["records"]:
            for label in row["labels"]:
                if label["class"] == "gripper":
                    label.update(candidate_bbox_xyxy=label["bbox_xyxy"], bbox_xyxy=None, bbox_valid=False,
                                 humanApproved=False)
            captures["records"].append({**row, "sha256": row["sourceSha256"]})
        lp, cp = self.root / "labels.json", self.root / "captures.json"
        lp.write_text(json.dumps(labels))
        cp.write_text(json.dumps(captures))
        return lp, cp

    def test_candidate_adapter_cannot_produce_training_targets(self):
        lp, cp = self.candidate_files()
        frames = load_candidate_frames(lp, cp, self.root)
        with self.assertRaises(ValueError):
            frames[0].target()
        with self.assertRaises(ValueError):
            evaluate(frames, self.predictions(), score_threshold=0.5, nms_threshold=0.5)
        with self.assertRaises(ValueError):
            CVDataset(lp)
        image, row = render_frame(frames[0])
        self.assertEqual(image.shape, (660, 960, 3))
        self.assertEqual(row["labelKind"], "candidate_reference")
        self.assertFalse(row["isModelPrediction"])

    def test_render_absent_and_failures_never_reuse_direction(self):
        frames = CVDataset(self.save()).frames
        self.assertIsNotNone(render_frame(frames[0])[1]["direction"])
        self.assertIsNone(render_frame(frames[1])[1]["direction"])
        (self.root / "pair.png").unlink()
        self.assertEqual(render_frame(frames[0])[1]["status"], "error")
        self.assertIsNone(render_frame(frames[0])[1]["direction"])
        out = self.root / "rendered"
        report = render_frames(frames, out)
        self.assertEqual(report["processingFailures"], 1)
        with self.assertRaises(FileExistsError):
            render_frames(frames, out)

    def test_thin_images_become_reported_failures_without_aborting_other_frames(self):
        self.add_frame("thin", [None, None], width=1, height=2000)
        self.add_frame("wide", [None, None], width=2000, height=1)
        dataset = CVDataset(self.save())
        out = self.root / "thin-preview"
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(["render", "--manifest", str(self.manifest), "--output-dir", str(out)]), 2)
        report = json.loads((out / "report.json").read_text())
        self.assertEqual(report["processingFailures"], 2)
        self.assertEqual([r["status"] for r in report["frames"]], ["ok", "ok", "error", "error"])
        self.assertEqual(dataset[2][0].shape, (2000, 1, 3))
        self.assertIn("too thin", report["frames"][2]["reason"])

    def test_cli_inspect_evaluate_report_and_no_overwrite(self):
        output = self.root / "inspection"
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            args = ["inspect", "--manifest", str(self.save()), "--output-dir", str(output)]
            self.assertEqual(main(args), 0)
            before = (output / "report.json").read_bytes()
            self.assertEqual(main(args), 2)
            self.assertEqual((output / "report.json").read_bytes(), before)
            p = self.predictions()
            p["frames"].pop()
            pp = self.root / "predictions.json"
            pp.write_text(json.dumps(p))
            dest = self.root / "evaluation"
            self.assertEqual(main(["evaluate", "--manifest", str(self.manifest), "--predictions", str(pp),
                                   "--score-threshold", "0.5", "--nms-threshold", "0.5", "--output-dir", str(dest)]), 2)
            self.assertEqual(json.loads((dest / "report.json").read_text())["metrics"]["processingFailures"], 1)


if __name__ == "__main__":
    unittest.main()
