"""Only tiny synthetic torch modules and box operations; never a detector model."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import random
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from learning.cv_dataset import CLASSES, COORDINATES
from learning.cv_sanity import SanityEvaluationError, _strict_success, evaluate_sanity, state_digest


class OptionalSanityDependencyTest(unittest.TestCase):
    def test_import_has_no_runtime_side_effect(self):
        result = subprocess.run([sys.executable, "-c", """
import sys
sys.modules['torch'] = None
sys.modules['torchvision'] = None
sys.modules['numpy'] = None
from learning.cv_sanity import evaluate_sanity, state_digest
"""], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_success_requires_every_denominator_failure_and_class_count(self):
        metrics = {"frames": 6, "processingFailures": 0,
                   "classes": {name: {"tp": n, "fp": 0, "fn": 0} for name, n in zip(CLASSES, (4, 6))}}
        for key in ("pairCoverage", "positionSuccess", "directionSuccess"):
            metrics[key] = {"count": 4, "denominator": 4}
        self.assertTrue(_strict_success({"metrics": metrics}))
        paths = [("frames",), ("processingFailures",), ("classes", "lip_balm", "tp"),
                 ("classes", "gripper", "fp"), ("classes", "gripper", "fn")]
        paths += [(key, field) for key in ("pairCoverage", "positionSuccess", "directionSuccess")
                  for field in ("count", "denominator")]
        for path in paths:
            changed = copy.deepcopy(metrics)
            obj = changed
            for name in path[:-1]:
                obj = obj[name]
            obj[path[-1]] += 1
            with self.subTest(path=path):
                self.assertFalse(_strict_success({"metrics": changed}))


@unittest.skipUnless(importlib.util.find_spec("torch") is not None, "optional torch runtime not installed")
class SanityEvaluationTest(unittest.TestCase):
    def setUp(self):
        import numpy as np
        import torch
        from PIL import Image
        from torchvision.ops import boxes as ops
        from learning.cv_tensors import TorchCVDataset

        self.torch, self.np = torch, np
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        records = []
        for i in range(6):
            path = self.root / f"{i}.png"
            Image.new("RGB", (20, 12), (i, 10, 20)).save(path)
            boxes = ([1, 1, 5, 5] if i < 4 else None, [10, 2, 16, 10])
            records.append({"id": f"frame-{i}", "imageId": i + 1, "image": path.name,
                            "sourceSha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                            "width": 20, "height": 12, "sessionId": "synthetic", "leakageGroup": "synthetic",
                            "split": "train_development_pilot", "coordinates": COORDINATES,
                            "labels": [{"class": name, "presence": "present" if box else "absent",
                                        "bbox_xyxy": box, "bbox_valid": bool(box), "occlusion": "none",
                                        "truncated": False, "humanApproved": True, "reviewStatus": "human_approved"}
                                       for name, box in zip(CLASSES, boxes)]})
        manifest = self.root / "manifest.json"
        manifest.write_text(json.dumps({"schema": "motionbrain.cv_dataset.v1",
                                       "classIds": {"background_reserved_not_annotation": 0, "lip_balm": 1, "gripper": 2},
                                       "records": records, "splits": {"train_development_pilot": [r["id"] for r in records]}}))
        dataset = TorchCVDataset(manifest)
        self.generator = torch.Generator(device="cpu").manual_seed(1234)
        generator = self.generator

        class TinyTransform(torch.nn.Module):
            def forward(self, images):
                # Resize by 2 to exercise GT coordinate scaling independently of padding.
                return SimpleNamespace(image_sizes=[(24, 40)], tensors=torch.zeros((1, 3, 32, 48))), None

        class TinyRPN(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.min_size, self.score_thresh, self.nms_thresh = 0.001, 0.05, 0.7
                self.fg_bg_sampler = SimpleNamespace(last_stats=[{"saved": "rpn"}])
                self.register_buffer("marker", torch.tensor(0.0))
                self.empty = False

            def _get_top_n_idx(self, logits, counts):
                pieces = logits.split(counts, dim=1)
                indexes, offset = [], 0
                for piece, count in zip(pieces, counts):
                    indexes.append(piece.topk(min(count, 2), dim=1).indices + offset)
                    offset += count
                return torch.cat(indexes, dim=1)

            def post_nms_top_n(self):
                return 3

            def filter_proposals(self, proposals, objectness, image_shapes, counts):
                # A small independent original filter; the observer must return this exact object.
                logits = objectness.reshape(1, -1)
                levels = torch.cat([torch.full((count,), i, dtype=torch.int64) for i, count in enumerate(counts)])
                ids = self._get_top_n_idx(logits, counts)[0]
                scores = logits[0].sigmoid()[ids]
                levels = levels[ids]
                boxes = ops.clip_boxes_to_image(proposals[0][ids], image_shapes[0])
                keep = ops.remove_small_boxes(boxes, self.min_size)
                boxes, scores, levels = boxes[keep], scores[keep], levels[keep]
                keep = torch.where(scores >= self.score_thresh)[0]
                boxes, scores, levels = boxes[keep], scores[keep], levels[keep]
                keep = ops.batched_nms(boxes, scores, levels, self.nms_thresh)[:self.post_nms_top_n()]
                self.last_return = ([boxes[keep]], [scores[keep]])
                return self.last_return

        class TinyModel(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.backbone = torch.nn.Linear(1, 1)
                self.transform = TinyTransform()
                self.rpn = TinyRPN()
                self.roi_heads = torch.nn.Module()
                self.roi_heads.fg_bg_sampler = SimpleNamespace(last_stats=[{"saved": "roi"}])
                self.fail = self.mutate = self.bad_prediction = self.rpn_mismatch = False
                self.tamper = None
                self.forward_count = 0
                self.received_targets = False

            def forward(self, images, *targets):
                self.received_targets = bool(targets)
                assert not targets and not self.training and not torch.is_grad_enabled()
                self.forward_count += 1
                # Deliberately consume every RNG and touch observer-only sampler stats.
                random.random(); np.random.random(); torch.rand(1); torch.rand(1, generator=generator)
                self.rpn.fg_bg_sampler.last_stats = ["eval"]
                self.roi_heads.fg_bg_sampler.last_stats = ["eval"]
                if self.mutate:
                    self.rpn.marker.add_(1)
                if self.fail:
                    raise RuntimeError("synthetic forward error")
                images_list, _ = self.transform(images)
                proposals = torch.tensor([[[2., 2., 10., 10.], [20., 4., 32., 20.],
                                           [2., 2., 10., 10.], [0., 0., 1., 1.],
                                           [20., 4., 32., 20.], [30., 0., 40., 4.]]])
                logits = torch.tensor([3., -4., 2., 1., 3., -5.])
                if self.rpn.empty:
                    logits.fill_(-20)
                result = self.rpn.filter_proposals(proposals, logits, images_list.image_sizes, [3, 3])
                assert result is self.rpn.last_return
                if self.tamper:
                    self.tamper.write_bytes(b"changed after decode")
                index = round(float(images[0][0, 0, 0]) * 255)
                boxes = [[1., 1., 5., 5.], [10., 2., 16., 10.]] if index < 4 else [[10., 2., 16., 10.]]
                labels = [1, 2] if index < 4 else [2]
                if self.bad_prediction:
                    boxes[0][0] = float("nan")
                return [{"boxes": torch.tensor(boxes), "labels": torch.tensor(labels),
                         "scores": torch.full((len(boxes),), 0.99)}]

        model = TinyModel()
        model.train()
        model.backbone.eval()
        self.backend = SimpleNamespace(model=model, dataset=dataset, torch=torch,
                                       loader=SimpleNamespace(generator=generator))
        self.state = {name: value.clone() for name, value in model.state_dict().items()}
        self.modes = [(module, module.training) for module in model.modules()]
        self.rng = (torch.get_rng_state().clone(), random.getstate(), np.random.get_state(), generator.get_state().clone())

    def assert_restored(self):
        t, model = self.torch, self.backend.model
        for name, value in model.state_dict().items():
            t.testing.assert_close(value, self.state[name], rtol=0, atol=0)
        self.assertTrue(all(module.training == mode for module, mode in self.modes))
        self.assertTrue(t.equal(t.get_rng_state(), self.rng[0]))
        self.assertEqual(random.getstate(), self.rng[1])
        state = self.np.random.get_state()
        self.assertEqual(state[0], self.rng[2][0])
        self.assertTrue(self.np.array_equal(state[1], self.rng[2][1]))
        self.assertEqual(state[2:], self.rng[2][2:])
        self.assertTrue(t.equal(self.generator.get_state(), self.rng[3]))
        self.assertNotIn("filter_proposals", model.rpn.__dict__)
        self.assertFalse(model.transform._forward_hooks)
        self.assertEqual(model.rpn.fg_bg_sampler.last_stats, [{"saved": "rpn"}])
        self.assertEqual(model.roi_heads.fg_bg_sampler.last_stats, [{"saved": "roi"}])

    def evaluate(self, deadline=lambda: None):
        with patch("torch.hub.download_url_to_file", side_effect=AssertionError("download prohibited")), \
             patch.dict(sys.modules, {"cv2": None}):
            return evaluate_sanity(self.backend, deadline)

    def test_success_observes_only_original_predictions_and_restores_every_state(self):
        result = self.evaluate()
        self.assertEqual(result["status"], "ok")
        self.assertTrue(result["observedSuccess"] and result["modelStateUnchanged"])
        self.assertEqual(result["modelStateSha256"], state_digest(self.state))
        self.assertEqual(result["evaluation"]["metrics"]["pairCoverage"]["count"], 4)
        self.assertEqual(self.backend.model.forward_count, 6)
        self.assertFalse(self.backend.model.received_targets)
        self.assertEqual(len(result["proposalDiagnostics"]), 6)
        self.assert_restored()

    def test_trace_levels_ranks_scaling_and_original_filter_identity(self):
        result = self.evaluate()
        row = result["proposalDiagnostics"][0]
        self.assertEqual(row["transformedSize"], [24, 40])
        self.assertEqual(row["paddedSize"], [32, 48])
        stages = {r["stage"]: r for r in row["stages"]}
        self.assertEqual(stages["decoded_all"]["candidateCount"], 6)
        self.assertEqual(stages["topn_per_level"]["countsByLevel"], [2, 2])
        self.assertEqual(stages["postcap"]["candidateCount"], 3)
        best = stages["decoded_all"]["groundTruth"][1]["highestObjectnessPositive"]
        self.assertEqual((best["anchorId"], best["level"]), (4, 1))
        self.assertEqual(best["rankLevelByStrictGreater"], 1)
        self.assertEqual(best["iou"], 1)
        self.assertEqual(stages["decoded_all"]["groundTruth"][1]["gtResizedXYXY"], [20, 4, 32, 20])

    def test_empty_filtered_stages_keep_zero_coverage(self):
        self.backend.model.rpn.empty = True
        result = self.evaluate()
        stage = result["proposalDiagnostics"][0]["stages"][-1]
        self.assertEqual(stage["candidateCount"], 0)
        self.assertEqual([x["countIoUPoint5"] for x in stage["groundTruth"]], [0, 0])
        self.assertTrue(all(x["best"] is None for x in stage["groundTruth"]))

    def test_forward_failure_preserves_all_frame_records_and_restores_state(self):
        self.backend.model.fail = True
        with self.assertRaises(SanityEvaluationError) as error:
            self.evaluate()
        result = error.exception.result
        self.assertIsInstance(error.exception.cause, RuntimeError)
        self.assertFalse(result["observedSuccess"])
        self.assertEqual(len(result["predictions"]["frames"]), 6)
        self.assertEqual([f["status"] for f in result["predictions"]["frames"]], ["error"] + ["not_evaluated"] * 5)
        self.assert_restored()

    def test_mutated_buffer_is_restored_but_still_fails(self):
        self.backend.model.mutate = True
        with self.assertRaises(SanityEvaluationError) as error:
            self.evaluate()
        self.assertFalse(error.exception.result["modelStateUnchanged"])
        self.assertFalse(error.exception.result["observedSuccess"])
        self.assertIn("mutated", str(error.exception))
        self.assert_restored()

    def test_interrupt_preserves_partial_result_and_original_cause(self):
        with patch.object(self.backend.model, "forward", side_effect=KeyboardInterrupt("synthetic interrupt")):
            with self.assertRaises(SanityEvaluationError) as error:
                self.evaluate()
        self.assertIsInstance(error.exception.cause, KeyboardInterrupt)
        self.assertEqual(error.exception.result["predictions"]["frames"][0]["status"], "error")
        self.assert_restored()

    def test_deadline_retains_original_exception_and_cleans_observer(self):
        class Expired(Exception):
            pass
        calls = 0
        def deadline():
            nonlocal calls
            calls += 1
            if calls >= 4:
                raise Expired("synthetic budget expired")
        with self.assertRaises(SanityEvaluationError) as error:
            self.evaluate(deadline)
        self.assertIsInstance(error.exception.cause, Expired)
        self.assertEqual(error.exception.result["status"], "failed")
        self.assert_restored()

    def test_wrong_inventory_fails_before_forward(self):
        self.backend.dataset.frames = self.backend.dataset.frames[:5]
        with self.assertRaises(SanityEvaluationError) as error:
            self.evaluate()
        self.assertIn("six", str(error.exception))
        self.assertEqual(self.backend.model.forward_count, 0)
        self.assert_restored()

    def test_deadline_after_decode_prevents_forward(self):
        calls = 0
        def deadline():
            nonlocal calls
            calls += 1
            if calls == 3:
                raise TimeoutError("decode consumed budget")
        with self.assertRaises(SanityEvaluationError):
            self.evaluate(deadline)
        self.assertEqual(self.backend.model.forward_count, 0)
        self.assert_restored()

    def test_deadline_includes_state_restoration(self):
        def deadline():
            if self.backend.model.forward_count == 6 and self.backend.model.training:
                raise TimeoutError("state verification consumed budget")
        with self.assertRaises(SanityEvaluationError) as error:
            self.evaluate(deadline)
        self.assertFalse(error.exception.result["observedSuccess"])
        self.assertIn("verification consumed", str(error.exception))
        self.assert_restored()

    def test_corrupt_source_is_failed_record_not_empty_success(self):
        self.backend.dataset.frames[0].path.write_bytes(b"corrupt")
        with self.assertRaises(SanityEvaluationError) as error:
            self.evaluate()
        self.assertIn("hash mismatch", str(error.exception))
        self.assertEqual(self.backend.model.forward_count, 0)
        self.assertEqual(error.exception.result["predictions"]["frames"][0]["status"], "error")
        self.assert_restored()

    def test_source_mutation_after_forward_is_caught_by_metric_reader(self):
        self.backend.model.tamper = self.backend.dataset.frames[0].path
        with self.assertRaises(SanityEvaluationError) as error:
            self.evaluate()
        result = error.exception.result
        self.assertEqual(result["evaluation"]["metrics"]["processingFailures"], 1)
        self.assertFalse(result["observedSuccess"])
        self.assert_restored()

    def test_nonfinite_prediction_is_failed_before_metrics(self):
        self.backend.model.bad_prediction = True
        with self.assertRaises(SanityEvaluationError) as error:
            self.evaluate()
        self.assertIn("nonfinite", str(error.exception))
        self.assert_restored()

    def test_original_instance_filter_is_restored_on_failure(self):
        original = self.backend.model.rpn.filter_proposals
        def changed(*args):
            boxes, scores = original(*args)
            return [boxes[0] + 1], scores
        self.backend.model.rpn.filter_proposals = changed
        with self.assertRaises(SanityEvaluationError) as error:
            self.evaluate()
        self.assertIn("differs", str(error.exception))
        self.assertIs(self.backend.model.rpn.filter_proposals, changed)
        del self.backend.model.rpn.filter_proposals
        self.assert_restored()

    def test_state_hash_includes_name_dtype_shape_and_scalar_values(self):
        t = self.torch
        a = {"a": t.tensor([1, 2], dtype=t.int32)}
        variants = [{"b": a["a"]}, {"a": a["a"].view(t.float32)}, {"a": a["a"].reshape(1, 2)},
                    {"a": t.tensor([1, 3], dtype=t.int32)}]
        self.assertTrue(all(state_digest(v) != state_digest(a) for v in variants))
        self.assertEqual(state_digest({"b": t.tensor(0), **a}), state_digest({**a, "b": t.tensor(0)}))
        self.assertEqual(len(state_digest({"scalar": t.tensor(0), "bf16": t.ones(1, dtype=t.bfloat16)})), 64)


if __name__ == "__main__":
    unittest.main()
