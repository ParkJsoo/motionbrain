"""Synthetic files, fake processes and tiny modules only; never a real detector."""
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from learning import cv_checkpoint as runner
from learning.cv_dataset import CLASSES, COORDINATES
from learning.cv_training import atomic_json
from tools.evaluate_cv import main


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Fixture(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()
        self.plan_path = self.root / "plan.json"
        self.manifest = self.root / "manifest.json"
        self.lock_path = self.root / "lock.json"
        self.make_data(3)
        sources = {}
        for name in (*runner.LOCKED_FILES, *runner.RUNTIME_FILES, *runner.RUNNER_FILES):
            p = self.root / name
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text("synthetic source " + name)
            sources[name] = digest(p)
        (self.root / "checkpoint.pt").write_bytes(b"fake checkpoint; not deserialized")
        self.lock = {"schema": "motionbrain.prospective_evaluation_lock.v1", "model": runner.MODEL,
                     "coordinates": COORDINATES, "preprocessing": copy.deepcopy(runner.PREPROCESSING),
                     "internalFilters": dict(runner.FILTERS), "operatingPoint": dict(runner.OPERATING_POINT),
                     "runtime": dict(runner.RUNTIME), "checkpoint": "checkpoint.pt",
                     "checkpointSha256": digest(self.root / "checkpoint.pt"), "checkpointStateSha256": "a"*64,
                     "sourceFiles": {k: v for k, v in sources.items() if k not in runner.RUNNER_FILES},
                     "trainingData": {"sourceHashes": {"old": "b"*64}, "sessionIds": ["old"], "leakageGroups": ["old"]},
                     "restorationContract": {"architectureNumClasses": 3, "weights": None, "weightsBackbone": None,
                        "strictStateDictLoad": True, "reinitializePredictorAfterLoad": False,
                        "beforeAfterInferenceStateUnchanged": True, "loadedStateDigestMustEqual": "a"*64}}
        self.doc = {"schema": "motionbrain.cv_checkpoint_plan.v1", "lock": "lock.json", "lock_sha256": "",
                    "manifest": "manifest.json", "manifest_sha256": digest(self.manifest), "split": "validation",
                    "frame_ids": [r["id"] for r in self.data["records"]], "output_dir": "result",
                    "runner_sources": {k: sources[k] for k in runner.RUNNER_FILES},
                    "scope": "separate_session_development", "seconds": 5}
        self.save()
        self.root_patch = patch.object(runner, "ROOT", self.root)
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)

    def make_data(self, count):
        records = []
        for i in range(count):
            width, height = 20+i, 12+i
            path = self.root / f"{i}.ppm"
            path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + bytes((i, 10, 20)) * width * height)
            boxes = ([1, 1, 5, 5] if i % 3 == 0 else None, [10, 2, 16, 10] if i % 3 != 2 else None)
            records.append({"id": f"f{i}", "imageId": i+1, "image": path.name, "sourceSha256": digest(path),
                            "width": width, "height": height, "sessionId": f"s{i%2}", "leakageGroup": f"g{i%2}",
                            "split": "validation", "coordinates": COORDINATES,
                            "labels": [{"class": n, "presence": "present" if b else "absent", "bbox_xyxy": b,
                                        "bbox_valid": b is not None, "humanApproved": True, "reviewStatus": "human_approved",
                                        "occlusion": "none", "truncated": False} for n, b in zip(CLASSES, boxes)]})
        self.data = {"schema": "motionbrain.cv_dataset.v1", "records": records,
                     "classIds": {"background_reserved_not_annotation": 0, "lip_balm": 1, "gripper": 2},
                     "splits": {"validation": [r["id"] for r in records]}}
        atomic_json(self.manifest, self.data)

    def save(self):
        atomic_json(self.lock_path, self.lock)
        self.doc["lock_sha256"] = digest(self.lock_path)
        atomic_json(self.plan_path, self.doc)

    def prepared(self):
        plan, _ = runner.load_plan(self.plan_path)
        lock, ds = runner.preflight(plan)
        return plan, lock, ds


class PlanTest(Fixture):
    def test_variable_counts_and_read_only_no_runtime(self):
        for count in (1, 3, 7):
            self.make_data(count)
            self.doc.update(manifest_sha256=digest(self.manifest), frame_ids=[r["id"] for r in self.data["records"]])
            self.save()
            with patch.dict(sys.modules, {"torch": None, "torchvision": None, "PIL": None, "numpy": None}):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main(["check", "--plan", str(self.plan_path)]), 0)
            self.assertFalse((self.root / "result").exists())
            self.assertFalse(Path(str(self.plan_path) + ".consumed").exists())

    def test_missing_image_is_not_excluded_by_preflight(self):
        (self.root / "1.ppm").unlink()
        self.assertEqual(len(self.prepared()[2]), 3)

    def test_source_checkpoint_manifest_lock_and_runner_hashes_fail_closed(self):
        for path in (self.manifest, self.lock_path, self.root/"checkpoint.pt",
                     self.root/runner.LOCKED_FILES[0], self.root/runner.RUNNER_FILES[0]):
            raw = path.read_bytes()
            path.write_bytes(raw+b" ")
            with self.subTest(path=path), self.assertRaises(ValueError):
                self.prepared()
            path.write_bytes(raw)

    def test_unapproved_and_inventory_mismatch_rejected(self):
        self.doc["frame_ids"] = ["f1", "f0", "f2"]
        self.save()
        with self.assertRaisesRegex(ValueError, "inventory"):
            self.prepared()
        self.doc["frame_ids"] = ["f0", "f1", "f2"]
        self.data["records"][0]["labels"][0]["humanApproved"] = False
        atomic_json(self.manifest, self.data)
        self.doc["manifest_sha256"] = digest(self.manifest)
        self.save()
        with self.assertRaisesRegex(ValueError, "approval"):
            self.prepared()

    def test_training_overlap_hash_session_group_and_development_scope(self):
        for key, value in (("sourceHashes", {"old": self.data["records"][0]["sourceSha256"]}),
                           ("sessionIds", ["s0"]), ("leakageGroups", ["g0"])):
            old = copy.deepcopy(self.lock["trainingData"])
            self.lock["trainingData"][key] = value
            self.save()
            with self.assertRaisesRegex(ValueError, "overlaps"):
                self.prepared()
            self.lock["trainingData"] = old
        self.doc["scope"] = "development"
        self.save()
        self.prepared()

    def test_contract_and_plan_tampering(self):
        original = copy.deepcopy(self.lock)
        for group, key, value in (("runtime", "optimizer", True), ("preprocessing", "resizeMin", 640),
                                  ("internalFilters", "roiScoreThreshold", 0.1),
                                  ("operatingPoint", "scoreThreshold", 0.1),
                                  ("restorationContract", "strictStateDictLoad", False)):
            self.lock = copy.deepcopy(original)
            self.lock[group][key] = value
            self.save()
            with self.assertRaises(ValueError):
                self.prepared()
        original_plan = copy.deepcopy(self.doc)
        for key, value in (("seconds", 0), ("seconds", True), ("seconds", 901), ("scope", "sealed_test"),
                           ("frame_ids", []), ("frame_ids", ["x", "x"]), ("runner_sources", {})):
            self.doc = dict(original_plan, **{key: value})
            self.save()
            with self.assertRaises(ValueError):
                runner.load_plan(self.plan_path)

    def test_duplicate_keys_and_import_without_optional_runtime(self):
        self.plan_path.write_text('{"schema": "x", "schema": "x"}')
        with self.assertRaisesRegex(ValueError, "duplicate"):
            runner.load_plan(self.plan_path)
        result = subprocess.run([sys.executable, "-c", "import sys; sys.modules['torch']=None; "
                                 "sys.modules['torchvision']=None; from tools.evaluate_cv import main"], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_run_requires_flag_and_check_rejects_flag(self):
        with patch.object(runner, "restore_model") as restore, contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main(["run", "--plan", str(self.plan_path)]), 2)
            self.assertEqual(main(["check", "--execute", "--plan", str(self.plan_path)]), 2)
            restore.assert_not_called()


class FakeProcess:
    def __init__(self, target, args, mode="success"):
        self.args, self.mode = args, mode
        self.pid, self.exitcode, self.alive = None, None, False
    def start(self):
        self.pid, self.alive = 1, True
        plan, output, _ = self.args
        if self.mode == "start_error":
            raise OSError("spawn failed")
        if self.mode == "success":
            predictions = json.loads((Path(output)/"predictions.json").read_text())
            for entry in predictions["frames"]:
                entry["status"] = "ok"  # Normal zero detections, not a model invocation.
            atomic_json(Path(output)/"predictions.json", predictions)
            atomic_json(Path(output)/"progress.json", {"status": "completed", "modelStateUnchanged": True,
                        "loadedStateSha256": "a"*64, "finalStateSha256": "a"*64})
    def join(self, timeout=None):
        if self.mode == "interrupt" and self.alive:
            self.mode = "timeout"
            raise KeyboardInterrupt()
        if self.mode in ("success", "crash"):
            self.alive, self.exitcode = False, 0 if self.mode == "success" else 9
    def is_alive(self): return self.alive
    def terminate(self): self.alive, self.exitcode = False, -15
    def kill(self): self.alive, self.exitcode = False, -9


class ExecutionTest(Fixture):
    def execute_fake(self, mode, render_failures=0):
        context = SimpleNamespace(Process=lambda target, args: FakeProcess(target, args, mode))
        with patch.object(runner.multiprocessing, "get_context", return_value=context), \
             patch("learning.cv_checkpoint_display.render_comparisons", return_value={"failures": render_failures, "frames": []}):
            return runner.execute(self.plan_path, confirmed=True)

    def test_success_keeps_variable_denominators_zero_detection_is_not_failure(self):
        result = self.execute_fake("success")
        self.assertEqual(result["status"], "completed")
        report = json.loads((self.root/"result/report.json").read_text())
        self.assertEqual(report["metrics"]["frames"], 3)
        self.assertEqual(report["metrics"]["pairCoverage"], {"count": 0, "denominator": 1, "rate": 0.0})
        self.assertEqual(report["metrics"]["classes"]["lip_balm"]["fn"], 1)
        self.assertEqual(report["metrics"]["processingFailures"], 0)
        self.assertEqual(len(report["sessions"]), 2)

    def test_render_failure_report_has_final_run_status(self):
        result = self.execute_fake("success", render_failures=1)
        report = json.loads((self.root/"result/report.json").read_text())
        self.assertEqual(result["status"], "render_failed")
        self.assertEqual(report["runStatus"], result["status"])

    def test_no_completed_frame_does_not_claim_model_accuracy(self):
        _, _, ds = self.prepared()
        report = runner.score_records(ds, runner.initial_predictions(ds), verified_state=True)
        self.assertFalse(report["modelAccuracyMeasured"])

    def test_consumed_plan_and_existing_output_never_retried(self):
        self.execute_fake("success")
        with self.assertRaises(ValueError):
            self.execute_fake("success")
        self.doc["output_dir"] = "other"
        self.save()
        with self.assertRaises(FileExistsError):
            self.execute_fake("success")
        self.assertFalse((self.root/"other").exists())

    def test_timeout_interrupt_crash_and_spawn_failure_preserve_inventory(self):
        for mode, status in (("timeout", "time_limit"), ("interrupt", "interrupted"),
                             ("crash", "worker_exit"), ("start_error", "failed")):
            with self.subTest(mode=mode):
                self.plan_path = self.root/f"{mode}.json"
                self.doc["output_dir"] = mode
                self.save()
                result = self.execute_fake(mode)
                self.assertEqual(result["status"], status)
                report = json.loads((self.root/mode/"report.json").read_text())
                self.assertEqual(report["metrics"]["processingFailures"], 3)
                self.assertFalse(report["modelAccuracyMeasured"])
                self.assertIsNone(report["metrics"]["classes"]["lip_balm"]["precision"]["rate"])

    def test_unverified_state_invalidates_raw_success_but_preserves_raw(self):
        _, _, ds = self.prepared()
        raw = runner.initial_predictions(ds)
        for entry in raw["frames"]:
            entry["status"] = "ok"
        report = runner.score_records(ds, raw, verified_state=False)
        self.assertEqual(report["metrics"]["processingFailures"], 3)
        self.assertTrue(all(r["status"] == "ok" for r in raw["frames"]))
        self.assertEqual(report["metrics"]["classes"]["lip_balm"]["absentClearFrames"], 0)


@unittest.skipUnless(importlib.util.find_spec("torch"), "optional torch runtime not installed")
class TensorRunnerTest(Fixture):
    def model(self):
        import torch
        class Tiny(torch.nn.Module):
            def __init__(self):
                super().__init__()
                self.backbone = torch.nn.Sequential(torch.nn.BatchNorm2d(1))
                self.register_buffer("marker", torch.tensor(2.0))
                self.calls, self.fail, self.mutate = 0, False, False
                self.transform = SimpleNamespace(min_size=(320,), max_size=640,
                    image_mean=runner.PREPROCESSING["imageMean"], image_std=runner.PREPROCESSING["imageStd"])
                self.rpn = SimpleNamespace(pre_nms_top_n=lambda: 150, post_nms_top_n=lambda: 150,
                                           score_thresh=0.05, nms_thresh=0.7)
                self.roi_heads = SimpleNamespace(score_thresh=0.05, nms_thresh=0.5, detections_per_img=100,
                    box_predictor=SimpleNamespace(cls_score=SimpleNamespace(out_features=3), bbox_pred=SimpleNamespace(out_features=12)))
            def forward(self, images):
                assert not torch.is_grad_enabled()
                assert len(images) == 1 and images[0].dtype == torch.float32
                self.calls += 1
                if self.mutate:
                    self.marker.add_(1)
                if self.fail:
                    raise RuntimeError("synthetic forward failure")
                return [{"boxes": torch.tensor([[1, 1, 5, 5], [10, 2, 16, 10]], dtype=torch.float32),
                         "scores": torch.tensor([0.9, 0.8]), "labels": torch.tensor([1, 2])}]
        return Tiny()

    def test_inference_only_images_variable_sizes_and_durable_intents(self):
        import torch
        _, _, ds = self.prepared()
        model = self.model()
        predictions, events = runner.initial_predictions(ds), []
        runner.infer_frames(model, torch, ds, predictions, lambda p: events.append(copy.deepcopy(p)), lambda: None)
        self.assertEqual(model.calls, 3)
        self.assertEqual([e["status"] for e in predictions["frames"]], ["ok"]*3)
        self.assertEqual(events[1]["frames"][0]["status"], "inference_intent")
        report = runner.score_records(ds, predictions, verified_state=True)
        self.assertEqual(report["metrics"]["pairCoverage"]["count"], 1)
        self.assertEqual(report["metrics"]["classes"]["lip_balm"]["fp"], 2)

    def test_decode_forward_and_deadline_failures_stop_without_retry(self):
        import torch
        _, _, ds = self.prepared()
        for failure in ("decode", "forward", "deadline"):
            with self.subTest(failure=failure):
                model = self.model()
                model.fail = failure == "forward"
                predictions = runner.initial_predictions(ds)
                path = self.root/"0.ppm"
                raw = path.read_bytes()
                if failure == "decode":
                    path.write_bytes(b"changed")
                def check():
                    if failure == "deadline": raise TimeoutError()
                with self.assertRaises((ValueError, RuntimeError, TimeoutError)):
                    runner.infer_frames(model, torch, ds, predictions, lambda p: None, check)
                path.write_bytes(raw)
                self.assertEqual(model.calls, int(failure == "forward"))
                self.assertEqual([r["status"] for r in predictions["frames"]], ["error", "not_evaluated", "not_evaluated"])
                report = runner.score_records(ds, predictions, verified_state=True)
                self.assertEqual(report["metrics"]["processingFailures"], 3)

    def test_restore_strict_frozen_norms_no_predictor_reset_and_filter_assertions(self):
        import torch
        import torchvision
        original = self.model()
        runner.frozen_backbone_norms(original.backbone)
        path = self.root/"checkpoint.pt"
        torch.save({"state_dict": original.state_dict()}, path)
        self.lock["checkpointSha256"] = digest(path)
        self.lock["checkpointStateSha256"] = runner.state_digest(original.state_dict())
        runtime_root = Path(torchvision.__file__).resolve().parent/"models/detection"
        self.lock["sourceFiles"] = {str(runtime_root/n): digest(runtime_root/n) for n in runner.RUNTIME_FILES}
        model = self.model()
        with patch("torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn", return_value=model) as factory:
            restored, _ = runner.restore_model(self.lock)
            factory.assert_called_once_with(weights=None, weights_backbone=None, num_classes=3)
        self.assertEqual(restored.calls, 0)
        self.assertEqual(runner.state_digest(restored.state_dict()), self.lock["checkpointStateSha256"])
        self.assertFalse(any(p.requires_grad for p in restored.parameters()))
        restored.rpn.score_thresh = 0.1
        with self.assertRaisesRegex(ValueError, "filters"):
            runner.assert_model_contract(restored)
        # A checkpoint can have a valid digest but keys incompatible with the architecture.
        torch.save({"state_dict": {"wrong": torch.tensor(1.)}}, path)
        self.lock["checkpointSha256"] = digest(path)
        self.lock["checkpointStateSha256"] = runner.state_digest({"wrong": torch.tensor(1.)})
        with patch("torchvision.models.detection.fasterrcnn_mobilenet_v3_large_320_fpn", return_value=self.model()), \
             self.assertRaises(RuntimeError):
            runner.restore_model(self.lock)

    def test_worker_records_model_mutation_and_load_failure(self):
        import torch
        plan, lock, ds = self.prepared()
        for mode in ("mutate", "load"):
            output = self.root/mode
            output.mkdir()
            model = self.model()
            model.mutate = True
            lock["checkpointStateSha256"] = runner.state_digest(model.state_dict())
            with patch.object(runner, "preflight", return_value=(lock, ds)), \
                 patch.object(runner, "restore_model", side_effect=RuntimeError("load failed") if mode == "load" else None,
                              return_value=(model, torch)):
                runner._worker(plan, output, float("inf"))
            progress = json.loads((output/"progress.json").read_text())
            self.assertEqual(progress["status"], "failed")
            self.assertFalse(progress["modelStateUnchanged"])

    def test_all_frame_previews_and_missing_image_placeholder(self):
        from PIL import Image
        from learning.cv_checkpoint_display import render_comparisons
        _, _, ds = self.prepared()
        report = runner.score_records(ds, runner.initial_predictions(ds), verified_state=False)
        (self.root/"1.ppm").unlink()
        out = self.root/"previews"
        result = render_comparisons(ds.frames, report["frames"], out)
        self.assertEqual(len(result["frames"]), 3)
        self.assertEqual(result["failures"], 1)
        for i in range(1, 4):
            with Image.open(out/f"{i:04d}.png") as image:
                self.assertEqual(image.size, (960, 620))


if __name__ == "__main__":
    unittest.main()
