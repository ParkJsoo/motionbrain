"""One-use saved-checkpoint evaluation; importing/checking never loads a model."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import copy
import hashlib
import io
import json
import multiprocessing
import os
from pathlib import Path
import signal
import threading
import time

from learning.cv_dataset import COORDINATES, SPLITS, CVDataset, require
from learning.cv_evaluation import evaluate
from learning.cv_sanity import _prediction, state_digest
from learning.cv_tensors import TorchCVDataset
from learning.cv_training import MODEL, atomic_json, frozen_backbone_norms, supervise, verified_bytes

ROOT = Path(__file__).resolve().parents[1]
RUNNER_FILES = ("learning/cv_checkpoint.py", "learning/cv_checkpoint_display.py", "tools/evaluate_cv.py")
LOCKED_FILES = ("learning/cv_dataset.py", "learning/cv_tensors.py", "learning/cv_training.py",
                "learning/cv_sanity.py", "learning/cv_evaluation.py", "learning/cv_display.py",
                "tools/offline_cv.py")
RUNTIME_FILES = ("faster_rcnn.py", "rpn.py", "roi_heads.py", "transform.py")
OPERATING_POINT = {"scoreThreshold": 0.5, "nmsThreshold": 0.5, "matchIoU": 0.5,
                   "relativeToleranceReferencePx": 10, "directionBandReferencePx": 10,
                   "referenceViewport": [960, 540]}
PREPROCESSING = {"decoder": "Pillow, RGB float32 CHW /255; no EXIF transpose/rotation/mirror",
                 "resizeMin": 320, "resizeMax": 640, "imageMean": [0.485, 0.456, 0.406],
                 "imageStd": [0.229, 0.224, 0.225],
                 "classMap": {"0": "background", "1": "lip_balm", "2": "gripper"}}
FILTERS = {"rpnPreNMSTopNTest": 150, "rpnPostNMSTopNTest": 150, "rpnScoreThreshold": 0.05,
           "rpnNMSThreshold": 0.7, "roiScoreThreshold": 0.05, "roiNMSThreshold": 0.5,
           "detectionsPerImage": 100}
RUNTIME = {"torch": "2.14.1", "torchvision": "0.29.1", "numpy": "2.2.6", "Pillow": "12.2.0",
           "device": "cpu", "dtype": "float32", "threads": 4, "inferenceBatchSize": 1,
           "evalMode": True, "weightsOnlyLoad": True, "optimizer": False, "targetInput": False,
           "augmentation": False, "download": False}


def _json(raw):
    def unique(pairs):
        require(len({k for k, _ in pairs}) == len(pairs), "duplicate JSON key")
        return dict(pairs)
    return json.loads(raw, object_pairs_hook=unique)


def _sha(value):
    require(isinstance(value, str) and len(value) == 64
            and all(c in "0123456789abcdef" for c in value), "invalid sha256")
    return value


@dataclass(frozen=True)
class EvaluationPlan:
    lock: str
    lock_sha256: str
    manifest: str
    manifest_sha256: str
    split: str
    frame_ids: list
    runner_sources: dict
    output_dir: str
    scope: str
    seconds: int


def load_plan(path):
    path = Path(path).resolve()
    raw = path.read_bytes()
    doc = _json(raw)
    require(isinstance(doc, dict) and doc.pop("schema", None) == "motionbrain.cv_checkpoint_plan.v1",
            "unsupported checkpoint evaluation plan")
    require(set(doc) == set(EvaluationPlan.__dataclass_fields__), "plan fields missing or unknown")
    for key in ("lock", "manifest", "output_dir"):
        require(isinstance(doc[key], str) and bool(doc[key].strip()), f"invalid {key}")
        doc[key] = str((path.parent / doc[key]).resolve())
    for key in ("lock_sha256", "manifest_sha256"):
        _sha(doc[key])
    require(doc["split"] in SPLITS, "invalid split")
    require(doc["scope"] in ("development", "separate_session_development"), "unsupported evaluation scope")
    require(type(doc["seconds"]) is int and 1 <= doc["seconds"] <= 900, "seconds must be in 1..900")
    ids = doc["frame_ids"]
    require(isinstance(ids, list) and ids and all(isinstance(i, str) and i.strip() for i in ids)
            and len(set(ids)) == len(ids), "unique ordered frame_ids required")
    sources = doc["runner_sources"]
    require(isinstance(sources, dict) and set(sources) == set(RUNNER_FILES), "runner source inventory mismatch")
    for digest in sources.values():
        _sha(digest)
    return EvaluationPlan(**doc), hashlib.sha256(raw).hexdigest()


def preflight(plan):
    """Verify bytes/contracts only; image read failures belong to the run denominator."""
    lock = _json(verified_bytes(plan.lock, plan.lock_sha256))
    require(lock.get("schema") == "motionbrain.prospective_evaluation_lock.v1", "unsupported model lock")
    require(lock.get("model") == MODEL and lock.get("coordinates") == COORDINATES, "model/coordinates mismatch")
    require(lock.get("preprocessing") == PREPROCESSING, "unsupported preprocessing")
    require(lock.get("operatingPoint") == OPERATING_POINT, "unsupported operating point")
    require(lock.get("runtime") == RUNTIME, "unsupported runtime contract")
    filters = lock.get("internalFilters", {})
    require(all(filters.get(k) == v for k, v in FILTERS.items()), "unsupported internal filters")
    state_sha = _sha(lock.get("checkpointStateSha256"))
    restoration = lock.get("restorationContract", {})
    require(restoration.get("architectureNumClasses") == 3
            and restoration.get("weights", "missing") is None
            and restoration.get("weightsBackbone", "missing") is None
            and restoration.get("strictStateDictLoad") is True
            and restoration.get("reinitializePredictorAfterLoad") is False
            and restoration.get("beforeAfterInferenceStateUnchanged") is True
            and restoration.get("loadedStateDigestMustEqual") == state_sha, "unsupported restoration contract")
    sources = lock.get("sourceFiles", {})
    require(isinstance(sources, dict) and set(LOCKED_FILES) <= set(sources), "locked sources missing")
    require(all(any(Path(p).name == name for p in sources) for name in RUNTIME_FILES), "runtime sources missing")
    for path, digest in {**sources, **plan.runner_sources}.items():
        verified_bytes(ROOT / path, _sha(digest))
    checkpoint = lock.get("checkpoint")
    require(isinstance(checkpoint, str) and checkpoint.strip(), "checkpoint path missing")
    verified_bytes(ROOT / checkpoint, _sha(lock.get("checkpointSha256")))
    dataset = TorchCVDataset(plan.manifest, split=plan.split)
    require(dataset.manifest_sha256 == plan.manifest_sha256, "manifest sha256 mismatch")
    require([f.id for f in dataset.frames] == plan.frame_ids, "selected frame inventory/order mismatch")
    training = lock.get("trainingData", {})
    require(isinstance(training.get("sourceHashes"), dict)
            and isinstance(training.get("sessionIds"), list)
            and isinstance(training.get("leakageGroups"), list), "training inventory missing")
    if plan.scope == "separate_session_development":
        require(bool(training["sourceHashes"]) and bool(training["sessionIds"])
                and bool(training["leakageGroups"]), "training inventory empty")
        require(all(f.sha256 not in training["sourceHashes"].values()
                    and f.session not in training["sessionIds"]
                    and f.group not in training["leakageGroups"] for f in dataset.frames),
                "separate session overlaps training hashes/session/leakage group")
    return lock, dataset


def assert_model_contract(model):
    t, rpn, roi = model.transform, model.rpn, model.roi_heads
    require(tuple(t.min_size) == (320,) and t.max_size == 640
            and list(t.image_mean) == PREPROCESSING["imageMean"]
            and list(t.image_std) == PREPROCESSING["imageStd"], "effective preprocessing mismatch")
    actual = {"rpnPreNMSTopNTest": rpn.pre_nms_top_n(), "rpnPostNMSTopNTest": rpn.post_nms_top_n(),
              "rpnScoreThreshold": rpn.score_thresh, "rpnNMSThreshold": rpn.nms_thresh,
              "roiScoreThreshold": roi.score_thresh, "roiNMSThreshold": roi.nms_thresh,
              "detectionsPerImage": roi.detections_per_img}
    require(actual == FILTERS, "effective filters mismatch")
    require(roi.box_predictor.cls_score.out_features == 3
            and roi.box_predictor.bbox_pred.out_features == 12, "predictor class count mismatch")
    require(all(not module.training for module in model.modules()), "model must be entirely in eval mode")


def restore_model(lock):
    """No pretrained builder, optimizer, or predictor replacement after strict load."""
    import numpy as np
    import PIL
    import torch
    import torchvision
    from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn

    actual = {"torch": torch.__version__.split("+")[0], "torchvision": torchvision.__version__.split("+")[0],
              "numpy": np.__version__, "Pillow": PIL.__version__}
    require(all(actual[k] == RUNTIME[k] for k in actual), "installed runtime versions mismatch")
    # Verify the imported implementation, not merely an unused file with the same name.
    runtime_root = Path(torchvision.__file__).resolve().parent / "models/detection"
    for name in RUNTIME_FILES:
        path = runtime_root / name
        matches = [digest for p, digest in lock["sourceFiles"].items() if (ROOT / p).resolve() == path]
        require(len(matches) == 1, "loaded torchvision source not in lock")
        verified_bytes(path, matches[0])
    torch.set_default_device("cpu")
    torch.set_default_dtype(torch.float32)
    torch.set_num_threads(4)
    model = fasterrcnn_mobilenet_v3_large_320_fpn(weights=None, weights_backbone=None, num_classes=3)
    frozen_backbone_norms(model.backbone)
    saved = torch.load(io.BytesIO(verified_bytes(ROOT / lock["checkpoint"], lock["checkpointSha256"])),
                       map_location="cpu", weights_only=True)
    require(isinstance(saved, dict) and isinstance(saved.get("state_dict"), dict), "invalid checkpoint")
    require(state_digest(saved["state_dict"]) == lock["checkpointStateSha256"], "saved tensor digest mismatch")
    require(all(bool(torch.isfinite(v).all()) for v in saved["state_dict"].values()), "nonfinite saved tensors")
    model.load_state_dict(saved["state_dict"], strict=True)
    require(state_digest(model.state_dict()) == lock["checkpointStateSha256"], "loaded tensor digest mismatch")
    model.requires_grad_(False)
    model.eval()
    assert_model_contract(model)
    require(all(t.device.type == "cpu" and (not t.is_floating_point() or t.dtype == torch.float32)
                for t in model.state_dict().values()), "model must be CPU float32")
    return model, torch


def initial_predictions(dataset):
    return {"schema": "motionbrain.cv_predictions.v1", "evidenceKind": "model", "coordinates": COORDINATES,
            "frames": [{"id": f.id, "sourceSha256": f.sha256, "status": "not_evaluated", "detections": []}
                       for f in dataset.frames]}


def infer_frames(model, torch, dataset, predictions, emit, check_deadline):
    """Stop on first failure, retain every predeclared frame, never retry."""
    with torch.inference_mode():
        for index, frame in enumerate(dataset.frames):
            entry = predictions["frames"][index]
            try:
                check_deadline()
                entry["status"] = "reading"
                emit(predictions)
                image, _ = dataset[index]
                check_deadline()
                entry["status"] = "inference_intent"
                emit(predictions)  # Durable before any forward call; interrupted intent stays uncertain.
                output = model([image])
                check_deadline()
                require(isinstance(output, (list, tuple)) and len(output) == 1, "expected one prediction")
                entry.update(status="ok", detections=_prediction(torch, frame, output[0]))
                emit(predictions)
            except BaseException as exc:
                entry.update(status="error", error=f"{type(exc).__name__}: {exc}")
                emit(predictions)
                raise


def _worker(plan, output, deadline):
    output = Path(output)
    progress = {"status": "loading", "modelStateUnchanged": False}
    model = None
    def check():
        if time.monotonic() >= deadline:
            raise TimeoutError("evaluation time limit")
    try:
        check()
        lock, dataset = preflight(plan)
        atomic_json(output / "progress.json", progress)
        model, torch = restore_model(lock)
        progress.update(status="running", loadedStateSha256=lock["checkpointStateSha256"])
        atomic_json(output / "progress.json", progress)
        predictions = initial_predictions(dataset)
        infer_frames(model, torch, dataset, predictions,
                     lambda p: atomic_json(output / "predictions.json", p), check)
        check()
        progress["status"] = "completed"
    except BaseException as exc:
        progress.update(status="failed", error=f"{type(exc).__name__}: {exc}")
    finally:
        if model is not None:
            try:
                progress["finalStateSha256"] = state_digest(model.state_dict())
                progress["modelStateUnchanged"] = progress["finalStateSha256"] == lock["checkpointStateSha256"]
                require(progress["modelStateUnchanged"], "model state changed during inference")
                check()
            except BaseException as exc:
                progress.update(status="failed", error=f"{type(exc).__name__}: {exc}")
        atomic_json(output / "progress.json", progress)


def score_records(dataset, predictions, *, verified_state):
    scoring = copy.deepcopy(predictions)
    if not verified_state:
        for row in scoring["frames"]:
            row.update(status="unverified_model_state", detections=[])
    # Successful rows were hash/size/decoder-checked in the worker. Never re-read
    # changed files to silently change the sample population during finalization.
    report = evaluate(dataset, scoring, score_threshold=0.5, nms_threshold=0.5,
                      image_reader=lambda frame: None)
    report.update(modelStateVerified=verified_state,
                  modelAccuracyMeasured=verified_state and any(r["status"] == "ok" for r in scoring["frames"]))
    return report


def execute(plan_path, *, confirmed=False):
    require(confirmed is True, "execution requires --execute and user authorization")
    require(threading.current_thread() is threading.main_thread(), "run from main thread")
    plan_path = Path(plan_path).resolve()
    plan, digest = load_plan(plan_path)
    lock, dataset = preflight(plan)
    output = Path(plan.output_dir)
    require(not output.exists(), "output exists; no overwrite/resume")
    claim = plan_path.with_name(plan_path.name + ".consumed")
    with claim.open("x", encoding="utf-8") as stream:
        json.dump({"planSha256": digest, "outputDir": str(output)}, stream)
        stream.flush()
        os.fsync(stream.fileno())
    output.mkdir(parents=True, exist_ok=False)
    predictions = initial_predictions(dataset)
    atomic_json(output / "predictions.json", predictions)
    record = {"schema": "motionbrain.cv_checkpoint_run.v1", "plan": asdict(plan), "planSha256": digest,
              "status": "starting", "runConsumed": True, "modelStateUnchanged": False,
              "checkpointSha256": lock["checkpointSha256"], "sealedFinalTest": False,
              "scope": plan.scope, "sessionIndependenceProven": False}
    atomic_json(output / "run.json", record)
    started = time.monotonic()
    def interrupt(signum, frame):
        raise InterruptedError("termination requested")
    previous = signal.signal(signal.SIGTERM, interrupt)
    process, reason, report = None, None, None
    try:
        process = multiprocessing.get_context("spawn").Process(target=_worker, args=(plan, str(output), started + plan.seconds))
        reason = supervise(process, started + plan.seconds)
    except Exception as exc:
        reason = "failed"
        record["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        signal.signal(signal.SIGTERM, previous)
        try:
            if (output / "progress.json").exists():
                record.update(_json((output / "progress.json").read_bytes()))
            predictions = _json((output / "predictions.json").read_bytes())
            if reason:
                record["status"] = reason
            elif process is None or process.exitcode != 0:
                record["status"] = "worker_exit"
            elif record["status"] != "completed":
                record["status"] = "failed"
            record["inferenceMayHaveStarted"] = any(r["status"] in ("inference_intent", "ok", "error")
                                                    for r in predictions["frames"])
            verified = (record.get("modelStateUnchanged") is True
                        and record.get("loadedStateSha256") == record.get("finalStateSha256") == lock["checkpointStateSha256"])
            report = score_records(dataset, predictions, verified_state=verified)
            report.update(scope=plan.scope, sealedFinalTest=False, sessionIndependenceProven=False,
                          checkpointSha256=lock["checkpointSha256"], lockSha256=plan.lock_sha256,
                          runStatus=record["status"])
            atomic_json(output / "report.json", report)
            if not verified or report["metrics"]["processingFailures"]:
                if record["status"] == "completed":
                    record["status"] = "failed"
            # Rendering/scoring only after the model worker has exited. No retries of inference.
            from learning.cv_checkpoint_display import render_comparisons
            previews = render_comparisons(dataset.frames, report["frames"], output / "comparisons")
            atomic_json(output / "comparisons.json", previews)
            if previews["failures"] and record["status"] == "completed":
                record["status"] = "render_failed"
        except Exception as exc:
            record.update(status="finalization_failed", error=f"{type(exc).__name__}: {exc}")
        record["elapsedSeconds"] = time.monotonic() - started
        record["finalizationOutsideModelDeadline"] = True
        if report is not None:
            report["runStatus"] = record["status"]
            atomic_json(output / "report.json", report)
        atomic_json(output / "run.json", record)
    return record
