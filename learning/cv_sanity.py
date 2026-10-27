"""Fixed six-image development-fit evaluation, imported without an ML runtime.

This is training-data diagnosis, never held-out accuracy. Model forward receives
images only. Ground truth is used after forward to summarize cached proposals.
"""
from __future__ import annotations

import hashlib
import json
import random
import copy

from learning.cv_dataset import CLASSES, COORDINATES, require
from learning.cv_evaluation import evaluate, select_predictions
from learning.cv_tensors import TorchCVDataset


class SanityEvaluationError(RuntimeError):
    """Retain partial records; caller must persist result before failing the run."""

    def __init__(self, result, cause):
        super().__init__(str(cause))
        self.result = result
        self.cause = cause


def state_digest(state_dict):
    """Hash sorted names, dtype, shape and exact tensor bytes, with framing."""
    import torch

    digest = hashlib.sha256()
    for name in sorted(state_dict):
        value = state_dict[name].detach().cpu().contiguous()
        metadata = json.dumps([name, str(value.dtype), list(value.shape)],
                              separators=(",", ":"), ensure_ascii=False).encode()
        data = value.reshape(-1).view(torch.uint8).numpy().tobytes()
        for payload in (metadata, data):
            digest.update(len(payload).to_bytes(8, "big"))
            digest.update(payload)
    return digest.hexdigest()


def _inventory(dataset):
    require(isinstance(dataset, TorchCVDataset), "sanity evaluation needs TorchCVDataset")
    frames = dataset.frames
    require(len(frames) == 6 and len({f.id for f in frames}) == 6,
            "sanity evaluation requires exactly six unique frames")
    require(all(f.approved and f.split == "train_development_pilot" for f in frames),
            "sanity evaluation requires approved development frames")
    require(len({f.session for f in frames}) == 1, "sanity inventory must be one session")
    require([sum(f.boxes[i] is not None for f in frames) for i in range(2)] == [4, 6],
            "sanity inventory requires four lip and six gripper targets")
    require(len({f.sha256 for f in frames}) == 6, "sanity inventory has duplicate images")


def _strict_success(evaluation):
    metrics = evaluation["metrics"]
    return (metrics["frames"] == 6 and metrics["processingFailures"] == 0
            and all(metrics["classes"][name]["tp"] == expected
                    and metrics["classes"][name]["fp"] == 0
                    and metrics["classes"][name]["fn"] == 0
                    for name, expected in zip(CLASSES, (4, 6)))
            and all(metrics[key]["count"] == metrics[key]["denominator"] == 4
                    for key in ("pairCoverage", "positionSuccess", "directionSuccess")))


class _ProposalTrace:
    """Observe the unmodified RPN filter return and check its reconstruction."""

    def __init__(self, model, torch, check_deadline):
        from torchvision.ops import boxes as ops

        self.model, self.torch, self.ops = model, torch, ops
        self.check_deadline = check_deadline
        self.cache = {}
        self.hooks = []
        self.rpn = model.rpn
        self.original_filter = self.rpn.filter_proposals
        self.had_instance_filter = "filter_proposals" in self.rpn.__dict__
        self.instance_filter = self.rpn.__dict__.get("filter_proposals")

    def install(self):
        self.hooks.append(self.model.transform.register_forward_hook(self._transform))
        self.rpn.filter_proposals = self._filter

    def remove(self):
        if self.had_instance_filter:
            self.rpn.filter_proposals = self.instance_filter
        else:
            self.rpn.__dict__.pop("filter_proposals", None)
        for hook in self.hooks:
            hook.remove()
        self.hooks.clear()

    def _transform(self, module, args, result):
        images = result[0]
        self.cache["imageSizes"] = list(images.image_sizes)
        self.cache["paddedSize"] = list(images.tensors.shape[-2:])

    def _filter(self, proposals, objectness, image_shapes, counts):
        self.check_deadline()
        result = self.original_filter(proposals, objectness, image_shapes, counts)
        torch, ops, rpn = self.torch, self.ops, self.rpn
        require(not rpn.training and len(image_shapes) == 1 and proposals.shape[0] == 1,
                "proposal trace requires single-image evaluation")
        require("stages" not in self.cache, "multiple RPN filter calls in one frame")
        logits = objectness.detach().reshape(1, -1)
        require(proposals.shape[1] == sum(counts) == logits.shape[1], "RPN candidate count mismatch")
        require(bool(torch.isfinite(proposals).all()) and bool(torch.isfinite(logits).all()),
                "nonfinite RPN candidates")
        device = proposals.device
        ids = torch.arange(proposals.shape[1], device=device)
        levels = torch.cat([torch.full((count,), i, dtype=torch.int64, device=device)
                            for i, count in enumerate(counts)])
        scores, all_boxes = logits[0].sigmoid(), proposals[0].detach()
        stages = {}

        def record(name, boxes, selected):
            stages[name] = (boxes.clone(), selected.clone())

        record("decoded_all", all_boxes, ids)
        selected = rpn._get_top_n_idx(logits, counts)[0]
        boxes = all_boxes[selected]
        record("topn_per_level", boxes, selected)
        boxes = ops.clip_boxes_to_image(boxes, image_shapes[0])
        record("clipped", boxes, selected)
        keep = ops.remove_small_boxes(boxes, rpn.min_size)
        boxes, selected = boxes[keep], selected[keep]
        record("size_filtered", boxes, selected)
        keep = torch.where(scores[selected] >= rpn.score_thresh)[0]
        boxes, selected = boxes[keep], selected[keep]
        record("score_filtered", boxes, selected)
        keep = ops.batched_nms(boxes, scores[selected], levels[selected], rpn.nms_thresh)
        boxes, selected = boxes[keep], selected[keep]
        record("nms", boxes, selected)
        boxes, selected = boxes[:rpn.post_nms_top_n()], selected[:rpn.post_nms_top_n()]
        record("postcap", boxes, selected)
        require(torch.equal(boxes, result[0][0]) and torch.equal(scores[selected], result[1][0]),
                "RPN trace differs from original filter output")
        self.cache.update(stages=stages, logits=logits[0].clone(), scores=scores.clone(),
                          levels=levels, counts=list(counts), filterImageSizes=list(image_shapes))
        self.check_deadline()
        return result  # The observer never replaces boxes or scores used by ROI.

    def summarize(self, frame, target):
        torch, cache = self.torch, self.cache
        require("stages" in cache and "imageSizes" in cache, "RPN/transform trace missing")
        require(cache["imageSizes"] == cache["filterImageSizes"], "RPN/transform size mismatch")
        height, width = cache["imageSizes"][0]
        scale = target["boxes"].new_tensor([width / frame.width, height / frame.height] * 2)
        gt = target["boxes"] * scale
        scores, logits, levels = cache["scores"], cache["logits"], cache["levels"]

        def candidate(index):
            index = int(index)
            level = int(levels[index])
            return {"anchorId": index, "level": level, "objectness": float(scores[index]),
                    "logit": float(logits[index]),
                    "rankGlobalByStrictGreater": 1 + int((logits > logits[index]).sum()),
                    "rankLevelByStrictGreater": 1 + int(((logits > logits[index]) & (levels == level)).sum()),
                    "sameLogitInLevel": int(((logits == logits[index]) & (levels == level)).sum())}

        rows = []
        for name, (boxes, ids) in cache["stages"].items():
            self.check_deadline()
            overlap = self.ops.box_iou(gt, boxes)
            require(bool(torch.isfinite(overlap).all()), "nonfinite diagnostic IoU")
            targets = []
            for g, label in enumerate(target["labels"]):
                values = overlap[g]
                good = torch.where(values >= 0.5)[0]
                row = {"class": CLASSES[int(label) - 1], "gtResizedXYXY": gt[g].tolist(),
                       "maxIoU": float(values.max()) if len(values) else 0.0,
                       "countIoUPoint3": int((values >= 0.3).sum()),
                       "countIoUPoint5": len(good), "best": None,
                       "highestObjectnessPositive": None}
                if len(values):
                    best = int(values.argmax())
                    row["best"] = dict(candidate(ids[best]), box=boxes[best].tolist())
                if len(good):
                    best = good[scores[ids[good]].argmax()]
                    row["highestObjectnessPositive"] = dict(candidate(ids[best]), iou=float(values[best]))
                targets.append(row)
            rows.append({"stage": name, "candidateCount": len(ids),
                         "countsByLevel": [int((levels[ids] == i).sum()) for i in range(len(cache["counts"]))],
                         "groundTruth": targets})
        return {"id": frame.id, "sourceSha256": frame.sha256,
                "originalSize": [frame.height, frame.width], "transformedSize": [height, width],
                "paddedSize": cache["paddedSize"], "anchorsByLevel": cache["counts"],
                "rpnOutputUnchanged": True, "stages": rows}


def _prediction(torch, frame, output):
    boxes, scores, labels = (output[key] for key in ("boxes", "scores", "labels"))
    require(boxes.ndim == 2 and boxes.shape[1] == 4 and scores.ndim == labels.ndim == 1
            and len(boxes) == len(scores) == len(labels), "invalid prediction tensor shapes")
    require(bool(torch.isfinite(boxes).all()) and bool(torch.isfinite(scores).all())
            and bool(((labels == 1) | (labels == 2)).all()), "nonfinite/unknown prediction")
    detections = [{"class": CLASSES[int(label) - 1], "score": float(score), "bbox_xyxy": box.tolist()}
                  for box, score, label in zip(boxes, scores, labels)]
    select_predictions(detections, frame, 0.5, 0.5)  # Validate every returned box, even below the cutoff.
    return detections


def evaluate_sanity(backend, check_deadline):
    """Evaluate one immutable model snapshot; raise with partial records on error.

    The caller owns persistence and the hard process deadline. Local checks also
    bracket model forward, trace summaries, decoding and metric computation.
    Model state mutations are restored and treated as failures, never hidden.
    """
    import numpy as np

    model, dataset, torch = backend.model, backend.dataset, backend.torch
    result = {"schema": "motionbrain.cv_sanity_evaluation.v1", "status": "failed",
              "scope": "same six development training images; no held-out/generalization claim",
              "observedSuccess": False, "modelStateSha256": None,
              "evaluation": None, "proposalDiagnostics": [],
              "predictions": {"schema": "motionbrain.cv_predictions.v1", "evidenceKind": "model",
                              "coordinates": COORDINATES,
                              "frames": [{"id": f.id, "sourceSha256": f.sha256, "status": "not_evaluated",
                                          "detections": []} for f in dataset.frames]}}
    cause = None
    state = None
    modes = [(module, module.training) for module in model.modules()]
    generator = backend.loader.generator
    require(generator is not None, "evaluation requires explicit loader generator")
    rng = (torch.get_rng_state().clone(), random.getstate(), np.random.get_state(), generator.get_state().clone())
    sampler_stats = [(sampler, copy.deepcopy(sampler.last_stats))
                     for sampler in (model.rpn.fg_bg_sampler, model.roi_heads.fg_bg_sampler)
                     if hasattr(sampler, "last_stats")]
    trace = None
    try:
        check_deadline()
        _inventory(dataset)
        require(all(t.device.type == "cpu" for t in model.state_dict().values()), "sanity model must be on CPU")
        state = {name: value.detach().clone() for name, value in model.state_dict().items()}
        result["modelStateSha256"] = state_digest(state)
        model.eval()
        trace = _ProposalTrace(model, torch, check_deadline)
        trace.install()
        with torch.inference_mode():
            for index, frame in enumerate(dataset.frames):
                entry = result["predictions"]["frames"][index]
                try:
                    check_deadline()
                    trace.cache.clear()
                    image, target = dataset[index]
                    check_deadline()
                    prediction = model([image])  # Never pass targets to the detector.
                    check_deadline()
                    require(isinstance(prediction, (list, tuple)) and len(prediction) == 1,
                            "expected exactly one image prediction")
                    entry.update(status="ok", detections=_prediction(torch, frame, prediction[0]))
                    result["proposalDiagnostics"].append(trace.summarize(frame, target))
                except BaseException as exc:
                    entry.update(status="error", error=f"{type(exc).__name__}: {exc}")
                    raise
        lookup = {frame.id: index for index, frame in enumerate(dataset.frames)}

        def image_reader(frame):
            check_deadline()
            return dataset[lookup[frame.id]][0]  # Recheck source hashes and decoding without cv2.

        result["evaluation"] = evaluate(dataset, result["predictions"], score_threshold=0.5,
                                        nms_threshold=0.5, match_iou=0.5, tolerance=10, band=10,
                                        image_reader=image_reader)
        check_deadline()
        require(result["evaluation"]["metrics"]["processingFailures"] == 0,
                "sanity evaluation contains processing failures")
        result["observedSuccess"] = _strict_success(result["evaluation"])
        result["status"] = "ok"
    except BaseException as exc:
        cause = exc
    finally:
        restoration_errors = []
        if trace is not None:
            try:
                trace.remove()
            except Exception as exc:
                restoration_errors.append(f"trace cleanup: {exc}")
        if state is not None:
            try:
                unchanged = state_digest(model.state_dict()) == result["modelStateSha256"]
                result["modelStateUnchanged"] = unchanged
                if not unchanged:
                    model.load_state_dict(state, strict=True)
                    require(state_digest(model.state_dict()) == result["modelStateSha256"], "state restore mismatch")
                    restoration_errors.append("model state mutated during evaluation (restored)")
            except Exception as exc:
                restoration_errors.append(f"model state check/restore: {exc}")
        for module, training in modes:
            module.training = training  # Preserve heterogeneous submodule modes exactly.
        for sampler, stats in sampler_stats:
            sampler.last_stats = stats
        # Restore every generator even when another restoration operation fails.
        for restore in (lambda: torch.set_rng_state(rng[0]), lambda: random.setstate(rng[1]),
                        lambda: np.random.set_state(rng[2]), lambda: generator.set_state(rng[3])):
            try:
                restore()
            except Exception as exc:
                restoration_errors.append(f"RNG restore: {exc}")
        try:
            require(all(module.training == training for module, training in modes), "module mode restoration mismatch")
            require(torch.equal(torch.get_rng_state(), rng[0]) and random.getstate() == rng[1]
                    and torch.equal(generator.get_state(), rng[3]), "RNG restoration mismatch")
            numpy_state = np.random.get_state()
            require(numpy_state[0] == rng[2][0] and np.array_equal(numpy_state[1], rng[2][1])
                    and numpy_state[2:] == rng[2][2:], "NumPy RNG restoration mismatch")
        except Exception as exc:
            restoration_errors.append(f"state restoration verification: {exc}")
        result["stateRestorationErrors"] = restoration_errors
        if restoration_errors:
            cause = cause or RuntimeError("; ".join(restoration_errors))
        if cause is not None:
            result.update(status="failed", observedSuccess=False, error=f"{type(cause).__name__}: {cause}")
    if cause is None:
        try:
            check_deadline()  # State hashing/restoration is also inside the evaluation budget.
        except BaseException as exc:
            cause = exc
            result.update(status="failed", observedSuccess=False, error=f"{type(exc).__name__}: {exc}")
    if cause is not None:
        raise SanityEvaluationError(result, cause) from cause
    return result
