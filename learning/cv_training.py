"""One-use, bounded offline development fitting. Imports no ML runtime at startup."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import copy
import io
import json
import math
import multiprocessing
import os
from pathlib import Path
import signal
import threading
import time

from learning.cv_dataset import CVDataset, require, validate_mixed_inventory
from learning.cv_sampling import RecordedROISampler, RecordedRPNSampler, sampling_settings

MODEL = "fasterrcnn_mobilenet_v3_large_320_fpn"
HEAD = "roi_heads.box_predictor."
DETECTION_MODES = ("detection_heads", "mixed_session_heads")
DETECTION_HEADS = ("rpn.head.", "roi_heads.box_head.", HEAD)
EVALUATION_STEPS = (0, 30, 100, 200, 300)
FINISHED = ("epoch_limit", "step_limit", "fit_observed")


@dataclass(frozen=True)
class Plan:
    manifest: str
    manifest_sha256: str
    weights: str
    weights_sha256: str
    output_dir: str
    epochs: int = 10
    steps: int = 30
    seconds: int = 900
    seed: int = 20261027
    roi_sampling: str = "default"
    mode: str = "box_predictor_only"


def load_plan(path):
    path = Path(path).resolve()
    raw = path.read_bytes()
    def unique_object(pairs):
        require(len({k for k, _ in pairs}) == len(pairs), "duplicate plan key")
        return dict(pairs)
    doc = json.loads(raw, object_pairs_hook=unique_object)
    require(isinstance(doc, dict) and doc.pop("schema", None) == "motionbrain.cv_training_plan.v1",
            "unsupported training plan")
    require(set(doc) <= set(Plan.__dataclass_fields__), "unknown plan fields")
    for key in ("manifest", "weights", "output_dir"):
        value = doc.get(key)
        require(isinstance(value, str) and bool(value.strip()), f"invalid {key}")
        doc[key] = str((path.parent / value).resolve())
    for key in ("manifest_sha256", "weights_sha256"):
        value = doc.get(key)
        require(isinstance(value, str) and len(value) == 64
                and all(c in "0123456789abcdef" for c in value), f"invalid {key}")
    plan = Plan(**doc)
    require(type(plan.mode) is str and plan.mode in ("box_predictor_only", *DETECTION_MODES),
            "invalid training mode")
    sanity = plan.mode in DETECTION_MODES
    for key, cap in (("epochs", 100 if sanity else 10), ("steps", 300 if sanity else 30), ("seconds", 900)):
        value = getattr(plan, key)
        require(type(value) is int and 1 <= value <= cap, f"{key} must be in 1..{cap}")
    require(type(plan.seed) is int and 0 <= plan.seed < 2**32, "invalid seed")
    sampling_settings(plan.roi_sampling)
    require(not sanity or plan.roi_sampling == "negative_cap_3x",
            "detection_heads requires negative_cap_3x ROI sampling")
    return plan, hashlib.sha256(raw).hexdigest()


def verified_bytes(path, expected):
    raw = Path(path).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == expected, f"sha256 mismatch: {path}")
    return raw


def preflight(plan):
    dataset = CVDataset(plan.manifest, split="train_development_pilot")
    require(dataset.manifest_sha256 == plan.manifest_sha256, "manifest sha256 mismatch")
    if plan.mode == "mixed_session_heads":
        validate_mixed_inventory(dataset.frames)
    else:
        require(len(dataset) == 6, "pilot must contain exactly six approved frames")
        require(len({f.session for f in dataset.frames}) == 1, "pilot must be one session")
        require(sum(b is not None for f in dataset.frames for b in f.boxes) == 10,
                "pilot must contain ten boxes")
        require(sum(f.boxes[0] is None for f in dataset.frames) == 2
                and all(f.boxes[1] is not None for f in dataset.frames), "pilot presence inventory mismatch")
    for frame in dataset.frames:
        verified_bytes(frame.path, frame.sha256)
    verified_bytes(plan.weights, plan.weights_sha256)
    return {"manifestSha256": dataset.manifest_sha256, "frames": [f.id for f in dataset.frames],
            "session": dataset.frames[0].session if plan.mode != "mixed_session_heads" else None,
            "sessions": sorted({f.session for f in dataset.frames}), "weightsSha256": plan.weights_sha256,
            "model": MODEL, "purpose": "train_development_only", "heldOutEvaluation": False}


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n"); stream.flush(); os.fsync(stream.fileno())
    os.replace(temporary, path)


class BudgetExpired(Exception):
    pass


def run_loop(backend, plan, emit, deadline, *, clock=time.monotonic):
    """Backend-independent control loop. A recorded intent may exceed completions."""
    state = {"status": "running", "stepIntents": 0, "stepsCompleted": 0, "epochsCompleted": 0}
    sanity = plan.mode in DETECTION_MODES
    if sanity:
        state.update(observedSuccess=False, verifiedArtifact=False,
                     evaluations=[{"step": step, "status": "not_reached"} for step in EVALUATION_STEPS])
    def check():
        if clock() >= deadline:
            raise BudgetExpired("wall-clock budget exhausted")
    def publish(event):
        emit(copy.deepcopy(dict(state, event=event)))
    def evaluate_if_due():
        if not sanity or state["stepsCompleted"] not in EVALUATION_STEPS:
            return False
        check()
        row = next(r for r in state["evaluations"] if r["step"] == state["stepsCompleted"])
        row["status"] = "running"
        publish("state_eval_start")
        try:
            check()
            result = backend.evaluate(state["stepsCompleted"], check)
            require(isinstance(result, dict) and isinstance(result.get("metrics"), dict),
                    "invalid evaluation summary")
            require(type(result.get("observedSuccess")) is bool, "invalid evaluation success flag")
            digest = result.get("modelStateSha256")
            require(isinstance(digest, str) and len(digest) == 64
                    and all(c in "0123456789abcdef" for c in digest), "invalid evaluation state digest")
            row.update(result, status="completed", step=state["stepsCompleted"])
            state["observedSuccess"] = result["observedSuccess"]
            publish("state_eval_complete")
            check()
        except Exception as exc:
            if row["status"] != "completed":
                row.update(status="interrupted" if isinstance(exc, BudgetExpired) else "failed",
                           error=f"{type(exc).__name__}: {exc}")
            raise
        if state["observedSuccess"]:
            state["status"] = "fit_observed"; publish("fit_observed")
            return True
        return False
    try:
        publish("ready")
        if evaluate_if_due():
            return state
        for epoch in range(plan.epochs):
            check()
            batches_seen = 0
            for batch in backend.batches(epoch):
                batches_seen += 1
                check()
                if state["stepIntents"] >= plan.steps:
                    state["status"] = "step_limit"; publish("finished"); return state
                state.update(epochIndex=epoch + 1, batchIndex=batches_seen)
                state.pop("lastLosses", None); state.pop("lastBatch", None)
                try:
                    losses = backend.loss_and_grad(batch)
                finally:
                    diagnostics = getattr(backend, "batch_diagnostics", None)
                    if diagnostics is not None:
                        state["lastBatch"] = diagnostics()
                require(bool(losses) and all(type(v) in (int, float) and math.isfinite(v)
                                            for v in losses.values()), "nonfinite/empty losses")
                state["lastLosses"] = losses
                check()
                state["stepIntents"] += 1
                publish("step_intent")  # Durable before optimizer mutation.
                check()
                backend.step()
                state["stepsCompleted"] += 1
                publish("step_complete")
                check()
                if evaluate_if_due():
                    return state
                if state["stepIntents"] >= plan.steps:
                    state["status"] = "step_limit"; publish("finished"); return state
            require(batches_seen > 0, "empty training epoch")
            state["epochsCompleted"] = epoch + 1
            publish("epoch_complete")
        state["status"] = "epoch_limit"; publish("finished"); return state
    except BudgetExpired as exc:
        state.update(status="time_limit", error=str(exc)); publish("stopped"); return state
    except Exception as exc:
        state.update(status="failed", error=f"{type(exc).__name__}: {exc}"); publish("failed"); return state


def frozen_backbone_norms(module):
    """Match the COCO builder's normalization without invoking a download API."""
    import torch
    from torchvision.ops.misc import FrozenBatchNorm2d
    for name, child in module.named_children():
        if isinstance(child, torch.nn.BatchNorm2d):
            setattr(module, name, FrozenBatchNorm2d(child.num_features, eps=child.eps))
        else:
            frozen_backbone_norms(child)


def restore_training_checkpoint(model, saved, frames, torch):
    """Warm-start all three-class weights; never replace the trained predictor."""
    from learning.cv_sanity import state_digest
    require(isinstance(saved, dict) and isinstance(saved.get("state_dict"), dict)
            and isinstance(saved.get("provenance"), dict), "training checkpoint required")
    provenance = saved["provenance"]
    require(provenance.get("model") == MODEL, "checkpoint model mismatch")
    parent_ids = provenance.get("frames")
    require(isinstance(parent_ids, list) and parent_ids
            and len(parent_ids) == len(set(parent_ids))
            and set(parent_ids) <= {f.id for f in frames}, "parent training frames must be retained")
    parent_plan = provenance.get("plan", {})
    parent_manifest = parent_plan.get("manifest")
    require(isinstance(parent_manifest, str), "parent manifest missing")
    verified_bytes(parent_manifest, provenance.get("manifestSha256"))
    parent_data = CVDataset(parent_manifest, split="train_development_pilot")
    require([f.id for f in parent_data.frames] == parent_ids, "parent frame inventory mismatch")
    current = {f.id: f for f in frames}
    for previous in parent_data.frames:
        retained = current[previous.id]
        require(all(getattr(previous, key) == getattr(retained, key)
                    for key in ("sha256", "session", "group", "width", "height", "presence", "boxes")),
                "parent images/labels/session must be retained unchanged")
    digest = state_digest(saved["state_dict"])
    require(digest == provenance.get("modelStateSha256"), "parent checkpoint state digest mismatch")
    require(all(bool(torch.isfinite(t).all()) for t in saved["state_dict"].values()),
            "nonfinite parent checkpoint")
    model.load_state_dict(saved["state_dict"], strict=True)
    require(state_digest(model.state_dict()) == digest, "warm-start state changed")
    return digest


class TorchBackend:
    def __init__(self, plan):
        import torch
        import torchvision
        from torchvision.models.detection import fasterrcnn_mobilenet_v3_large_320_fpn
        from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
        from learning.cv_tensors import TorchCVDataset, detection_collate

        require(torch.__version__.split("+")[0] == "2.14.1"
                and torchvision.__version__.split("+")[0] == "0.29.1", "unvalidated runtime versions")
        self.torch = torch
        self.sanity = plan.mode in DETECTION_MODES
        self.mixed_session = plan.mode == "mixed_session_heads"
        self.plan = plan
        torch.set_num_threads(4); torch.manual_seed(plan.seed)
        torch.set_default_device("cpu"); torch.set_default_dtype(torch.float32)
        self.dataset = TorchCVDataset(plan.manifest, split="train_development_pilot")
        require(self.dataset.manifest_sha256 == plan.manifest_sha256, "manifest changed after preflight")
        self.model = fasterrcnn_mobilenet_v3_large_320_fpn(
            weights=None, weights_backbone=None, num_classes=3 if self.mixed_session else 91)
        frozen_backbone_norms(self.model.backbone)
        state = torch.load(io.BytesIO(verified_bytes(plan.weights, plan.weights_sha256)),
                           map_location="cpu", weights_only=True)
        if self.mixed_session:
            self.initial_state_sha = restore_training_checkpoint(self.model, state, self.dataset.frames, torch)
        else:
            self.model.load_state_dict(state, strict=True)
            features = self.model.roi_heads.box_predictor.cls_score.in_features
            self.model.roi_heads.box_predictor = FastRCNNPredictor(features, 3)
        self.roi_sampler = RecordedROISampler(self.model.roi_heads.fg_bg_sampler, plan.roi_sampling)
        self.model.roi_heads.fg_bg_sampler = self.roi_sampler
        self.rpn_sampler = None
        if self.sanity:
            self.rpn_sampler = RecordedRPNSampler(self.model.rpn.fg_bg_sampler)
            self.model.rpn.fg_bg_sampler = self.rpn_sampler
        self.frame_ids = {f.image_id: f.id for f in self.dataset.frames}
        self.current_batch_ids = []
        prefixes = DETECTION_HEADS if self.sanity else (HEAD,)
        for name, parameter in self.model.named_parameters():
            parameter.requires_grad_(name.startswith(prefixes))
        self.trainable_names = {name for name, p in self.model.named_parameters() if p.requires_grad}
        if self.sanity:
            require(all(any(name.startswith(prefix) for name in self.trainable_names)
                        for prefix in DETECTION_HEADS), "missing detection head parameters")
        self.model.train(); self.model.backbone.eval()
        self.parameters = [p for p in self.model.parameters() if p.requires_grad]
        require(bool(self.parameters), "empty trainable head")
        self.optimizer = torch.optim.SGD(self.parameters, lr=0.005, momentum=0.9, weight_decay=0.0005)
        self.loader = torch.utils.data.DataLoader(self.dataset, batch_size=2, shuffle=True,
            num_workers=0, collate_fn=detection_collate,
            generator=torch.Generator(device="cpu").manual_seed(plan.seed))
        self.frozen_before = self.frozen_digest()
        self.last_gradient_norms = {}
        if self.sanity:
            self.check_scope()

    def frozen_digest(self):
        if self.sanity:
            from learning.cv_sanity import state_digest
            return state_digest({name: value for name, value in self.model.state_dict().items()
                                 if name not in self.trainable_names})
        digest = hashlib.sha256()
        for name, tensor in self.model.state_dict().items():
            if not name.startswith(HEAD):
                digest.update(name.encode()); digest.update(tensor.detach().cpu().contiguous().numpy().tobytes())
        return digest.hexdigest()

    def check_scope(self):
        require({name for name, p in self.model.named_parameters() if p.requires_grad}
                == self.trainable_names, "trainable scope changed")
        expected = [p for name, p in self.model.named_parameters() if name in self.trainable_names]
        actual = [p for group in self.optimizer.param_groups for p in group["params"]]
        require(len(actual) == len(expected) and {id(p) for p in actual} == {id(p) for p in expected},
                "optimizer parameter membership mismatch")
        require(not any(module.training for module in self.model.backbone.modules()),
                "frozen backbone must remain in evaluation mode")
        require(self.frozen_digest() == self.frozen_before, "frozen parameters/buffers changed")

    def batches(self, epoch):
        return iter(self.loader)

    def loss_and_grad(self, batch):
        torch = self.torch
        self.current_batch_ids = []
        self.roi_sampler.last_stats = []
        self.last_gradient_norms = {}
        if self.sanity:
            self.rpn_sampler.last_stats = []
            self.check_scope()
        for target in batch[1]:
            self.current_batch_ids.append(self.frame_ids[int(target["image_id"].item())])
        self.optimizer.zero_grad(set_to_none=True)
        losses = self.model(*batch)
        if self.sanity:
            require(set(losses) == {"loss_objectness", "loss_rpn_box_reg", "loss_classifier", "loss_box_reg"},
                    "detection training requires all four losses")
            require(len(self.rpn_sampler.last_stats) == len(self.current_batch_ids),
                    "RPN sampling diagnostics missing or misaligned")
        require(len(self.roi_sampler.last_stats) == len(self.current_batch_ids),
                "ROI sampling diagnostics missing or misaligned")
        require(bool(losses) and all(bool(torch.isfinite(v).all()) for v in losses.values()),
                "nonfinite/empty losses")
        sum(losses.values()).backward()
        require(all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in self.parameters),
                "missing/nonfinite head gradients")
        if self.sanity:
            for prefix in DETECTION_HEADS:
                total = sum(float(p.grad.detach().double().square().sum())
                            for name, p in self.model.named_parameters() if name.startswith(prefix))
                require(math.isfinite(total), "nonfinite group gradient norm")
                self.last_gradient_norms[prefix] = math.sqrt(total)
            self.check_scope()
        return {k:float(v.detach()) for k,v in losses.items()}

    def batch_diagnostics(self):
        rows = self.roi_sampler.last_stats
        complete = len(rows) == len(self.current_batch_ids) and bool(rows)
        total = sum(row["selectedTotal"] for row in rows) if complete else None
        result = {"roiSampling": self.roi_sampler.policy, "imageIds": list(self.current_batch_ids),
                "samplingComplete": complete,
                "images": [dict(row, id=image_id) for row, image_id in zip(rows, self.current_batch_ids)],
                "classificationDenominator": total, "boxRegressionDenominator": total}
        if self.sanity:
            rpn = self.rpn_sampler.last_stats
            complete = len(rpn) == len(self.current_batch_ids) and bool(rpn)
            denominator = sum(row["selectedTotal"] for row in rpn) if complete else None
            result.update(gradientNorms=dict(self.last_gradient_norms), rpn={
                "samplingComplete": complete,
                "images": [dict(row, id=i) for row, i in zip(rpn, self.current_batch_ids)],
                "objectnessDenominator": denominator, "boxRegressionDenominator": denominator,
                "batchSizePerImage": 256, "positiveFractionCap": 0.5})
        return result

    def step(self):
        if self.sanity:
            self.check_scope()
        self.optimizer.step()
        require(all(bool(self.torch.isfinite(p).all()) for p in self.parameters),
                "nonfinite parameters after optimizer update")
        if self.sanity:
            self.check_scope()

    def evaluate(self, step, check_deadline):
        from learning.cv_sanity import evaluate_sanity, SanityEvaluationError
        self.check_scope()
        directory = self.output / "evaluations"
        directory.mkdir(exist_ok=True)
        path = directory / f"step-{step:06d}.json"
        require(not path.exists(), "evaluation record already exists")
        try:
            result = evaluate_sanity(self, check_deadline)
        except SanityEvaluationError as exc:
            atomic_json(path, dict(exc.result, step=step))
            raise exc.cause
        atomic_json(path, dict(result, step=step))
        self.check_scope()
        return {"observedSuccess": result["observedSuccess"],
                "modelStateSha256": result["modelStateSha256"],
                "metrics": result["evaluation"]["metrics"],
                "record": str(path.relative_to(self.output))}

    def save(self, path, provenance):
        require(self.frozen_digest() == self.frozen_before, "frozen parameters/buffers changed")
        with Path(path).open("xb") as stream:
            self.torch.save({"state_dict": self.model.state_dict(), "provenance": provenance}, stream)
            stream.flush(); os.fsync(stream.fileno())

    def verify_checkpoint(self, path, provenance):
        from learning.cv_sanity import state_digest
        self.check_scope()
        saved = self.torch.load(path, map_location="cpu", weights_only=True)
        require(saved.get("provenance") == provenance, "checkpoint provenance mismatch")
        require(state_digest(saved["state_dict"]) == state_digest(self.model.state_dict())
                == provenance["modelStateSha256"], "checkpoint tensor state mismatch")
        require(all(bool(self.torch.isfinite(t).all()) for t in saved["state_dict"].values()),
                "nonfinite checkpoint tensors")
        return provenance["modelStateSha256"]


def _worker(plan, output, deadline):
    """Only launched after the parent consumes a plan and creates a new directory."""
    import socket
    import urllib.request
    def deny_network(*args, **kwargs):
        raise RuntimeError("network access is disabled in the training worker")
    socket.create_connection = deny_network; socket.socket.connect = deny_network
    urllib.request.urlopen = deny_network
    output = Path(output)
    def emit(state):
        with (output / "events.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(dict(state, monotonic=time.monotonic()), allow_nan=False) + "\n")
            stream.flush(); os.fsync(stream.fileno())
        atomic_json(output / "progress.json", state)
    try:
        sanity = plan.mode in DETECTION_MODES
        source_paths = [Path(__file__), Path(__file__).with_name("cv_sampling.py")]
        if sanity:
            source_paths += [Path(__file__).with_name(name) for name in
                             ("cv_sanity.py", "cv_evaluation.py", "cv_tensors.py", "cv_dataset.py")]
        source_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
        provenance = preflight(plan)
        initial_provenance = copy.deepcopy(provenance)
        backend = TorchBackend(plan)
        if plan.mode == "mixed_session_heads":
            provenance["initialStateSha256"] = backend.initial_state_sha
        if sanity:
            backend.output = output
        state = run_loop(backend, plan, emit, deadline)
        if state["status"] not in FINISHED:
            return
        if time.monotonic() >= deadline:
            raise BudgetExpired("no budget remains for checkpoint")
        provenance.update(plan=asdict(plan), result=state, torch=str(backend.torch.__version__),
                          runtimeRequired={"torch": "2.14.1", "torchvision": "0.29.1"},
                          settings={"device": "cpu", "dtype": "float32", "batchSize": 2,
                                    "workers": 0, "threads": 4, "shuffle": True, "augmentation": False,
                                    "optimizer": "SGD", "lr": 0.005, "momentum": 0.9, "weightDecay": 0.0005,
                                    "roiSampling": sampling_settings(plan.roi_sampling)},
                          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                          samplingImplementationSha256=hashlib.sha256(
                              Path(__file__).with_name("cv_sampling.py").read_bytes()).hexdigest(),
                          trainable=plan.mode, frozenSha256=backend.frozen_before)
        if sanity:
            from learning.cv_sanity import state_digest
            require(preflight(plan) == initial_provenance,
                    "input provenance changed during execution")
            require(source_hashes == {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths},
                    "implementation changed during execution")
            digest = state_digest(backend.model.state_dict())
            current_eval = next((r for r in state["evaluations"]
                                 if r["step"] == state["stepsCompleted"] and r["status"] == "completed"), None)
            require(current_eval is None or current_eval["modelStateSha256"] == digest,
                    "evaluated state differs from checkpoint state")
            require(not state["observedSuccess"] or current_eval is not None,
                    "success is not bound to final evaluated state")
            provenance.update(modelStateSha256=digest, implementationHashes=source_hashes,
                              evaluationStep=current_eval["step"] if current_eval else None)
            if time.monotonic() >= deadline:
                raise BudgetExpired("no budget remains for checkpoint validation")
        temporary = output / "checkpoint.pt.partial"
        backend.save(temporary, provenance)
        if time.monotonic() >= deadline:
            raise BudgetExpired("budget exhausted while saving checkpoint")
        if sanity:
            verified_digest = backend.verify_checkpoint(temporary, provenance)
            require(verified_digest == digest, "verified checkpoint state mismatch")
            if time.monotonic() >= deadline:
                raise BudgetExpired("budget exhausted while verifying checkpoint")
        os.replace(temporary, output / "checkpoint.pt")
        checkpoint_sha = hashlib.sha256((output / "checkpoint.pt").read_bytes()).hexdigest()
        if sanity:
            if time.monotonic() >= deadline:
                raise BudgetExpired("budget exhausted while finalizing checkpoint")
            state.update(verifiedArtifact=True, checkpointStateSha256=digest,
                         checkpointEvaluationStep=provenance["evaluationStep"],
                         outcome="fit_verified" if state["observedSuccess"] else "inconclusive_budget")
        state.update(checkpoint="checkpoint.pt", checkpointSha256=checkpoint_sha)
        emit(dict(state, event="checkpoint_complete"))
    except Exception as exc:
        progress = output / "progress.json"
        state = json.loads(progress.read_text()) if progress.exists() else {
            "stepIntents": 0, "stepsCompleted": 0, "epochsCompleted": 0}
        state.update(status="time_limit" if isinstance(exc, BudgetExpired) else "failed",
                     error=f"{type(exc).__name__}: {exc}", event="worker_failed")
        if plan.mode in DETECTION_MODES:
            state.update(verifiedArtifact=False, outcome="incomplete")
        emit(state)


def supervise(process, deadline, *, clock=time.monotonic):
    """A killed worker's last durable intent is retained; it is never retried."""
    reason = None
    try:
        process.start()
        process.join(max(0, deadline - clock()))
        if process.is_alive():
            reason = "time_limit"
    except (KeyboardInterrupt, InterruptedError):
        reason = "interrupted"
    except BaseException:
        reason = "supervisor_failure"
        raise
    finally:
        if reason and process.pid and process.is_alive():
            process.terminate(); process.join(1)
            if process.is_alive():
                process.kill(); process.join(1)
            require(not process.is_alive(), "worker did not stop after kill")
    return reason


def execute(plan_path, *, confirmed=False):
    require(confirmed is True, "execution requires --execute and user authorization")
    require(threading.current_thread() is threading.main_thread(), "run from the main thread")
    plan_path = Path(plan_path).resolve()
    plan, plan_sha = load_plan(plan_path)
    output = Path(plan.output_dir)
    require(not output.exists(), "output directory already exists; no resume or overwrite")
    claim = plan_path.with_name(plan_path.name + ".consumed")
    with claim.open("x", encoding="utf-8") as stream:
        json.dump({"planSha256": plan_sha, "outputDir": str(output), "pid": os.getpid()}, stream)
        stream.flush(); os.fsync(stream.fileno())
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic(); deadline = started + plan.seconds
    record = {"schema": "motionbrain.cv_training_run.v1", "plan": asdict(plan),
              "planSha256": plan_sha, "runConsumed": True, "status": "starting",
              "heldOutEvaluation": False, "stepsCompleted": 0, "stepIntents": 0}
    atomic_json(output / "run.json", record)
    def interrupted(signum, frame):
        raise InterruptedError("termination requested")
    previous = signal.signal(signal.SIGTERM, interrupted)
    reason = None
    error = None
    process = None
    try:
        process = multiprocessing.get_context("spawn").Process(target=_worker, args=(plan, str(output), deadline))
        reason = supervise(process, deadline)
    except Exception as exc:
        reason = "failed"
        error = f"{type(exc).__name__}: {exc}"
    finally:
        signal.signal(signal.SIGTERM, previous)
        # Recover durable counts even when spawning/joining/termination raised.
        try:
            progress = output / "progress.json"
            if progress.exists():
                record.update(json.loads(progress.read_text()))
        except Exception as exc:
            reason = "failed"
            error = f"progress recovery failed: {exc}"
        if reason:
            record["status"] = reason
        elif process is None or process.exitcode != 0:
            record.update(status="worker_exit", exitCode=process.exitcode if process else None)
        elif record["status"] in FINISHED and not record.get("checkpoint"):
            record["status"] = "checkpoint_incomplete"
        elif record["status"] in ("starting", "running"):
            record["status"] = "worker_incomplete"
        if error:
            record["error"] = error
        if record["status"] in FINISHED:
            try:
                require(record.get("checkpoint") == "checkpoint.pt", "invalid checkpoint path")
                verified_bytes(output / "checkpoint.pt", record.get("checkpointSha256"))
                if plan.mode in DETECTION_MODES:
                    require(record.get("verifiedArtifact") is True, "checkpoint safe reload not verified")
                    require(record["status"] != "fit_observed" or record.get("observedSuccess") is True,
                            "fit completion missing observation")
            except Exception as exc:
                record.update(status="failed", error=f"checkpoint verification failed: {exc}")
        record["elapsedSeconds"] = time.monotonic() - started
        if record["elapsedSeconds"] >= plan.seconds and record["status"] in FINISHED:
            record["status"] = "time_limit"
        if plan.mode in DETECTION_MODES:
            record.setdefault("observedSuccess", False)
            record.setdefault("verifiedArtifact", False)
            if record["status"] not in FINISHED:
                record.update(verifiedArtifact=False, outcome="incomplete")
                for evaluation in record.get("evaluations", []):
                    if evaluation.get("status") == "running":
                        evaluation.update(status="interrupted" if record["status"] in
                                          ("time_limit", "interrupted") else "failed",
                                          error="worker ended before evaluation completed",
                                          termination=record["status"])
        record["trainingMayHaveStarted"] = record.get("stepIntents", 0) > 0
        atomic_json(output / "run.json", record)
    return record
