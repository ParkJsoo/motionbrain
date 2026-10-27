"""One-use, bounded offline development fitting. Imports no ML runtime at startup."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import io
import json
import math
import multiprocessing
import os
from pathlib import Path
import signal
import threading
import time

from learning.cv_dataset import CVDataset, require

MODEL = "fasterrcnn_mobilenet_v3_large_320_fpn"
HEAD = "roi_heads.box_predictor."


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
    for key, cap in (("epochs", 10), ("steps", 30), ("seconds", 900)):
        value = getattr(plan, key)
        require(type(value) is int and 1 <= value <= cap, f"{key} must be in 1..{cap}")
    require(type(plan.seed) is int and 0 <= plan.seed < 2**32, "invalid seed")
    return plan, hashlib.sha256(raw).hexdigest()


def verified_bytes(path, expected):
    raw = Path(path).read_bytes()
    require(hashlib.sha256(raw).hexdigest() == expected, f"sha256 mismatch: {path}")
    return raw


def preflight(plan):
    dataset = CVDataset(plan.manifest, split="train_development_pilot")
    require(dataset.manifest_sha256 == plan.manifest_sha256, "manifest sha256 mismatch")
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
            "session": dataset.frames[0].session, "weightsSha256": plan.weights_sha256,
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
    def check():
        if clock() >= deadline:
            raise BudgetExpired("wall-clock budget exhausted")
    def publish(event):
        emit(dict(state, event=event))
    try:
        publish("ready")
        for epoch in range(plan.epochs):
            check()
            batches_seen = 0
            for batch in backend.batches(epoch):
                batches_seen += 1
                check()
                if state["stepIntents"] >= plan.steps:
                    state["status"] = "step_limit"; publish("finished"); return state
                losses = backend.loss_and_grad(batch)
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
        torch.set_num_threads(4); torch.manual_seed(plan.seed)
        torch.set_default_device("cpu"); torch.set_default_dtype(torch.float32)
        self.dataset = TorchCVDataset(plan.manifest, split="train_development_pilot")
        require(self.dataset.manifest_sha256 == plan.manifest_sha256, "manifest changed after preflight")
        self.model = fasterrcnn_mobilenet_v3_large_320_fpn(
            weights=None, weights_backbone=None, num_classes=91)
        frozen_backbone_norms(self.model.backbone)
        state = torch.load(io.BytesIO(verified_bytes(plan.weights, plan.weights_sha256)),
                           map_location="cpu", weights_only=True)
        self.model.load_state_dict(state, strict=True)
        features = self.model.roi_heads.box_predictor.cls_score.in_features
        self.model.roi_heads.box_predictor = FastRCNNPredictor(features, 3)
        for name, parameter in self.model.named_parameters():
            parameter.requires_grad_(name.startswith(HEAD))
        self.model.train(); self.model.backbone.eval()
        self.parameters = [p for p in self.model.parameters() if p.requires_grad]
        require(bool(self.parameters), "empty trainable head")
        self.optimizer = torch.optim.SGD(self.parameters, lr=0.005, momentum=0.9, weight_decay=0.0005)
        self.loader = torch.utils.data.DataLoader(self.dataset, batch_size=2, shuffle=True,
            num_workers=0, collate_fn=detection_collate,
            generator=torch.Generator(device="cpu").manual_seed(plan.seed))
        self.frozen_before = self.frozen_digest()

    def frozen_digest(self):
        digest = hashlib.sha256()
        for name, tensor in self.model.state_dict().items():
            if not name.startswith(HEAD):
                digest.update(name.encode()); digest.update(tensor.detach().cpu().contiguous().numpy().tobytes())
        return digest.hexdigest()

    def batches(self, epoch):
        return iter(self.loader)

    def loss_and_grad(self, batch):
        torch = self.torch
        self.optimizer.zero_grad(set_to_none=True)
        losses = self.model(*batch)
        require(bool(losses) and all(bool(torch.isfinite(v).all()) for v in losses.values()),
                "nonfinite/empty losses")
        sum(losses.values()).backward()
        require(all(p.grad is not None and bool(torch.isfinite(p.grad).all()) for p in self.parameters),
                "missing/nonfinite head gradients")
        return {k:float(v.detach()) for k,v in losses.items()}

    def step(self):
        self.optimizer.step()
        require(all(bool(self.torch.isfinite(p).all()) for p in self.parameters),
                "nonfinite parameters after optimizer update")

    def save(self, path, provenance):
        require(self.frozen_digest() == self.frozen_before, "frozen parameters/buffers changed")
        with Path(path).open("xb") as stream:
            self.torch.save({"state_dict": self.model.state_dict(), "provenance": provenance}, stream)
            stream.flush(); os.fsync(stream.fileno())


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
        provenance = preflight(plan)
        backend = TorchBackend(plan)
        state = run_loop(backend, plan, emit, deadline)
        if state["status"] not in ("epoch_limit", "step_limit"):
            return
        if time.monotonic() >= deadline:
            raise BudgetExpired("no budget remains for checkpoint")
        provenance.update(plan=asdict(plan), result=state, torch=backend.torch.__version__,
                          runtimeRequired={"torch": "2.14.1", "torchvision": "0.29.1"},
                          settings={"device": "cpu", "dtype": "float32", "batchSize": 2,
                                    "workers": 0, "threads": 4, "shuffle": True, "augmentation": False,
                                    "optimizer": "SGD", "lr": 0.005, "momentum": 0.9, "weightDecay": 0.0005},
                          implementationSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                          trainable="box_predictor_only", frozenSha256=backend.frozen_before)
        temporary = output / "checkpoint.pt.partial"
        backend.save(temporary, provenance)
        if time.monotonic() >= deadline:
            raise BudgetExpired("budget exhausted while saving checkpoint")
        os.replace(temporary, output / "checkpoint.pt")
        state.update(checkpoint="checkpoint.pt", checkpointSha256=hashlib.sha256(
            (output / "checkpoint.pt").read_bytes()).hexdigest())
        emit(dict(state, event="checkpoint_complete"))
    except Exception as exc:
        progress = output / "progress.json"
        state = json.loads(progress.read_text()) if progress.exists() else {
            "stepIntents": 0, "stepsCompleted": 0, "epochsCompleted": 0}
        state.update(status="time_limit" if isinstance(exc, BudgetExpired) else "failed",
                     error=f"{type(exc).__name__}: {exc}", event="worker_failed")
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
        elif record["status"] in ("epoch_limit", "step_limit") and not record.get("checkpoint"):
            record["status"] = "checkpoint_incomplete"
        elif record["status"] in ("starting", "running"):
            record["status"] = "worker_incomplete"
        if error:
            record["error"] = error
        if record["status"] in ("epoch_limit", "step_limit"):
            try:
                require(record.get("checkpoint") == "checkpoint.pt", "invalid checkpoint path")
                verified_bytes(output / "checkpoint.pt", record.get("checkpointSha256"))
            except Exception as exc:
                record.update(status="failed", error=f"checkpoint verification failed: {exc}")
        record["elapsedSeconds"] = time.monotonic() - started
        if record["elapsedSeconds"] >= plan.seconds and record["status"] in ("epoch_limit", "step_limit"):
            record["status"] = "time_limit"
        record["trainingMayHaveStarted"] = record.get("stepIntents", 0) > 0
        atomic_json(output / "run.json", record)
    return record
