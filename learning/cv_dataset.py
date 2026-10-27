"""Offline two-class image data. No torch, model download, or device access."""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

CLASSES = ("lip_balm", "gripper")
COORDINATES = "original_xyxy_no_rotation_no_mirror"
SCHEMAS = {"motionbrain.cv_dataset.v1", "motionbrain-p1-six-frame-development-export-v1"}
SPLITS = {"train", "train_development_pilot", "validation", "test"}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def box(value, width, height):
    require(isinstance(value, (list, tuple)) and len(value) == 4
            and all(number(v) for v in value), "bbox must contain four finite numbers")
    x0, y0, x1, y1 = value
    require(0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height, "bbox outside image or empty")
    return tuple(float(v) for v in value)


def text(value, name):
    require(isinstance(value, str) and bool(value.strip()), f"invalid {name}")
    return value


def indexed(rows, key="id"):
    require(isinstance(rows, list), "records must be a list")
    result = {}
    for row in rows:
        require(isinstance(row, dict), "record must be an object")
        name = text(row.get(key), key)
        require(name not in result, f"duplicate {key}: {name}")
        result[name] = row
    return result


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


@dataclass(frozen=True)
class Frame:
    id: str
    image_id: int
    path: Path
    sha256: str
    width: int
    height: int
    session: str
    group: str
    split: str
    presence: tuple
    boxes: tuple
    approved: bool

    def target(self):
        require(self.approved, "candidate reference cannot be a training/evaluation target")
        present = [(i + 1, b) for i, b in enumerate(self.boxes) if b is not None]
        return {"image_id": self.image_id, "boxes": [list(b) for _, b in present],
                "labels": [i for i, _ in present],
                "area": [(b[2] - b[0]) * (b[3] - b[1]) for _, b in present],
                "iscrowd": [0] * len(present)}

    def read_image(self):
        """Hash and decode each access; return RGB uint8 HWC without resizing."""
        import cv2
        import numpy as np

        raw = self.path.read_bytes()
        require(hashlib.sha256(raw).hexdigest() == self.sha256, f"image hash mismatch: {self.id}")
        require(bool(raw), f"empty image: {self.id}")
        try:
            image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8),
                                 cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        except cv2.error as exc:
            raise ValueError(f"image decode failed: {self.id}") from exc
        require(image is not None and image.shape[:2] == (self.height, self.width),
                f"image decode/size mismatch: {self.id}")
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def _frame(row, root, approved):
    width, height = row.get("width"), row.get("height")
    require(type(width) is int and type(height) is int and width > 0 and height > 0,
            "invalid image dimensions")
    require(row.get("coordinates") == COORDINATES, "unsupported coordinates")
    relative = Path(text(row.get("image"), "image"))
    require(not relative.is_absolute() and ".." not in relative.parts, "image path must be contained")
    path = (root / relative).resolve()
    require(path.is_relative_to(root), "image symlink escapes root")
    digest = row.get("sourceSha256")
    require(isinstance(digest, str) and len(digest) == 64
            and all(c in "0123456789abcdef" for c in digest), "invalid image sha256")
    image_id = row.get("imageId")
    require(type(image_id) is int and image_id > 0, "imageId must be a positive integer")
    labels = indexed(row.get("labels"), "class")
    require(set(labels) == set(CLASSES), "both class presence decisions are required")
    boxes, states = [], []
    for name in CLASSES:
        label = labels[name]
        state = label.get("presence")
        require(state in ("present", "absent"), "unknown presence cannot become background")
        if approved:
            require(label.get("humanApproved") is True and label.get("reviewStatus") == "human_approved",
                    "label approval required")
        if state == "absent":
            require(label.get("bbox_xyxy") is None and label.get("bbox_valid") is False,
                    "absent label must have no bbox")
            b = None
        else:
            if approved:
                require(label.get("bbox_valid") is True, "present label requires valid bbox")
                require(label.get("occlusion") in ("none", "partial")
                        and type(label.get("truncated")) is bool, "unresolved metadata")
            b = label.get("bbox_xyxy")
            if not approved and b is None:
                b = label.get("candidate_bbox_xyxy")
            b = box(b, width, height)
        boxes.append(b)
        states.append(state)
    split = row.get("split")
    require(split in SPLITS, "invalid split")
    return Frame(text(row.get("id"), "id"), image_id, path, digest, width, height,
                 text(row.get("sessionId"), "sessionId"), text(row.get("leakageGroup"), "leakageGroup"),
                 split, tuple(states), tuple(boxes), approved)


class CVDataset:
    """Validated manifest, lazy image reads; items are (RGB ndarray, list target).

    Empty boxes/labels remain empty lists. A future tensor adapter must explicitly
    reshape empty boxes to (0, 4); no tensor/model code runs here.
    """
    def __init__(self, manifest_path, *, split=None):
        self.manifest_path = Path(manifest_path).resolve()
        raw = self.manifest_path.read_bytes()
        self.manifest_sha256 = hashlib.sha256(raw).hexdigest()
        doc = json.loads(raw)
        require(doc.get("schema") in SCHEMAS, "unsupported dataset schema")
        require(doc.get("classIds") == {"background_reserved_not_annotation": 0, "lip_balm": 1, "gripper": 2},
                "class mapping mismatch")
        records = indexed(doc.get("records"))
        require(bool(records), "empty dataset")
        frames = tuple(_frame(row, self.manifest_path.parent, True) for row in records.values())
        require(len({f.image_id for f in frames}) == len(frames), "duplicate imageId")
        require(len({f.sha256 for f in frames}) == len(frames), "duplicate source images")
        roles = {}
        for f in frames:
            for key in (("session", f.session), ("group", f.group)):
                require(key not in roles or roles[key] == f.split, "session/leakage group crosses splits")
                roles[key] = f.split
        declared = doc.get("splits")
        require(isinstance(declared, dict) and set(declared) <= SPLITS, "split inventory required")
        assigned = []
        for role, ids in declared.items():
            require(isinstance(ids, list) and all(isinstance(i, str) for i in ids), "invalid split IDs")
            require(set(ids) == {f.id for f in frames if f.split == role}, "split inventory mismatch")
            assigned.extend(ids)
        require(len(assigned) == len(frames) and set(assigned) == set(records), "split omission/duplicate")
        require(split is None or split in SPLITS, "unknown requested split")
        self.frames = tuple(f for f in frames if split is None or f.split == split)
        require(bool(self.frames), "requested split has no frames")

    def __len__(self):
        return len(self.frames)

    def __getitem__(self, index):
        f = self.frames[index]
        return f.read_image(), f.target()


def load_candidate_frames(labels_path, capture_manifest_path, source_root):
    """Explicit preview-only adapter for archived side-view annotation records."""
    labels, captures = read_json(labels_path), read_json(capture_manifest_path)
    require(labels.get("schema") == "motionbrain-p1-reviewed-working-v1", "unsupported candidate schema")
    session = text(labels.get("sessionId"), "sessionId")
    require(session == captures.get("sessionId"), "candidate/capture session mismatch")
    lr, cr = indexed(labels.get("records")), indexed(captures.get("records"))
    require(set(lr) == set(cr) and bool(lr), "candidate/capture IDs mismatch")
    frames = []
    for image_id, (key, row) in enumerate(lr.items(), 1):
        capture = cr[key]
        require(row.get("sourceSha256") == capture.get("sha256"), "candidate/capture hash mismatch")
        require((row.get("width"), row.get("height")) == (capture.get("width"), capture.get("height")),
                "candidate/capture size mismatch")
        merged = dict(row, image=capture.get("image"), imageId=image_id, sessionId=session,
                      leakageGroup=capture.get("leakageGroup"), split=capture.get("split"))
        frames.append(_frame(merged, Path(source_root).resolve(), False))
    return tuple(frames)
