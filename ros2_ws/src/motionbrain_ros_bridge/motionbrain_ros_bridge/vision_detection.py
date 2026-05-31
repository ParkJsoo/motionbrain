from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from typing import Iterable
from typing import Protocol
from typing import Sequence

from motionbrain_ros_bridge.payload_utils import ALIGN_DEADBAND
from motionbrain_ros_bridge.payload_utils import classify_alignment
from motionbrain_ros_bridge.payload_utils import command_suggestion_for_alignment


TARGET_RATIO_THRESHOLD = 0.02
MAX_OBJECT_PAYLOADS = 40


@dataclass(frozen=True)
class DetectionConfig:
    mode: str = "color"
    color: str = "red"
    align_deadband: float = ALIGN_DEADBAND
    target_ratio_threshold: float = TARGET_RATIO_THRESHOLD
    object_target: str = ""
    object_min_confidence: float = 0.45
    object_nms_threshold: float = 0.45
    object_input_size: int = 320
    object_backend: str = "fake"
    object_model: str = ""
    object_labels: str = ""
    target_policy: str = "largest"


@dataclass(frozen=True)
class DetectionCandidate:
    target_type: str
    label: str
    x: float
    y: float
    width: float
    height: float
    frame_width: int
    frame_height: int
    confidence: float | None = None
    class_id: int | None = None
    color: str | None = None


class DetectorBackend(Protocol):
    name: str

    def detect(self, frame: bytes, config: DetectionConfig) -> Iterable[DetectionCandidate]:
        ...


def _candidate_area(candidate: DetectionCandidate) -> float:
    return max(candidate.width, 0.0) * max(candidate.height, 0.0)


def _candidate_area_ratio(candidate: DetectionCandidate) -> float:
    frame_area = max(candidate.frame_width * candidate.frame_height, 1)
    return _candidate_area(candidate) / frame_area


def _candidate_center(candidate: DetectionCandidate) -> tuple[float, float]:
    return candidate.x + candidate.width / 2.0, candidate.y + candidate.height / 2.0


def _candidate_offsets(candidate: DetectionCandidate) -> tuple[float, float]:
    center_x, center_y = _candidate_center(candidate)
    frame_center_x = (candidate.frame_width - 1) / 2.0
    frame_center_y = (candidate.frame_height - 1) / 2.0
    offset_x = (center_x - frame_center_x) / max(frame_center_x, 1.0)
    offset_y = (center_y - frame_center_y) / max(frame_center_y, 1.0)
    return offset_x, offset_y


def _box_payload(candidate: DetectionCandidate) -> dict[str, int]:
    return {
        "x": max(int(round(candidate.x)), 0),
        "y": max(int(round(candidate.y)), 0),
        "width": max(int(round(candidate.width)), 0),
        "height": max(int(round(candidate.height)), 0),
    }


def candidate_payload(candidate: DetectionCandidate, config: DetectionConfig) -> dict[str, Any]:
    center_x, center_y = _candidate_center(candidate)
    offset_x, offset_y = _candidate_offsets(candidate)
    area_ratio = _candidate_area_ratio(candidate)
    alignment = classify_alignment(offset_x, config.align_deadband)
    payload: dict[str, Any] = {
        "targetType": candidate.target_type,
        "label": candidate.label,
        "classId": candidate.class_id,
        "confidence": candidate.confidence,
        "targetBox": _box_payload(candidate),
        "centerX": center_x,
        "centerY": center_y,
        "centroidX": center_x,
        "centroidY": center_y,
        "offsetX": offset_x,
        "offsetY": offset_y,
        "areaRatio": area_ratio,
        "ratio": area_ratio,
        "pixels": int(round(_candidate_area(candidate))),
        "width": candidate.frame_width,
        "height": candidate.frame_height,
        "alignment": alignment,
        "commandSuggestion": command_suggestion_for_alignment(alignment),
    }
    if candidate.color:
        payload["color"] = candidate.color
    return payload


def select_target(
    candidates: Iterable[DetectionCandidate],
    config: DetectionConfig,
) -> DetectionCandidate | None:
    filtered: list[DetectionCandidate] = []
    wanted = config.object_target.strip().lower()
    for candidate in candidates:
        if candidate.confidence is not None and candidate.confidence < config.object_min_confidence:
            continue
        if wanted and candidate.label.strip().lower() != wanted:
            continue
        filtered.append(candidate)

    if not filtered:
        return None

    policy = config.target_policy.strip().lower()
    if policy == "center":
        return min(filtered, key=lambda candidate: sum(abs(value) for value in _candidate_offsets(candidate)))
    if policy == "highest-confidence":
        return max(filtered, key=lambda candidate: candidate.confidence if candidate.confidence is not None else 0.0)
    return max(filtered, key=_candidate_area)


def load_labels(labels_path: str) -> list[str]:
    labels: list[str] = []
    if not labels_path:
        return labels

    for raw_line in Path(labels_path).read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(maxsplit=1)
        if len(parts) == 2 and parts[0].isdigit():
            label_index = int(parts[0])
            while len(labels) <= label_index:
                labels.append(f"class_{len(labels)}")
            labels[label_index] = parts[1].strip()
        else:
            labels.append(line)
    return labels


def _label_for_class_id(labels: Sequence[str], class_id: int) -> str:
    if 0 <= class_id < len(labels) and labels[class_id]:
        return labels[class_id]
    return f"class_{class_id}"


def _scaled_box(
    values: Sequence[float],
    frame_width: int,
    frame_height: int,
    *,
    input_width: int | None = None,
    input_height: int | None = None,
) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = [float(value) for value in values[:4]]
    if max(abs(x1), abs(y1), abs(x2), abs(y2)) <= 1.5:
        x1 *= frame_width
        x2 *= frame_width
        y1 *= frame_height
        y2 *= frame_height
    elif input_width and input_height:
        x_scale = frame_width / max(float(input_width), 1.0)
        y_scale = frame_height / max(float(input_height), 1.0)
        x1 *= x_scale
        x2 *= x_scale
        y1 *= y_scale
        y2 *= y_scale

    left = max(min(x1, x2), 0.0)
    top = max(min(y1, y2), 0.0)
    right = min(max(x1, x2), float(frame_width))
    bottom = min(max(y1, y2), float(frame_height))
    return left, top, max(right - left, 0.0), max(bottom - top, 0.0)


def _scaled_center_box(
    values: Sequence[float],
    frame_width: int,
    frame_height: int,
    *,
    input_width: int | None = None,
    input_height: int | None = None,
) -> tuple[float, float, float, float]:
    center_x, center_y, width, height = [float(value) for value in values[:4]]
    return _scaled_box(
        (
            center_x - width / 2.0,
            center_y - height / 2.0,
            center_x + width / 2.0,
            center_y + height / 2.0,
        ),
        frame_width,
        frame_height,
        input_width=input_width,
        input_height=input_height,
    )


def _candidate_iou(left: DetectionCandidate, right: DetectionCandidate) -> float:
    left_x2 = left.x + left.width
    left_y2 = left.y + left.height
    right_x2 = right.x + right.width
    right_y2 = right.y + right.height

    overlap_left = max(left.x, right.x)
    overlap_top = max(left.y, right.y)
    overlap_right = min(left_x2, right_x2)
    overlap_bottom = min(left_y2, right_y2)
    overlap_width = max(overlap_right - overlap_left, 0.0)
    overlap_height = max(overlap_bottom - overlap_top, 0.0)
    overlap_area = overlap_width * overlap_height
    if overlap_area <= 0.0:
        return 0.0
    union_area = _candidate_area(left) + _candidate_area(right) - overlap_area
    return overlap_area / max(union_area, 1.0)


def apply_nms(candidates: Iterable[DetectionCandidate], threshold: float) -> list[DetectionCandidate]:
    if threshold <= 0.0:
        return list(candidates)

    kept: list[DetectionCandidate] = []
    sorted_candidates = sorted(
        candidates,
        key=lambda candidate: candidate.confidence if candidate.confidence is not None else 0.0,
        reverse=True,
    )
    for candidate in sorted_candidates:
        if all(_candidate_iou(candidate, existing) <= threshold for existing in kept):
            kept.append(candidate)
    return kept


def decode_dnn_detections(
    outputs: Any,
    *,
    frame_width: int,
    frame_height: int,
    labels: Sequence[str],
    min_confidence: float,
    nms_threshold: float,
    input_width: int | None = None,
    input_height: int | None = None,
) -> list[DetectionCandidate]:
    try:
        import numpy as np  # type: ignore
    except ImportError:
        return []

    candidates: list[DetectionCandidate] = []
    output_items = outputs if isinstance(outputs, (list, tuple)) else [outputs]
    for output in output_items:
        array = np.asarray(output)
        if array.size == 0:
            continue

        if array.ndim == 4 and array.shape[-1] >= 7:
            rows = array.reshape(-1, array.shape[-1])
            for row in rows:
                if float(row[0]) < 0:
                    continue
                confidence = float(row[2])
                if confidence < min_confidence:
                    continue
                class_id = int(row[1])
                x, y, width, height = _scaled_box(
                    row[3:7],
                    frame_width,
                    frame_height,
                    input_width=input_width,
                    input_height=input_height,
                )
                if width <= 0.0 or height <= 0.0:
                    continue
                candidates.append(
                    DetectionCandidate(
                        target_type="object",
                        label=_label_for_class_id(labels, class_id),
                        class_id=class_id,
                        confidence=confidence,
                        x=x,
                        y=y,
                        width=width,
                        height=height,
                        frame_width=frame_width,
                        frame_height=frame_height,
                    )
                )
            continue

        if array.ndim == 3 and array.shape[0] == 1:
            matrix = array[0]
            if labels and matrix.shape[0] == 4 + len(labels):
                rows = matrix.transpose()
            elif matrix.shape[0] >= 5 and matrix.shape[0] < matrix.shape[1]:
                rows = matrix.transpose()
            else:
                rows = matrix
            if rows.shape[-1] > 6:
                for row in rows:
                    if row.shape[0] < 5:
                        continue
                    if labels and row.shape[0] == 5 + len(labels):
                        objectness = float(row[4])
                        class_scores = row[5:]
                    else:
                        objectness = 1.0
                        class_scores = row[4:]
                    if class_scores.size == 0:
                        continue
                    class_id = int(np.argmax(class_scores))
                    confidence = objectness * float(class_scores[class_id])
                    if confidence < min_confidence:
                        continue
                    x, y, width, height = _scaled_center_box(
                        row[0:4],
                        frame_width,
                        frame_height,
                        input_width=input_width,
                        input_height=input_height,
                    )
                    if width <= 0.0 or height <= 0.0:
                        continue
                    candidates.append(
                        DetectionCandidate(
                            target_type="object",
                            label=_label_for_class_id(labels, class_id),
                            class_id=class_id,
                            confidence=confidence,
                            x=x,
                            y=y,
                            width=width,
                            height=height,
                            frame_width=frame_width,
                            frame_height=frame_height,
                        )
                    )
                continue

        if array.ndim >= 2 and array.shape[-1] >= 6:
            rows = array.reshape(-1, array.shape[-1])
            for row in rows:
                confidence = float(row[4])
                if confidence < min_confidence:
                    continue
                class_id = int(row[5])
                x, y, width, height = _scaled_box(
                    row[0:4],
                    frame_width,
                    frame_height,
                    input_width=input_width,
                    input_height=input_height,
                )
                if width <= 0.0 or height <= 0.0:
                    continue
                candidates.append(
                    DetectionCandidate(
                        target_type="object",
                        label=_label_for_class_id(labels, class_id),
                        class_id=class_id,
                        confidence=confidence,
                        x=x,
                        y=y,
                        width=width,
                        height=height,
                        frame_width=frame_width,
                        frame_height=frame_height,
                    )
                )

    return apply_nms(candidates, nms_threshold)


@dataclass
class OpenCvDnnObjectDetector:
    net: Any
    labels: Sequence[str]
    input_size: int = 320
    name: str = "opencv-dnn"

    @classmethod
    def from_model(
        cls,
        model_path: str,
        labels_path: str = "",
        *,
        input_size: int = 320,
    ) -> "OpenCvDnnObjectDetector":
        if not model_path:
            raise ValueError("--object-model is required for opencv-dnn object detection")
        model = Path(model_path)
        if not model.exists():
            raise FileNotFoundError(f"object model not found: {model}")

        try:
            import cv2  # type: ignore
        except ImportError as exc:
            raise RuntimeError("opencv_unavailable") from exc

        labels = load_labels(labels_path) if labels_path else []
        if model.suffix.lower() == ".onnx":
            net = cv2.dnn.readNetFromONNX(str(model))
        else:
            net = cv2.dnn.readNet(str(model))
        return cls(net=net, labels=labels, input_size=input_size)

    def detect(self, frame: bytes, config: DetectionConfig) -> Iterable[DetectionCandidate]:
        try:
            import cv2  # type: ignore
            import numpy as np  # type: ignore
        except ImportError as exc:
            raise RuntimeError("opencv_unavailable") from exc

        data = np.frombuffer(frame, dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            return []

        frame_height, frame_width = image.shape[:2]
        input_size = max(int(config.object_input_size or self.input_size), 32)
        blob = cv2.dnn.blobFromImage(
            image,
            scalefactor=1.0 / 255.0,
            size=(input_size, input_size),
            mean=(0.0, 0.0, 0.0),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        outputs = self.net.forward()
        return decode_dnn_detections(
            outputs,
            frame_width=frame_width,
            frame_height=frame_height,
            labels=self.labels,
            min_confidence=config.object_min_confidence,
            nms_threshold=config.object_nms_threshold,
            input_width=input_size,
            input_height=input_size,
        )


def payload_from_candidate(
    candidate: DetectionCandidate,
    config: DetectionConfig,
    frame_bytes: int,
    *,
    all_candidates: Iterable[DetectionCandidate] = (),
    detector_name: str = "",
    latency_ms: float | None = None,
) -> dict[str, Any]:
    payload = candidate_payload(candidate, config)
    object_payloads = [candidate_payload(item, config) for item in list(all_candidates)[:MAX_OBJECT_PAYLOADS]]
    payload.update(
        {
            "available": True,
            "detected": True,
            "frameBytes": frame_bytes,
            "alignDeadband": config.align_deadband,
            "detector": {
                "mode": config.mode,
                "backend": config.object_backend,
                "model": config.object_model,
                "name": detector_name,
                "latencyMs": latency_ms,
                "targetPolicy": config.target_policy,
            },
            "objects": object_payloads,
            "target": candidate_payload(candidate, config),
            "stableFrames": 0,
        }
    )
    return payload


def _empty_payload(
    config: DetectionConfig,
    frame_bytes: int,
    reason: str,
    *,
    available: bool,
    detector_name: str = "",
    latency_ms: float | None = None,
    candidates: Iterable[DetectionCandidate] = (),
) -> dict[str, Any]:
    objects = [candidate_payload(item, config) for item in list(candidates)[:MAX_OBJECT_PAYLOADS]]
    payload: dict[str, Any] = {
        "available": available,
        "detected": False,
        "targetType": config.mode,
        "color": config.color if config.mode == "color" else None,
        "label": config.object_target or config.color,
        "classId": None,
        "confidence": None,
        "reason": reason,
        "frameBytes": frame_bytes,
        "centerX": None,
        "centerY": None,
        "centroidX": None,
        "centroidY": None,
        "targetBox": None,
        "offsetX": None,
        "offsetY": None,
        "alignDeadband": config.align_deadband,
        "alignment": "LOST",
        "commandSuggestion": "none",
        "detector": {
            "mode": config.mode,
            "backend": config.object_backend,
            "model": config.object_model,
            "name": detector_name,
            "latencyMs": latency_ms,
            "targetPolicy": config.target_policy,
        },
        "objects": objects,
        "target": None,
        "stableFrames": 0,
    }
    if objects:
        payload["width"] = objects[0].get("width")
        payload["height"] = objects[0].get("height")
    return payload


def detect_colored_target(
    frame: bytes,
    color: str = "red",
    align_deadband: float = ALIGN_DEADBAND,
    target_ratio_threshold: float = TARGET_RATIO_THRESHOLD,
) -> dict[str, Any]:
    config = DetectionConfig(
        mode="color",
        color=color,
        align_deadband=align_deadband,
        target_ratio_threshold=target_ratio_threshold,
        object_backend="color",
    )
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError:
        return _empty_payload(config, len(frame), "opencv_unavailable", available=False, detector_name="color")

    data = np.frombuffer(frame, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        return _empty_payload(config, len(frame), "decode_failed", available=True, detector_name="color")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if color == "red":
        mask1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 80, 80), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == "green":
        mask = cv2.inRange(hsv, (40, 70, 70), (85, 255, 255))
    elif color == "blue":
        mask = cv2.inRange(hsv, (95, 70, 70), (130, 255, 255))
    else:
        payload = _empty_payload(config, len(frame), "unsupported_color", available=True, detector_name="color")
        height, width = image.shape[:2]
        payload["width"] = width
        payload["height"] = height
        return payload

    pixels = int(cv2.countNonZero(mask))
    height, width = image.shape[:2]
    area = max(height * width, 1)
    ratio = pixels / area
    detected = ratio >= target_ratio_threshold

    if detected and pixels > 0:
        x, y, box_width, box_height = cv2.boundingRect(mask)
        moments = cv2.moments(mask)
        if moments["m00"] != 0:
            centroid_x = float(moments["m10"] / moments["m00"])
            centroid_y = float(moments["m01"] / moments["m00"])
        else:
            centroid_x = float(x + box_width / 2.0)
            centroid_y = float(y + box_height / 2.0)
        center_x = (width - 1) / 2.0
        center_y = (height - 1) / 2.0
        offset_x = (centroid_x - center_x) / max(center_x, 1.0)
        offset_y = (centroid_y - center_y) / max(center_y, 1.0)
        alignment = classify_alignment(offset_x, align_deadband)
        command_suggestion = command_suggestion_for_alignment(alignment)
        candidate = DetectionCandidate(
            target_type="color",
            label=color,
            color=color,
            x=float(x),
            y=float(y),
            width=float(box_width),
            height=float(box_height),
            frame_width=width,
            frame_height=height,
            confidence=None,
            class_id=None,
        )
        payload = payload_from_candidate(
            candidate,
            config,
            len(frame),
            all_candidates=[candidate],
            detector_name="color",
            latency_ms=None,
        )
        payload.update(
            {
                "color": color,
                "ratio": ratio,
                "areaRatio": ratio,
                "pixels": pixels,
                "centerX": centroid_x,
                "centerY": centroid_y,
                "centroidX": centroid_x,
                "centroidY": centroid_y,
                "offsetX": offset_x,
                "offsetY": offset_y,
                "alignment": alignment,
                "commandSuggestion": command_suggestion,
            }
        )
        payload["target"].update(
            {
                "color": color,
                "ratio": ratio,
                "areaRatio": ratio,
                "pixels": pixels,
                "centerX": centroid_x,
                "centerY": centroid_y,
                "centroidX": centroid_x,
                "centroidY": centroid_y,
                "offsetX": offset_x,
                "offsetY": offset_y,
                "alignment": alignment,
                "commandSuggestion": command_suggestion,
            }
        )
        payload["objects"][0].update(payload["target"])
        return payload

    payload = _empty_payload(config, len(frame), "", available=True, detector_name="color")
    payload.pop("reason", None)
    payload.update(
        {
            "color": color,
            "ratio": ratio,
            "areaRatio": ratio,
            "pixels": pixels,
            "width": width,
            "height": height,
        }
    )
    return payload


def detect_frame(
    frame: bytes,
    config: DetectionConfig | None = None,
    detector: DetectorBackend | None = None,
) -> dict[str, Any]:
    config = config or DetectionConfig()
    mode = config.mode.strip().lower()
    if mode == "color":
        return detect_colored_target(
            frame,
            config.color,
            config.align_deadband,
            config.target_ratio_threshold,
        )
    if mode != "object":
        return _empty_payload(config, len(frame), "unsupported_detector_mode", available=False)
    if detector is None:
        return _empty_payload(config, len(frame), "object_detector_unconfigured", available=False)

    started = time.monotonic()
    candidates = list(detector.detect(frame, config))
    latency_ms = (time.monotonic() - started) * 1000.0
    selected = select_target(candidates, config)
    detector_name = getattr(detector, "name", detector.__class__.__name__)
    if selected is None:
        return _empty_payload(
            config,
            len(frame),
            "target_not_found" if candidates else "no_objects",
            available=True,
            detector_name=detector_name,
            latency_ms=latency_ms,
            candidates=candidates,
        )
    return payload_from_candidate(
        selected,
        config,
        len(frame),
        all_candidates=candidates,
        detector_name=detector_name,
        latency_ms=latency_ms,
    )
