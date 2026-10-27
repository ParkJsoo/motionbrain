"""Operating-point detection and relative-position evaluation of saved predictions.

This is not AP, a model runner, or a physical-control success evaluator.
"""
from __future__ import annotations

import math
import statistics

from learning.cv_dataset import CLASSES, COORDINATES, CVDataset, box, indexed, number, require


def iou(a, b):
    area = lambda v: (v[2] - v[0]) * (v[3] - v[1])
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    return intersection / (area(a) + area(b) - intersection)


def center(b):
    return ((b[0] + b[2]) / 2, (b[1] + b[3]) / 2)


def relative_vector(boxes):
    if any(b is None for b in boxes):
        return None
    lip, grip = map(center, boxes)
    return [lip[0] - grip[0], lip[1] - grip[1]]


def reference_vector(v, width, height):
    require(number(width) and number(height) and width > 0 and height > 0, "invalid dimensions")
    require(len(v) == 2 and all(number(x) for x in v), "invalid relative vector")
    s = min(960 / width, 540 / height)
    return [x * s for x in v]


def direction(v, band=10):
    require(number(band) and band > 0 and len(v) == 2 and all(number(x) for x in v), "invalid direction input")
    return [-1 if x < -band else 1 if x > band else 0 for x in v]


def select_predictions(detections, frame, score_threshold, nms_threshold):
    require(isinstance(detections, list), "detections must be a list")
    grouped = {name: [] for name in CLASSES}
    for detection in detections:
        require(isinstance(detection, dict) and detection.get("class") in CLASSES, "invalid prediction class")
        score = detection.get("score")
        require(number(score) and 0 <= score <= 1, "invalid prediction score")
        b = box(detection.get("bbox_xyxy"), frame.width, frame.height)
        if score >= score_threshold:
            grouped[detection["class"]].append({"bbox_xyxy": b, "score": float(score)})
    for name, values in grouped.items():
        # Deterministic coordinate tie-break, never ground truth or input order.
        ordered = sorted(values, key=lambda p: (-p["score"], p["bbox_xyxy"]))
        kept = []
        for prediction in ordered:
            if all(iou(prediction["bbox_xyxy"], p["bbox_xyxy"]) <= nms_threshold for p in kept):
                kept.append(prediction)
        grouped[name] = kept
    return grouped


def _fraction(count, denominator):
    return {"count": count, "denominator": denominator,
            "rate": count / denominator if denominator else None}


def summarize(rows):
    pairs = [r for r in rows if r["pairApplicable"]]
    errors = [r["errorReferencePx"] for r in pairs if r["errorReferencePx"] is not None]
    result = {"frames": len(rows), "processingFailures": sum(r["processingFailure"] for r in rows),
              "pairCoverage": _fraction(sum(r["pairMatched"] for r in pairs), len(pairs)),
              "positionSuccess": _fraction(sum(r["positionSuccess"] for r in pairs), len(pairs)),
              "directionSuccess": _fraction(sum(r["directionSuccess"] for r in pairs), len(pairs)),
              "conditionalErrorReferencePx": {"count": len(errors), "expectedPairs": len(pairs),
                                              "median": statistics.median(errors) if errors else None,
                                              "max": max(errors) if errors else None}, "classes": {}}
    for name in CLASSES:
        entries = [r["classes"][name] for r in rows]
        tp, fp, fn = (sum(e[k] for e in entries) for k in ("tp", "fp", "fn"))
        absent = [e for e in entries if e["absent"]]
        result["classes"][name] = {"tp": tp, "fp": fp, "fn": fn, "precision": _fraction(tp, tp + fp),
                                    "recall": _fraction(tp, tp + fn),
                                    "absentFalsePositive": _fraction(sum(e["fp"] > 0 for e in absent), len(absent)),
                                    "absentProcessingFailures": sum(e["failed"] for e in absent),
                                    "absentClearFrames": sum(not e["failed"] and e["fp"] == 0 for e in absent)}
    return result


def evaluate(dataset, predictions, *, score_threshold, nms_threshold,
             match_iou=0.5, tolerance=10, band=10):
    require(isinstance(dataset, CVDataset) and all(f.approved for f in dataset.frames),
            "evaluation requires an approved CVDataset")
    for name, value in (("score", score_threshold), ("nms", nms_threshold)):
        require(number(value) and 0 <= value <= 1, f"invalid {name} threshold")
    require(number(match_iou) and 0 < match_iou <= 1, "invalid match IoU")
    require(number(tolerance) and tolerance > 0 and number(band) and band > 0, "invalid geometry thresholds")
    require(isinstance(predictions, dict) and predictions.get("schema") == "motionbrain.cv_predictions.v1",
            "unsupported predictions schema")
    require(predictions.get("coordinates") == COORDINATES, "prediction coordinate convention required")
    evidence = predictions.get("evidenceKind")
    require(evidence in ("synthetic", "model"), "prediction evidenceKind must be explicit")
    entries = indexed(predictions.get("frames"))
    require(set(entries) <= {f.id for f in dataset.frames}, "prediction IDs outside selected dataset")
    rows = []
    for frame in dataset.frames:
        truth_vector = relative_vector(frame.boxes)
        truth_reference = reference_vector(truth_vector, frame.width, frame.height) if truth_vector else None
        failed, reason = False, None
        grouped = {name: [] for name in CLASSES}
        try:
            frame.read_image()  # Decode failures retain their expected denominators.
            entry = entries.get(frame.id)
            require(entry is not None, "missing prediction record")
            require(entry.get("sourceSha256") == frame.sha256, "prediction source hash mismatch")
            require(entry.get("status") == "ok", "prediction processing status is not ok")
            grouped = select_predictions(entry.get("detections"), frame, score_threshold, nms_threshold)
        except (OSError, ValueError) as exc:
            failed, reason = True, str(exc)
        classes, selected, matched = {}, [], []
        for i, name in enumerate(CLASSES):
            gt, values = frame.boxes[i], grouped[name]
            top = values[0]["bbox_xyxy"] if values else None
            # Detection counts use one-to-one greedy score matching. Pair metrics
            # still use the highest-score prediction even if a lower one matches.
            tp = int(gt is not None and any(iou(p["bbox_xyxy"], gt) >= match_iou for p in values))
            classes[name] = {"tp": tp, "fp": len(values) - tp, "fn": int(gt is not None) - tp,
                             "absent": gt is None, "failed": failed}
            selected.append(top)
            matched.append(gt is not None and top is not None and iou(top, gt) >= match_iou)
        pair_matched = all(matched) and not failed
        reference_error = original_error = normalized_error = None
        axis_error = None
        direction_success = False
        if pair_matched:
            vector = relative_vector(selected)
            delta = [vector[i] - truth_vector[i] for i in (0, 1)]
            axis_error = [abs(x) for x in delta]
            original_error = math.hypot(*delta)
            reference_error = math.hypot(*reference_vector(delta, frame.width, frame.height))
            normalized_error = math.hypot(delta[0] / frame.width, delta[1] / frame.height)
            direction_success = direction(reference_vector(vector, frame.width, frame.height), band) == direction(truth_reference, band)
        near = (min(abs(abs(x) - band) for x in truth_reference) <= tolerance) if truth_reference is not None else None
        rows.append({"id": frame.id, "sessionId": frame.session, "leakageGroup": frame.group, "split": frame.split,
                     "processingFailure": failed, "failureReason": reason, "classes": classes,
                     "selectedBoxes": [list(b) if b else None for b in selected],
                     "pairApplicable": truth_vector is not None, "pairMatched": pair_matched,
                     "nearBoundary": near, "errorReferencePx": reference_error, "errorOriginalPx": original_error,
                     "axisErrorOriginalPx": axis_error, "errorNormalized": normalized_error,
                     "positionSuccess": pair_matched and reference_error <= tolerance,
                     "directionSuccess": direction_success})
    return {"schema": "motionbrain.cv_evaluation.v1", "evidenceKind": evidence,
            "modelAccuracyMeasured": evidence == "model", "predictionEvidenceIsUserDeclared": True,
            "datasetManifestSha256": dataset.manifest_sha256,
            "operatingPoint": {"scoreThreshold": score_threshold, "nmsThreshold": nms_threshold,
                               "matchIoU": match_iou, "relativeToleranceReferencePx": tolerance,
                               "directionBandReferencePx": band, "referenceViewport": [960, 540]},
            "metrics": summarize(rows), "sessions": {s: summarize([r for r in rows if r["sessionId"] == s])
                                                       for s in sorted({f.session for f in dataset.frames})},
            "boundaryStrata": {name: summarize([r for r in rows if r["nearBoundary"] is value])
                               for name, value in (("near", True), ("stable", False))},
            "frames": rows, "limitations": ["operating-point metrics only; AP not implemented",
                                              "metrics against approved image annotations, not physical success",
                                              "absent processing failures are not clear negative frames",
                                              "split role and evidence kind must accompany any reported metric"]}
