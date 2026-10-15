"""Conservative translation-only ROI measurement for read-only experiments.

This is template correspondence, not a new object classification, grasp detector,
or control observation. The original semantic anchor is never updated by tracking.
"""
from __future__ import annotations

import hashlib
import math
import re
import time
from concurrent.futures import ThreadPoolExecutor
from functools import partial

import cv2
import numpy as np

from learning.artifacts import MAX_IMAGE_BYTES


class MeasurementError(ValueError):
    pass


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _identity(source):
    boot, captured = source.get("cameraBootId"), source.get("capturedUs")
    if (not isinstance(boot, str) or not re.fullmatch(r"[0-9a-f]{32}", boot)
            or type(captured) is not int or not 0 < captured < 10**19
            or source.get("sourceFrameId") != f"{boot}:{captured}"
            or source.get("sourceMetadataValid") is not True
            or source.get("sourceClockDomain") != "camera_boot_us"
            or source.get("sourceTimestampSemantics") != "first_dma_buffer"):
        raise MeasurementError("invalid_source_identity")
    return boot, captured


def _decode(body, digest):
    if (not isinstance(body, bytes) or not 0 < len(body) <= MAX_IMAGE_BYTES
            or hashlib.sha256(body).hexdigest() != digest):
        raise MeasurementError("image_hash_mismatch")
    if not body.startswith(b"\xff\xd8") or not body.endswith(b"\xff\xd9"):
        raise MeasurementError("invalid_jpeg")
    frame = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if frame is None or not (16 <= frame.shape[0] <= 960 and 16 <= frame.shape[1] <= 1280):
        raise MeasurementError("unsupported_image_dimensions")
    return frame


def _distant_peak(scores, center_x, center_y, candidate_width, candidate_height, width, height):
    """Exact complement of the near-peak rectangle, without per-pixel grids."""
    rows, cols = scores.shape
    x0 = max(0, min(cols, math.ceil(center_x-candidate_width/2-width/2)))
    x1 = max(0, min(cols, math.floor(center_x-candidate_width/2+width/2)+1))
    y0 = max(0, min(rows, math.ceil(center_y-candidate_height/2-height/2)))
    y1 = max(0, min(rows, math.floor(center_y-candidate_height/2+height/2)+1))
    regions = (scores[:y0], scores[y1:], scores[y0:y1, :x0], scores[y0:y1, x1:])
    return max((float(cv2.minMaxLoc(region)[1]) for region in regions if region.size), default=-1.0)


def _match_one(frame, item):
    scale, template = item
    th, tw = template.shape
    scores = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
    _, score, _, (px, py) = cv2.minMaxLoc(scores)
    return float(score), scale, px, py, tw, th, scores


def _match_candidates(frame, templates, workers):
    """One frame, all scales, deterministic input order; no persistent workers.

    Context exit joins every submitted task, also on error or interruption.
    This does not queue frames or change OpenCV's process-wide thread setting.
    """
    if workers == 1 or len(templates) == 1:
        return [_match_one(frame, item) for item in templates]
    with ThreadPoolExecutor(max_workers=min(workers, len(templates)),
                            thread_name_prefix="roi-scale") as pool:
        return list(pool.map(partial(_match_one, frame), templates))


class AnchoredRoiMeasurement:
    """Fixed anchor, bounded lifetime, no adaptation/reacquisition/coordinate hold.

    Defaults are experimental acceptance gates, not validated physical thresholds.
    Every accepted point is computed from the named current image. Rejected
    results contain no target coordinates. No result is eligible for control.
    """
    POLICY = {"maxSourceAgeMs": 100.0, "maxAnchorAgeMs": 15000.0,
              "minScore": 0.85, "minPeakMargin": 0.08,
              "maxTranslationPx": 40.0, "minTextureStd": 8.0,
              "scaleProbes": [0.8, 1.0, 1.2]}

    def __init__(self, observation, body, *, clock=time, match_workers=3):
        if type(match_workers) is not int or not 1 <= match_workers <= 3:
            raise ValueError("match_workers must be 1..3")
        self.match_workers = match_workers
        self.boot, self.anchor_us = _identity(observation)
        if (observation.get("detected") is not True or observation.get("liveDetected") is not True
                or observation.get("held") is not False
                or observation.get("detectionSourceFrameId") != observation["sourceFrameId"]
                or not _number(observation.get("confidence")) or not 0.5 <= observation["confidence"] <= 1):
            raise MeasurementError("anchor_not_live_detection")
        frame = _decode(body, observation.get("imageSha256"))
        height, width = frame.shape
        if (observation.get("width"), observation.get("height")) != (width, height):
            raise MeasurementError("anchor_dimensions_mismatch")
        box = observation.get("targetBox", {})
        if not isinstance(box, dict) or any(type(box.get(k)) is not int for k in ("x", "y", "width", "height")):
            raise MeasurementError("invalid_anchor_box")
        x, y, w, h = (box[k] for k in ("x", "y", "width", "height"))
        if not (x >= 0 and y >= 0 and w >= 12 and h >= 12 and x + w <= width and y + h <= height):
            raise MeasurementError("invalid_anchor_box")
        self.template = frame[y:y+h, x:x+w].copy()
        if float(self.template.std()) < self.POLICY["minTextureStd"]:
            raise MeasurementError("anchor_texture_insufficient")
        self.anchor = {"sourceFrameId": observation["sourceFrameId"],
                       "imageSha256": observation["imageSha256"], "targetBox": dict(box),
                       "label": observation.get("label"), "classId": observation.get("classId")}
        self.shape, self.box, self.last_us, self.clock = frame.shape, (x, y, w, h), self.anchor_us, clock
        self.invalidated = False
        self.templates = []
        for scale in self.POLICY["scaleProbes"]:
            tw, th = round(w*scale), round(h*scale)
            if tw <= width and th <= height:
                self.templates.append((scale, cv2.resize(self.template, (tw, th), interpolation=cv2.INTER_LINEAR)))

    def measure(self, body, source, *, request_started, received, host_clock_id):
        started = self.clock.monotonic()
        result = {"schemaVersion": "motionbrain.roi_measurement.v1", "method": "fixed_template_translation",
                  "semanticAnchor": self.anchor, "sourceFrameId": source.get("sourceFrameId"),
                  "imageSha256": source.get("imageSha256"), "hostClockId": host_clock_id,
                  "accepted": False, "controlEligible": False, "trainingEligible": False,
                  "captureHostBoundsCalibrated": False}
        try:
            if self.invalidated:
                raise MeasurementError("new_anchor_required")
            boot, captured = _identity(source)
            if boot != self.boot:
                raise MeasurementError("camera_rebooted")
            if captured <= self.last_us:
                raise MeasurementError("frame_repeated_or_regressed")
            self.last_us = captured  # A rejected frame cannot later be relabelled as a new measurement.
            anchor_age = (captured - self.anchor_us) / 1000
            result["anchorAgeMs"] = anchor_age
            if anchor_age > self.POLICY["maxAnchorAgeMs"]:
                raise MeasurementError("anchor_expired")
            reported_age = source.get("reportedSourceAgeMs")
            if (not all(_number(v) for v in (request_started, received, started, reported_age))
                    or not request_started <= received <= started or reported_age < 0
                    or not isinstance(host_clock_id, str) or not host_clock_id):
                raise MeasurementError("invalid_local_timing")
            result.update(requestStartedMonotonic=request_started, receivedMonotonic=received,
                          measurementStartedMonotonic=started, reportedSourceAgeMs=reported_age,
                          httpIntervalMs=(received-request_started)*1000,
                          preProcessingElapsedMs=(started-received)*1000)
            frame = _decode(body, source.get("imageSha256"))
            if frame.shape != self.shape:
                raise MeasurementError("frame_dimensions_changed")
            x, y, w, h = self.box
            candidates = _match_candidates(frame, self.templates, self.match_workers)
            best = max(candidates, key=lambda c: c[0])
            score, scale, px, py, tw, th, _ = best
            # Compare all distant hypotheses, including differently scaled ones.
            # Nearby correlation peaks describe the same object, not another candidate.
            second = -1.0
            for _, _, _, _, cw, ch, scores in candidates:
                second = max(second, _distant_peak(scores, px+tw/2, py+th/2, cw, ch, w, h))
            result.update(score=score, peakMargin=score-second, bestScaleProbe=scale)
            if score < self.POLICY["minScore"]:
                raise MeasurementError("appearance_mismatch_or_target_lost")
            if score - second < self.POLICY["minPeakMargin"]:
                raise MeasurementError("ambiguous_match")
            if scale != 1.0:
                raise MeasurementError("scale_change_requires_new_anchor")
            if math.hypot(px-x, py-y) > self.POLICY["maxTranslationPx"]:
                raise MeasurementError("translation_out_of_range")
            completed = self.clock.monotonic()
            if not _number(completed) or completed < started:
                raise MeasurementError("invalid_local_timing")
            # Entire HTTP interval + compute age conservatively; no subtraction
            # between Pi and camera clocks and no physical calibration claim.
            age = reported_age + (completed - request_started) * 1000
            result.update(reportedAgePlusElapsedMs=age, processingMs=(completed-started)*1000)
            if age > self.POLICY["maxSourceAgeMs"]:
                raise MeasurementError("source_age_exceeds_policy")
            result.update(accepted=True, targetBox={"x": px, "y": py, "width": w, "height": h},
                          centerX=px+w/2, centerY=py+h/2, deltaX=px-x, deltaY=py-y)
        except MeasurementError as exc:
            result["reason"] = str(exc)
            # Once correspondence/identity is lost, a later lookalike must not
            # silently reacquire the old semantic identity. Timing-only misses
            # can recover on a genuinely new frame; identity misses cannot.
            if str(exc) not in ("source_age_exceeds_policy", "frame_repeated_or_regressed"):
                self.invalidated = True
        completed = self.clock.monotonic()
        if _number(completed) and completed >= started:
            result["measurementCompletedMonotonic"] = completed
            result["processingMs"] = (completed-started)*1000
            if (all(_number(v) for v in (request_started, received, source.get("reportedSourceAgeMs")))
                    and request_started <= received <= started and source["reportedSourceAgeMs"] >= 0):
                result["reportedAgePlusElapsedMs"] = source["reportedSourceAgeMs"] + (completed-request_started)*1000
                if result["accepted"] and result["reportedAgePlusElapsedMs"] > self.POLICY["maxSourceAgeMs"]:
                    result.update(accepted=False, reason="source_age_exceeds_policy")
                    for key in ("targetBox", "centerX", "centerY", "deltaX", "deltaY"):
                        result.pop(key, None)
        return result
