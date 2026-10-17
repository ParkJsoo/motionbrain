"""Offline image-gate diagnostics; never grants live/control/training eligibility."""
import math

import cv2

from learning.fast_observation import _decode, _distant_peak, _identity


def diagnose(tracker, body, source):
    boot, captured = _identity(source)
    if boot != tracker.boot or captured <= tracker.anchor_us:
        raise ValueError("camera boot/chronology mismatch")
    frame = _decode(body, source["imageSha256"])
    if frame.shape != tracker.shape:
        raise ValueError("frame dimensions changed")
    candidates = []
    for scale, template in tracker.templates:
        scores = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
        _, score, _, (x, y) = cv2.minMaxLoc(scores)
        h, w = template.shape
        candidates.append((float(score), scale, x, y, w, h, scores))
    score, scale, x, y, w, h, _ = max(candidates, key=lambda c: c[0])
    ax, ay, aw, ah = tracker.box
    second = max(_distant_peak(scores, x+w/2, y+h/2, cw, ch, aw, ah)
                 for _, _, _, _, cw, ch, scores in candidates)
    distance = math.hypot(x-ax, y-ay)
    p = tracker.POLICY
    reason = ("appearance_mismatch_or_target_lost" if score < p["minScore"] else
              "ambiguous_match" if score-second < p["minPeakMargin"] else
              "scale_change_requires_new_anchor" if scale != 1.0 else
              "translation_out_of_range" if distance > p["maxTranslationPx"] else
              "image_gates_pass_only")
    return {"sourceFrameId": source["sourceFrameId"], "imageSha256": source["imageSha256"],
            "score": score, "peakMargin": score-second, "bestScaleProbe": scale,
            "diagnosticCandidateBox": {"x": x, "y": y, "width": w, "height": h},
            "diagnosticDeltaPx": [x-ax, y-ay], "translationDistancePx": distance,
            "imageGateReason": reason, "anchorAgeMs": (captured-tracker.anchor_us)/1000,
            "anchorExpired": (captured-tracker.anchor_us)/1000 > p["maxAnchorAgeMs"],
            "liveAcceptanceEvaluated": False, "controlEligible": False, "trainingEligible": False}

