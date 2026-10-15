"""Offline image-only diagnosis, never a replay of live acceptance.

Run from repository root: python3 docs/evidence/manual-scene-2026-10-15/analyze.py
Requires the Git-excluded original runs named in session.json. Human-paced
phases exceed the live anchor lifetime. No timestamps/policies are rewritten.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from learning.artifacts import read_events, read_json_artifact, verify_artifacts
from learning.device_probe import audit_device_probe
from learning.fast_observation import AnchoredRoiMeasurement, _decode, _distant_peak, _identity


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


def main():
    session = json.loads(Path(__file__).with_name("session.json").read_text())
    baseline = ROOT / session["baselineRun"]
    report = json.loads((baseline / "run.json").read_text())
    events, errors = read_events(baseline)
    if (errors or events[-1]["kind"] != "finished" or
            read_json_artifact(baseline, events[-1]["payload"]["report"]) != report or
            verify_artifacts(baseline, report["artifacts"]) or report["status"] != "completed"
            or not report["outputLogUnchanged"]):
        raise ValueError("baseline audit failed")
    observation = json.loads((baseline / "anchor.json").read_text())["observation"]
    tracker = AnchoredRoiMeasurement(observation, (baseline / "images/anchor/source.jpg").read_bytes())
    phases = []
    seen = set()
    for phase in session["phases"]:
        root = ROOT / phase["run"]
        if not audit_device_probe(root)["ok"]:
            raise ValueError("phase audit failed")
        run = json.loads((root / "run.json").read_text())
        if run["status"] != "completed" or not run["outputLogUnchanged"] or run["completedSamples"] != 6:
            raise ValueError("incomplete phase or changed outputs")
        rows = []
        for path in sorted((root / "clock").glob("*.json")):
            sample = json.loads(path.read_text())
            if sample["sourceFrameId"] in seen:
                raise ValueError("duplicate source frame")
            seen.add(sample["sourceFrameId"])
            source = {**sample, "imageSha256": sample["image"]["sha256"]}
            rows.append(diagnose(tracker, (root / sample["image"]["artifactRef"]).read_bytes(), source))
        if len(rows) != 6:
            raise ValueError("camera samples missing")
        phases.append({"phase": phase["label"], "run": phase["run"], "samples": rows,
                       "outputLogUnchanged": True, "artifactCount": len(run["artifacts"])})
    return {"mode": "offline_image_diagnostic", "baselineSourceFrameId": observation["sourceFrameId"],
            "baselineImageSha256": observation["imageSha256"], "policy": tracker.POLICY,
            "controlEligible": False, "trainingEligible": False, "phases": phases,
            "sourceSha256": {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in
                ["learning/fast_observation.py", str(Path(__file__).relative_to(ROOT))]},
            "limitations": ["not_live_tracking_or_freshness_validation", "no_pixel_ground_truth",
                            "independent_per_image_diagnosis_not_latch_recovery_test"]}


if __name__ == "__main__":
    print(json.dumps(main(), indent=2))
