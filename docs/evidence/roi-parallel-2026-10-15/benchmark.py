"""Paired offline ROI comparison on hash-verified originals, no device requests.

Run from checkout root. Uses frame paths in adjacent inputs.json (originals are
Git-excluded). Includes decode/hash, pool creation/join and competing peaks.
Never rewrites source timestamps or evaluates live acceptance of old images.
"""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from learning.fast_observation import AnchoredRoiMeasurement, _decode, _identity, _match_candidates, _distant_peak


def evaluate(tracker, body, digest, workers):
    frame = _decode(body, digest)
    if frame.shape != tracker.shape:
        raise ValueError("changed frame dimensions")
    candidates = _match_candidates(frame, tracker.templates, workers)
    score, scale, x, y, w, h, _ = max(candidates, key=lambda c: c[0])
    _, _, aw, ah = tracker.box
    second = max(_distant_peak(scores, x+w/2, y+h/2, cw, ch, aw, ah)
                 for _, _, _, _, cw, ch, scores in candidates)
    return candidates, (score, scale, x, y, w, h, score-second)


def stats(values):
    return {"n": len(values), "min": min(values), "median": statistics.median(values), "max": max(values)}


def main(repeats):
    inputs = json.loads(Path(__file__).with_name("inputs.json").read_text())
    anchor = json.loads((ROOT/inputs["anchorObservation"]).read_text())["observation"]
    tracker = AnchoredRoiMeasurement(anchor, (ROOT/inputs["anchorImage"]).read_bytes())
    loaded = []
    seen = set()
    for item in inputs["frames"]:
        boot, captured = _identity(item["source"])
        if boot != tracker.boot or captured <= tracker.anchor_us or item["source"]["sourceFrameId"] in seen:
            raise ValueError("mixed boot, repeated or pre-anchor frame")
        seen.add(item["source"]["sourceFrameId"])
        body = (ROOT/item["path"]).read_bytes()
        _decode(body, item["source"]["imageSha256"])
        loaded.append((item, body))
    # Warm both execution modes with identical data; warmups excluded below.
    for workers in (1, 3):
        evaluate(tracker, loaded[0][1], loaded[0][0]["source"]["imageSha256"], workers)
    rows = []
    for repeat in range(repeats):
        for index, (item, body) in enumerate(loaded):
            values, elapsed = {}, {}
            for workers in ((1, 3) if (repeat+index)%2 == 0 else (3, 1)):
                start = time.perf_counter()
                values[workers] = evaluate(tracker, body, item["source"]["imageSha256"], workers)
                elapsed[workers] = (time.perf_counter()-start)*1000
            for a,b in zip(values[1][0], values[3][0]):
                if a[:6] != b[:6] or not np.array_equal(a[6], b[6]):
                    raise ValueError("serial/parallel score map or candidate mismatch")
            if values[1][1] != values[3][1]:
                raise ValueError("winner/competitor mismatch")
            score,scale,x,y,w,h,margin = values[3][1]
            rows.append({"phase": item["phase"], "sourceFrameId": item["source"]["sourceFrameId"],
                         "imageSha256": item["source"]["imageSha256"], "repeat": repeat,
                         "serialMs": elapsed[1], "parallelMs": elapsed[3], "mapsExactlyEqual": True,
                         "score": score, "peakMargin": margin, "scale": scale,
                         "diagnosticCandidateBox": [x,y,w,h]})
    return {"mode": "offline_paired_compute_benchmark", "controlEligible": False, "trainingEligible": False,
            "uniqueFrames": len(loaded), "repeats": repeats, "opencvVersion": cv2.__version__,
            "numpyVersion": np.__version__, "opencvThreads": cv2.getNumThreads(),
            "sourceSha256": {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in
                ["learning/fast_observation.py", str(Path(__file__).relative_to(ROOT)),
                 str(Path(__file__).with_name("inputs.json").relative_to(ROOT))]},
            "serial": stats([r["serialMs"] for r in rows]),
            "parallel": stats([r["parallelMs"] for r in rows]), "samples": rows,
            "limits": ["disk_io_and_anchor_initialization_excluded", "not_live_freshness_or_optical_accuracy",
                       "same_scale_search_and_competitor_checks_no_model_change"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, choices=range(1,11), default=3)
    print(json.dumps(main(parser.parse_args().repeats), indent=2))
