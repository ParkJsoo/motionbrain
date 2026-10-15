"""Offline Pi benchmark of existing detector settings. No device/network IO.

Example: python3 benchmark.py --repo REPO --model MODEL --labels LABELS
  --frames IMAGE1 IMAGE2 --output report.json
Small stored-image sample: timing/compatibility evidence, not accuracy or control.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import statistics
import sys
import time


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--labels", type=Path, required=True)
    parser.add_argument("--frames", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    bridge = args.repo / "ros2_ws/src/motionbrain_ros_bridge"
    sys.path.insert(0, str(bridge))
    import cv2
    import numpy as np
    from motionbrain_ros_bridge.vision_detection import DetectionConfig, OpenCvDnnObjectDetector, detect_frame

    if args.output.exists():
        parser.error("output already exists; choose a new evidence file")
    frames = [(path, path.read_bytes()) for path in args.frames]
    report = {"schemaVersion": "motionbrain.detector_settings_benchmark.v1",
              "platform": platform.platform(), "opencvVersion": cv2.__version__, "python": sys.executable,
              "benchmarkSha256": digest(__file__),
              "modelSha256": digest(args.model), "labelsSha256": digest(args.labels),
              "detectorSourceSha256": digest(bridge / "motionbrain_ros_bridge/vision_detection.py"),
              "motorCommandsSent": 0, "networkRequests": 0, "trainingEligible": False,
              "scope": "offline stored-image timing; concurrent production perception remains active",
              "frames": [{"path": str(p), "sha256": hashlib.sha256(b).hexdigest()} for p, b in frames],
              "decodeSamplesMs": [], "configurations": []}
    cv2.setNumThreads(1)
    for path, data in frames:
        for _ in range(5):
            start = time.monotonic()
            image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("invalid input JPEG")
            report["decodeSamplesMs"].append((time.monotonic() - start) * 1000)
    for size, threads in ((640, 1), (320, 1), (640, 2), (320, 2)):
        result = {"inputSize": size, "requestedThreads": threads, "samples": []}
        report["configurations"].append(result)
        cv2.setNumThreads(threads)
        result["actualThreads"] = cv2.getNumThreads()
        config = DetectionConfig(mode="object", object_target="cup", object_backend="opencv-dnn",
                                 object_input_size=size, object_model=str(args.model),
                                 object_labels=str(args.labels), object_min_confidence=.20,
                                 object_target_aliases=("chair", "toilet"))
        result["config"] = asdict(config)
        try:
            detector = OpenCvDnnObjectDetector.from_model(str(args.model), str(args.labels), input_size=size)
            warm = time.monotonic()
            detect_frame(frames[0][1], config, detector)
            result["warmupMs"] = (time.monotonic() - warm) * 1000
            for repeat in range(2):
                for path, data in frames:
                    begin = time.monotonic()
                    detection = detect_frame(data, config, detector)
                    result["samples"].append({"frameSha256": hashlib.sha256(data).hexdigest(),
                        "repeat": repeat, "elapsedMs": (time.monotonic() - begin) * 1000,
                        "detected": detection["detected"], "label": detection.get("label"),
                        "classId": detection.get("classId"),
                        "confidence": detection.get("confidence"), "targetBox": detection.get("targetBox")})
            times = [s["elapsedMs"] for s in result["samples"]]
            result.update(status="completed", medianMs=statistics.median(times), maxMs=max(times),
                          cupDetections=sum(s["detected"] and s["label"] == "cup" for s in result["samples"]))
        except Exception as exc:
            result.update(status="failed", errorType=type(exc).__name__, error=str(exc))
        print(json.dumps({k: v for k, v in result.items() if k != "samples"}), flush=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")


if __name__ == "__main__":
    main()
