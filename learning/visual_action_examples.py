"""Tiny synthetic BGRA/action examples; values are not a physical experiment plan."""
import copy
import hashlib
from pathlib import Path

from learning.artifacts import ArtifactStore
from learning.visual_action_collection import SESSION_SCHEMA, VisualActionCollector

SCENARIOS = ("complete", "no_motion", "missing_after", "unknown_result", "controller_failed",
             "missing_frame", "corrupt_frame", "interrupted")


def synthetic_bundle(scenario="complete"):
    if scenario not in SCENARIOS:
        raise ValueError("unsupported_synthetic_scenario")
    session = {"schemaVersion": SESSION_SCHEMA, "sessionId": "synthetic-session", "viewId": "fixed-view",
               "streamId": "synthetic-stream", "hostClockId": "synthetic-host", "sourceClockId": "synthetic-pts",
               "mode": "synthetic", "cameraViewFixed": True, "width": 2, "height": 2, "rowBytes": 8,
               "videoMirrored": False, "videoRotationAngle": 0, "maxAttempts": 1, "maxFrames": 6,
               "maxDurationSeconds": 10}
    request = {"bootId": "a" * 32, "commandId": "action-1", "kind": "pulse", "motor": 1,
               "pwm": 30, "durationMs": 100, "targetDegrees": 0, "percent": 0}
    def event(kind, at, **fields):
        return {"type": kind, "at": at, "hostClockId": session["hostClockId"], "actionId": "action-1", **fields}
    items = []
    def add(e, pixels=None):
        items.append((e, pixels))
    def frame(name, at, tick, value, x):
        pixels = bytes([value, value, value, 255]) * 4
        e = event("frame", at, frameId=name, viewId=session["viewId"], streamId=session["streamId"],
                  sourceClockId=session["sourceClockId"], pts={"value": tick, "timescale": 1000, "epoch": 0},
                  timestampSemantics="presentation_time", pixelFormat="BGRA8",
                  sha256=hashlib.sha256(pixels).hexdigest(),
                  features=[{"id": "gripper-tip", "x": x, "y": 1, "visible": True, "confidence": 1}])
        add(e, pixels)
    add(event("begin", 100, episodeId="episode-1", startPoseId="pose-1", request=request))
    frame("frame-1", 100.1, 10, 0, 0)
    add(event("dispatch", 100.2))
    frame("frame-2", 100.25, 20, 0 if scenario == "no_motion" else 20, 0)
    if scenario == "interrupted":
        return session, items
    if scenario == "unknown_result":
        add(event("result", 100.4, executionState="outcome_unknown", record=None, baselineOutputs=None, outputLog=None))
        frame("frame-3", 100.5, 30, 20, 1)
        add(event("end", 100.6, motionObservation="unknown", settlingObservation="unverified"))
        return session, items
    record = {k: v for k, v in request.items() if k != "pwm"}
    record.update(requestedPwm=30, ordinal=1, startedAtMs=1000, endedAtMs=1100,
                  elapsedMs=100, overrunMs=0, state="completed", reason="duration_elapsed",
                  outputOff=True, targetReached=False)
    if scenario == "controller_failed":
        record.update(state="failed", reason="safety_blocked")
    baseline = {"bootId": request["bootId"], "clockDomain": "controller_boot_millis", "evidence": "driver_write",
                "dropped": 0, "entries": []}
    outputs = {**baseline, "entries": [
        {"sequence": 1, "atMs": 1000, "actionOrdinal": 1, "motor": 1, "pwm": 30, "reason": "driver_write"},
        {"sequence": 2, "atMs": 1100, "actionOrdinal": 1, "motor": 1, "pwm": 0, "reason": "forced_off"}]}
    add(event("result", 100.4, executionState="known_terminal", record=record,
              baselineOutputs=baseline, outputLog=outputs))
    if scenario != "missing_after":
        frame("frame-3", 100.5, 30, 0 if scenario == "no_motion" else 40, 0 if scenario == "no_motion" else 1)
    add(event("end", 100.6, motionObservation="no_visible_motion" if scenario == "no_motion" else "observed",
                  settlingObservation="unverified"))
    if scenario in ("missing_frame", "corrupt_frame"):
        e, pixels = items[1]
        items[1] = (e, None if scenario == "missing_frame" else bytes([12]) * len(pixels))
    return copy.deepcopy(session), items


def run_synthetic(output_dir, scenario="complete"):
    session, items = synthetic_bundle(scenario)
    collector = VisualActionCollector(output_dir, session)
    for event, pixels in items:
        if not collector.record(event, pixels):
            break
    return collector.close()


def write_example(output_dir):
    session, items = synthetic_bundle()
    store = ArtifactStore(Path(output_dir))
    events = []
    for i, (event, pixels) in enumerate(items):
        image_path = None
        if pixels is not None:
            image_path = f"frames/{i:04d}.bgra"
            store._write(image_path, pixels)
        events.append({"event": event, "imagePath": image_path})
    store.write_json("bundle.json", {"session": session, "events": events})
    return store.root / "bundle.json"
