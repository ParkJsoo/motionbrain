"""Static annotation overlays; deliberately not a model or live-video runner."""
from __future__ import annotations

import cv2
import numpy as np

from learning.cv_dataset import CLASSES, Frame, require
from learning.cv_evaluation import direction, reference_vector, relative_vector


def render_frame(frame: Frame):
    canvas = np.full((660, 960, 3), (36, 26, 17), dtype=np.uint8)
    heading = "APPROVED LABEL PREVIEW" if frame.approved else "CANDIDATE REFERENCE - GEOMETRY UNRESOLVED"
    def text(message, x, y, color=(235, 240, 245), size=0.55):
        cv2.putText(canvas, message, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)
    text(heading, 16, 26, (105, 215, 255), 0.65)
    text("NOT MODEL OUTPUT | image axes; not reachability or grasp success", 16, 50)
    # IDs stay in the JSON report; arbitrary Unicode/long IDs never obscure the heading.
    try:
        image = cv2.cvtColor(frame.read_image(), cv2.COLOR_RGB2BGR)
        s = min(960 / frame.width, 540 / frame.height)
        width, height = round(frame.width * s), round(frame.height * s)
        require(width >= 1 and height >= 1, "image too thin for the reference viewport")
        resized = cv2.resize(image, (width, height))
    except (OSError, ValueError, cv2.error) as exc:
        text("IMAGE READ FAILED - no relative position", 16, 640, (100, 130, 255))
        return canvas, {"id": frame.id, "status": "error", "reason": str(exc), "direction": None}
    ox, oy = (960 - width) // 2, 80 + (540 - height) // 2
    canvas[oy:oy+height, ox:ox+width] = resized
    colors = ((205, 235, 25), (95, 200, 255))
    centers = []
    for name, b, color in zip(CLASSES, frame.boxes, colors):
        if b is None:
            centers.append(None)
            continue
        x0, y0, x1, y1 = b
        start = (round(ox + x0*s), round(oy + y0*s))
        end = (round(ox + x1*s), round(oy + y1*s))
        point = (round(ox + (x0+x1)*s/2), round(oy + (y0+y1)*s/2))
        cv2.rectangle(canvas, start, end, color, 2)
        cv2.circle(canvas, point, 3, color, -1)
        text(name, max(0, min(start[0], 840)), max(78, start[1]-7), color, 0.45)
        centers.append(point)
    vector = relative_vector(frame.boxes)
    label = None
    if vector is not None:
        reference = reference_vector(vector, frame.width, frame.height)
        label = direction(reference)
        if centers[0] != centers[1]:
            cv2.arrowedLine(canvas, centers[1], centers[0], (250, 250, 250), 2, tipLength=0.08)
        horizontal = {-1: "LEFT", 0: "HORIZONTAL CENTER", 1: "RIGHT"}[label[0]]
        vertical = {-1: "UP", 0: "VERTICAL CENTER", 1: "DOWN"}[label[1]]
        text(f"lip_balm relative to gripper: {horizontal} / {vertical}", 16, 644)
    else:
        absent = ", ".join(name for name, b in zip(CLASSES, frame.boxes) if b is None)
        text(f"ANNOTATED ABSENT: {absent} | no relative position", 16, 644)
    return canvas, {"id": frame.id, "status": "ok", "direction": label,
                    "labelKind": "approved" if frame.approved else "candidate_reference",
                    "isModelPrediction": False, "sourceSha256": frame.sha256}


def render_frames(frames, output_dir):
    from pathlib import Path

    frames = tuple(frames)
    require(bool(frames), "no frames to render")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)
    rows = []
    for index, frame in enumerate(frames, 1):
        image, row = render_frame(frame)
        name = f"{index:04d}.png"
        require(cv2.imwrite(str(output_dir / name), image), "could not write preview")
        rows.append(dict(row, preview=name))
    return {"schema": "motionbrain.cv_preview.v1", "isModelPrediction": False,
            "directionBandReferencePx": 10, "referenceViewport": [960, 540],
            "frames": rows, "processingFailures": sum(r["status"] != "ok" for r in rows)}
