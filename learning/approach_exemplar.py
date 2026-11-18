"""Reuse a physically crossed approach waypoint in one fixed two-camera setup."""

from math import isfinite

import cv2
import numpy as np


AS5600_TICK = 360 / 4096
DISPLAY_ERROR = 0.01


def near_object_extent(image):
    """Bounding edges of a single pink component on the model's gripper thumbnail."""
    thumb = np.asarray(image).reshape(2, 24, 32, 3)[1]
    rgb = np.rint(thumb * 255).astype(np.uint8)
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (147, 95, 95), (179, 255, 255))
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count != 2:
        return None
    x, y, w, h, _ = stats[1]
    return [int(x), int(y), int(x + w), int(y + h)]


def exemplar_waypoint(exemplar, rows, decision, angle, scene_mse, max_mse, obj, extent):
    if (
        decision["targetAngleDeg"] is not None
        or decision["action"] != "shoulder/down"
        or decision["reason"] != "no_demonstration_matches_angle_and_scene"
    ):
        return None
    if not (187 <= angle <= 224 and scene_mse <= max_mse and obj and obj["upright"] is True):
        return None
    reference = exemplar["object"]
    if (
        not isfinite(obj["bottom"])
        or not isfinite(obj["center"][0])
        or abs(obj["bottom"] - reference["bottom"]) > 6
        or abs(obj["center"][0] - reference["center"][0]) > 12
    ):
        return None
    if (
        extent is None
        or len(extent) != 4
        or not all(lo <= v <= hi for lo, v, hi in zip(exemplar["nearMin"], extent, exemplar["nearMax"]))
    ):
        return None
    lower, upper = exemplar["anchors"]
    matches = {c["episode"]: c for c in decision["comparisons"]}
    if not matches[lower]["sceneSupported"] or not matches[upper]["angleSupported"]:
        return None
    row = next(r for r in rows if r["episode"] == upper)
    goal = exemplar["goal"]
    if not (
        exemplar["startAngle"] <= angle <= row["stationary"][1] + AS5600_TICK + DISPLAY_ERROR
        and goal in row["candidates"]
        and 1 <= angle - goal <= 8
    ):
        return None
    return dict(
        kind="crossed_approach_waypoint",
        targetAngleDeg=goal,
        sourceTrial=exemplar["sourceTrial"],
        anchorEpisodes=[lower, upper],
    )
