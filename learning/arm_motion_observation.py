"""두 저장 영상 사이의 투영 회전 관측. 관절각 보정이나 실물 제어가 아니다.

특징점은 첫 영상의 지정 영역에서 한 번만 선택한다. 실패 시 새 특징점을
획득하거나 각도 후보를 반환하지 않는다. 임계값은 영상 품질 검사일 뿐
실물 안전 조건이나 측정 정확도의 보증이 아니다.
"""
import math

import cv2
import numpy as np


_DEFAULTS = {
    "max_features": 180, "quality_level": .01, "min_distance_px": 4.,
    "min_features": 12, "min_inliers": 8, "min_inlier_ratio": .70,
    "min_fb_fraction": .50, "max_fb_error_px": .75,
    "min_spread_px": 4., "ransac_threshold_px": 1.5,
    "max_inlier_rmse_px": .8, "max_scale_deviation": .025,
    "max_background_scale_deviation": .015,
    "max_background_translation_px": 2., "max_background_rotation_deg": .5,
    "max_projected_rotation_deg": 15.,
}


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _configuration(parameters):
    _require(parameters is None or type(parameters) is dict, "invalid_observer_parameters")
    parameters = parameters or {}
    _require(not (set(parameters) - set(_DEFAULTS)), "unknown_observer_parameter")
    result = dict(_DEFAULTS, **parameters)
    for key, value in result.items():
        _require(type(value) in (int, float) and 0 < value < 1e6 and math.isfinite(value),
                 "invalid_observer_parameter:" + key)
    for key in ("max_features", "min_features", "min_inliers"):
        _require(type(result[key]) is int and result[key] >= 3, "invalid_observer_count:" + key)
    _require(result["max_features"] >= result["min_features"] >= result["min_inliers"],
             "invalid_observer_count_order")
    for key in ("quality_level", "min_inlier_ratio", "min_fb_fraction", "max_scale_deviation",
                "max_background_scale_deviation"):
        _require(result[key] <= 1, "invalid_observer_fraction:" + key)
    return result


def _gray(frame):
    _require(isinstance(frame, np.ndarray) and frame.dtype == np.uint8,
             "observer_frame_must_be_uint8")
    _require(frame.ndim in (2, 3) and min(frame.shape[:2]) >= 32, "invalid_observer_frame_shape")
    if frame.ndim == 2:
        return np.ascontiguousarray(frame)
    _require(frame.shape[2] in (3, 4), "invalid_observer_frame_channels")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY if frame.shape[2] == 3 else cv2.COLOR_BGRA2GRAY)


def _mask(polygon, shape):
    _require(type(polygon) is list and 3 <= len(polygon) <= 64, "invalid_observer_polygon")
    for point in polygon:
        _require(type(point) is list and len(point) == 2 and all(
            type(x) in (int, float) and -1e9 < x < 1e9 and math.isfinite(x) for x in point),
            "invalid_observer_polygon_point")
    points = np.array(polygon, dtype=np.float32)
    height, width = shape
    _require(bool(np.all(points >= 0)) and bool(np.all(points[:, 0] < width))
             and bool(np.all(points[:, 1] < height)), "observer_polygon_outside_frame")
    # Convex ROIs avoid ambiguous self-intersections and rasterization holes.
    _require(cv2.isContourConvex(points) and abs(cv2.contourArea(points)) >= 100
             and bool(np.all(np.ptp(points, axis=0) >= 10)), "observer_polygon_tiny_or_nonconvex")
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 255)
    return mask


def _track(before, after, masks, cfg):
    corners, labels, seeded = [], [], []
    for label, mask in enumerate(masks):
        found = cv2.goodFeaturesToTrack(before, maxCorners=cfg["max_features"],
            qualityLevel=cfg["quality_level"], minDistance=cfg["min_distance_px"], mask=mask, blockSize=5)
        count = 0 if found is None else len(found)
        seeded.append(count)
        if count:
            corners.append(found)
            labels.extend([label] * count)
    diagnostic = {"seeded": sum(seeded), "seededPerRegion": seeded,
                  "forwardTracked": 0, "backwardTracked": 0, "fbAccepted": 0,
                  "fbAcceptedPerRegion": [0] * len(masks)}
    if any(n < cfg["min_features"] for n in seeded):
        return None, diagnostic, "insufficient_baseline_features"
    p0 = np.concatenate(corners).astype(np.float32)
    labels = np.array(labels)
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
    p1, status, _ = cv2.calcOpticalFlowPyrLK(before, after, p0, None, **lk)
    if p1 is None or status is None:
        return None, diagnostic, "forward_tracking_failed"
    a, b = p0.reshape(-1, 2), p1.reshape(-1, 2)
    height, width = before.shape
    good = (status.ravel() != 0) & np.isfinite(b).all(axis=1)
    good &= (b[:, 0] >= 0) & (b[:, 0] < width) & (b[:, 1] >= 0) & (b[:, 1] < height)
    diagnostic["forwardTracked"] = int(good.sum())
    if not good.any():
        return None, diagnostic, "forward_tracking_failed"
    a, b, labels = a[good], b[good], labels[good]
    back, status_back, _ = cv2.calcOpticalFlowPyrLK(after, before, b.reshape(-1, 1, 2), None, **lk)
    if back is None or status_back is None:
        return None, diagnostic, "backward_tracking_failed"
    back = back.reshape(-1, 2)
    backward_valid = (status_back.ravel() != 0) & np.isfinite(back).all(axis=1)
    diagnostic["backwardTracked"] = int(backward_valid.sum())
    fb = np.linalg.norm(back - a, axis=1)
    accepted = backward_valid & (fb <= cfg["max_fb_error_px"])
    counts = [int(np.sum(accepted & (labels == region))) for region in range(len(masks))]
    diagnostic.update(fbAccepted=int(accepted.sum()), fbAcceptedPerRegion=counts,
                      fbMedianErrorPx=float(np.median(fb[accepted])) if accepted.any() else None,
                      fbMaxErrorPx=float(np.max(fb[accepted])) if accepted.any() else None)
    if any(n < cfg["min_inliers"] or n / initial < cfg["min_fb_fraction"]
           for n, initial in zip(counts, seeded)):
        return None, diagnostic, "insufficient_forward_backward_tracks"
    return (a[accepted], b[accepted], labels[accepted]), diagnostic, None


def _fit(tracks, cfg):
    source, target, regions = tracks
    matrix, mask = cv2.estimateAffinePartial2D(source, target, method=cv2.RANSAC,
        ransacReprojThreshold=cfg["ransac_threshold_px"], maxIters=2000, confidence=.99, refineIters=10)
    if matrix is None or mask is None or not np.isfinite(matrix).all():
        return None, "rigid_fit_failed"
    inlier = mask.ravel() != 0
    residual = np.linalg.norm(source @ matrix[:, :2].T + matrix[:, 2] - target, axis=1)
    count = int(inlier.sum())
    spread = float(np.sqrt(max(0., np.linalg.eigvalsh(np.cov(source[inlier].T)).min()))) if count >= 3 else 0.
    region_counts = [int(np.sum(inlier & (regions == region))) for region in range(int(regions.max()) + 1)]
    scale = float(math.hypot(matrix[0, 0], matrix[1, 0]))
    diagnostic = {
        "transform": matrix.tolist(), "inliers": count, "tracked": len(source),
        "inlierRatio": count / len(source), "inliersPerRegion": region_counts,
        "minorAxisSpreadPx": spread, "scale": scale,
        "rawScreenRotationDeg": math.degrees(math.atan2(matrix[1, 0], matrix[0, 0])),
        "affineTranslationPx": matrix[:, 2].tolist(),
        "translationNormPx": float(np.linalg.norm(matrix[:, 2])),
        "inlierRmsePx": float(np.sqrt(np.mean(residual[inlier] ** 2))) if count else None,
        "allTrackedRmsePx": float(np.sqrt(np.mean(residual ** 2))),
    }
    if any(n < cfg["min_inliers"] or n / int(np.sum(regions == region)) < cfg["min_inlier_ratio"]
           for region, n in enumerate(region_counts)):
        return diagnostic, "insufficient_rigid_inliers"
    if spread < cfg["min_spread_px"]:
        return diagnostic, "insufficient_spatial_spread"
    if diagnostic["inlierRmsePx"] > cfg["max_inlier_rmse_px"]:
        return diagnostic, "excessive_rigid_residual"
    return diagnostic, None


def observe_pair(before, after, moving_polygon, background_polygons, forward_image_sign, parameters=None) -> dict:
    """Analyst 지정 영역의 투영 회전을 관측한다. 입력 오류는 ValueError다.

    background_polygons는 서로 겹치지 않는 볼록 영역 두 개 이상이다.
    배경이 움직이거나 확대되면 각도 차를 출력하지 않는다.
    """
    cfg = _configuration(parameters)
    _require(type(forward_image_sign) is int and forward_image_sign in (-1, 1), "invalid_observer_forward_sign")
    first, second = _gray(before), _gray(after)
    _require(first.shape == second.shape, "observer_frame_shape_mismatch")
    _require(type(background_polygons) is list and len(background_polygons) >= 2,
             "observer_requires_two_background_regions")
    moving = _mask(moving_polygon, first.shape)
    backgrounds = [_mask(polygon, first.shape) for polygon in background_polygons]
    occupied = moving.copy()
    for mask in backgrounds:
        _require(not bool(np.any((occupied != 0) & (mask != 0))), "observer_regions_overlap")
        occupied |= mask
    result = {"schema": "motionbrain.arm_motion_observation.pair.v1", "valid": False,
              "reason": None, "projectedDeltaDeg": None, "moving": {}, "background": {},
              "featurePolicy": "baseline_only_no_reacquisition", "qualityParameters": cfg,
              "measurement": "relative_image_plane_rotation_not_3d_joint_angle",
              "physicallyCalibrated": False, "controlEligible": False}
    for name, masks in (("moving", [moving]), ("background", backgrounds)):
        try:
            tracks, diagnostic, reason = _track(first, second, masks, cfg)
            result[name]["tracking"] = diagnostic
            if reason:
                result["reason"] = name + ":" + reason
                return result
            fit, reason = _fit(tracks, cfg)
            result[name]["fit"] = fit
            if reason:
                result["reason"] = name + ":" + reason
                return result
        except cv2.error:
            result["reason"] = name + ":opencv_processing_failed"
            return result
    moving_fit, background_fit = result["moving"]["fit"], result["background"]["fit"]
    checks = [
        (abs(moving_fit["scale"] - 1) <= cfg["max_scale_deviation"], "moving_scale_changed"),
        (abs(background_fit["scale"] - 1) <= cfg["max_background_scale_deviation"], "background_scale_changed"),
        (abs(background_fit["rawScreenRotationDeg"]) <= cfg["max_background_rotation_deg"], "background_rotated"),
        (background_fit["translationNormPx"] <= cfg["max_background_translation_px"], "background_translated"),
    ]
    for valid, reason in checks:
        if not valid:
            result["reason"] = reason
            return result
    difference = moving_fit["rawScreenRotationDeg"] - background_fit["rawScreenRotationDeg"]
    difference = math.degrees(math.atan2(math.sin(math.radians(difference)), math.cos(math.radians(difference))))
    if abs(difference) > cfg["max_projected_rotation_deg"]:
        result["reason"] = "projected_rotation_outside_tracking_range"
        return result
    result.update(valid=True, reason="projected_rotation_observed",
                  projectedDeltaDeg=-forward_image_sign * difference)
    return result
