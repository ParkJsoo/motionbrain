"""Fixed-view central silhouette arcs, not control or model authority.

This file does not write annotations, change a registration, access devices, or
fit an action model. Its small fixed-view checks are not measured accuracy.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import time

import cv2
import numpy as np

METHOD = 'central_dual_arc_circle.v1'
PARAMETERS = dict(imageShape=[240, 320], columns=[140, 176], rows=[50, 148],
    darkThreshold=90, minimumDarkRun=55, maximumDarkRun=88,
    minimumColumns=32, minimumColumnSpan=30, edgeProbeDistance=2,
    minimumMedianEdgeContrast=45., minimumEdgeContrast=30.,
    outlierDistancePx=1.5, minimumInlierFraction=.9,
    maximumRmsPx=.8, maximumResidualPx=1.5,
    radiusRangePx=[30., 45.], centerXRangePx=[148., 168.], centerYRangePx=[88., 108.])


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def parameter_sha(p):
    return sha(json.dumps(p, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def _fit(points):
    design = np.column_stack([points, np.ones(len(points))])
    require(np.linalg.matrix_rank(design) == 3, 'object_arc_degenerate')
    coefficients = np.linalg.lstsq(design, np.sum(points*points, axis=1), rcond=None)[0]
    center = coefficients[:2]/2
    radius_squared = float(coefficients[2]+np.sum(center*center))
    require(np.isfinite(center).all() and math.isfinite(radius_squared) and radius_squared > 0,
            'object_arc_fit_invalid')
    radius = math.sqrt(radius_squared)
    errors = np.abs(np.linalg.norm(points-center, axis=1)-radius)
    return center, radius, errors


def extract_object_arc(jpeg, parameters=None):
    started = time.monotonic()
    p = copy.deepcopy(PARAMETERS if parameters is None else parameters)
    # A future different contract must use another explicitly reviewed method.
    require(p == PARAMETERS and parameter_sha(p) == parameter_sha(PARAMETERS),
            'object_arc_fixed_parameters_required')
    require(isinstance(jpeg, bytes) and 0 < len(jpeg) <= 2_000_000, 'object_arc_image_bytes')
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    require(image is not None and list(image.shape[:2]) == p['imageShape'], 'object_arc_image_shape')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    xmin, xmax = p['columns']; ymin, ymax = p['rows']; probe = p['edgeProbeDistance']
    upper, lower, edge_contrasts, columns = [], [], [], []
    rejected_columns = []
    for x in range(xmin, xmax+1):
        ys = np.flatnonzero(gray[ymin:ymax+1, x] < p['darkThreshold'])+ymin
        if not len(ys):
            rejected_columns.append(x); continue
        segments = np.split(ys, np.flatnonzero(np.diff(ys) > 1)+1)
        segment = max(segments, key=len)
        a, b = int(segment[0]), int(segment[-1])
        if not (p['minimumDarkRun'] <= len(segment) <= p['maximumDarkRun']
                and ymin < a < b < ymax):
            rejected_columns.append(x); continue
        contrasts = [int(gray[a-probe, x])-int(gray[a+probe, x]),
                     int(gray[b+probe, x])-int(gray[b-probe, x])]
        if min(contrasts) < p['minimumEdgeContrast']:
            rejected_columns.append(x); continue
        upper.append([float(x), a-.5]); lower.append([float(x), b+.5])
        edge_contrasts.extend(contrasts); columns.append(x)
    require(len(columns) >= p['minimumColumns'] and max(columns)-min(columns) >= p['minimumColumnSpan'],
            'object_arc_coverage_insufficient')
    require(float(np.median(edge_contrasts)) >= p['minimumMedianEdgeContrast'], 'object_arc_edges_blurred')
    points = np.asarray(upper+lower, dtype=float)
    center, radius, errors = _fit(points)
    inliers = errors <= p['outlierDistancePx']
    require(float(inliers.mean()) >= p['minimumInlierFraction'], 'object_arc_outlier_fraction')
    n = len(upper)
    require(int(inliers[:n].sum()) >= p['minimumColumns']-2
            and int(inliers[n:].sum()) >= p['minimumColumns']-2,
            'object_arc_both_arcs_required')
    # At most one predetermined robust refit, never tune thresholds to a frame.
    used = points[inliers]
    if not inliers.all():
        center, radius, errors = _fit(used)
    rms = float(np.sqrt(np.mean(errors*errors)))
    require(rms <= p['maximumRmsPx'] and float(errors.max()) <= p['maximumResidualPx'],
            'object_arc_residual_exceeded')
    require(p['radiusRangePx'][0] <= radius <= p['radiusRangePx'][1]
            and p['centerXRangePx'][0] <= center[0] <= p['centerXRangePx'][1]
            and p['centerYRangePx'][0] <= center[1] <= p['centerYRangePx'][1],
            'object_arc_geometry_changed')
    return dict(method=METHOD, parametersSha256=parameter_sha(p), imageSha256=sha(jpeg),
        centerPx=center.tolist(), radiusPx=radius, rmsPx=rms,
        maximumResidualPx=float(errors.max()), columns=len(columns),
        rejectedColumns=rejected_columns, inlierPoints=len(used), listedPoints=len(points),
        medianEdgeContrast=float(np.median(edge_contrasts)), elapsedSeconds=time.monotonic()-started,
        qualityFlags=[], trainingEligible=False, controlEligible=False,
        measurementAccuracyVerified=False, generalizationVerified=False)


def registered_object_point(reference_jpeg, current_jpeg, reference_point):
    require(isinstance(reference_point, list) and len(reference_point) == 2
            and all(type(v) in (int, float) and math.isfinite(v) for v in reference_point),
            'object_arc_reference_point_required')
    before = extract_object_arc(reference_jpeg); after = extract_object_arc(current_jpeg)
    displacement = np.asarray(after['centerPx'])-before['centerPx']
    return dict(point=(np.asarray(reference_point)+displacement).tolist(),
        displacementPx=displacement.tolist(), reference=before, current=after,
        meaning='analyst_seed_plus_registered_arc_center_displacement_not_absolute_calibration',
        trainingEligible=False, controlEligible=False)
