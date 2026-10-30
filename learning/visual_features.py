"""Markerless, manually localized point tracking. No detector or learned model."""
from __future__ import annotations

import math
from learning.visual_action_collection import identifier


class PointTracker:
    def __init__(self, width, height, points, *, max_displacement_px, reference_limit_px):
        import cv2
        import numpy as np
        self.cv, self.np = cv2, np
        if not (type(width) is int and type(height) is int and 32 <= width <= 1920 and 32 <= height <= 1080):
            raise ValueError('invalid_tracking_shape')
        if not isinstance(points, list) or not 3 <= len(points) <= 12:
            raise ValueError('three_to_twelve_tracking_points_required')
        self.points, names = [], set()
        for p in points:
            if set(p) != {'id', 'x', 'y', 'reference'}:
                raise ValueError('invalid_tracking_point_fields')
            name = identifier(p['id'])
            if name in names or type(p['reference']) is not bool:
                raise ValueError('duplicate_or_invalid_tracking_point')
            names.add(name)
            if any(type(p[k]) not in (int, float) or not math.isfinite(p[k]) for k in ('x', 'y')) or not (16 <= p['x'] < width-16 and 16 <= p['y'] < height-16):
                raise ValueError('point_outside_tracking_margin')
            self.points.append(dict(p))
        if not any(p['reference'] for p in points) or sum(not p['reference'] for p in points) < 2:
            raise ValueError('reference_and_two_moving_points_required')
        if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 < v <= 200 for v in (max_displacement_px, reference_limit_px)):
            raise ValueError('invalid_visual_limits')
        self.width, self.height = width, height
        self.limits = np.array([reference_limit_px if p['reference'] else max_displacement_px for p in points])
        self.positions = np.array([[p['x'], p['y']] for p in points], np.float32).reshape(-1, 1, 2)
        self.initial = self.positions.copy()
        self.alive = np.ones(len(points), bool)
        self.previous = None
        self.fault = None
        self.settings = dict(winSize=(21, 21), maxLevel=3,
                             criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01), minEigThreshold=1e-4)

    def update(self, pixels):
        cv, np = self.cv, self.np
        if not isinstance(pixels, bytes) or len(pixels) != self.width*self.height*4:
            raise ValueError('tracking_frame_shape_mismatch')
        gray = cv.cvtColor(np.frombuffer(pixels, np.uint8).reshape(self.height, self.width, 4), cv.COLOR_BGRA2GRAY)
        if self.previous is None:
            # Refuse textureless initial points; do not silently relocate point IDs.
            eigen = cv.cornerMinEigenVal(gray, blockSize=3)
            for i, point in enumerate(self.positions.reshape(-1, 2)):
                x, y = np.rint(point).astype(int)
                self.alive[i] = eigen[y, x] >= 1e-4
        else:
            active = np.flatnonzero(self.alive)
            if len(active):
                p = self.positions[active]
                q, f, error = cv.calcOpticalFlowPyrLK(self.previous, gray, p, None, **self.settings)
                if q is None:
                    self.alive[active] = False
                else:
                    back, b, _ = cv.calcOpticalFlowPyrLK(gray, self.previous, q, None, **self.settings)
                    good = np.zeros(len(active), bool)
                    if back is not None:
                        xy = q.reshape(-1, 2)
                        good = (f.ravel() == 1) & (b.ravel() == 1) & np.isfinite(xy).all(axis=1)
                        good &= np.linalg.norm(back.reshape(-1, 2)-p.reshape(-1, 2), axis=1) <= 1
                        good &= np.isfinite(error.ravel()) & (error.ravel() <= 15)
                        good &= (xy[:, 0] >= 16) & (xy[:, 0] < self.width-16) & (xy[:, 1] >= 16) & (xy[:, 1] < self.height-16)
                    self.alive[active] &= good
                    self.positions[active[good]] = q[good]
        displacement = np.linalg.norm((self.positions-self.initial).reshape(-1, 2), axis=1)
        if not self.alive.all():
            self.fault = self.fault or 'tracking_lost'
        elif (displacement > self.limits).any():
            self.fault = self.fault or 'visual_travel_limit'
        self.previous = gray
        return [dict(id=p['id'], x=float(self.positions[i, 0, 0]) if self.alive[i] else None,
                     y=float(self.positions[i, 0, 1]) if self.alive[i] else None,
                     visible=bool(self.alive[i]), confidence=1.0 if self.alive[i] else 0.0)
                for i, p in enumerate(self.points)]
