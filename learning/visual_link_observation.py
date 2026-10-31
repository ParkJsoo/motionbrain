"""Causal, device-free projected-link observations from fixed tracked point IDs.

This module neither tracks images nor dispatches commands. It does not certify
physical settling, optical timing, training eligibility, or collision clearance.
"""
from __future__ import annotations

from collections import deque
import math
from statistics import mean


GROUPS = ('background_a', 'background_b', 'parent_proximal', 'parent_distal',
          'child_proximal', 'child_distal')


def _number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def _wrap(value):
    return (value+180) % 360-180


def projected_relative_angle(parent_a, parent_b, child_a, child_b):
    """Signed 2D child-minus-parent direction; screen y may point down."""
    for point in (parent_a, parent_b, child_a, child_b):
        if not isinstance(point, (list, tuple)) or len(point) != 2 or not all(_number(v) for v in point):
            raise ValueError('finite_2d_points_required')
    if _distance(parent_a, parent_b) == 0 or _distance(child_a, child_b) == 0:
        raise ValueError('zero_length_link')
    parent = math.atan2(parent_b[1]-parent_a[1], parent_b[0]-parent_a[0])
    child = math.atan2(child_b[1]-child_a[1], child_b[0]-child_a[0])
    return _wrap(math.degrees(child-parent))


def _p90(values):
    values = sorted(values)
    index = .9*(len(values)-1)
    lo = math.floor(index)
    hi = math.ceil(index)
    return values[lo] + (values[hi]-values[lo])*(index-lo)


class CausalLinkObserver:
    """One fixed view and one episode; never reweights or reacquires lost IDs.

    Configure at least three manually assigned tracked IDs per landmark patch.
    Two *off-arm* background patches are distinct from both moving link frames.
    Patch centres use the same cohort throughout the episode. A quality fault
    latches until a new instance is explicitly created for a new episode.
    """

    def __init__(self, groups, *, max_background_motion_px=2.,
                 max_patch_deformation_px=2., max_link_length_change=.15,
                 min_link_length_px=20., min_confidence=.5,
                 max_frame_gap_seconds=.25, dwell_seconds=.75,
                 stable_diameter_px=.5, max_window_frames=64):
        if not isinstance(groups, dict) or set(groups) != set(GROUPS):
            raise ValueError('six_explicit_point_groups_required')
        names = []
        self.groups = {}
        for group in GROUPS:
            ids = groups[group]
            if not isinstance(ids, (list, tuple)) or not 3 <= len(ids) <= 8:
                raise ValueError('three_to_eight_fixed_ids_per_patch_required')
            if any(not isinstance(i, str) or not i or len(i) > 128 for i in ids):
                raise ValueError('point_id_required')
            names.extend(ids)
            self.groups[group] = tuple(ids)
        if len(names) != len(set(names)):
            raise ValueError('point_ids_must_be_disjoint_across_groups')
        limits = (max_background_motion_px, max_patch_deformation_px,
                  min_link_length_px, max_frame_gap_seconds, dwell_seconds,
                  stable_diameter_px)
        if any(not _number(v) or v <= 0 for v in limits):
            raise ValueError('positive_finite_observation_limits_required')
        if (not _number(max_link_length_change) or not 0 < max_link_length_change < 1
                or not _number(min_confidence) or not 0 <= min_confidence <= 1
                or type(max_window_frames) is not int or not 6 <= max_window_frames <= 512
                or max_frame_gap_seconds > dwell_seconds):
            raise ValueError('invalid_quality_or_window_limits')
        self.names = tuple(names)
        self.background_limit = max_background_motion_px
        self.deformation_limit = max_patch_deformation_px
        self.length_change = max_link_length_change
        self.min_length = min_link_length_px
        self.min_confidence = min_confidence
        self.max_gap = max_frame_gap_seconds
        self.dwell = dwell_seconds
        self.stable_limit = stable_diameter_px
        self.max_window_frames = max_window_frames
        self.initial = None
        self.initial_centres = None
        self.initial_lengths = None
        self.initial_angle = None
        self.last_at = None
        self.history = deque()
        self.fault = None

    def _centres(self, points):
        return {group: (mean(points[i][0] for i in ids),
                        mean(points[i][1] for i in ids))
                for group, ids in self.groups.items()}

    def _geometry(self, centres):
        p, q = centres['parent_proximal'], centres['parent_distal']
        r, s = centres['child_proximal'], centres['child_distal']
        lengths = (_distance(p, q), _distance(r, s))
        if min(lengths) < self.min_length:
            raise ValueError('projected_link_too_short')
        return lengths, projected_relative_angle(p, q, r, s)

    def _read(self, features):
        if not isinstance(features, list) or len(features) > 128:
            raise ValueError('feature_list_required')
        by_id = {}
        for f in features:
            if (not isinstance(f, dict) or set(f) != {'id', 'x', 'y', 'visible', 'confidence'}
                    or not isinstance(f['id'], str) or not f['id'] or f['id'] in by_id
                    or type(f['visible']) is not bool
                    or not _number(f['confidence']) or not 0 <= f['confidence'] <= 1):
                raise ValueError('invalid_or_duplicate_feature')
            if ((f['visible'] and not all(_number(f[k]) for k in ('x', 'y')))
                    or (not f['visible'] and (f['x'] is not None or f['y'] is not None))):
                raise ValueError('invalid_feature_coordinates')
            if f['visible'] and any(abs(f[k]) > 1_000_000 for k in ('x', 'y')):
                raise ValueError('feature_coordinates_out_of_range')
            by_id[f['id']] = f
        points = {}
        for name in self.names:
            f = by_id.get(name)
            if f is None or not f['visible'] or f['confidence'] < self.min_confidence:
                raise ValueError('fixed_point_missing_or_unreliable')
            points[name] = (float(f['x']), float(f['y']))
        return points

    def update(self, at, features):
        """Consume this frame only, in one declared time domain (seconds).

        Returns a new JSON-compatible record. Invalid input and lost points latch
        a quality fault; previous records are never changed. No hardware action
        is triggered by that fault. A caller must enforce its own execution guard.
        """
        record = dict(schemaVersion='motionbrain.visual_link_observation.v1',
                      at=at if _number(at) else None, valid=False, reason=self.fault,
                      relativeAngleDeg=None, relativeDeltaDeg=None,
                      patchCentresPx=None, linkLengthsPx=None,
                      stableWindow=False, windowStart=None, windowEnd=None,
                      patchTemporalDiameterP90Px=None,
                      fixedPointCount=len(self.names),
                      trainingEligible=False, controlEligible=False)
        if self.fault:
            return record
        try:
            if not _number(at):
                raise ValueError('finite_time_required')
            if self.last_at is not None:
                if at <= self.last_at:
                    raise ValueError('frame_time_repeated_or_regressed')
                if at-self.last_at > self.max_gap:
                    raise ValueError('frame_gap_exceeded')
            points = self._read(features)
            centres = self._centres(points)
            lengths, angle = self._geometry(centres)
            if self.initial is None:
                for ids in self.groups.values():
                    if any(_distance(points[i],points[j]) < .5
                           for n,i in enumerate(ids) for j in ids[n+1:]):
                        raise ValueError('distinct_patch_points_required')
                self.initial = points
                self.initial_centres = centres
                self.initial_lengths = lengths
                self.initial_angle = angle
            else:
                # Per-point checks prevent opposed movement cancelling at centres.
                for group in ('background_a', 'background_b'):
                    if any(_distance(points[i], self.initial[i]) > self.background_limit
                           for i in self.groups[group]):
                        raise ValueError('external_background_motion')
                for ids in self.groups.values():
                    for pos, i in enumerate(ids):
                        for j in ids[pos+1:]:
                            change = abs(_distance(points[i], points[j])-
                                         _distance(self.initial[i], self.initial[j]))
                            if change > self.deformation_limit:
                                raise ValueError('patch_not_rigid_or_tracking_drift')
                if any(abs(value/initial-1) > self.length_change
                       for value, initial in zip(lengths, self.initial_lengths)):
                    raise ValueError('projected_link_length_changed')
            self.last_at = float(at)
            self.history.append((float(at), points))
            # Keep the one sample at/before the dwell boundary as well as all
            # later samples. Never use a sample after the current frame.
            while len(self.history) > 1 and self.history[1][0] <= at-self.dwell:
                self.history.popleft()
            if len(self.history) > self.max_window_frames:
                raise ValueError('observation_window_capacity_exceeded')
            record.update(valid=True, reason=None, relativeAngleDeg=angle,
                          relativeDeltaDeg=_wrap(angle-self.initial_angle),
                          patchCentresPx={k:list(v) for k,v in centres.items()},
                          linkLengthsPx=list(lengths))
            if len(self.history) >= 6 and at-self.history[0][0] >= self.dwell:
                frames = [p for _,p in self.history]
                diameters = {}
                for group, ids in self.groups.items():
                    per_point = [max(_distance(a[i], b[i]) for a in frames for b in frames)
                                 for i in ids]
                    diameters[group] = _p90(per_point)
                record.update(stableWindow=all(v <= self.stable_limit for v in diameters.values()),
                              windowStart=self.history[0][0], windowEnd=float(at),
                              patchTemporalDiameterP90Px=diameters)
        except ValueError as exc:
            self.fault = str(exc)
            self.history.clear()
            record.update(valid=False, reason=self.fault)
        return record
