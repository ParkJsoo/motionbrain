"""Offline dynamics and action ranking for the open-jaw, two-camera approach.

The source must supply observations under one camera/landmark contract, with
signed state [side tool minus object center X, side tool minus object top Y,
near object minus jaw midpoint X, near object minus jaw midpoint Y]. Camera
coordinates increase right and down. Model outputs do not authorize motion.
"""
from dataclasses import dataclass
from math import dist, isfinite
from statistics import median


# Development neighborhood in object widths, object heights, jaw spans,
# shoulder degrees and jaw-span pixels, respectively.
NEIGHBORHOOD = (.85, .55, .10, .22, 8., 12.)
STATE_FIELDS = ('sideXObjectWidths', 'sideYObjectHeights', 'nearXJawSpans',
                'nearYJawSpans', 'shoulderDeg', 'jawSpanPx')


def _numbers(values, length):
    if (not isinstance(values, (list, tuple)) or len(values) != length
            or any(type(v) not in (int, float) or not isfinite(v) for v in values)):
        raise ValueError('finite_numeric_vector_required')
    return tuple(float(v) for v in values)


def relative_state(observation):
    x, y, near_x, near_y = _numbers(observation['state'], 4)
    _, _, width, height = _numbers(observation['box'], 4)
    gap, angle = _numbers([observation['gap'], observation['angle']], 2)
    if observation['upright'] is not True or min(width, height, gap) <= 0 or not 0 <= angle < 360:
        raise ValueError('upright_object_and_open_jaw_measurement_required')
    return _numbers((x / width, y / height, near_x / gap, near_y / gap, angle, gap), 6)


def _command(values):
    if not isinstance(values, (list, tuple)) or len(values) != 5:
        raise ValueError('joint_direction_speed_duration_profile_required')
    joint, direction, speed, duration, profile = values
    if (joint not in ('base', 'shoulder', 'elbow', 'wrist')
            or direction not in ('left', 'right', 'up', 'down')
            or type(speed) is not int or not 0 < speed <= 100
            or type(duration) is not int or duration <= 0
            or not isinstance(profile, str) or not profile):
        raise ValueError('invalid_approach_command')
    return tuple(values)


def _median_vector(vectors):
    return tuple(median(axis) for axis in zip(*vectors))


def _plus(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _minus(a, b):
    return tuple(x - y for x, y in zip(a, b))


@dataclass(frozen=True)
class Transition:
    setup: str
    command: tuple
    before: tuple
    delta: tuple


class RelativeApproachModel:
    """Fit observed deltas, weighting separate physical setups equally."""

    def __init__(self, rows):
        self.transitions = []
        setup_by_frame = {}
        for row in rows:
            setup = row['setup']
            if not isinstance(setup, str) or not setup:
                raise ValueError('physical_setup_id_required')
            before = relative_state(row['before'])
            after = row['after']
            frames = [row['before']['frame'], *(item['frame'] for item in after)]
            if (len(after) != 2 or any(not isinstance(f, str) or not f for f in frames)
                    or len(set(frames)) != len(frames)):
                raise ValueError('distinct_before_and_two_settled_after_frames_required')
            for frame in frames:
                if setup_by_frame.setdefault(frame, setup) != setup:
                    raise ValueError('frame_shared_across_physical_setups')
            end = _median_vector([relative_state(item) for item in after])
            self.transitions.append(Transition(setup, _command(row['command']), before, _minus(end, before)))

    def _predict(self, state, command, *, exclude_setup=None):
        query = tuple(v / scale for v, scale in zip(state, NEIGHBORHOOD))
        by_setup = {}
        for row in self.transitions:
            if row.command != command or row.setup == exclude_setup:
                continue
            key = tuple(v / scale for v, scale in zip(row.before, NEIGHBORHOOD))
            by_setup.setdefault(row.setup, []).append((dist(query, key), row.delta))

        # One nearest response per setup. Repetitions in a single approach
        # trajectory cannot outvote evidence from another physical setup.
        nearest = {}
        for setup, samples in sorted(by_setup.items()):
            distance = min(d for d, _ in samples)
            nearest[setup] = distance, _median_vector([delta for d, delta in samples if d == distance])
        baseline = _median_vector([_median_vector([delta for _, delta in samples])
                                   for samples in by_setup.values()])
        outcomes = [_plus(state, delta) for d, delta in nearest.values() if d <= 1.]
        return dict(
            predictedState=_median_vector(outcomes) if outcomes else None,
            commandOnlyState=_plus(state, baseline) if baseline else None,
            localOutcomes=outcomes,
            nearestDistanceBySetup={setup: d for setup, (d, _) in nearest.items()},
        )

    def select(self, observation, goal_bands, commands, *, minimum_setups=2, exclude_setup=None):
        """Rank the caller's candidate commands against four relative goal bands.

        The representative response per nearby setup must preserve or improve
        each axis and improve at least one. Actuator admission is separate.
        """
        state = relative_state(observation)
        bands = tuple(_numbers(band, 2) for band in goal_bands)
        if len(bands) != 4 or any(lo > hi for lo, hi in bands):
            raise ValueError('four_ordered_relative_goal_bands_required')
        if type(minimum_setups) is not int or minimum_setups < 1:
            raise ValueError('positive_minimum_setups_required')

        def error(values):
            return tuple(max(lo - v, v - hi, 0.) for v, (lo, hi) in zip(values, bands))

        current = error(state)
        candidates = []
        for command in sorted(set(map(_command, commands))):
            candidate = self._predict(state, command, exclude_setup=exclude_setup)
            outcomes = [error(end) for end in candidate['localOutcomes']]
            rejection = None
            improvement = None
            if len(outcomes) < minimum_setups:
                rejection = 'insufficient_local_setup_support'
            elif any(any(b > a + 1e-9 for a, b in zip(current, after)) for after in outcomes):
                rejection = 'observed_response_worsens_goal_axis'
            else:
                improvement = min(sum(current) - sum(after) for after in outcomes)
                if improvement <= 1e-9:
                    rejection = 'no_consistent_progress'
            candidates.append(dict(command=command, rejection=rejection,
                                   worstCaseObservedImprovement=improvement, **candidate))
        eligible = [c for c in candidates if c['rejection'] is None]
        best = max(eligible, key=lambda c: c['worstCaseObservedImprovement'], default=None)
        reason = ('relative_goal_progress' if best else
                  'within_goal_bands' if not any(current) else 'no_supported_improving_command')
        return dict(stateFields=STATE_FIELDS, inputState=state, selected=best['command'] if best else None,
                    reason=reason, candidates=candidates, controlEligible=False)

    def replay(self):
        """Predict only logged commands, excluding the entire physical setup."""
        decisions = []
        for row in self.transitions:
            prediction = self._predict(row.before, row.command, exclude_setup=row.setup)
            target = _plus(row.before, row.delta)
            decisions.append(dict(setup=row.setup, command=row.command, observedState=target,
                                  noMotionState=row.before, **prediction))
        matched = [row for row in decisions if row['predictedState'] is not None]
        fields = ('predictedState', 'commandOnlyState', 'noMotionState')
        mae = {name: tuple(sum(abs(v) for v in axis) / len(matched) for axis in zip(*(
            _minus(row[name], row['observedState']) for row in matched))) if matched else None
            for name in fields}
        return dict(stateFields=STATE_FIELDS, transitions=len(decisions), matchedTransitions=len(matched),
                    sameSubsetMeanAbsoluteError=mae, decisions=decisions, controlEligible=False)
