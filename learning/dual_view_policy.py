"""Two-view visual imitation and bounded, single-owner action switching."""
import time
import uuid

import numpy as np
from PIL import Image

from learning.action_execution import FIELDS, canonical_request, finite
from learning.supervised_transport import JOINTS
from learning.whole_arm_release import fresh_pair_after


ACTIONS = {f'{joint}/{direction}': (motor, sign)
           for joint, (motor, directions) in JOINTS.items() for direction, sign in directions.items()}


def thumbnail(image):
    return np.asarray(image.convert('RGB').resize((32, 24), Image.Resampling.BOX), dtype=np.float32) / 255


class DualViewPolicy:
    """Independent nearest-demonstration votes using the project's paired RGB features."""
    def __init__(self, examples, max_rmse):
        self.labels = [label for label, _, _ in examples]
        if not self.labels or any(label not in ACTIONS and label != 'hold' for label in self.labels):
            raise ValueError('unknown_or_empty_demonstration')
        if len(max_rmse) != 2 or not all(finite(v) and 0 < v < 1 for v in max_rmse):
            raise ValueError('invalid_view_error_limits')
        self.limits = max_rmse
        self.views = [np.stack([r[i] for r in examples]) for i in (1, 2)]
        if any(v.shape[1:] != (24, 32, 3) or not np.isfinite(v).all() for v in self.views):
            raise ValueError('invalid_demonstration_pixels')

    def predict(self, side, near):
        votes, errors = [], []
        for bank, image in zip(self.views, (side, near)):
            error = np.sqrt(np.mean((bank - thumbnail(image)) ** 2, axis=(1, 2, 3)))
            best = int(np.argmin(error))
            errors.append(float(error[best]))
            ties = {self.labels[i] for i in np.flatnonzero(error == error[best])}
            votes.append(self.labels[best] if len(ties) == 1 else 'hold')
        supported = all(e <= limit for e, limit in zip(errors, self.limits))
        return dict(action=votes[0] if supported and votes[0] == votes[1] else 'hold',
                    viewVotes=votes, viewRmse=errors)


class Catalog:
    """Bind semantic predictions to exact finite commands and measured starting envelopes."""
    def __init__(self, profiles):
        self.profiles = {}
        for label, profile in profiles.items():
            motor, sign = ACTIONS[label]
            command = profile['command']
            if set(command) != FIELDS - {'bootId', 'commandId'}:
                raise ValueError('invalid_command_fields')
            r = canonical_request(dict(command, commandId='validate', bootId='0'*32))
            lo, hi = profile['shoulderRangeDeg']
            if not (finite(lo) and finite(hi) and 0 <= lo <= hi < 360 and r['motor'] == motor):
                raise ValueError('invalid_start_envelope')
            if motor == 4:
                if sign * (r['targetDegrees'] - (hi if sign > 0 else lo)) <= 0:
                    raise ValueError('shoulder_target_direction_mismatch')
            elif sign * r['pwm'] <= 0:
                raise ValueError('motor_direction_mismatch')
            self.profiles[label] = (command.copy(), lo, hi)

    def validate(self, request):
        r = canonical_request(request)
        if not any(all(r[k] == v for k,v in cmd.items()) for cmd, _, _ in self.profiles.values()):
            raise ValueError('action_outside_catalog')
        return r

    def request(self, label, angle, boot, cid):
        command, lo, hi = self.profiles[label]
        if not finite(angle) or not lo <= angle <= hi:
            raise ValueError('outside_demonstrated_start_pose')
        return self.validate(dict(command, bootId=boot, commandId=cid))


def usable(sample, boundary, now, max_age, max_skew):
    if not fresh_pair_after(sample, boundary, now):
        return False
    a, b = sample['extClock'], sample['nearClock']
    earliest = min(a['sourceEarliest'], b['sourceEarliest'])
    skew = max(a['sourceLatest']-b['sourceEarliest'], b['sourceLatest']-a['sourceEarliest'])
    return now-earliest <= max_age and skew <= max_skew


class VisualControl:
    def __init__(self, latest, max_age, max_skew, *, clock=time.monotonic):
        if not all(finite(x) and 0 < x <= .69 for x in (max_age, max_skew)):
            raise ValueError('invalid_visual_time_budget')
        self.latest, self.age, self.skew, self.clock = latest, max_age, max_skew, clock

    def decision(self, after):
        sample = self.latest()
        if sample is None or not usable(sample, after, self.clock(), self.age, self.skew):
            return 'hold'
        return sample['action']

    def admission(self, sample, after):
        now = self.clock()
        if not usable(sample, after, now, self.age, self.skew):
            return None
        first = sample.get('previousPair')
        if (not usable(first, after, now, self.age, self.skew) or first['action'] != sample['action']
                or any(first[k] == sample[k] for k in ('frameId', 'nearFrameId'))
                or any(first[k]['sourceEarliest'] >= sample[k]['sourceEarliest']
                       for k in ('extClock', 'nearClock'))):
            return None
        return first, sample

    def feedback(self, label, boundary, trace=None, *, admitted_pair=None):
        # Only new evidence can sustain the current command. While the first
        # post-ack pair is in flight the last admitted OFF pair expires normally.
        last = self.latest() if admitted_pair is None else admitted_pair
        if not usable(last, boundary, self.clock(), self.age, self.skew) or last['action'] != label:
            raise ValueError('visual_start_changed')
        admitted = min(last[k]['sourceEarliest'] for k in ('extClock', 'nearClock'))
        deadline = admitted + self.age
        def check(request, acknowledged):
            nonlocal deadline
            now = self.clock()
            sample = self.latest()
            outcome, reason = 'continue', 'awaiting_post_ack_pair'
            if now >= deadline or sample is None or sample.get('fault'):
                outcome, reason = 'cancel', 'observation_expired_or_camera_stopped'
            elif sample['lastDissent'].get(label, 0) > admitted:
                outcome, reason = 'cancel', 'visual_dissent_latched'
            elif usable(sample, acknowledged, now, self.age, self.skew):
                if sample['action'] != label:
                    outcome, reason = 'cancel', 'visual_choice_changed'
                else:
                    reason = 'same_action'
                    deadline = min(sample[k]['sourceEarliest'] for k in ('extClock', 'nearClock')) + self.age
            elif all(sample[k]['sourceEarliest'] > acknowledged for k in ('extClock', 'nearClock')):
                outcome, reason = 'cancel', 'unusable_pair'
            if trace is not None:
                trace.append(dict(at=now, acknowledged=acknowledged, decision=outcome, reason=reason,
                                  postAckPair=usable(sample, acknowledged, now, self.age, self.skew),
                                  frames=[(sample or {}).get(k) for k in ('frameId', 'nearFrameId')],
                                  modelAction=(sample or {}).get('action')))
            return outcome
        return check


def run_actions(control, catalog, executor, boot, off_pose, max_actions, record, *, sleep=time.sleep):
    """One owner at a time. Each dispatch needs two independent pairs after OFF."""
    for _ in range(max_actions):
        _, boundary = off_pose()
        deadline = control.clock() + 3
        while control.clock() < deadline:
            sample = control.latest()
            if sample and sample.get('fault'):
                return 'camera_stopped'
            if usable(sample, boundary, control.clock(), control.age, control.skew):
                label = sample['action']
                if label not in catalog.profiles:
                    return 'hold_or_uncommissioned'
                admission = control.admission(sample, boundary)
                if admission:
                    break
            sleep(.02)
        else:
            return 'fresh_pair_timeout'
        angle, _ = off_pose()
        cid = uuid.uuid4().hex
        request = catalog.request(label, angle, boot, cid)
        executor.propose(request)
        feedback = None
        trace = []

        def guard(req):
            nonlocal feedback, admission
            angle, _ = off_pose()
            if catalog.request(label, angle, boot, cid) != req:
                raise ValueError('pose_or_profile_changed')
            current = control.latest()
            admission = control.admission(current, boundary)
            if not admission or current['action'] != label:
                raise ValueError('visual_start_changed')
            feedback = control.feedback(label, boundary, trace, admitted_pair=current)

        result = executor.confirm(cid, confirmed_request=request, dispatch_guard=guard,
                                  motion_feedback=lambda req, ack: feedback(req, ack))
        unique_pairs = {tuple(row['frames']) for row in trace if row['postAckPair']}
        record(dict(result, observedRunningPairs=len(unique_pairs), motionFeedback=trace, actionLabel=label, offConfirmedAt=boundary,
                    admissionPairs=[[x[k] for k in ('frameId', 'nearFrameId')] for x in (admission or ())]))
        if result['executionState'] != 'known_terminal':
            return result['executionState']
        terminal = result['action']['record']
        if result.get('cancelAttempted') and not result.get('cancelConfirmed'):
            return 'unowned_or_late_cancel'
        if terminal['state'] != 'completed' and not result.get('cancelConfirmed'):
            return 'device_stopped'
    return 'action_budget'
