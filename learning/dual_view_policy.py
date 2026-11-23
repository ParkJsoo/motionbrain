"""Two-view visual imitation and bounded, single-owner action switching."""
import time
import uuid

import numpy as np
from PIL import Image

from learning.action_execution import FIELDS, SCHEMA as ACTION_RESULT_SCHEMA, canonical_request, finite
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


class OutsideDemonstratedStartPose(ValueError):
    pass


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
            raise OutsideDemonstratedStartPose('outside_demonstrated_start_pose')
        return self.validate(dict(command, bootId=boot, commandId=cid))


class GripperDispatchGate:
    """Pause between GETs, then let the capture process request its next frame."""

    def __init__(self, requested, paused):
        self.requested, self.paused = requested, paused

    def request_pause(self):
        if self.requested.is_set() or self.paused.is_set():
            raise ValueError('gripper_pause_already_pending')
        self.requested.set()

    def resume(self):
        self.requested.clear()

    def before_capture(self, stopping, until, *, check, clock=time.monotonic):
        check()
        if self.requested.is_set():
            self.paused.set()
            try:
                while self.requested.is_set():
                    if clock() >= until or stopping.wait(.005):
                        return False
                    check()
            finally:
                self.paused.clear()
            check()
        return not stopping.is_set() and clock() < until


def usable(sample, boundary, now, max_age, max_skew):
    if not fresh_pair_after(sample, boundary, now):
        return False
    a, b = sample['extClock'], sample['nearClock']
    earliest = min(a['sourceEarliest'], b['sourceEarliest'])
    skew = max(a['sourceLatest']-b['sourceEarliest'], b['sourceLatest']-a['sourceEarliest'])
    return now-earliest <= max_age and skew <= max_skew


class DispatchObservationExpired(ValueError):
    pass


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
        if (not usable(first, after, now, self.age, self.skew)
                or first['action'] != sample['action'] or not self._independent(first, sample)):
            return None
        return first, sample

    @staticmethod
    def _independent(first, sample):
        return (not sample.get('activeCommandId') and not first.get('activeCommandId')
                and sample.get('closeCommandId') == first.get('closeCommandId')
                and all(first[k] != sample[k] for k in ('frameId','nearFrameId'))
                and all(first[k]['sourceEarliest'] < sample[k]['sourceEarliest']
                        for k in ('extClock','nearClock')))

    def _check_dispatch_vote(self, sample, label, boundary, now, *, oldest=None):
        if (not isinstance(sample, dict) or sample.get('fault')
                or sample.get('modelAction',sample.get('action')) != label
                or (oldest is not None and sample['lastDissent'].get(label,0) > oldest)):
            raise ValueError('visual_start_changed')
        if not usable(sample,boundary,now,self.age,self.skew):
            # Diagnose age on this same snapshot, never use the wider test to admit a vote.
            if usable(sample,boundary,now,3.,self.skew):
                raise DispatchObservationExpired('dispatch_observation_expired')
            raise ValueError('visual_start_changed')
        if sample['action'] != label:
            raise ValueError('visual_start_changed')

    def wait_for_fresh_admission(self, label, boundary, *, sleep):
        """A reserved request can proceed only if two new pairs still select it."""
        deadline = self.clock() + 3.
        while self.clock() < deadline:
            sample = self.latest()
            if sample and sample.get('fault'):
                raise ValueError('camera_stopped_during_admission_refresh')
            pairs = self.admission(sample, boundary)
            if pairs:
                if sample['action'] != label:
                    raise ValueError('model_choice_changed_during_admission_refresh')
                return pairs
            sleep(.02)
        raise ValueError('fresh_pair_timeout_during_admission_refresh')

    def dispatch_admission(self, label, boundary, verify_pose, *, synchronize=False,
                           gripper_gate=None, sleep=time.sleep, before_aligned_pose=None):
        """Keep the last pose verification next to submit; optionally align it with a new vote."""
        clocks = ('extClock', 'nearClock')
        if synchronize or gripper_gate is not None:
            initial = self.latest()
            started = self.clock()
            self._check_dispatch_vote(initial,label,boundary,started)
            oldest = min(initial[k]['sourceEarliest'] for k in clocks)
            if gripper_gate is not None:
                gripper_gate.request_pause()
            # Wait with all outputs OFF, ahead of the final device read. Never extend an
            # active feedback deadline to compensate for device, camera or disk latency.
            while self.clock() < started + self.age:
                sample = self.latest()
                self._check_dispatch_vote(sample,label,boundary,self.clock(),oldest=oldest)
                ready = (gripper_gate.paused.is_set() if gripper_gate is not None else
                         all(sample[k]['sourceEarliest'] > initial[k]['sourceEarliest'] for k in clocks)
                         and all(sample[k] != initial[k] for k in ('frameId', 'nearFrameId')))
                pairs = self.admission(sample, boundary) if ready else None
                if pairs:
                    if before_aligned_pose is not None:
                        before_aligned_pose(pairs)
                    break
                sleep(.01)
            else:
                raise ValueError('final_admission_expired')
        verify_pose()
        current = self.latest()
        now = self.clock()
        aligned = synchronize or gripper_gate is not None
        first = current.get('previousPair') if isinstance(current,dict) else None
        if (not usable(first,boundary,now,3.,self.skew) or not usable(current,boundary,now,3.,self.skew)
                or not self._independent(first,current)
                or first['action'] != label or first.get('fault')
                or (gripper_gate is not None and not gripper_gate.paused.is_set())
                or (aligned and any(current[k]['sourceEarliest'] < sample[k]['sourceEarliest'] for k in clocks))):
            raise ValueError('visual_start_changed')
        self._check_dispatch_vote(current,label,boundary,now,oldest=oldest if aligned else None)
        self._check_dispatch_vote(first,label,boundary,now)
        return first,current

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
                                  modelAction=(sample or {}).get('modelAction', (sample or {}).get('action')),
                                  effectiveAction=(sample or {}).get('action')))
            return outcome
        return check


def run_actions(control, catalog, executor, boot, off_pose, max_actions, record, *, sleep=time.sleep,
                on_active=None, sync_dispatch=False, gripper_gate=None, side_stop=None, finish_after=None,
                lookup_worker=None, refresh_expired_dispatch=False):
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
        _, lo, hi = catalog.profiles[label]
        pose_checks = []

        def read_start_pose():
            angle, proved_at = off_pose()
            pose_checks.append(dict(shoulderDeg=angle, offConfirmedAt=proved_at))
            return angle, proved_at

        def pair_ids(pairs):
            return [[x['frameId'], x['nearFrameId']] for x in (pairs or ())]

        angle, _ = read_start_pose()
        cid = uuid.uuid4().hex
        try:
            request = catalog.request(label, angle, boot, cid)
        except OutsideDemonstratedStartPose as exc:
            record(dict(schemaVersion=ACTION_RESULT_SCHEMA, commandId=cid, executionState='not_sent',
                        reason=str(exc), dispatchAttempted=False,
                        automaticRetry=False, physicalArrivalVerified=False, trainingEligible=False,
                        observedRunningPairs=0, motionFeedback=[], actionLabel=label, offConfirmedAt=boundary,
                        admissionPairs=pair_ids(admission), shoulderRangeDeg=[lo, hi], startPoseChecks=pose_checks))
            return str(exc)
        executor.propose(request)
        feedback = None
        trace = []
        gate_timing = {}
        final_boundary = boundary

        def guard(req):
            nonlocal feedback, admission, final_boundary
            def verify_pose():
                angle, proved_at = read_start_pose()
                if catalog.request(label, angle, boot, cid) != req:
                    raise ValueError('pose_or_profile_changed')
                return proved_at
            def check_admission():
                def remember_aligned_pose(pairs):
                    gate_timing.setdefault('alignedPoseAdmissions', []).append(pair_ids(pairs))

                return control.dispatch_admission(label, final_boundary, verify_pose,
                    synchronize=sync_dispatch, gripper_gate=gripper_gate, sleep=sleep,
                    before_aligned_pose=remember_aligned_pose)
            try:
                admission = check_admission()
            except ValueError as exc:
                if not refresh_expired_dispatch or not isinstance(exc,DispatchObservationExpired):
                    gate_timing['finalAdmissionError'] = str(exc)
                    raise
                if gripper_gate is not None:
                    gripper_gate.resume()
                refresh = gate_timing['dispatchRefresh'] = {
                    'expiredPairs': [[x[k] for k in ('frameId','nearFrameId')] for x in admission]}
                admission = None
                try:
                    final_boundary = verify_pose()
                    refresh['offConfirmedAt'] = final_boundary
                    fresh = control.wait_for_fresh_admission(label, final_boundary, sleep=sleep)
                    refresh['pairs'] = [[x[k] for k in ('frameId','nearFrameId')] for x in fresh]
                    admission = check_admission()
                except ValueError as retry_error:
                    refresh['reason'] = str(retry_error)
                    raise
            if gripper_gate is not None:
                gate_timing['gripperPauseConfirmedAt'] = control.clock()
            if side_stop:
                veto = side_stop.check(label, final_boundary, admission=True)
                if veto:
                    trace.append(dict(sideStop=veto))
                    raise ValueError('side_stop_before_submit')
                latest = control.latest()
                if (not control.admission(latest, final_boundary) or latest['action'] != label
                        or not side_stop.covers(latest['frameId'])
                        or latest['lastDissent'].get(label, 0) > min(
                            admission[1][key]['sourceEarliest'] for key in ('extClock', 'nearClock'))):
                    raise ValueError('side_check_outlasted_admission')
                admission = latest['previousPair'], latest
            feedback = control.feedback(label, final_boundary, trace, admitted_pair=admission[1])

        published = False
        def during(req, ack):
            nonlocal published
            if on_active is not None and not published:
                on_active(req, admission[1], ack)
                published = True
            if gripper_gate is not None and 'gripperReleaseAfterAckAt' not in gate_timing:
                gate_timing['gripperReleaseAfterAckAt'] = control.clock()
                gripper_gate.resume()
            if side_stop:
                veto = side_stop.check(label, ack)
                if veto:
                    trace.append(dict(sideStop=veto, at=control.clock(), acknowledged=ack))
                    return 'cancel'
            return feedback(req, ack)
        try:
            result = executor.confirm(cid, confirmed_request=request, dispatch_guard=guard,
                                      motion_feedback=during, lookup_worker=lookup_worker)
        finally:
            if gripper_gate is not None:
                gripper_gate.resume()
            if published:
                on_active(None, None, None)
        if side_stop and result['executionState'] == 'known_terminal' and not any('sideStop' in r for r in trace):
            # Drain observations through OFF. A late veto stops the episode without another device write.
            ack = result['action']['startHostBounds']['latest']
            off = result['action']['outputOffHostBounds']['latest']
            until = control.clock() + control.age
            while control.clock() < until:
                veto = side_stop.check(label, ack)
                if veto:
                    trace.append(dict(sideStop=veto, terminalAlreadyObserved=True))
                    break
                if side_stop.last_source is not None and side_stop.last_source > off:
                    break
                sleep(.01)
            else:
                trace.append(dict(sideStop=dict(reason='side_frames_missing_at_off'), terminalAlreadyObserved=True))
        unique_pairs = {tuple(row['frames']) for row in trace if row.get('postAckPair')}
        record(dict(result, observedRunningPairs=len(unique_pairs), motionFeedback=trace, actionLabel=label, offConfirmedAt=final_boundary,
                    admissionPairs=pair_ids(admission), shoulderRangeDeg=[lo, hi], startPoseChecks=pose_checks,
                    **gate_timing))
        if result['executionState'] != 'known_terminal':
            return result['executionState']
        terminal = result['action']['record']
        if result.get('cancelAttempted') and not result.get('cancelConfirmed'):
            return 'unowned_or_late_cancel'
        if any('sideStop' in row for row in trace):
            return 'side_stop'
        if terminal['state'] != 'completed' and not result.get('cancelConfirmed'):
            return 'device_stopped'
        if label == finish_after:
            return 'observe_after_lift'
    return 'action_budget'
