"""Measure a short wrist or elbow response when the comparative model proposes it."""
import time

from learning.action_execution import finite
from learning.dual_view_policy import usable
from learning.open_side_approach import LEGACY_SIDE_VIEW

LABEL = 'wrist/down'
COMMAND = dict(motor=2, kind='pulse', percent=0, pwm=-120, targetDegrees=0, durationMs=400)
ELBOW_LABEL = 'elbow/down'
ELBOW_COMMAND = dict(motor=3, kind='pulse', percent=0, pwm=95, targetDegrees=0, durationMs=400)
RESPONSE_FIELDS = ('sx', 'sy', 'nx', 'ny', 'gap')


def profiles(plan, *, elbow=False):
    key = 'elbowResponseMeasurement' if elbow else 'wristResponseMeasurement'
    label, command = (ELBOW_LABEL, ELBOW_COMMAND) if elbow else (LABEL, COMMAND)
    if (plan.get(key) != command
            or type(plan.get('maxActions')) is not int or plan['maxActions'] != 1
            or type(plan.get('seconds')) is not int or plan['seconds'] < 8
            or plan['external']['viewConfigurationId'] != LEGACY_SIDE_VIEW
            or plan.get('useDevicePoseLimits') is not True or 'responsePolicy' not in plan
            or label not in plan['profiles'] or 'developmentPick' in plan or plan.get('poseAwareCandidates')
            or any(plan.get(key) is not True for key in (
                'ackAlignedGripperCapture', 'asyncActionLookup', 'refreshExpiredDispatch'))):
        raise ValueError(f'invalid_{label.split("/")[0]}_response_measurement_plan')
    learned = plan['profiles'][label]['command']
    if (any(learned[k] != command[k] for k in command if k != 'durationMs')
            or learned['durationMs'] <= command['durationMs']):
        raise ValueError(f'{label.split("/")[0]}_measurement_requires_matching_longer_response')
    return {label: dict(command=command)}


def measurement_vote(vote, policy, labels, *, elbow=False):
    """Calibration stays separate from the existing model's recorded action."""
    f = vote.get('features')
    keys = ('sy', 'nx', 'gap', 'leftMargin', 'rightMargin', 'objectWidth', 'objectHeight')
    supported = (vote.get('reason') in ('no_predicted_two_view_gain', 'two_view_response_improvement')
                 and isinstance(f, dict) and all(finite(f.get(key)) for key in keys))
    clear = (supported and f['sy'] < -15 and abs(f['nx']) <= 8 and 110 <= f['gap'] <= 150
             and min(f['leftMargin'], f['rightMargin']) >= 12
             and f['objectWidth'] > 0 and f['objectHeight'] > 0)
    label = ELBOW_LABEL if elbow else LABEL
    joint = label.split('/')[0]
    proposal = None
    if clear:
        proposal = elbow_proposal(f, policy, labels) if elbow else policy.decide(
            f, eligible=labels, compare_goal_set=True)['action']
    vote['policyAction'] = vote['action']
    vote['policyReason'] = vote.get('reason')
    vote['responseMeasurementProposal'] = proposal
    vote['action'] = label if proposal == label else 'hold'
    vote['reason'] = f'{joint}_response_measurement' if proposal == label else f'{joint}_measurement_visual_stop'


def elbow_proposal(f, policy, labels):
    """Compare the demonstrated direction with staying put for this calibration.

    The longer pulse supplies directional evidence. The short pulse's endpoint is measured.
    """
    if ELBOW_LABEL not in labels or not all(finite(f.get(key)) for key in ('ny', 'height')):
        return 'hold'
    _, dy, dx_near, dy_near, _ = policy.responses[ELBOW_LABEL]['delta']
    if f['sy']+dy > .45*f['height']:
        return 'hold'

    def deficit(sy, nx, ny):
        lateral = max(abs(nx)-8, 0)
        shallow = max(20-sy, sy-42, 0) + max(14-ny, ny-44, 0)
        deep = max(22-sy, sy-82, 0) + max(8-ny, ny-44, 0)
        return lateral + min(shallow, deep)

    before = deficit(f['sy'], f['nx'], f['ny'])
    after = deficit(f['sy']+dy, f['nx']+dx_near, f['ny']+dy_near)
    return ELBOW_LABEL if after < before else 'hold'


def observe_after_off(control, off_pose, result, *, elbow=False, sleep=time.sleep):
    """Record four two-view measurements after a verified physical stop."""
    label, command = (ELBOW_LABEL, ELBOW_COMMAND) if elbow else (LABEL, COMMAND)
    motor, pwm = command['motor'], command['pwm']
    action = result['action']
    terminal = action['record']
    entries = action['outputLog']['entries']
    drives = [e for e in entries if e['pwm']]
    last_drive = max((i for i, e in enumerate(entries) if e['pwm']), default=-1)
    valid = (all(action['request'].get(k) == v for k, v in command.items())
             and terminal['outputOff'] is True and drives and
             all(e['motor'] == motor and 0 < e['pwm']*pwm <= pwm*pwm for e in drives)
             and any(e['motor'] == motor and e['pwm'] == 0
                     and (e['atMs'] - terminal['startedAtMs']) % 2**32 <= command['durationMs'] + 1
                     for e in entries[last_drive + 1:]))
    report = dict(commandId=result['commandId'], outputVerified=bool(valid), status='response_unverified', pairs=[])
    if not valid:
        return report
    _, boundary = off_pose()
    boundary = max(boundary, action['outputOffHostBounds']['latest'])
    report['offConfirmedAt'] = boundary
    until = control.clock() + 3.
    sides, grippers = set(), set()
    while control.clock() < until and len(report['pairs']) < 4:
        sample = control.latest()
        if sample and sample.get('fault'):
            break
        if usable(sample, boundary, control.clock(), control.age, control.skew):
            side, gripper = sample['frameId'], sample['nearFrameId']
            if side not in sides and gripper not in grippers:
                sides.add(side)
                grippers.add(gripper)
                report['pairs'].append({k: sample.get(k) for k in (
                    'frameId', 'nearFrameId', 'policyAction', 'extClock', 'nearClock', 'features')})
        sleep(.02)
    report['observedShoulderDeg'], report['endingOffConfirmedAt'] = off_pose()
    stop = (terminal['state'], terminal['reason'])
    expected_stop = (stop in (('completed', 'duration_elapsed'), ('timed_out', 'deadline_exceeded'))
                     or (stop == ('cancelled', 'cancelled') and result.get('cancelConfirmed') is True))
    if (expected_stop and (not result.get('cancelAttempted') or result.get('cancelConfirmed') is True)
            and len(report['pairs']) == 4 and all(
                isinstance(pair['features'], dict)
                and all(finite(pair['features'].get(key)) for key in RESPONSE_FIELDS)
                for pair in report['pairs'])):
        report['status'] = f'{label.split("/")[0]}_response_measured'
    return report
