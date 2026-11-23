"""Measure a short wrist response when the comparative model proposes it."""
import time

from learning.action_execution import finite
from learning.dual_view_policy import usable
from learning.open_side_approach import LEGACY_SIDE_VIEW

LABEL = 'wrist/down'
COMMAND = dict(motor=2, kind='pulse', percent=0, pwm=-120, targetDegrees=0, durationMs=400)
RESPONSE_FIELDS = ('sx', 'sy', 'nx', 'ny', 'gap')


def profiles(plan):
    if (plan.get('wristResponseMeasurement') != COMMAND
            or type(plan.get('maxActions')) is not int or plan['maxActions'] != 1
            or type(plan.get('seconds')) is not int or plan['seconds'] < 8
            or plan['external']['viewConfigurationId'] != LEGACY_SIDE_VIEW
            or plan.get('useDevicePoseLimits') is not True or 'responsePolicy' not in plan
            or LABEL not in plan['profiles'] or 'developmentPick' in plan or plan.get('poseAwareCandidates')
            or any(plan.get(key) is not True for key in (
                'ackAlignedGripperCapture', 'asyncActionLookup', 'refreshExpiredDispatch'))):
        raise ValueError('invalid_wrist_response_measurement_plan')
    learned = plan['profiles'][LABEL]['command']
    if any(learned[key] != COMMAND[key] for key in COMMAND if key != 'durationMs') or learned['durationMs'] <= 400:
        raise ValueError('wrist_measurement_requires_matching_longer_response')
    return {LABEL: dict(command=COMMAND)}


def measurement_vote(vote, policy, labels):
    """Calibration stays separate from the existing model's recorded action."""
    f = vote.get('features')
    keys = ('sy', 'nx', 'gap', 'leftMargin', 'rightMargin', 'objectWidth', 'objectHeight')
    supported = (vote.get('reason') in ('no_predicted_two_view_gain', 'two_view_response_improvement')
                 and isinstance(f, dict) and all(finite(f.get(key)) for key in keys))
    clear = (supported and f['sy'] < -15 and abs(f['nx']) <= 8 and 110 <= f['gap'] <= 150
             and min(f['leftMargin'], f['rightMargin']) >= 12
             and f['objectWidth'] > 0 and f['objectHeight'] > 0)
    comparison = policy.decide(f, eligible=labels, compare_goal_set=True) if clear else {}
    vote['policyAction'] = vote['action']
    vote['policyReason'] = vote.get('reason')
    vote['responseMeasurementProposal'] = comparison.get('action')
    vote['action'] = LABEL if comparison.get('action') == LABEL else 'hold'
    vote['reason'] = 'wrist_response_measurement' if vote['action'] == LABEL else 'wrist_measurement_visual_stop'


def observe_after_off(control, off_pose, result, *, sleep=time.sleep):
    """Record four two-view measurements after a verified physical stop."""
    action = result['action']
    terminal = action['record']
    entries = action['outputLog']['entries']
    drives = [e for e in entries if e['pwm']]
    last_drive = max((i for i, e in enumerate(entries) if e['pwm']), default=-1)
    valid = (all(action['request'].get(k) == v for k, v in COMMAND.items())
             and terminal['outputOff'] is True and drives and
             all(e['motor'] == 2 and -120 <= e['pwm'] < 0 for e in drives)
             and any(e['motor'] == 2 and e['pwm'] == 0
                     and (e['atMs'] - terminal['startedAtMs']) % 2**32 <= COMMAND['durationMs'] + 1
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
        report['status'] = 'wrist_response_measured'
    return report
