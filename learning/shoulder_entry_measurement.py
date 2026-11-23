"""One-shot calibration of a downward shoulder step before a grasp."""
import time

from learning.action_execution import finite
from learning.dual_view_policy import usable
from learning.open_side_approach import LEGACY_SIDE_VIEW


LABEL = 'shoulder/down'
START_RANGE = (215.85, 215.87)
CLOSE_RANGE = (214.97, 215.15)
DEFAULT_SPEED = 100
COMMAND = dict(motor=4, kind='shoulder', percent=75, pwm=0, targetDegrees=213, durationMs=300)
# Separate one-shot step from the measured 300 ms endpoint. Truncating that
# action's driver ramp to 225 ms gives a screening estimate, not a travel guarantee.
FOLLOWUP_COMMAND = dict(COMMAND, durationMs=225)
FOLLOWUP_START_RANGE = (215.32, 215.34)


def validate_plan(plan):
    profile = plan.get('shoulderEntryMeasurement')
    supported_profiles = (
        dict(command=COMMAND, shoulderRangeDeg=list(START_RANGE)),
        dict(command=FOLLOWUP_COMMAND, shoulderRangeDeg=list(FOLLOWUP_START_RANGE)),
    )
    if (plan['external']['viewConfigurationId'] != LEGACY_SIDE_VIEW or 'responsePolicy' not in plan
            or profile not in supported_profiles
            or type(plan['maxActions']) is not int or plan['maxActions'] != 1
            or type(plan['seconds']) is not int or plan['seconds'] < 8
            or plan.get('ackAlignedGripperCapture') is not True or plan.get('asyncActionLookup') is not True
            or plan.get('refreshExpiredDispatch') is not True or 'developmentPick' in plan):
        raise ValueError('invalid_shoulder_entry_measurement_plan')
    return {LABEL: profile}


def measurement_vote(vote):
    """Retain the actual policy vote while applying the calibration clearance gate."""
    fields = vote.get('features')
    keys = ('sy', 'nx', 'gap', 'leftMargin', 'rightMargin', 'objectWidth', 'objectHeight')
    clearance = (
        isinstance(fields, dict) and all(finite(fields.get(key)) for key in keys)
        and fields['sy'] < -20 and abs(fields['nx']) <= 8 and 110 <= fields['gap'] <= 150
        and min(fields['leftMargin'], fields['rightMargin']) >= 12
        and fields['objectWidth'] > 0 and fields['objectHeight'] > 0
    )
    supported_open_approach = vote.get('reason') == 'two_view_response_improvement' and vote['action'] != 'hold'
    vote['policyAction'] = vote['action']
    vote['policyReason'] = vote.get('reason')
    vote['action'] = LABEL if clearance and supported_open_approach else 'hold'
    vote['reason'] = 'measurement_clearance' if vote['action'] == LABEL else 'measurement_visual_stop'


def supports_close(vote):
    return (vote.get('policyAction') == 'gripper/close'
            and vote.get('policyReason') == 'preclose_dual_view_match' and bool(vote.get('features')))


def measured_output(result, command):
    """Require same-command DOWN writes below the existing duty/time ceiling."""
    if (result['executionState'] != 'known_terminal'
            or (result.get('cancelAttempted') and result.get('cancelConfirmed') is not True)):
        return False
    action = result['action']
    if any(action['request'].get(key) != value for key, value in command.items()):
        return False
    terminal = action['record']
    ordinary_stop = (terminal['state'], terminal['reason']) in (
        ('completed', 'target_reached'), ('timed_out', 'deadline_exceeded'))
    targeted_stop = (terminal['state'], terminal['reason']) == ('cancelled', 'cancelled') and result.get('cancelConfirmed') is True
    if terminal['outputOff'] is not True or not (ordinary_stop or targeted_stop):
        return False
    entries = action['outputLog']['entries']
    motor = command['motor']
    driven = [e for e in entries if e['pwm'] != 0]
    if not driven or any(e['motor'] != motor or not -75 <= e['pwm'] < 0 for e in driven):
        return False
    last_power = max(i for i, e in enumerate(entries) if e['pwm'] != 0)
    # Driver timestamps are millisecond samples of the independent cutoff timer.
    return any(e['motor'] == motor and e['pwm'] == 0
               and (e['atMs'] - terminal['startedAtMs']) % 2**32 <= command['durationMs'] + 1
               for e in entries[last_power + 1:])


def observe_entry(control, off_pose, result, command, *, sleep=time.sleep):
    """Read settled OFF pose and four independent later image pairs."""
    report = dict(commandId=result['commandId'], status='unverified_output', pairs=[], readings=[])
    if not measured_output(result, command):
        return report
    off = result['action']['outputOffHostBounds']['latest']
    # Match the shoulder controller's settle window even if its timer ended
    # the command before it could decide that the shoulder had settled.
    sleep(max(0., off + .6 - control.clock()))
    angle, boundary = off_pose()
    report['readings'].append(dict(shoulderDeg=angle, offConfirmedAt=boundary))
    deadline = control.clock() + 3.
    seen_side, seen_gripper = set(), set()
    while control.clock() < deadline and len(report['pairs']) < 4:
        sample = control.latest()
        if sample and sample.get('fault'):
            break
        if usable(sample, boundary, control.clock(), control.age, control.skew):
            side, gripper = sample['frameId'], sample['nearFrameId']
            if (side not in seen_side and gripper not in seen_gripper
                    and (command == FOLLOWUP_COMMAND or sample.get('features'))):
                seen_side.add(side)
                seen_gripper.add(gripper)
                report['pairs'].append({key: sample.get(key) for key in (
                    'frameId', 'nearFrameId', 'extClock', 'nearClock', 'decidedAt', 'features', 'policyAction',
                    'policyReason', 'closeDissentAt')})
        sleep(.02)
    last_angle, last_read = off_pose()
    report['readings'].append(dict(shoulderDeg=last_angle, offConfirmedAt=last_read))
    if len(report['pairs']) != 4:
        report['status'] = 'post_off_pairs_missing'
    elif not all(CLOSE_RANGE[0] <= a <= CLOSE_RANGE[1] for a in (angle, last_angle)):
        report['status'] = 'outside_close_range'
    elif command == FOLLOWUP_COMMAND:
        first = report['pairs'][0]
        first_source = min(first[key]['sourceEarliest'] for key in ('extClock', 'nearClock'))
        ready = all(supports_close(pair) and finite(pair['closeDissentAt']) and pair['closeDissentAt'] <= first_source
                    for pair in report['pairs'])
        report['status'] = 'measured_in_close_range' if ready else 'close_visual_not_ready'
    else:
        report['status'] = 'measured_in_close_range'
    return report
