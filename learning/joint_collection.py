"""Finite M2 pilot protocol. Pure planning: no hardware, camera or fitted policy."""
import math

SPLITS = ('train', 'train', 'validation', 'test')
POSES = {'A': (7., 10.), 'B': (3., 6.)}
ANGLE_ENVELOPE = (-10., 24.)
MAX_POSITIONING_PER_TARGET = 2
MAX_POSITIONING_PER_SESSION = 16
CAPTURE_SECONDS = 150
MAX_FRAMES = 1520
CONDITIONS = (('A','down',700), ('B','up',700), ('A','down',900),
              ('B','up',900), ('A','up',700), ('B','down',700),
              ('A','up',900), ('B','down',900))


def conditions(session):
    if type(session) is not int or not 0 <= session < 4:
        raise ValueError('four_fixed_sessions_required')
    # Keep all eight conditions in every split; freeze order before collection.
    order = CONDITIONS if session % 2 == 0 else tuple(reversed(CONDITIONS))
    return [dict(conditionId=f'{p}-{d}-{ms}', pose=p, direction=d, durationMs=ms)
            for p,d,ms in order]


def check_angle(angle):
    if type(angle) not in (int,float) or not math.isfinite(angle) or not ANGLE_ENVELOPE[0] <= angle <= ANGLE_ENVELOPE[1]:
        raise ValueError('outside_projected_observation_envelope')


def in_pose(angle, pose):
    check_angle(angle)
    lo,hi=POSES[pose]
    return lo <= angle <= hi


def action_for(condition, angle, positioning_used, total_positioning):
    """Choose only a pre-authorized pulse; an exhausted budget never retries."""
    if condition not in conditions(0):
        raise ValueError('unknown_condition')
    check_angle(angle)
    if in_pose(angle,condition['pose']):
        direction=condition['direction'];duration=condition['durationMs'];kind='target'
    else:
        if positioning_used >= MAX_POSITIONING_PER_TARGET or total_positioning >= MAX_POSITIONING_PER_SESSION:
            raise ValueError('positioning_budget_exhausted')
        centre=sum(POSES[condition['pose']])/2;error=centre-angle
        direction='up' if error>0 else 'down';kind='positioning'
        duration=900 if abs(error)>6 else 700 if abs(error)>3 else 550
    return dict(kind=kind,conditionId=condition['conditionId'],pose=condition['pose'],
                joint='wrist',motor=2,direction=direction,percent=63,
                expectedPwm=160 if direction=='up' else -160,durationMs=duration)


def validate_action(action):
    if action.get('conditionId') not in {c['conditionId'] for c in conditions(0)}:
        raise ValueError('unknown_condition')
    c=next(c for c in conditions(0) if c['conditionId']==action['conditionId'])
    expected=dict(joint='wrist',motor=2,percent=63,pose=c['pose'])
    if any(action.get(k)!=v for k,v in expected.items()):raise ValueError('action_scope_mismatch')
    if action.get('direction') not in ('up','down') or action.get('expectedPwm')!=(160 if action['direction']=='up' else -160):raise ValueError('action_direction_mismatch')
    if action.get('kind')=='target':
        if action['direction']!=c['direction'] or action.get('durationMs')!=c['durationMs']:raise ValueError('target_condition_mismatch')
    elif action.get('kind')=='positioning':
        if type(action.get('durationMs')) is not int or action['durationMs'] not in (550,700,900):raise ValueError('positioning_duration_mismatch')
    else:raise ValueError('action_kind_mismatch')
    return action
