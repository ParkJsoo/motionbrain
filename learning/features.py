"""Planned-action dynamics features, independent of eligibility/provenance gates.

Only the loader may establish dataset eligibility. This pure transform cannot.
No post-action driver trace, terminal state or targetReached is an input feature.
"""
import math

FEATURE_VERSION = 'motionbrain.planned_visual_m4.v1'
INPUT_NAMES = ['before_center_x', 'before_center_y', 'before_width', 'before_height',
               'm4_sin', 'm4_cos', *[f'motor_{i}' for i in range(1, 6)],
               'is_shoulder', 'requested_pwm_fraction', 'requested_duration_seconds',
               'requested_target_delta_half_turn', 'requested_percent_fraction']
TARGET_NAMES = ['delta_center_x', 'delta_center_y', 'delta_width', 'delta_height', 'delta_m4_degrees']
TARGET_UNITS = ['image_width', 'image_height', 'image_width', 'image_height', 'degrees']


def number(value, name, low=None, high=None):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{name}: finite numeric value required')
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f'{name}: out of range')
    return float(value)


def wrap_degrees(angle):
    return (angle + 180) % 360 - 180


def visual_state(observation):
    if (observation.get('detected') is not True or observation.get('liveDetected') is not True or
            observation.get('held') is not False or not isinstance(observation.get('label'), str) or
            not observation['label'].strip() or
            observation.get('detectionSourceFrameId') != observation.get('sourceFrameId')):
        raise ValueError('exact live target required')
    width = number(observation.get('width'), 'width', 1)
    height = number(observation.get('height'), 'height', 1)
    box = observation.get('targetBox')
    if not isinstance(box, dict):
        raise ValueError('targetBox required; no inferred/default coordinates')
    x, y, w, h = [number(box.get(k), f'targetBox.{k}', 0) for k in ('x','y','width','height')]
    if w <= 0 or h <= 0 or x+w > width or y+h > height:
        raise ValueError('targetBox outside image')
    m4 = observation.get('m4', {})
    if m4.get('motor') != 4 or m4.get('sensor') != 'AS5600' or m4.get('measured') is not True:
        raise ValueError('measured M4 AS5600 required')
    angle = number(m4.get('angleDegrees'), 'm4.angleDegrees', -1e6, 1e6)
    return [(x+w/2)/width, (y+h/2)/height, w/width, h/height], angle


def transition_features(transition):
    before, after = transition['before'], transition['after']
    if any(before.get(k) != after.get(k) for k in ('width','height','label','classId')):
        raise ValueError('target category or image dimensions changed')
    b, angle = visual_state(before)
    a, after_angle = visual_state(after)
    request = transition['action']['request']
    motor, kind = request['motor'], request['kind']
    if type(motor) is not int or motor not in range(1,6) or kind not in ('pulse','shoulder'):
        raise ValueError('unsupported command')
    pwm = number(request['pwm'], 'pwm', -255, 255)
    duration = number(request['durationMs'], 'durationMs', 1, 10000)
    target = number(request['targetDegrees'], 'targetDegrees', -1e6, 1e6)
    percent = number(request['percent'], 'percent', 0, 100)
    if ((kind == 'shoulder' and (motor != 4 or pwm != 0 or percent == 0)) or
            (kind == 'pulse' and (motor == 4 or pwm == 0 or target != 0 or percent != 0))):
        raise ValueError('inconsistent command fields')
    radians = math.radians(angle)
    x = b + [math.sin(radians), math.cos(radians)] + [float(motor==i) for i in range(1,6)] + [
        float(kind=='shoulder'), pwm/255, duration/1000,
        wrap_degrees(target-angle)/180 if kind=='shoulder' else 0., percent/100]
    y = [new-old for old,new in zip(b,a)] + [wrap_degrees(after_angle-angle)]
    return x, y
