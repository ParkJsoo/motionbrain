"""명시적으로 선택하는 감독 수집 조건. 기존 기본 프로필은 바꾸지 않는다."""
from __future__ import annotations

from learning.supervised_transport import JOINTS

ATTENDED_60_1000 = 'attended-60pct-1000ms-v1'


def collection_profile(name):
    if name != ATTENDED_60_1000:
        raise ValueError('unknown_attended_collection_profile')
    return dict(schemaVersion='motionbrain.attended_collection_profile.v1', name=name,
                minimumPercent=60, maximumPercent=100,
                minimumDurationMs=1000, maximumDurationMs=1500,
                maximumCommands=1,
                directions={joint: list(directions) if joint != 'shoulder' else ['down']
                            for joint, (_, directions) in JOINTS.items()},
                supervision='onsite_operator', automaticRetry=False)


def validate_command(name, command):
    profile = collection_profile(name)
    joint = command['joint']
    if command['direction'] not in profile['directions'].get(joint, []):
        raise ValueError('joint_direction_not_enabled')
    if (type(command['percent']) is not int
            or not profile['minimumPercent'] <= command['percent'] <= profile['maximumPercent']
            or type(command['durationMs']) is not int
            or not profile['minimumDurationMs'] <= command['durationMs'] <= profile['maximumDurationMs']):
        raise ValueError('attended_collection_output_limit')
    motor, directions = JOINTS[joint]
    return dict(**command, motor=motor,
                expectedPwm=directions[command['direction']] * (command['percent'] * 255 // 100))


def audit_profile(plan, session, pulse):
    """자료를 학습할 때도 원 계획·종료 기록·명령이 같은 정책인지 확인한다."""
    profile = plan.get('collectionProfile')
    if profile is None or profile != session.get('collectionProfile'):
        raise ValueError('attended_collection_profile_provenance')
    if not isinstance(profile, dict) or profile != collection_profile(profile.get('name')):
        raise ValueError('attended_collection_profile_provenance')
    command = {k: pulse[k] for k in ('joint', 'direction', 'percent', 'durationMs')}
    if pulse != validate_command(profile['name'], command):
        raise ValueError('attended_collection_profile_pulse')
    return profile.copy()
