"""명시적으로 선택하는 감독 수집 조건. 기존 기본 프로필은 바꾸지 않는다."""
from __future__ import annotations

from learning.supervised_transport import JOINTS

ATTENDED_60_1000 = 'attended-60pct-1000ms-v1'
ATTENDED_SHOULDER_DOWN_12 = 'attended-shoulder-down-12deg-v1'


def shoulder_down_window():
    return dict(schemaVersion='motionbrain.attended_shoulder_down_window.v1',
                maximumDecreaseDeg=12., maximumIncreaseDeg=3., beforeRunDriftDeg=3.,
                physicalStopDistanceGuaranteed=False)


def collection_profile(name):
    if name not in (ATTENDED_60_1000, ATTENDED_SHOULDER_DOWN_12):
        raise ValueError('unknown_attended_collection_profile')
    profile = dict(schemaVersion='motionbrain.attended_collection_profile.v1', name=name,
                minimumPercent=60, maximumPercent=100,
                minimumDurationMs=1000, maximumDurationMs=1500,
                maximumCommands=1,
                directions={joint: list(directions) if joint != 'shoulder' else ['down']
                            for joint, (_, directions) in JOINTS.items()},
                supervision='onsite_operator', automaticRetry=False)
    if name == ATTENDED_SHOULDER_DOWN_12:
        profile['directions'] = {'shoulder': ['down']}
        profile['attendedShoulderDownWindow'] = shoulder_down_window()
    return profile


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


def audit_profile(plan, session, pulse, receipt=None, *, motion_not_started=False):
    """자료를 학습할 때도 원 계획·종료 기록·명령이 같은 정책인지 확인한다."""
    if type(motion_not_started) is not bool:
        raise ValueError('attended_collection_profile_not_started_boolean_required')
    if motion_not_started:
        # Recovery-only metadata audit. The caller must first audit every raw
        # HTTP intent as GET-only. This never admits a training transition.
        events = session.get('events')
        if (receipt is not None or session.get('phase') != 'failed'
                or session.get('error') != 'command_failed_no_retry'
                or session.get('finalIdleOffVerified') is not False
                or session.get('datasetEligible') is not False
                or not isinstance(events, list) or not events
                or not all(isinstance(e, dict) for e in events)
                or sum(e.get('kind') == 'command' for e in events) != 1
                or any(e.get('kind') not in ('command', 'http', 'http_error')
                       or e.get('kind') in ('http', 'http_error') and e.get('payload') is not None
                       for e in events)):
            raise ValueError('attended_collection_profile_not_started_evidence_required')
    profile = plan.get('collectionProfile')
    if profile is None or profile != session.get('collectionProfile'):
        raise ValueError('attended_collection_profile_provenance')
    if not isinstance(profile, dict) or profile != collection_profile(profile.get('name')):
        raise ValueError('attended_collection_profile_provenance')
    command = {k: pulse[k] for k in ('joint', 'direction', 'percent', 'durationMs')}
    if pulse != validate_command(profile['name'], command):
        raise ValueError('attended_collection_profile_pulse')
    window = profile.get('attendedShoulderDownWindow')
    if window is not None:
        if (plan.get('attendedShoulderDownWindow') != window
                or session.get('attendedShoulderDownWindow') != window
                or not motion_not_started and (not isinstance(receipt, dict)
                    or receipt.get('attendedShoulderDownWindow') != window)):
            raise ValueError('attended_shoulder_down_window_provenance')
    elif any('attendedShoulderDownWindow' in value for value in (plan, session, receipt or {})):
        raise ValueError('attended_shoulder_down_window_unplanned')
    return profile.copy()


def audit_shoulder_down_observations(statuses, pulse, receipt):
    """새 정책의 전·중 상태를 원본 HTTP 시간 경계에서 다시 검증한다."""
    from learning.supervised_transport import SequencePort
    policy = shoulder_down_window()
    if receipt.get('attendedShoulderDownWindow') != policy:
        raise ValueError('attended_shoulder_down_window_provenance')
    command = {k: pulse[k] for k in ('joint', 'direction', 'percent', 'durationMs')}
    if validate_command(ATTENDED_SHOULDER_DOWN_12, command) != pulse:
        raise ValueError('attended_shoulder_down_window_exact_pulse')
    idle = [e for e in statuses if e['responseReceivedAt'] < receipt['dispatchBefore']
            and e['response'].get('state') == 'IDLE']
    armed = [e for e in statuses if e['responseReceivedAt'] < receipt['dispatchBefore']
             and e['response'].get('state') == 'ARMED']
    active = [e for e in statuses if receipt['dispatchAfter'] <= e['requestStartedAt'] < receipt['offAt']
              and e['response'].get('state') == 'ARMED']
    if not idle or not armed or not active:
        raise ValueError('attended_shoulder_down_window_status_evidence_required')
    port = object.__new__(SequencePort)
    port._attended_shoulder_down_window_policy = policy
    baseline = port._shoulder_angle(idle[-1]['response'])
    for event in armed:
        port._check_status(event['response'], 'ARMED')
        if abs(port._shoulder_angle(event['response'])-baseline) > policy['beforeRunDriftDeg']:
            raise ValueError('m4_observed_change_before_run')
    for event in active:
        port._check_status(event['response'], 'ARMED', pulse, baseline)
