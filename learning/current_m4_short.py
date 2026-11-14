"""현재 197.22° 탁상 코호트의 새 짧은 UP/DOWN 계획 정책.

.9°는 센서 관측에 따른 거절 한도이며 물리 정지거리 보장이 아니다. 명령의
UP350·450·550/DOWN200·250ms는 요청 cap이다. 명시한 각 정책과 기존 고정·TARGET·복구
정책을 자동 혼합하지 않는다.
"""
from __future__ import annotations

import copy
import math
from learning.manual_collection_dataset import require

SCHEMA='motionbrain.m4_current_short_task.v1'
POLICY_SCHEMA='motionbrain.m4_current_short_paired_policy.v1'
UP_MONITOR_POLICY='m4-current-short-up-deadline-350-v1'
UP_MONITOR_POLICY450='m4-current-short-up-deadline-450-v1'
UP_MONITOR_POLICY550='m4-current-short-up-deadline-550-v1'
DOWN_MONITOR_POLICY='m4-current-short-down-deadline-200-v1'
DOWN_MONITOR_POLICY250='m4-current-short-down-deadline-250-v1'


def current_short_policy(up_duration_ms=350,down_duration_ms=200):
    require(type(up_duration_ms) is int and up_duration_ms in (350,450,550),'current_short_UP_cap_required')
    require(type(down_duration_ms) is int and down_duration_ms in (200,250),'current_short_DOWN_cap_required')
    return dict(schemaVersion=POLICY_SCHEMA,tableReferenceDeg=197.22,upGoalAngleDeg=197.85,
        downGoalAngleDeg=197.22,originalGlobalReferenceDeg=195.29,originalGlobalUpperDeg=198.29,
        maxObservedDeltaDeg=.9,upPercent=50,upDurationCapMs=up_duration_ms,downPercent=35,downDurationCapMs=down_duration_ms,
        upMonitorTimingPolicy={350:UP_MONITOR_POLICY,450:UP_MONITOR_POLICY450,550:UP_MONITOR_POLICY550}[up_duration_ms],
        downMonitorTimingPolicy=DOWN_MONITOR_POLICY if down_duration_ms==200 else DOWN_MONITOR_POLICY250)


def validate_current_short_policy(policy):
    require(isinstance(policy,dict),'current_short_policy_fields')
    expected=current_short_policy(policy.get('upDurationCapMs'),policy.get('downDurationCapMs'))
    require(isinstance(policy,dict) and set(policy)==set(expected),'current_short_policy_fields')
    for key,value in expected.items():
        supplied=policy[key]
        if type(value) is int:
            require(type(supplied) is int and supplied==value,'current_short_policy_changed')
        elif type(value) is float:
            require(type(supplied) in (int,float) and math.isfinite(supplied) and supplied==value,
                    'current_short_policy_changed')
        else:require(type(supplied) is str and supplied==value,'current_short_policy_changed')
    return copy.deepcopy(policy)


def short_task_pulse(direction,policy=None):
    require(direction in ('up','down'),'current_short_direction_required')
    policy=validate_current_short_policy(current_short_policy() if policy is None else policy)
    return dict(joint='shoulder',motor=4,direction=direction,percent=50 if direction=='up' else 35,
                durationMs=policy['upDurationCapMs'] if direction=='up' else policy['downDurationCapMs'],expectedPwm=127 if direction=='up' else -89)
