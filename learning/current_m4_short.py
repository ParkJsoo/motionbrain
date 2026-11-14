"""현재 197.22° 탁상 코호트의 새 짧은 UP/DOWN 계획 정책.

.9°는 센서 관측에 따른 거절 한도이며 물리 정지거리 보장이 아니다. 명령의
350/200ms는 요청 cap이다. 기존 고정·TARGET·복구 정책과 자동 혼합하지 않는다.
"""
from __future__ import annotations

import copy
import math
from learning.manual_collection_dataset import require

SCHEMA='motionbrain.m4_current_short_task.v1'
POLICY_SCHEMA='motionbrain.m4_current_short_paired_policy.v1'
UP_MONITOR_POLICY='m4-current-short-up-deadline-350-v1'
DOWN_MONITOR_POLICY='m4-current-short-down-deadline-200-v1'


def current_short_policy():
    return dict(schemaVersion=POLICY_SCHEMA,tableReferenceDeg=197.22,upGoalAngleDeg=197.85,
        downGoalAngleDeg=197.22,originalGlobalReferenceDeg=195.29,originalGlobalUpperDeg=198.29,
        maxObservedDeltaDeg=.9,upPercent=50,upDurationCapMs=350,downPercent=35,downDurationCapMs=200,
        upMonitorTimingPolicy=UP_MONITOR_POLICY,downMonitorTimingPolicy=DOWN_MONITOR_POLICY)


def validate_current_short_policy(policy):
    expected=current_short_policy()
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


def short_task_pulse(direction):
    require(direction in ('up','down'),'current_short_direction_required')
    return dict(joint='shoulder',motor=4,direction=direction,percent=50 if direction=='up' else 35,
                durationMs=350 if direction=='up' else 200,expectedPwm=127 if direction=='up' else -89)
