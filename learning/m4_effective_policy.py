"""기존 출력 상한과 목표각을 유지하는 명시1200ms goal-capped M4 정책.

긴 요청 cap은 램프 시동을 능동 관측 구간에 두기 위한 것이다. 초기 램프나
실제 정지거리·목표 도달을 보장하지 않으며 기존 short 자료를 이전하지 않는다.
"""
import copy
from learning.manual_collection_dataset import require
SCHEMA='motionbrain.m4_current_effective_task.v1'
POLICY_SCHEMA='motionbrain.m4_current_effective_paired_policy.v1'
UP_MONITOR_POLICY='m4-effective-up-deadline-1200-v1'
DOWN_MONITOR_POLICY='m4-effective-down-deadline-1200-v1'
def effective_policy():
 return dict(schemaVersion=POLICY_SCHEMA,tableReferenceDeg=197.22,upGoalAngleDeg=197.85,
  downGoalAngleDeg=197.22,originalGlobalReferenceDeg=195.29,originalGlobalUpperDeg=198.29,
  maxObservedDeltaDeg=.9,upPercent=50,upDurationCapMs=1200,downPercent=35,downDurationCapMs=1200,
  upMonitorTimingPolicy=UP_MONITOR_POLICY,downMonitorTimingPolicy=DOWN_MONITOR_POLICY,
  observationAcquisition='after_ARMED_all_OFF_before_RUN_v1')
def validate_effective_policy(policy):
 expected=effective_policy()
 require(isinstance(policy,dict) and set(policy)==set(expected)
  and all(type(policy[k]) is type(v) and policy[k]==v for k,v in expected.items()),'effective_exact_policy_required')
 return copy.deepcopy(policy)
def effective_task_pulse(direction,policy=None):
 validate_effective_policy(effective_policy() if policy is None else policy)
 require(direction in ('up','down'),'effective_direction_required')
 return dict(joint='shoulder',motor=4,direction=direction,percent=50 if direction=='up' else 35,
  durationMs=1200,expectedPwm=127 if direction=='up' else -89)
