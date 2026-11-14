"""장치 내부 raw AS5600 목표/OFF용 새 코호트. 실물 검증 전 기본 비활성."""
import copy
from learning.manual_collection_dataset import require
from learning.m4_device_guard_transport import POLICY_SHA256
SCHEMA='motionbrain.m4_current_device_guard_task.v1'
POLICY_SCHEMA='motionbrain.m4_current_device_guard_paired_policy.v1'
def device_guard_policy():
 from learning.m4_effective_policy import effective_policy
 p=effective_policy();p.update(schemaVersion=POLICY_SCHEMA,
  upMonitorTimingPolicy='m4-device-goal-terminal-up-v1',downMonitorTimingPolicy='m4-device-goal-terminal-down-v1',
  deviceGuardPolicySha256=POLICY_SHA256)
 return p
def validate_device_guard_policy(policy):
 expected=device_guard_policy()
 require(isinstance(policy,dict) and set(policy)==set(expected) and all(
  type(policy[k]) is type(v) and policy[k]==v for k,v in expected.items()),'device_guard_exact_policy_required')
 return copy.deepcopy(policy)
def device_guard_task_pulse(direction,policy=None):
 validate_device_guard_policy(device_guard_policy() if policy is None else policy)
 require(direction in ('up','down'),'device_guard_direction_required')
 return dict(joint='shoulder',motor=4,direction=direction,percent=50 if direction=='up' else 35,
  durationMs=1200,expectedPwm=127 if direction=='up' else -89)
