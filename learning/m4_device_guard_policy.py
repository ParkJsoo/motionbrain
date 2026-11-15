"""장치 내부 장착 보정 AS5600 각도 목표/OFF용 새 코호트. 실물 검증 전 기본 비활성."""
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


def guard_request(scope):
 """소모되는 actor scope에 nonce를 고정한다. 재호출용 새 ID를 만들지 않는다."""
 from learning.supervised_collection import plan_digest
 from learning.m4_device_guard_transport import validate_request
 require(scope.get('schemaVersion')==SCHEMA,'device_guard_scope_required')
 validate_device_guard_policy(scope['deviceGuardPolicy'])
 scope_sha=plan_digest(scope)
 return validate_request(dict(nonce=scope_sha[:32],bootId=scope['controllerBootId'],
  scopeSha256=scope_sha,policySha256=POLICY_SHA256,
  expectedStartDeg=scope['expectedStartAngleDeg'],goalDeg=scope['goalAngleDeg']),scope['pulse'])
