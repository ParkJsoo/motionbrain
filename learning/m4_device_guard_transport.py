"""장치 내부 목표 정지의 명시 wire 계약. 기존 호스트 감시를 대체하지 않는다.

이 모듈은 장치 발견/실행을 하지 않는다. 새 guarded scope에서만 호출하며,
단 한 번의 RUN 이후 HTTP가 정지된 동안 메모리 tick만 수행한다.
전용 펌웨어의 HTTP/mainloop 어댑터와 같은 capability 계약을 검사하며
실제 지원이 없으면 첫 검사에서 거절한다. actor/raw/모델 연결의 소프트웨어
검증과 펌웨어 배포·장치 실측은 별개다. 기본 펌웨어는 비활성이다.
"""
from __future__ import annotations
import copy
import math
import re

CAPABILITY_SCHEMA = 'motionbrain.m4_goal_guard.v1'
POLICY_SHA256 = 'ee16b5b5f8cbc1df7d4927dd4139b7f8b767927eb39c72de5fb843d6494d09f9'
CURRENT_POLICY_CONTRACT = 'motionbrain.m4_goal_guard.current_table.v2|global=192.29:198.29|local=.9|up=50:1200:table+.63|down=35:1200:table|table_headroom=.9|down_span=.1:.9|sensor_age=60|tick_gap=40|start_tolerance=.1'
CURRENT_POLICY_SHA256 = '77d5ce74eacb65cb37ee2875a611e32489894b981c0aaf0b41442cf617b8f87a'
TERMINAL_TIMEOUT_SECONDS = 2.


def _require(ok, code):
    if not ok:
        raise ValueError('m4_device_guard_' + code)


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_request(request, pulse):
    current = isinstance(request,dict) and request.get('policySha256') == CURRENT_POLICY_SHA256
    _require(isinstance(request, dict) and set(request) == {
        'nonce', 'bootId', 'scopeSha256', 'policySha256', 'expectedStartDeg', 'goalDeg'} | ({'tableReferenceDeg'} if current else set()), 'request_fields')
    for key, size in (('nonce',32), ('bootId',32), ('scopeSha256',64)):
        _require(isinstance(request[key],str) and re.fullmatch('[0-9a-f]{'+str(size)+'}',request[key]), 'request_identity')
    _require(request['policySha256'] in (POLICY_SHA256,CURRENT_POLICY_SHA256), 'policy_changed')
    up = pulse.get('direction') == 'up'
    _require(pulse == dict(joint='shoulder', motor=4, direction='up' if up else 'down',
        percent=50 if up else 35, durationMs=1200, expectedPwm=127 if up else -89), 'pulse')
    start,goal=request['expectedStartDeg'],request['goalDeg']
    if current:
        table=request['tableReferenceDeg']
        _require(_number(table) and 192.29 <= table <= 197.39
            and _number(start) and _number(goal) and 192.29 <= start <= 198.29
            and abs(goal-(table+.63 if up else table)) <= .00001
            and (start == table if up else table+.1 < start <= table+.9), 'angles')
    else:
        _require(_number(start) and _number(goal) and goal == (197.85 if up else 197.22)
            and 192.29 <= start <= 198.29 and (start == 197.22 if up else start > 197.32), 'angles')
    return copy.deepcopy(request)


def capability(response, request, *, state=None, pulse=None):
    value=response.get('m4GoalGuard') if isinstance(response,dict) else None
    _require(isinstance(value,dict) and value.get('schemaVersion') == CAPABILITY_SCHEMA
        and value.get('enabled') is True and value.get('policySha256') == POLICY_SHA256
        and value.get('bootId') == request['bootId']
        and value.get('execution') == 'main_loop_http_deferred'
        and value.get('nonceCapacity') == 32 and type(value.get('nonceCount')) is int
        and 0 <= value['nonceCount'] <= 32, 'capability_required')
    _require(request['policySha256'] == POLICY_SHA256 or
        (request['policySha256'] == CURRENT_POLICY_SHA256
         and isinstance(value.get('supportedPolicySha256'),list)
         and CURRENT_POLICY_SHA256 in value['supportedPolicySha256']), 'policy_capability_required')
    if state is None:
        _require(value['nonceCount'] < 32, 'nonce_capacity')
        return copy.deepcopy(value)
    r=value.get('record')
    _require(isinstance(r,dict), 'record_required')
    current=request['policySha256']==CURRENT_POLICY_SHA256
    exact=dict(nonce=request['nonce'],bootId=request['bootId'],scopeSha256=request['scopeSha256'],
        policySha256=request['policySha256'],expectedStartDeg=request['expectedStartDeg'],
        goalDeg=request['goalDeg'],up=pulse['direction']=='up',percent=pulse['percent'],durationMs=1200,state=state)
    if request['policySha256']==CURRENT_POLICY_SHA256:exact['tableReferenceDeg']=request['tableReferenceDeg']
    angle_keys={'expectedStartDeg','goalDeg','tableReferenceDeg'}
    _require(all((_number(r.get(k)) and abs(r[k]-v)<=.0001) if k in angle_keys
        else type(r.get(k)) is type(v) and r[k]==v for k,v in exact.items()), 'record_changed')
    if state == 'terminal':
        _require(r.get('reason') in ('goal','deadline') and r.get('offVerified') is True
            and r.get('outputMayHaveOccurred') is True, 'terminal_failure')
        fields=('queuedAtMs','runAtMs','decisionAtMs','offAtMs','sampleAtMs','sampleAgeMs','maxTickGapMs','firstTickDelayMs')
        _require(all(type(r.get(k)) is int and 0<=r[k]<=0xffffffff for k in fields), 'terminal_time')
        elapsed=lambda a,b:(r[a]-r[b]) & 0xffffffff
        _require(elapsed('decisionAtMs','runAtMs')<=1240
            and elapsed('offAtMs','decisionAtMs')<=5
            and elapsed('decisionAtMs','sampleAtMs')==r['sampleAgeMs']
            and r['sampleAgeMs']<=60 and r['maxTickGapMs']<=40 and r['firstTickDelayMs']<=200, 'terminal_timing')
        start,last=r.get('startAngleDeg'),r.get('lastAngleDeg')
        boundary_epsilon=.00005 if current else 0.
        _require(_number(start) and _number(last) and abs(start-request['expectedStartDeg'])<=.1001
            and 192.29-boundary_epsilon<=last<=198.29+boundary_epsilon and abs(last-start)<=.9001
            and (last>=start-.1001 if r['up'] else last<=start+.1001), 'terminal_angle')
        _require(r['reason']!='deadline' or elapsed('decisionAtMs','runAtMs')>=1200, 'terminal_deadline')
        # The device compares float32 values, and serializes five decimal places.
        # Its goal is already bound to the requested goal within 0.0001 degree.
        goal=r['goalDeg'] if current else request['goalDeg']
        _require(r['reason']!='goal' or (last>=goal if r['up'] else last<=goal), 'terminal_goal')
    return copy.deepcopy(r)


def add_payload(request,pulse):
    validate_request(request,pulse)
    payload=dict(action='add_m4_guarded',joint='shoulder',direction=pulse['direction'],speed=pulse['percent'],
        duration=1200,guardNonce=request['nonce'],guardBootId=request['bootId'],
        guardScopeSha256=request['scopeSha256'],guardPolicySha256=request['policySha256'],
        expectedStartDeg=request['expectedStartDeg'],goalDeg=request['goalDeg'])
    if request['policySha256']==CURRENT_POLICY_SHA256:payload['tableReferenceDeg']=request['tableReferenceDeg']
    return payload


def run_payload(request):
    return dict(action='run_m4_guarded',guardNonce=request['nonce'],guardBootId=request['bootId'],
        guardScopeSha256=request['scopeSha256'],guardPolicySha256=request['policySha256'])


def await_terminal(port, request, pulse, tick):
    # HTTP is intentionally deferred in the device during guarded motion. Do not
    # use the old active GET monitor, nor retry a lost RUN reply.
    end=port.last_dispatch_after+1.2
    while port.clock.monotonic()<end:
        tick()
        port.clock.sleep(max(0.,min(.025,end-port.clock.monotonic())))
    tick()
    response=port._call('/sequence',request_timeout=TERMINAL_TIMEOUT_SECONDS,device_guard_terminal=True)
    record=capability(response,request,state='terminal',pulse=pulse)
    port._save('m4-device-guard-terminal.json',dict(request=copy.deepcopy(request),record=record))
    return record
