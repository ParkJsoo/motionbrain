"""장치 내부 목표 정지의 명시 wire 계약. 기존 호스트 감시를 대체하지 않는다.

이 모듈은 장치 발견/실행을 하지 않는다. 새 guarded scope에서만 호출하며,
단 한 번의 RUN 이후 HTTP가 정지된 동안 메모리 tick만 수행한다.
현재 펌웨어 HTTP/mainloop 어댑터는 미구현이다. 아래 capability는 설계된
계약이며 실제 지원이 없으면 첫 검사에서 거절한다. actor/raw/모델 연결과
장치 실측을 완료한 실행 경로로 사용해서는 안 된다.
"""
from __future__ import annotations
import copy
import math
import re

CAPABILITY_SCHEMA = 'motionbrain.m4_goal_guard.v1'
POLICY_SHA256 = 'ee16b5b5f8cbc1df7d4927dd4139b7f8b767927eb39c72de5fb843d6494d09f9'
TERMINAL_TIMEOUT_SECONDS = 2.


def _require(ok, code):
    if not ok:
        raise ValueError('m4_device_guard_' + code)


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def validate_request(request, pulse):
    _require(isinstance(request, dict) and set(request) == {
        'nonce', 'bootId', 'scopeSha256', 'policySha256', 'expectedStartDeg', 'goalDeg'}, 'request_fields')
    for key, size in (('nonce',32), ('bootId',32), ('scopeSha256',64)):
        _require(isinstance(request[key],str) and re.fullmatch('[0-9a-f]{'+str(size)+'}',request[key]), 'request_identity')
    _require(request['policySha256'] == POLICY_SHA256, 'policy_changed')
    up = pulse.get('direction') == 'up'
    _require(pulse == dict(joint='shoulder', motor=4, direction='up' if up else 'down',
        percent=50 if up else 35, durationMs=1200, expectedPwm=127 if up else -89), 'pulse')
    start,goal=request['expectedStartDeg'],request['goalDeg']
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
    if state is None:
        _require(value['nonceCount'] < 32, 'nonce_capacity')
        return copy.deepcopy(value)
    r=value.get('record')
    _require(isinstance(r,dict), 'record_required')
    exact=dict(nonce=request['nonce'],bootId=request['bootId'],scopeSha256=request['scopeSha256'],
        policySha256=request['policySha256'],expectedStartDeg=request['expectedStartDeg'],
        goalDeg=request['goalDeg'],up=pulse['direction']=='up',percent=pulse['percent'],durationMs=1200,state=state)
    _require(all(type(r.get(k)) is type(v) and r[k] == v if isinstance(v,(str,bool,int))
        else _number(r.get(k)) and abs(r[k]-v)<=.0001 for k,v in exact.items()), 'record_changed')
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
        _require(_number(start) and _number(last) and abs(start-request['expectedStartDeg'])<=.1001
            and 192.29<=last<=198.29 and abs(last-start)<=.9001
            and (last>=start-.1001 if r['up'] else last<=start+.1001), 'terminal_angle')
        _require(r['reason']!='deadline' or elapsed('decisionAtMs','runAtMs')>=1200, 'terminal_deadline')
        _require(r['reason']!='goal' or (last>=request['goalDeg'] if r['up'] else last<=request['goalDeg']), 'terminal_goal')
    return copy.deepcopy(r)


def add_payload(request,pulse):
    validate_request(request,pulse)
    return dict(action='add_m4_guarded',joint='shoulder',direction=pulse['direction'],speed=pulse['percent'],
        duration=1200,guardNonce=request['nonce'],guardBootId=request['bootId'],
        guardScopeSha256=request['scopeSha256'],guardPolicySha256=request['policySha256'],
        expectedStartDeg=request['expectedStartDeg'],goalDeg=request['goalDeg'])


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
