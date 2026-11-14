"""현재 코호트의 수동 보강과 기존 유한 모델 과업을 잇는 명시적 어댑터.

장치 주소·기본 관측자·자동 승인·모델 이전은 제공하지 않는다. 수집 포트는
기존 관절별 actor의 ARM 직후 새 관측과 RUN 직전 검사를 그대로 연결한다.
"""
from __future__ import annotations

import copy
import time

from learning.finite_task_cycle import FiniteTaskCycle, STAGES, bounded_command, finite, sha
from learning.manual_collection import ManualCollection
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest, identifier
from learning.supervised_transport import ELBOW_DEADLINE_POLICY
from learning.task_cycle_workflow import FrozenCycleModelBank

CATEGORIES = ('preparation', 'M1', 'M3', 'M4_OPEN')
SCHEMA = 'motionbrain.current_task_adapter.v1'


class ManualCollectionCyclePort:
    """이미 시작한 ManualCollection에 기존 모델 실행 훅을 연결한다.

    manager_for(command)는 현재 촬영 중인 소유 세션만 반환해야 한다. 세션 시작과
    원본 마감은 호출자의 단발 actor가 소유한다. M4는 ScopedCycleExecutor로 보낸다.
    """
    def __init__(self, manager_for):
        require(callable(manager_for), 'current_manager_provider_required')
        self.manager_for = manager_for

    def check_armed(self, request):
        """ARM 후 큐 1개·전축 OFF를 기존 transport GET 감사로 입증한다."""
        manager = self.manager_for(copy.deepcopy(request['command']))
        require(isinstance(manager, ManualCollection) and manager.port is not None,
                'current_recording_owner_required')
        port = manager.port
        actions = port._call('/actions')
        require(actions.get('bootId') == port.boot_id and actions.get('active') is False,
                'current_armed_boot_or_action_changed')
        status = port._status()
        port._check_status(status, 'ARMED')
        outputs, _ = port._outputs()
        require(port.last_outputs == {i: 0 for i in range(1, 6)}, 'current_armed_driver_not_off')
        queue = port._queue(1)
        proof = dict(actions=actions,status=status,outputs=outputs,queue=queue)
        return dict(bootId=port.boot_id,state='ARMED',queueCount=1,allFiveOff=True,
                    evidenceSha256=plan_digest(proof))

    def __call__(self, command, *, after_armed_before_run, before_run, decision):
        require(command.get('joint') in ('elbow', 'gripper'), 'current_general_port_joint')
        require(callable(after_armed_before_run) and callable(before_run), 'current_run_guards_required')
        manager = self.manager_for(copy.deepcopy(command))
        require(isinstance(manager, ManualCollection) and manager.current is not None
                and manager.current['phase'] == 'recording', 'current_recording_owner_required')
        before = manager.command_count
        policy = ELBOW_DEADLINE_POLICY if command['joint'] == 'elbow' else None
        manager.pulse(copy.deepcopy(command), after_armed_before_run=after_armed_before_run,
                      before_run=before_run, monitor_policy=policy)
        require(manager.command_count == before+1, 'current_command_count_changed')
        results = [e for e in manager.current['events'] if e.get('kind') == 'result']
        require(bool(results), 'current_command_receipt_missing')
        receipt = copy.deepcopy(results[-1]['receipt'])
        require(receipt.get('offVerified') is True, 'current_command_off_unverified')
        return receipt


class CurrentTaskAdapter:
    """명령 선택과 실물 안전 판단을 주입하며 단발 전체 세션 분모를 보존한다.

    inspect(request)는 두 카메라/센서 현재 근거를 반환한다. audit_entry는 원본과
    등록·외부 여유·자세 계약을 재감사한다. collect(request, prepare, before_run)는
    ARM 뒤 prepare를 정확히 한 번, RUN 직전 before_run을 정확히 한 번 호출한다.
    check_armed(request)는 현재 ARMED/큐1/전축OFF의 GET 근거를 반환한다.
    verify_assets의 파일 I/O는 입구·prepare에서만 수행한다. RUN 직전은 메모리만 검사한다.
    audit_row는 수집 완료/OFF 원본을 다시 읽고 행을 생성한다. freeze(rows)는
    현재 코호트 모델을 고정하고 make_cycle은 기존 FiniteTaskCycle을 만든다.
    """
    def __init__(self, scope, requests, *, inspect, audit_entry, collect, audit_row,
                 freeze, make_cycle, verify_assets, check_idle, check_armed, store, clock=time):
        require(isinstance(scope, dict) and set(scope) == {
            'schemaVersion','sessionId','controllerBootId','cameraBootId','object',
            'M4ReferenceDeg','maxObservationAgeSeconds','budgets','maxRequestedMs','contexts'},
            'current_scope_fields')
        require(scope['schemaVersion'] == SCHEMA and identifier(scope['sessionId'])
                and isinstance(scope['object'], str) and bool(scope['object']), 'current_scope_identity')
        for key in ('controllerBootId','cameraBootId'):
            require(isinstance(scope[key], str) and len(scope[key]) == 32
                    and all(c in '0123456789abcdef' for c in scope[key]), 'current_boot_identity')
        require(finite(scope['M4ReferenceDeg']) and 127.58 < scope['M4ReferenceDeg'] < 296.12
                and finite(scope['maxObservationAgeSeconds'])
                and .1 <= scope['maxObservationAgeSeconds'] <= 3., 'current_observation_limits')
        require(isinstance(scope['budgets'], dict) and set(scope['budgets']) == set(CATEGORIES)
                and all(type(v) is int and 0 <= v <= 12 for v in scope['budgets'].values())
                and 1 <= sum(scope['budgets'].values()) <= 32
                and type(scope['maxRequestedMs']) is int and 1 <= scope['maxRequestedMs'] <= 20800,
                'current_collection_budget')
        require(isinstance(scope['contexts'], dict) and set(scope['contexts']) == set(STAGES)
                and all(isinstance(c, dict) and c.get('object') == scope['object']
                        for c in scope['contexts'].values()), 'current_context_object')
        require(isinstance(requests, list) and 1 <= len(requests) <= sum(scope['budgets'].values()),
                'current_requests_required')
        counts = dict.fromkeys(CATEGORIES, 0)
        for request in requests:
            require(isinstance(request, dict) and set(request) == {'id','category','stage','command'}
                    and identifier(request['id']) and request['category'] in CATEGORIES,
                    'current_request_fields')
            bounded_command(request['stage'], request['command'])
            category = request['category']; stage = request['stage']
            require(category == 'preparation' or category == 'M1' and stage in ('close','release')
                    or category == 'M3' and stage == 'approach'
                    or category == 'M4_OPEN' and stage in ('lift','lower','release'),
                    'current_request_category')
            counts[category] += 1
        require(len({r['id'] for r in requests}) == len(requests)
                and all(counts[k] <= scope['budgets'][k] for k in counts)
                and sum(r['command']['durationMs'] for r in requests) <= scope['maxRequestedMs'],
                'current_request_budget')
        for callback in (inspect,audit_entry,collect,audit_row,freeze,make_cycle,verify_assets,check_idle,check_armed):
            require(callable(callback), 'current_callback_required')
        require(callable(getattr(store,'write_json',None)), 'current_store_required')
        self.scope,self.requests = copy.deepcopy(scope),copy.deepcopy(requests)
        self.inspect,self.audit_entry,self.collect,self.audit_row = inspect,audit_entry,collect,audit_row
        self.freeze,self.make_cycle,self.verify_assets,self.check_idle = freeze,make_cycle,verify_assets,check_idle
        self.check_armed=check_armed
        self.store,self.clock = store,clock
        self.digest = plan_digest(dict(scope=self.scope,requests=self.requests)); self.used=False; self.seen=set()
        self._asset_revision=0; self._asset_proof=None

    def _memory(self):
        require(plan_digest(dict(scope=self.scope,requests=self.requests)) == self.digest, 'current_scope_changed')

    def _assets(self):
        self._memory()
        self._asset_proof=plan_digest(self.verify_assets())
        self._asset_revision+=1

    def _entry(self, request, *, armed=False):
        self._assets()
        if armed:
            proof = self.check_armed(copy.deepcopy(request))
            require(isinstance(proof,dict) and proof.get('bootId') == self.scope['controllerBootId']
                    and proof.get('state') == 'ARMED' and type(proof.get('queueCount')) is int
                    and proof['queueCount'] == 1 and proof.get('allFiveOff') is True
                    and sha(proof.get('evidenceSha256')), 'current_armed_or_off_changed')
        else:
            require(self.check_idle() == self.scope['controllerBootId'], 'current_boot_or_off_changed')
        record = self.inspect(copy.deepcopy(request))
        require(isinstance(record,dict) and record.get('controllerBootId') == self.scope['controllerBootId']
                and record.get('cameraBootId') == self.scope['cameraBootId']
                and record.get('context') == self.scope['contexts'][request['stage']],
                'current_entry_contract')
        angle=record.get('m4AngleDeg')
        maximum = 3. if request['stage'] in ('lift','lower') else .25
        require(finite(angle) and abs(angle-self.scope['M4ReferenceDeg']) <= maximum,
                'current_entry_M4_pose')
        frames=record.get('frames')
        require(isinstance(frames,list) and len(frames)==2
                and {f.get('camera') for f in frames} == {'external','gripper'}, 'current_two_cameras')
        now=self.clock.monotonic(); ids=[]
        for frame in frames:
            times=[frame.get(k) for k in ('sourceEarliest','sourceLatest','persistedAt')]
            require(isinstance(frame.get('frameId'),str) and bool(frame['frameId'])
                    and sha(frame.get('pixelSha256')) and all(finite(t) for t in times)
                    and times==sorted(times) and self.started <= times[0] <= times[-1] <= now
                    and now-times[0] <= self.scope['maxObservationAgeSeconds'], 'current_frame_stale')
            ids.append(frame['frameId'])
        require(len(set(ids))==2 and not self.seen.intersection(ids), 'current_frame_reused')
        require(self.audit_entry(copy.deepcopy(request),copy.deepcopy(record)) is True,
                'current_registration_or_external_guard_rejected')
        # Audit I/O cannot spend the freshness budget silently.
        require(all(self.clock.monotonic()-f['sourceEarliest'] <= self.scope['maxObservationAgeSeconds']
                    for f in frames), 'current_entry_audit_stale')
        self.seen.update(ids);self._assets()
        return copy.deepcopy(record)

    def run(self):
        require(not self.used, 'current_session_single_use'); self.used=True
        self.started=self.clock.monotonic()
        report=dict(status='failed',collectionAttempts=dict.fromkeys(CATEGORIES,0),
                    collectionCompleted=dict.fromkeys(CATEGORIES,0),requestedMs=0,
                    modelCycleAttempts=0,modelCycleResult=None,errors=[],stoppedAt=None)
        rows=[]
        try:
            self.store.write_json('current-session-consumed.json',dict(scope=self.scope,requests=self.requests))
            for index,request in enumerate(self.requests,1):
                report['stoppedAt']=request['id']; prepared=[]; called=[]; prepared_assets=[]
                # Input eligibility is assessed before consuming a motion attempt.
                entry=self._entry(request)
                def prepare():
                    require(not prepared, 'current_prepare_repeated')
                    prepared.append(self._entry(request,armed=True))
                    prepared_assets.append((self._asset_revision,self._asset_proof))
                def before_run():
                    require(len(prepared)==1 and not called, 'current_prepare_or_dispatch_count')
                    called.append(True);self._memory()
                    require(prepared_assets == [(self._asset_revision,self._asset_proof)],
                            'current_cached_asset_proof_changed')
                    require(all(self.clock.monotonic()-f['sourceEarliest'] <= self.scope['maxObservationAgeSeconds']
                                for f in prepared[0]['frames']), 'current_run_observation_stale')
                self.store.write_json(f'current-{index:03d}-intent.json',dict(request=request,entry=entry))
                report['collectionAttempts'][request['category']]+=1
                report['requestedMs']+=request['command']['durationMs']
                result=self.collect(copy.deepcopy(request),prepare=prepare,before_run=before_run)
                self.store.write_json(f'current-{index:03d}-result.json',result)
                require(len(prepared)==1 and len(called)==1 and result.get('status')=='completed'
                        and result.get('offVerified') is True, 'current_collection_failed_no_retry')
                row=self.audit_row(copy.deepcopy(request),copy.deepcopy(result))
                require(isinstance(row,dict) and row.get('command')==request['command']
                        and row.get('context')==self.scope['contexts'][request['stage']],
                        'current_audited_row_contract')
                if request['category']!='preparation': rows.append(row)
                report['collectionCompleted'][request['category']]+=1
            self._assets();report['stoppedAt']='freeze'
            bank=self.freeze(copy.deepcopy(rows))
            require(isinstance(bank,FrozenCycleModelBank)
                    and all(m['context']==self.scope['contexts'][kind] for kind,m in bank.models.items()),
                    'current_frozen_bank_contract')
            cycle=self.make_cycle(bank)
            require(isinstance(cycle,FiniteTaskCycle)
                    and cycle.scope['controllerBootId']==self.scope['controllerBootId']
                    and all(stage['context']==self.scope['contexts'][stage['kind']]
                            and stage['modelSha256']==bank.hashes[stage['kind']]
                            for stage in cycle.scope['stages'])
                    and cycle.choose==bank.choose, 'current_finite_cycle_required')
            report['modelCycleAttempts']=1;report['stoppedAt']='model_cycle'
            report['modelCycleResult']=cycle.run();report['status']=report['modelCycleResult']['status']
        except (Exception,KeyboardInterrupt) as exc:
            report['errors'].append(dict(type=type(exc).__name__,reason=str(exc) if isinstance(exc,ValueError)
                                         else 'current_session_failed'))
        self.store.write_json('current-session-result.json',report)
        return report
