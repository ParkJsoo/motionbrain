"""실패한 M4 RUN의 재부팅 후 현재 OFF 해소. 원실패·정답·재송 금지 보존.

호출 자체는 원장 변경이므로 해당 원본과 새 부팅에 대한 명시 실행 허용이
필요하다. import/원본 감사에는 장치 접근이 없다. 촬영·학습·모터 쓰기 없음.
"""
from __future__ import annotations
import copy
import fcntl
import hashlib
from pathlib import Path
import re
import sqlite3
import time

from learning import action_journal
from learning import m4_up_commissioning as core
from learning import m4_device_guard_transport as guard
from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort, response_json, validate_target

PHASE = 'resolved_current_off_after_failed_m4_run'
ATTEMPT = 'readonly-failed-m4-reboot-recovery'
# 과거 실패 자료 감사에만 허용한다. 이 구현으로 새 RUN을 허용하지 않는다.
HISTORICAL_EFFECTIVE_DIGEST = 'd48218d0997af084ce6bf285c066950d94496e0c31d4a1f9ae1b3d600baa7dea'


def _identity(value, size=32):
    require(isinstance(value,str) and re.fullmatch('[0-9a-f]{'+str(size)+'}',value) is not None,
            'm4_recovery_identity')
    return value


def audit_failed_m4_source(source_root, *, result_sha256, scope_sha256, original_boot_id, original_controller_url):
    """원단일 RUN/실패를 읽기 전용 검증한다. 학습 적격 감사와 별개다."""
    root=Path(source_root).resolve();hashes={}
    def read(name):
        raw=_read(root,name);hashes[name]=hashlib.sha256(raw).hexdigest()
        return _decode_json(raw)
    _identity(result_sha256,64);_identity(scope_sha256,64);_identity(original_boot_id)
    original_controller_url=validate_target(original_controller_url)
    scope=read('scope.json');result=read('result.json')
    require(hashes['result.json']==result_sha256 and hashes['scope.json']==scope_sha256,'m4_recovery_source_changed')
    require(scope.get('schemaVersion')==core.EFFECTIVE_TASK_SCHEMA
            and scope.get('implementationSha256') in (HISTORICAL_EFFECTIVE_DIGEST,core.implementation_digest()),
            'm4_recovery_known_effective_scope_required')
    candidate=copy.deepcopy(scope);candidate['implementationSha256']=core.implementation_digest();core.validate_scope(candidate)
    require(scope['controllerBootId']==original_boot_id and scope['controllerUrl']==original_controller_url
            and scope['pulse']['direction']=='down','m4_recovery_original_binding')
    require(result.get('schemaVersion')==scope['schemaVersion'] and result.get('scopeSha256')==plan_digest(scope)
            and result.get('status')=='failed' and result.get('commandOutcome')=='unknown'
            and result.get('mayHaveRun') is True and result.get('finalIdleOffVerified') is True
            and result.get('trainingEligible') is False and result.get('modelSelected') is False
            and result.get('parentModelSelected',False) is False and 'receipt' not in result
            and isinstance(result.get('errors'),list) and bool(result['errors']), 'm4_recovery_failed_unknown_required')
    posts=[];boots=set();run_count=0;previous=None
    paths=sorted(root.glob('http-*-intent.json'))
    require(bool(paths),'m4_recovery_raw_HTTP_required')
    for i,path in enumerate(paths,1):
        stem=f'http-{i:06d}'
        require(path.name==stem+'-intent.json','m4_recovery_HTTP_gap')
        intent=read(stem+'-intent.json');response=read(stem+'-response.json')
        endpoint=intent['path'];payload=intent['payload']
        require(intent.get('method')==('GET' if payload is None else 'POST'),'m4_recovery_HTTP_method')
        parsed=response_json(response['rawUtf8'],endpoint)
        require(response.get('httpStatus')==200 and parsed==response.get('parsed'),'m4_recovery_HTTP_response')
        started,ended=intent['startedAt'],response['receivedAt']
        require(core.transport.finite_number(started) and core.transport.finite_number(ended)
                and started<=ended and (previous is None or previous<=started),'m4_recovery_HTTP_time')
        previous=ended
        if payload is not None:
            require(parsed.get('success') is True,'m4_recovery_original_command_rejected')
            posts.append((endpoint,payload))
        else:
            require(endpoint.split('?')[0] in ('/actions','/status','/collection_status','/sequence','/action_outputs'),
                    'm4_recovery_unexpected_read')
        if endpoint=='/actions':boots.add(parsed.get('bootId'))
        if endpoint.split('?')[0]=='/action_outputs':
            require(parsed.get('bootId')==original_boot_id,'m4_recovery_output_boot_changed')
        if endpoint=='/sequence' and payload=={'action':'run'}:run_count+=1
    pulse=scope['pulse']
    expected=[('/motor',dict(action='default',speed=255)),('/sequence',dict(action='add',joint='shoulder',
        direction='down',speed=pulse['percent'],duration=pulse['durationMs'])),('/command',dict(cmd='arm')),
        ('/sequence',dict(action='run'))]
    cleanup=[('/command',dict(cmd='stop')),('/sequence',dict(action='clear')),('/motor',dict(action='default',speed=100))]
    require(boots=={original_boot_id} and run_count==1 and posts==expected+cleanup+cleanup,
            'm4_recovery_single_original_RUN_required')
    # 원본 최상위 파일도 묶는다. 새 HTTP 기록/실행 파일 추가 역시 사후 변경이다.
    for path in sorted(root.iterdir()):
        if path.is_file():hashes[path.name]=hashlib.sha256(_read(root,path.name)).hexdigest()
    return dict(scope=scope,result=result,hashes=hashes,sourceRoot=str(root),scopeDigest=plan_digest(scope))


class _ReadOnlyPort(SequencePort):
    def _call(self,path,payload=None,**kwargs):
        require(payload is None and not kwargs.get('run') and not kwargs.get('emergency')
                and path.split('?')[0] in ('/actions','/status','/collection_status','/sequence','/action_outputs'),
                'm4_recovery_GET_only')
        return super()._call(path,payload,**kwargs)


def prepare_failed_m4_reboot_recovery(*, repo_root, source_root, result_sha256, scope_sha256,
        original_boot_id, current_boot_id, original_controller_url, current_controller_url):
    """기존 원본·원장만 읽어 승인할 정확한 제안을 만든다. 장치/파일 쓰기 없음."""
    repo=Path(repo_root).resolve();root=Path(source_root).resolve()
    require(root.is_relative_to(repo/'.local') and not Path(source_root).is_symlink(),'m4_recovery_local_source_required')
    _identity(current_boot_id);require(current_boot_id!=original_boot_id,'m4_recovery_distinct_boots_required')
    current_controller_url=validate_target(current_controller_url)
    audit=audit_failed_m4_source(root,result_sha256=result_sha256,scope_sha256=scope_sha256,
        original_boot_id=original_boot_id,original_controller_url=original_controller_url)
    path=repo/'.local/visual-actions/controller-commands.sqlite'
    require(path.is_file() and not path.is_symlink() and Path(str(path)+'.lock').is_file()
            and not Path(str(path)+'.lock').is_symlink(),'m4_recovery_existing_shared_journal_required')
    with Path(str(path)+'.lock').open('rb') as lock:
        fcntl.flock(lock,fcntl.LOCK_SH|fcntl.LOCK_NB)
        db=sqlite3.connect(path.as_uri()+'?mode=ro',uri=True)
        try:
            db.execute('BEGIN')
            row=db.execute('SELECT plan_hash,output_path,phase,result_json FROM supervised_blocks WHERE session_id=?',
                           (audit['scope']['trialId'],)).fetchone()
            require(row is not None and row[:3]==(audit['scopeDigest'],str(root),'pending') and row[3] is not None
                    and _decode_json(row[3].encode())==audit['result'],'m4_recovery_owned_block_required')
            require(db.execute("SELECT session_id FROM supervised_blocks WHERE phase IN ('pending','off_unverified')").fetchall()==[(audit['scope']['trialId'],)],
                    'm4_recovery_other_block_pending')
            rows=[action_journal.M4CommandJournal._decode(r) for r in db.execute('SELECT * FROM commands ORDER BY rowid')]
            require(action_journal.readiness(rows,action_journal._resolutions(db))['ready'],'m4_recovery_other_command_pending')
            return dict(schemaVersion='motionbrain.failed_m4_reboot_recovery_proposal.v1',repoRoot=str(repo),
                sourceRoot=str(root),originalSourceHashes=audit['hashes'],sessionId=audit['scope']['trialId'],
                originalScopeSha256=audit['scopeDigest'],originalResultJsonSha256=hashlib.sha256(row[3].encode()).hexdigest(),
                originalControllerBootId=original_boot_id,currentControllerBootId=current_boot_id,
                originalControllerUrl=original_controller_url,currentControllerUrl=current_controller_url,
                journalPath=str(path),fromPhase='pending',toPhase=PHASE,automaticRetry=False,
                controllerWritesAllowed=False,originalOutcomeChanged=False,datasetEligible=False,replayAllowed=False)
        finally:db.close()


def recover_failed_m4_after_reboot_readonly(*, repo_root, source_root, result_sha256, scope_sha256,
        original_boot_id, current_boot_id, original_controller_url, current_controller_url, token, confirmed_proposal_sha256, clock=time):
    """승인된 한 원장 행의 현재 OFF만 해소한다. 실패한 복구 시도도 재시도 불가."""
    repo=Path(repo_root).resolve();root=Path(source_root).resolve()
    require(root.is_relative_to(repo/'.local') and not Path(source_root).is_symlink(),'m4_recovery_local_source_required')
    current_boot_id=_identity(current_boot_id);_identity(original_boot_id)
    require(current_boot_id!=original_boot_id,'m4_recovery_distinct_boots_required')
    current_controller_url=validate_target(current_controller_url)
    proposal=prepare_failed_m4_reboot_recovery(repo_root=repo,source_root=root,result_sha256=result_sha256,scope_sha256=scope_sha256,
        original_boot_id=original_boot_id,current_boot_id=current_boot_id,original_controller_url=original_controller_url,current_controller_url=current_controller_url)
    require(plan_digest(proposal)==confirmed_proposal_sha256,'m4_recovery_proposal_approval_required')
    audit=audit_failed_m4_source(root,result_sha256=result_sha256,scope_sha256=scope_sha256,
        original_boot_id=original_boot_id,original_controller_url=original_controller_url)
    require(audit['hashes']==proposal['originalSourceHashes'],'m4_recovery_source_changed')
    sid=audit['scope']['trialId'];digest=audit['scopeDigest']
    path=repo/'.local/visual-actions/controller-commands.sqlite'
    require(path.is_file() and not path.is_symlink() and Path(str(path)+'.lock').is_file()
            and not Path(str(path)+'.lock').is_symlink(),'m4_recovery_existing_shared_journal_required')
    def unchanged():
        require({p.name for p in root.iterdir() if p.is_file()}==set(audit['hashes'])
                and all(hashlib.sha256(_read(root,name)).hexdigest()==value for name,value in audit['hashes'].items()),
                'm4_recovery_source_changed')
    with Path(str(path)+'.lock').open('rb') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        db=sqlite3.connect(path.as_uri()+'?mode=rw',uri=True)
        port=None
        try:
            db.execute('BEGIN IMMEDIATE')
            def owned():
                rows=[action_journal.M4CommandJournal._decode(r) for r in db.execute('SELECT * FROM commands ORDER BY rowid')]
                require(action_journal.readiness(rows,action_journal._resolutions(db))['ready'],'m4_recovery_other_command_pending')
                require(db.execute("SELECT session_id FROM supervised_blocks WHERE phase IN ('pending','off_unverified')").fetchall()==[(sid,)],
                        'm4_recovery_other_block_pending')
                row=db.execute('SELECT plan_hash,output_path,phase,result_json FROM supervised_blocks WHERE session_id=?',(sid,)).fetchone()
                require(row is not None and row[:3]==(digest,str(root),'pending') and row[3] is not None
                        and _decode_json(row[3].encode())==audit['result'],'m4_recovery_owned_block_required')
                return row
            row=owned();unchanged()
            require(hashlib.sha256(row[3].encode()).hexdigest()==proposal['originalResultJsonSha256'],'m4_recovery_block_changed')
            # 생성 자체가 승인된 원본의 단일 시도를 소모한다. 오류 후 새 ID/폴더로 우회하지 않는다.
            store=ArtifactStore(root/ATTEMPT)
            port=_ReadOnlyPort(current_controller_url,token,current_boot_id,store,clock=clock,compact_reads=True,timeout=2.)
            snapshot=port.preflight()
            capability=snapshot['sequence'].get('m4GoalGuard')
            require(isinstance(capability,dict) and capability.get('schemaVersion')==guard.CAPABILITY_SCHEMA
                    and capability.get('bootId')==current_boot_id and capability.get('enabled') is True
                    and capability.get('policySha256')==guard.POLICY_SHA256
                    and capability.get('execution')=='main_loop_http_deferred'
                    and capability.get('nonceCapacity')==32 and type(capability.get('nonceCount')) is int
                    and capability['nonceCount']==0 and capability.get('record') is None,
                    'm4_recovery_clean_guard_boot_required')
            require(port.off_verified and not port.may_have_run and not port.writes_possible
                    and port.last_outputs=={i:0 for i in range(1,6)},'m4_recovery_current_OFF_required')
            unchanged();require(owned()==row,'m4_recovery_block_changed')
            proof=dict(schemaVersion='motionbrain.failed_m4_reboot_current_off.v1',sessionId=sid,
                confirmedProposalSha256=confirmed_proposal_sha256,proposal=proposal,
                originalSourceHashes=audit['hashes'],originalScopeSha256=digest,
                originalResultJsonSha256=hashlib.sha256(row[3].encode()).hexdigest(),
                originalControllerBootId=original_boot_id,currentControllerBootId=current_boot_id,
                originalControllerUrl=original_controller_url,currentControllerUrl=current_controller_url,
                snapshot=snapshot,currentOffVerified=True,noWrites=True,originalOutcomeChanged=False,
                originalOutcomeReevaluated=False,datasetEligible=False,replayAllowed=False,
                originalConfigurationRestoreConfirmed=False,followupRequiresNewProspectiveChecks=True,
                meaning='현재 OFF 근거이며 과거 unknown 해명·실물 성공·학습 적격 증명이 아님')
            store.write_json('proof.json',proof);unchanged()
            changed=db.execute('UPDATE supervised_blocks SET phase=? WHERE session_id=? AND plan_hash=? AND output_path=? AND phase=? AND result_json=?',
                (PHASE,sid,digest,str(root),'pending',row[3]))
            require(changed.rowcount==1,'m4_recovery_block_changed');db.commit()
            return dict(sessionId=sid,phase=PHASE,currentOffVerified=True,noWrites=True,
                originalOutcomeChanged=False,replayAllowed=False,proofPath=str(store.root/'proof.json'))
        finally:
            if port is not None:port.token=''
            db.close()
