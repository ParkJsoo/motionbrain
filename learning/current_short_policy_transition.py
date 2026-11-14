"""UP600 원본의 미래 DOWN250→300/350 선언만 명시적으로 이전하는 오프라인 감사.

실제 명령·관측·원본 행은 바꾸지 않는다. proof 없이 기존 strict row builder를
완화하지 않으며, 파생 행과 원행의 동일 episode/frame은 독립 자료가 아니다.
"""
from __future__ import annotations
import copy
from hashlib import sha256
from pathlib import Path
from learning.artifacts import _read,_decode_json
from learning.current_m4_short import current_short_policy
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest as digest
from learning import task_transition_data as data

SCHEMA='motionbrain.UP600_future_DOWN_only_equivalence.v1'
CHANGED_FIELDS=['downDurationCapMs','downMonitorTimingPolicy']
PROOF_MEANING='unchanged_executed_UP600_only_future_DOWN_declaration_changed_not_new_execution'


def target_registration(source_registration,source_context,*,down_duration_ms=300):
    """별도 새 등록의 내용만 만든다. 원본 수정·자료 전이 승인·파일 쓰기는 없다."""
    require(type(down_duration_ms) is int and down_duration_ms in (300,350),
            'future_DOWN_explicit_target_required')
    result=copy.deepcopy(source_registration)
    result['context']['executionPolicy']=current_short_policy(600,down_duration_ms)
    result['acceptedSourceContexts']=[dict(copy.deepcopy(source_context),supportMode=s)
                                     for s in ('table_supported','gripper_supported')]
    return result


def _inputs(root,source_registration,target,annotation):
    root=Path(root).resolve()
    # 기존 원본·single RUN·OFF·영상·숫자 주석 검사를 그대로 두 번 재사용한다.
    source_row=data.build_manual_m4_row(root,source_registration,annotation)
    report=data.audit_manual_m4(root)
    scope_raw=_read(root,'scope.json');scope=_decode_json(scope_raw)
    require(report.get('executionPolicy')==current_short_policy(600,250)
            and source_row['command']==dict(joint='shoulder',direction='up',percent=50,durationMs=600)
            and report['supportMode']=='table_supported','future_DOWN_only_UP600_source_required')
    require(source_row['sourceHash']==digest(dict(sourceProofs=report['sourceProofs'],
            registration=source_registration,annotation=annotation))
            and sha256(scope_raw).hexdigest()==report['sourceProofs']['scope.json'],
            'future_DOWN_source_changed_during_audit')
    require(isinstance(target,dict),'future_DOWN_target_registration_changed')
    cap=target.get('context',{}).get('executionPolicy',{}).get('downDurationCapMs')
    expected=target_registration(source_registration,report['originalContext'],down_duration_ms=cap)
    require(isinstance(target,dict) and target==expected,'future_DOWN_target_registration_changed')
    source_policy=report['executionPolicy'];target_policy=target['context']['executionPolicy']
    changed=sorted(k for k in source_policy if source_policy[k]!=target_policy[k])
    require(changed==sorted(CHANGED_FIELDS),'future_DOWN_executed_UP_contract_changed')
    # scope의 identity/부팅/관측/한도/실제 pulse/UP monitor까지 고정해 비교한다.
    original=copy.deepcopy(scope);proposed=copy.deepcopy(scope)
    proposed['shortTaskPolicy']=copy.deepcopy(target_policy)
    for obj in (original,proposed):
        for key in CHANGED_FIELDS:obj['shortTaskPolicy'].pop(key)
    require(original==proposed,'future_DOWN_executed_UP_contract_changed')
    return source_row,report,scope_raw,digest(original)


def make_equivalence_proof(root,source_registration,target,annotation,*,review_statement):
    """root가 검토·별도 보존할 정확한 proof 내용. 파생 row 생성은 별도 호출이다."""
    require(isinstance(review_statement,str) and bool(review_statement.strip())
            and len(review_statement)<=2048,'future_DOWN_explicit_review_required')
    row,report,scope_raw,projection=_inputs(root,source_registration,target,annotation)
    return dict(schemaVersion=SCHEMA,sourceScopeRawSha256=sha256(scope_raw).hexdigest(),
        sourceResultSha256=report['sourceResultSha256'],sourceProofsSha256=digest(report['sourceProofs']),
        sourceRegistrationSha256=digest(source_registration),sourceAnnotationSha256=digest(annotation),
        sourceRowSha256=digest(row),targetRegistrationSha256=digest(target),
        sourcePolicy=copy.deepcopy(report['executionPolicy']),targetPolicy=copy.deepcopy(target['context']['executionPolicy']),
        changedFutureFields=list(CHANGED_FIELDS),executedUPContractSha256=projection,
        reviewedSourceEquivalence=True,reviewStatement=review_statement,meaning=PROOF_MEANING,
        newExecution=False,originalOutcomeChanged=False,independentNewSource=False)


def derive_up_row(root,source_registration,target,annotation,*,proof=None):
    """명시 proof가 정확히 재검증될 때만 새 context/sourceHash의 파생 행을 반환한다."""
    require(isinstance(proof,dict),'future_DOWN_equivalence_proof_required')
    expected=make_equivalence_proof(root,source_registration,target,annotation,
                                   review_statement=proof.get('reviewStatement'))
    require(proof==expected,'future_DOWN_equivalence_proof_changed')
    original,report,_,_=_inputs(root,source_registration,target,annotation)
    derived=copy.deepcopy(original)
    derived['context']=dict(copy.deepcopy(target['context']),registrationSha256=digest(target))
    derived['sourceHash']=digest(dict(originalSourceHash=original['sourceHash'],
        equivalenceProofSha256=digest(proof),targetRegistrationSha256=digest(target)))
    # id/episode/frames/command/전후 관측은 원행과 동일해 중복 학습 검사가 작동한다.
    return dict(row=derived,audit=dict(schemaVersion=SCHEMA,status='verified_future_DOWN_only_derived_UP_row',
        sourceRowSha256=digest(original),derivedRowSha256=digest(derived),proofSha256=digest(proof),
        sourceResultSha256=report['sourceResultSha256'],originalOutcomeChanged=False,
        independentNewSource=False,hardwareRequests=0,modelSelectedCommands=0))
