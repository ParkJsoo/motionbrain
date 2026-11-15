"""M4 최초 GET 시간 초과의 현재 OFF 해소. 실패 원본·모터 재송 금지는 보존한다.

prepare는 기존 파일/원장만 읽는다. recover는 같은 부팅에서 GET만 수행하고,
별도 근거를 저장한 뒤 정확히 소유한 off_unverified 행의 phase만 변경한다.
현재 자세가 과거와 달라도 현재 OFF 해소에는 무관하며 새 구동 적격을 뜻하지 않는다.
"""
from __future__ import annotations

import fcntl
import hashlib
from pathlib import Path
import re
import sqlite3
import time

from learning import action_journal
from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.manual_collection_dataset import require
from learning.m4_up_commissioning import DEVICE_GUARD_TASK_SCHEMA
from learning.supervised_collection import plan_digest
from learning.supervised_transport import SequencePort, finite_number, validate_target

PHASE = 'resolved_current_off_after_m4_initial_get_failure'
ATTEMPT = 'readonly-initial-get-recovery'


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def audit_failed_start(source_root, *, result_sha256, scope_sha256, controller_boot_id, controller_url):
    """최초 /actions GET 한 건만 있고 쓰기·관측·명령 영수증이 없는 실패만 허용한다."""
    root = Path(source_root).resolve(strict=True)
    require(re.fullmatch('[0-9a-f]{32}', controller_boot_id or '') is not None,
            'm4_start_recovery_boot_identity')
    validate_target(controller_url)
    hashes = {}

    def read(name):
        raw = _read(root, name)
        hashes[name] = _sha(raw)
        return _decode_json(raw)

    scope = read('scope.json')
    result = read('result.json')
    require(hashes['scope.json'] == scope_sha256 and hashes['result.json'] == result_sha256,
            'm4_start_recovery_source_changed')
    require(scope.get('schemaVersion') == DEVICE_GUARD_TASK_SCHEMA
            and scope.get('controllerBootId') == controller_boot_id
            and scope.get('controllerUrl') == controller_url
            and isinstance(scope.get('trialId'), str), 'm4_start_recovery_source_scope')
    require(result.get('schemaVersion') == DEVICE_GUARD_TASK_SCHEMA
            and result.get('scopeSha256') == plan_digest(scope)
            and result.get('status') == 'failed' and result.get('commandOutcome') == 'not_started'
            and result.get('mayHaveRun') is False and result.get('finalIdleOffVerified') is False
            and result.get('trainingEligible') is False and 'receipt' not in result
            and result.get('errors') == [dict(phase='preflight', type='TimeoutError')]
            and result.get('angleObservations') == [], 'm4_start_recovery_initial_failure_required')
    require(sorted(p.name for p in root.glob('http-*-intent.json')) == ['http-000001-intent.json']
            and not list(root.glob('http-*-response.json'))
            and not list(root.glob('*observation.json')),
            'm4_start_recovery_only_initial_GET_required')
    intent = read('http-000001-intent.json')
    error = read('http-000001-error.json')
    require(set(intent) == {'method', 'path', 'payload', 'startedAt', 'timeoutSeconds'}
            and intent['method'] == 'GET' and intent['path'] == '/actions' and intent['payload'] is None
            and finite_number(intent['startedAt']) and finite_number(intent['timeoutSeconds'])
            and 0 < intent['timeoutSeconds'] <= 2
            and set(error) == {'endedAt', 'error'} and error['error'] == 'TimeoutError'
            and finite_number(error['endedAt']) and error['endedAt'] >= intent['startedAt'],
            'm4_start_recovery_initial_GET_proof')
    # Bind every top-level artifact, including the optional transport phase trace.
    # New command/intent files or mutations while checking the device invalidate recovery.
    for path in root.iterdir():
        if path.is_file():
            hashes[path.name] = _sha(_read(root, path.name))
    return dict(scope=scope, result=result, hashes=hashes, scopeDigest=plan_digest(scope))


def _location(repo_root, source_root):
    repo = Path(repo_root).resolve(strict=True)
    root = Path(source_root).resolve(strict=True)
    require(root.is_relative_to(repo / '.local') and not Path(source_root).is_symlink(),
            'm4_start_recovery_private_source')
    path = repo / '.local/visual-actions/controller-commands.sqlite'
    lock = Path(str(path) + '.lock')
    require(path.is_file() and not path.is_symlink() and lock.is_file() and not lock.is_symlink(),
            'm4_start_recovery_existing_journal')
    return root, path, lock


def _owned(db, audit, root):
    sid = audit['scope']['trialId']
    row = db.execute('SELECT plan_hash,output_path,phase,result_json FROM supervised_blocks WHERE session_id=?',
                     (sid,)).fetchone()
    require(row is not None and row[:3] == (audit['scopeDigest'], str(root), 'off_unverified')
            and row[3] is not None and _decode_json(row[3].encode()) == audit['result'],
            'm4_start_recovery_owned_block')
    require(db.execute("SELECT session_id FROM supervised_blocks WHERE phase IN ('pending','off_unverified')").fetchall()
            == [(sid,)], 'm4_start_recovery_other_pending')
    rows = [action_journal.M4CommandJournal._decode(r) for r in db.execute('SELECT * FROM commands ORDER BY rowid')]
    require(action_journal.readiness(rows, action_journal._resolutions(db))['ready'],
            'm4_start_recovery_other_commands')
    return row


def prepare_failed_m4_start_recovery(*, repo_root, source_root, result_sha256, scope_sha256,
                                     controller_boot_id, controller_url):
    """장치 요청/파일 생성 없이 원본과 소유 원장 행을 결합한다."""
    root, path, lock_path = _location(repo_root, source_root)
    audit = audit_failed_start(root, result_sha256=result_sha256, scope_sha256=scope_sha256,
                               controller_boot_id=controller_boot_id, controller_url=controller_url)
    with lock_path.open('rb') as lock:
        fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        db = sqlite3.connect(path.as_uri() + '?mode=ro', uri=True)
        try:
            db.execute('BEGIN')
            row = _owned(db, audit, root)
            return dict(schemaVersion='motionbrain.m4_initial_get_recovery_proposal.v1',
                        sourceRoot=str(root), sourceHashes=audit['hashes'], journalPath=str(path),
                        sessionId=audit['scope']['trialId'], scopeDigest=audit['scopeDigest'],
                        resultJsonSha256=_sha(row[3].encode()), controllerBootId=controller_boot_id,
                        controllerUrl=controller_url, fromPhase='off_unverified', toPhase=PHASE,
                        automaticRetry=False, controllerWritesAllowed=False,
                        originalOutcomeChanged=False, replayAllowed=False, datasetEligible=False)
        finally:
            db.close()


class _ReadOnlyPort(SequencePort):
    def _call(self, path, payload=None, **kwargs):
        require(payload is None and not kwargs.get('run') and not kwargs.get('emergency')
                and path.split('?')[0] in ('/actions', '/status', '/collection_status', '/sequence', '/action_outputs'),
                'm4_start_recovery_GET_only')
        return super()._call(path, payload, **kwargs)


def _current_cursor_snapshot(port):
    """유한 ring의 만료된 과거와 현재 5OFF를 구분하는 GET 전용 판독.

    현재 speed/enabled를 읽는 동안 출력 쓰기가 없었다는 firmware cursor와
    최신 ring 꼬리를 대조한다. 누락된 과거 모터별 OFF를 복원했다고 주장하지 않는다.
    이 근거는 모터 실행용 SequencePort baseline으로 이전하지 않는다.
    """
    actions = port._call('/actions')
    require(actions.get('bootId') == port.boot_id and actions.get('active') is False,
            'm4_start_recovery_current_boot_or_action')
    status = port._status()
    port._check_status(status, 'IDLE')
    cursor = status.get('outputProof')
    require(isinstance(cursor, dict) and set(cursor) == {
                'schemaVersion', 'bootId', 'sequenceBefore', 'sequenceAfter', 'stable'}
            and cursor['schemaVersion'] == 'driver_cursor.v1' and cursor['bootId'] == port.boot_id
            and cursor['stable'] is True and type(cursor['sequenceBefore']) is int
            and type(cursor['sequenceAfter']) is int and cursor['sequenceBefore'] > 0
            and cursor['sequenceBefore'] == cursor['sequenceAfter'],
            'm4_start_recovery_stable_driver_cursor_required')
    sequence = cursor['sequenceAfter']
    after = max(0, sequence - 8)
    outputs = port._call(f'/action_outputs?after={after}&limit=8')
    require(outputs.get('bootId') == port.boot_id and outputs.get('evidence') == 'driver_write'
            and outputs.get('clockDomain') == 'controller_boot_millis'
            and outputs.get('latestSequence') == sequence and outputs.get('gap') is False
            and outputs.get('hasMore') is False and isinstance(outputs.get('entries'), list)
            and len(outputs['entries']) == sequence - after
            and type(outputs.get('oldestSequence')) is int and 0 < outputs['oldestSequence'] <= after + 1,
            'm4_start_recovery_output_epoch_changed')
    require([e.get('sequence') for e in outputs['entries']] == list(range(after + 1, sequence + 1)),
            'm4_start_recovery_output_tail_gap')
    # Reuse strict entry types/time/identity validation, but never seed unknown
    # motors from status into the transport's execution baseline.
    port._consume(outputs)
    require(all(value == 0 for value in port.last_outputs.values()), 'm4_start_recovery_tail_not_OFF')
    queue = port._queue(0)
    require(not port.may_have_run and not port.writes_possible, 'm4_start_recovery_GET_only')
    return dict(actions=actions, status=status, outputs=outputs, sequence=queue)


def recover_failed_m4_start_readonly(*, repo_root, source_root, result_sha256, scope_sha256,
                                    controller_boot_id, controller_url, token,
                                    confirmed_proposal_sha256, clock=time):
    """현재 두 GET 묶음의 동일 부팅·출력 epoch·OFF·빈 큐를 확인하고 phase만 해소한다."""
    args = dict(repo_root=repo_root, source_root=source_root, result_sha256=result_sha256,
                scope_sha256=scope_sha256, controller_boot_id=controller_boot_id, controller_url=controller_url)
    proposal = prepare_failed_m4_start_recovery(**args)
    require(plan_digest(proposal) == confirmed_proposal_sha256, 'm4_start_recovery_approval_changed')
    root, path, lock_path = _location(repo_root, source_root)
    audit = audit_failed_start(root, result_sha256=result_sha256, scope_sha256=scope_sha256,
                               controller_boot_id=controller_boot_id, controller_url=controller_url)

    def unchanged():
        require({p.name: _sha(_read(root, p.name)) for p in root.iterdir() if p.is_file()} == proposal['sourceHashes'],
                'm4_start_recovery_source_changed')

    with lock_path.open('rb') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        db = sqlite3.connect(path.as_uri() + '?mode=rw', uri=True)
        port = None
        try:
            db.execute('BEGIN IMMEDIATE')
            row = _owned(db, audit, root)
            require(_sha(row[3].encode()) == proposal['resultJsonSha256'], 'm4_start_recovery_block_changed')
            unchanged()
            store = ArtifactStore(root / ATTEMPT)
            port = _ReadOnlyPort(controller_url, token, controller_boot_id, store,
                                 clock=clock, compact_reads=True, timeout=2.)
            snapshots = []
            epochs = []
            for _ in range(2):
                snapshot = _current_cursor_snapshot(port)
                snapshots.append(snapshot)
                epochs.append((port.boot_id, port.last_sequence))
            require(epochs[0] == epochs[1] and epochs[0][0] == controller_boot_id,
                    'm4_start_recovery_output_epoch_changed')
            uptimes = [s['status'].get('uptimeMs') for s in snapshots]
            require(all(type(v) is int and v >= 0 for v in uptimes) and uptimes[1] >= uptimes[0],
                    'm4_start_recovery_uptime_reverted')
            unchanged()
            require(_owned(db, audit, root) == row, 'm4_start_recovery_block_changed')
            store.write_json('proof.json', dict(schemaVersion='motionbrain.m4_initial_get_current_off.v1',
                proposal=proposal, confirmedProposalSha256=confirmed_proposal_sha256, snapshots=snapshots,
                controllerBootId=controller_boot_id, outputSequence=port.last_sequence,
                currentOffVerified=True, noWrites=True, originalOutcomeChanged=False,
                originalOutcomeReevaluated=False, datasetEligible=False, replayAllowed=False,
                currentOffMeaning='stable_driver_cursor_and_current_motor_state_not_complete_historical_OFF_trace',
                executableTransportBaselineCreated=False,
                followupRequiresNewProspectiveChecks=True))
            unchanged()
            changed = db.execute('UPDATE supervised_blocks SET phase=? WHERE session_id=? AND plan_hash=? '
                'AND output_path=? AND phase=? AND result_json=?',
                (PHASE, proposal['sessionId'], audit['scopeDigest'], str(root), 'off_unverified', row[3]))
            require(changed.rowcount == 1, 'm4_start_recovery_block_changed')
            db.commit()
            return dict(sessionId=proposal['sessionId'], phase=PHASE, currentOffVerified=True,
                        noWrites=True, originalOutcomeChanged=False, replayAllowed=False,
                        proofPath=str(store.root / 'proof.json'))
        finally:
            if port is not None:
                port.token = ''
            db.close()
