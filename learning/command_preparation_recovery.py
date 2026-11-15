"""명령 의도는 저장됐지만 HTTP 쓰기 전 실패한 수동 수집의 GET 전용 해소."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re

from learning.action_journal import BoundedActionJournal
from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.attended_collection_profile import audit_profile
from learning.manual_collection_dataset import require
from learning.supervised_collection import identifier
from learning.supervised_transport import SequencePort, response_json, validate_target

ATTEMPT = 'readonly-command-preparation-recovery'
PHASE = 'resolved_current_off_after_command_preparation'


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _source_hashes(root):
    result = {}
    for path in root.rglob('*'):
        relative = path.relative_to(root)
        if relative.parts[0] == ATTEMPT:
            continue
        require(not path.is_symlink(), 'command_preparation_recovery_source_symlink')
        if path.is_file():
            result[relative.as_posix()] = _sha(_read(root, relative.as_posix()))
    return result


def audit_command_preparation(root, source_sha256, *, session_id, controller_boot_id,
                              original_controller_url, device_id, view_id):
    """원본·명령 artifact·HTTP 원본을 읽는다. 장치 요청과 파일 쓰기는 없다."""
    require(identifier(session_id) and re.fullmatch('[0-9a-f]{32}', controller_boot_id or '') is not None,
            'command_preparation_recovery_identity')
    original_controller_url = validate_target(original_controller_url)
    root = Path(root)
    raw = _read(root, 'session.json')
    require(_sha(raw) == source_sha256, 'command_preparation_recovery_source_changed')
    original = _decode_json(raw)
    plan = _decode_json(_read(root, 'plan.json'))
    require(original.get('schemaVersion') == 'motionbrain.manual_collection.v1'
            and original.get('sessionId') == session_id and original.get('phase') == 'failed'
            and original.get('error') == 'command_failed_no_retry'
            and original.get('finalIdleOffVerified') is False and original.get('datasetEligible') is False
            and isinstance(original.get('frames'), list) and bool(original['frames'])
            and isinstance(original.get('recording'), dict)
            and plan.get('sessionId') == session_id and plan.get('controllerUrl') == original_controller_url
            and plan.get('camera', {}).get('deviceId') == device_id
            and plan.get('camera', {}).get('viewId') == view_id,
            'command_preparation_recovery_owned_source')
    events = original.get('events')
    require(isinstance(events, list) and bool(events)
            and all(isinstance(e, dict) and e.get('kind') in ('http', 'http_error', 'command') for e in events),
            'command_preparation_recovery_no_result_required')
    commands = [e for e in events if e['kind'] == 'command']
    require(len(commands) == 1 and sorted(p.name for p in root.glob('command-*')) == ['command-001-intent.json'],
            'command_preparation_recovery_single_command_required')
    command = commands[0]
    require(command == _decode_json(_read(root, 'command-001-intent.json'))
            and set(command) == {'at', 'commandId', 'kind', 'pulse'}
            and command['commandId'] == session_id + '-001' and command['pulse']['joint'] == original.get('joint')
            and plan.get('joint') == original.get('joint'), 'command_preparation_recovery_command_binding')
    audit_profile(plan, original, command['pulse'])

    def allowed(path):
        return isinstance(path, str) and (path in ('/actions', '/status', '/collection_status', '/sequence', '/action_outputs')
            or re.fullmatch(r'/action_outputs\?after=[0-9]+(?:&limit=[1-9][0-9]*)?', path) is not None)

    http_events = [e for e in events if e['kind'] != 'command']
    require(all(allowed(e.get('path')) and e.get('payload') is None for e in http_events)
            and {e.get('response', {}).get('bootId') for e in http_events
                 if e['kind'] == 'http' and e['path'] == '/actions'} == {controller_boot_id},
            'command_preparation_recovery_GET_only_required')
    intents = sorted(root.glob('http-*-intent.json'))
    require(bool(intents) and [p.name for p in intents] == [f'http-{i:06d}-intent.json' for i in range(1, len(intents)+1)],
            'command_preparation_recovery_HTTP_gap')
    responses = []
    for path in intents:
        intent = _decode_json(_read(root, path.name))
        require(intent.get('method') == 'GET' and intent.get('payload') is None and allowed(intent.get('path')),
                'command_preparation_recovery_GET_only_required')
        response_path = path.with_name(path.name.replace('-intent', '-response'))
        error_path = path.with_name(path.name.replace('-intent', '-error'))
        require(response_path.is_file() != error_path.is_file(), 'command_preparation_recovery_HTTP_terminal_required')
        if response_path.is_file():
            response = _decode_json(_read(root, response_path.name))
            parsed = response_json(response['rawUtf8'], intent['path'])
            require(response.get('httpStatus') == 200 and parsed == response.get('parsed'),
                    'command_preparation_recovery_raw_response_changed')
            if intent['path'] == '/actions':
                require(parsed.get('bootId') == controller_boot_id, 'command_preparation_recovery_source_boot')
            responses.append((intent['path'], parsed))
        else:
            error = _decode_json(_read(root, error_path.name))
            require(error.get('error') == 'TimeoutError', 'command_preparation_recovery_known_read_failure')
    require(any(e['kind'] == 'http_error' and e.get('error') == 'TimeoutError' for e in http_events),
            'command_preparation_recovery_timeout_required')
    require(all(any(e['kind'] == 'http' and e['path'] == path and e.get('response') == parsed for e in http_events)
                for path, parsed in responses), 'command_preparation_recovery_HTTP_event_binding')
    require(not any(p.name.startswith('readonly-') for p in root.iterdir()),
            'command_preparation_recovery_attempt_consumed')
    return dict(original=original, plan=plan, sourceHashes=_source_hashes(root),
                planHash=_sha(json.dumps(plan, sort_keys=True).encode()))


class _ReadOnlyPort(SequencePort):
    def _call(self, path, payload=None, **kwargs):
        require(payload is None and not kwargs.get('run') and not kwargs.get('emergency')
                and path.split('?')[0] in ('/actions', '/status', '/collection_status', '/sequence', '/action_outputs'),
                'command_preparation_recovery_GET_only_required')
        return super()._call(path, payload, **kwargs)


def recover_command_preparation_readonly(manager, session_id, source_sha256, controller_boot_id,
                                         *, original_controller_url):
    require(not manager.busy and all(getattr(manager, name) is None for name in ('current', 'port', 'camera', 'store')),
            'command_preparation_recovery_idle_owner_required')
    require(identifier(session_id), 'command_preparation_recovery_identity')
    root = manager.root / session_id
    audit = audit_command_preparation(root, source_sha256, session_id=session_id,
        controller_boot_id=controller_boot_id, original_controller_url=original_controller_url,
        device_id=manager.device_id, view_id=manager.view_id)
    require(manager.journal_path.is_file(), 'command_preparation_recovery_existing_journal_required')
    if manager.journal is None:
        manager.journal = BoundedActionJournal(str(manager.journal_path), 10000)
    db = manager.journal.connection
    row = db.execute('SELECT plan_hash,output_path,phase,result_json FROM supervised_blocks WHERE session_id=?',
                     (session_id,)).fetchone()

    def source_unchanged():
        require(_source_hashes(root) == audit['sourceHashes'], 'command_preparation_recovery_source_changed')

    def owned():
        current = db.execute('SELECT plan_hash,output_path,phase,result_json FROM supervised_blocks WHERE session_id=?',
                             (session_id,)).fetchone()
        require(row is not None and current == row and row[:3] == (audit['planHash'], str(root), 'off_unverified')
                and row[3] is not None and _decode_json(row[3].encode()) == audit['original']
                and manager.journal.readiness()['ready']
                and db.execute("SELECT session_id FROM supervised_blocks WHERE phase IN ('pending','off_unverified')").fetchall()
                    == [(session_id,)], 'command_preparation_recovery_owned_block_required')

    owned()
    manager.busy = True
    port = None
    try:
        store = ArtifactStore(root / ATTEMPT)
        port = _ReadOnlyPort(manager.controller_url, manager.token, controller_boot_id, store,
                             clock=manager.clock, compact_reads=True, timeout=2., full_output_snapshot=True)
        snapshots = []
        for _ in range(2):
            snapshot = port.preflight()
            require(port.off_verified and not port.may_have_run and not port.writes_possible
                    and port.last_outputs == {i: 0 for i in range(1, 6)}, 'command_preparation_recovery_current_OFF_required')
            snapshots.append(snapshot)
        source_unchanged()
        owned()
        store.write_json('proof.json', dict(sessionId=session_id, originalSessionSha256=source_sha256,
            originalResultSha256=_sha(row[3].encode()), originalSourceHashes=audit['sourceHashes'],
            originalControllerUrl=original_controller_url, currentControllerUrl=manager.controller_url,
            controllerBootId=controller_boot_id, snapshots=snapshots, currentOffVerified=True,
            noWrites=True, originalOutcomeChanged=False, originalOutcomeReevaluated=False,
            datasetEligible=False, replayAllowed=False, executableTransportBaselineCreated=False))
        with db:
            source_unchanged()
            owned()
            changed = db.execute('UPDATE supervised_blocks SET phase=? WHERE session_id=? AND plan_hash=? '
                'AND output_path=? AND phase=? AND result_json=?',
                (PHASE, session_id, audit['planHash'], str(root), 'off_unverified', row[3]))
            require(changed.rowcount == 1, 'command_preparation_recovery_owned_block_required')
        return dict(sessionId=session_id, phase=PHASE, currentOffVerified=True, noWrites=True,
                    originalOutcomeChanged=False, replayAllowed=False, proofPath=str(store.root / 'proof.json'))
    finally:
        if port is not None:
            port.token = ''
        manager.busy = False
