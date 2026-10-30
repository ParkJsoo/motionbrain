"""Bounded-action journal: immutable resolution evidence, no physical commands.

The legacy ROS M4 journal keeps its original policy. Only this subclass understands
resolutions. Original unknown rows and consumed IDs are never cleared or replayed.
"""
from __future__ import annotations
import copy
from contextlib import contextmanager
import fcntl
import hashlib
import json
from pathlib import Path
import sqlite3
import time

from learning.artifacts import _decode_json
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.m4_command_journal import M4CommandJournal

RESOLUTION_SCHEMA = 'motionbrain.action_resolution.v1'
EVIDENCE_SCHEMA = 'motionbrain.action_resolution_evidence.v1'


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',', ':'),allow_nan=False).encode()).hexdigest()


def validate_row(row):
    from learning.action_execution import SCHEMA, canonical_request
    if row['request'].get('executorSchema') != SCHEMA:
        raise ValueError('journal_protocol_mismatch')
    request=canonical_request(row['request']['action'])
    fingerprint=hashlib.sha256(json.dumps(request,sort_keys=True,allow_nan=False).encode()).hexdigest()
    if row['commandId']!=request['commandId'] or row['fingerprint']!=fingerprint:
        raise ValueError('journal_request_mismatch')
    return request


def validate_evidence(row, evidence):
    from learning.action_execution import checked_record, finite, selected_outputs
    from learning.runner import KNOWN_TERMINAL
    request=validate_row(row)
    if (row['phase']!='outcome_unknown' or evidence.get('schemaVersion')!=EVIDENCE_SCHEMA
            or evidence.get('originalRowSha256')!=digest(row) or evidence.get('commandId')!=row['commandId']
            or evidence.get('request')!=request):
        raise ValueError('resolution_row_binding_mismatch')
    elapsed=evidence.get('readDurationSeconds')
    if not finite(elapsed) or not 0<=elapsed<=4: raise ValueError('resolution_reads_expired')
    record=checked_record(evidence['terminalRecord'],request)
    if record['state'] not in KNOWN_TERMINAL or record['outputOff'] is not True:
        raise ValueError('terminal_off_required')
    capability,status,outputs=(evidence[k] for k in ('capability','status','outputs'))
    for cap in (evidence['initialCapability'],capability):
        if (cap.get('bootId')!=request['bootId'] or cap.get('active') is not False
                or cap.get('enabled') is not True or type(cap.get('recordCount')) is not int
                or cap['recordCount']!=record['ordinal']):
            raise ValueError('same_boot_idle_unchanged_last_action_required')
    sensor,teleop,motors=(status.get(k,{}) for k in ('sensor','teleop','motors'))
    if (status.get('state')!='IDLE' or set(motors)!={f'M{i}' for i in range(1,6)}
            or any(not isinstance(m,dict) or m.get('enabled') is not False or type(m.get('speed')) is not int or m['speed']!=0 for m in motors.values())
            or any(sensor.get(k) is not True for k in ('connected','imuOk'))
            or any(sensor.get(k) is not False for k in ('simulated','faultLatched','blocked'))
            or teleop.get('deadman') is not False or teleop.get('controlActive') is not False):
        raise ValueError('fresh_idle_off_safety_required')
    if outputs.get('bootId')!=request['bootId'] or outputs.get('dropped')!=0:
        raise ValueError('complete_same_boot_outputs_required')
    entries=outputs.get('entries',[]);latest={};previous=0
    for e in entries:
        if (type(e.get('sequence')) is not int or e['sequence']!=previous+1
                or type(e.get('actionOrdinal')) is not int or not 0<=e['actionOrdinal']<=record['ordinal']
                or type(e.get('motor')) is not int or not 1<=e['motor']<=5
                or type(e.get('pwm')) is not int or not -255<=e['pwm']<=255):
            raise ValueError('output_sequence_or_value_invalid')
        previous=e['sequence'];latest[e['motor']]=e['pwm']
    if latest!={i:0 for i in range(1,6)}: raise ValueError('latest_five_outputs_off_required')
    first=next((i for i,e in enumerate(entries) if e.get('actionOrdinal')==record['ordinal']),None)
    if first is None: raise ValueError('terminal_output_evidence_missing')
    baseline={**outputs,'entries':entries[:first]}
    selected_outputs(outputs,record,baseline)
    return record


def observe_resolution(row, reader, *, clock=time):
    request=validate_row(row);start=clock.monotonic()
    initial_capability=reader.capability()
    if initial_capability.get('bootId')!=request['bootId'] or initial_capability.get('active') is not False:
        raise ValueError('same_boot_idle_capability_required')
    record=reader.lookup(request)
    status=reader.status()
    outputs=reader.outputs()
    capability=reader.capability()
    evidence=dict(schemaVersion=EVIDENCE_SCHEMA,commandId=row['commandId'],originalRowSha256=digest(row),
                  request=request,terminalRecord=record,status=status,outputs=outputs,initialCapability=initial_capability,capability=capability,
                  readDurationSeconds=clock.monotonic()-start)
    evidence['terminalRecord']=validate_evidence(row,evidence)
    return evidence


def resolution_valid(row, resolution):
    try:
        if (resolution.get('schemaVersion')!=RESOLUTION_SCHEMA
                or resolution.get('originalRowSha256')!=digest(row)
                or resolution.get('permitFutureNewCommandsOnly') is not True
                or resolution.get('trainingEligible') is not False
                or resolution.get('originalOutcomeChanged') is not False
                or resolution.get('confirmedSha256')!=digest(resolution['preparedEvidence'])):
            return False
        a=validate_evidence(row,resolution['preparedEvidence'])
        b=validate_evidence(row,resolution['freshEvidence'])
        return a==b
    except (KeyError,TypeError,ValueError):
        return False


def readiness(rows, resolutions, *, command_id=None):
    blockers=[];resolved=[]
    for row in rows:
        validate_row(row)
        cid=row['commandId'];phase=row['phase']
        if phase=='outcome_unknown':
            if cid in resolutions and resolution_valid(row,resolutions[cid]):resolved.append(cid)
            else:blockers.append(dict(commandId=cid,reason='unresolved_outcome_unknown'))
        elif phase in ('pending','validating','dispatching'):
            blockers.append(dict(commandId=cid,phase=phase,reason='interrupted_command_requires_recovery'))
        elif phase not in ('not_sent','known_terminal'):
            raise ValueError('unknown_journal_phase')
        if command_id is not None and cid==command_id:
            blockers.append(dict(commandId=cid,reason='command_id_already_used'))
    return dict(ready=not blockers,blockers=blockers,resolvedUnknownIds=resolved,
                snapshotSha256=digest(rows),resolutionSnapshotSha256=digest(resolutions),
                commandCount=len(rows),readOnly=True,hardwareCalls=0)


def _resolutions(connection):
    exists=connection.execute("SELECT 1 FROM sqlite_master WHERE type='table' AND name='action_resolutions'").fetchone()
    if not exists:return {}
    return {cid:_decode_json(raw.encode()) for cid,raw in connection.execute('SELECT command_id, resolution_json FROM action_resolutions')}


@contextmanager
def journal_snapshot(path):
    """Hold the shared lock and read transaction; never create or recover files."""
    path=Path(path).resolve()
    if not path.exists():
        yield [], {}, False
        return
    with Path(str(path)+'.lock').open('rb') as lock:
        fcntl.flock(lock,fcntl.LOCK_SH|fcntl.LOCK_NB)
        connection=sqlite3.connect(path.as_uri()+'?mode=ro',uri=True)
        try:
            connection.execute('BEGIN')
            rows=[M4CommandJournal._decode(row) for row in connection.execute('SELECT * FROM commands ORDER BY rowid')]
            yield rows, _resolutions(connection), True
        finally:connection.close()


def read_journal(path, *, command_id=None):
    """No constructor, recovery, schema migration, lock-file creation or device I/O."""
    with journal_snapshot(path) as (rows,resolutions,exists):
        return {**readiness(rows,resolutions,command_id=command_id),'journalExists':exists}


def prepare_resolution(path, command_id, reader, *, clock=time):
    with journal_snapshot(path) as (rows,resolutions,exists):
        readiness(rows,resolutions)
        row=next((row for row in rows if row['commandId']==command_id),None)
        if row is None:raise ValueError('command_missing')
        if command_id in resolutions:raise ValueError('resolution_already_recorded')
        return observe_resolution(row,reader,clock=clock)


class ResolutionReader:
    """Only GET methods; no token, ARM, capture, cancel or submit capability."""
    def __init__(self, controller_url):
        from learning.action_port import BoundedActionHTTP
        self._port=BoundedActionHTTP(controller_url,writes_enabled=False,timeout=.6,total_deadline=True)

    def lookup(self, request):return self._port.lookup(request)
    def outputs(self):return self._port.outputs()
    def status(self):
        with self._port._deadline():return self._port.reader.status()
    def capability(self):
        with self._port._deadline():return self._port.reader.capability()


class BoundedActionJournal(M4CommandJournal):
    def __init__(self,path,max_records):
        super().__init__(path,max_records)
        try:
            # Reject a different protocol before adding any bounded-action tables.
            for row in self.records():validate_row(row)
            with self.connection:
                self.connection.execute('CREATE TABLE IF NOT EXISTS action_resolutions (command_id TEXT PRIMARY KEY, resolution_json TEXT NOT NULL)')
                self.connection.execute('CREATE TABLE IF NOT EXISTS action_history (id INTEGER PRIMARY KEY, command_id TEXT NOT NULL, before_json TEXT NOT NULL, after_json TEXT NOT NULL)')
                for table in ('action_resolutions','action_history'):
                    for operation in ('UPDATE','DELETE'):
                        self.connection.execute(f"CREATE TRIGGER IF NOT EXISTS {table}_no_{operation.lower()} BEFORE {operation} ON {table} BEGIN SELECT RAISE(ABORT,'append_only'); END")
        except BaseException:
            self.close();raise

    def readiness(self,command_id=None):
        with self.lock:
            return readiness(self.records(),_resolutions(self.connection),command_id=command_id)

    def reserve(self,command_id):
        with self.lock,self.connection:
            row=self.get(command_id)
            if row is None:return None,'proposal_not_found'
            if row['phase']!='pending':return row,'proposal_already_consumed'
            resolutions=_resolutions(self.connection)
            for other in self.records():
                validate_row(other)
                if other['phase']=='outcome_unknown' and not resolution_valid(other,resolutions.get(other['commandId'],{})):
                    return row,'unresolved_outcome_unknown'
                if other['phase'] in ('validating','dispatching'):return row,'executor_busy'
            self.connection.execute("UPDATE commands SET phase='validating' WHERE command_id=? AND phase='pending'",(command_id,))
            return row,None

    def save(self,command_id,phase,result):
        with self.lock,self.connection:
            before=self.get(command_id)
            if before is None:raise ValueError('command_missing')
            self._save_history(before,phase,result)

    def _save_history(self,before,phase,result):
        # Caller holds the lock and transaction; recovery commits all rows together.
        command_id=before['commandId']
        self.connection.execute('UPDATE commands SET phase=?,result_json=? WHERE command_id=?',
            (phase,json.dumps(result,sort_keys=True,allow_nan=False),command_id))
        after=self.get(command_id)
        self.connection.execute('INSERT INTO action_history(command_id,before_json,after_json) VALUES (?,?,?)',
            (command_id,json.dumps(before,sort_keys=True,allow_nan=False),json.dumps(after,sort_keys=True,allow_nan=False)))

    def resolve(self,prepared,confirmed_sha256,reader,*,clock=time):
        """Explicit local append, using fresh GET evidence; never calls motor methods."""
        if confirmed_sha256!=digest(prepared):raise ValueError('exact_resolution_confirmation_required')
        with self.lock:
            row=self.get(prepared['commandId'])
            validate_evidence(row,prepared)
            if row['commandId'] in _resolutions(self.connection):raise ValueError('resolution_already_recorded')
            fresh=observe_resolution(row,reader,clock=clock)
            if fresh['terminalRecord']!=prepared['terminalRecord']:raise ValueError('terminal_record_changed')
            resolution=dict(schemaVersion=RESOLUTION_SCHEMA,originalRowSha256=digest(row),
                            confirmedSha256=confirmed_sha256,preparedEvidence=copy.deepcopy(prepared),freshEvidence=fresh,
                            permitFutureNewCommandsOnly=True,originalOutcomeChanged=False,trainingEligible=False)
            if not resolution_valid(row,resolution):raise ValueError('resolution_invalid')
            with self.connection:
                self.connection.execute('INSERT INTO action_resolutions VALUES (?,?)',
                    (row['commandId'],json.dumps(resolution,sort_keys=True,allow_nan=False)))
            return resolution

    def recover_interrupted(self,confirmed_snapshot_sha256):
        """Retire stale proposals; uncertain dispatch remains blocked, preserving both states."""
        from learning.action_execution import BoundedActionExecutor
        with self.lock,self.connection:
            if digest(self.records())!=confirmed_snapshot_sha256:raise ValueError('exact_recovery_confirmation_required')
            ids=[]
            for row in self.interrupted():
                attempted=row['phase']=='dispatching';phase='outcome_unknown' if attempted else 'not_sent'
                self._save_history(row,phase,BoundedActionExecutor._result(row['commandId'],phase,'explicit_interrupted_recovery',attempted))
                ids.append(row['commandId'])
            return ids
