"""Attended all-joint recording sessions using the existing bounded sequence path.

Call on the main thread: SequencePort enforces a wall-clock HTTP deadline with
SIGALRM. No motor/camera access occurs in the constructor or replay methods.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import shutil
import time
import uuid

from learning.action_journal import BoundedActionJournal
from learning.action_recorder import ActionRecorder
from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.supervised_collection import PROFILES, identifier
from learning.supervised_transport import SequencePort, validate_target, validate_monitor_policy, WRIST_READ_BUDGET_REVISION

SCHEMA = 'motionbrain.manual_collection.v1'
REPO = Path(__file__).resolve().parents[1]


def _boot_identity(capability):
    boot = capability.get('bootId')
    if not isinstance(boot, str) or len(boot) != 32 or any(c not in '0123456789abcdef' for c in boot):
        raise ValueError('controller_boot_identity_invalid')
    return boot


class RecordingPort(SequencePort):
    """Expose already-durable transport observations for the session viewer."""
    def __init__(self, *args, observer, **kwargs):
        super().__init__(*args, **kwargs)
        self.observer = observer
        self._observed_path = None
        self._observed_payload = None

    def _save(self, path, value, emergency=False):
        super()._save(path, value, emergency)
        # Only successfully persisted retry evidence enters the viewer. These
        # attempts are also kept as immutable raw files by the base transport.
        if path.endswith(('-attempt-1-error.json', '-attempt-2-intent.json', '-recovery-intent.json')):
            self.observer(dict(kind='http_retry', at=value.get('endedAt', value.get('startedAt')),
                               path=self._observed_path, payload=copy.deepcopy(self._observed_payload),
                               attempt=1 if path.endswith('-error.json') else 2,
                               evidenceFile=path, detail=copy.deepcopy(value)))

    def _call(self, path, payload=None, **kwargs):
        started = self.clock.monotonic()
        self._observed_path, self._observed_payload = path, payload
        try:
            result = super()._call(path, payload, **kwargs)
        except BaseException as exc:
            self.observer(dict(kind='http_error', at=self.clock.monotonic(),
                               requestStartedAt=started, path=path, payload=copy.deepcopy(payload),
                               error=type(exc).__name__))
            raise
        self.observer(dict(kind='http', at=self.clock.monotonic(),
                           requestStartedAt=started, responseReceivedAt=self.clock.monotonic(),
                           path=path, payload=copy.deepcopy(payload), response=copy.deepcopy(result)))
        return result


class ManualCollection:
    def __init__(self, root, helper, controller_url, token, device_id, view_id='manual', *,
                 journal_path=None, clock=time, camera_factory=ActionRecorder,
                 port_factory=RecordingPort, sample_fps=8):
        if type(sample_fps) is not int or sample_fps not in (4, 8):
            raise ValueError('collection_sample_fps')
        self.sample_fps = sample_fps
        self.root = Path(root).resolve()
        self.helper = Path(helper)
        self.controller_url = validate_target(controller_url)
        self.token, self.device_id, self.view_id = token, device_id, view_id
        self.journal_path = Path(journal_path or REPO/'.local/visual-actions/controller-commands.sqlite')
        self.clock, self.camera_factory, self.port_factory = clock, camera_factory, port_factory
        self.journal = self.camera = self.port = self.store = None
        self.current = None
        self.busy = False
        self.last_poll = 0.
        self.started = None
        self.command_count = 0

    def _event(self, event):
        # SequencePort owns immutable HTTP intent/response files, including failures.
        # This in-memory projection is written with the final session report.
        self.current['events'].append(copy.deepcopy(event))
        if event.get('kind') == 'http' and event.get('path') in ('/status', '/collection_status'):
            self.current['latestStatus'] = copy.deepcopy(event['response'])

    def _journal(self):
        if self.journal is None:
            self.journal = BoundedActionJournal(str(self.journal_path), 10000)
        if not self.journal.readiness()['ready']:
            raise ValueError('legacy_journal_unresolved')
        db = self.journal.connection
        with db:
            db.execute('CREATE TABLE IF NOT EXISTS supervised_blocks (session_id TEXT PRIMARY KEY, plan_hash TEXT NOT NULL, output_path TEXT NOT NULL, phase TEXT NOT NULL, result_json TEXT)')
        if db.execute("SELECT 1 FROM supervised_blocks WHERE phase IN ('pending','off_unverified') LIMIT 1").fetchone():
            raise ValueError('interrupted_collection_requires_readonly_recovery_no_replay')
        return db

    def start(self, payload):
        if self.busy or self.current and self.current['phase'] in ('starting', 'recording', 'finishing'):
            raise ValueError('session_already_active')
        if (not isinstance(payload, dict) or set(payload) != {'joint', 'poseContext', 'onsiteConfirmed'}
                or payload.get('joint') not in PROFILES or payload.get('onsiteConfirmed') is not True
                or not isinstance(payload.get('poseContext'), str) or not 1 <= len(payload['poseContext'].strip()) <= 500):
            raise ValueError('attended_joint_pose_required')
        if not self.device_id or not identifier(self.view_id):
            raise ValueError('camera_identity_required')
        helper_hash = hashlib.sha256(self.helper.read_bytes()).hexdigest()
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        maximum_frames = self.sample_fps*60+20
        budget = maximum_frames * (1920*1080*4+65536)
        if shutil.disk_usage(self.root).free < budget + 2*1024**3:
            raise ValueError('storage_reserve_required')
        db = self._journal()
        sid = 'manual-' + uuid.uuid4().hex
        self.store = ArtifactStore(self.root/sid)
        self.current = dict(schemaVersion=SCHEMA, sessionId=sid, joint=payload['joint'],
                            poseContext=payload['poseContext'], phase='starting', frames=[], events=[],
                            latestStatus=None, error=None, finalIdleOffVerified=False,
                            trainingPerformed=False, datasetEligible=False,
                            timingSemantics='camera_source_bounds_and_http_receipt_brackets_not_controller_clock_sync')
        self.camera = self.port = None
        self.command_count = 0
        self.started = self.clock.monotonic()
        plan = dict(**payload, sessionId=sid, helperSha256=helper_hash, controllerUrl=self.controller_url,
                    camera=dict(deviceId=self.device_id, sessionId=sid, viewId=self.view_id,
                                width=1920, height=1080, maxFrames=maximum_frames, seconds=60, sampleFps=self.sample_fps, maxBytes=budget))
        try:
            self.store.write_json('plan.json', plan)
            with db:
                db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,NULL)',
                           (sid, hashlib.sha256(json.dumps(plan, sort_keys=True).encode()).hexdigest(), str(self.store.root), 'pending'))
        except BaseException:
            self.current['phase'] = 'failed'
            self.current['error'] = 'session_preparation_failed'
            raise
        self.busy = True
        self.camera = self.port = None
        try:
            self.port = self.port_factory(self.controller_url, self.token, 'discovering', self.store,
                                          observer=self._event, clock=self.clock, compact_reads=True, timeout=2)
            try:
                self.port.boot_id = _boot_identity(self.port._call('/actions'))
                self.port.preflight()
            finally:
                # Only initial, read-only discovery gets the longer deadline.
                # Camera failure cleanup and every later operation retain .8s.
                self.port.timeout = .8
            self.current['outputBaselineSequence'] = getattr(self.port, 'last_sequence', None)
            self.camera = self.camera_factory(self.store.root/'native-camera', self.helper, plan['camera'])
            self.camera.start()
            self.camera.wait_ready(timeout=15)
            self.current['phase'] = 'recording'
            self.last_poll = self.clock.monotonic()
            return self.status()
        except BaseException:
            self.current['error'] = 'session_start_failed'
            self._finish(failed=True)
            raise
        finally:
            self.busy = False

    def recover_failed_run_readonly(self, session_id, source_sha256, controller_boot_id):
        """소유한 실패 RUN의 현재 OFF만 확인한다. 원결과·정답·명령은 복구하지 않는다."""
        if (self.busy or self.current is not None or self.port is not None or self.camera is not None
                or self.store is not None or not identifier(session_id)):
            raise ValueError('failed_run_readonly_recovery_idle_owner_required')
        controller_boot_id = _boot_identity({'bootId': controller_boot_id})
        root = self.root/session_id
        raw = _read(self.root, session_id+'/session.json')
        if hashlib.sha256(raw).hexdigest() != source_sha256:
            raise ValueError('failed_run_recovery_source_changed')
        original = _decode_json(raw)
        plan = _decode_json(_read(self.root, session_id+'/plan.json'))
        events = original.get('events', [])
        runs = [e for e in events if e.get('kind') == 'http' and e.get('path') == '/sequence'
                and e.get('payload') == {'action': 'run'} and e.get('response', {}).get('success') is True]
        boots = {e.get('response', {}).get('bootId') for e in events
                 if e.get('kind') == 'http' and e.get('path') == '/actions'}
        if (original.get('schemaVersion') != SCHEMA or original.get('sessionId') != session_id
                or original.get('phase') != 'failed' or original.get('error') != 'command_failed_no_retry'
                or original.get('finalIdleOffVerified') is not False or len(runs) != 1
                or len([e for e in events if e.get('kind') == 'command']) != 1
                or boots != {controller_boot_id} or plan.get('sessionId') != session_id
                or plan.get('controllerUrl') != self.controller_url
                or plan.get('camera', {}).get('deviceId') != self.device_id):
            raise ValueError('failed_run_recovery_owned_source_required')
        if self.journal is None:
            self.journal = BoundedActionJournal(str(self.journal_path), 10000)
        db = self.journal.connection
        expected_plan = hashlib.sha256(json.dumps(plan, sort_keys=True).encode()).hexdigest()
        row = db.execute('SELECT plan_hash,output_path,phase,result_json FROM supervised_blocks WHERE session_id=?',
                         (session_id,)).fetchone()
        if (row is None or row[:3] != (expected_plan, str(root), 'off_unverified') or row[3] is None
                or _decode_json(row[3].encode()) != original or not self.journal.readiness()['ready']
                or db.execute("SELECT session_id FROM supervised_blocks WHERE phase IN ('pending','off_unverified')").fetchall()
                   != [(session_id,)]):
            raise ValueError('failed_run_recovery_owned_block_required')
        self.busy = True
        port = None
        try:
            # 별도 폴더 생성 자체가 이 원본의 단 한 번 복구 시도를 소비한다.
            store = ArtifactStore(root/'readonly-failed-run-recovery')
            port = SequencePort(self.controller_url, self.token, controller_boot_id, store,
                                clock=self.clock, compact_reads=True, timeout=2)
            if _boot_identity(port._call('/actions')) != controller_boot_id:
                raise ValueError('failed_run_recovery_boot_changed')
            snapshot = port.preflight()
            if not port.off_verified or port.may_have_run or port.writes_possible:
                raise ValueError('failed_run_recovery_current_off_not_proven')
            if _read(self.root, session_id+'/session.json') != raw:
                raise ValueError('failed_run_recovery_source_changed')
            store.write_json('proof.json', dict(sessionId=session_id, originalSessionSha256=source_sha256,
                originalResultSha256=hashlib.sha256(row[3].encode()).hexdigest(), snapshot=snapshot,
                currentOffVerified=True, noWrites=True, originalOutcomeChanged=False,
                originalConfigurationRestoreConfirmed=False, datasetEligible=False, replayAllowed=False,
                followupRequiresNewProspectiveChecks=True))
            with db:
                if (not self.journal.readiness()['ready'] or
                    db.execute("SELECT session_id FROM supervised_blocks WHERE phase IN ('pending','off_unverified')").fetchall()
                        != [(session_id,)]):
                    raise ValueError('failed_run_recovery_owned_block_required')
                changed = db.execute('UPDATE supervised_blocks SET phase=? WHERE session_id=? AND plan_hash=? '
                    'AND output_path=? AND phase=? AND result_json=?',
                    ('resolved_current_off_after_failed_run', session_id, expected_plan, str(root), 'off_unverified', row[3]))
                if changed.rowcount != 1:
                    raise ValueError('failed_run_recovery_owned_block_required')
            return dict(sessionId=session_id, currentOffVerified=True, noWrites=True,
                        originalOutcomeChanged=False, replayAllowed=False)
        finally:
            if port is not None:
                port.token = ''
            self.busy = False

    def recover_readonly(self):
        """Retire only this manager's never-started session using fresh GET proof.

        The failed session, its transport and all original evidence stay intact.
        Creating the separate store consumes the sole recovery attempt, even if
        a read or persistence step fails; no cleanup or motion is dispatched.
        """
        if (self.busy or not self.current or self.current.get('phase') != 'failed'
                or self.current.get('error') != 'session_start_failed' or self.command_count != 0
                or self.camera is not None or self.port is None or self.store is None or self.journal is None
                or getattr(self.port, 'may_have_run', None) is not False
                or getattr(self.port, 'writes_possible', None) is not False):
            raise ValueError('readonly_recovery_not_eligible')
        sid, root = self.current['sessionId'], self.store.root
        if (root != self.root/sid or (root/'native-camera').exists()
                or any(root.glob('command-*.json'))
                or any(event.get('kind') in ('command', 'result') or event.get('payload') is not None
                       for event in self.current['events'])):
            raise ValueError('readonly_recovery_write_or_camera_evidence')
        for path in root.glob('http-*-intent.json'):
            intent = _decode_json(_read(root, path.name))
            if intent.get('method') != 'GET' or intent.get('payload') is not None:
                raise ValueError('readonly_recovery_write_or_camera_evidence')
        original = _read(root, 'session.json')
        if _decode_json(original) != self.current:
            raise ValueError('readonly_recovery_session_changed')
        db = self.journal.connection
        row = db.execute('SELECT output_path,phase,result_json FROM supervised_blocks WHERE session_id=?', (sid,)).fetchone()
        if (row is None or row[0] != str(root) or row[1] != 'off_unverified'
                or row[2] is None or _decode_json(row[2].encode()) != self.current):
            raise ValueError('readonly_recovery_owned_failed_row_required')
        if not self.journal.readiness()['ready']:
            raise ValueError('legacy_journal_unresolved')
        self.busy = True
        try:
            proof_store = ArtifactStore(root/'readonly-recovery')
            port = SequencePort(self.controller_url, self.token, 'discovering', proof_store,
                                clock=self.clock, compact_reads=True, timeout=2)
            port.boot_id = _boot_identity(port._call('/actions'))
            snapshot = port.preflight()
            if not port.off_verified or port.may_have_run or port.writes_possible:
                raise ValueError('readonly_recovery_off_not_proven')
            proof = dict(sessionId=sid, offVerified=True, noWrites=True,
                         originalSessionSha256=hashlib.sha256(original).hexdigest(),
                         originalResultSha256=hashlib.sha256(row[2].encode()).hexdigest(),
                         snapshot=snapshot)
            proof_store.write_json('proof.json', proof)
            with db:
                if not self.journal.readiness()['ready']:
                    raise ValueError('legacy_journal_unresolved')
                changed = db.execute('UPDATE supervised_blocks SET phase=? WHERE session_id=? AND output_path=? AND phase=? AND result_json=?',
                    ('resolved_no_run_after_stop', sid, str(root), 'off_unverified', row[2]))
                if changed.rowcount != 1:
                    raise ValueError('readonly_recovery_owned_failed_row_required')
            return dict(sessionId=sid, offVerified=True, noWrites=True)
        finally:
            self.busy = False

    def _validate_pulse(self, payload):
        if not isinstance(payload, dict) or set(payload) != {'joint', 'direction', 'percent', 'durationMs'}:
            raise ValueError('pulse_fields_invalid')
        if payload['joint'] != self.current['joint']:
            raise ValueError('session_joint_is_fixed')
        profile = PROFILES[payload['joint']]
        limits = profile['directions'].get(payload['direction'])
        if limits is None:
            raise ValueError('joint_direction_not_enabled')
        sign, maximum, duration = limits
        if (type(payload['percent']) is not int or not 1 <= payload['percent'] <= maximum
                or type(payload['durationMs']) is not int or not 100 <= payload['durationMs'] <= duration):
            raise ValueError('joint_output_limit')
        if self.command_count >= (1 if payload['joint'] == 'shoulder' else 20):
            raise ValueError('session_command_budget_reached')
        return dict(**payload, motor=profile['motor'], expectedPwm=sign*(payload['percent']*255//100))

    def pulse(self, payload, *, before_run=None, monitor_policy=None):
        """선택적으로 RUN 직전의 비차단 관측·선택 가드를 전달한다."""
        if before_run is not None and not callable(before_run):
            raise ValueError('before_run_callback_required')
        if self.busy or not self.current or self.current['phase'] != 'recording':
            raise ValueError('recording_required')
        pulse = self._validate_pulse(payload)
        validate_monitor_policy(pulse, monitor_policy)
        self.busy = True
        try:
            self.camera.check()
            if self.clock.monotonic()-self.started > 50:
                raise ValueError('recording_near_budget_end')
            self.command_count += 1
            cid = f"{self.current['sessionId']}-{self.command_count:03d}"
            intent = dict(kind='command', at=self.clock.monotonic(), commandId=cid, pulse=pulse)
            if monitor_policy is not None:
                intent['monitorTimingPolicy'] = monitor_policy
                intent['monitorReadBudgetRevision'] = WRIST_READ_BUDGET_REVISION
            if 'validationScopeSha256' in self.current:
                scope_raw = _read(self.store.root, 'commissioning-validation-scope.json')
                if hashlib.sha256(scope_raw).hexdigest() != self.current['validationScopeSha256']:
                    raise ValueError('commissioning_validation_scope_changed')
                intent['validationScopeSha256'] = self.current['validationScopeSha256']
            if 'fixedEvaluationProofSha256' in self.current:
                proof_raw = _read(self.store.root, 'batch-fixed-prospective.json')
                if hashlib.sha256(proof_raw).hexdigest() != self.current['fixedEvaluationProofSha256']:
                    raise ValueError('fixed_evaluation_prospective_changed')
                intent['fixedEvaluationProofSha256'] = self.current['fixedEvaluationProofSha256']
            self.store.write_json(f'command-{self.command_count:03d}-intent.json', intent)
            self._event(intent)
            guard = {} if before_run is None else {'before_run': before_run}
            if monitor_policy is not None:
                guard['monitor_policy'] = monitor_policy
            receipt = self.port.execute(pulse, self.camera.check, **guard)
            event = dict(kind='result', at=self.clock.monotonic(), commandId=cid, receipt=receipt)
            self.store.write_json(f'command-{self.command_count:03d}-result.json', event)
            self._event(event)
            self.last_poll = self.clock.monotonic()
            return self.status()
        except BaseException:
            self.current['error'] = 'command_failed_no_retry'
            self._finish(failed=True)
            raise
        finally:
            self.busy = False

    def tick(self):
        if self.busy or not self.current or self.current['phase'] != 'recording':
            return
        if self.clock.monotonic()-self.last_poll < .5:
            return
        try:
            if self.clock.monotonic()-self.started >= 55:
                self.finish()
                return
            self.camera.check()
            self.port.check_idle(allow_quiet_disconnect_retry=True, allow_quiet_monitor_timeout=True,
                                 allow_quiet_read_recovery=True)
            self.last_poll = self.clock.monotonic()
        except Exception:
            self.current['error'] = 'recording_or_controller_monitor_failed'
            try:
                self._finish(failed=True)
            except Exception:
                # Keep serving replay/status after storage failure; the durable
                # shared pending row continues to block every new collection.
                self.current['phase'] = 'failed'
                self.current['error'] = 'terminal_persistence_failed'

    def _finish(self, failed=False):
        self.current['phase'] = 'finishing'
        off = False
        interrupted = None
        try:
            if self.port is not None:
                # Fresh reads prevent an old successful finish from certifying now.
                self.port.check_idle()
                off = self.port.finish() is True
        except BaseException as exc:
            if not isinstance(exc, Exception):
                interrupted = exc
            if self.port is not None:
                try:
                    off = self.port.finish() is True
                except BaseException as exc:
                    if not isinstance(exc, Exception):
                        interrupted = exc
        recording = None
        if self.camera is not None:
            try:
                recording = self.camera.close()
            except BaseException as exc:
                if not isinstance(exc, Exception):
                    interrupted = exc
                recording = dict(status='failed', reason='recording_close_failed')
            self.current['frames'] = copy.deepcopy(self.camera.frames)
        evidence_failed = self.port is not None and getattr(self.port, 'evidence_failed', False)
        if evidence_failed:
            self.current['error'] = 'evidence_persistence_failed'
        self.current.update(finalIdleOffVerified=off, recording=recording,
                            phase='failed' if failed or evidence_failed or interrupted is not None or not off or not recording or recording.get('status') == 'failed' else 'completed',
                            endedAt=self.clock.monotonic())
        if not off:
            self.current['error'] = self.current['error'] or 'output_off_unverified'
        # Terminal record must be durable BEFORE clearing the shared pending row.
        try:
            self.store.write_json('session.json', self.current)
            with self.journal.connection:
                self.journal.connection.execute('UPDATE supervised_blocks SET phase=?,result_json=? WHERE session_id=?',
                    ('closed' if off else 'off_unverified', json.dumps(self.current, allow_nan=False), self.current['sessionId']))
        except BaseException:
            self.current['phase'] = 'failed'
            self.current['error'] = 'terminal_persistence_failed'
            raise
        if interrupted is not None:
            raise interrupted
        return copy.deepcopy(self.current)

    def finish(self):
        if self.busy:
            raise ValueError('command_in_progress')
        if not self.current:
            raise ValueError('no_session')
        if self.current['phase'] in ('completed', 'failed'):
            return copy.deepcopy(self.current)
        self.busy = True
        try:
            return self._finish()
        finally:
            self.busy = False

    def stop(self):
        if self.busy:
            raise ValueError('command_in_progress')
        if not self.current or self.current['phase'] in ('completed', 'failed'):
            return self.status()
        self.current['error'] = 'operator_stopped'
        return self._finish(failed=True)

    def status(self):
        c = self.current or {}
        frames = self.camera.frames if self.camera is not None else c.get('frames', [])
        return dict(phase=c.get('phase', 'idle'), sessionId=c.get('sessionId'), joint=c.get('joint'),
                    busy=self.busy, frameCount=len(frames), lastFrameIndex=len(frames)-1,
                    error=c.get('error'), latestStatus=copy.deepcopy(c.get('latestStatus')),
                    commandCount=self.command_count, finalIdleOffVerified=c.get('finalIdleOffVerified', False))

    def sessions(self):
        result = []
        if self.root.exists():
            for path in sorted(self.root.iterdir(), reverse=True):
                if path.is_symlink() or not path.is_dir() or not identifier(path.name):
                    continue
                try:
                    report = self.session(path.name)
                    result.append({k: report.get(k) for k in ('sessionId', 'joint', 'phase', 'error', 'finalIdleOffVerified')})
                except (OSError, ValueError, RuntimeError):
                    continue
        return dict(sessions=result)

    def session(self, sid):
        if not identifier(sid):
            raise ValueError('session_id_invalid')
        if self.current and sid == self.current['sessionId']:
            result = copy.deepcopy(self.current)
            if self.camera is not None:
                result['frames'] = copy.deepcopy(self.camera.frames)
            return result
        directory = self.root/sid
        if directory.is_symlink():
            raise ValueError('session_path_invalid')
        if (directory/'session.json').exists():
            return _decode_json(_read(directory, 'session.json'))
        # Interrupted episodes remain visible; never silently resume them.
        plan = _decode_json(_read(directory, 'plan.json'))
        frames, events = [], []
        for path in sorted((directory/'native-camera'/'host-ingress').glob('[0-9][0-9][0-9][0-9].json')):
            frames.append(_decode_json(_read(directory, str(path.relative_to(directory)))))
        for path in sorted(directory.glob('http-*-response.json')):
            response = _decode_json(_read(directory, path.name))
            intent = _decode_json(_read(directory, path.name.replace('-response', '-intent')))
            events.append(dict(kind='http', at=response['receivedAt'], requestStartedAt=intent['startedAt'],
                               responseReceivedAt=response['receivedAt'], path=intent['path'], payload=intent['payload'], response=response['parsed']))
        for path in sorted(directory.glob('http-*-error.json')):
            error = _decode_json(_read(directory, path.name))
            # Retry-attempt evidence may use a longer name, but still belongs to
            # its original request intent. No missing response becomes success.
            prefix = path.name.split('-attempt-')[0].removesuffix('-error.json')
            intent = _decode_json(_read(directory, prefix + '-intent.json'))
            if '-attempt-' in path.name:
                events.append(dict(kind='http_retry', at=error['endedAt'], path=intent['path'],
                                   payload=intent['payload'], attempt=1, evidenceFile=path.name, detail=error))
            else:
                events.append(dict(kind='http_error', at=error['endedAt'], requestStartedAt=intent['startedAt'],
                                   path=intent['path'], payload=intent['payload'], error=error['error']))
        for path in sorted(directory.glob('http-*-attempt-2-intent.json')):
            intent = _decode_json(_read(directory, path.name))
            events.append(dict(kind='http_retry', at=intent['startedAt'], path=intent['path'],
                               payload=intent['payload'], attempt=2, evidenceFile=path.name, detail=intent))
        for path in sorted(directory.glob('command-*.json')):
            events.append(_decode_json(_read(directory, path.name)))
        events.sort(key=lambda event: event['at'])
        baseline = next((event['response']['entries'][-1]['sequence'] for event in events
                         if event.get('kind') == 'http' and event.get('path', '').split('?')[0] == '/action_outputs'
                         and event.get('response', {}).get('entries')
                         and event['response'].get('hasMore', False) is False), None)
        return dict(schemaVersion=SCHEMA, sessionId=sid, joint=plan['joint'], phase='interrupted',
                    error='missing_terminal_record_no_replay', frames=frames, events=events,
                    outputBaselineSequence=baseline,
                    finalIdleOffVerified=False, datasetEligible=False)

    def close(self):
        try:
            if self.current and self.current['phase'] in ('starting', 'recording'):
                self.finish()
        finally:
            if self.journal:
                self.journal.close()
                self.journal = None
