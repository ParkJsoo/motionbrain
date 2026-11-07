"""Explicit, single-use M4-up commissioning core. No live CLI or default observer.

General collection restrictions remain unchanged. The caller must supply current
dual-camera acquisition/review and a private runtime token; this module never
discovers devices, extracts tokens, fits a model or selects a grasp policy.
"""
from __future__ import annotations

import copy
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import re
import time

from learning.action_journal import BoundedActionJournal
from learning.artifacts import _decode_json, _read
from learning.gripper_camera_dataset import read_frame
from learning.lossless_frame import ENCODING, SUFFIX, decode
from learning.mac_clock_bridge import validate_sample
from learning.manual_collection_dataset import require
from learning.supervised_collection import identifier, plan_digest
from learning import supervised_transport as transport

REPO = Path(__file__).resolve().parents[1]
SCHEMA = 'motionbrain.m4_up_commissioning.v1'
PULSE = dict(joint='shoulder', motor=4, direction='up', percent=16,
             durationMs=200, expectedPwm=40)
MAX_CONTROLLER_PROOF_AGE_SECONDS = .25


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def implementation_digest():
    return sha(Path(__file__).read_bytes()+b'\0'+Path(transport.__file__).read_bytes())


def _hash(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def validate_scope(scope):
    require(isinstance(scope, dict) and set(scope) == {
        'schemaVersion', 'trialId', 'controllerUrl', 'controllerBootId', 'hostClockId',
        'implementationSha256', 'context', 'pulse', 'defaultSpeed', 'maxCommands',
        'expectedStartAngleDeg', 'startAngleToleranceDeg', 'maxObservedDeltaDeg',
        'maxSeconds', 'maxObservationAgeSeconds', 'postOffSeconds'}, 'm4_scope_fields')
    require(scope['schemaVersion'] == SCHEMA and identifier(scope['trialId'])
            and identifier(scope['hostClockId']) and isinstance(scope['controllerBootId'], str)
            and re.fullmatch('[0-9a-f]{32}', scope['controllerBootId']) is not None,
            'm4_scope_identity')
    transport.validate_target(scope['controllerUrl'])
    require(scope['implementationSha256'] == implementation_digest(), 'm4_implementation_changed')
    require(scope['pulse'] == PULSE and all(type(scope['pulse'][k]) is type(v) for k, v in PULSE.items())
            and type(scope['defaultSpeed']) is int and scope['defaultSpeed'] == 255
            and type(scope['maxCommands']) is int and scope['maxCommands'] == 1,
            'm4_exact_single_pulse_required')
    for k, value in [('startAngleToleranceDeg', .5), ('maxObservedDeltaDeg', 1.),
                     ('maxSeconds', 60.), ('maxObservationAgeSeconds', 10.), ('postOffSeconds', 3.)]:
        require(transport.finite_number(scope[k]) and scope[k] == value, 'm4_fixed_limits_required')
    require(transport.finite_number(scope['expectedStartAngleDeg'])
            and 127.58 < scope['expectedStartAngleDeg'] < 296.12, 'm4_start_angle_invalid')
    ctx = scope['context']
    require(isinstance(ctx, dict) and set(ctx) == {
        'physicalView', 'poseRegime', 'object', 'supportMode', 'externalDeviceId',
        'externalViewId', 'gripperCameraBootId'} and ctx['supportMode'] == 'table_supported'
        and all(isinstance(v, str) and bool(v) for v in ctx.values())
        and re.fullmatch('[0-9a-f]{32}', ctx['gripperCameraBootId']) is not None,
        'm4_current_contact_context_required')
    return copy.deepcopy(scope)


def _external_frame(root, source, scope):
    require(isinstance(source, dict) and set(source) == {'root', 'ingressFile', 'ingressSha256', 'clockSha256'}
            and _hash(source['ingressSha256']) and _hash(source['clockSha256']), 'm4_external_source_fields')
    prefix = source['root']+'/'
    raw = _read(root, prefix+source['ingressFile'])
    require(sha(raw) == source['ingressSha256'], 'm4_external_ingress_changed')
    f = _decode_json(raw)
    require(type(f.get('frameIndex')) is int and f['frameIndex'] >= 0
            and source['ingressFile'] == f"host-ingress/{f['frameIndex']:04d}.json"
            and f['frameId'] == f"{scope['hostClockId']}:{f['frameIndex']}"
            and f['qualityFlags'] == [] and f['metadataFile'] == f"frame-{f['frameIndex']:04d}.json"
            and f['imageFile'] == f"frame-{f['frameIndex']:04d}{SUFFIX}",
            'm4_external_frame_invalid')
    meta_raw = _read(root, prefix+f['metadataFile']); image = _read(root, prefix+f['imageFile'])
    require(sha(meta_raw) == f['metadataSha256'] and sha(image) == f['imageSha256'], 'm4_external_source_changed')
    meta = _decode_json(meta_raw)
    require(meta.get('schemaVersion') == 'motionbrain.mac_camera_frame.v2'
            and meta.get('clockDomain') == 'coremedia_host_clock'
            and meta.get('timestampSemantics') == 'cmsamplebuffer_presentation_timestamp'
            and meta.get('storageEncoding') == ENCODING and f.get('storageEncoding') == ENCODING
            and meta.get('sessionId') == scope['hostClockId'] and meta.get('sourceFrameId') == f['frameId']
            and type(meta.get('frameIndex')) is int and meta['frameIndex'] == f['frameIndex']
            and meta.get('deviceUniqueId') == scope['context']['externalDeviceId']
            and meta.get('viewConfigurationId') == scope['context']['externalViewId']
            and meta.get('imageFile') == f['imageFile'] and meta.get('imageSha256') == f['imageSha256']
            and meta.get('pixelSha256') == f['pixelSha256'] and meta.get('rawByteLength') == f['rawByteLength']
            and type(f['width']) is int and type(f['height']) is int
            and 1 <= f['width'] <= 1920 and 1 <= f['height'] <= 1080
            and meta.get('width') == f['width'] and meta.get('height') == f['height']
            and meta.get('pixelFormat') == 'BGRA8' and meta.get('rowBytes') == f['width']*4
            and type(meta.get('videoMirrored')) is bool and transport.finite_number(meta.get('videoRotationAngle'))
            and len(image) == meta.get('storedByteLength') == f.get('storedByteLength')
            and f['rawByteLength'] == f['width']*f['height']*4, 'm4_external_metadata_invalid')
    pts = meta.get('pts', {})
    require(isinstance(pts, dict) and all(type(pts.get(k)) is int for k in ('value','timescale','epoch','flags'))
            and pts['value'] >= 0 and pts['timescale'] > 0 and pts['epoch'] == 0
            and pts['flags'] & 1 and not pts['flags'] & ~3, 'm4_external_pts_invalid')
    native, callback = meta.get('convertedHostPtsSeconds'), meta.get('callbackHostSeconds')
    persistence, read_at = f.get('persistenceMs'), f.get('stdoutReadAt')
    require(all(transport.finite_number(v) for v in (native, callback, persistence, read_at))
            and 0 <= native <= callback and 0 <= persistence <= 250
            and f.get('convertedHostPtsSeconds') == native and f.get('callbackHostSeconds') == callback,
            'm4_external_receipt_invalid')
    require(isinstance(meta.get('actualConfigurationJson'), str)
            and sha(meta['actualConfigurationJson'].encode()) == meta.get('configurationSha256'),
            'm4_external_configuration_invalid')
    decode(image, f['rawByteLength'], f['pixelSha256'])
    clock_raw = _read(root, prefix+'host-ingress/clock-before.json')
    require(sha(clock_raw) == source['clockSha256'], 'm4_external_clock_changed')
    clock = _decode_json(clock_raw); low, high = validate_sample(clock)
    require(clock['hostClockId'] == scope['hostClockId'] and clock['pairs'][-1]['after'] <= f['sourceEarliest']
            and f['clockOffsetBoundsSeconds'] == [low, high]
            and f['sourceEarliest'] == meta['convertedHostPtsSeconds']+low
            and f['sourceLatest'] == meta['convertedHostPtsSeconds']+high
            and f['sourceLatest'] <= read_at+.01 and 0 <= read_at-f['sourceEarliest'] <= 2
            and clock['pairs'][-1]['after'] <= f['sourceEarliest'] <= clock['pairs'][-1]['after']+120,
            'm4_external_clock_binding')
    return dict(frameId=f['frameId'], frameIndex=f['frameIndex'], pts=[pts['value'],pts['timescale']],
                pixelSha256=f['pixelSha256'], width=f['width'], height=f['height'],
                videoMirrored=meta['videoMirrored'], videoRotationAngle=meta['videoRotationAngle'],
                configurationSha256=meta['configurationSha256'],
                sourceEarliest=f['sourceEarliest'], sourceLatest=f['sourceLatest'],
                persistedAt=f['stdoutReadAt']+f['persistenceMs']/1000)


def _gripper_frame(root, source, scope, label):
    require(isinstance(source, dict) and set(source) == {'root', 'label', 'metadataSha256', 'imageSha256'}
            and source['label'] == label, 'm4_gripper_source_fields')
    prefix = source['root']+'/'
    meta = _read(root, prefix+f'esp32-{label}.json'); image = _read(root, prefix+f'esp32-{label}.jpg')
    require(sha(meta) == source['metadataSha256'] and sha(image) == source['imageSha256'], 'm4_gripper_source_changed')
    location = (root/source['root']).resolve(strict=True)
    require(location.is_relative_to(root), 'm4_private_source_required')
    frame, receipt, boot = read_frame(location, label, scope['hostClockId'])
    require(boot == scope['context']['gripperCameraBootId'] and frame['imageSha256'] == sha(image),
            'm4_gripper_boot_or_bytes_changed')
    require(sha(_read(root, prefix+f'esp32-{label}.json')) == sha(meta), 'm4_gripper_receipt_changed')
    return dict(frameId=frame['frameId'], pixelSha256=frame['pixelSha256'], capturedUs=frame['capturedUs'],
                width=frame['width'], height=frame['height'],
                sourceEarliest=receipt['requestStartedAt'], sourceLatest=receipt['receivedAt'],
                persistedAt=receipt['persistedAt'])


class _CommissioningPort(transport.SequencePort):
    def __init__(self, scope, token, store, owner, *, clock):
        super().__init__(scope['controllerUrl'], token, scope['controllerBootId'], store,
                         clock=clock, compact_reads=True)
        self.owner = owner
        self.proving_off = False
        self.proof_started = {}

    def execute(self, *args, **kwargs):
        raise ValueError('m4_use_scoped_commissioning_only')

    def _execute_validated(self, pulse, tick, *, before_run=None):
        self.owner._guard()
        require(self.owner.in_pulse and pulse == PULSE, 'm4_scope_not_active')
        return super()._execute_validated(pulse, tick, before_run=before_run)

    def _call(self, path, payload=None, **kwargs):
        if not kwargs.get('emergency'):
            self.owner._guard()
            remaining = self.owner.deadline-self.clock.monotonic()
            kwargs['request_timeout'] = min(kwargs.get('request_timeout', remaining), remaining)
        return super()._call(path, payload, **kwargs)

    def _snapshot(self, *, emergency=False):
        old = self.proving_off; self.proving_off = emergency
        try: return super()._snapshot(emergency=emergency)
        finally: self.proving_off = old

    def _status(self, **kwargs):
        started = self.clock.monotonic()
        result = super()._status(**kwargs)
        self.proof_started['status'] = started
        return result

    def _outputs(self, **kwargs):
        started = self.clock.monotonic()
        result = super()._outputs(**kwargs)
        self.proof_started['outputs'] = started
        return result

    def _check_status(self, status, expected, pulse=None, baseline_angle=None):
        super()._check_status(status, expected, pulse, baseline_angle)
        angle = self._shoulder_angle(status)
        self.owner.angles.append(dict(at=self.clock.monotonic(), angleDeg=angle, state=status['state']))
        if self.proving_off: return  # Original OFF proof stays distinct from a local excursion failure.
        scope = self.owner.scope
        if not self.may_have_run:
            require(abs(angle-scope['expectedStartAngleDeg']) <= scope['startAngleToleranceDeg'], 'm4_start_context_changed')
        if self.owner.baseline_angle is not None:
            delta = angle-self.owner.baseline_angle
            require(abs(delta) <= scope['maxObservedDeltaDeg'] and delta >= -.25, 'm4_observed_excursion')


class M4UpCommissioning:
    """One fixed manual identification attempt, not a learned lift controller.

    Observer: start(deadline), observe(after_off,deadline,tick), latest(), check(),
    close(). observe returns a durable review plus actual native/gripper sources;
    latest/check must be nonblocking and must never capture or perform HTTP.
    Only run() acquires the fixed shared journal and may contact a controller.
    """
    def __init__(self, scope, observer, token, store, *, evidence_root, clock=time):
        self.scope = validate_scope(scope); self.digest = plan_digest(self.scope)
        self.observer, self.store, self.clock = observer, store, clock
        self.root = Path(evidence_root).resolve(strict=True)
        require(self.root.is_relative_to((REPO/'.local').resolve())
                and Path(store.root).resolve().is_relative_to((REPO/'.local').resolve()), 'm4_private_evidence_required')
        self.port = _CommissioningPort(self.scope, token, store, self, clock=clock)
        self.used = self.in_pulse = False
        self.journal = None; self.baseline_angle = None; self.angles = []

    def _guard(self):
        require(self.clock.monotonic() < self.deadline, 'm4_trial_deadline')
        require(plan_digest(self.scope) == self.digest
                and self.scope['implementationSha256'] == implementation_digest(), 'm4_scope_or_code_changed')
        require(self.journal is not None and self.journal.readiness()['ready'], 'm4_owner_missing')
        rows = self.journal.connection.execute("SELECT session_id,plan_hash,output_path FROM supervised_blocks WHERE phase IN ('pending','off_unverified')").fetchall()
        require(rows == [(self.scope['trialId'], self.digest, str(self.store.root))], 'm4_owner_changed')

    def _observe(self, record, *, after_off=None):
        require(isinstance(record, dict) and set(record) == {
            'hostClockId', 'context', 'external', 'gripper', 'stable', 'onsite', 'evidenceFile', 'evidenceSha256'},
            'm4_dual_observation_required')
        require(record['hostClockId'] == self.scope['hostClockId'] and record['context'] == self.scope['context']
                and record['stable'] is True and _hash(record['evidenceSha256']), 'm4_observation_context_changed')
        raw = _read(self.root, record['evidenceFile'])
        require(len(raw) <= 65536 and sha(raw) == record['evidenceSha256']
                and _decode_json(raw) == {k: v for k, v in record.items() if k not in ('evidenceFile','evidenceSha256')},
                'm4_review_changed')
        onsite = record['onsite']
        require(isinstance(onsite, dict) and set(onsite) == {'operatorPresent', 'clearanceVerified', 'jawContactVerified', 'tableSupportVerified'}
                and all(type(v) is bool for v in onsite.values()) and onsite['operatorPresent'] and onsite['clearanceVerified'],
                'm4_onsite_review_required')
        if after_off is None:
            require(onsite['jawContactVerified'] and onsite['tableSupportVerified'], 'm4_initial_contact_required')
        frames = {'external': _external_frame(self.root, record['external'], self.scope),
                  'gripper': _gripper_frame(self.root, record['gripper'], self.scope, 'before' if after_off is None else 'after')}
        for frame in frames.values():
            times = [frame[k] for k in ('sourceEarliest','sourceLatest','persistedAt')]
            now = self.clock.monotonic()
            require(all(transport.finite_number(v) for v in times) and self.started <= times[0]
                    and times == sorted(times) and times[-1] <= now, 'm4_observation_time_invalid')
            if after_off is None:
                require(now-times[0] <= self.scope['maxObservationAgeSeconds'] and times[-1] <= now-.1,
                        'm4_before_observation_not_ready')
            else:
                require(times[0] >= after_off+self.scope['postOffSeconds'], 'm4_post_off_observation_too_early')
        return frames

    def _tick(self, idle=False):
        self._guard(); self.observer.check(); self._guard()
        if idle: self.port.check_idle()

    def _before_run(self):
        started = self.clock.monotonic(); self._tick()
        self._require_fresh_controller_proof(started)
        latest = self.observer.latest()
        require(latest == self.before, 'm4_input_changed_before_run')
        frames = self._observe(latest)
        self._guard()
        self.store.write_json('pre-run-guard.json', dict(scopeSha256=self.digest, frames=frames,
            controllerProofStarted=copy.deepcopy(self.port.proof_started),
            validatedAt=self.clock.monotonic(), selectionKind='manual_commissioning_fixed_profile'))
        self._guard()
        now = self.clock.monotonic()
        require(now-started <= .1 and now < self.deadline
                and all(now-frame['sourceEarliest'] <= self.scope['maxObservationAgeSeconds']
                        for frame in frames.values()), 'm4_pre_run_guard_stale')
        self._require_fresh_controller_proof(now)

    def _require_fresh_controller_proof(self, now):
        require(set(self.port.proof_started) == {'status','outputs'}
                and all(0 <= now-at <= MAX_CONTROLLER_PROOF_AGE_SECONDS
                        for at in self.port.proof_started.values()), 'm4_controller_proof_stale')

    def run(self, *, confirmed_scope_sha256):
        require(not self.used and confirmed_scope_sha256 == self.digest, 'm4_exact_unconsumed_scope_required')
        self.used = True; self.started = self.clock.monotonic(); self.deadline = self.started+self.scope['maxSeconds']
        report = dict(schemaVersion=SCHEMA, scopeSha256=self.digest, status='failed', errors=[],
            selectionKind='manual_commissioning_fixed_profile', mayHaveRun=False,
            finalIdleOffVerified=False, trainingEligible=False, modelSelected=False, graspVerified=False, liftVerified=False)
        observer_started = claimed = False; phase = 'journal'
        try:
            self.journal = BoundedActionJournal(str(REPO/'.local/visual-actions/controller-commands.sqlite'), 10000)
            require(self.journal.readiness()['ready'], 'm4_shared_journal_unresolved')
            db = self.journal.connection
            with db:
                db.execute('CREATE TABLE IF NOT EXISTS supervised_blocks (session_id TEXT PRIMARY KEY, plan_hash TEXT NOT NULL, output_path TEXT NOT NULL, phase TEXT NOT NULL, result_json TEXT)')
                require(not db.execute("SELECT 1 FROM supervised_blocks WHERE phase IN ('pending','off_unverified') LIMIT 1").fetchone(), 'm4_shared_block_unresolved')
                require(not db.execute('SELECT 1 FROM supervised_blocks WHERE session_id=?', (self.scope['trialId'],)).fetchone(), 'm4_scope_consumed')
                require(db.execute('SELECT COUNT(*) FROM supervised_blocks').fetchone()[0] < 10000, 'm4_scope_capacity')
                db.execute('INSERT INTO supervised_blocks VALUES (?,?,?,?,NULL)', (self.scope['trialId'],self.digest,str(self.store.root),'pending'))
            claimed = True
            self.store.write_json('scope.json', self.scope)
            phase = 'preflight'; snapshot = self.port.preflight()
            self.baseline_angle = self.port._shoulder_angle(snapshot['status'])
            phase = 'before_observation'; observer_started = True
            self.observer.start(deadline=self.deadline)
            self.before = self.observer.observe(after_off=None, deadline=self.deadline, tick=lambda: self._tick(True))
            before_frames = self._observe(self.before); self._guard()
            self.store.write_json('before-observation.json', self.before)
            phase = 'pulse'; self.in_pulse = True
            try: receipt = self.port._execute_validated(copy.deepcopy(PULSE), self._tick, before_run=self._before_run)
            finally: self.in_pulse = False
            report['receipt'] = receipt
            phase = 'after_observation'
            after = self.observer.observe(after_off=receipt['offAt'], deadline=self.deadline, tick=lambda: self._tick(True))
            frames = self._observe(after, after_off=receipt['offAt'])
            require(all(frames[k]['frameId'] != before_frames[k]['frameId'] for k in frames)
                    and all([frames[k]['width'], frames[k]['height']] == [before_frames[k]['width'], before_frames[k]['height']] for k in frames)
                    and frames['external']['configurationSha256'] == before_frames['external']['configurationSha256']
                    and all(frames['external'][k] == before_frames['external'][k] for k in ('videoMirrored','videoRotationAngle'))
                    and frames['external']['frameIndex'] > before_frames['external']['frameIndex']
                    and Fraction(*frames['external']['pts']) > Fraction(*before_frames['external']['pts'])
                    and frames['gripper']['capturedUs'] > before_frames['gripper']['capturedUs'], 'm4_duplicate_or_reversed_after')
            self.store.write_json('after-observation.json', after); self._guard()
            report['status'] = 'completed'
        except (Exception, KeyboardInterrupt) as exc:
            report['errors'].append(dict(phase=phase, type=type(exc).__name__))
        finally:
            if claimed:
                if observer_started:
                    try:
                        report['observerClose'] = self.observer.close()
                        require(isinstance(report['observerClose'], dict) and report['observerClose'].get('status') == 'completed', 'm4_observer_close_failed')
                    except (Exception, KeyboardInterrupt) as exc:
                        report['errors'].append(dict(phase='observer_close', type=type(exc).__name__))
                try:
                    if self.port.off_verified and not self.port.evidence_failed: self.port.check_idle()
                except (Exception, KeyboardInterrupt) as exc:
                    report['errors'].append(dict(phase='final_monitor', type=type(exc).__name__))
                try: report['finalIdleOffVerified'] = self.port.finish() is True
                except (Exception, KeyboardInterrupt) as exc:
                    report['errors'].append(dict(phase='finish', type=type(exc).__name__))
                report.update(mayHaveRun=self.port.may_have_run, angleObservations=self.angles)
                if report['errors'] or not report['finalIdleOffVerified']: report['status'] = 'failed'
                report['commandOutcome'] = ('not_started' if not report['mayHaveRun'] else
                    'receipt_recorded' if 'receipt' in report else 'unknown')
                report['requiresOperatorReview'] = (report['commandOutcome'] == 'unknown'
                                                    or not report['finalIdleOffVerified'])
                # OFF proof does not resolve a lost RUN reply or failed execution.
                # Keep the shared block pending even for a new ID until reviewed.
                terminal_phase = ('off_unverified' if not report['finalIdleOffVerified'] else
                                  'pending' if report['commandOutcome'] == 'unknown' else 'closed')
                try:
                    self.store.write_json('result.json', report)
                    with self.journal.connection:
                        self.journal.connection.execute('UPDATE supervised_blocks SET phase=?,result_json=? WHERE session_id=?',
                            (terminal_phase,json.dumps(report,allow_nan=False),self.scope['trialId']))
                finally: self.journal.close(); self.journal = None
            elif self.journal is not None:
                self.journal.close(); self.journal = None
        return report
