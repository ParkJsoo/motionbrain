#!/usr/bin/env python3
"""Run a finite two-camera policy session on the selected capture volume."""
from contextlib import contextmanager, nullcontext
import argparse
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.action_execution import BoundedActionExecutor
from learning.action_lookup_worker import ActionLookupWorker
from learning.action_port import BoundedActionHTTP
from learning.artifacts import _decode_json
from learning.device_adapter import _object
from learning.dual_view_capture import (capture, load_policy, publish_active_context, publish_candidates,
                                        publish_close_context, side_sample_fps, verified)
from learning.dual_view_policy import Catalog, GripperDispatchGate, VisualControl, run_actions
from learning.external_media import _mount_fd
from learning import shoulder_entry_measurement
from learning.supervised_transport import SequencePort


class IdlePose:
    def __init__(self, port, boot, *, shoulder_entry=False, state='ARMED'):
        self.port, self.boot, self.cursor = port, boot, None
        self.shoulder_entry = shoulder_entry
        self.state = state

    def __call__(self):
        if self.shoulder_entry:
            with self.port._deadline():
                status = _object(self.port.transport.get('controller', '/status')[0])
            shoulder = status['shoulderAngle']
            if (shoulder.get('motorDefaultSpeed') != shoulder_entry_measurement.DEFAULT_SPEED
                    or shoulder.get('boundedDownCorrectionEnabled') is not False
                    or shoulder.get('manualCalibrationUnlock') is not False):
                raise ValueError('shoulder_measurement_output_limit_unverified')
        with self.port._deadline():
            receipt, _ = self.port.transport.get('controller', '/collection_status')
        s = _object(receipt)
        proof, motors = s.get('outputProof', {}), s.get('motors', {})
        cursor = proof.get('sequenceAfter')
        if (s.get('schemaVersion') != 'collection.v1' or s.get('state') != self.state
                or proof.get('bootId') != self.boot or proof.get('schemaVersion') != 'driver_cursor.v1'
                or proof.get('stable') is not True or type(cursor) is not int or not 0 <= cursor < 2**32
                or type(proof.get('sequenceBefore')) is not int or proof['sequenceBefore'] != cursor
                or self.cursor is not None and self.cursor != cursor
                or set(motors) != {f'M{i}' for i in range(1, 6)}
                or any(m.get('enabled') is not False or type(m.get('speed')) is not int
                       or m['speed'] != 0 for m in motors.values())):
            raise ValueError('all_off_epoch_unverified')
        self.cursor = cursor
        return SequencePort._shoulder_angle(s), receipt['receivedMonotonic']


@contextmanager
def capture_parent(plan):
    parent = Path(plan['captureRoot'])
    storage = plan.get('captureStorage', 'external')
    if storage == 'local':
        base = ROOT / '.local'
        if (base not in parent.parents or parent.resolve(strict=True) != parent
                or not parent.is_dir() or parent.stat().st_dev != ROOT.stat().st_dev):
            raise ValueError('local_capture_root_changed')
        yield parent
    elif storage == 'external':
        mount = Path(plan['mount'])
        fd = _mount_fd(mount, plan['volumeUuid'], ROOT)
        try:
            if (not parent.is_relative_to(mount) or parent.resolve(strict=True) != parent
                    or parent.stat().st_dev != os.fstat(fd).st_dev):
                raise ValueError('external_capture_root_changed')
            yield parent
        finally:
            os.close(fd)
    else:
        raise ValueError('invalid_capture_storage')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--observe', action='store_true')
    modes.add_argument('--execute', action='store_true')
    parser.add_argument('--allow-observation-stop', action='store_true',
                        help='allow an empty stop-observation file in this capture directory to finish --observe')
    parser.add_argument('--confirm-plan-sha256')
    parser.add_argument('--measure-shoulder-entry', action='store_true',
                        help='use the separately gated single shoulder calibration step')
    parser.add_argument('--rerun-url', help='display these same collected frames in an existing Rerun viewer')
    parser.add_argument('--part-overlay', action='store_true', help='show part contours and landmarks in the Rerun viewer')
    args = parser.parse_args(argv)
    if args.allow_observation_stop and not args.observe:
        parser.error('--allow-observation-stop requires --observe')
    if args.part_overlay and not args.rerun_url:
        parser.error('--part-overlay requires --rerun-url')
    raw = args.plan.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    p = _decode_json(raw)
    if args.measure_shoulder_entry != ('shoulderEntryMeasurement' in p):
        raise ValueError('shoulder_entry_measurement_requires_explicit_mode_and_plan')
    profiles = shoulder_entry_measurement.validate_plan(p) if args.measure_shoulder_entry else p['profiles']
    pose_aware = p.get('poseAwareCandidates', False)
    if type(pose_aware) is not bool or (pose_aware and (
            args.measure_shoulder_entry or 'responsePolicy' not in p or 'developmentPick' in p)):
        raise ValueError('invalid_pose_aware_candidates')
    if 'forearmApproachObserver' in p and not args.observe:
        parser.error('forearmApproachObserver requires --observe')
    if 'wristHeightProposal' in p and (not args.observe or 'developmentPick' not in p):
        parser.error('wristHeightProposal requires --observe and developmentPick')
    if 'phaseObserver' in p and 'developmentPick' not in p and not args.observe:
        parser.error('phaseObserver requires --observe')
    fps = side_sample_fps(p)
    sync_dispatch = p.get('syncDispatchToFrame', False)
    if type(sync_dispatch) is not bool:
        raise ValueError('invalid_dispatch_sync')
    ack_aligned_capture = p.get('ackAlignedGripperCapture', False)
    if type(ack_aligned_capture) is not bool or (ack_aligned_capture and sync_dispatch):
        raise ValueError('invalid_gripper_dispatch_alignment')
    async_lookup = p.get('asyncActionLookup', False)
    if type(async_lookup) is not bool:
        raise ValueError('invalid_async_action_lookup')
    refresh_expired_dispatch = p.get('refreshExpiredDispatch', False)
    if type(refresh_expired_dispatch) is not bool:
        raise ValueError('invalid_dispatch_refresh')
    dispatch_reserve = p.get('minDispatchVisualReserve', 0.)
    if dispatch_reserve and not ack_aligned_capture:
        raise ValueError('dispatch_visual_reserve_requires_ack_alignment')
    policy = load_policy(p)
    catalog = Catalog(profiles)
    verified(Path(p['helper']), p['helperSha256'])
    if (type(p['seconds']) is not int or not 2 <= p['seconds'] <= 120 or
            type(p['maxActions']) is not int or not 1 <= p['maxActions'] <= 32):
        raise ValueError('invalid_session_budget')
    ctx = multiprocessing.get_context('spawn')
    mailbox = ctx.Array('c', 8192)

    def latest():
        lock = mailbox.get_lock()
        if not lock.acquire(timeout=.02):
            return {'fault': True, 'action': 'hold'}
        try:
            data = mailbox.get_obj().value
        finally:
            lock.release()
        return json.loads(data) if data else None

    control = VisualControl(latest, p['maxAge'], p['maxSkew'], dispatch_reserve=dispatch_reserve)
    print(json.dumps(dict(planSha256=digest, sideSampleFps=fps, syncDispatchToFrame=sync_dispatch,
                          ackAlignedGripperCapture=ack_aligned_capture, asyncActionLookup=async_lookup,
                          refreshExpiredDispatch=refresh_expired_dispatch,
                          minDispatchVisualReserve=control.dispatch_reserve,
                          demonstrated=sorted(set(policy.labels)),
                          commissioned=sorted(catalog.profiles))), flush=True)
    if not (args.observe or args.execute):
        return 0
    if args.execute and (args.confirm_plan_sha256 != digest or not catalog.profiles
                         or not os.environ.get('MOTIONBRAIN_HTTP_TOKEN')):
        raise ValueError('execution_requires_confirmed_plan_profiles_and_token')
    with capture_parent(p) as parent:
        session = uuid.uuid4().hex
        root = parent/session
        root.mkdir(mode=0o700)
        (root/'plan.json').write_bytes(raw)
        print(root, flush=True)
        stop = ctx.Event()
        use_response = 'responsePolicy' in p
        close_context = ctx.Array('c', 8192) if use_response else None
        active_context = ctx.Array('c', 8192) if use_response else None
        candidate_context = ctx.Array('c', 8192) if pose_aware else None
        def on_off_pose(labels, verified_off_at):
            publish_candidates(candidate_context, session, p['controllerBootId'], labels, verified_off_at)
        if pose_aware and args.observe:
            read_port = BoundedActionHTTP(p['controllerUrl'], timeout=.6, total_deadline=True)
            read_pose = IdlePose(read_port, p['controllerBootId'], state='IDLE')
            angle, checked_at = read_pose()
            on_off_pose(catalog.eligible_at(angle), checked_at)
        gripper_gate = GripperDispatchGate(ctx.Event(), ctx.Event()) if args.execute and ack_aligned_capture else None
        child = ctx.Process(target=capture, args=(p, root, session, stop, mailbox, close_context, active_context),
                            kwargs={'allow_observation_stop':args.allow_observation_stop,'gripper_gate':gripper_gate,
                                    'shoulder_entry':args.measure_shoulder_entry,'candidate_mailbox':candidate_context})
        executor = None
        if args.execute:
            port = BoundedActionHTTP(p['controllerUrl'], token_provider=lambda: os.environ['MOTIONBRAIN_HTTP_TOKEN'],
                                     writes_enabled=True, required_profile='dual_view_timer_v1', timeout=.6,
                                     total_deadline=True, allow_rolled_history=True)
            with port._deadline():
                capability = port.reader.capability()
            if (capability['bootId'] != p['controllerBootId'] or not capability['enabled']
                    or capability['active'] or capability.get('profile') != 'dual_view_timer_v1'):
                raise ValueError('dual_view_timer_owner_unavailable')
            pose = IdlePose(port, p['controllerBootId'], shoulder_entry=args.measure_shoulder_entry)
            pose()
            executor = BoundedActionExecutor(ROOT/'.local'/'dual-view-actions.sqlite', port, catalog, max_polls=200)
        def interrupt(signum, frame):
            raise KeyboardInterrupt()
        viewer = None
        previous = signal.signal(signal.SIGTERM, interrupt)
        try:
            if args.rerun_url:
                viewer_log = (root/'display.jsonl').open('x')
                try:
                    viewer_args = [sys.executable, str(ROOT/'tools/visualization/collector_live.py'),
                                   '--source', str(root), '--rerun-url', args.rerun_url]
                    if args.part_overlay:
                        viewer_args.append('--part-overlay')
                    viewer = subprocess.Popen(viewer_args, stdout=viewer_log)
                finally:
                    viewer_log.close()
            child.start()
            if args.observe:
                child.join()
                outcome = 'observation_failed' if args.allow_observation_stop and child.exitcode != 0 else 'observation_finished'
            else:
                measurement_status = None
                until = time.monotonic()+20
                while child.is_alive() and latest() is None and time.monotonic() < until:
                    time.sleep(.05)
                lookup_context = ActionLookupWorker(p['controllerUrl']) if async_lookup else nullcontext()
                with lookup_context as lookup_worker, (root/'actions.jsonl').open('x') as log:
                    def record(result):
                        nonlocal measurement_status
                        log.write(json.dumps(result)+'\n')
                        log.flush()
                        if result['executionState'] == 'known_terminal':
                            pose.cursor = result['action']['outputSequenceBounds']['last']
                            if args.measure_shoulder_entry:
                                with (root/'shoulder-entry-measurement.json').open('x') as evidence:
                                    try:
                                        measurement = shoulder_entry_measurement.observe_entry(
                                            control, pose, result, profiles[shoulder_entry_measurement.LABEL]['command'])
                                    except Exception as exc:
                                        json.dump(dict(commandId=result['commandId'], status='post_off_unverified',
                                                       reason=str(exc)), evidence)
                                        raise
                                    json.dump(measurement, evidence, allow_nan=False)
                                    measurement_status = measurement['status']
                            if use_response and result['actionLabel'] == 'gripper/open':
                                publish_close_context(close_context, session, p['controllerBootId'], None)
                            if use_response and result['actionLabel'] == 'gripper/close':
                                publish_close_context(close_context, session, p['controllerBootId'], result)
                    def on_active(req, vote, at):
                        if vote and vote['action'] == 'gripper/open':
                            publish_close_context(close_context, session, p['controllerBootId'], None)
                        publish_active_context(active_context, session, p['controllerBootId'], req, vote, at,
                                               max_age=control.age if gripper_gate is not None else None)
                    side_stop = None
                    if p['external']['viewConfigurationId'] == 'wide-side-manual-20261121':
                        from learning.wide_side_stop import WideSideStop
                        side_stop = WideSideStop(root, p['external'], p['maxAge'])
                    outcome = run_actions(control, catalog, executor, p['controllerBootId'], pose, p['maxActions'],
                                          record, on_active=on_active if use_response else None, sync_dispatch=sync_dispatch,
                                          gripper_gate=gripper_gate, side_stop=side_stop,
                                          finish_after='elbow/up' if 'developmentPick' in p else None,
                                          lookup_worker=lookup_worker, refresh_expired_dispatch=refresh_expired_dispatch,
                                          on_off_pose=on_off_pose if pose_aware else None)
                if measurement_status is not None and outcome in ('action_budget', 'device_stopped'):
                    outcome = measurement_status
            if outcome == 'observe_after_lift':
                # Retain fresh OFF observations for scoring without issuing further commands.
                child.join(timeout=3)
            print(outcome, flush=True)
            viewer_failed = viewer is not None and viewer.poll() is not None
        finally:
            stop.set()
            if child.pid is not None:
                child.join()
            if viewer is not None:
                viewer.terminate()
                try:
                    viewer.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    viewer.kill()
                    viewer.wait()
            if executor:
                executor.close()
            signal.signal(signal.SIGTERM, previous)
        return int(child.exitcode != 0 or viewer_failed or
                   (args.execute and args.measure_shoulder_entry and outcome != 'measured_in_close_range') or
                   outcome not in ('action_budget', 'hold_or_uncommissioned', 'observation_finished', 'observe_after_lift',
                                   'measured_in_close_range'))


if __name__ == '__main__':
    raise SystemExit(main())
