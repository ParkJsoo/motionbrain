#!/usr/bin/env python3
"""Run a finite two-camera policy session on the external capture volume."""
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
from learning.action_port import BoundedActionHTTP
from learning.artifacts import _decode_json
from learning.device_adapter import _object
from learning.dual_view_capture import capture, load_policy, publish_active_context, publish_close_context, side_sample_fps, verified
from learning.dual_view_policy import Catalog, VisualControl, run_actions
from learning.external_media import _mount_fd
from learning.supervised_transport import SequencePort


class IdlePose:
    def __init__(self, port, boot):
        self.port, self.boot, self.cursor = port, boot, None

    def __call__(self):
        with self.port._deadline():
            receipt, _ = self.port.transport.get('controller', '/collection_status')
        s = _object(receipt)
        proof, motors = s.get('outputProof', {}), s.get('motors', {})
        cursor = proof.get('sequenceAfter')
        if (s.get('schemaVersion') != 'collection.v1' or s.get('state') != 'ARMED'
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--observe', action='store_true')
    modes.add_argument('--execute', action='store_true')
    parser.add_argument('--confirm-plan-sha256')
    parser.add_argument('--rerun-url', help='display these same collected frames in an existing Rerun viewer')
    parser.add_argument('--part-overlay', action='store_true', help='show part contours and landmarks in the Rerun viewer')
    args = parser.parse_args(argv)
    if args.part_overlay and not args.rerun_url:
        parser.error('--part-overlay requires --rerun-url')
    raw = args.plan.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    p = _decode_json(raw)
    fps = side_sample_fps(p)
    sync_dispatch = p.get('syncDispatchToFrame', False)
    if type(sync_dispatch) is not bool:
        raise ValueError('invalid_dispatch_sync')
    policy = load_policy(p)
    catalog = Catalog(p['profiles'])
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

    control = VisualControl(latest, p['maxAge'], p['maxSkew'])
    print(json.dumps(dict(planSha256=digest, sideSampleFps=fps, syncDispatchToFrame=sync_dispatch, demonstrated=sorted(set(policy.labels)),
                          commissioned=sorted(catalog.profiles))), flush=True)
    if not (args.observe or args.execute):
        return 0
    if args.execute and (args.confirm_plan_sha256 != digest or not catalog.profiles
                         or not os.environ.get('MOTIONBRAIN_HTTP_TOKEN')):
        raise ValueError('execution_requires_confirmed_plan_profiles_and_token')
    mount, parent = Path(p['mount']), Path(p['captureRoot'])
    fd = _mount_fd(mount, p['volumeUuid'], ROOT)
    try:
        if (not parent.is_relative_to(mount) or parent.resolve(strict=True) != parent
                or parent.stat().st_dev != os.fstat(fd).st_dev):
            raise ValueError('external_capture_root_changed')
        session = uuid.uuid4().hex
        root = parent/session
        root.mkdir(mode=0o700)
        (root/'plan.json').write_bytes(raw)
        print(root, flush=True)
        stop = ctx.Event()
        use_response = 'responsePolicy' in p
        close_context = ctx.Array('c', 8192) if use_response else None
        active_context = ctx.Array('c', 8192) if use_response else None
        child = ctx.Process(target=capture, args=(p, root, session, stop, mailbox, close_context, active_context))
        executor = None
        if args.execute:
            port = BoundedActionHTTP(p['controllerUrl'], token_provider=lambda: os.environ['MOTIONBRAIN_HTTP_TOKEN'],
                                     writes_enabled=True, required_profile='dual_view_timer_v1', timeout=.6, total_deadline=True)
            with port._deadline():
                capability = port.reader.capability()
            if (capability['bootId'] != p['controllerBootId'] or not capability['enabled']
                    or capability['active'] or capability.get('profile') != 'dual_view_timer_v1'):
                raise ValueError('dual_view_timer_owner_unavailable')
            pose = IdlePose(port, p['controllerBootId'])
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
                outcome = 'observation_finished'
            else:
                until = time.monotonic()+20
                while child.is_alive() and latest() is None and time.monotonic() < until:
                    time.sleep(.05)
                with (root/'actions.jsonl').open('x') as log:
                    def record(result):
                        log.write(json.dumps(result)+'\n')
                        log.flush()
                        if result['executionState'] == 'known_terminal':
                            pose.cursor = result['action']['outputSequenceBounds']['last']
                            if use_response and result['actionLabel'] == 'gripper/open':
                                publish_close_context(close_context, session, p['controllerBootId'], None)
                            if use_response and result['actionLabel'] == 'gripper/close':
                                publish_close_context(close_context, session, p['controllerBootId'], result)
                    def on_active(req, vote, at):
                        if vote and vote['action'] == 'gripper/open':
                            publish_close_context(close_context, session, p['controllerBootId'], None)
                        publish_active_context(active_context, session, p['controllerBootId'], req, vote, at)
                    outcome = run_actions(control, catalog, executor, p['controllerBootId'], pose, p['maxActions'],
                                          record, on_active=on_active if use_response else None, sync_dispatch=sync_dispatch)
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
                   outcome not in ('action_budget', 'hold_or_uncommissioned', 'observation_finished'))
    finally:
        os.close(fd)


if __name__ == '__main__':
    raise SystemExit(main())
