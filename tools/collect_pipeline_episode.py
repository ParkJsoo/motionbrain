#!/usr/bin/env python3
"""참석자가 지정한 단발 수집. 기본은 계획 검사만 하며 자동 반복·학습·제어는 없다."""
import argparse
import getpass
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.manual_collection import ManualCollection
from learning.pipeline_collection import collect_once
from learning.relative_action_learning import command_key
from learning.supervised_collection import identifier


def verified_helper(plan):
    helper = (ROOT/plan['helperPath']).resolve(strict=True)
    if not helper.is_relative_to(ROOT) or hashlib.sha256(helper.read_bytes()).hexdigest() != plan['helperSha256']:
        raise ValueError('collection_helper_hash_changed')
    if not os.access(helper, os.X_OK):
        raise ValueError('collection_helper_not_executable')
    return helper


def recheck_plan(path, digest, plan, manager):
    if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
        raise ValueError('collection_plan_hash_changed')
    helper = verified_helper(plan)
    if helper != manager.helper:
        raise ValueError('collection_helper_binding_changed')


def validate(path, expected=None):
    raw = Path(path).read_bytes(); digest = hashlib.sha256(raw).hexdigest()
    if expected is not None and digest != expected:
        raise ValueError('collection_plan_hash_changed')
    plan = json.loads(raw)
    fields = {'schemaVersion', 'helperPath', 'helperSha256', 'controllerUrl', 'deviceId',
              'viewId', 'sessionRoot', 'requestRoot', 'poseContext', 'command'}
    if not isinstance(plan, dict) or set(plan) != fields or plan['schemaVersion'] != 'motionbrain.pipeline_collection_plan.v1':
        raise ValueError('explicit_collection_plan_required')
    command_key(plan['command'])
    if not isinstance(plan['deviceId'], str) or not plan['deviceId'].strip():
        raise ValueError('collection_device_identity_required')
    if not identifier(plan['viewId']):
        raise ValueError('collection_view_identity_required')
    if not isinstance(plan['poseContext'], str) or not 1 <= len(plan['poseContext'].strip()) <= 500:
        raise ValueError('explicit_pose_context_required')
    helper = verified_helper(plan)
    for key in ('sessionRoot', 'requestRoot'):
        if not (ROOT/plan[key]).resolve().is_relative_to(ROOT/'.local'):
            raise ValueError('private_collection_paths_required')
    # Existing constructor validates URL and sampling settings without opening a device.
    manager = ManualCollection(ROOT/plan['sessionRoot'], helper, plan['controllerUrl'], '',
                               plan['deviceId'], plan['viewId'], sample_fps=8)
    return plan, digest, manager


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--plan-sha')
    parser.add_argument('--request-id')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--onsite-confirmed', action='store_true')
    args = parser.parse_args(argv)
    plan, digest, manager = validate(args.plan, args.plan_sha)
    if not args.execute:
        print(json.dumps({'status': 'plan_checked_no_device_access', 'planSha256': digest,
                          'motorCommands': 0}, ensure_ascii=False))
        return
    if not args.plan_sha or not args.request_id or not args.onsite_confirmed or not sys.stdin.isatty():
        parser.error('실행에는 고정 계획 SHA·고유 요청 ID·현재 현장 확인·비공개 TTY가 필요합니다')
    try:
        manager.token = getpass.getpass('장치 인증값(메모리에서만 사용): ')
        recheck_plan(args.plan, digest, plan, manager)
        report = collect_once(manager, args.request_id, plan['command'], plan['poseContext'],
                              onsite_confirmed=True, request_root=ROOT/plan['requestRoot'],
                              approval_plan_sha256=digest)
        print(json.dumps(report, ensure_ascii=False))
    finally:
        try:
            manager.close()
        finally:
            manager.token = ''
            if manager.port:
                manager.port.token = ''


if __name__ == '__main__':
    main()
