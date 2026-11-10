#!/usr/bin/env python3
"""전체 관절 묶음 수집. 기본은 파일·계획 검사만 하며 실제 실행은 명시적으로 선택한다."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from learning.batch_campaign import Campaign, digest, validate_plan
from learning.batch_runtime import BatchRuntime, private_path, validate_runtime
from learning.macos_credentials import load_keychain_secret


def inspect_plan(path, repository=ROOT):
    path = Path(path).resolve(strict=True)
    if not path.is_relative_to((Path(repository)/'.local').resolve()):
        raise ValueError('private_batch_plan_required')
    plan = json.loads(path.read_bytes())
    checked = validate_plan(plan)
    runtime = validate_runtime(plan, repository)
    # Construction validates runtime files but never opens a camera or keychain.
    driver = BatchRuntime(plan, repository, path.parent/'runtime', lambda: None)
    ready, rejected = [], list(checked['rejected'])
    for condition in checked['ready']:
        reason = driver.validate_condition(condition)
        if reason:
            rejected.append(dict(conditionId=condition['id'], reason=reason))
        else:
            ready.append(condition)
    return plan, dict(status='plan_checked_no_device_access', planSha256=digest(plan),
                      ready=ready, rejected=rejected, motorCommands=0,
                      runtimeStatus=runtime['status'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--plan-sha')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--onsite-confirmed', action='store_true')
    parser.add_argument('--keychain-service', default='MotionBrain.local.controller')
    parser.add_argument('--keychain-account', default='esp32-controller')
    args = parser.parse_args(argv)
    plan, checked = inspect_plan(args.plan)
    if args.plan_sha is not None and args.plan_sha != checked['planSha256']:
        parser.error('계획 SHA가 일치하지 않습니다')
    if not args.execute:
        print(json.dumps(checked, ensure_ascii=False))
        return checked
    if not args.plan_sha or not args.onsite_confirmed:
        parser.error('실행에는 고정 계획 SHA와 현재 현장 확인이 필요합니다')
    if not checked['ready']:
        parser.error('실행 가능한 조건이 없습니다')
    output = args.output or args.plan.resolve().parent/'runtime'
    output = private_path(ROOT, str(output))
    original_hash = checked['planSha256']
    def credential():
        return load_keychain_secret(args.keychain_service, args.keychain_account)
    driver = BatchRuntime(plan, ROOT, output, credential)
    def stop(*_):
        raise KeyboardInterrupt()
    previous = {sig:signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    campaign = None
    try:
        # Recheck plan file after argument processing and before constructing the
        # state machine. No changed file may retain the earlier authorization.
        current = json.loads(args.plan.read_bytes())
        if digest(current) != original_hash:
            raise ValueError('batch_plan_changed_before_start')
        campaign = Campaign(plan, output, driver,
                            lock_path=private_path(ROOT, plan['runtime']['dispatchLockPath']))
        result = campaign.run()
        print(json.dumps(result, ensure_ascii=False))
        return result
    finally:
        try:
            driver.close()
        finally:
            try:
                if campaign is not None:
                    campaign.close()
            finally:
                for sig, handler in previous.items():
                    signal.signal(sig, handler)


if __name__ == '__main__':
    main()
