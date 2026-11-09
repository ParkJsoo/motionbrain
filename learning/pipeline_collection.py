"""단발 수집 수명주기. 원본을 먼저 닫고 관측·학습은 별도 pipeline에서 수행한다.

manager는 기존 ManualCollection이며 이 모듈은 출력 한도·OFF·journal을 변경하지
않는다. 생성·import·요청 재조회에서는 장치에 접근하지 않는다.
"""
from __future__ import annotations

import hashlib
import json
import copy
from pathlib import Path
import re

from learning.experiment_pipeline import atomic, encoded, sync_directory
from learning.relative_action_learning import command_key
from learning.artifacts import ArtifactStore, _read


def collect_once(manager, request_id, command, pose_context, *, onsite_confirmed,
                 request_root, pre_seconds=3.5, post_seconds=6.5, approval_plan_sha256=None,
                 gripper_source_factory=None):
    """명시적 요청당 최대 한 번 pulse한다. 오류·부분 수집·중복은 자동 재전송하지 않는다.

    수동 자료 수집용이며 모델 선택 제어기가 아니다. 요청 ID는 호출자가 지정한다.
    진행 중/불명 요청은 같은 ID로 다시 호출해도 구동되지 않는다.
    """
    if not isinstance(request_id, str) or re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', request_id) is None:
        raise ValueError('explicit_safe_request_id_required')
    if gripper_source_factory is not None and not callable(gripper_source_factory):
        raise ValueError('gripper_source_factory_required')
    command = copy.deepcopy(command)
    command_key(command)
    if not isinstance(pose_context, str) or not 1 <= len(pose_context.strip()) <= 500:
        raise ValueError('explicit_pose_context_required')
    if onsite_confirmed is not True:
        raise ValueError('current_onsite_confirmation_required')
    if approval_plan_sha256 is not None and (
            not isinstance(approval_plan_sha256, str)
            or re.fullmatch(r'[0-9a-f]{64}', approval_plan_sha256) is None):
        raise ValueError('approval_plan_sha256_required')
    if (type(pre_seconds) not in (int, float) or not 3.5 <= pre_seconds <= 8
            or type(post_seconds) not in (int, float) or not 6.5 <= post_seconds <= 12):
        raise ValueError('bounded_complete_capture_windows_required')
    request = {'requestId': request_id, 'command': command, 'poseContext': pose_context,
               'preSeconds': pre_seconds, 'postSeconds': post_seconds,
               'deviceId': manager.device_id, 'viewId': manager.view_id,
               'controllerUrl': manager.controller_url}
    if approval_plan_sha256 is not None:
        request['approvalPlanSha256'] = approval_plan_sha256
    if gripper_source_factory is not None:
        request['pairedGripperCapture'] = True
    binding = hashlib.sha256(encoded(request)).hexdigest()
    root = Path(request_root).resolve()
    missing = []; parent = root
    while not parent.exists():
        missing.append(parent); parent = parent.parent
    root.mkdir(parents=True, exist_ok=True)
    for created in reversed(missing):
        sync_directory(created.parent)
    path = root/(request_id+'.json')
    try:
        with path.open('x') as out:
            json.dump({'requestSha256': binding, 'status': 'reserved', 'mayHaveRun': True,
                       'automaticRetry': False, 'request': request}, out, ensure_ascii=False)
            out.flush()
            import os
            os.fsync(out.fileno())
        sync_directory(root)
    except FileExistsError:
        saved = json.loads(path.read_text())
        if saved.get('requestSha256') != binding:
            raise ValueError('request_id_payload_changed')
        return saved
    report = {'requestSha256': binding, 'request': request, 'automaticRetry': False,
              'modelSelected': False, 'status': 'unknown', 'sessionId': None,
              'offVerified': False, 'observation': 'not_processed', 'trainingPerformed': False,
              'gripperPair': {'status': 'not_requested' if gripper_source_factory is None else 'pending'}}
    gripper = pair_store = None

    def paired_capture(label):
        record = gripper.capture(deadline=manager.clock.monotonic()+6.)
        gripper.verify(record)
        if record['hostClockId'] != manager.current['sessionId']:
            raise ValueError('paired_camera_session_changed')
        binding = gripper._bindings[record['frame']['frameId']]
        for suffix in ('.jpg', '.json'):
            pair_store._write('esp32-'+label+suffix,
                _read(gripper.store.root, binding['folder']+'/esp32-frame'+suffix))
        report['gripperPair'][label] = record

    def link_source():
        sid = (manager.current or {}).get('sessionId')
        if sid:
            report['sessionId'] = sid
            report['sessionPath'] = str(manager.root/sid)
            source = manager.root/sid/'session.json'
            if source.is_file():
                report['sourceSha256'] = hashlib.sha256(source.read_bytes()).hexdigest()

    def wait_until(deadline):
        while manager.clock.monotonic() < deadline:
            manager.tick()
            if manager.current['phase'] != 'recording':
                raise ValueError('recording_ended_no_retry')
            manager.clock.sleep(.08)

    try:
        manager.start({'joint': command['joint'], 'poseContext': pose_context, 'onsiteConfirmed': True})
        report['sessionId'] = manager.current['sessionId']
        report['status'] = 'recording'
        atomic(path, encoded(report))
        if gripper_source_factory is not None:
            try:
                gripper = gripper_source_factory(report['sessionId'], ArtifactStore(manager.store.root/'gripper-camera'))
                if gripper.session_id != report['sessionId']:
                    raise ValueError('paired_camera_session_changed')
                pair_store = ArtifactStore(manager.store.root/'gripper-pair')
                report['gripperPair']['path'] = str(pair_store.root)
                gripper.start(deadline=manager.clock.monotonic()+6.)
                paired_capture('before')
            except Exception as exc:
                report['gripperPair'].update(status='rejected', reason='before:'+type(exc).__name__+':'+str(exc))
                raise
        wait_until(manager.clock.monotonic()+pre_seconds)
        manager.pulse(command)
        receipt = [e['receipt'] for e in manager.current['events'] if e.get('kind') == 'result'][-1]
        wait_until(receipt['offAt']+post_seconds)
        if gripper is not None:
            try:
                paired_capture('after')
                report['gripperPair']['status'] = 'recorded'
            except Exception as exc:
                # Camera/processing failure never shortens the external raw post window.
                report['gripperPair'].update(status='rejected', reason='after:'+type(exc).__name__+':'+str(exc))
        session = manager.finish()
        off = session['finalIdleOffVerified'] is True
        report.update(status=('unknown_output' if not off else
                              'recorded' if session['phase'] == 'completed' else 'failed'),
                      offVerified=off)
        if session.get('error'):
            report['reason'] = session['error']
    except BaseException as exc:
        report['reason'] = type(exc).__name__+':'+str(exc)
        # Completed raw collection must never be reclassified because a later processor failed.
        if manager.current and manager.current.get('phase') in ('starting', 'recording', 'finishing'):
            try:
                manager.stop()
            except BaseException:
                pass  # The manager's durable journal continues to block unresolved output.
        session = manager.current or {}
        report.update(status='interrupted' if isinstance(exc, (KeyboardInterrupt, SystemExit)) else 'failed',
                      offVerified=session.get('finalIdleOffVerified', False))
        if not report['offVerified']:
            report['status'] = 'unknown_output'
        link_source()
        atomic(path, encoded(report))
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        return report
    finally:
        if gripper is not None:
            gripper.close()
    link_source()
    atomic(path, encoded(report))
    return report
