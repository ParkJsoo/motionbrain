#!/usr/bin/env python3
"""명시한 수집 폴더의 저장 파일만 Rerun에 표시한다. 카메라·제어기 요청 없음."""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time
import urllib.parse

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from learning.artifacts import StorageError, _decode_json, _read
from tools.visualization.pipeline import native_image

ERRORS = (ValueError, OSError, KeyError, TypeError, StorageError)


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def private(path):
    path = Path(path).resolve()
    require(path.is_relative_to((ROOT/'.local').resolve()), 'private_collection_path_required')
    return path


def read_json(root, name):
    return _decode_json(_read(root, name))


def _session(sessions):
    if not sessions.exists():
        return None
    if (sessions/'plan.json').is_file():
        return sessions
    candidates = [p for p in sessions.iterdir() if p.is_dir() and not p.is_symlink()
                  and (p/'plan.json').is_file()]
    return max(candidates, key=lambda p: ((p/'plan.json').stat().st_mtime_ns, p.name)) if candidates else None


def _latest_frame(session, plan):
    camera = session/'native-camera'
    paths = sorted(camera.glob('frame-*.json'), reverse=True)
    if not paths:
        return None
    path = paths[0]
    meta = read_json(camera, path.name)
    require(meta['schemaVersion'] == 'motionbrain.mac_camera_frame.v2', 'native_frame_contract_changed')
    expected = plan['camera']; index = meta['frameIndex']
    require(type(index) is int and index >= 0 and meta['sessionId'] == plan['sessionId']
            and meta['sourceFrameId'] == f'{plan["sessionId"]}:{index}', 'native_frame_identity_changed')
    require(meta['deviceUniqueId'] == expected['deviceId']
            and meta['viewConfigurationId'] == expected['viewId']
            and meta['width'] == expected['width'] and meta['height'] == expected['height'],
            'native_camera_context_changed')
    w, h, raw, stored = (meta[k] for k in ('width','height','rawByteLength','storedByteLength'))
    require(all(type(v) is int for v in (w,h,raw,stored)) and 0<w<=1920 and 0<h<=1080
            and raw == w*h*4 and 0<stored<=raw+65536, 'native_frame_size_invalid')
    require(meta['storageEncoding'] == 'apple-lzbitmap-bgra8-v1', 'native_frame_encoding_changed')
    # _read rejects absolute paths, traversal, symlinks and non-regular files.
    payload = _read(camera, meta['imageFile'])
    require(len(payload) == stored and hashlib.sha256(payload).hexdigest() == meta['imageSha256'],
            'native_stored_frame_changed')
    image = native_image(session, meta)
    return dict(metadata=meta, image=image, fileTime=path.stat().st_mtime)


def snapshot(sessions, *, now=None, stale_seconds=3.):
    """Rerun와 장치 접근 없이 한 번 읽는다. 오래된 이미지는 갱신 중에 숨긴다."""
    require(type(stale_seconds) in (int,float) and math.isfinite(stale_seconds) and stale_seconds>0,
            'positive_stale_seconds_required')
    now = time.time() if now is None else now
    sessions = Path(sessions)
    result = dict(status=dict(running=True, followerPhase='waiting', source='collector_files_only',
        operationPath='manual_collection', modelSelectionVerified=False, cameraRequests=0,
        controllerRequests=0, motorCommands=0,
        gripperDisplay='이 표시기는 외부 영상 표시; 집게 전후 관측은 사후 파이프라인에서 확인',
        M4Display='수집 파일에 유효한 센서 값 없음',
        viewerImageEncoding='jpeg85_display_only_original_hashes_in_status',
        synchronizationVerified=False, label='수집 폴더 대기 — 실시간 노출 동기화 아님'),
        image=None, imageKey=None, sensor=None)
    info = result['status']
    try:
        session = _session(sessions)
        if session is None:
            return result
        plan = read_json(session, 'plan.json')
        require(plan['sessionId'] == session.name, 'collection_session_identity_changed')
        info.update(session=plan['sessionId'], sessionPath=str(session), joint=plan['joint'],
                    collectorPhase='recording_or_unclosed')
        terminal = None
        if (session/'session.json').exists():
            terminal = read_json(session, 'session.json')
            require(terminal['sessionId'] == plan['sessionId'], 'terminal_session_identity_changed')
            info['collectorPhase'] = terminal.get('phase', 'unknown')
            info['collectorError'] = terminal.get('error')
            info['finalIdleOffVerified'] = terminal.get('finalIdleOffVerified', False)
        ended = info['collectorPhase'] in ('completed', 'failed')
        try:
            frame = _latest_frame(session, plan)
            if frame:
                meta, written = frame['metadata'], frame['fileTime']
                age = now-written
                require(age >= -1., 'frame_file_clock_in_future')
                info.update(externalFrameId=meta['sourceFrameId'], frameIndex=meta['frameIndex'],
                    pixelSha256=meta['pixelSha256'], imageSha256=meta['imageSha256'],
                    cameraContext={k:meta.get(k) for k in
                        ('deviceUniqueId','viewConfigurationId','configurationSha256','width','height')},
                    sourceHostSeconds=meta['convertedHostPtsSeconds'], fileAgeSeconds=max(0.,age),
                    lastFrameFileTime=datetime.datetime.fromtimestamp(written,datetime.timezone.utc).isoformat())
                if ended:
                    info.update(followerPhase='last_recorded', label='수집 종료 — 마지막 저장 영상, 현재 실시간 영상 아님')
                elif age>stale_seconds:
                    info.update(followerPhase='stale', label='파일 갱신 지연 — 영상 숨김, 종료 여부 미확인')
                else:
                    info.update(followerPhase='updating', label='수집기 파일 갱신 중 — 저장 파일을 따라 표시')
                if ended or age<=stale_seconds:
                    result['image'] = frame['image']
                    result['imageKey'] = (plan['sessionId'], meta['frameIndex'], meta['pixelSha256'])
            else:
                info.update(followerPhase='missing', label='완전한 외부 영상 파일 없음')
        except ERRORS as exc:
            info.update(followerPhase='invalid_frame', label='최신 외부 영상 검증 실패 — 영상 숨김', frameError=str(exc))
        intents = sorted(session.glob('command-*-intent.json'))
        results = list(session.glob('command-*-result.json'))
        info.update(commandIntents=len(intents), commandResults=len(results))
        if intents:
            info['lastRecordedCommand'] = read_json(session,intents[-1].name).get('pulse')
        for response in sorted(session.glob('http-*-response.json'),reverse=True)[:20]:
            try:
                payload = read_json(session,response.name)
                angle = payload.get('parsed',{}).get('shoulderAngle',{})
                value = angle.get('angleDeg')
                if (angle.get('sensorFresh') is True and angle.get('sensorReady') is True
                    and type(value) in (int,float) and math.isfinite(value)):
                    info.update(M4LastRecordedDeg=value, M4ResponseHostSeconds=payload['receivedAt'],
                                M4Display='마지막 파일에 기록된 값 — 현재 센서 직접 조회 아님')
                    result['sensor'] = value
                    break
            except ERRORS:
                continue
    except ERRORS as exc:
        info.update(followerPhase='error', label='수집 파일 읽기 오류 — 영상 숨김', reason=str(exc))
        result['image'] = result['imageKey'] = result['sensor'] = None
    return result


def _status(output, info):
    pending = output/'status.pending'
    pending.write_text(json.dumps(info,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    pending.replace(output/'status.json')


def follow(sessions, output, *, connect=None, seconds=0., interval=.5, rr_module=None, stop=None):
    sessions, output = private(sessions), private(output)
    require(type(seconds) in (int,float) and math.isfinite(seconds) and seconds>=0,
            'nonnegative_seconds_required')
    require(type(interval) in (int,float) and math.isfinite(interval) and interval>0,
            'positive_interval_required')
    if connect:
        url = urllib.parse.urlsplit(connect)
        require(url.scheme=='rerun+http' and url.hostname in ('localhost','127.0.0.1')
                and url.username is None and url.password is None, 'explicit_local_rerun_connection_required')
    output.mkdir(parents=True,exist_ok=True)
    rrd = output/'collection.rrd'
    require(not rrd.exists(), 'new_collection_rrd_required')
    real_sdk = rr_module is None
    if real_sdk:
        import rerun as rr_module
    rr = rr_module
    rec = rr.RecordingStream('motionbrain_collection_file_follower')
    if connect:
        rec.set_sinks(rr.FileSink(str(rrd)),rr.GrpcSink(connect))
    else:
        rec.save(str(rrd))
    if real_sdk:
        import rerun.blueprint as bp
        rec.send_blueprint(bp.Blueprint(bp.Vertical(bp.Spatial2DView(origin='/external'),
            bp.Horizontal(bp.TimeSeriesView(origin='/sensors'),bp.TextDocumentView(origin='/status')))))
    started = time.monotonic(); last_image = None; last_session = None; info = dict(running=True)
    try:
        while not (stop and stop()) and (seconds==0 or time.monotonic()-started<seconds):
            view = snapshot(sessions)
            info = view['status'];info['sessionsRoot'] = str(sessions)
            rec.set_time('viewer_file_time',timestamp=time.time())
            if info.get('session') != last_session:
                rec.log('external',rr.Clear(recursive=True))
                rec.log('sensors',rr.Clear(recursive=True))
                last_image=None;last_session=info.get('session')
            if view['image'] is None:
                rec.log('external',rr.Clear(recursive=True));last_image=None
            elif view['imageKey'] != last_image:
                rec.log('external',rr.Image(view['image']).compress(jpeg_quality=85));last_image=view['imageKey']
            if view['sensor'] is not None:
                rec.log('sensors/M4_last_recorded_deg',rr.Scalars(view['sensor']))
            rec.log('status',rr.TextDocument(json.dumps(info,ensure_ascii=False,indent=2)))
            _status(output,info)
            time.sleep(interval)
    finally:
        info.update(running=False, followerPhase='stopped', label='파일 표시기 종료 — 실시간 표시 아님')
        _status(output,info);rec.log('status',rr.TextDocument(json.dumps(info,ensure_ascii=False,indent=2)))
        rec.flush();rec.disconnect()
    return dict(output=str(rrd),cameraRequests=0,controllerRequests=0,motorCommands=0)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--connect')
    parser.add_argument('--seconds',type=float,default=0.)
    args=parser.parse_args(argv)
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        result=follow(args.sessions,args.output,connect=args.connect,seconds=args.seconds,stop=lambda:stopped)
    except (ValueError,OSError,StorageError,ImportError) as exc:
        parser.error(str(exc))
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
