#!/usr/bin/env python3
"""두 카메라·관측점·실측 M4를 Rerun에 표시한다. 모터·학습·영구 영상 저장 없음."""
from __future__ import annotations

import argparse
import datetime
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import urllib.parse
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from learning.artifacts import ArtifactStore
from learning.grasp_observation import GripperObservationSource, _NoRedirect
from learning.live_observation import LatestFeed, read_preview, require, shoulder_sample
from learning.mac_clock_bridge import MacHostClock, validate_sample
from learning.registered_scene_landmarks import RegisteredSceneLandmarks


def source(path):
    p = (ROOT / path).resolve(strict=True)
    require(p.is_relative_to(ROOT), 'source_outside_repository')
    return p


def origin(value):
    p = urllib.parse.urlsplit(value)
    require(p.scheme == 'http' and p.hostname and not p.username and not p.password
            and p.path in ('', '/') and not p.query and not p.fragment, 'plain_http_origin_required')
    return value.rstrip('/')


def get_json(base, endpoint):
    require(endpoint in ('/status', '/actions'), 'read_only_endpoint_required')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())
    with opener.open(base+endpoint, timeout=2.) as response:
        raw = response.read(65537)
    require(len(raw) <= 65536, 'status_response_oversized')
    return json.loads(raw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=0., help='0이면 Ctrl-C까지 표시')
    args = parser.parse_args()
    require(args.seconds >= 0, 'nonnegative_duration_required')
    config = json.loads(args.config.read_text())
    external, gripper, controller = (config[k] for k in ('external', 'gripper', 'controller'))
    external_tracker = RegisteredSceneLandmarks(json.loads(source(external['registration']).read_text()),
        source(external['reference']).read_bytes())
    gripper_tracker = RegisteredSceneLandmarks(json.loads(source(gripper['registration']).read_text()),
        source(gripper['reference']).read_bytes())
    expected = json.loads(source(external['metadata']).read_text())
    require(external_tracker.registration['imageShape'] == [expected['height'], expected['width']],
            'external_registration_shape_changed')
    gripper_origin, controller_origin = origin(gripper['url']), origin(controller['url'])
    require(get_json(controller_origin, '/actions')['bootId'] == controller['bootId'], 'controller_boot_changed')
    require(get_json(gripper_origin, '/status')['cameraBootId'] == gripper['bootId'], 'camera_boot_changed')
    output = (ROOT / config['output']).resolve()
    require(output.is_relative_to(ROOT / '.local'), 'private_runtime_directory_required')
    output.mkdir(exist_ok=False)
    import cv2
    import numpy as np
    import rerun as rr
    import rerun.blueprint as bp
    rec = rr.RecordingStream('motionbrain_live_observation', recording_id=output.name)
    rec.connect_grpc(config['rerunUrl'])
    rec.send_blueprint(bp.Blueprint(bp.Vertical(
        bp.Horizontal(bp.Spatial2DView(origin='/cameras/external'), bp.Spatial2DView(origin='/cameras/gripper')),
        bp.Horizontal(bp.TimeSeriesView(origin='/sensors'), bp.TextDocumentView(origin='/status')))))
    feeds = {name: LatestFeed() for name in ('external', 'gripper', 'M4')}
    stop = threading.Event()
    started = time.monotonic()
    offset, _ = validate_sample(MacHostClock().sample(output.name))
    process = subprocess.Popen([str(source(config['helper'])), external['deviceId'], '2'],
        stdout=subprocess.PIPE, stderr=(output/'external.stderr').open('xb'))

    def observe(tracker, raw, frame_id):
        try:
            return tracker.observe(raw, frame_id), None
        except ValueError as exc:
            return None, str(exc)

    def external_worker():
        previous = -1
        while not stop.is_set():
            h, pixels, at = read_preview(process.stdout, expected, previous, time.monotonic, offset)
            previous = h['sequence']
            image = np.frombuffer(pixels, np.uint8).reshape(h['height'], h['width'], 4)
            ok, raw = cv2.imencode('.png', image)
            require(ok, 'preview_encoding_failed')
            observation, error = observe(external_tracker, raw.tobytes(), f'{output.name}:external:{previous}')
            feeds['external'].publish(dict(sourceAt=at, image=cv2.cvtColor(image, cv2.COLOR_BGRA2RGB),
                observation=observation, observationError=error, droppedFrames=h['droppedFrames']))

    previous_gripper = None
    scene = dict(gripper_tracker.registration['sceneContext'], landmarks=gripper_tracker.landmark_contract)

    def gripper_once():
        nonlocal previous_gripper
        # 기존 절대 요청 기한은 메인 스레드의 SIGALRM을 사용한다.
        # 한 번에 한 장만 임시 검증하며 학습·실험 원본으로 취급하지 않는다.
        with tempfile.TemporaryDirectory(dir=output, prefix='preview-') as folder:
            camera = GripperObservationSource(gripper_origin, output.name, gripper['bootId'],
                scene, ArtifactStore(Path(folder)/'frames'), timeout=2., max_captures=1)
            try:
                camera.start(deadline=time.monotonic()+2.5)
                item = camera.capture(deadline=time.monotonic()+2.5)
                camera.verify(item)
                frame_id = item['frame']['frameId']
                require(previous_gripper is None or int(frame_id.split(':')[-1]) > previous_gripper,
                        'camera_frame_regressed')
                previous_gripper = int(frame_id.split(':')[-1])
                raw = (Path(folder)/'frames/capture-0001/esp32-frame.jpg').read_bytes()
                image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                require(image is not None, 'gripper_image_invalid')
                observation, error = observe(gripper_tracker, raw, frame_id)
                feeds['gripper'].publish(dict(sourceAt=item['sourceEarliest'], image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB),
                    observation=observation, observationError=error))
            finally:
                camera.close()

    def sensor_worker():
        previous = None
        while not stop.is_set():
            at = time.monotonic()
            status = get_json(controller_origin, '/status')
            require(get_json(controller_origin, '/actions')['bootId'] == controller['bootId'],
                    'controller_boot_changed')
            angle = shoulder_sample(status)
            require(previous is None or status['uptimeMs'] >= previous, 'controller_restarted')
            previous = status['uptimeMs']
            feeds['M4'].publish(dict(sourceAt=at, angleDeg=angle))
            stop.wait(1.)

    def guarded(name, worker):
        try:
            worker()
        except Exception as exc:
            if not stop.is_set():
                feeds[name].fail(str(exc))
                if name == 'external':
                    process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()

    workers = [threading.Thread(target=guarded, args=(name, worker), daemon=True)
        for name, worker in [('external', external_worker), ('M4', sensor_worker)]]
    for worker in workers:
        worker.start()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stop.set())
    logged, phases = {}, {}
    last_status, next_gripper = 0., 0.
    try:
        while not stop.is_set() and (not args.seconds or time.monotonic()-started < args.seconds):
            now = time.monotonic()
            if now >= next_gripper and feeds['gripper'].failure is None:
                guarded('gripper', gripper_once)
                next_gripper = time.monotonic()+.5
                now = time.monotonic()
            states = {}
            for name, feed in feeds.items():
                state, item = feed.snapshot(now, 2.5 if name == 'M4' else 1.5)
                states[name] = state
                if item is not None:
                    if name == 'M4':
                        state['angleDeg'] = item['angleDeg']
                    else:
                        state['droppedFrames'] = item.get('droppedFrames', 0)
                rec.set_time('host_elapsed', duration=now-started)
                if item is None:
                    if phases.get(name) != state['phase']:
                        rec.log('sensors/M4_angle_deg' if name == 'M4' else 'cameras/'+name, rr.Clear(recursive=True))
                elif item['ordinal'] != logged.get(name):
                    rec.set_time('host_elapsed', duration=max(0., item['sourceAt']-started))
                    if name == 'M4':
                        rec.log('sensors/M4_angle_deg', rr.Scalars(item['angleDeg']))
                        state['angleDeg'] = item['angleDeg']
                    else:
                        rec.log('cameras/'+name, rr.Image(item['image']).compress(jpeg_quality=85))
                        observation = item['observation']
                        path = 'cameras/'+name+'/landmarks'
                        if observation:
                            points = observation['annotation']['points']
                            rec.log(path, rr.Points2D(list(points.values()), labels=list(points), radii=3))
                        else:
                            rec.log(path, rr.Clear(recursive=True))
                        state['droppedFrames'] = item.get('droppedFrames', 0)
                    logged[name] = item['ordinal']
                phases[name] = state['phase']
            if now-last_status >= 1.:
                status = dict(updatedAt=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    running=True, sources=states, controllerWrites=0, trainingRuns=0,
                    recording=False, synchronizedCameras=False, controlEligible=False)
                rec.set_time('host_elapsed', duration=now-started)
                rec.log('status', rr.TextDocument('LIVE READ-ONLY PREVIEW\n'+json.dumps(status, indent=2)+
                    '\nOnly M4 is measured. Other joints unmeasured. Not synchronized or control feedback.'))
                pending = output/'status.pending'
                pending.write_text(json.dumps(status, indent=2)+'\n')
                pending.replace(output/'status.json')
                print(json.dumps(status), flush=True)
                last_status = now
            stop.wait(.03)
    finally:
        stop.set()
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill();process.wait(timeout=2)
        for worker in workers:
            worker.join(timeout=3)
        rec.set_time('host_elapsed', duration=time.monotonic()-started)
        for name in ('external', 'gripper'):
            rec.log('cameras/'+name, rr.Clear(recursive=True))
        rec.log('status', rr.TextDocument('STOPPED — no live camera feed'))
        rec.flush();rec.disconnect()
        stopped = dict(updatedAt=datetime.datetime.now(datetime.timezone.utc).isoformat(),
            running=False, sources={name: dict(phase='stopped') for name in feeds},
            controllerWrites=0, trainingRuns=0, recording=False, controlEligible=False)
        (output/'status.json').write_text(json.dumps(stopped, indent=2)+'\n')
        (output/'stopped.json').write_text(json.dumps(stopped)+'\n')


if __name__ == '__main__':
    main()
