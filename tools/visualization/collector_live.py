"""수집 중 확정된 최신 프레임을 같은 호스트의 Rerun으로 보낸다."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import socket
import sys
import threading
import time
from urllib.parse import urlsplit


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from learning.lossless_frame import SUFFIX, decode

MAX_AGE = 1.5
DIGITS = '[0-9]' * 4


def latest(root, camera, single_command=False):
    if camera == 'external':
        return max((root / ('native-camera' if single_command else 'external-stream') / 'host-ingress').glob(DIGITS + '.json'), default=None)
    # A new capture directory can exist before its binding is committed.
    blocks = sorted(root.glob('gripper-[0-9][0-9]'), reverse=True)
    for block in [*blocks, root / ('gripper-camera' if single_command else 'gripper')]:
        for folder in sorted(block.glob('capture-' + DIGITS), reverse=True):
            binding = folder / 'source-binding.json'
            if binding.is_file():
                return binding
    return None


def read_frame(path, camera, now, *, include_rgb=False):
    receipt = json.loads(path.read_bytes())
    record = receipt if camera == 'external' else receipt['record']
    at = record['sourceEarliest']
    if not (0 <= now() - at <= MAX_AGE and 0 <= time.time() - path.stat().st_mtime <= MAX_AGE):
        return None
    if camera == 'external':
        filename = 'frame-' + path.stem + SUFFIX
        if receipt['imageFile'] != filename:
            raise ValueError('external_filename_mismatch')
        raw = (path.parent.parent / filename).read_bytes()
    else:
        raw = (path.parent / 'esp32-frame.jpg').read_bytes()
    if hashlib.sha256(raw).hexdigest() != receipt['imageSha256']:
        raise ValueError('collector_image_hash_mismatch')
    rgb = None
    if camera == 'external':
        import cv2
        import numpy as np
        pixels = decode(raw, receipt['rawByteLength'], receipt['pixelSha256'])
        bgra = np.frombuffer(pixels, np.uint8).reshape(receipt['height'], receipt['width'], 4)
        if include_rgb:
            rgb = cv2.cvtColor(bgra, cv2.COLOR_BGRA2RGB)
        ok, encoded = cv2.imencode('.jpg', bgra, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise ValueError('preview_encoding_failed')
        raw = encoded.tobytes()
    elif include_rgb:
        import cv2
        import numpy as np
        bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError('invalid_gripper_jpeg')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    identity = record.get('frameId') or record['frame']['frameId']
    return (at, identity, raw, rgb) if include_rgb else (at, identity, raw)


def watch(root, camera, single_command, slots, lock, stop, include_rgb=False):
    seen = None
    try:
        while not stop.is_set():
            path = latest(root, camera, single_command)
            if path and path != seen:
                seen = path
                frame = read_frame(path, camera, time.monotonic, include_rgb=include_rgb)
                with lock:
                    slots[camera] = (path, frame)
            stop.wait(.025 if include_rgb else .1)
    except Exception as exc:
        with lock:
            slots[camera] = (None, exc)


def monitor(root, rec, rr, stop, single_command=False, on_closing=None):
    slots, seen, displayed = {}, {}, {}
    lock = threading.Lock()
    for camera in ('external', 'gripper'):
        threading.Thread(target=watch, args=(root, camera, single_command, slots, lock, stop),
                         daemon=True).start()
    try:
        while not stop.is_set():
            with lock:
                snapshot = slots.copy()
            for camera in ('external', 'gripper'):
                entity = 'cameras/' + camera
                path, frame = snapshot.get(camera, (None, None))
                changed = False
                if isinstance(frame, Exception):
                    raise frame
                if path and seen.get(camera) != path:
                    seen[camera] = path
                    submitted = time.monotonic()
                    if frame and 0 <= submitted - frame[0] <= MAX_AGE:
                        rec.log(entity, rr.EncodedImage(contents=frame[2], media_type='image/jpeg'))
                        changed = True
                        displayed[camera] = frame[0]
                        print(json.dumps(dict(camera=camera, frameId=frame[1], receipt=str(path),
                                              sourceEarliest=frame[0], submittedAt=submitted)), flush=True)
                if camera in displayed and time.monotonic() - displayed[camera] > MAX_AGE:
                    rec.log(entity, rr.Clear(recursive=True))
                    changed = True
                    del displayed[camera]
                    print(json.dumps(dict(camera=camera, staleAt=time.monotonic())), flush=True)
                if changed:
                    rec.flush(timeout_sec=.25)
            stop.wait(.05)
    finally:
        if on_closing:
            on_closing()
        for camera in ('external', 'gripper'):
            rec.log('cameras/' + camera, rr.Clear(recursive=True))
        stop.set()
        try:
            rec.flush(timeout_sec=1.)
        finally:
            rec.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--part-overlay', action='store_true', help='수집 원본에 부품과 관측점을 겹쳐 표시')
    parser.add_argument('--single-command', action='store_true', help='native-camera 및 gripper-camera 폴더 사용')
    parser.add_argument('--rerun-url', default='rerun+http://127.0.0.1:9876/proxy')
    args = parser.parse_args()
    import rerun as rr
    import rerun.blueprint as bp
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    endpoint = urlsplit(args.rerun_url)
    with socket.create_connection((endpoint.hostname, endpoint.port or 9876), timeout=2):
        pass
    rec = rr.RecordingStream('motionbrain_collector_live')
    # Native SDK connect/disconnect may block through Python signal handlers.
    signal.alarm(5)
    rec.connect_grpc(args.rerun_url)
    layout = bp.Horizontal(bp.Spatial2DView(origin='/cameras/external'), bp.Spatial2DView(origin='/cameras/gripper'))
    if args.part_overlay:
        from tools.visualization.collector_parts import legend_image
        layout = bp.Vertical(layout, bp.Horizontal(bp.Spatial2DView(origin='/legend', name='Part colors'),
                                                    bp.TextDocumentView(origin='/decision', name='Matched decision'),
                                                    column_shares=[2.7, 1]), row_shares=[6, 1.1])
        rec.log('legend', rr.Image(legend_image()), static=True)
    rec.send_blueprint(bp.Blueprint(layout, collapse_panels=args.part_overlay))
    rec.flush(timeout_sec=1.)
    signal.alarm(0)
    display = monitor
    if args.part_overlay:
        from tools.visualization.collector_parts import monitor_parts
        display = monitor_parts
    display(args.source, rec, rr, stop, args.single_command, lambda: signal.alarm(4))
    signal.alarm(0)


if __name__ == '__main__':
    main()
