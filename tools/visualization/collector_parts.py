"""수집 원본을 우선 표시하고 같은 프레임에서 계산한 부품 윤곽만 합성한다."""
from functools import partial
import hashlib
import json
from pathlib import Path
import sys
import threading
import time

import cv2

from learning.dual_view_policy import ACTIONS
from tools.visualization.collector_live import MAX_AGE, watch

CAMERAS = ('external', 'gripper')
VOTES = frozenset(ACTIONS) | {'hold'}


def legend_image():
    """Draw native swatches and labels once; the Rerun UI font may lack Hangul."""
    from PIL import Image, ImageDraw, ImageFont
    import numpy as np
    from learning.side_part_overlay import COLORS as side
    from learning.near_part_overlay import COLORS as near, TIP_COLOR

    entries = [
        (side['fixed_base'], '고정 바닥', 'Fixed base'), (side['rotating_base'], '회전 받침', 'Rotating base'),
        (side['upper_arm'], '위팔', 'Upper arm'), (side['forearm'], '아래팔', 'Forearm'),
        (side['wrist_body'], '손목·몸체', 'Wrist / body'), (near['leftJaw'], '측면 턱 / 왼턱', 'Side / left jaw'),
        (near['rightJaw'], '오른턱', 'Right jaw'), (near['target'], '목표 물체', 'Target'),
        (TIP_COLOR, '턱끝·영상 간격', 'Tip / image gap'),
    ]
    korean = True
    for path in ('/System/Library/Fonts/AppleSDGothicNeo.ttc',
                 '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'):
        try:
            font = ImageFont.truetype(path, 20)
            break
        except OSError:
            pass
    else:
        korean = False
        try:
            font = ImageFont.truetype('DejaVuSans.ttf', 17)
        except OSError:
            font = ImageFont.load_default()
    image = Image.new('RGB', (840, 78), '#202329')
    draw = ImageDraw.Draw(image)
    for i, (color, ko, en) in enumerate(entries):
        x, y = 12 + i % 5 * 166, 10 + i // 5 * 36
        draw.rectangle((x, y, x+18, y+18), fill=color)
        draw.text((x+27, y+9), ko if korean else en, fill='white', font=font, anchor='lm')
    return np.asarray(image)


def latest_decision(path, frame_ids):
    """Read only the newest complete model decision; an in-progress append is ignored."""
    if any(camera not in frame_ids for camera in CAMERAS):
        return None
    try:
        with path.open('rb') as stream:
            end = stream.seek(0, 2)
            # A decision also fits in the collector's shared mailbox. Bound work
            # independently of the length of the experiment.
            offset = max(0, end - 65536)
            stream.seek(offset)
            lines = stream.read(65536).split(b'\n')[:-1]
    except OSError:
        return None
    if offset:
        lines = lines[1:]
    if not lines:
        return None
    try:
        vote = json.loads(lines[-1])
    except (ValueError, RecursionError):
        return None
    if not isinstance(vote, dict) or not all(isinstance(vote.get(key), str)
                                             for key in ('frameId', 'nearFrameId', 'action', 'modelAction')):
        return None
    stamp = vote.get('decidedAt')
    if (vote['action'] not in VOTES or vote['modelAction'] not in VOTES
            or type(stamp) not in (float, int) or not 0 <= stamp <= sys.float_info.max
            or (vote.get('reason') is not None and not isinstance(vote['reason'], str))):
        return None
    if (frame_ids['external'], frame_ids['gripper']) != (vote['frameId'], vote['nearFrameId']):
        return None
    return {key: vote.get(key) for key in ('frameId', 'nearFrameId', 'action', 'modelAction', 'reason', 'decidedAt')}


def render_latest(camera, slots, ready, lock, stop, render):
    seen = None
    while not stop.is_set():
        with lock:
            path, frame = slots.get(camera, (None, None))
        if path is None or path == seen or not frame or isinstance(frame, Exception):
            stop.wait(.01)
            continue
        seen = path
        started = time.monotonic()
        if not 0 <= started - frame[0] <= MAX_AGE:
            continue
        try:
            image, metadata = render(frame[3])
            if image.shape != frame[3].shape or image.dtype != frame[3].dtype:
                raise ValueError('invalid_overlay_pixels')
            ok, encoded = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                                       [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise ValueError('overlay_encoding_failed')
            jpeg = encoded.tobytes()
            finished = time.monotonic()
        except Exception as exc:
            # A rendering failure on an individual scene must leave the raw view usable.
            print(json.dumps(dict(camera=camera, frameId=frame[1], kind='overlay_error',
                                  error=type(exc).__name__, detail=str(exc))), flush=True)
            continue
        with lock:
            if slots[camera][0] == path and finished - frame[0] <= MAX_AGE:
                ready[camera] = (path, jpeg, metadata, started, finished)


def monitor_parts(root, rec, rr, stop, single_command=False, on_closing=None):
    from learning.near_part_overlay import render_near
    from learning.open_side_approach import WIDE_SIDE_VIEW
    from learning.side_part_overlay import render_side

    plan_file = Path(root) / 'plan.json'
    if not single_command and plan_file.is_file():
        view_id = json.loads(plan_file.read_bytes()).get('external', {}).get('viewConfigurationId')
        render_side = partial(render_side, view_id=view_id)
        if view_id == WIDE_SIDE_VIEW:
            render_near = partial(render_near, wide_approach=True)
    slots, ready, displayed, annotated = {}, {}, {}, {}
    lock = threading.Lock()
    for camera, render in zip(CAMERAS, (render_side, render_near)):
        threading.Thread(target=watch, args=(root, camera, single_command, slots, lock, stop, True),
                         daemon=True).start()
        threading.Thread(target=render_latest, args=(camera, slots, ready, lock, stop, render),
                         daemon=True).start()

    last_vote = None
    decision_file = Path(root) / 'decisions.jsonl'
    try:
        while not stop.is_set():
            for camera in CAMERAS:
                with lock:
                    path, frame = slots.get(camera, (None, None))
                    overlay = ready.get(camera)
                if isinstance(frame, Exception):
                    raise frame
                entity = 'cameras/' + camera
                shown = displayed.get(camera)
                now = time.monotonic()
                if frame and 0 <= now - frame[0] <= MAX_AGE:
                    use_overlay = overlay is not None and overlay[0] == path
                    if shown is None or shown[0] != path or (use_overlay and annotated.get(camera) != path):
                        raw = overlay[1] if use_overlay else frame[2]
                        image = rr.EncodedImage(contents=raw, media_type='image/jpeg')
                        submitted = time.monotonic()
                        # The reader or clock may have advanced while preparing an SDK image.
                        with lock:
                            current_path = slots[camera][0]
                        if current_path != path or submitted - frame[0] > MAX_AGE:
                            continue
                        if last_vote and (shown is None or shown[2] != frame[1]):
                            rec.log('decision', rr.Clear(recursive=True))
                            last_vote = None
                        rec.log(entity, image)
                        displayed[camera] = (path, frame[0], frame[1])
                        event = dict(camera=camera, kind='overlay' if use_overlay else 'raw', frameId=frame[1],
                                     receipt=str(path), sourceEarliest=frame[0], submittedAt=submitted)
                        if use_overlay:
                            annotated[camera] = path
                            event.update(analysisStartedAt=overlay[3], analysisFinishedAt=overlay[4],
                                         imageSha256=hashlib.sha256(raw).hexdigest(), observation=overlay[2])
                        print(json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)
                elif shown and now - shown[1] > MAX_AGE:
                    if last_vote:
                        rec.log('decision', rr.Clear(recursive=True))
                        last_vote = None
                    rec.log(entity, rr.Clear(recursive=True))
                    del displayed[camera]
                    print(json.dumps(dict(camera=camera, staleAt=now)), flush=True)

            ids = {k: shown[2] for k, shown in displayed.items()}
            vote = latest_decision(decision_file, ids)
            if vote != last_vote:
                if vote is None:
                    rec.log('decision', rr.Clear(recursive=True))
                else:
                    text = (f"Model vote: {vote['modelAction']}   Gated vote: {vote['action']}"
                            f"\n{vote['reason'] or ''}")
                    rec.log('decision', rr.TextDocument(text))
                    print(json.dumps(dict(kind='paired_decision', submittedAt=time.monotonic(), **vote)), flush=True)
                last_vote = vote
            stop.wait(.015)
    finally:
        stop.set()
        if on_closing:
            on_closing()
        for entity in ('cameras/external', 'cameras/gripper', 'decision'):
            rec.log(entity, rr.Clear(recursive=True))
        try:
            rec.flush(timeout_sec=1.)
        finally:
            rec.disconnect()
