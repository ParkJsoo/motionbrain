"""Continuous acquisition in its own process, independent of motor HTTP latency."""
import hashlib
import io
import json
import re
import time
from pathlib import Path

from PIL import Image

from learning.action_recorder import ActionRecorder
from learning.artifacts import ArtifactStore
from learning.dual_view_policy import ACTIONS, DualViewPolicy, thumbnail, usable
from learning.action_execution import finite
from learning.grasp_observation import GripperObservationSource
from learning.lossless_frame import decode


def read(path):
    return json.loads(path.read_bytes())


def verified(path, expected):
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected:
        raise ValueError('camera_asset_digest_changed')
    return raw


def side_image(root, frame, config):
    metadata = json.loads(verified(root/frame['metadataFile'], frame['metadataSha256']))
    for key in ('deviceUniqueId', 'viewConfigurationId', 'configurationSha256',
                'width', 'height', 'videoRotationAngle', 'videoMirrored'):
        if metadata[key] != config[key]:
            raise ValueError('external_view_changed')
    if (metadata['sourceFrameId'] != frame['frameId'] or metadata['imageSha256'] != frame['imageSha256']
            or metadata['pixelSha256'] != frame['pixelSha256']
            or metadata['sessionId'] != frame['frameId'].rsplit(':', 1)[0]):
        raise ValueError('external_frame_identity_changed')
    raw = verified(root/frame['imageFile'], frame['imageSha256'])
    pixels = decode(raw, frame['rawByteLength'], frame['pixelSha256'])
    return Image.frombytes('RGBA', (frame['width'], frame['height']), pixels, 'raw', 'BGRA')


def gripper_image(folder, binding):
    raw = verified(folder/'esp32-frame.jpg', binding['imageSha256'])
    with Image.open(io.BytesIO(raw)) as image:
        if image.format != 'JPEG' or image.size != (320, 240):
            raise ValueError('gripper_view_changed')
        return image.convert('RGB')


def policy_example(episode_file, episode, index, plan):
    """Replay only the source images available at the human action's press."""
    if (episode.get('schemaVersion') != 'motionbrain.manual_il_episode.v1'
            or episode.get('source') != 'human_teleoperation'
            or type(index) is not int or not 1 <= index <= len(episode['actions'])):
        raise ValueError('human_demonstration_required')
    event = episode['actions'][index - 1]
    command, press = event['command'], event['press']
    label = f"{command['joint']}/{command['direction']}"
    if event['index'] != index or event['motor'] != ACTIONS[label][0]:
        raise ValueError('demonstration_action_mismatch')
    prior = press['availablePriorObservation']
    selected = {}
    for view in ('external', 'gripper'):
        matches = [frame for frame in episode['frames'][view] if frame['id'] == prior[view]]
        if len(matches) != 1:
            raise ValueError('demonstration_pair_missing')
        selected[view] = matches[0]
        start, end = matches[0]['sourceHostBounds']
        available = matches[0]['availableHostTime']
        at = press['hostBounds'][0]
        if not all(finite(t) for t in (start, end, available, at)) or not 0 <= start <= end <= available < at:
            raise ValueError('demonstration_not_precommand')
    side, near = selected['external'], selected['gripper']
    if not re.fullmatch(r'external-stream/frame-[0-9]{4}\.bgra\.lzbitmap', side['file']) or not re.fullmatch(
            r'gripper(?:-[0-9]{2})?/capture-[0-9]{4}/esp32-frame\.jpg', near['file']):
        raise ValueError('demonstration_frame_path_invalid')
    side_path, near_path = (episode_file.parent / row['file'] for row in (side, near))
    n = side_path.name[6:10]
    frame = read(side_path.parent/'host-ingress'/f'{n}.json')
    binding = read(near_path.parent/'source-binding.json')
    record = binding['record']
    metadata = json.loads(verified(near_path.with_suffix('.json'), binding['metadataSha256']))
    headers = {k.lower(): v for k, v in metadata['headers'].items()}
    if (side['id'] != frame['frameId'] or side['id'] != f"{episode['episode']}:{int(n)}"
            or side_path.name != frame['imageFile'] or frame['metadataFile'] != f'frame-{n}.json'
            or side['sha256'] != frame['imageSha256'] or near['sha256'] != binding['imageSha256']
            or record['frame']['frameId'] != near['id'] or headers['x-motionbrain-frame-id'] != near['id']
            or metadata['sha256'] != near['sha256'] or record['hostClockId'] != episode['episode']
            or metadata['sessionId'] != episode['episode'] or metadata['hostClockId'] != episode['episode']
            or side['sourceHostBounds'] != [frame['sourceEarliest'], frame['sourceLatest']]
            or side['availableHostTime'] != frame['stdoutReadAt']
            or not metadata['requestStartedAt'] <= near['sourceHostBounds'][0] <= near['sourceHostBounds'][1] <= metadata['receivedAt']
            or near['availableHostTime'] != record['persistedAt']
            or record['persistedAt'] != metadata['persistedAt'] or record['persistedAt'] >= at
            or any(record['context'].get(k) != v for k, v in plan['scene'].items())):
        raise ValueError('demonstration_receipt_mismatch')
    return (label, thumbnail(side_image(side_path.parent, frame, plan['external'])),
            thumbnail(gripper_image(near_path.parent, binding)))


def load_policy(plan):
    episodes, examples, seen = {}, [], set()
    for ref in plan['examples']:
        path = Path(ref['episode'])
        key = path, ref['episodeSha256']
        if key not in episodes:
            episodes[key] = json.loads(verified(*key))
        index = ref['actionIndex']
        identity = episodes[key]['episode'], index
        if identity in seen:
            raise ValueError('duplicate_demonstration')
        seen.add(identity)
        examples.append(policy_example(path, episodes[key], index, plan))
    return DualViewPolicy(examples, plan['maxViewRmse'])


def effective_vote(vote, last_dissent, max_age, max_skew):
    """Preserve dissent even when a slower controller skips intermediate packets."""
    vote['modelAction'] = vote['action']
    if not usable(vote, 0, vote['decidedAt'], max_age, max_skew):
        vote['action'] = 'hold'
    earliest = min(vote[view]['sourceEarliest'] for view in ('extClock', 'nearClock'))
    for label in ACTIONS:
        if label != vote['action']:
            last_dissent[label] = max(earliest, last_dissent.get(label, 0.))
    vote['lastDissent'] = last_dissent.copy()


def capture(plan, root, session, stopping, mailbox):
    root = Path(root)
    def publish(value):
        packet = json.dumps(dict(value, session=session), allow_nan=False).encode()
        with mailbox.get_lock():
            mailbox.get_obj().value = packet
    camera = gripper = None
    emitted = 0
    try:
        policy = load_policy(plan)
        duration = plan['seconds']
        frames = 4*duration+20
        config = plan['external']
        helper = Path(plan['helper'])
        verified(helper, plan['helperSha256'])
        camera = ActionRecorder(root/'external-stream', helper, dict(
            deviceId=config['deviceUniqueId'], viewId=config['viewConfigurationId'], sessionId=session,
            maxFrames=frames, seconds=duration, sampleFps=4, maxBytes=frames*(1920*1080*4+65536)))
        until = time.monotonic()+duration
        camera.start()
        camera.wait_ready()
        previous = -1
        block = 0
        last_dissent = {}
        previous_pair = None
        previous_capture_us = None
        with (root/'decisions.jsonl').open('x') as log:
            while not stopping.is_set() and time.monotonic() < until:
                if gripper is None or gripper.count == gripper.max_captures:
                    if gripper is not None:
                        gripper.close()
                    block += 1
                    folder = root/('gripper' if block == 1 else f'gripper-{block:02d}')
                    gripper = GripperObservationSource(plan['gripperUrl'], session, plan['gripperBootId'],
                        plan['scene'], ArtifactStore(folder), timeout=2., max_captures=100)
                    # The directory rotates, but device time and the camera boot continue.
                    gripper.previous_capture_us = previous_capture_us
                    gripper.start(deadline=time.monotonic()+2.)
                camera.check()
                item = gripper.capture(deadline=time.monotonic()+2.)
                gripper.verify(item)
                previous_capture_us = gripper.previous_capture_us
                binding = gripper._bindings[item['frame']['frameId']]
                near = gripper_image(gripper.store.root/binding['folder'], binding)
                # Consume the newest durable side frame once. Never queue old scenes.
                with camera.lock:
                    frame = camera.frames[-1].copy()
                if frame['frameIndex'] <= previous:
                    continue
                previous = frame['frameIndex']
                side = side_image(camera.root, frame, config)
                vote = policy.predict(side, near)
                vote.update(frameId=frame['frameId'], nearFrameId=item['frame']['frameId'],
                    extClock=dict(sourceEarliest=frame['sourceEarliest'], sourceLatest=frame['sourceLatest'],
                                  availableAt=frame['stdoutReadAt']),
                    nearClock=dict(sourceEarliest=item['sourceEarliest'], sourceLatest=item['sourceLatest'],
                                   availableAt=item['persistedAt']), decidedAt=time.monotonic())
                effective_vote(vote, last_dissent, plan['maxAge'], plan['maxSkew'])
                vote['previousPair'] = previous_pair
                previous_pair = {key: vote[key] for key in ('action', 'extClock', 'nearClock', 'frameId', 'nearFrameId')}
                log.write(json.dumps(vote, allow_nan=False)+'\n')
                log.flush()
                publish(vote)
                emitted += 1
        if emitted == 0 and not stopping.is_set():
            raise ValueError("no_paired_observations")
    finally:
        try:
            publish(dict(fault=True, action='hold'))
        finally:
            try:
                if gripper is not None:
                    gripper.close()
            finally:
                if camera is not None:
                    closure = camera.close()
                    if closure["status"] == "failed" or not closure["clockBridge"]:
                        raise ValueError("external_capture_closure_unverified")
