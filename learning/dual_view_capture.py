"""Continuous acquisition in its own process, independent of motor HTTP latency."""
import hashlib
import io
import json
import re
import time
from pathlib import Path

from PIL import Image

from learning.action_recorder import ActionRecorder
from learning.artifacts import ArtifactStore, _decode_json
from learning.dual_view_policy import ACTIONS, DualViewPolicy, thumbnail, usable
from learning.action_execution import canonical_request, checked_record, finite
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
    if 'responsePolicy' in plan:
        from learning.relative_response_policy import RelativeResponsePolicy
        ref = plan['responsePolicy']
        policy = RelativeResponsePolicy(_decode_json(verified(Path(ref['artifact']), ref['sha256'])),
                                        view_id=plan['external']['viewConfigurationId'])
        for label, profile in plan['profiles'].items():
            if label not in policy.responses or profile['command'] != policy.responses[label]['command']:
                raise ValueError('response_profile_mismatch')
        return policy
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


def _send_context(mailbox, value):
    packet = json.dumps(value, allow_nan=False, separators=(',', ':')).encode() if value else b''
    with mailbox.get_lock():
        mailbox.get_obj().value = packet


def _read_context(mailbox, session, boot):
    if mailbox is None:
        return None
    with mailbox.get_lock():
        packet = mailbox.get_obj().value
    if not packet:
        return None
    value = _decode_json(packet)
    if value['session'] != session or value['boot'] != boot:
        raise ValueError('control_epoch_changed')
    return value


def publish_active_context(mailbox, session, boot, request, start_vote=None, acknowledged_at=None):
    """Publish only after verified running ACK. Reset at the end of the command."""
    if request is None:
        _send_context(mailbox, None)
        return
    req = canonical_request(request)
    label = start_vote['action']
    motor, sign = ACTIONS[label]
    if (req['bootId'] != boot or motor != req['motor'] or
            (motor != 4 and req['pwm'] * sign <= 0) or
            not finite(acknowledged_at) or not finite(start_vote['decidedAt']) or
            not 0 <= start_vote['decidedAt'] <= acknowledged_at or
            not isinstance(start_vote['features'], dict)):
        raise ValueError('active_context_mismatch')
    _send_context(mailbox, dict(session=session, boot=boot, commandId=req['commandId'],
        after=acknowledged_at, action=dict(label=label, startFeatures=start_vote['features'])))


def publish_close_context(mailbox, session, boot, result):
    """Keep evidence of commanded closure separate from visual contact evidence."""
    if result is None:
        _send_context(mailbox, None)
        return
    a = result['action']
    req = canonical_request(a['request'])
    rec = checked_record(a['record'], req)
    log = a['outputLog']
    entries = [e for e in log['entries'] if e['motor'] == 1]
    after = a['outputOffHostBounds']['latest']
    if (result['executionState'] != 'known_terminal' or req['bootId'] != boot or req['motor'] != 1
            or req['pwm'] >= 0 or not finite(after) or after <= 0 or not rec['outputOff']
            or log['bootId'] != boot or log['evidence'] != 'driver_write' or log['dropped'] != 0
            or not entries or entries[-1]['pwm'] != 0 or not any(e['pwm'] < 0 for e in entries)
            or any(e['actionOrdinal'] != rec['ordinal'] or not req['pwm'] <= e['pwm'] <= 0 for e in entries)
            or not (rec['state'] == 'completed' or
                    (rec['state'] == 'cancelled' and result.get('cancelConfirmed') is True) or
                    rec['state'] == 'timed_out')):
        raise ValueError('close_context_unproven')
    final_power = max(i for i, e in enumerate(entries) if e['pwm'] < 0)
    if (entries[final_power + 1]['atMs'] - rec['startedAtMs']) % 2**32 > req['durationMs'] + 1:
        raise ValueError('close_driver_deadline_exceeded')
    receipt = dict(executionState='known_terminal', commandId=req['commandId'],
                   action=dict(request=req, record=rec, outputLog=log, outputOffHostBounds=a['outputOffHostBounds']))
    _send_context(mailbox, dict(session=session, boot=boot, after=after, receipt=receipt))


def response_vote(policy, side, near, earliest, active, close):
    close_ready = close is not None and earliest > close['after']
    active_ready = active is not None and earliest > active['after']
    if close is not None and not close_ready:
        vote = dict(action='hold', reason='await_post_close_pair', viewVotes=['hold', 'hold'], features={})
    else:
        vote = policy.predict(side, near, close_receipt=close['receipt'] if close_ready else None,
                              active_action=active['action'] if active_ready else None)
    vote.update(activeCommandId=active['commandId'] if active_ready else None,
                activeAcknowledgedAt=active['after'] if active_ready else None,
                closeCommandId=close['receipt']['commandId'] if close_ready else None)
    return vote


def side_sample_fps(plan):
    fps = plan.get('sideSampleFps', 4)
    if type(fps) is not int or fps not in (4, 8):
        raise ValueError('invalid_side_sample_fps')
    return fps


def capture(plan, root, session, stopping, mailbox, context_mailbox=None, active_mailbox=None):
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
        fps = side_sample_fps(plan)
        frames = fps*duration+20
        config = plan['external']
        helper = Path(plan['helper'])
        verified(helper, plan['helperSha256'])
        camera = ActionRecorder(root/'external-stream', helper, dict(
            deviceId=config['deviceUniqueId'], viewId=config['viewConfigurationId'], sessionId=session,
            maxFrames=frames, seconds=duration, sampleFps=fps, maxBytes=frames*(1920*1080*4+65536)))
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
                if 'responsePolicy' in plan:
                    earliest = min(frame['sourceEarliest'], item['sourceEarliest'])
                    boot = plan['controllerBootId']
                    active = _read_context(active_mailbox, session, boot)
                    close = _read_context(context_mailbox, session, boot)
                    vote = response_vote(policy, side, near, earliest, active, close)
                else:
                    vote = policy.predict(side, near)
                vote.update(frameId=frame['frameId'], nearFrameId=item['frame']['frameId'],
                    extClock=dict(sourceEarliest=frame['sourceEarliest'], sourceLatest=frame['sourceLatest'],
                                  availableAt=frame['stdoutReadAt']),
                    nearClock=dict(sourceEarliest=item['sourceEarliest'], sourceLatest=item['sourceLatest'],
                                   availableAt=item['persistedAt']), decidedAt=time.monotonic())
                effective_vote(vote, last_dissent, plan['maxAge'], plan['maxSkew'])
                vote['previousPair'] = previous_pair
                previous_pair = {key: vote[key] for key in ('action', 'extClock', 'nearClock', 'frameId', 'nearFrameId')}
                if 'responsePolicy' in plan:
                    previous_pair.update(activeCommandId=vote['activeCommandId'], closeCommandId=vote['closeCommandId'])
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
