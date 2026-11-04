"""Bind recorded gripper-camera images to an audited single command, offline."""
import copy
import math
import re

import cv2
import numpy as np

from learning.artifacts import _read, _decode_json
from learning.manual_collection_dataset import require, digest


POLICY = dict(inputRequestMaximumSecondsBeforeCommand=40.,
              inputPersistMinimumSecondsBeforeCommand=.1,
              targetRequestMinimumSecondsAfterOff=3.,
              targetPersistMaximumSecondsAfterOff=30.,
              containment='one_audited_single_command_camera_session',
              exposureSynchronizationVerified=False)


def read_frame(root, name, session_id):
    metadata = _decode_json(_read(root, f'esp32-{name}.json'))
    body = _read(root, f'esp32-{name}.jpg')
    require(metadata.get('sessionId') == session_id and metadata.get('hostClockId') == session_id
            and metadata.get('clockDomain') == 'python_monotonic', 'gripper_host_clock_binding')
    times = [metadata.get(k) for k in ('requestStartedAt', 'receivedAt', 'persistedAt')]
    require(all(type(x) in (int, float) and math.isfinite(x) for x in times)
            and times == sorted(times), 'gripper_receipt_times')
    require(digest(body) == metadata.get('sha256') and body.startswith(b'\xff\xd8')
            and body.endswith(b'\xff\xd9'), 'gripper_image_hash')
    headers = {k.lower(): v for k, v in metadata['headers'].items()}
    prefix = 'x-motionbrain-'
    boot = headers.get(prefix+'camera-boot-id', '')
    captured = headers.get(prefix+'captured-us', '')
    requested = headers.get(prefix+'capture-requested-us', '')
    require(isinstance(boot, str) and re.fullmatch(r'[0-9a-f]{32}', boot)
            and isinstance(captured, str) and captured.isdecimal()
            and isinstance(requested, str) and requested.isdecimal()
            and 0 < int(requested) < int(captured)
            and headers.get(prefix+'frame-id') == f'{boot}:{captured}'
            and headers.get(prefix+'capture-mode') == 'after_request'
            and headers.get(prefix+'clock-domain') == 'camera_boot_us'
            and headers.get(prefix+'timestamp-semantics') == 'first_dma_buffer', 'gripper_fresh_source_identity')
    require(len(body) <= 2_000_000, 'gripper_image_size')
    image = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)
    require(image is not None and 16 <= image.shape[0] <= 960
            and 16 <= image.shape[1] <= 1280, 'gripper_image_dimensions')
    pixels = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
    return dict(frameId=f'{boot}:{captured}', pixelSha256=digest(pixels.tobytes()),
                imageSha256=digest(body), width=image.shape[1], height=image.shape[0],
                capturedUs=int(captured), requestedUs=int(requested)), metadata, boot


def bind_gripper_pair(row, root):
    """Keep external evidence, but use the separate camera's projected coordinates.

    Timing is host request/receipt/persistence evidence, not exact exposure sync.
    The external camera and gripper camera pixels are never concatenated here.
    """
    before, pre, boot = read_frame(root, 'before', row['sessionId'])
    after, post, after_boot = read_frame(root, 'after', row['sessionId'])
    require(boot == after_boot and before['frameId'] != after['frameId'], 'gripper_camera_reboot_or_duplicate')
    require(after['requestedUs'] > before['capturedUs'], 'gripper_source_time_reversal')
    command_at = row['evidence']['commandAt']; off_at = row['evidence']['offReceiptAt']
    start, end = row['evidence']['cameraHostBounds']
    require(start <= pre['requestStartedAt'] and post['persistedAt'] <= end, 'gripper_outside_audited_session')
    require(command_at-40. <= pre['requestStartedAt'] and pre['persistedAt'] <= command_at-.1,
            'gripper_input_outside_window')
    require(post['requestStartedAt'] >= off_at+3. and post['persistedAt'] <= off_at+30.,
            'gripper_target_outside_window')
    require(before['width'] == after['width'] and before['height'] == after['height'], 'gripper_camera_shape_changed')
    result = copy.deepcopy(row)
    result['evidence']['externalCameraIdentity'] = result['evidence']['cameraIdentity']
    result['evidence']['externalFrameIds'] = [row[k]['image']['frameId'] for k in ('inputs', 'targets')]
    result['evidence']['cameraIdentity'] = dict(camera='gripper', cameraBootId=boot,
                                               timestampSemantics='first_dma_buffer', observationPolicy=copy.deepcopy(POLICY))
    result['evidence']['gripperFrameEvidence'] = dict(before=pre, after=post)
    result['inputs']['image'] = before; result['targets']['image'] = after
    return result
