"""Offline, same-session USB marker / camera PTS associations.

This is not an optical or AS5600 conversion calibration. Source identities are
supplied by the live collector before capture, never assigned to old receipts.
"""
from pathlib import Path
import re

from learning.artifacts import describe_artifact, read_json_artifact, verify_artifacts
from learning.mac_camera_probe import evaluate_capture
from learning.mac_clock_bridge import bridge, convert_pts, finite
from learning.serial_m4_probe import validate_receipt

SCHEMA = 'motionbrain.usb_camera_clock_link.v1'


def _window(value, error):
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or not all(finite(x) for x in value) or not 0 <= value[0] <= value[1]):
        raise ValueError(error)
    return value[0], value[1]


def link_saved_session(root, before_ref, after_ref, receipt_refs, *, session_id,
                       host_clock_id, controller_boot_id, nonces,
                       camera_process_window, transport_trace=True):
    """Recompute 21 PTS / six S3 associations from original artifact bytes.

    The caller must audit its acquisition plan/seal and pass those original
    identifiers and process times. Hashes detect changes, not forged provenance.
    Every receipt must already contain collectionSessionId and hostBootId.
    """
    root = Path(root)
    if (not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', session_id)
            or not isinstance(host_clock_id, str) or not host_clock_id
            or not isinstance(controller_boot_id, str) or not re.fullmatch('[0-9a-f]{32}', controller_boot_id)
            or type(transport_trace) is not bool):
        raise ValueError('link_identity_invalid')
    if (not isinstance(nonces, (list, tuple)) or len(nonces) != 6
            or any(not isinstance(n, str) or not re.fullmatch('[0-9a-f]{8}', n) for n in nonces)
            or len(set(nonces)) != 6):
        raise ValueError('link_nonces_invalid')
    if (not isinstance(receipt_refs, (list, tuple)) or len(receipt_refs) != 6
            or any(not isinstance(ref, dict) for ref in receipt_refs)):
        raise ValueError('link_receipt_refs_invalid')
    # Fixed source locations avoid presenting one receipt as several samples.
    source_locations = [(before_ref, 'clock/before.json'), (after_ref, 'clock/after.json')]
    source_locations += list(zip(receipt_refs, [f'serial/{i:02d}.json' for i in range(1, 7)]))
    for ref, path in source_locations:
        if not isinstance(ref, dict) or ref.get('artifactRef') != path:
            raise ValueError('link_source_location_invalid')
    before = read_json_artifact(root, before_ref)
    after = read_json_artifact(root, after_ref)
    evidence = bridge(before, after)
    if evidence['hostClockId'] != host_clock_id:
        raise ValueError('link_clock_identity_mismatch')
    start, end = _window(camera_process_window, 'link_process_window_invalid')
    valid_start, valid_end = evidence['validPythonBounds']
    if not valid_start <= start <= end <= valid_end:
        raise ValueError('link_process_outside_bridge')

    camera_paths = ['plan.json', 'session.json', 'terminal.json', 'execution.json']
    camera_paths += [f'frame-{i:04d}.{suffix}' for i in range(21) for suffix in ('json', 'bgra')]
    camera_refs = {name: describe_artifact(root, 'camera/' + name) for name in camera_paths}
    camera = evaluate_capture(root / 'camera')
    if (camera['issues'] or camera['validatedFrames'] != 21 or not camera['captureProcessingPassed']
            or camera.get('sessionId') != session_id):
        raise ValueError('link_camera_capture_invalid')
    if verify_artifacts(root, list(camera_refs.values())):
        raise ValueError('link_camera_changed_during_audit')
    camera_terminal = read_json_artifact(root, camera_refs['terminal.json'])

    sensors = []
    previous_end = start
    for ordinal, (ref, nonce) in enumerate(zip(receipt_refs, nonces), 1):
        row = read_json_artifact(root, ref)
        if (not isinstance(row, dict) or row.get('collectionSessionId') != session_id
                or row.get('hostBootId') != evidence['hostBootId'] or row.get('hostClockId') != host_clock_id):
            raise ValueError('link_receipt_identity_mismatch')
        sample = validate_receipt(row, nonce, controller_boot_id, fresh_sample=True,
                                  compact_sample=True, transport_trace=transport_trace)
        requested, received = row['requestStartedMonotonic'], row['receivedMonotonic']
        if not previous_end <= requested <= received <= end:
            raise ValueError('link_receipt_outside_process_or_order')
        previous_end = received
        bounds = sample['sampledHostBounds']
        if not start <= bounds['earliest'] <= bounds['latest'] <= end:
            raise ValueError('link_marker_outside_process')
        if sensors:
            for key in ('polledMs', 'reportedMs'):
                if not 0 < ((sample[key] - sensors[-1]['sample'][key]) & 0xffffffff) < 1000:
                    raise ValueError('link_sensor_marker_not_new')
        sensors.append(dict(ordinal=ordinal, receipt=ref, nonce=nonce,
                            requestStartedMonotonic=requested, receivedMonotonic=received,
                            sample=sample))

    frames = []
    for i in range(21):
        metadata = camera_refs[f'frame-{i:04d}.json']
        frame = read_json_artifact(root, metadata)
        if (frame.get('sessionId') != session_id or frame.get('sourceFrameId') != f'{session_id}:{i}'
                or frame.get('clockDomain') != 'coremedia_host_clock'
                or camera['frames'][i]['metadataSha256'] != metadata['sha256']):
            raise ValueError('link_frame_identity_mismatch')
        pts = convert_pts(frame['convertedHostPtsSeconds'], before, after)
        if not start <= pts['earliest'] <= pts['latest'] <= end:
            raise ValueError('link_pts_outside_process')
        # These are host observations, not additional exposure measurements.
        # Even valid PTS cannot excuse observations after the process ended.
        observed = [frame[key] for key in ('callbackHostSeconds', 'processingCompleteHostSeconds',
                                          'storageStartedHostSeconds')]
        observed.append(camera_terminal['frames'][i]['storageCompleteHostSeconds'])
        offset_low, offset_high = evidence['offsetBoundsSeconds']
        if any(not start <= value + offset_low <= value + offset_high <= end for value in observed):
            raise ValueError('link_camera_observation_outside_process')
        distances = []
        for sensor in sensors:
            marker = sensor['sample']['sampledHostBounds']
            distances.append(max(abs(pts['latest'] - marker['earliest']),
                                 abs(marker['latest'] - pts['earliest'])) * 1000)
        nearest = min(range(6), key=distances.__getitem__)
        frames.append(dict(sourceFrameId=frame['sourceFrameId'], metadata=metadata,
                           pixels=camera_refs[f'frame-{i:04d}.bgra'], presentationHostBounds=pts,
                           nearestSensorOrdinal=nearest + 1, nearestSensorNonce=nonces[nearest],
                           pollMarkerSeparationUpperDiagnosticMs=distances[nearest],
                           within20ms=distances[nearest] <= 20))
    return dict(schemaVersion=SCHEMA, collectionSessionId=session_id, hostClockId=host_clock_id,
                hostBootId=evidence['hostBootId'], controllerBootId=controller_boot_id,
                cameraProcessWindow=[start, end], bridge=evidence,
                sources=dict(before=before_ref, after=after_ref, receipts=list(receipt_refs),
                             camera=list(camera_refs.values())), sensors=sensors, frames=frames,
                presentationClockLinked=True, postHocAssociation=True,
                associationSemantics='nearest_interval_upper_distance_not_causal_training_pair',
                mcuClockRateCalibrated=False, mcuIntervalMarginMs=2,
                opticalExposureVerified=False, sensorPhysicalSampleTimeVerified=False,
                physicalSampleAlignmentVerified=False, semanticAnchorVerified=False,
                trainingEligible=False, controlEligible=False, physicalLearningEligibleRows=0)
