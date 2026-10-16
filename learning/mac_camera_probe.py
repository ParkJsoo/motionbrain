"""Finite Mac-camera commissioning evidence; never an action-training adapter.

CoreMedia PTS conversion is diagnostic. Neither callback arrival nor converted
presentation timestamps establish optical capture bounds. All eligibility stays
false, including when direct capture and pixel processing pass their own gate.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
import uuid

from learning import mac_camera_instrumentation as instrumentation

POLICY = {
    'maxSourceAgeMs': 100, 'maxAnchorAgeMs': 15000, 'maxPersistenceMs': 250,
    'referenceFrames': 1, 'evaluationFrames': 20, 'expectedFrames': 21,
    'minimumDiagnosticPassingEvaluationFrames': 16,
    'warmupSeconds': 1, 'maxWarmupCallbacks': 30, 'startupSeconds': 30,
    'measurementSeconds': 10, 'processTimeoutSeconds': 45, 'retryCount': 0,
}
_IDENTIFIER = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z')


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


def _identifier(value):
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ValueError('invalid_session_or_view_identifier')
    return value


def _json_bytes(value):
    return (json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + '\n').encode()


def _write_new(path, body):
    with Path(path).open('xb') as stream:
        stream.write(body)
        stream.flush()
        os.fsync(stream.fileno())
    directory = os.open(Path(path).parent, os.O_RDONLY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def _read_file(root, name):
    # Only direct regular files produced by this collector are in the contract.
    if not isinstance(name, str) or Path(name).name != name or name in ('.', '..'):
        raise ValueError('unsafe_artifact_path')
    path = root / name
    if path.is_symlink() or not path.is_file():
        raise ValueError('missing_or_unsafe_artifact:' + name)
    return path.read_bytes()


def _read_json(root, name):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate_json_key')
            result[key] = value
        return result
    value = json.loads(_read_file(root, name), object_pairs_hook=pairs,
                       parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite_json')))
    if not isinstance(value, dict):
        raise ValueError('json_object_required')
    return value


def create_plan(device_id, view_id, session_id=None, *, helper_sha256=None, required_instrumentation_version=None):
    if not isinstance(device_id, str) or not device_id.strip():
        raise ValueError('explicit_device_unique_id_required')
    plan = {
        'schemaVersion': 'motionbrain.mac_camera_plan.v1',
        'sessionId': _identifier(session_id or uuid.uuid4().hex),
        'viewConfigurationId': _identifier(view_id), 'deviceUniqueId': device_id,
        'policy': dict(POLICY), 'helperSha256': helper_sha256,
        'cameraViewChanged': True,
        'sourceTimestampSemantics': 'cmsamplebuffer_presentation_timestamp',
        'processingMethod': 'bgra_luma_mean_v1',
        'persistenceScope': 'raw_bgra_and_frame_metadata_fsync_readback_directory_fsync',
        'captureProcessingAcceptance': '21_integrity_valid_frames_and_all_storage_at_most_250ms_and_16_of_20_pts_diagnostics_within_100ms_and_reference_15000ms',
        'freshnessAcceptance': 'capture_age_upper_bound_at_processing_at_most_100ms_requires_capture_evidence',
        'anchorAcceptance': 'reference_capture_age_upper_bound_at_processing_at_most_15000ms_requires_capture_evidence',
        'captureHostBoundsCalibrated': False, 'controlEligible': False,
        'trainingEligible': False, 'physicalLearningEligibleRows': 0,
    }
    if required_instrumentation_version is not None:
        if type(required_instrumentation_version) is not int or required_instrumentation_version != 1:
            raise ValueError('unsupported_instrumentation_version')
        plan['requiredInstrumentationVersion'] = required_instrumentation_version
    return plan


def _identity(record, plan):
    return all(record.get(key) == plan[key]
               for key in ('sessionId', 'viewConfigurationId', 'deviceUniqueId'))


def evaluate_capture(directory):
    """Recheck every original frame and native inline measurement without mutation."""
    root = Path(directory)
    result = {
        'schemaVersion': 'motionbrain.mac_camera_evaluation.v1',
        'captureProcessingPassed': False, 'overallPassed': False,
        'sourceFreshnessGate': 'unverified', 'anchorFreshnessGate': 'unverified',
        'sourceAgeUpperBoundMs': None, 'anchorAgeUpperBoundMs': None,
        'captureHostBoundsCalibrated': False, 'controlEligible': False,
        'trainingEligible': False, 'physicalLearningEligibleRows': 0,
        'diagnosticPassingEvaluationFrames': 0,
        'policy': dict(POLICY), 'validatedFrames': 0, 'issues': [], 'frames': [],
        'limitations': ['no_optical_capture_bounds', 'pts_is_presentation_time',
                       'callback_arrival_is_not_capture_time', 'pixel_processing_is_not_object_detection',
                       'no_motion_or_action_transition', 'no_physical_learning_performance'],
    }
    issues = result['issues']
    try:
        if root.is_symlink() or not root.is_dir():
            raise ValueError('unsafe_or_missing_run_directory')
        plan = _read_json(root, 'plan.json')
        if (plan.get('schemaVersion') != 'motionbrain.mac_camera_plan.v1'
                or _json_bytes(plan.get('policy')) != _json_bytes(POLICY)
                or any(plan.get(key) is not False for key in ('captureHostBoundsCalibrated', 'controlEligible', 'trainingEligible'))):
            raise ValueError('invalid_or_modified_plan')
        _identifier(plan.get('sessionId'))
        _identifier(plan.get('viewConfigurationId'))
        execution = _read_json(root, 'execution.json')
        result['execution'] = execution
        elapsed = execution.get('elapsedSeconds')
        if (type(execution.get('returnCode')) is not int or execution['returnCode'] != 0
                or execution.get('timedOut') is not False or type(execution.get('retryCount')) is not int
                or execution['retryCount'] != 0 or execution.get('interrupted') is not False
                or execution.get('launchError') is not None
                or not _number(elapsed) or not 0 <= elapsed <= POLICY['processTimeoutSeconds']):
            issues.append('helper_execution_not_successful')
        result.update(sessionId=plan['sessionId'], viewConfigurationId=plan['viewConfigurationId'],
                      deviceUniqueId=plan.get('deviceUniqueId'))
        session = _read_json(root, 'session.json')
        if (session.get('schemaVersion') != 'motionbrain.mac_camera_session.v1'
                or not _identity(session, plan) or session.get('isContinuityCamera') is not True
                or session.get('deviceType') != 'AVCaptureDeviceTypeContinuityCamera'
                or session.get('requestedPreset') != '640x480'
                or session.get('requestedPixelFormat') != 'BGRA8'
                or session.get('processingMethod') != 'bgra_luma_mean_v1'
                or session.get('transportVerifiedUSB') is not False
                or any(session.get(k) is not False for k in ('captureHostBoundsCalibrated', 'controlEligible', 'trainingEligible'))):
            raise ValueError('session_configuration_invalid')
        result['sessionSha256'] = hashlib.sha256(_read_file(root, 'session.json')).hexdigest()
        terminal = _read_json(root, 'terminal.json')
        if (terminal.get('schemaVersion') != 'motionbrain.mac_camera_terminal.v1' or not _identity(terminal, plan)
                or any(terminal.get(k) is not False for k in ('captureHostBoundsCalibrated', 'controlEligible', 'trainingEligible'))):
            raise ValueError('terminal_identity_or_schema_invalid')
        if terminal.get('status') != 'completed' or terminal.get('reason') != 'sample_budget_complete':
            issues.append('capture_did_not_complete:' + str(terminal.get('reason')))
        if (type(terminal.get('warmupFrames')) is not int
                or not 0 <= terminal['warmupFrames'] <= POLICY['maxWarmupCallbacks']
                or type(terminal.get('droppedFrames')) is not int or terminal['droppedFrames'] < 0):
            raise ValueError('terminal_capture_counters_invalid')
        count = terminal.get('recordedFrames')
        if terminal.get('expectedFrames') != POLICY['expectedFrames']:
            raise ValueError('terminal_expected_frame_count_invalid')
        if type(count) is not int or not 0 <= count <= POLICY['expectedFrames']:
            raise ValueError('invalid_terminal_frame_count')
        if count != POLICY['expectedFrames']:
            issues.append('sample_budget_incomplete')
        receipts = terminal.get('frames')
        if (not isinstance(receipts, list) or len(receipts) != count
                or any(not isinstance(r, dict) or type(r.get('frameIndex')) is not int
                       or r['frameIndex'] != i for i, r in enumerate(receipts))):
            raise ValueError('terminal_receipt_index_invalid')
        instrument_state = instrumentation.begin(plan, session, terminal)
        original_files = {p.name for p in root.glob('frame-*')}
        expected_files = {f'frame-{i:04d}.{extension}' for i in range(count) for extension in ('json', 'bgra')}
        if original_files != expected_files:
            issues.append('unaccounted_or_missing_frame_artifact')
        shape = None
        orientation = None
        last_converted = None
        last_pts = None
        last_callback = None
        reference_pts = None
        first_callback = None
        for i, receipt in enumerate(receipts):
            metadata_name = f'frame-{i:04d}.json'
            if (receipt.get('durable') is not True or receipt.get('metadataFile') != metadata_name
                    or receipt.get('metadataSha256') != hashlib.sha256(_read_file(root, metadata_name)).hexdigest()
                    or receipt.get('imageFile') != f'frame-{i:04d}.bgra'):
                raise ValueError(f'terminal_artifact_binding_invalid:{i}')
            frame = _read_json(root, metadata_name)
            if (frame.get('schemaVersion') != 'motionbrain.mac_camera_frame.v1'
                    or not _identity(frame, plan) or type(frame.get('frameIndex')) is not int
                    or frame['frameIndex'] != i or frame.get('sourceFrameId') != f"{plan['sessionId']}:{i}"
                    or frame.get('timestampSemantics') != 'cmsamplebuffer_presentation_timestamp'
                    or frame.get('clockDomain') != 'coremedia_host_clock'
                    or frame.get('clockConversion') != 'session_synchronization_clock_to_host_clock'
                    or any(frame.get(k) is not False for k in ('captureHostBoundsCalibrated', 'controlEligible', 'trainingEligible'))):
                raise ValueError(f'frame_identity_or_semantics_invalid:{i}')
            width, height = frame.get('width'), frame.get('height')
            if (type(width) is not int or type(height) is not int
                    or not 1 <= width <= 1920 or not 1 <= height <= 1080
                    or frame.get('pixelFormat') != 'BGRA8' or type(frame.get('rowBytes')) is not int
                    or frame.get('rowBytes') != width * 4):
                raise ValueError(f'frame_format_invalid:{i}')
            if shape is not None and shape != (width, height):
                raise ValueError(f'frame_dimensions_changed:{i}')
            shape = (width, height)
            mirrored, rotation = frame.get('videoMirrored'), frame.get('videoRotationAngle')
            if type(mirrored) is not bool or not _number(rotation):
                raise ValueError(f'orientation_metadata_invalid:{i}')
            if orientation is not None and orientation != (mirrored, rotation):
                raise ValueError(f'orientation_changed:{i}')
            orientation = (mirrored, rotation)
            if frame.get('imageFile') != f'frame-{i:04d}.bgra':
                raise ValueError(f'frame_image_reference_invalid:{i}')
            raw = _read_file(root, frame['imageFile'])
            digest = hashlib.sha256(raw).hexdigest()
            if len(raw) != width * height * 4 or frame.get('imageSha256') != digest or receipt.get('imageSha256') != digest:
                raise ValueError(f'frame_bytes_or_hash_mismatch:{i}')
            pts = frame.get('pts')
            if (not isinstance(pts, dict)
                    or any(type(pts.get(k)) is not int for k in ('value', 'timescale', 'flags', 'epoch'))
                    or pts['timescale'] <= 0 or pts['value'] < 0 or not (pts['flags'] & 1)
                    or pts['flags'] & ~3):
                raise ValueError(f'presentation_timestamp_invalid:{i}')
            if last_pts is not None and (pts['epoch'] != last_pts['epoch']
                    or pts['value'] * last_pts['timescale'] <= last_pts['value'] * pts['timescale']):
                raise ValueError(f'presentation_timestamp_repeated_or_regressed:{i}')
            last_pts = pts
            callback, processed, storage = (frame.get(k) for k in
                                            ('callbackHostSeconds', 'processingCompleteHostSeconds', 'storageStartedHostSeconds'))
            completed, persistence = receipt.get('storageCompleteHostSeconds'), receipt.get('persistenceMs')
            stamps = (callback, processed, storage, completed)
            if (not all(_number(t) and t >= 0 for t in stamps) or list(stamps) != sorted(stamps)
                    or not _number(persistence) or persistence < 0
                    or abs((completed-storage)*1000-persistence) > 0.01
                    or (last_callback is not None and callback < last_callback)):
                raise ValueError(f'host_timing_or_persistence_invalid:{i}')
            last_callback = callback
            if first_callback is None:
                first_callback = callback
            if completed - first_callback > POLICY['measurementSeconds']:
                issues.append(f'measurement_budget_exceeded:{i}')
            if persistence > POLICY['maxPersistenceMs']:
                issues.append(f'persistence_exceeds_250ms:{i}')
            measure = frame.get('processing', {})
            mean = (77 * sum(raw[2::4]) + 150 * sum(raw[1::4]) + 29 * sum(raw[0::4])) / (256 * width * height)
            if (not isinstance(measure, dict) or measure.get('method') != 'bgra_luma_mean_v1'
                    or type(measure.get('pixelCount')) is not int
                    or measure.get('pixelCount') != width * height or not _number(measure.get('lumaMean'))
                    or abs(measure['lumaMean']-mean) > 1e-6):
                raise ValueError(f'pixel_processing_mismatch:{i}')
            converted = frame.get('convertedHostPtsSeconds')
            if (not _number(converted) or not 0 <= converted <= callback
                    or (last_converted is not None and converted <= last_converted)):
                raise ValueError(f'converted_pts_invalid:{i}')
            last_converted = converted
            instrumentation.check_frame(instrument_state, frame, receipt)
            if i == 0:
                reference_pts = converted
            row = {'frameIndex': i, 'imageSha256': digest,
                   'metadataSha256': hashlib.sha256(_read_file(root, metadata_name)).hexdigest(),
                   'callbackToProcessingMs': (processed-callback)*1000,
                   'persistenceMs': persistence, 'verifiedLumaMean': mean,
                   'ptsToProcessingDiagnosticMs': None if converted is None else (processed-converted)*1000,
                   'referencePtsToProcessingDiagnosticMs': None if reference_pts is None else (processed-reference_pts)*1000}
            row['diagnosticTimingPassed'] = (i > 0 and row['ptsToProcessingDiagnosticMs'] is not None
                and row['referencePtsToProcessingDiagnosticMs'] is not None
                and 0 <= row['ptsToProcessingDiagnosticMs'] <= POLICY['maxSourceAgeMs']
                and 0 <= row['referencePtsToProcessingDiagnosticMs'] <= POLICY['maxAnchorAgeMs'])
            if row['diagnosticTimingPassed']:
                result['diagnosticPassingEvaluationFrames'] += 1
            result['frames'].append(row)
            result['validatedFrames'] += 1
        if result['diagnosticPassingEvaluationFrames'] < POLICY['minimumDiagnosticPassingEvaluationFrames']:
            issues.append('insufficient_pts_diagnostic_timing_passes')
        result['captureProcessingPassed'] = not issues and count == POLICY['expectedFrames']
        result['instrumentation'] = instrumentation.summary(instrument_state)
        result['nativeStatus'] = terminal.get('status')
        result['droppedFrames'] = terminal.get('droppedFrames')
        result['warmupFrames'] = terminal.get('warmupFrames')
    except (OSError, ValueError, TypeError, KeyError) as exc:
        issues.append(str(exc))
    return result


def run_probe(helper, output, device_id, view_id, *, session_id=None):
    """One finite subprocess. No retry, service, network or robot-control calls."""
    helper = Path(helper).resolve(strict=True)
    if not helper.is_file() or not os.access(helper, os.X_OK):
        raise ValueError('executable_capture_helper_required')
    plan = create_plan(device_id, view_id, session_id,
                       helper_sha256=hashlib.sha256(helper.read_bytes()).hexdigest(),
                       required_instrumentation_version=1)
    output = Path(output).absolute()
    output.mkdir(parents=True, exist_ok=False)
    _write_new(output / 'plan.json', _json_bytes(plan))
    command = [str(helper), '--capture', device_id, str(output), plan['sessionId'], view_id]
    started = time.monotonic()
    process = None
    timed_out = False
    interrupted = None
    launch_error = None
    stdout, stderr = b'', b''

    def cleanup():
        process.terminate()
        try:
            return process.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.communicate()

    try:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate(timeout=POLICY['processTimeoutSeconds'])
    except subprocess.TimeoutExpired:
        timed_out = True
        stdout, stderr = cleanup()
    except OSError as exc:
        launch_error = str(exc)
        if process is not None:
            stdout, stderr = cleanup()
    except BaseException as exc:
        interrupted = exc
        if process is not None:
            stdout, stderr = cleanup()
    elapsed_seconds = time.monotonic()-started
    _write_new(output / 'helper.stdout', stdout)
    _write_new(output / 'helper.stderr', stderr)
    execution = {'returnCode': None if process is None else process.returncode,
                 'timedOut': timed_out, 'interrupted': interrupted is not None,
                 'launchError': launch_error, 'cleanupGraceSeconds': 2,
                 'elapsedSeconds': elapsed_seconds, 'retryCount': 0}
    _write_new(output / 'execution.json', _json_bytes(execution))
    report = evaluate_capture(output)
    if timed_out or interrupted is not None or launch_error is not None or execution['returnCode'] != 0:
        report['captureProcessingPassed'] = False
        reason = ('helper_timeout' if timed_out else 'helper_interrupted' if interrupted is not None
                  else 'helper_launch_failed' if launch_error is not None else 'helper_nonzero_exit')
        report['issues'].append(reason)
    report['execution'] = execution
    _write_new(output / 'evaluation.json', _json_bytes(report))
    if interrupted is not None:
        raise interrupted
    return report
