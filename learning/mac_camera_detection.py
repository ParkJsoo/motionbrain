"""Frozen, finite offline detector evaluation of immutable Mac commissioning frames.

Detection existence is not accuracy, and replay inference time is not a live
capture-to-result measurement. This module never creates action-training rows.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from learning.mac_camera_probe import evaluate_capture, _read_file, _read_json, _write_new, _json_bytes

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / 'ros2_ws/src/motionbrain_ros_bridge'
if str(BRIDGE) not in sys.path:
    sys.path.insert(0, str(BRIDGE))
from motionbrain_ros_bridge.vision_detection import DetectionConfig, OpenCvDnnObjectDetector, detect_frame

POLICY = {'evaluationFrames': 21, 'warmupFrames': 1, 'warmupFrameIndex': 0,
          'deadlineSeconds': 60, 'cleanupGraceSeconds': 2, 'retryCount': 0,
          'objectTarget': 'cup', 'objectMinConfidence': 0.20, 'objectNmsThreshold': 0.45,
          'objectInputSize': 640, 'objectTargetAliases': [], 'targetPolicy': 'largest',
          'opencvThreads': 1, 'opencvSetNumThreadsArgument': 0,
          'opencvParallelRegionsDisabled': True, 'backend': 'opencv-dnn', 'computeTarget': 'CPU'}
SOURCES = ['learning/mac_camera_detection.py', 'learning/mac_camera_probe.py',
           'learning/mac_camera_instrumentation.py',
           'tools/evaluate_mac_camera_detection.py',
           'ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/vision_detection.py',
           'ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/payload_utils.py']


def _digest(raw):
    return hashlib.sha256(raw).hexdigest()


def _hash_file(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError('regular_model_or_label_file_required')
    return _digest(path.read_bytes())


def _write_json(path, document):
    _write_new(path, _json_bytes(document))


def source_evidence(directory):
    """Accept only an intact complete capture; timing-only rejection stays recorded."""
    root = Path(directory)
    audit = evaluate_capture(root)
    allowed = {'insufficient_pts_diagnostic_timing_passes'}
    if (audit.get('validatedFrames') != 21 or audit.get('nativeStatus') != 'completed'
            or any(issue not in allowed for issue in audit.get('issues', []))):
        raise ValueError('source_capture_incomplete_or_integrity_invalid:' + ';'.join(audit.get('issues', [])))
    names = ['plan.json', 'session.json', 'terminal.json', 'execution.json']
    names += [f'frame-{i:04d}.{extension}' for i in range(21) for extension in ('json', 'bgra')]
    hashes = {name: _digest(_read_file(root, name)) for name in names}
    return {'captureDirectory': str(root.absolute()), 'sessionId': audit['sessionId'],
            'viewConfigurationId': audit['viewConfigurationId'], 'files': hashes,
            'evidenceSha256': _digest(_json_bytes(hashes)), 'sourceTimingIssues': audit['issues'],
            'captureProcessingPassed': audit['captureProcessingPassed']}


def prepare_plan(capture_directory, model, labels):
    model, labels = Path(model).absolute(), Path(labels).absolute()
    if model.suffix.lower() != '.onnx':
        raise ValueError('onnx_model_required_for_frozen_backend')
    return {'schemaVersion': 'motionbrain.mac_camera_detection_plan.v1', 'policy': dict(POLICY),
            'source': source_evidence(capture_directory),
            'model': {'path': str(model), 'sha256': _hash_file(model), 'declaredFamily': 'YOLOv5s-ONNX-pretrained'},
            'labels': {'path': str(labels), 'sha256': _hash_file(labels)},
            'sourceCodeSha256': {name: _hash_file(ROOT/name) for name in SOURCES},
            'inputConversion': 'exact_BGRA8_to_BGR_then_lossless_PNG_for_existing_detector',
            'trainingEligible': False, 'controlEligible': False,
            'accuracyAvailable': False, 'liveLatencyEstablished': False}


def _verify_plan(plan):
    if (plan.get('schemaVersion') != 'motionbrain.mac_camera_detection_plan.v1'
            or _json_bytes(plan.get('policy')) != _json_bytes(POLICY)
            or any(plan.get(k) is not False for k in ('trainingEligible', 'controlEligible',
                                                    'accuracyAvailable', 'liveLatencyEstablished'))):
        raise ValueError('invalid_frozen_detection_plan')
    actual = source_evidence(plan['source']['captureDirectory'])
    if actual != plan['source']:
        raise ValueError('source_capture_changed_since_plan')
    for key in ('model', 'labels'):
        if _hash_file(plan[key]['path']) != plan[key]['sha256']:
            raise ValueError(key + '_changed_since_plan')
    if {name: _hash_file(ROOT/name) for name in SOURCES} != plan['sourceCodeSha256']:
        raise ValueError('implementation_changed_since_plan')


def _image(plan, index, cv2, numpy):
    root = Path(plan['source']['captureDirectory'])
    metadata_name, image_name = f'frame-{index:04d}.json', f'frame-{index:04d}.bgra'
    if _digest(_read_file(root, metadata_name)) != plan['source']['files'][metadata_name]:
        raise ValueError('frame_metadata_changed')
    metadata = _read_json(root, metadata_name)
    raw = _read_file(root, image_name)
    if _digest(raw) != plan['source']['files'][image_name]:
        raise ValueError('frame_pixels_changed')
    pixels = numpy.frombuffer(raw, dtype=numpy.uint8).reshape(metadata['height'], metadata['width'], 4)
    bgr = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    ok, encoded = cv2.imencode('.png', bgr)
    if not ok:
        raise ValueError('lossless_input_encoding_failed')
    return encoded.tobytes(), metadata


def _candidate(value):
    if not isinstance(value, dict):
        return None
    return {key: value.get(key) for key in ('label', 'classId', 'confidence', 'targetBox', 'width', 'height')}


def worker(output):
    """Called only inside the parent's bounded child process; never a retry loop."""
    output = Path(output)
    plan = _read_json(output, 'plan.json')
    _verify_plan(plan)
    import cv2
    import numpy
    # Disabling OpenCV parallel regions works with GCD builds where requesting
    # one thread still reports the machine CPU count. This is a fixed policy,
    # not runtime thread-count tuning or a second model execution.
    cv2.setNumThreads(POLICY['opencvSetNumThreadsArgument'])
    if cv2.getNumThreads() != 1:
        raise ValueError('opencv_sequential_processing_unavailable')
    config = DetectionConfig(mode='object', object_target='cup', object_min_confidence=.20,
                             object_nms_threshold=.45, object_input_size=640,
                             object_backend='opencv-dnn', object_model=plan['model']['path'],
                             object_labels=plan['labels']['path'], object_target_aliases=(), target_policy='largest')
    load_started = time.monotonic()
    detector = OpenCvDnnObjectDetector.from_model(plan['model']['path'], plan['labels']['path'], input_size=640)
    detector.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    detector.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    if list(detector.labels).count('cup') != 1:
        raise ValueError('exactly_one_cup_label_required')
    if _hash_file(plan['model']['path']) != plan['model']['sha256'] or _hash_file(plan['labels']['path']) != plan['labels']['sha256']:
        raise ValueError('model_or_labels_changed_during_load')
    loaded = time.monotonic()
    encoded, _ = _image(plan, 0, cv2, numpy)
    warm_started = time.monotonic()
    warmup = detect_frame(encoded, config, detector)
    if warmup.get('available') is not True:
        raise ValueError('warmup_detector_unavailable')
    warm_ms = (time.monotonic()-warm_started)*1000
    rows = []
    for index in range(21):
        started = time.monotonic()
        encoded, metadata = _image(plan, index, cv2, numpy)
        infer_started = time.monotonic()
        detection = detect_frame(encoded, config, detector)
        completed = time.monotonic()
        if detection.get('available') is not True or type(detection.get('detected')) is not bool:
            raise ValueError('detector_unavailable_or_invalid')
        target = _candidate(detection.get('target')) if detection['detected'] else None
        if detection['detected'] and (target is None or target['label'] != 'cup'):
            raise ValueError('detector_selected_wrong_target')
        row = {'schemaVersion': 'motionbrain.mac_camera_detection_frame.v1',
               'frameIndex': index, 'sourceFrameId': metadata['sourceFrameId'],
               'sourceImageSha256': metadata['imageSha256'], 'derivedPngSha256': _digest(encoded),
               'detected': detection['detected'], 'target': target,
               'objects': [_candidate(item) for item in detection.get('objects', [])],
               'reason': detection.get('reason'),
               'inputPreparationMs': (infer_started-started)*1000,
               'inferenceMs': (completed-infer_started)*1000,
               'offlineProcessingMs': (completed-started)*1000,
               'trainingEligible': False, 'controlEligible': False}
        _write_json(output/f'detection-{index:04d}.json', row)
        rows.append(row)
    _verify_plan(plan)
    report = {'schemaVersion': 'motionbrain.mac_camera_detection_worker.v1', 'status': 'completed',
              'planSha256': _digest(_read_file(output, 'plan.json')),
              'evaluationFrames': len(rows), 'warmupFrames': 1, 'warmupFrameIndex': 0,
              'loadModelMs': (loaded-load_started)*1000, 'warmupInferenceMs': warm_ms,
              'opencvVersion': cv2.__version__, 'numpyVersion': numpy.__version__,
              'opencvThreads': cv2.getNumThreads(), 'opencvParallelRegionsDisabled': True,
              'detectedFrames': sum(row['detected'] for row in rows),
              'frames': rows, 'accuracyAvailable': False, 'accuracyReason': 'no_manual_ground_truth',
              'liveLatencyEstablished': False, 'trainingEligible': False, 'controlEligible': False,
              'physicalLearningEligibleRows': 0,
              'limitations': ['offline_replay_not_live_capture_latency', 'detection_existence_is_not_accuracy',
                              'no_manual_ground_truth_or_map', 'temporally_adjacent_single_scene',
                              'no_action_transition_or_learning']}
    _write_json(output/'worker-result.json', report)
    return report


def run_evaluation(capture_directory, model, labels, output):
    """Freeze inputs before model load, then supervise exactly one worker for 60 s."""
    plan = prepare_plan(capture_directory, model, labels)
    output = Path(output).absolute()
    output.mkdir(parents=True, exist_ok=False)
    _write_json(output/'plan.json', plan)
    plan_hash = _digest(_read_file(output, 'plan.json'))
    command = [sys.executable, str(ROOT/'tools/evaluate_mac_camera_detection.py'), '--worker-output', str(output)]
    started = time.monotonic()
    process = None
    timed_out, interrupted, error = False, None, None
    stdout, stderr = b'', b''

    def cleanup():
        process.terminate()
        try:
            return process.communicate(timeout=POLICY['cleanupGraceSeconds'])
        except subprocess.TimeoutExpired:
            process.kill()
            return process.communicate()

    try:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate(timeout=POLICY['deadlineSeconds'])
    except subprocess.TimeoutExpired:
        timed_out = True
        stdout, stderr = cleanup()
    except OSError as exc:
        error = str(exc)
        if process is not None:
            stdout, stderr = cleanup()
    except BaseException as exc:
        interrupted = exc
        if process is not None:
            stdout, stderr = cleanup()
    elapsed = time.monotonic()-started
    _write_new(output/'worker.stdout', stdout)
    _write_new(output/'worker.stderr', stderr)
    execution = {'returnCode': None if process is None else process.returncode, 'timedOut': timed_out,
                 'interrupted': interrupted is not None, 'error': error, 'elapsedSeconds': elapsed,
                 'retryCount': 0, 'cleanupGraceSeconds': POLICY['cleanupGraceSeconds']}
    _write_json(output/'execution.json', execution)
    report = {'schemaVersion': 'motionbrain.mac_camera_detection_evaluation.v1',
              'status': 'failed', 'offlineEvaluationComplete': False, 'execution': execution,
              'planSha256': plan_hash, 'trainingEligible': False, 'controlEligible': False,
              'physicalLearningEligibleRows': 0, 'accuracyAvailable': False, 'liveLatencyEstablished': False}
    try:
        if timed_out or interrupted is not None or error is not None or execution['returnCode'] != 0 or elapsed > 60:
            raise ValueError('worker_did_not_complete_within_frozen_budget')
        if _digest(_read_file(output, 'plan.json')) != plan_hash:
            raise ValueError('frozen_plan_changed')
        _verify_plan(plan)
        result = _read_json(output, 'worker-result.json')
        if (result.get('schemaVersion') != 'motionbrain.mac_camera_detection_worker.v1'
                or result.get('status') != 'completed' or result.get('planSha256') != plan_hash
                or result.get('evaluationFrames') != 21 or result.get('warmupFrames') != 1
                or result.get('warmupFrameIndex') != 0 or result.get('opencvThreads') != 1
                or any(result.get(k) is not False for k in ('trainingEligible', 'controlEligible', 'accuracyAvailable', 'liveLatencyEstablished'))
                or not isinstance(result.get('frames'), list) or len(result['frames']) != 21):
            raise ValueError('worker_result_incomplete_or_invalid')
        for index, row in enumerate(result['frames']):
            if row.get('frameIndex') != index or _read_json(output, f'detection-{index:04d}.json') != row:
                raise ValueError('worker_frame_evidence_mismatch')
        report.update(status='completed', offlineEvaluationComplete=True, results=result)
    except (ValueError, TypeError, KeyError, OSError) as exc:
        report['reason'] = str(exc)
    _write_json(output/'evaluation.json', report)
    if interrupted is not None:
        raise interrupted
    return report
