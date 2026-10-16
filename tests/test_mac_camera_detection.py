"""Synthetic BGRA records and fake detector outputs; no pretrained inference."""
import json
from pathlib import Path
import subprocess
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

import test_mac_camera_probe as camera_fixture
from learning.mac_camera_detection import (POLICY, prepare_plan, source_evidence, worker,
                                           run_evaluation, _write_json, _verify_plan)
from motionbrain_ros_bridge.vision_detection import DetectionCandidate


class FakeDetector:
    def __init__(self, detected=True):
        self.labels = ['person', 'cup']
        self.net = Mock()
        self.calls = []
        self.detected = detected
        self.name = 'synthetic-detector'

    def detect(self, encoded, config):
        image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape != (1, 2, 3):
            raise AssertionError('lossless fake source image required')
        np.testing.assert_array_equal(image[0], [[10, 20, 30], [40, 50, 60]])
        self.calls.append(config)
        if self.detected:
            return [DetectionCandidate('object', 'cup', 0, 0, 1, 1, 2, 1, confidence=.75, class_id=1)]
        return []


class OfflineDetectionTest(unittest.TestCase):
    def setUp(self):
        camera_fixture.CameraProbeTest.setUp(self)
        self.model = Path(self.tmp.name)/'fixture.onnx'
        self.model.write_bytes(b'synthetic-model-not-for-inference')
        self.labels = Path(self.tmp.name)/'labels.txt'
        self.labels.write_text('person\ncup\n')
        self.output = Path(self.tmp.name)/'evaluation'
        # These are fake-detector contract tests, not measurements of the host's
        # OpenCV parallel backend. The frozen Mac runtime guard is tested below.
        set_threads = patch('cv2.setNumThreads')
        get_threads = patch('cv2.getNumThreads', return_value=1)
        self.set_threads = set_threads.start()
        self.get_threads = get_threads.start()
        self.addCleanup(set_threads.stop)
        self.addCleanup(get_threads.stop)

    save_frame = camera_fixture.CameraProbeTest.save_frame
    save_terminal = camera_fixture.CameraProbeTest.save_terminal

    def make_plan(self):
        return prepare_plan(self.root, self.model, self.labels)

    def setup_worker(self):
        self.output.mkdir()
        _write_json(self.output/'plan.json', self.make_plan())

    def evaluate_worker(self, fake=None):
        fake = fake or FakeDetector()
        with patch('learning.mac_camera_detection.OpenCvDnnObjectDetector.from_model', return_value=fake):
            report = worker(self.output)
        return report, fake

    def test_timing_failed_originals_are_valid_for_offline_processing_only(self):
        for i in range(1, 6):
            self.frames[i]['convertedHostPtsSeconds'] -= .15
            self.save_frame(i)
        source = source_evidence(self.root)
        self.assertFalse(source['captureProcessingPassed'])
        self.assertEqual(source['sourceTimingIssues'], ['insufficient_pts_diagnostic_timing_passes'])

    def test_hash_and_incomplete_session_rejected_before_model_load(self):
        (self.root/'frame-0001.bgra').write_bytes(bytes(8))
        with patch('learning.mac_camera_detection.OpenCvDnnObjectDetector.from_model') as load:
            with self.assertRaisesRegex(ValueError, 'integrity_invalid'):
                self.make_plan()
        load.assert_not_called()

    def test_missing_durable_receipt_rejected(self):
        self.terminal['frames'][0]['durable'] = False
        self.save_terminal()
        with self.assertRaisesRegex(ValueError, 'integrity_invalid'):
            self.make_plan()

    def test_missing_execution_record_rejected(self):
        (self.root/'execution.json').unlink()
        with self.assertRaisesRegex(ValueError, 'integrity_invalid'):
            self.make_plan()

    def test_plan_has_frozen_inputs_policy_and_implementation(self):
        plan = self.make_plan()
        self.assertEqual(plan['policy'], POLICY)
        self.assertEqual(plan['policy']['objectTargetAliases'], [])
        self.assertEqual(len(plan['source']['files']), 46)
        self.assertEqual(len(plan['model']['sha256']), 64)
        self.assertGreaterEqual(len(plan['sourceCodeSha256']), 5)
        self.assertFalse(plan['accuracyAvailable'])
        self.assertFalse(plan['liveLatencyEstablished'])

    def test_one_warmup_and_twenty_one_exact_frame_evaluations(self):
        self.setup_worker()
        report, fake = self.evaluate_worker()
        self.assertEqual(len(fake.calls), 22)
        self.set_threads.assert_called_once_with(0)
        self.assertEqual(report['evaluationFrames'], 21)
        self.assertEqual(report['warmupFrames'], 1)
        self.assertEqual(report['detectedFrames'], 21)
        self.assertEqual(len(list(self.output.glob('detection-*.json'))), 21)
        for config in fake.calls:
            self.assertEqual(config.object_target, 'cup')
            self.assertEqual(config.object_min_confidence, .20)
            self.assertEqual(config.object_nms_threshold, .45)
            self.assertEqual(config.object_input_size, 640)
            self.assertEqual(config.object_target_aliases, ())
            self.assertEqual(config.target_policy, 'largest')
        fake.net.setPreferableTarget.assert_called_once_with(cv2.dnn.DNN_TARGET_CPU)
        fake.net.setPreferableBackend.assert_called_once_with(cv2.dnn.DNN_BACKEND_OPENCV)
        self.assertFalse(report['accuracyAvailable'])
        self.assertFalse(report['trainingEligible'])
        self.assertFalse(report['controlEligible'])
        self.assertFalse(report['liveLatencyEstablished'])
        self.assertEqual(report['physicalLearningEligibleRows'], 0)
        self.assertNotIn('mAP', report)
        self.assertNotIn('sourceAgeMs', report['frames'][0])
        self.assertNotIn('commandSuggestion', report['frames'][0])

    def test_zero_detections_complete_evaluation_without_claiming_accuracy(self):
        self.setup_worker()
        report, _ = self.evaluate_worker(FakeDetector(detected=False))
        self.assertEqual(report['detectedFrames'], 0)
        self.assertTrue(all(row['target'] is None for row in report['frames']))
        self.assertEqual(report['status'], 'completed')
        self.assertFalse(report['accuracyAvailable'])

    def test_unsupported_thread_report_is_rejected_before_model_load(self):
        self.setup_worker()
        for reported_threads in (0, 2):
            with self.subTest(reported_threads=reported_threads):
                self.get_threads.return_value = reported_threads
                self.set_threads.reset_mock()
                with patch('learning.mac_camera_detection.OpenCvDnnObjectDetector.from_model') as load:
                    with self.assertRaisesRegex(ValueError, 'opencv_sequential_processing_unavailable'):
                        worker(self.output)
                self.set_threads.assert_called_once_with(0)
                load.assert_not_called()
                self.assertFalse((self.output/'worker-result.json').exists())
                self.assertFalse(list(self.output.glob('detection-*.json')))

    def test_wrong_or_ambiguous_labels_are_rejected(self):
        self.setup_worker()
        fake = FakeDetector()
        fake.labels = ['cup', 'cup']
        with patch('learning.mac_camera_detection.OpenCvDnnObjectDetector.from_model', return_value=fake):
            with self.assertRaisesRegex(ValueError, 'exactly_one_cup'):
                worker(self.output)
        self.assertEqual(fake.calls, [])

    def test_model_change_is_rejected_before_loading(self):
        self.setup_worker()
        self.model.write_bytes(b'changed')
        with patch('learning.mac_camera_detection.OpenCvDnnObjectDetector.from_model') as load:
            with self.assertRaisesRegex(ValueError, 'model_changed_since_plan'):
                worker(self.output)
        load.assert_not_called()

    def test_configuration_changes_are_rejected(self):
        plan = self.make_plan()
        plan['policy']['objectMinConfidence'] = .1
        with self.assertRaisesRegex(ValueError, 'invalid_frozen'):
            _verify_plan(plan)

    def test_source_changes_after_plan_rejected(self):
        self.setup_worker()
        self.frames[1]['videoRotationAngle'] = 90
        self.save_frame(1)
        with self.assertRaisesRegex(ValueError, 'integrity_invalid'):
            worker(self.output)

    def test_non_onnx_model_does_not_change_frozen_backend(self):
        other = Path(self.tmp.name)/'fixture.bin'
        other.write_bytes(b'synthetic')
        with self.assertRaisesRegex(ValueError, 'onnx_model_required'):
            prepare_plan(self.root, other, self.labels)

    def test_unavailable_warmup_does_not_continue_evaluation(self):
        self.setup_worker()
        with patch('learning.mac_camera_detection.OpenCvDnnObjectDetector.from_model', return_value=FakeDetector()), \
             patch('learning.mac_camera_detection.detect_frame', return_value={'available':False}) as detect:
            with self.assertRaisesRegex(ValueError, 'warmup_detector_unavailable'):
                worker(self.output)
        detect.assert_called_once()
        self.assertFalse((self.output/'worker-result.json').exists())

    def test_source_originals_are_not_modified_by_replay(self):
        original = {p.name:p.read_bytes() for p in self.root.iterdir()}
        self.setup_worker()
        self.evaluate_worker()
        self.assertEqual(original, {p.name:p.read_bytes() for p in self.root.iterdir()})

    def test_successful_supervision_requires_worker_artifacts(self):
        fake = FakeDetector()
        process = Mock(returncode=0)
        def communicate(timeout=None):
            self.assertEqual(timeout, 60)
            worker(self.output)
            return b'finished', b''
        process.communicate.side_effect = communicate
        with patch('learning.mac_camera_detection.subprocess.Popen', return_value=process) as spawn, \
             patch('learning.mac_camera_detection.OpenCvDnnObjectDetector.from_model', return_value=fake):
            report = run_evaluation(self.root, self.model, self.labels, self.output)
        spawn.assert_called_once()
        self.assertTrue(report['offlineEvaluationComplete'], report)
        self.assertEqual(report['results']['detectedFrames'], 21)
        self.assertFalse(report['liveLatencyEstablished'])
        self.assertTrue((self.output/'execution.json').exists())
        self.assertEqual(len(fake.calls), 22)

    def test_success_exit_without_terminal_worker_artifacts_is_failure(self):
        process = Mock(returncode=0)
        process.communicate.return_value = b'', b''
        with patch('learning.mac_camera_detection.subprocess.Popen', return_value=process):
            report = run_evaluation(self.root, self.model, self.labels, self.output)
        self.assertFalse(report['offlineEvaluationComplete'])

    def test_timeout_terminates_kills_and_never_retries(self):
        process = Mock(returncode=-9)
        process.communicate.side_effect = [subprocess.TimeoutExpired('fixture', 60),
                                          subprocess.TimeoutExpired('fixture', 2), (b'partial', b'failed')]
        with patch('learning.mac_camera_detection.subprocess.Popen', return_value=process) as spawn:
            report = run_evaluation(self.root, self.model, self.labels, self.output)
        spawn.assert_called_once()
        process.terminate.assert_called_once()
        process.kill.assert_called_once()
        self.assertFalse(report['offlineEvaluationComplete'])
        self.assertTrue(report['execution']['timedOut'])
        self.assertEqual((self.output/'worker.stdout').read_bytes(), b'partial')

    def test_interrupt_preserves_failure_and_cleans_child_before_reraise(self):
        process = Mock(returncode=-15)
        process.communicate.side_effect = [KeyboardInterrupt(), (b'partial', b'interrupted')]
        with patch('learning.mac_camera_detection.subprocess.Popen', return_value=process):
            with self.assertRaises(KeyboardInterrupt):
                run_evaluation(self.root, self.model, self.labels, self.output)
        process.terminate.assert_called_once()
        self.assertTrue(json.loads((self.output/'execution.json').read_text())['interrupted'])
        self.assertFalse(json.loads((self.output/'evaluation.json').read_text())['offlineEvaluationComplete'])

    def test_launch_failure_and_existing_output_preserved(self):
        with patch('learning.mac_camera_detection.subprocess.Popen', side_effect=OSError('fixture launch failed')):
            report = run_evaluation(self.root, self.model, self.labels, self.output)
        self.assertFalse(report['offlineEvaluationComplete'])
        self.assertIn('fixture launch failed', report['execution']['error'])
        with self.assertRaises(FileExistsError):
            run_evaluation(self.root, self.model, self.labels, self.output)


    def test_instrumentation_implementation_is_frozen(self):
        plan=self.make_plan()
        self.assertIn('learning/mac_camera_instrumentation.py',plan['sourceCodeSha256'])
        plan['sourceCodeSha256']['learning/mac_camera_instrumentation.py']='0'*64
        with self.assertRaisesRegex(ValueError,'implementation_changed_since_plan'):
            _verify_plan(plan)


if __name__ == '__main__':
    unittest.main()
