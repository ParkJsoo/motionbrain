"""Synthetic BGRA evidence and fake processes, never physical-camera performance."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

from learning.mac_camera_probe import POLICY, create_plan, evaluate_capture, run_probe


def write_json(path, value):
    path.write_text(json.dumps(value), encoding='utf-8')


class CameraProbeTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name) / 'run'
        self.root.mkdir()
        self.plan = create_plan('synthetic-device', 'synthetic-new-desk', 'synthetic-session')
        write_json(self.root / 'plan.json', self.plan)
        self.execution = dict(returnCode=0, timedOut=False, retryCount=0, interrupted=False,
                              launchError=None, elapsedSeconds=5.0, cleanupGraceSeconds=2)
        write_json(self.root/'execution.json', self.execution)
        self.session = dict(schemaVersion='motionbrain.mac_camera_session.v1',
                            sessionId=self.plan['sessionId'], viewConfigurationId=self.plan['viewConfigurationId'],
                            deviceUniqueId=self.plan['deviceUniqueId'], isContinuityCamera=True,
                            deviceType='AVCaptureDeviceTypeContinuityCamera', requestedPreset='640x480',
                            requestedPixelFormat='BGRA8', processingMethod='bgra_luma_mean_v1',
                            transportVerifiedUSB=False, captureHostBoundsCalibrated=False,
                            controlEligible=False, trainingEligible=False)
        write_json(self.root/'session.json', self.session)
        self.frames = []
        self.terminal = dict(schemaVersion='motionbrain.mac_camera_terminal.v1',
                             sessionId=self.plan['sessionId'], viewConfigurationId=self.plan['viewConfigurationId'],
                             deviceUniqueId=self.plan['deviceUniqueId'], status='completed', reason='sample_budget_complete',
                             recordedFrames=21, expectedFrames=21, droppedFrames=0, warmupFrames=30, frames=[],
                             captureHostBoundsCalibrated=False, controlEligible=False, trainingEligible=False)
        raw = bytes([10, 20, 30, 255, 40, 50, 60, 255])
        for i in range(21):
            timestamp = 100 + i * .2
            image_file = f'frame-{i:04d}.bgra'
            (self.root / image_file).write_bytes(raw)
            digest = hashlib.sha256(raw).hexdigest()
            frame = dict(schemaVersion='motionbrain.mac_camera_frame.v1',
                         sessionId=self.plan['sessionId'], viewConfigurationId=self.plan['viewConfigurationId'],
                         deviceUniqueId=self.plan['deviceUniqueId'], sourceFrameId=f"{self.plan['sessionId']}:{i}",
                         frameIndex=i, imageFile=image_file, imageSha256=digest,
                         width=2, height=1, pixelFormat='BGRA8', rowBytes=8,
                         pts=dict(value=100000+i*200, timescale=1000, flags=1, epoch=0),
                         convertedHostPtsSeconds=timestamp, callbackHostSeconds=timestamp+.002,
                         processingCompleteHostSeconds=timestamp+.003, storageStartedHostSeconds=timestamp+.004,
                         processing=dict(method='bgra_luma_mean_v1', pixelCount=2,
                                         lumaMean=(77*90+150*70+29*50)/(256*2)),
                         timestampSemantics='cmsamplebuffer_presentation_timestamp',
                         clockDomain='coremedia_host_clock', clockConversion='session_synchronization_clock_to_host_clock',
                         videoMirrored=False, videoRotationAngle=0,
                         captureHostBoundsCalibrated=False, controlEligible=False, trainingEligible=False)
            self.frames.append(frame)
            self.terminal['frames'].append(dict(frameIndex=i, metadataFile=f'frame-{i:04d}.json',
                                                imageFile=image_file,imageSha256=digest,
                                                durable=True, persistenceMs=10, storageCompleteHostSeconds=timestamp+.014))
            self.save_frame(i)
        self.save_terminal()

    def save_terminal(self):
        write_json(self.root / 'terminal.json', self.terminal)

    def save_frame(self, index):
        name = f'frame-{index:04d}.json'
        write_json(self.root / name, self.frames[index])
        self.terminal['frames'][index]['metadataSha256'] = hashlib.sha256((self.root/name).read_bytes()).hexdigest()
        self.save_terminal()

    def result(self):
        return evaluate_capture(self.root)

    def rejected(self, text):
        result = self.result()
        self.assertFalse(result['captureProcessingPassed'], result)
        self.assertTrue(any(text in s for s in result['issues']), result)
        self.assertFalse(result['overallPassed'])
        self.assertFalse(result['trainingEligible'])
        return result

    def test_valid_processing_is_not_capture_freshness_or_learning(self):
        before = {p.name: p.read_bytes() for p in self.root.iterdir()}
        result = self.result()
        self.assertTrue(result['captureProcessingPassed'], result)
        self.assertEqual(result['validatedFrames'], 21)
        self.assertEqual(result['diagnosticPassingEvaluationFrames'], 20)
        self.assertFalse(result['overallPassed'])
        self.assertFalse(result['captureHostBoundsCalibrated'])
        self.assertFalse(result['controlEligible'])
        self.assertFalse(result['trainingEligible'])
        self.assertEqual(result['physicalLearningEligibleRows'], 0)
        self.assertEqual(result['sourceFreshnessGate'], 'unverified')
        self.assertEqual(result['anchorFreshnessGate'], 'unverified')
        self.assertIsNone(result['sourceAgeUpperBoundMs'])
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.root.iterdir()})

    def test_no_callback_substitution_when_pts_conversion_missing(self):
        for i in range(21):
            self.frames[i]['convertedHostPtsSeconds'] = None
            self.save_frame(i)
        result = self.rejected('converted_pts_invalid')
        self.assertEqual(result['frames'], [])

    def test_fixed_sixteen_of_twenty_diagnostic_budget(self):
        for i in range(1, 5):
            self.frames[i]['convertedHostPtsSeconds'] -= .15
            self.save_frame(i)
        self.assertTrue(self.result()['captureProcessingPassed'])
        self.frames[5]['convertedHostPtsSeconds'] -= .15
        self.save_frame(5)
        self.rejected('insufficient_pts_diagnostic_timing_passes')

    def test_reference_age_limit_not_relaxed(self):
        self.frames[0]['convertedHostPtsSeconds'] -= 16
        self.save_frame(0)
        self.rejected('insufficient_pts_diagnostic_timing_passes')

    def test_future_converted_pts_cannot_pass_with_negative_age(self):
        for i in range(21):
            self.frames[i]['convertedHostPtsSeconds'] += 1
            self.save_frame(i)
        self.rejected('converted_pts_invalid')

    def test_storage_limit_includes_reference(self):
        self.terminal['frames'][0]['persistenceMs'] = 250.001
        self.terminal['frames'][0]['storageCompleteHostSeconds'] = self.frames[0]['storageStartedHostSeconds'] + .250001
        self.save_terminal()
        self.rejected('persistence_exceeds_250ms:0')

    def test_raw_hash_mismatch(self):
        (self.root / 'frame-0001.bgra').write_bytes(bytes(8))
        self.rejected('frame_bytes_or_hash_mismatch')

    def test_metadata_hash_binding(self):
        self.frames[1]['processing']['lumaMean'] += 1
        write_json(self.root / 'frame-0001.json', self.frames[1])
        self.rejected('terminal_artifact_binding_invalid')

    def test_recomputed_processing_verification(self):
        self.frames[1]['processing']['lumaMean'] += 1
        self.save_frame(1)
        self.rejected('pixel_processing_mismatch')

    def test_rejects_changed_view_and_eligibility_promotion(self):
        original = copy.deepcopy(self.frames[1])
        for field, value in [('viewConfigurationId', 'old-view'), ('controlEligible', True),
                             ('trainingEligible', True), ('captureHostBoundsCalibrated', True),
                             ('timestampSemantics', 'callback_arrival')]:
            with self.subTest(field=field):
                self.frames[1] = {**original, field:value}
                self.save_frame(1)
                self.rejected('frame_identity_or_semantics_invalid')

    def test_repeated_or_regressing_pts(self):
        self.frames[1]['pts'] = self.frames[0]['pts'].copy()
        self.save_frame(1)
        self.rejected('presentation_timestamp_repeated_or_regressed')

    def test_pts_epoch_change_rejected(self):
        self.frames[1]['pts']['epoch'] = 1
        self.save_frame(1)
        self.rejected('presentation_timestamp_repeated_or_regressed')

    def test_nonfinite_and_invalid_host_order(self):
        self.frames[1]['processingCompleteHostSeconds'] = self.frames[1]['callbackHostSeconds'] - .001
        self.save_frame(1)
        self.rejected('host_timing_or_persistence_invalid')

    def test_receipt_storage_duration_must_match_clock_stamps(self):
        self.terminal['frames'][1]['persistenceMs'] = 1
        self.save_terminal()
        self.rejected('host_timing_or_persistence_invalid')

    def test_missing_terminal_stays_incomplete(self):
        (self.root/'terminal.json').unlink()
        self.rejected('missing_or_unsafe_artifact')

    def test_stopped_or_missing_evaluation_frame_cannot_pass(self):
        self.terminal.update(status='failed', reason='storage_failed')
        self.save_terminal()
        self.rejected('capture_did_not_complete')

    def test_original_unaccounted_frame_is_not_ignored(self):
        (self.root/'frame-0021.bgra').write_bytes(bytes(8))
        self.rejected('unaccounted_or_missing_frame_artifact')

    def test_symlink_and_path_escape_rejected(self):
        (self.root/'frame-0001.bgra').unlink()
        (self.root/'frame-0001.bgra').symlink_to(self.root/'frame-0000.bgra')
        self.rejected('missing_or_unsafe_artifact')

    def test_execution_watchdog_receipt_is_required(self):
        (self.root/'execution.json').unlink()
        self.rejected('missing_or_unsafe_artifact:execution.json')

    def test_success_exit_does_not_waive_execution_time_limit(self):
        for elapsed in (45.001, -1, None, True):
            with self.subTest(elapsed=elapsed):
                write_json(self.root/'execution.json', {**self.execution, 'elapsedSeconds': elapsed})
                self.rejected('helper_execution_not_successful')

    def test_offline_audit_preserves_helper_failure(self):
        write_json(self.root/'execution.json', {'returnCode': 3, 'timedOut': False, 'retryCount': 0})
        self.rejected('helper_execution_not_successful')

    def test_session_continuity_and_configuration_required(self):
        original = self.session.copy()
        for field, value in [('isContinuityCamera', False), ('deviceType', 'external'),
                             ('requestedPreset', '1920x1080'), ('captureHostBoundsCalibrated', True),
                             ('viewConfigurationId', 'previous-view')]:
            with self.subTest(field=field):
                write_json(self.root/'session.json', {**original, field:value})
                self.rejected('session_configuration_invalid')

    def test_durable_receipt_required(self):
        self.terminal['frames'][1]['durable'] = False
        self.save_terminal()
        self.rejected('terminal_artifact_binding_invalid')

    def test_clock_domain_and_conversion_are_not_interchangeable(self):
        self.frames[1]['clockDomain'] = 'python_monotonic'
        self.save_frame(1)
        self.rejected('frame_identity_or_semantics_invalid')

    def test_orientation_change_requires_new_view(self):
        self.frames[1]['videoMirrored'] = True
        self.save_frame(1)
        self.rejected('orientation_changed')

    def test_converted_pts_must_remain_monotonic(self):
        self.frames[1]['convertedHostPtsSeconds'] = self.frames[0]['convertedHostPtsSeconds']
        self.save_frame(1)
        self.rejected('converted_pts_invalid')

    def test_capture_counter_budget_is_enforced(self):
        self.terminal['warmupFrames'] = 31
        self.save_terminal()
        self.rejected('terminal_capture_counters_invalid')

    def test_measurement_wall_budget_cannot_be_overridden_by_completed_terminal(self):
        for field in ('convertedHostPtsSeconds', 'callbackHostSeconds', 'processingCompleteHostSeconds', 'storageStartedHostSeconds'):
            self.frames[20][field] += 7
        self.terminal['frames'][20]['storageCompleteHostSeconds'] += 7
        self.save_frame(20)
        self.rejected('measurement_budget_exceeded')

    def test_plan_policy_cannot_be_relaxed(self):
        self.plan['policy']['maxPersistenceMs'] = 300
        write_json(self.root/'plan.json', self.plan)
        self.rejected('invalid_or_modified_plan')

    def test_no_reuse_output_and_no_retry_process(self):
        helper = Path(self.tmp.name)/'fake-helper'
        helper.write_text('#!/bin/sh\nexit 3\n')
        helper.chmod(0o700)
        out = Path(self.tmp.name)/'failed-run'
        result = run_probe(helper, out, 'synthetic-device', 'synthetic-view')
        self.assertFalse(result['captureProcessingPassed'])
        self.assertEqual(result['execution']['returnCode'], 3)
        self.assertEqual(result['execution']['retryCount'], 0)
        self.assertTrue((out/'plan.json').exists())
        self.assertTrue((out/'helper.stderr').exists())
        with self.assertRaises(FileExistsError):
            run_probe(helper, out, 'synthetic-device', 'synthetic-view')

    def test_launch_failure_is_preserved_without_retry(self):
        helper = Path(self.tmp.name)/'fake-helper'
        helper.write_text('#!/bin/sh\nexit 0\n')
        helper.chmod(0o700)
        out = Path(self.tmp.name)/'launch-error'
        with patch('learning.mac_camera_probe.subprocess.Popen', side_effect=OSError('fixture launch failed')) as spawn:
            result = run_probe(helper, out, 'synthetic-device', 'synthetic-view')
        spawn.assert_called_once()
        self.assertFalse(result['captureProcessingPassed'])
        self.assertIn('fixture launch failed', result['execution']['launchError'])
        self.assertTrue((out/'execution.json').exists())
        self.assertTrue((out/'evaluation.json').exists())

    def test_interrupt_cleans_camera_child_and_preserves_evidence_before_reraise(self):
        helper = Path(self.tmp.name)/'fake-helper'
        helper.write_text('#!/bin/sh\nexit 0\n')
        helper.chmod(0o700)
        process = Mock(returncode=-15)
        process.communicate.side_effect = [KeyboardInterrupt(), (b'partial', b'interrupted')]
        out = Path(self.tmp.name)/'interrupt'
        with patch('learning.mac_camera_probe.subprocess.Popen', return_value=process) as spawn:
            with self.assertRaises(KeyboardInterrupt):
                run_probe(helper, out, 'synthetic-device', 'synthetic-view')
        process.terminate.assert_called_once()
        spawn.assert_called_once()
        self.assertEqual((out/'helper.stdout').read_bytes(), b'partial')
        self.assertTrue(json.loads((out/'execution.json').read_text())['interrupted'])
        self.assertFalse(json.loads((out/'evaluation.json').read_text())['captureProcessingPassed'])

    def test_timeout_terminates_then_kills_without_retry(self):
        helper = Path(self.tmp.name)/'fake-helper'
        helper.write_text('#!/bin/sh\nexit 0\n')
        helper.chmod(0o700)
        process = Mock(returncode=-9)
        process.communicate.side_effect = [subprocess.TimeoutExpired('fixture', 45),
                                          subprocess.TimeoutExpired('fixture', 2), (b'partial', b'failure')]
        with patch('learning.mac_camera_probe.subprocess.Popen', return_value=process) as spawn:
            result = run_probe(helper, Path(self.tmp.name)/'timeout', 'synthetic-device', 'synthetic-view')
        self.assertTrue(result['execution']['timedOut'])
        self.assertFalse(result['captureProcessingPassed'])
        spawn.assert_called_once()
        process.terminate.assert_called_once()
        process.kill.assert_called_once()
        self.assertEqual((Path(self.tmp.name)/'timeout/helper.stdout').read_bytes(), b'partial')


if __name__ == '__main__':
    unittest.main()
