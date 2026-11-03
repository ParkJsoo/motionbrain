import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from learning import finite_profile_measurement as m
from learning import supervised_measurement as shared
try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None


@unittest.skipIf(cv2 is None, 'OpenCV required')
class FiniteMeasurementTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)/'native-camera'
        self.root.mkdir()
        points = []
        for name, roi in [('p1', [0, 0, 15, 18]), ('p2', [16, 0, 16, 18]),
                          ('c1', [0, 20, 15, 20]), ('c2', [16, 20, 16, 20])]:
            points.append(dict(id=name, roi=roi, hsvLower=[0, 200, 200], hsvUpper=[5, 255, 255], minAreaPx=5, maxAreaPx=20))
        self.config = dict(schemaVersion=shared.SCHEMA, width=64, height=48, stabilityThreshold=.5,
                           minLinkLengthPx=5, points=points,
                           representation=dict(kind='relative_angle', parent=['p1', 'p2'], child=['c1', 'c2']))
        from learning.mac_clock_bridge import SCHEMA, MARGIN_SECONDS
        snapshot = dict(schemaVersion=SCHEMA, hostClockId='session', hostBootId='boot', sourceClock='coremedia_host_clock',
                        targetClock='python_monotonic', sharedTimebase='mach_absolute_time', marginSeconds=MARGIN_SECONDS,
                        opticalExposureVerified=False, trainingEligible=False, controlEligible=False,
                        pythonClock=dict(implementation='mach_absolute_time()', monotonic=True, adjustable=False, resolution=1e-9),
                        pairs=[dict(before=10+i*.01, after=10+i*.01, value=1000+i, timescale=100, flags=1, epoch=0) for i in range(8)])
        (self.root/'host-ingress').mkdir()
        (self.root/'host-ingress/clock-before.json').write_text(json.dumps(snapshot))
        frames = []
        for i, t in enumerate([19.3, 19.5, 19.7, 19.85, 23.3, 23.5, 23.7, 23.9]):
            image = np.zeros((48, 64, 4), np.uint8)
            image[:, :, 3] = 255
            for x, y in [(5, 7), (22, 7), (5, 27), (22, 27 if t < 20 else 30)]:
                image[y-1:y+2, x-1:x+2, :3] = [0, 0, 255]
            raw_image = image.tobytes()
            image_name, meta_name = f'frame-{i}.bgra', f'frame-{i}.json'
            (self.root/image_name).write_bytes(raw_image)
            meta = dict(schemaVersion='motionbrain.mac_camera_frame.v2', storageEncoding='apple-lzbitmap-bgra8-v1',
                        frameIndex=i, imageFile=image_name, width=64, height=48, rowBytes=256, pixelFormat='BGRA8',
                        rawByteLength=len(raw_image), imageSha256=shared.sha(raw_image), pixelSha256=shared.sha(raw_image),
                        storedByteLength=len(raw_image), convertedHostPtsSeconds=t, clockDomain='coremedia_host_clock',
                        timestampSemantics='cmsamplebuffer_presentation_timestamp', actualConfigurationJson='{}',
                        configurationSha256=shared.sha(b'{}'), sessionId='session', viewConfigurationId='view',
                        deviceUniqueId='camera', videoMirrored=False, videoRotationAngle=0)
            raw = json.dumps(meta).encode()
            (self.root/meta_name).write_bytes(raw)
            frames.append(dict(frameIndex=i, imageFile=image_name, imageSha256=shared.sha(raw_image), metadataFile=meta_name,
                               metadataSha256=shared.sha(raw), sourceEarliest=t-1e-6, sourceLatest=t+1e-6,
                               stdoutReadAt=t+.01, clockOffsetBoundsSeconds=[-1e-6, 1e-6]))
        self.report = dict(schemaVersion=m.SCHEMA, actionContract='finite_profile.v1', captureGroup='finite-new-group',
                           split='development', expectedBootId='device-boot', expectedEpoch=1, trial=dict(id='finite-one'), camera=dict(sessionId='session', viewId='view', deviceId='camera',
                           hostClockId='session', width=64, height=48), frames=frames)
        points = [dict(offsetMs=i*50, pwm=pwm) for i, pwm in enumerate([10, 20, 30, 40, 30, 20, 10, 0])]
        entries = [dict(sequence=i+1, atMs=1000+i*50, motor=2, pwm=p['pwm'], reason='driver_write') for i,p in enumerate(points)]
        entries += [dict(sequence=9+i, atMs=1350+(i==4), motor=i+1, pwm=0, reason='forced_off') for i in range(5)]
        profile = dict(motor=2, direction=1, points=points, maxLatenessMs=20, startLeadMs=300)
        self.report['requestedProfile'] = dict(profile)
        record = dict(bootId='device-boot', commandId='finite-one', requestEpoch=1, state='Completed', reason='NominalEnd',
                      executed=True, anyNonzeroWrite=True, softwareOffConfirmed=True, offWriteRecorded=True,
                      offConfirmationRecorded=True, timingConformant=True, traceComplete=True, quietHeld=False,
                      offFailureLatched=False, nextEventIndex=8, requestedProfile=profile, executeAcceptedAt=700,
                      plannedStartAt=1000, plannedOffAt=1350, firstNonzeroWriteAt=1000, offWriteAt=1350,
                      offConfirmedAt=1351, plannedEventReceipts=[dict(index=i,present=True,atMs=1000+i*50,pwm=p['pwm']) for i,p in enumerate(points)])
        output = dict(bootId='device-boot', commandId='finite-one', requestEpoch=1, evidence='driver_write',
                      clockDomain='controller_boot_millis', closed=True, captureComplete=True, dropped=0,
                      lease=1, ordinal=1, openedAt=650, closedAt=5000, totalWrites=13, entries=entries)
        self.decode = patch('learning.lossless_frame.decode', side_effect=lambda stored, size, digest: stored)
        self.decode.start()
        self.addCleanup(self.decode.stop)
        receipt = m.preflight(self.report, self.config, self.root, 20.)
        self.assertEqual(receipt['status'], 'eligible', receipt)
        self.report['trial'] = dict(id='finite-one', beforeReceipt=receipt, dispatchBefore=20.1, dispatchAfter=20.2,
                                    hostOffReceiptAt=21., quietVerifiedUntil=24.1, postEnd=24.1, releaseStartedAt=24.2,
                                    closedEvidence=dict(record=record, actualOutput=output))
        (self.root.parent/'before-receipt.json').write_text(json.dumps(receipt))
        self.save_evidence()

    def save_evidence(self):
        (self.root.parent/'closed-actual-evidence.json').write_text(json.dumps(self.report['trial']['closedEvidence']))

    def measure(self, sync_evidence=True):
        if sync_evidence:
            self.save_evidence()
        return m.measure(self.report, self.config, self.root)

    def test_valid_separate_contract_and_waveform(self):
        result = self.measure()
        self.assertEqual(result['status'], 'eligible', result)
        self.assertAlmostEqual(result['delta'], 10.007979801441337)
        self.assertEqual(result['actualOutputSummary']['activeMs'], 350)
        self.assertEqual(result['actualOutputSummary']['signedPwmIntegralCountMs'], 8000)
        self.assertNotEqual(result['schemaVersion'], shared.DATA_SCHEMA)
        self.assertFalse(result['physicalJointAngleCalibrated'])
        self.assertFalse(result['predictionPerformed'])

    def test_before_gate_rejects_missing_marker(self):
        frame = self.report['frames'][0]
        (self.root/frame['imageFile']).write_bytes(b'corrupt')
        receipt = m.preflight(self.report, self.config, self.root, 20.)
        self.assertEqual(receipt['status'], 'rejected')
        self.assertIn('hash_mismatch', receipt['reason'])

    def test_before_gate_requires_already_received_frames(self):
        self.report['frames'][0]['stdoutReadAt'] = 20.1
        self.assertEqual(m.preflight(self.report, self.config, self.root, 20.)['reason'], 'before_frame_not_available')

    def test_receipt_cannot_be_modified(self):
        self.report['trial']['beforeReceipt']['before']['value'] = 99.
        self.assertEqual(self.measure()['reason'], 'before_receipt_invalid')

    def test_stale_before_rejects(self):
        self.report['trial']['dispatchBefore'] = 22.01
        self.report['trial']['dispatchAfter'] = 22.02
        self.report['trial']['hostOffReceiptAt'] = 23.
        self.assertEqual(self.measure()['reason'], 'finite_host_timing_or_stale_before')

    def test_quiet_release_before_window_end_rejects(self):
        self.report['trial']['releaseStartedAt'] = 23.99
        self.assertEqual(self.measure()['reason'], 'finite_quiet_window_incomplete')

    def test_missing_post_frames_not_success(self):
        self.report['frames'] = self.report['frames'][:-1]
        self.assertEqual(self.measure()['reason'], 'measurement_window_missing_or_gapped')

    def test_cancelled_not_normal_training_data(self):
        self.report['trial']['closedEvidence']['record']['state'] = 'Cancelled'
        self.assertEqual(self.measure()['reason'], 'finite_execution_not_complete')

    def test_wrong_boot_or_unclosed_capture_rejected(self):
        output = self.report['trial']['closedEvidence']['actualOutput']
        for key, value in [('bootId','other'), ('closed',False), ('captureComplete',False), ('dropped',1)]:
            old = output[key]
            output[key] = value
            self.assertEqual(self.measure()['reason'], 'finite_execution_identity' if key == 'bootId' else 'finite_output_identity_or_completeness')
            output[key] = old

    def test_extra_nonzero_during_quiet_rejects(self):
        self.report['trial']['closedEvidence']['actualOutput']['entries'][-1]['pwm'] = 10
        self.assertEqual(self.measure()['reason'], 'finite_quiet_output')

    def test_missing_sequence_or_mismatched_receipt_rejects(self):
        evidence = self.report['trial']['closedEvidence']
        evidence['actualOutput']['entries'][2]['sequence'] = 4
        self.assertEqual(self.measure()['reason'], 'finite_output_sequence')
        evidence['actualOutput']['entries'][2]['sequence'] = 3
        evidence['record']['plannedEventReceipts'][2]['atMs'] += 1
        self.assertEqual(self.measure()['reason'], 'finite_event_receipt')

    def test_device_clock_never_used_as_host_anchor(self):
        evidence = self.report['trial']['closedEvidence']
        for key in ('executeAcceptedAt','plannedStartAt','plannedOffAt','firstNonzeroWriteAt','offWriteAt','offConfirmedAt'):
            evidence['record'][key] = (evidence['record'][key] + 2**32 - 1200) % 2**32
        for item in evidence['record']['plannedEventReceipts'] + evidence['actualOutput']['entries']:
            item['atMs'] = (item['atMs'] + 2**32 - 1200) % 2**32
        for key in ('openedAt','closedAt'):
            evidence['actualOutput'][key] = (evidence['actualOutput'][key] + 2**32 - 1200) % 2**32
        result = self.measure()
        self.assertEqual(result['status'], 'eligible', result)
        self.assertEqual(result['before']['window'], [19.25,19.9])
        self.assertEqual(result['after']['window'], [23.25,24.])

    def test_training_group_or_legacy_contract_not_accepted(self):
        self.report['split'] = 'test'
        self.assertEqual(self.measure()['reason'], 'finite_capture_contract')
        self.report['split'] = 'development'
        self.report['actionContract'] = 'legacy-run'
        self.assertEqual(self.measure()['reason'], 'finite_capture_contract')

    def test_after_config_change_rejects(self):
        frame = self.report['frames'][-1]
        path = self.root/frame['metadataFile']
        meta = json.loads(path.read_text())
        meta['videoMirrored'] = True
        raw = json.dumps(meta).encode()
        path.write_bytes(raw)
        frame['metadataSha256'] = shared.sha(raw)
        self.assertEqual(self.measure()['reason'], 'measurement_configuration_changed')

    def test_requested_profile_is_frozen_before_execution(self):
        self.report['trial']['closedEvidence']['record']['requestedProfile'] = dict(self.report['requestedProfile'], direction=-1)
        self.assertEqual(self.measure()['reason'], 'requested_profile_changed')
        self.report['requestedProfile']['direction'] = -1
        self.assertEqual(self.measure()['reason'], 'before_receipt_invalid')

    def test_later_arriving_before_pts_does_not_reselect_before_window(self):
        extra = dict(self.report['frames'][1], frameIndex=99, sourceEarliest=19.6-1e-6,
                     sourceLatest=19.6+1e-6, stdoutReadAt=20.5)
        self.report['frames'].insert(2, extra)
        result = self.measure()
        self.assertEqual(result['status'], 'eligible', result)
        self.assertEqual(result['before']['frameIds'], [0,1,2,3])

    def test_before_gate_rejects_unsettled_values(self):
        original = shared.frame_value
        def unstable(capture, frame, config, clock, hashes, camera):
            value, identity = original(capture, frame, config, clock, hashes, camera)
            return value + (.6 if frame['frameIndex'] == 0 else 0), identity
        with patch('learning.supervised_measurement.frame_value', side_effect=unstable):
            self.assertEqual(m.preflight(self.report, self.config, self.root, 20.)['reason'], 'measurement_window_unsettled')

    def test_missing_or_altered_saved_evidence_rejects(self):
        path = self.root.parent/'closed-actual-evidence.json'
        path.unlink()
        result = self.measure(sync_evidence=False)
        self.assertEqual(result['status'], 'rejected')
        self.assertIn('closed-actual-evidence.json', result['reason'])
        path.write_text('{}')
        self.assertEqual(self.measure(sync_evidence=False)['reason'],
                         'finite_source_receipt_mismatch:closed-actual-evidence.json')

    def test_saved_before_receipt_is_required_and_bound(self):
        path = self.root.parent/'before-receipt.json'
        path.write_text('{}')
        self.assertEqual(self.measure()['reason'], 'finite_source_receipt_mismatch:before-receipt.json')
        path.unlink()
        self.assertEqual(self.measure()['status'], 'rejected')

    def test_changing_both_output_boot_ids_does_not_change_frozen_identity(self):
        evidence = self.report['trial']['closedEvidence']
        evidence['record']['bootId'] = 'other-boot'
        evidence['actualOutput']['bootId'] = 'other-boot'
        self.assertEqual(self.measure()['reason'], 'finite_execution_identity')

    def test_fixed_receipt_identity_and_original_file_hashes(self):
        result = self.measure()
        self.assertEqual(result['status'], 'eligible', result)
        self.assertEqual(self.report['trial']['beforeReceipt']['executionIdentity'],
                         dict(bootId='device-boot', commandId='finite-one', requestEpoch=1))
        for name in ('before-receipt.json', 'closed-actual-evidence.json'):
            self.assertEqual(result['sourceHashes']['../'+name], shared.sha((self.root.parent/name).read_bytes()))
        self.report['expectedEpoch'] = 2
        self.assertEqual(self.measure()['reason'], 'before_receipt_invalid')

if __name__ == '__main__':
    unittest.main()
