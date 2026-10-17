"""Same-session linkage from synthetic pixels and USB bytes; no devices."""
import base64
import copy
import hashlib
import json
from pathlib import Path
import shutil
import unittest

from learning.artifacts import StorageError, describe_artifact
from learning.serial_m4_probe import parse_reply, wire_request
from learning.usb_camera_clock_link import link_saved_session
from tests import test_mac_camera_probe as camera_fixture
from tests import test_mac_clock_bridge as clock_fixture
from tests import test_compact_m4_probe as compact_fixture


class USBClockLinkTest(unittest.TestCase):
    def setUp(self):
        self.camera = camera_fixture.CameraProbeTest()
        self.camera.setUp()
        self.addCleanup(self.camera.doCleanups)
        self.root = Path(self.camera.tmp.name) / 'linked'
        self.root.mkdir()
        shutil.copytree(self.camera.root, self.root / 'camera')
        self.camera.root = self.root / 'camera'
        (self.root / 'clock').mkdir()
        (self.root / 'serial').mkdir()
        self.before = self.save('clock/before.json', clock_fixture.sample(99, 'shared-clock'))
        self.after = self.save('clock/after.json', clock_fixture.sample(106, 'shared-clock'))
        self.nonces = [f'{i:08x}' for i in range(1, 7)]
        self.refs = []
        for i, nonce in enumerate(self.nonces):
            received = 1000 + i * 100
            raw = compact_fixture.compact_line(nonce=nonce, received=received,
                                               polled=received + 10, reported=received + 13)
            row = dict(schemaVersion='motionbrain.serial_m4_receipt.v1',
                       collectionSessionId='synthetic-session', hostBootId='test-boot',
                       hostClockId='shared-clock', clockDomain='python_monotonic',
                       requestBase64=base64.b64encode(wire_request(nonce, compact_sample=True)).decode(),
                       rawBase64=base64.b64encode(raw).decode(), rawSha256=hashlib.sha256(raw).hexdigest(),
                       sample=parse_reply(raw, nonce, compact_fixture.BOOT, compact_sample=True),
                       transportCompleted=True, requestStartedMonotonic=100 + i * .4,
                       receivedMonotonic=100 + i * .4 + .03)
            self.refs.append(self.save(f'serial/{i + 1:02d}.json', row))
        self.options = dict(session_id='synthetic-session', host_clock_id='shared-clock',
                            controller_boot_id=compact_fixture.BOOT, nonces=self.nonces,
                            camera_process_window=[99.1, 105], transport_trace=False)

    def save(self, name, value):
        (self.root / name).write_text(json.dumps(value))
        return describe_artifact(self.root, name)

    def mutate_receipt(self, index, change):
        name = self.refs[index]['artifactRef']
        row = json.loads((self.root / name).read_text())
        change(row)
        self.refs[index] = self.save(name, row)

    def link(self, **changes):
        return link_saved_session(self.root, self.before, self.after, self.refs,
                                  **{**self.options, **changes})

    def test_original_sources_are_linked_without_promoting_physical_time(self):
        result = self.link()
        self.assertEqual((len(result['frames']), len(result['sensors'])), (21, 6))
        self.assertEqual(result['sources']['receipts'], self.refs)
        self.assertEqual(result['hostBootId'], 'test-boot')
        self.assertTrue(result['presentationClockLinked'])
        self.assertTrue(result['postHocAssociation'])
        for field in ('opticalExposureVerified', 'sensorPhysicalSampleTimeVerified',
                      'physicalSampleAlignmentVerified', 'trainingEligible', 'controlEligible',
                      'semanticAnchorVerified', 'mcuClockRateCalibrated'):
            self.assertIs(result[field], False)
        self.assertEqual(result['physicalLearningEligibleRows'], 0)
        self.assertEqual(result['mcuIntervalMarginMs'], 2)
        for sensor in result['sensors']:
            self.assertEqual(sensor['sample']['timestampSemantics'],
                             'firmware_update_marker_before_adc_not_physical_conversion')
            self.assertFalse(sensor['sample']['sensorPhysicalSampleTimeVerified'])
        for frame in result['frames']:
            bounds = frame['presentationHostBounds']
            distances = [max(abs(bounds['latest'] - s['sample']['sampledHostBounds']['earliest']),
                             abs(s['sample']['sampledHostBounds']['latest'] - bounds['earliest'])) * 1000
                         for s in result['sensors']]
            nearest = min(range(6), key=distances.__getitem__)
            self.assertEqual(frame['nearestSensorNonce'], self.nonces[nearest])
            self.assertEqual(frame['pollMarkerSeparationUpperDiagnosticMs'], distances[nearest])
            self.assertEqual(frame['within20ms'], distances[nearest] <= 20)

    def test_same_bytes_cannot_be_claimed_as_another_session_or_host(self):
        for changes in ({'session_id': 'other-session'}, {'host_clock_id': 'other-clock'},
                        {'controller_boot_id': 'b' * 32}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.link(**changes)

    def test_raw_receipt_identity_is_required_before_linking(self):
        original = copy.deepcopy(self.refs)
        for key in ('collectionSessionId', 'hostBootId', 'hostClockId'):
            old = (self.root / 'serial/01.json').read_text()
            self.mutate_receipt(0, lambda row: row.pop(key))
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'receipt_identity'):
                self.link()
            (self.root / 'serial/01.json').write_text(old)
            self.refs = copy.deepcopy(original)

    def test_camera_session_must_be_the_collection_session(self):
        self.options['session_id'] = 'renamed'
        for i in range(6):
            self.mutate_receipt(i, lambda row: row.update(collectionSessionId='renamed'))
        with self.assertRaisesRegex(ValueError, 'camera_capture'):
            self.link()

    def test_bridge_boot_change_and_expiry_are_rejected(self):
        after = clock_fixture.sample(106, 'shared-clock')
        after['hostBootId'] = 'other-boot'
        self.after = self.save('clock/after.json', after)
        with self.assertRaisesRegex(ValueError, 'identity_changed'):
            self.link()
        self.after = self.save('clock/after.json', clock_fixture.sample(160, 'shared-clock'))
        with self.assertRaisesRegex(ValueError, 'bridge_interval'):
            self.link()

    def test_entire_process_window_must_be_inside_clock_brackets(self):
        for window in ([98, 105], [99.1, 107], [105, 99.1], [float('nan'), 105], [99.1, 10**400]):
            with self.subTest(window=window), self.assertRaises(ValueError):
                self.link(camera_process_window=window)

    def test_pts_outside_actual_camera_process_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'pts_outside_process'):
            self.link(camera_process_window=[99.1, 103.9])

    def test_persistence_after_process_end_cannot_hide_behind_valid_pts(self):
        with self.assertRaisesRegex(ValueError, 'camera_observation_outside_process'):
            self.link(camera_process_window=[99.1, 104.005])

    def test_receipts_outside_process_or_in_wrong_order_are_rejected(self):
        self.mutate_receipt(0, lambda row: row.update(requestStartedMonotonic=98, receivedMonotonic=98.03))
        with self.assertRaisesRegex(ValueError, 'receipt_outside_process'):
            self.link()

    def test_reordered_raw_receipts_are_rejected_even_when_rebound(self):
        self.mutate_receipt(1, lambda row: row.update(requestStartedMonotonic=99.8, receivedMonotonic=99.83))
        with self.assertRaisesRegex(ValueError, 'receipt_outside_process_or_order'):
            self.link()

    def test_nonce_uniqueness_and_source_count_are_required(self):
        with self.assertRaisesRegex(ValueError, 'nonces_invalid'):
            self.link(nonces=['00000001'] * 6)
        self.refs.pop()
        with self.assertRaisesRegex(ValueError, 'receipt_refs_invalid'):
            self.link()

    def test_duplicate_or_relocated_receipt_reference_is_rejected(self):
        self.refs[1] = self.refs[0]
        with self.assertRaisesRegex(ValueError, 'source_location'):
            self.link()

    def test_changed_original_receipt_bytes_fail_hash_verification(self):
        (self.root / 'serial/01.json').write_text('{}')
        with self.assertRaises(StorageError):
            self.link()

    def test_modified_decoded_sample_does_not_override_original_wire_bytes(self):
        self.mutate_receipt(0, lambda row: row['sample'].update(reportedAgeMs=0))
        with self.assertRaisesRegex(ValueError, 'recomputation_mismatch'):
            self.link()

    def test_repeated_sensor_marker_is_rejected(self):
        first = json.loads((self.root / 'serial/01.json').read_text())['sample']
        def change(row):
            nonce = row['sample']['nonce']
            raw = compact_fixture.compact_line(nonce, received=first['requestReceivedMs'],
                                               polled=first['polledMs'], reported=first['reportedMs'])
            row.update(rawBase64=base64.b64encode(raw).decode(), rawSha256=hashlib.sha256(raw).hexdigest(),
                       sample=parse_reply(raw, nonce, compact_fixture.BOOT, compact_sample=True))
        self.mutate_receipt(1, change)
        with self.assertRaisesRegex(ValueError, 'marker_not_new'):
            self.link()

    def test_corrupted_pixels_or_partial_capture_cannot_link(self):
        (self.root / 'camera/frame-0000.bgra').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'camera_capture_invalid'):
            self.link()

    def test_trace_choice_cannot_promote_untraced_receipts(self):
        with self.assertRaisesRegex(ValueError, 'trace_presence_mismatch'):
            self.link(transport_trace=True)


if __name__ == '__main__':
    unittest.main()
