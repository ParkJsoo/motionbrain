"""Virtual-time shadow consumption, durable barriers and stop latching. No devices."""
import hashlib
import json
import threading
import unittest
from unittest.mock import patch

from learning.artifacts import ArtifactStore, read_events, verify_artifacts
from learning.dry_run import VirtualClock
from learning.fast_observation import AnchoredRoiMeasurement
from learning.observation_consumer import ShadowObservationConsumer, ConsumptionStopped
from test_fresh_source_age import fresh_receipt
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers
import test_fast_observation as fixtures


class ConsumerTest(unittest.TestCase):
    def setUp(self):
        self.clock = VirtualClock(); self.clock.advance(.09)
        self.body = fixtures.jpeg(); self.receipt = fresh_receipt()
        self.receipt['sha256'] = hashlib.sha256(self.body).hexdigest()
        self.source = {**parse_capture_headers(self.receipt['headers']), 'imageSha256': self.receipt['sha256']}
        self.measurement = {'sourceFrameId': self.source['sourceFrameId'], 'imageSha256': self.source['imageSha256'],
            'hostClockId': 'host-A', 'measurementCompletedMonotonic': 100.08, 'accepted': True,
            'anchorAgeMs': 500, 'targetBox': {'x': 1, 'y': 2, 'width': 20, 'height': 30}}
        self.consumer = ShadowObservationConsumer(self.clock, 'host-A')

    def consume(self):
        return self.consumer.consume(self.measurement, self.source, self.receipt, self.body)

    def finish(self):
        self.consumer.finish_persistence(self.source['sourceFrameId'], 'host-A')

    def test_pending_blocks_new_use_and_completion_is_exactly_once(self):
        row = self.consume()
        self.assertTrue(row['consumed'])
        self.assertEqual(row['storageStatus'], 'pending')
        self.assertFalse(row['controlEligible'])
        with self.assertRaises(ConsumptionStopped): self.consumer.check_ready()
        row['shadowDecision']['targetBox']['x'] = 99
        self.assertEqual(self.consumer.snapshot()[0]['shadowDecision']['targetBox']['x'], 1)
        self.finish(); self.consumer.check_ready()
        with self.assertRaises(ConsumptionStopped): self.finish()
        with self.assertRaises(ConsumptionStopped): self.consume()
        self.assertTrue(self.consumer.halted)

    def test_late_consumption_never_uses_previously_accepted_measurement(self):
        self.clock.advance(.05)
        row = self.consume()
        self.assertFalse(row['consumed'])
        self.assertEqual(row['reason'], 'source_age_at_consumption_exceeds_policy')
        self.assertNotIn('shadowDecision', row)
        self.finish(); self.consumer.check_ready()

    def test_legacy_clock_margin_crosses_100ms_without_changing_measurement(self):
        self.receipt['query'] = {}
        self.clock._seconds = 100.10199
        row = self.consume()
        self.assertFalse(row['consumed'])
        self.assertGreater(row['sourceAgeAtConsumptionMs'], 100)
        self.assertAlmostEqual(row['legacyReportedAgeUpperMs'], 10/.999+.002)
        self.assertEqual(row['reason'], 'source_age_at_consumption_exceeds_policy')
        self.assertTrue(self.measurement['accepted'])

    def test_rejected_measurement_and_old_anchor_cannot_be_promoted(self):
        self.measurement['accepted'] = False
        self.assertEqual(self.consume()['reason'], 'measurement_rejected')
        self.setUp(); self.measurement['anchorAgeMs'] = 14999
        self.assertEqual(self.consume()['reason'], 'anchor_expired_at_consumption')

    def test_anchor_age_includes_capture_to_consume_and_slow_clock_margin(self):
        for delta_ms, expected in ((14900, True), (14940, False), (14990, False)):
            self.setUp()
            r = self.receipt
            r['headers'].update({'x-motionbrain-captured-us': '20000000',
                'x-motionbrain-frame-id': 'a'*32+':20000000',
                'x-motionbrain-capture-requested-us': '19980000'})
            self.source = {**parse_capture_headers(r['headers']), 'imageSha256': r['sha256']}
            a = fixtures.anchor(self.body)
            a.update(fixtures.metadata(self.body, captured=20000000-delta_ms*1000))
            a['detectionSourceFrameId'] = a['sourceFrameId']
            tracker = AnchoredRoiMeasurement(a, self.body, clock=self.clock)
            self.measurement = tracker.measure(self.body, self.source, request_started=100,
                received=100.08, host_clock_id='host-A', capture_receipt=r)
            self.assertTrue(self.measurement['accepted'])
            row = self.consume()
            self.assertEqual(row['consumed'], expected, row)
            self.assertAlmostEqual(row['anchorAgeAtConsumptionUpperMs'],
                                   delta_ms/.999+row['sourceAgeAtConsumptionMs']+.002)
            if not expected:
                self.assertEqual(row['reason'], 'anchor_expired_at_consumption')
                self.assertNotIn('shadowDecision', row)
            if delta_ms == 14940:
                self.assertLess(delta_ms+row['sourceAgeAtConsumptionMs'], 15000)
                self.assertGreater(row['anchorAgeAtConsumptionUpperMs'], 15000)

    def test_rebuilds_bound_and_includes_evidence_validation_delay(self):
        # Forged precomputed lower bound is ignored, and time spent validating counts.
        self.measurement['sourceCaptureLowerBoundMonotonic'] = 200
        from learning.observation_consumer import source_age_basis
        def slow(*args, **kwargs):
            value = source_age_basis(*args, **kwargs); self.clock.advance(.05); return value
        with patch('learning.observation_consumer.source_age_basis', side_effect=slow): row = self.consume()
        self.assertFalse(row['consumed'])
        self.assertGreater(row['sourceAgeAtConsumptionMs'], 100)

    def test_mismatched_body_frame_host_and_receipt_stop_after_preservation(self):
        for field, value in [('imageSha256', 'd'*64), ('sourceFrameId', 'other'), ('hostClockId', 'other')]:
            self.setUp(); self.measurement[field] = value
            self.assertFalse(self.consume()['consumed'])
            with self.assertRaises(ConsumptionStopped): self.finish()
        self.setUp(); self.body += b'corrupt'
        self.assertFalse(self.consume()['consumed'])
        self.setUp(); self.receipt['query'] = {'fresh': '0'}
        self.assertFalse(self.consume()['consumed'])

    def test_nonfinite_regressed_clocks_and_stale_finish_identity_stop(self):
        for now in (100.07, float('nan'), float('inf')):
            self.setUp(); self.clock._seconds = now
            self.assertFalse(self.consume()['consumed'])
            with self.assertRaises(ConsumptionStopped): self.finish()
        for frame, host in [('other', 'host-A'), (self.source['sourceFrameId'], 'other')]:
            self.setUp(); self.consume()
            with self.assertRaises(ConsumptionStopped): self.consumer.finish_persistence(frame, host)
            self.assertEqual(self.consumer.snapshot()[0]['storageStatus'], 'unconfirmed')

    def test_persistence_exact_deadline_and_overrun(self):
        self.consume(); self.clock.advance(.250); self.finish()
        self.assertEqual(self.consumer.snapshot()[0]['storageStatus'], 'completed')
        self.setUp(); self.consume(); self.clock.advance(.251)
        with self.assertRaises(ConsumptionStopped): self.finish()
        self.assertEqual(self.consumer.snapshot()[0]['storageStatus'], 'late')
        with self.assertRaises(ConsumptionStopped): self.consumer.check_ready()

    def test_invalid_budget_is_rejected(self):
        for budget in (True, 0, -1, 30001, float('nan'), float('inf')):
            with self.assertRaises(ValueError): ShadowObservationConsumer(self.clock, 'host-A', max_persistence_ms=budget)

    def test_storage_clock_regression_and_nonfinite_time_do_not_confirm_completion(self):
        for now in (100.08, float('nan'), float('inf')):
            self.setUp(); self.consume(); self.clock._seconds = now
            with self.assertRaises(ConsumptionStopped): self.finish()
            self.assertEqual(self.consumer.snapshot()[0]['storageStatus'], 'unconfirmed')
            with self.assertRaises(ConsumptionStopped): self.consumer.check_ready()


class ConsumerProbeTest(unittest.TestCase):
    setUp = fixtures.FastProbeTest.setUp
    run_probe = fixtures.FastProbeTest.run_probe

    def assert_stopped(self, report, state='failed'):
        self.assertEqual(report['status'], state, report)
        self.assertEqual(self.capture_count, 1)
        self.assertEqual(report['motorCommandsSent'], 0)
        self.assertTrue(report['consumptionHalted'])
        self.assertFalse(any(t.name.startswith('capture-image') for t in threading.enumerate()))

    def test_consume_before_storage_then_complete_even_if_image_is_stale_at_storage(self):
        image = ArtifactStore.save_image
        def delayed(store, index, phase, body, digest):
            if index != 'anchor': self.transport.clock.advance(.120)
            return image(store, index, phase, body, digest)
        with patch.object(ArtifactStore, 'save_image', delayed): report = self.run_probe(shadow_consumption=True)
        self.assertEqual(report['status'], 'completed', report)
        self.assertEqual(report['consumedObservations'], 2)
        self.assertTrue(all(r['storageStatus']=='completed' for r in report['consumptions']))
        for i in range(2):
            saved = json.loads((self.root/f'consumption/{i:04d}.json').read_text())
            self.assertEqual(saved['storageStatus'], 'pending')
            self.assertLessEqual(saved['sourceAgeAtConsumptionMs'], 100)
            persisted = json.loads((self.root/f'persistence/{i:04d}.json').read_text())
            self.assertFalse(persisted['withinAgePolicyAtPersistence'])
        self.assertEqual(verify_artifacts(self.root, report['artifacts']), [])

    def test_scheduler_delay_after_measurement_rejected_before_storage(self):
        measure = AnchoredRoiMeasurement.measure
        def delayed(*args, **kwargs):
            result = measure(*args, **kwargs); self.transport.clock.advance(.100); return result
        with patch.object(AnchoredRoiMeasurement, 'measure', delayed): report = self.run_probe(shadow_consumption=True)
        self.assertEqual(report['acceptedMeasurements'], 2)
        self.assertEqual(report['consumedObservations'], 0)
        self.assertEqual(report['status'], 'completed')
        self.assertTrue(all(r['reason']=='source_age_at_consumption_exceeds_policy' for r in report['consumptions']))

    def test_fresh_clock_error_crosses_100ms_at_consumption_without_commands(self):
        from test_fresh_source_age import FreshSourceAgeProbeTest
        FreshSourceAgeProbeTest.configure(self)
        measure = AnchoredRoiMeasurement.measure
        def delayed(*args, **kwargs):
            result = measure(*args, **kwargs); self.transport.clock.advance(.09399); return result
        with patch.object(AnchoredRoiMeasurement, 'measure', delayed):
            report = self.run_probe(shadow_consumption=True, fresh_capture=True)
        self.assertEqual(report['acceptedMeasurements'], 2)
        self.assertEqual(report['consumedObservations'], 0)
        self.assertEqual(report['motorCommandsSent'], 0)
        self.assertTrue(all(r['sourceAgeAtConsumptionMs'] > 100 for r in report['consumptions']))

    def test_legacy_clock_error_crosses_100ms_with_zero_commands(self):
        measure = AnchoredRoiMeasurement.measure
        def delayed(*args, **kwargs):
            result = measure(*args, **kwargs); self.transport.clock.advance(.078999); return result
        with patch.object(AnchoredRoiMeasurement, 'measure', delayed): report = self.run_probe(shadow_consumption=True)
        self.assertEqual(report['acceptedMeasurements'], 2)
        self.assertEqual(report['consumedObservations'], 0)
        self.assertEqual(report['motorCommandsSent'], 0)
        self.assertTrue(all(r['sourceAgeAtConsumptionMs'] > 100 for r in report['consumptions']))

    def test_capture_to_consumption_anchor_expiry_rejects_with_zero_commands(self):
        original = self.transport.fault
        def old_anchor(device, path, receipt, body):
            body = original(device, path, receipt, body)
            if path == '/capture':
                captured = 15990000 + (self.capture_count-1)*100000
                receipt['headers'].update({'x-motionbrain-captured-us': str(captured),
                                          'x-motionbrain-frame-id': 'a'*32+':'+str(captured)})
            return body
        self.transport.fault = old_anchor
        report = self.run_probe(shadow_consumption=True)
        self.assertEqual(report['acceptedMeasurements'], 1)
        self.assertEqual(report['consumedObservations'], 0)
        self.assertEqual(report['consumptions'][0]['reason'], 'anchor_expired_at_consumption')
        self.assertEqual(report['motorCommandsSent'], 0)

    def test_image_overrun_is_preserved_and_no_next_get(self):
        image = ArtifactStore.save_image
        def delayed(store, index, phase, body, digest):
            if index != 'anchor': self.transport.clock.advance(.251)
            return image(store, index, phase, body, digest)
        with patch.object(ArtifactStore, 'save_image', delayed): report = self.run_probe(shadow_consumption=True)
        self.assert_stopped(report)
        self.assertEqual(report['consumedObservations'], 1)
        self.assertEqual(report['consumptions'][0]['storageStatus'], 'late')
        self.assertEqual((self.root/'images/0/source.jpg').read_bytes(), self.body)

    def test_sidecar_and_measured_event_delay_are_included_in_barrier(self):
        write, append = ArtifactStore.write_json, ArtifactStore.append
        for stage in ('sidecar', 'event'):
            self.setUp()
            def delayed_json(store, path, value):
                if stage=='sidecar' and path.startswith('consumption/'): self.transport.clock.advance(.251)
                return write(store, path, value)
            def delayed_event(store, kind, transition, payload):
                if stage=='event' and kind=='measured': self.transport.clock.advance(.251)
                return append(store, kind, transition, payload)
            with patch.object(ArtifactStore, 'write_json', delayed_json), patch.object(ArtifactStore, 'append', delayed_event):
                report = self.run_probe(shadow_consumption=True)
            self.assert_stopped(report)
            self.assertEqual(report['consumptions'][0]['storageStatus'], 'late')

    def test_receipt_image_measurement_sidecar_event_failures_stop_without_retry(self):
        write, image, append = ArtifactStore.write_json, ArtifactStore.save_image, ArtifactStore.append
        for stage in ('receipt', 'image', 'measurement', 'sidecar', 'event'):
            self.setUp()
            def fail_json(store, path, value):
                if ((stage=='receipt' and value.get('path')=='/capture')
                        or (stage=='measurement' and path.startswith('measurements/'))
                        or (stage=='sidecar' and path.startswith('consumption/'))): raise OSError('fixture disk failure')
                return write(store, path, value)
            def fail_image(store, index, phase, body, digest):
                if stage=='image' and index!='anchor': raise OSError('fixture disk failure')
                return image(store, index, phase, body, digest)
            def fail_event(store, kind, transition, payload):
                if stage=='event' and kind=='measured': raise OSError('fixture disk failure')
                return append(store, kind, transition, payload)
            with patch.object(ArtifactStore, 'write_json', fail_json), patch.object(ArtifactStore, 'save_image', fail_image), patch.object(ArtifactStore, 'append', fail_event):
                report = self.run_probe(shadow_consumption=True)
            self.assert_stopped(report)
            self.assertEqual(report['consumedObservations'], 1)
            self.assertEqual(report['consumptions'][0]['storageStatus'], 'unconfirmed')
            self.assertEqual(read_events(self.root)[0][-1]['kind'], 'finished')

    def test_interrupt_keeps_consumption_and_joins_current_write(self):
        write = ArtifactStore.write_json
        def interrupt(store, path, value):
            if value.get('path') == '/capture': raise KeyboardInterrupt()
            return write(store, path, value)
        with patch.object(ArtifactStore, 'write_json', interrupt): report = self.run_probe(shadow_consumption=True)
        self.assert_stopped(report, 'interrupted')
        self.assertEqual(report['consumedObservations'], 1)
        self.assertEqual(report['consumptions'][0]['storageStatus'], 'unconfirmed')
        self.assertEqual((self.root/'images/0/source.jpg').read_bytes(), self.body)

    def test_permanent_disk_failure_cannot_claim_a_durable_terminal(self):
        write = ArtifactStore.write_json
        failed = False
        def fail(store, path, value):
            nonlocal failed
            if value.get('path')=='/capture': failed = True
            if failed: raise OSError('fixture permanent disk failure')
            return write(store, path, value)
        with patch.object(ArtifactStore, 'write_json', fail), self.assertRaises(OSError):
            self.run_probe(shadow_consumption=True)
        self.assertEqual(self.capture_count, 1)
        self.assertNotEqual(read_events(self.root)[0][-1]['kind'], 'finished')
        self.assertFalse((self.root/'run.json').exists())

    def test_default_is_unchanged_and_invalid_opt_in_rejected(self):
        self.assertNotIn('consumptions', self.run_probe())
        self.setUp()
        with self.assertRaises(ValueError): self.run_probe(shadow_consumption='yes')
        self.assertEqual(self.capture_count, 0)
