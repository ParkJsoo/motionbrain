"""Fresh causal bound with virtual clocks; no physical actuation."""
import unittest
from unittest.mock import Mock, patch

from learning.source_age import source_age_upper_ms
from learning.fast_observation import AnchoredRoiMeasurement
from learning.capture_timing import capture_timing
from learning.artifacts import ArtifactStore, read_events
from test_source_age import timed_receipt, basis
from test_capture_timing import status
from test_fast_observation import jpeg, anchor, metadata
import test_fast_observation as fixtures
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def fresh_receipt(delta_us=20000):
    r = timed_receipt()
    r.update(query={'fresh': '1'}, headersAvailableMonotonic=100.06,
             bodyReadCompletedMonotonic=100.079, receivedMonotonic=100.08)
    r['headers'].update({'x-motionbrain-capture-mode': 'after_request',
                         'x-motionbrain-capture-requested-us': str(5000000-delta_us),
                         'x-motionbrain-discarded-frames': '2',
                         'x-motionbrain-source-age-us': '10000'})
    return r


class FreshSourceAgeTest(unittest.TestCase):
    def test_upper_contains_asymmetric_network_and_clock_rate_cases(self):
        for rate in (.999, 1, 1.001):
            for network in (0, .003, .015):
                r = fresh_receipt()
                b = basis(r)
                capture = 100.012 + network + .02/rate
                self.assertGreaterEqual(source_age_upper_ms(b, 10, 100.1), (100.1-capture)*1000)
                self.assertEqual(b['sourceAgeBoundStartedMonotonic'], 100.012)
                self.assertFalse(b['freshClockErrorCalibrated'])
                self.assertAlmostEqual(b['freshCaptureDelayLowerMs'], 20/1.001-.002)

    def test_diagnostics_use_same_formula_and_preserve_legacy_interval(self):
        r = fresh_receipt(); b = basis(r)
        d = capture_timing(r, status(0), status(1))
        self.assertTrue(d['valid'])
        self.assertAlmostEqual(d['causalAgeAtReceiptUpperMs'], source_age_upper_ms(b, 10, 100.08))
        self.assertAlmostEqual(d['reportedAgeAtReceiptIntervalMs'][1], 90)
        self.assertAlmostEqual(d['causalBoundReductionMs'], 12+b['sourceAgeBoundReductionMs'])

    def test_unknown_send_semantics_keeps_full_request_bound(self):
        r = fresh_receipt()
        for tag in (None, 'unknown'):
            r['connectionTiming']['sendStartSemantics'] = tag
            b = basis(r)
            self.assertNotIn('sourceCaptureLowerBoundMonotonic', b)
            self.assertAlmostEqual(source_age_upper_ms(b, 10, 100.08), 90)
        r.pop('connectionTiming')
        self.assertEqual(basis(r)['sourceAgeBoundBasis'], 'request_start')

    def test_impossible_camera_interval_or_selection_budget_fails_closed(self):
        for delta in (50000, 200000, 200001):
            with self.subTest(delta=delta), self.assertRaisesRegex(ValueError, 'timing_inconsistent'):
                basis(fresh_receipt(delta))
        r = fresh_receipt(); r['headers']['x-motionbrain-source-age-us'] = '1000000'
        with self.assertRaises(ValueError): basis(r)

    def test_exact_query_required_and_invalid_clock_assumptions_rejected(self):
        for query in (None, [], {'fresh': '1', 'extra': '1'}, {'fresh': True}, {'fresh': '0'}):
            r = fresh_receipt(); r['query'] = query
            with self.assertRaises(ValueError): basis(r)
        for name in ('FRESH_CLOCK_RATE_ERROR_PPM', 'FRESH_TIMESTAMP_ERROR_MS'):
            for value in (True, -1, float('nan'), float('inf')):
                with patch('learning.source_age.'+name, value), self.assertRaises(ValueError):
                    basis(fresh_receipt())

    def test_large_camera_uptime_is_subtracted_as_integer_first(self):
        r = fresh_receipt()
        c = 9007199254740993
        r['headers'].update({'x-motionbrain-captured-us': str(c),
            'x-motionbrain-frame-id': 'a'*32+':'+str(c),
            'x-motionbrain-capture-requested-us': str(c-20001)})
        self.assertEqual(basis(r)['freshCaptureDelayMs'], 20.001)

    def measure(self, times):
        body = jpeg(); r = fresh_receipt()
        r['sha256'] = metadata(body)['imageSha256']
        source = {**parse_capture_headers(r['headers']), 'imageSha256': r['sha256']}
        tracker = AnchoredRoiMeasurement(anchor(body), body, clock=Mock(monotonic=Mock(side_effect=times)))
        return tracker.measure(body, source, request_started=100, received=100.08,
                               host_clock_id='host-A', capture_receipt=r)

    def test_clock_error_crossing_100ms_rejects_and_removes_coordinates(self):
        # With zero margin the bound is 99.99ms. Rate + timestamp margin adds 0.02198ms.
        result = self.measure([100.13199]*3)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'source_age_exceeds_policy')
        self.assertGreater(result['reportedAgePlusElapsedMs'], 100)
        self.assertNotIn('targetBox', result)
        self.assertFalse(result['controlEligible'])
        with patch('learning.source_age.FRESH_CLOCK_RATE_ERROR_PPM', 0), patch('learning.source_age.FRESH_TIMESTAMP_ERROR_MS', 0):
            self.assertTrue(self.measure([100.13199]*3)['accepted'])

    def test_compute_finalization_regression_and_valid_acceptance(self):
        self.assertTrue(self.measure([100.09]*3)['accepted'])
        for times in ([100.09, 100.14, 100.14], [100.09, 100.10, 100.14],
                      [100.09, 100.10, 100.095], [100.09, 100.10, float('nan')]):
            r = self.measure(times)
            self.assertFalse(r['accepted'])
            self.assertNotIn('targetBox', r)


class FreshSourceAgeProbeTest(unittest.TestCase):
    setUp = fixtures.FastProbeTest.setUp
    run_probe = fixtures.FastProbeTest.run_probe

    def configure(self):
        original = self.transport.fault
        def fault(device, path, r, body):
            body = original(device, path, r, body)
            if device == 'camera' and path == '/status': r['json'] = {'freshCaptureSupported': True}
            if path == '/capture':
                s = r['requestStartedMonotonic']
                r.update(headersAvailableMonotonic=s+.018, bodyReadCompletedMonotonic=s+.019)
                r['headers'].update({'x-motionbrain-capture-mode': 'after_request',
                    'x-motionbrain-capture-requested-us': str(int(r['headers']['x-motionbrain-captured-us'])-10000),
                    'x-motionbrain-discarded-frames': '1', 'x-motionbrain-source-age-us': '2000'})
                r['connectionTiming'] = dict(resolveStartedMonotonic=s+.001, resolveCompletedMonotonic=s+.002,
                    connectedMonotonic=s+.003, requestSendStartedMonotonic=s+.004,
                    requestSendCompletedMonotonic=s+.005, addressAttempts=1,
                    sendStartSemantics='direct_http_first_send_before_socket_send_v1')
            return body
        self.transport.fault = fault

    def test_error_margin_rejection_sends_zero_commands(self):
        self.configure()
        from learning.fast_observation import _match_candidates
        def delayed(*args):
            value = _match_candidates(*args)
            self.transport.clock.advance(.09399)  # zero-error upper =99.99ms
            return value
        with patch('learning.fast_observation._match_candidates', side_effect=delayed):
            report = self.run_probe(fresh_capture=True)
        self.assertEqual(report['acceptedMeasurements'], 0, report)
        self.assertEqual(report['motorCommandsSent'], 0)
        self.assertTrue(report['outputLogUnchanged'])
        self.assertTrue(all(m['reason']=='source_age_exceeds_policy' for m in report['results']))
        self.assertTrue(all('targetBox' not in m for m in report['results']))

    def test_persistence_rechecks_same_bound_after_io(self):
        self.configure()
        original = ArtifactStore.save_image
        def slow(store, index, phase, body, digest):
            if index != 'anchor': self.transport.clock.advance(.100)
            return original(store, index, phase, body, digest)
        with patch.object(ArtifactStore, 'save_image', slow): report = self.run_probe(fresh_capture=True)
        self.assertEqual(report['acceptedMeasurements'], 2, report)
        rows = [e['payload'] for e in read_events(self.root)[0] if e['kind']=='measured']
        for row in rows:
            self.assertEqual(row['sourceAgeBoundBasis'], 'fresh_capture_lower_bound')
            self.assertFalse(row['withinAgePolicyAtPersistence'])
            self.assertGreater(row['reportedAgeAtPersistenceMs'], 100)
            self.assertFalse(row['freshClockErrorCalibrated'])
            self.assertAlmostEqual(row['reportedAgeAtPersistenceMs'],
                                   source_age_upper_ms(row, 2, row['persistedMonotonic']))
