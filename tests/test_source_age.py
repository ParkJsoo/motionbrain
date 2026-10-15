"""Causal send-start bound; virtual clocks and sockets, no hardware output."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from learning.capture_timing import capture_timing
from learning.dry_run import VirtualClock
from learning.fast_observation import AnchoredRoiMeasurement
from learning.http_timing import TimedHTTPHandler, SEND_START_SEMANTICS
from learning.source_age import source_age_basis
from test_capture_timing import receipt, status
from test_fast_observation import jpeg, anchor, metadata
import test_fast_observation as fast_fixtures
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def timed_receipt():
    r = receipt()
    r['connectionTiming'] = dict(resolveStartedMonotonic=100.001, resolveCompletedMonotonic=100.002,
        connectedMonotonic=100.01, requestSendStartedMonotonic=100.012,
        requestSendCompletedMonotonic=100.015, addressAttempts=1, sendStartSemantics=SEND_START_SEMANTICS)
    return r


def basis(r):
    return source_age_basis(r, {**parse_capture_headers(r['headers']), 'imageSha256': r['sha256']},
        request_started=r['requestStartedMonotonic'], received=r['receivedMonotonic'], host_clock_id=r['hostClockId'])


class SourceAgeTest(unittest.TestCase):
    def test_start_is_before_socket_send_and_completion_can_follow_server_sample(self):
        clock = VirtualClock()
        trace = {}
        handler = TimedHTTPHandler()
        req = SimpleNamespace(motionbrain_clock=clock, motionbrain_connection_timing=trace)
        server_event = []
        def opened(factory, request):
            conn = factory('fixture')
            conn.sock = Mock()
            def sendall(data):
                self.assertEqual(trace['requestSendStartedMonotonic'], clock.monotonic())
                self.assertEqual(trace['sendStartSemantics'], SEND_START_SEMANTICS)
                clock.advance(.001)
                server_event.append(clock.monotonic())
                clock.advance(.030)  # local scheduling after server already samples age
            conn.sock.sendall.side_effect = sendall
            conn.send(b'GET /capture HTTP/1.1\r\n\r\n')
        with patch.object(handler, 'do_open', side_effect=opened):
            handler.http_open(req)
        self.assertLess(trace['requestSendStartedMonotonic'], server_event[0])
        self.assertGreater(trace['requestSendCompletedMonotonic'], server_event[0])

    def test_bound_contains_every_possible_server_event_not_just_rtt_midpoint(self):
        r = timed_receipt()
        b = basis(r)['sourceAgeBoundStartedMonotonic']
        for event in (100.012, 100.013, 100.02, 100.039, 100.04):
            actual_age = 30 + (100.08-event)*1000
            self.assertGreaterEqual(30+(100.08-b)*1000, actual_age)
        self.assertEqual(b, 100.012)
        d = capture_timing(r, status(0), status(1))
        self.assertAlmostEqual(d['causalBoundReductionMs'], 12)
        self.assertAlmostEqual(d['causalAgeAtReceiptUpperMs'], 79)
        self.assertAlmostEqual(d['reportedAgeAtReceiptIntervalMs'][1], 91)

    def test_missing_or_unknown_semantics_falls_back_without_relabeling_request(self):
        for semantics in (None, 'socket_send_completed'):
            r = timed_receipt()
            r['connectionTiming']['sendStartSemantics'] = semantics
            self.assertEqual(basis(r)['sourceAgeBoundStartedMonotonic'], 100)
            self.assertEqual(r['requestStartedMonotonic'], 100)
        r.pop('connectionTiming')
        self.assertEqual(basis(r)['sourceAgeBoundBasis'], 'request_start')

    def test_bad_trace_never_narrows_bound(self):
        for field, value in [('requestSendStartedMonotonic', 99), ('connectedMonotonic', 100.05),
                             ('requestSendCompletedMonotonic', 101), ('addressAttempts', True),
                             ('resolveCompletedMonotonic', float('nan'))]:
            r = timed_receipt(); r['connectionTiming'][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                basis(r)
        for trace in (None, {}):
            r = timed_receipt(); r['connectionTiming'] = trace
            with self.assertRaises(ValueError): basis(r)

    def measurement(self, age=30, end=100.08, changes=None, clock_values=None):
        body = jpeg()
        r = timed_receipt()
        r['sha256'] = metadata(body)['imageSha256']
        r['headers']['x-motionbrain-source-age-us'] = str(round(age*1000))
        source = {**parse_capture_headers(r['headers']), 'imageSha256': r['sha256']}
        if changes: changes(r)
        clock = Mock()
        clock.monotonic.side_effect = clock_values or [end, end, end]
        tracker = AnchoredRoiMeasurement(anchor(body), body, clock=clock)
        return tracker.measure(body, source, request_started=100, received=100.061,
                               host_clock_id='host-A', capture_receipt=r)

    def test_measurement_accepts_only_tighter_valid_bound_preserves_full_bound(self):
        result = self.measurement()
        self.assertTrue(result['accepted'], result)
        self.assertAlmostEqual(result['reportedAgePlusElapsedMs'], 98)
        self.assertAlmostEqual(result['fullRequestAgePlusElapsedMs'], 110)
        self.assertEqual(result['requestStartedMonotonic'], 100)
        self.assertFalse(result['controlEligible'])
        self.assertFalse(result['trainingEligible'])

    def test_compute_and_finalization_still_count_and_drop_coordinates(self):
        for values in ([100.07, 100.09, 100.09], [100.07, 100.08, 100.09]):
            r = self.measurement(clock_values=values)
            self.assertFalse(r['accepted'])
            self.assertEqual(r['reason'], 'source_age_exceeds_policy')
            self.assertNotIn('targetBox', r)

    def test_final_clock_regression_and_nan_fail_closed(self):
        for end in (100.06, 100.075, float('nan')):
            r = self.measurement(clock_values=[100.07, 100.08, end])
            self.assertFalse(r['accepted'])
            self.assertNotIn('targetBox', r)
            self.assertEqual(r['reason'], 'invalid_local_timing')

    def test_other_host_frame_hash_age_and_failed_http_cannot_supply_bound(self):
        mutations = [lambda r: r.update(hostClockId='other'), lambda r: r.update(sha256='d'*64),
                     lambda r: r.update(status=500), lambda r: r.update(error='TimeoutError'),
                     lambda r: r.update(receivedMonotonic=100.062),
                     lambda r: r['headers'].update({'x-motionbrain-source-age-us':'1'}),
                     lambda r: r['headers'].update({'x-motionbrain-camera-boot-id':'b'*32}),
                     lambda r: r['headers'].update({'x-motionbrain-frame-id':'wrong'})]
        for change in mutations:
            result = self.measurement(changes=change)
            self.assertFalse(result['accepted'], result)
            self.assertNotIn('targetBox', result)


# Reuse fixture setup only, without inheriting its tests a second time.
class SourceAgeProbeTest(unittest.TestCase):
    setUp = fast_fixtures.FastProbeTest.setUp
    run_probe = fast_fixtures.FastProbeTest.run_probe

    def configure(self, age):
        original = self.transport.fault
        def timed(device, path, r, body):
            body = original(device, path, r, body)
            if path == '/capture':
                start = r['requestStartedMonotonic']
                r.update(headersAvailableMonotonic=start+.015, bodyReadCompletedMonotonic=start+.019)
                r['headers']['x-motionbrain-source-age-us'] = str(age*1000)
                r['connectionTiming'] = dict(resolveStartedMonotonic=start+.001,
                    resolveCompletedMonotonic=start+.002, connectedMonotonic=start+.008,
                    requestSendStartedMonotonic=start+.010, requestSendCompletedMonotonic=start+.011,
                    addressAttempts=1, sendStartSemantics=SEND_START_SEMANTICS)
            return body
        self.transport.fault = timed

    def test_over_100_rejected_with_zero_motor_commands(self):
        self.configure(91)
        report = self.run_probe()
        self.assertEqual(report['status'], 'completed', report)
        self.assertEqual(report['acceptedMeasurements'], 0)
        self.assertEqual(report['motorCommandsSent'], 0)
        self.assertTrue(report['outputLogUnchanged'])
        self.assertTrue(all(r['reason']=='source_age_exceeds_policy' for r in report['results']))

    def test_persistence_uses_same_bound_and_includes_later_io(self):
        from learning.artifacts import ArtifactStore, read_events
        self.configure(85)
        original = ArtifactStore.save_image
        def slow(store, index, phase, body, digest):
            if index != 'anchor': self.transport.clock.advance(.02)
            return original(store, index, phase, body, digest)
        with patch.object(ArtifactStore, 'save_image', slow): report = self.run_probe()
        self.assertEqual(report['acceptedMeasurements'], 2, report)
        rows = [e['payload'] for e in read_events(self.root)[0] if e['kind']=='measured']
        for row in rows:
            self.assertFalse(row['withinAgePolicyAtPersistence'])
            self.assertAlmostEqual(row['reportedAgeAtPersistenceMs'], 115)
            self.assertAlmostEqual(row['fullRequestAgeAtPersistenceMs'], 125)
            self.assertEqual(row['sourceAgeBoundBasis'], 'first_get_send_start')
