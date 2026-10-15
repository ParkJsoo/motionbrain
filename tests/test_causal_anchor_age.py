"""Causal anchor expiry with virtual time and GET-only fake devices."""
import copy
import hashlib
import json
import unittest
from unittest.mock import Mock, patch

from learning.anchor_age import causal_anchor_basis, anchor_age_upper_ms
from learning.fast_observation import AnchoredRoiMeasurement
from learning.observation_consumer import ShadowObservationConsumer, ConsumptionStopped
from learning.artifacts import ArtifactStore, verify_artifacts
from test_fresh_source_age import fresh_receipt
import test_fresh_source_age as fresh_fixtures
import test_fast_observation as fixtures
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def receipt(body, captured=5_000_000, shift=0):
    r = fresh_receipt()
    r['sha256'] = hashlib.sha256(body).hexdigest()
    r['headers'].update({'x-motionbrain-captured-us': str(captured),
        'x-motionbrain-frame-id': fixtures.BOOT+':'+str(captured),
        'x-motionbrain-capture-requested-us': str(captured-10000)})
    for container in (r, r['connectionTiming']):
        for k, v in container.items():
            if k.endswith('Monotonic'): container[k] = v+shift
    return r


def source(r):
    return {**parse_capture_headers(r['headers']), 'imageSha256': r['sha256']}


def evidence(body):
    r = receipt(body)
    obs = {**fixtures.anchor(body), **fixtures.metadata(body, 6_000_000),
           'detectionSourceFrameId': fixtures.BOOT+':6000000'}
    return dict(hostClockId='host-A', checkpointReceipt=r, checkpointSource=source(r),
                anchorObservation=obs, anchorRetrievedMonotonic=103.)


class CausalAnchorTest(unittest.TestCase):
    def setUp(self):
        self.body = fixtures.jpeg()
        self.e = evidence(self.body)
        self.clock = Mock(monotonic=Mock(return_value=103.))

    def tracker(self):
        return AnchoredRoiMeasurement(self.e['anchorObservation'], self.body, clock=self.clock,
                                      fresh_age_mode='send_start', anchor_evidence=self.e)

    def measurement(self, now=114.99, captured=7_000_000):
        tracker = self.tracker()
        self.clock.monotonic.return_value = now
        r = receipt(self.body, captured, now-100.08)
        s = source(r)
        m = tracker.measure(self.body, s, request_started=r['requestStartedMonotonic'],
            received=r['receivedMonotonic'], host_clock_id='host-A', capture_receipt=r)
        return m, s, r

    def consumer(self):
        return ShadowObservationConsumer(self.clock, 'host-A', fresh_age_mode='send_start', anchor_evidence=self.e)

    def test_order_only_independent_of_camera_duration_and_rate_constants(self):
        b = causal_anchor_basis(self.e, 'host-A')
        self.assertEqual(b['anchorCaptureLowerBoundMonotonic'], 100.012)
        self.assertAlmostEqual(anchor_age_upper_ms(b, 115.012, 'host-A', 'host-A'), 15000)
        with patch.object(ShadowObservationConsumer, 'ANCHOR_CLOCK_MIN_RATE', float('nan')), \
             patch.object(ShadowObservationConsumer, 'ANCHOR_TIMESTAMP_ERROR_MS', float('nan')):
            m, s, r = self.measurement(captured=9_007_199_254_740_993)
            self.assertTrue(m['accepted'], m)
            row = self.consumer().consume(m, s, r, self.body)
            self.assertTrue(row['consumed'], row)
            self.assertFalse(row['anchorAgeClockRateAssumptionUsed'])
            self.assertNotIn('anchorClockMinRate', row)

    def test_old_reboot_held_missing_trace_host_and_receipt_mismatch_rejected(self):
        for fault in ('old','boot','held','trace','host','hash','clock','missing','type'):
            e = copy.deepcopy(self.e)
            if fault == 'old': e['anchorObservation']['capturedUs'] = 5_000_000
            if fault == 'boot': e['anchorObservation']['cameraBootId'] = 'b'*32
            if fault == 'held': e['anchorObservation']['held'] = True
            if fault == 'trace': e['checkpointReceipt'].pop('connectionTiming')
            if fault == 'host': e['hostClockId'] = 'another'
            if fault == 'hash': e['checkpointSource']['imageSha256'] = '0'*64
            if fault == 'clock': e['anchorRetrievedMonotonic'] = 99
            if fault == 'missing': e.pop('checkpointReceipt')
            if fault == 'type': e['anchorObservation'] = []
            with self.subTest(fault=fault), self.assertRaises((ValueError, KeyError)):
                causal_anchor_basis(e, 'host-A')

    def test_no_receipt_time_reset_or_cross_host_subtraction(self):
        b = causal_anchor_basis(self.e, 'host-A')
        self.assertGreater(anchor_age_upper_ms(b, 104., 'host-A', 'host-A'), 3900)
        for now, host in ((102., 'host-A'), (104., 'producer-clock'), (float('nan'), 'host-A')):
            with self.assertRaises(ValueError): anchor_age_upper_ms(b, now, host, 'host-A')
        self.clock.monotonic.return_value = 116.
        with self.assertRaisesRegex(ValueError, 'anchor_expired'): self.tracker()

    def test_15000_boundary_and_finalization_expiry_removes_coordinates(self):
        self.assertTrue(self.measurement(115.012)[0]['accepted'])
        self.clock.monotonic.return_value = 103.
        tracker = self.tracker()
        r = receipt(self.body, 7_000_000, 14.92)
        self.clock.monotonic.side_effect = [115.,115.001,115.013]
        m = tracker.measure(self.body, source(r), request_started=r['requestStartedMonotonic'],
            received=r['receivedMonotonic'], host_clock_id='host-A', capture_receipt=r)
        self.assertFalse(m['accepted']); self.assertEqual(m['reason'],'anchor_expired')
        self.assertNotIn('targetBox',m); self.assertTrue(tracker.invalidated)

    def test_consumption_rebuilds_pinned_bound_expiry_stops_after_storage(self):
        m,s,r = self.measurement()
        m['anchorCaptureLowerBoundMonotonic'] = 114.9
        m['anchorAgeMs'] = 0
        c = self.consumer()
        self.clock.monotonic.return_value = 115.013
        row = c.consume(m,s,r,self.body)
        self.assertFalse(row['consumed']); self.assertEqual(row['reason'],'anchor_expired_at_consumption')
        self.assertEqual(row['anchorCaptureLowerBoundMonotonic'],100.012)
        with self.assertRaises(ConsumptionStopped): c.finish_persistence(s['sourceFrameId'],'host-A')
        with self.assertRaises(ConsumptionStopped): c.check_ready()

    def test_anchor_identity_and_mode_cannot_be_swapped(self):
        m,s,r = self.measurement()
        for field in ('sourceFrameId','imageSha256','targetBox'):
            modified = copy.deepcopy(m); modified['semanticAnchor'][field] = 'wrong'
            row = self.consumer().consume(modified,s,r,self.body)
            self.assertFalse(row['consumed']); self.assertTrue(row['haltAfterPersistence'])
        m.pop('anchorAgeMode')
        self.assertEqual(self.consumer().consume(m,s,r,self.body)['reason'],'consumption_anchor_mismatch')

    def test_current_frame_cannot_precede_or_reboot_from_pinned_anchor(self):
        m,s,r = self.measurement()
        for change in ({'capturedUs': 5_000_000}, {'cameraBootId': 'b'*32}):
            row = self.consumer().consume(m,{**s,**change},r,self.body)
            self.assertFalse(row['consumed']); self.assertTrue(row['haltAfterPersistence'])


class CausalProbeTest(unittest.TestCase):
    setUp = fixtures.FastProbeTest.setUp
    run_probe = fixtures.FastProbeTest.run_probe

    def configure(self, newer=True):
        fresh_fixtures.FreshSourceAgeProbeTest.configure(self)
        self.transport.connection_timing = True
        if newer:
            self.transport.observation.update(capturedUs=1_150_000, sourceFrameId=fixtures.BOOT+':1150000',
                detectionSourceFrameId=fixtures.BOOT+':1150000')

    def run_causal(self, **kwargs):
        return self.run_probe(fresh_capture=True, fresh_age_mode='send_start', causal_anchor=True,
                              shadow_consumption=True, sleep=self.transport.clock.advance, **kwargs)

    def test_one_checkpoint_one_exact_anchor_and_bound_records(self):
        self.configure(); report = self.run_causal()
        self.assertEqual(report['status'],'completed',report)
        self.assertEqual(report['consumedObservations'],2)
        self.assertEqual(self.capture_count,3) # checkpoint + two current frames
        self.assertEqual(verify_artifacts(self.root,report['artifacts']),[])
        e = json.loads((self.root/'anchor_evidence.json').read_text())
        for m,row in zip(report['results'],report['consumptions']):
            self.assertEqual(m['semanticAnchor']['sourceFrameId'],e['anchorObservation']['sourceFrameId'])
            self.assertEqual(row['anchorAgeMode'],'causal_send_start')
            self.assertLessEqual(row['sourceAgeAtConsumptionMs'],100)
            self.assertLessEqual(row['persistenceElapsedMs'],250)
        self.assertEqual(report['motorCommandsSent'],0)

    def test_stale_anchor_stops_before_current_capture_no_retry(self):
        self.configure(newer=False); report = self.run_causal()
        self.assertEqual(report['status'],'failed'); self.assertIn('anchor_wait_exhausted',report['error'])
        self.assertEqual(self.capture_count,1); self.assertEqual(report['completedSamples'],0)
        self.assertEqual(sum(p=='/api/detection' for _,p,_ in self.transport.calls),8)
        self.assertFalse((self.root/'anchor.json').exists())

    def test_checkpoint_write_failure_stops_before_perception(self):
        self.configure(); original = ArtifactStore.save_image
        def fail(store,index,phase,body,digest):
            if index=='checkpoint': raise OSError('checkpoint disk failure')
            return original(store,index,phase,body,digest)
        with patch.object(ArtifactStore,'save_image',fail): report = self.run_causal()
        self.assertEqual(report['status'],'failed'); self.assertEqual(self.capture_count,1)
        self.assertFalse(any(p=='/api/detection' for _,p,_ in self.transport.calls))

    def test_late_or_failed_storage_prevents_next_current_capture(self):
        original = ArtifactStore.save_image
        for failed in (False,True):
            self.setUp(); self.configure()
            def fault(store,index,phase,body,digest):
                if index=='0':
                    if failed: raise OSError('frame disk failure')
                    self.transport.clock.advance(.251)
                return original(store,index,phase,body,digest)
            with patch.object(ArtifactStore,'save_image',fault): report = self.run_causal()
            self.assertEqual(report['status'],'failed'); self.assertEqual(self.capture_count,2)
            self.assertEqual(report['motorCommandsSent'],0)

    def test_invalid_mode_sends_zero_requests(self):
        with self.assertRaises(ValueError): self.run_probe(causal_anchor=True)
        self.assertEqual(self.transport.calls,[])

    def test_expired_acquisition_stops_before_current_capture(self):
        self.configure(newer=False)
        def delayed(_):
            self.transport.clock.advance(16)
            self.transport.observation.update(capturedUs=1_150_000,sourceFrameId=fixtures.BOOT+':1150000',
                detectionSourceFrameId=fixtures.BOOT+':1150000')
        report = self.run_probe(fresh_capture=True, fresh_age_mode='send_start', causal_anchor=True,
            shadow_consumption=True, sleep=delayed)
        self.assertEqual(report['status'],'failed'); self.assertIn('anchor_expired',report['error'])
        self.assertEqual(self.capture_count,1)

    def test_later_detection_arrives_within_bounded_metadata_polls(self):
        self.configure(newer=False)
        original=self.transport.fault
        polls=[]
        def advancing(device,path,r,body):
            body=original(device,path,r,body)
            if path=='/api/detection':
                polls.append(1)
                if len(polls)==3:
                    self.transport.observation.update(capturedUs=1_150_000,sourceFrameId=fixtures.BOOT+':1150000',
                        detectionSourceFrameId=fixtures.BOOT+':1150000')
                    r['json']=copy.deepcopy(self.transport.observation)
            return body
        self.transport.fault=advancing
        report=self.run_causal()
        self.assertEqual(report['status'],'completed',report)
        self.assertEqual(len(polls),4) # three polls + exact adapter discovery
        self.assertEqual(self.capture_count,3)

    def test_wait_transport_or_identity_error_stops_without_retry(self):
        for fault in ('transport','boot'):
            self.setUp(); self.configure(newer=False); original=self.transport.fault
            def failed(device,path,r,body):
                body=original(device,path,r,body)
                if path=='/api/detection':
                    if fault=='transport': r['status']=503
                    else: r['json']['cameraBootId']='b'*32
                return body
            self.transport.fault=failed
            report=self.run_causal()
            self.assertEqual(report['status'],'failed')
            self.assertEqual(sum(p=='/api/detection' for _,p,_ in self.transport.calls),1)
            self.assertEqual(self.capture_count,1)
