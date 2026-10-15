"""Rate-free current-frame fresh bound; no devices or motor commands."""
import copy
import hashlib
import json
import unittest
from unittest.mock import Mock, patch

from learning.source_age import source_age_basis, source_age_upper_ms
from learning.fast_observation import AnchoredRoiMeasurement
from learning.observation_consumer import ShadowObservationConsumer
from learning.artifacts import ArtifactStore
from learning.capture_timing import capture_timing
from test_fresh_source_age import fresh_receipt
import test_fresh_source_age as fresh_fixtures
import test_fast_observation as fixtures
from test_capture_timing import status
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def bound(receipt, mode='send_start'):
    source={**parse_capture_headers(receipt['headers']), 'imageSha256':receipt['sha256']}
    return source_age_basis(receipt,source,request_started=receipt['requestStartedMonotonic'],
        received=receipt['receivedMonotonic'],host_clock_id=receipt['hostClockId'],fresh_age_mode=mode)


class ConservativeBoundTest(unittest.TestCase):
    def test_bound_uses_first_send_not_completion_or_camera_duration(self):
        r=fresh_receipt(); b=bound(r)
        self.assertEqual(b['sourceCaptureLowerBoundMonotonic'],100.012)
        self.assertEqual(b['sourceAgeBoundBasis'],'fresh_send_start')
        self.assertAlmostEqual(source_age_upper_ms(b,9999,100.09),78)
        for actual_capture in (100.012,100.013,100.04):
            self.assertGreaterEqual(source_age_upper_ms(b,0,100.09),(100.09-actual_capture)*1000)
        self.assertFalse(b['sourceAgeClockRateAssumptionUsed'])
        self.assertFalse(b['sourceAgeTimestampMarginUsed'])
        self.assertNotIn('freshClockRateErrorPpm',b)

    def test_no_rate_constants_or_duration_budget_gate(self):
        r=fresh_receipt(); r['headers']['x-motionbrain-capture-requested-us']='1'
        with self.assertRaises(ValueError): bound(r,'clock_margin')
        with patch('learning.source_age.FRESH_CLOCK_RATE_ERROR_PPM',float('nan')), \
             patch('learning.source_age.FRESH_TIMESTAMP_ERROR_MS',float('nan')):
            self.assertEqual(bound(r)['sourceAgeBoundBasis'],'fresh_send_start')

    def test_missing_or_invalid_proof_never_falls_back(self):
        r=fresh_receipt()
        for mutation in ('legacy','missing_trace','unknown_send','bad_order','wrong_frame','bad_fresh'):
            v=copy.deepcopy(r)
            if mutation=='legacy': v['query']={}
            if mutation=='missing_trace': v.pop('connectionTiming')
            if mutation=='unknown_send': v['connectionTiming']['sendStartSemantics']='unknown'
            if mutation=='bad_order': v['connectionTiming']['requestSendStartedMonotonic']=101
            if mutation=='wrong_frame': v['headers']['x-motionbrain-frame-id']='wrong'
            if mutation=='bad_fresh': v['headers']['x-motionbrain-capture-requested-us']='5000000'
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): bound(v)
        with self.assertRaises(ValueError):
            source_age_basis(None,{},request_started=1,received=2,host_clock_id='host',fresh_age_mode='send_start')
        for mode in ('unknown',None,True):
            with self.assertRaises(ValueError): bound(r,mode)

    def measurement(self,now,mode):
        body=fixtures.jpeg(); r=fresh_receipt(); r['sha256']=hashlib.sha256(body).hexdigest()
        source={**parse_capture_headers(r['headers']),'imageSha256':r['sha256']}
        clock=Mock(); clock.monotonic.return_value=now
        tracker=AnchoredRoiMeasurement(fixtures.anchor(body),body,clock=clock,fresh_age_mode=mode)
        m=tracker.measure(body,source,request_started=100,received=100.08,host_clock_id='host-A',capture_receipt=r)
        return m,r,source,body,clock

    def test_measurement_100ms_boundary_and_conservative_rejection(self):
        self.assertTrue(self.measurement(100.112,'send_start')[0]['accepted'])
        strict=self.measurement(100.113,'send_start')[0]
        self.assertFalse(strict['accepted']); self.assertNotIn('targetBox',strict)
        self.assertTrue(self.measurement(100.113,'clock_margin')[0]['accepted'])

    def test_consumption_rebuilds_strict_bound_and_checks_mode(self):
        m,r,s,body,clock=self.measurement(100.10,'send_start')
        m['sourceCaptureLowerBoundMonotonic']=999
        clock.monotonic.return_value=100.113
        consumer=ShadowObservationConsumer(clock,'host-A',fresh_age_mode='send_start')
        row=consumer.consume(m,s,r,body)
        self.assertFalse(row['consumed']); self.assertGreater(row['sourceAgeAtConsumptionMs'],100)
        m['freshAgeMode']='clock_margin'
        row=ShadowObservationConsumer(clock,'host-A',fresh_age_mode='send_start').consume(m,s,r,body)
        self.assertEqual(row['reason'],'consumption_age_mode_mismatch')
        self.assertTrue(row['haltAfterPersistence'])

    def test_diagnostic_uses_same_bound(self):
        r=fresh_receipt(); d=capture_timing(r,status(0),status(1),fresh_age_mode='send_start')
        self.assertTrue(d['valid']); self.assertEqual(d['sourceAgeBoundBasis'],'fresh_send_start')
        self.assertAlmostEqual(d['causalAgeAtReceiptUpperMs'],68)


class ConservativeProbeTest(unittest.TestCase):
    setUp=fixtures.FastProbeTest.setUp
    run_probe=fixtures.FastProbeTest.run_probe

    def configure(self):
        fresh_fixtures.FreshSourceAgeProbeTest.configure(self)
        self.transport.connection_timing=True

    def run_strict(self):
        return self.run_probe(fresh_capture=True,shadow_consumption=True,capture_timing=True,fresh_age_mode='send_start')

    def test_all_stages_record_selected_mode(self):
        self.configure(); report=self.run_strict()
        self.assertEqual(report['status'],'completed',report)
        self.assertEqual(report['consumedObservations'],2)
        for i in range(2):
            for folder in ('measurements','consumption','persistence','timing'):
                obj=json.loads((self.root/f'{folder}/{i:04d}.json').read_text())
                self.assertEqual(obj['freshAgeMode'],'send_start')
                self.assertEqual(obj['sourceAgeBoundBasis'],'fresh_send_start')
            self.assertEqual(report['consumptions'][i]['storageStatus'],'completed')
        self.assertEqual(json.loads((self.root/'session.json').read_text())['freshAgeMode'],'send_start')
        self.assertEqual(report['motorCommandsSent'],0)

    def test_misconfiguration_sends_zero_requests(self):
        for kwargs in ({},{'fresh_capture':True}):
            self.setUp()
            with self.assertRaises(ValueError): self.run_probe(fresh_age_mode='send_start',**kwargs)
        self.assertEqual(self.transport.calls,[])

    def test_storage_deadline_and_failure_still_stop_after_one_capture(self):
        image=ArtifactStore.save_image
        for fail in (False,True):
            self.setUp(); self.configure()
            def injected(store,index,phase,body,digest):
                if index=='0':
                    if fail: raise OSError('test image failure')
                    self.transport.clock.advance(.251)
                return image(store,index,phase,body,digest)
            with patch.object(ArtifactStore,'save_image',injected): report=self.run_strict()
            self.assertEqual(report['status'],'failed')
            self.assertEqual(self.capture_count,1); self.assertEqual(report['motorCommandsSent'],0)
            self.assertEqual(report['consumptions'][0]['storageStatus'],'unconfirmed' if fail else 'late')

    def test_late_consumption_has_zero_shadow_use(self):
        self.configure(); measure=AnchoredRoiMeasurement.measure
        def delayed(*args,**kwargs):
            result=measure(*args,**kwargs); self.transport.clock.advance(.101); return result
        with patch.object(AnchoredRoiMeasurement,'measure',delayed): report=self.run_strict()
        self.assertEqual(report['acceptedMeasurements'],2)
        self.assertEqual(report['consumedObservations'],0)
        self.assertEqual(report['motorCommandsSent'],0)
