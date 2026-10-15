"""Opt-in fresh capture, unsupported firmware and no silent fallback. No devices."""
import unittest

from learning.artifacts import read_events, verify_artifacts
from learning.fresh_capture import validate_fresh_capture
from learning.source_age import source_age_basis
from test_capture_timing import receipt, status
from learning.capture_timing import capture_timing
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers
import test_fast_observation as fixtures


class FreshCaptureContractTest(unittest.TestCase):
    def good(self):
        r = receipt()
        r['query'] = {'fresh':'1'}
        r['headers'].update({'x-motionbrain-capture-mode':'after_request',
                             'x-motionbrain-capture-requested-us':'4999999',
                             'x-motionbrain-discarded-frames':'2'})
        return r

    def test_strict_after_request_and_legacy_stays_legacy(self):
        r = self.good()
        self.assertEqual(validate_fresh_capture(r, parse_capture_headers(r['headers']))['discardedFrames'], 2)
        self.assertIsNone(validate_fresh_capture(receipt(), parse_capture_headers(receipt()['headers'])))

    def test_selection_duration_is_not_reported_as_one_fb_get_wait(self):
        r = self.good()
        timing = capture_timing(r, status(0), status(1))
        self.assertTrue(timing['valid'])
        self.assertEqual(timing['serverFreshFrameSelectionMs'],2)
        self.assertNotIn('serverFrameAcquireWaitMs',timing)

    def test_missing_malformed_equal_and_future_request_proof_rejected(self):
        for key, value in [('x-motionbrain-capture-mode','latest'), ('x-motionbrain-capture-requested-us',''),
                           ('x-motionbrain-capture-requested-us','5000000'),
                           ('x-motionbrain-capture-requested-us','5000001'),
                           ('x-motionbrain-capture-requested-us','-1'),
                           ('x-motionbrain-discarded-frames','4')]:
            r = self.good(); r['headers'][key] = value
            source = {**parse_capture_headers(r['headers']), 'imageSha256':r['sha256']}
            with self.subTest(key=key,value=value), self.assertRaises(ValueError):
                source_age_basis(r, source, request_started=r['requestStartedMonotonic'],
                                 received=r['receivedMonotonic'], host_clock_id=r['hostClockId'])


class FreshProbeTest(unittest.TestCase):
    setUp = fixtures.FastProbeTest.setUp
    run_probe = fixtures.FastProbeTest.run_probe

    def configure(self, *, supported=True, proof=True, boot_changed=False):
        original = self.transport.fault
        def fresh(device, path, r, body):
            body = original(device,path,r,body)
            if device == 'camera' and path == '/status':
                r['json'] = {'freshCaptureSupported': supported}
            if path == '/capture' and proof:
                r['headers'].update({'x-motionbrain-capture-mode':'after_request',
                    'x-motionbrain-capture-requested-us':str(int(r['headers']['x-motionbrain-captured-us'])-1),
                    'x-motionbrain-discarded-frames':'1'})
                if boot_changed:
                    r['headers']['x-motionbrain-camera-boot-id'] = 'b'*32
                    r['headers']['x-motionbrain-frame-id'] = 'b'*32+':'+r['headers']['x-motionbrain-captured-us']
            return body
        self.transport.fault = fresh

    def test_opt_in_verified_and_originals_saved_without_commands(self):
        self.configure()
        r = self.run_probe(fresh_capture=True)
        self.assertEqual(r['status'],'completed',r)
        self.assertEqual(r['acceptedMeasurements'],2)
        self.assertEqual(r['motorCommandsSent'],0)
        self.assertEqual(verify_artifacts(self.root,r['artifacts']),[])
        self.assertTrue(all(q=={'fresh':'1'} for _,p,q in self.transport.calls if p=='/capture'))
        self.assertTrue(all(not m['controlEligible'] for m in r['results']))

    def test_unsupported_firmware_fails_before_capture(self):
        self.configure(supported=False)
        r = self.run_probe(fresh_capture=True)
        self.assertEqual(r['status'],'failed')
        self.assertIn('fresh_capture_unsupported',r['error'])
        self.assertEqual(self.capture_count,0)

    def test_silent_legacy_response_is_failed_and_original_preserved(self):
        self.configure(proof=False)
        r = self.run_probe(fresh_capture=True)
        self.assertEqual(r['status'],'failed')
        self.assertIn('fresh_capture_contract_mismatch',r['error'])
        self.assertEqual(self.capture_count,1)
        self.assertEqual((self.root/'images/0/source.jpg').read_bytes(),self.body)
        self.assertEqual(r['acceptedMeasurements'],0)
        self.assertEqual(read_events(self.root)[0][-1]['kind'],'finished')

    def test_reboot_does_not_reuse_previous_anchor(self):
        self.configure(boot_changed=True)
        r = self.run_probe(fresh_capture=True)
        self.assertEqual(r['acceptedMeasurements'],0)
        self.assertEqual(r['results'][0]['reason'],'camera_rebooted')

    def test_selection_503_is_recorded_without_retry_or_fallback(self):
        self.configure()
        original = self.transport.fault
        def fail(device,path,r,body):
            body = original(device,path,r,body)
            if path == '/capture': r['status'] = 503
            return body
        self.transport.fault = fail
        r = self.run_probe(fresh_capture=True)
        self.assertEqual(r['status'],'failed')
        self.assertEqual(self.capture_count,1)
        self.assertEqual(r['motorCommandsSent'],0)

    def test_default_never_requests_fresh_or_requires_capability(self):
        r = self.run_probe()
        self.assertEqual(r['status'],'completed')
        self.assertTrue(all(q is None for _,p,q in self.transport.calls if p=='/capture'))

    def test_fresh_proof_never_overrides_100ms_age_limit(self):
        self.configure()
        original = self.transport.fault
        def delayed(device,path,r,body):
            body = original(device,path,r,body)
            if path == '/capture': r['headers']['x-motionbrain-source-age-us'] = '100000'
            return body
        self.transport.fault = delayed
        r = self.run_probe(fresh_capture=True)
        self.assertEqual(r['acceptedMeasurements'],0)
        self.assertEqual(r['motorCommandsSent'],0)
        self.assertTrue(all(m['reason']=='source_age_exceeds_policy' for m in r['results']))

    def test_non_boolean_option_refused(self):
        with self.assertRaises(ValueError):self.run_probe(fresh_capture='1')
