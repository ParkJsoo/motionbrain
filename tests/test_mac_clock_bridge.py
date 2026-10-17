"""Synthetic clock brackets; no camera, network or platform assumptions."""
import copy
import unittest
from unittest.mock import patch
from learning.mac_clock_bridge import bridge,convert_pts,validate_sample,sensor_poll_marker_bounds,MacHostClock
from tests import test_mac_controller_probe as fixture


def sample(t,clock_id='test-clock'):
    return dict(schemaVersion='motionbrain.mac_clock_bridge.v1',hostClockId=clock_id,hostBootId='test-boot',
                sourceClock='coremedia_host_clock',targetClock='python_monotonic',sharedTimebase='mach_absolute_time',
                pythonClock=dict(implementation='mach_absolute_time()',monotonic=True,adjustable=False,resolution=1e-9),
                marginSeconds=.000001,opticalExposureVerified=False,trainingEligible=False,controlEligible=False,
                pairs=[dict(before=t+i*.001,after=t+i*.001+.000004,value=round((t+i*.001+.000002)*1e9),
                            timescale=10**9,flags=1,epoch=0) for i in range(8)])


class BridgeTest(unittest.TestCase):
    def test_bracket_bounds_do_not_promote_exposure(self):
        a,b=sample(100),sample(102)
        r=convert_pts(101,a,b)
        self.assertLess(r['earliest'],101);self.assertGreater(r['latest'],101)
        self.assertLess(r['latest']-r['earliest'],.00001)
        self.assertFalse(r['opticalExposureVerified'])
        self.assertFalse(bridge(a,b)['trainingEligible'])
    def test_no_extrapolation_before_or_after_calibration(self):
        for t in (99,100,103):
            with self.assertRaises(ValueError):convert_pts(t,sample(100),sample(102))
    def test_reboot_or_collector_change_rejected(self):
        for key in ('hostBootId','hostClockId'):
            b=sample(102);b[key]='different'
            with self.assertRaises(ValueError):bridge(sample(100),b)
    def test_clock_discontinuity_or_offset_rejected(self):
        b=sample(102)
        for p in b['pairs']:p['value']+=10**9
        with self.assertRaises(ValueError):bridge(sample(100),b)
    def test_slow_bracket_and_long_session_rejected(self):
        a=sample(100);a['pairs'][0]['after']+=.02
        with self.assertRaises(ValueError):validate_sample(a)
        with self.assertRaises(ValueError):bridge(sample(100),sample(151))
    def test_nonfinite_huge_int_and_bad_native_time_rejected(self):
        for key,value in [('value',10**400),('timescale',0),('flags',4),('epoch',1),('before',float('nan'))]:
            a=sample(100);a['pairs'][0][key]=value
            with self.assertRaises(ValueError):validate_sample(a)
    def test_other_python_clock_never_assumed_equal(self):
        a=sample(100);a['pythonClock']['implementation']='clock_gettime(CLOCK_MONOTONIC)'
        with self.assertRaises(ValueError):validate_sample(a)
        with patch('learning.mac_clock_bridge.platform.system',return_value='Linux'),self.assertRaises(ValueError):MacHostClock()
    def test_bad_shapes_and_missing_pairs(self):
        for change in ({'pairs':[]},{'pythonClock':None},{'hostClockId':None},{'opticalExposureVerified':True}):
            with self.assertRaises(ValueError):validate_sample({**sample(100),**change})
    def test_sensor_bounds_include_transport_and_integer_quantization(self):
        f=fixture.ProbeTest();f.setUp();self.addCleanup(f.doCleanups)
        row=f.read('http://example','status',clock=f.clock);row['hostClockId']='test-clock'
        r=sensor_poll_marker_bounds(row)
        self.assertAlmostEqual(r['earliest'],row['requestStartedMonotonic']-.002)
        self.assertAlmostEqual(r['latest'],row['receivedMonotonic'])
        self.assertFalse(r['sensorPhysicalSampleTimeVerified'])

class LinkedProbeTest(unittest.TestCase):
    def setUp(self):
        self.f=fixture.ProbeTest();self.f.setUp();self.addCleanup(self.f.doCleanups)
        for i,frame in enumerate(self.f.fixture.frames):
            for key in ('convertedHostPtsSeconds','callbackHostSeconds','processingCompleteHostSeconds','storageStartedHostSeconds'):
                frame[key]+=.1
            frame['pts']['value']+=100
            self.f.fixture.terminal['frames'][i]['storageCompleteHostSeconds']+=.1
            self.f.fixture.save_frame(i)
        original=self.f.camera
        def camera(*a):
            p=original(*a);self.f.clock.sleep(4.2);return p
        self.f.launch.side_effect=camera
        outer=self
        class FakeProbe:
            def sample(self,clock_id):
                r=sample(outer.f.clock.monotonic(),clock_id);outer.f.clock.sleep(.008);return r
        self.probe=FakeProbe()
    def test_opt_in_producer_and_audit_recompute_same_clock_link(self):
        from learning.mac_controller_probe import audit_probe
        r=self.f.run_probe(clock_probe=self.probe)
        self.assertEqual(r['status'],'recorded',r)
        self.assertTrue(audit_probe(self.f.root)['ok'])
        import json
        linked=json.loads((self.f.root/'clock/link.json').read_text())
        self.assertEqual(len(linked['frames']),21)
        self.assertTrue(linked['presentationClockLinked'])
        self.assertTrue(linked['postHocAssociation'])
        self.assertFalse(linked['physicalSampleAlignmentVerified'])
    def test_invalid_clock_or_expired_frame_rejects_session(self):
        from unittest.mock import Mock
        bad=Mock();bad.sample.side_effect=ValueError('clock_invalid')
        r=self.f.run_probe(clock_probe=bad)
        self.assertEqual(r['status'],'rejected')
        self.assertEqual(self.f.calls,[])
    def test_changed_link_rejected_even_with_rebound_manifest(self):
        import json
        from learning.artifacts import describe_artifact
        from learning.mac_controller_probe import audit_probe
        self.f.run_probe(clock_probe=self.probe)
        p=self.f.root/'clock/link.json';r=json.loads(p.read_text());r['frames'][0]['within20ms']=True;p.write_text(json.dumps(r))
        def rebind(r):
            r['artifacts']=[describe_artifact(self.f.root,x['artifactRef']) for x in r['artifacts']]
            r['clockLink']=describe_artifact(self.f.root,'clock/link.json')
        self.f.rewrite_summary(rebind)
        self.assertIn('clock_link_recomputation_mismatch',audit_probe(self.f.root)['issues'])


if __name__=='__main__':unittest.main()
