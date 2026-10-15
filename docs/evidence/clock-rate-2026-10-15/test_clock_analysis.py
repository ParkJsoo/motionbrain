"""Deterministic timing fixtures, no devices or network."""
import copy
from pathlib import Path
import sys
import unittest

REPO=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(REPO/'tests'))
from test_fresh_source_age import fresh_receipt
from clock_analysis import analyze, sample_from_receipt


def samples(rate=1, width=.01, count=4, spacing=60, origin=10**16):
    return [dict(cameraNowUs=origin+round(i*spacing*rate*1e6),hostLower=100+i*spacing-width,
                 hostUpper=100+i*spacing+width, cameraBootId='a',hostClockId='host',sourceFrameId=str(i))
            for i in range(count)]


class ClockAnalysisTest(unittest.TestCase):
    def test_known_rate_is_inside_bounds_despite_asymmetric_network(self):
        data=samples(rate=1.0002)
        for i,s in enumerate(data):
            s['hostLower']-=i*.003; s['hostUpper']+=.008*(3-i)
        r=analyze(data)
        for key in ('endpointAverage','constantAffineModel'):
            lo,hi=r[key]['cameraPerHostRate']
            self.assertLessEqual(lo,1.0002); self.assertGreaterEqual(hi,1.0002)
            self.assertEqual(r[key]['classification'],'contained_in_assumed_range')
        self.assertFalse(r['physicalCalibration'] or r['usableForControl'])

    def test_fast_and_slow_clocks_outside_assumption_are_not_filtered(self):
        for rate in (1.002,.998):
            r=analyze(samples(rate))
            self.assertEqual(r['endpointAverage']['classification'],'outside_assumed_range')

    def test_wide_network_window_is_inconclusive_not_pass(self):
        r=analyze(samples(width=.5))
        self.assertEqual(r['endpointAverage']['classification'],'overlaps_but_not_bounded_by_assumed_range')

    def test_short_excursion_can_hide_in_long_average(self):
        data=samples(width=.0001,spacing=1)
        data[1]['cameraNowUs']+=10000
        r=analyze(data)
        self.assertEqual(r['endpointAverage']['classification'],'contained_in_assumed_range')
        self.assertFalse(r['constantAffineModel']['feasible'])
        self.assertFalse(r['usableForControl'])

    def test_microsecond_margin_is_assumption_and_large_uptime_retains_delta(self):
        a=analyze(samples(origin=10**18),delta_error_us=0)
        b=analyze(samples(origin=10**18),delta_error_us=2)
        self.assertLess(b['endpointAverage']['ppm'][0],a['endpointAverage']['ppm'][0])
        self.assertGreater(b['endpointAverage']['ppm'][1],a['endpointAverage']['ppm'][1])
        self.assertFalse(b['timestampMarginMeasured'])

    def test_bad_identity_order_and_numbers_rejected(self):
        for field,value in [('cameraBootId','other'),('hostClockId','other'),('sourceFrameId','0'),
                            ('cameraNowUs',1),('cameraNowUs',True),('hostLower',float('nan')),
                            ('hostLower',99),('hostUpper',float('inf'))]:
            data=samples(); data[1][field]=value
            with self.subTest(field=field,value=value), self.assertRaises(ValueError): analyze(data)
        for margin in (True,-1,1.2,1001):
            with self.assertRaises(ValueError): analyze(samples(),delta_error_us=margin)
        with self.assertRaises(ValueError): analyze(samples(count=1))

    def test_receipt_uses_integer_age_event_and_certified_send(self):
        r=fresh_receipt(); s=sample_from_receipt(r)
        self.assertEqual(s['cameraNowUs'],int(r['headers']['x-motionbrain-captured-us'])+int(r['headers']['x-motionbrain-source-age-us']))
        self.assertEqual(s['hostLower'],r['connectionTiming']['requestSendStartedMonotonic'])
        for change in ('trace','status','host','age'):
            bad=copy.deepcopy(r)
            if change=='trace': bad['connectionTiming']['sendStartSemantics']='unknown'
            if change=='status': bad['status']=500
            if change=='host': bad['hostClockId']=''
            if change=='age': bad['headers']['x-motionbrain-source-age-us']='0.1'
            with self.assertRaises(ValueError): sample_from_receipt(bad)

    def test_fresh_rate_violation_not_selected_out_by_age_policy(self):
        r=fresh_receipt()
        r['headers']['x-motionbrain-capture-requested-us']='1'
        # This cannot fit the production fresh-age rate/budget guard, but clock
        # observation must be retained for independent diagnosis of that guard.
        self.assertTrue(sample_from_receipt(r)['fresh'])


if __name__=='__main__': unittest.main()
