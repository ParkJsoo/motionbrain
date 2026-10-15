"""Experiment control tests; no network or devices."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock,patch

spec=importlib.util.spec_from_file_location('causal_collect',Path(__file__).with_name('collect.py'))
runner=importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)


class InjectionTest(unittest.TestCase):
    def test_only_second_status_delays_until_checkpoint_deadline(self):
        clock=Mock(); clock.monotonic.side_effect=[105.,115.05]
        http=Mock(); http.get.return_value=({'connectionTiming':{'requestSendStartedMonotonic':100.}},b'jpeg')
        wrap=runner.ExpiryInjection(http,True,clock)
        wrap.get('controller','/status'); wrap.get('camera','/capture',{'fresh':'1'})
        wrap.get('controller','/status'); wrap.get('controller','/status')
        self.assertAlmostEqual(clock.sleep.call_args.args[0],10.05)
        self.assertEqual(clock.sleep.call_count,1); self.assertEqual(wrap.captures,1)
        self.assertEqual(wrap.injection['checkpointSendMonotonic'],100.)

    def test_normal_has_no_added_delay(self):
        http=Mock(); http.get.return_value=({},b'')
        clock=Mock(); wrap=runner.ExpiryInjection(http,False,clock)
        for _ in range(3): wrap.get('controller','/status')
        clock.sleep.assert_not_called()

    def test_unexpected_normal_failure_never_runs_expiry(self):
        state={'outputs':{},'camera':{},'controller':{'lastCommand':dict(id=9,type='disarm',source='web',executedAtMs=1)}}
        report=dict(status='failed',error='fixture storage error',completedSamples=0,acceptedMeasurements=0,
                    consumedObservations=0,consumptionHalted=True,outputLogUnchanged=True,motorCommandsSent=0)
        with tempfile.TemporaryDirectory() as d, patch.object(runner,'snapshot',return_value=state), \
             patch('learning.fast_probe.run_fast_probe',return_value=report) as probe:
            self.assertFalse(runner.collect(Path(d)))
            self.assertEqual(probe.call_count,1)
            self.assertTrue(probe.call_args.kwargs['causal_anchor'])
            self.assertTrue(probe.call_args.kwargs['shadow_consumption'])
            self.assertEqual(probe.call_args.kwargs['max_persistence_ms'],250)
            self.assertTrue((Path(d)/'stop.json').exists())


if __name__=='__main__': unittest.main()
