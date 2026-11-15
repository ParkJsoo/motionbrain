"""Explicit read-only startup ring loading; monitoring remains paged."""
import copy
import unittest

from learning.supervised_transport import SequencePort
from tests import test_supervised_transport as fixture
from tests.test_supervised_transport import Response, pulse


class InitialFullOutputSnapshotTests(unittest.TestCase):
    def setup_port(self):
        port, device, store = fixture.CompactTransportTests().setup_port()
        port.initial_full_output_snapshot = True
        original = port.opener.open
        def opened(request, timeout):
            if request.full_url.endswith('/action_outputs'):
                device.clock.sleep(.9)
                device.requests.append(('/action_outputs', None))
                retained = copy.deepcopy(device.entries[-128:])
                return Response(dict(bootId=device.boot, evidence='driver_write',
                    clockDomain='controller_boot_millis', dropped=retained[0]['sequence']-1,
                    entries=retained))
            return original(request, timeout)
        port.opener.open = opened
        return port, device, store

    def test_explicit_stop_full_ring_bootstraps_once_then_monitor_uses_pages(self):
        port, device, store = self.setup_port()
        for _ in range(128): device.append(4, 0)
        for motor in range(1, 6): device.append(motor, 0)
        before = device.clock.monotonic()
        snapshot = port.preflight()
        self.assertTrue(port.off_verified)
        self.assertEqual(port.last_outputs, {motor:0 for motor in range(1,6)})
        self.assertLess(device.clock.monotonic()-before, 1.5)
        self.assertEqual(len(snapshot['outputs']['entries']),128)
        self.assertEqual(snapshot['outputs']['latestSequence'],138)
        self.assertEqual([path for path, _ in device.requests if path.startswith('/action_outputs')],
                         ['/action_outputs'])
        device.requests.clear()
        port.check_idle()
        self.assertEqual([path for path, _ in device.requests if path.startswith('/action_outputs')],
                         ['/action_outputs?after=138&limit=8'])
        self.assertTrue(all(payload is None for _,payload in device.requests))
        self.assertFalse(any('attempt-2' in name for name in store.rows))

    def test_missing_axis_OFF_is_still_rejected(self):
        port, device, _ = self.setup_port()
        for _ in range(128): device.append(4,0)
        with self.assertRaisesRegex(ValueError,'driver_off_not_proven'): port.preflight()
        self.assertFalse(port.off_verified)

    def test_changed_or_incomplete_ring_rejected_before_OFF(self):
        for mode in ('boot','drop','too_many','gap','nonzero'):
            with self.subTest(mode=mode):
                port,device,_=self.setup_port()
                original=port.opener.open
                def changed(request,timeout):
                    response=original(request,timeout)
                    if request.full_url.endswith('/action_outputs'):
                        import json
                        value=json.loads(response.raw)
                        if mode=='boot':value['bootId']='other'
                        if mode=='drop':value['dropped']=1
                        if mode=='too_many':value['entries']*=26
                        if mode=='gap':value['entries'][2]['sequence']+=1
                        if mode=='nonzero':value['entries'][-1]['pwm']=10
                        return Response(value)
                    return response
                port.opener.open=changed
                with self.assertRaises(ValueError):port.preflight()
                self.assertFalse(port.off_verified)
                self.assertTrue(all(payload is None for _,payload in device.requests))

    def test_active_output_cannot_select_unpaged_bootstrap(self):
        port,device,_=self.setup_port()
        port._active_pulse=pulse('shoulder','down',20,250)
        port.may_have_run=port.writes_possible=True
        port._outputs()
        self.assertFalse(any(path=='/action_outputs' for path,_ in device.requests))
        self.assertTrue(any(path.startswith('/action_outputs?') for path,_ in device.requests))

    def test_failed_full_read_has_no_retry_or_paged_fallback(self):
        port,device,store=self.setup_port()
        original=port.opener.open
        attempts=[]
        def failed(request,timeout):
            if request.full_url.endswith('/action_outputs'):
                attempts.append(request.full_url)
                raise TimeoutError('read_timeout')
            return original(request,timeout)
        port.opener.open=failed
        with self.assertRaises(TimeoutError):port.preflight()
        self.assertEqual(len(attempts),1)
        self.assertFalse(port.off_verified)
        self.assertIsNone(port.last_sequence)
        self.assertFalse(any(path.startswith('/action_outputs?') for path,_ in device.requests))
        self.assertFalse(any('attempt-2' in name for name in store.rows))

    def test_option_requires_compact_reads(self):
        with self.assertRaisesRegex(ValueError,'requires_compact_reads'):
            SequencePort('http://controller.invalid','secret','boot',None,
                         initial_full_output_snapshot=True)
