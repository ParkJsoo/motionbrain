"""Explicit single-HTTP output drains preserve active output audits and budgets."""
import copy
import unittest

from tests import test_supervised_transport as fixture
from tests.test_supervised_transport import Response, pulse


class FullOutputSnapshotTests(unittest.TestCase):
    def setup_port(self):
        port,device,store=fixture.CompactTransportTests().setup_port()
        port.full_output_snapshot=True
        original=port.opener.open
        seen=[]
        def opened(request,timeout):
            if request.full_url.endswith('/action_outputs'):
                seen.append(timeout);device.clock.sleep(.45)
                device.requests.append(('/action_outputs',None))
                retained=copy.deepcopy(device.entries[-128:])
                return Response(dict(bootId=device.boot,evidence='driver_write',
                    clockDomain='controller_boot_millis',dropped=retained[0]['sequence']-1,
                    entries=retained))
            return original(request,timeout)
        port.opener.open=opened
        return port,device,store,seen

    def armed_port(self):
        port,device,store,seen=self.setup_port()
        port.preflight();device.requests.clear();seen.clear()
        port._active_pulse=pulse('elbow','up',70,1000)
        port.may_have_run=port.writes_possible=True
        return port,device,store,seen

    def test_active_seventeen_entry_ramp_drains_once_inside_same_budget(self):
        port,device,store,seen=self.armed_port()
        for pwm in range(10,180,10):device.append(3,-pwm)
        before=device.clock.monotonic()
        output,new=port._outputs()
        self.assertEqual(len(new),17)
        self.assertEqual(port._trace,new)
        self.assertEqual(port.last_outputs[3],-170)
        self.assertEqual(seen,[.8])
        self.assertLess(device.clock.monotonic()-before,.8)
        self.assertEqual(device.requests,[('/action_outputs',None)])
        self.assertFalse(any('attempt-2' in name for name in store.rows))

    def test_wrong_motor_reverse_excess_and_sequence_gap_rejected(self):
        for mode in ('wrong_motor','reverse','excess','gap'):
            with self.subTest(mode=mode):
                port,device,_,_=self.armed_port()
                if mode=='wrong_motor':device.append(2,-10)
                elif mode=='reverse':device.append(3,10)
                elif mode=='excess':device.append(3,-200)
                else:
                    for _ in range(129):device.append(3,0)
                with self.assertRaisesRegex(ValueError,'unexpected_driver_output|output_history_gap'):
                    port._outputs()
                self.assertEqual(device.requests,[('/action_outputs',None)])

    def test_quiet_new_nonzero_and_mutated_known_history_rejected(self):
        for mode in ('quiet','mutated'):
            with self.subTest(mode=mode):
                port,device,_,_=self.setup_port()
                port.preflight()
                if mode=='quiet':device.append(3,-10);device.append(3,0)
                else:device.entries[0]['reason']='changed'
                with self.assertRaisesRegex(ValueError,'unexpected_quiet_output|output_history_changed'):
                    port._outputs(quiet=True)

    def test_active_full_read_timeout_keeps_point_eight_no_retry(self):
        port,device,store,seen=self.armed_port()
        original=port.opener.open
        def delayed(request,timeout):
            device.clock.sleep(.4)
            return original(request,timeout)
        port.opener.open=delayed
        with self.assertRaisesRegex(TimeoutError,'request_deadline'):port._outputs()
        self.assertEqual(seen,[.8])
        self.assertFalse(any('attempt-2' in name for name in store.rows))

    def test_constructor_rejects_noncompact_or_nonboolean_optin(self):
        from learning.supervised_transport import SequencePort
        for compact,option in [(False,True),(True,1)]:
            with self.assertRaisesRegex(ValueError,'full_output_snapshot_requires_compact_reads'):
                SequencePort('http://controller.invalid','secret','boot',None,
                             compact_reads=compact,full_output_snapshot=option)
