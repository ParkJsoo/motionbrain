"""Attended GET-only preparation budgets never extend RUN or active reads."""
import copy
import json
import unittest
import urllib.parse

from learning.supervised_transport import SequencePort
from tests import test_supervised_transport as fixture
from tests.test_supervised_transport import Response, pulse


class PreparationReadTimeoutTests(unittest.TestCase):
    def setup_port(self, *, slow_sequence=False, slow_armed=False, slow_active=False):
        port,device,store=fixture.CompactTransportTests().setup_port()
        port.full_output_snapshot=True
        port.preparation_read_timeout=2.
        original=port.opener.open
        seen=[]
        def opened(request,timeout):
            path=urllib.parse.urlsplit(request.full_url).path
            get=request.data is None
            seen.append(dict(path=path,get=get,timeout=timeout,preparing=port._preparing_execution,
                             running=device.running,state=device.state))
            slow=(get and ((slow_sequence and path=='/sequence' and not device.running)
                  or (slow_armed and path=='/collection_status' and device.state=='ARMED' and not device.running)
                  or (slow_active and path=='/collection_status' and device.running)))
            if slow:
                if timeout < 1.1:
                    device.clock.sleep(timeout)
                    raise TimeoutError('slow_response')
                device.clock.sleep(1.1)
            if get and path=='/action_outputs':
                device.clock.sleep(.01);device.requests.append((path,None))
                retained=copy.deepcopy(device.entries[-128:])
                return Response(dict(bootId=device.boot,evidence='driver_write',clockDomain='controller_boot_millis',
                    dropped=retained[0]['sequence']-1,entries=retained))
            return original(request,timeout)
        port.opener.open=opened
        return port,device,store,seen

    def test_slow_preflight_and_armed_GETs_complete_without_extending_writes(self):
        for slow in ('slow_sequence','slow_armed'):
            with self.subTest(slow=slow):
                port,device,store,seen=self.setup_port(**{slow:True})
                result=port.execute(pulse('elbow','up',70,1000),lambda:None)
                self.assertTrue(result['offVerified'])
                self.assertFalse(port._preparing_execution)
                intents=[value for name,value in store.rows.items() if name.endswith('-intent.json')]
                active_writes=[value for value in intents if value['method']=='POST'
                    and (value['path']=='/command' or value['payload'].get('action') in ('add','run'))]
                self.assertTrue(active_writes)
                self.assertTrue(all(value['timeoutSeconds']<=.8 for value in active_writes))
                prep_gets=[row for row in seen if row['get'] and row['preparing']]
                self.assertTrue(prep_gets)
                self.assertTrue(all(row['timeout']==2. for row in prep_gets))
                active_gets=[row for row in seen if row['get'] and row['running']]
                self.assertTrue(active_gets)
                self.assertTrue(all(row['timeout']<=.8 and not row['preparing'] for row in active_gets))
                self.assertEqual(port.timeout,.8)

    def test_after_ARMED_failure_clears_flag_before_cleanup_and_never_RUNs(self):
        port,device,_,seen=self.setup_port(slow_armed=True)
        def reject():
            self.assertTrue(port._preparing_execution)
            raise ValueError('observation_rejected')
        with self.assertRaisesRegex(ValueError,'observation_rejected'):
            port.execute(pulse('elbow','up',70,1000),lambda:None,after_armed_before_run=reject)
        self.assertFalse(port._preparing_execution)
        self.assertFalse(any(payload=={'action':'run'} for _,payload in device.requests))
        stop=[row for row in seen if row['path']=='/command' and not row['get'] and not row['preparing']]
        self.assertTrue(stop)
        self.assertTrue(all(row['timeout']<=.8 for row in stop))

    def test_slow_active_GET_still_times_out_point_eight_and_stops_once(self):
        port,device,_,seen=self.setup_port(slow_active=True)
        with self.assertRaisesRegex(TimeoutError,'slow_response'):
            port.execute(pulse('elbow','up',70,1000),lambda:None)
        self.assertFalse(port._preparing_execution)
        active=[row for row in seen if row['path']=='/collection_status' and row['running']]
        self.assertEqual(len(active),1)
        self.assertEqual(active[0]['timeout'],.8)
        self.assertEqual(sum(payload=={'action':'run'} for _,payload in device.requests),1)
        self.assertEqual(sum(payload=={'cmd':'stop'} for _,payload in device.requests),1)

    def test_nonblocking_RUN_guard_sees_flag_cleared(self):
        port,_,_,_=self.setup_port()
        seen=[]
        def guard():seen.append(port._preparing_execution)
        port.execute(pulse('elbow','up',70,1000),lambda:None,before_run=guard)
        self.assertEqual(seen,[False])

    def test_option_is_explicit_bounded_and_false_without_execution_scope(self):
        for value in (True,1.,2.1,float('nan')):
            with self.assertRaisesRegex(ValueError,'must_be_two_seconds'):
                SequencePort('http://controller.invalid','secret','boot',None,preparation_read_timeout=value)
        port,_,_,_=self.setup_port()
        self.assertEqual(port._read_timeout('/sequence'),.8)
        port._preparing_execution=True
        self.assertEqual(port._read_timeout('/sequence'),2.)
        self.assertEqual(port._read_timeout('/sequence',payload={'action':'run'},run=True),.8)
        self.assertEqual(port._read_timeout('/sequence',emergency=True),.8)
        port.may_have_run=True
        self.assertEqual(port._read_timeout('/sequence'),.8)
