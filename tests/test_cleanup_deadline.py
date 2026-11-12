"""정지 전송 기한과 정지 이후 확인 기한을 독립 검증한다."""
import unittest
from tests import test_supervised_transport as fixtures


class CleanupDeadlineTests(unittest.TestCase):
    def test_slow_post_stop_read_gets_own_budget_and_stop_remains_immediate(self):
        port, device, _ = fixtures.TransportTests().setup_port()
        original = device.open
        calls=[];stopped=False
        def open_request(request, timeout):
            nonlocal stopped
            is_stop=request.data==b'cmd=stop'
            if is_stop: stopped=True
            calls.append((request.full_url,request.data,timeout,stopped,is_stop))
            if stopped and not is_stop:
                self.assertTrue(.9 < timeout <= 2)
                device.clock.sleep(.9)
            return original(request,timeout)
        port.opener.open=open_request
        receipt=port.execute(fixtures.pulse(),lambda:None)
        self.assertTrue(receipt['offVerified'])
        stops=[c for c in calls if c[-1]]
        self.assertEqual(len(stops),1)
        self.assertTrue(0 < stops[0][2] <= .8)
        self.assertTrue(all(0 < c[2] <= .8 for c in calls if not c[3]))
        self.assertEqual(port.timeout,.8)
    def test_failed_stop_and_cleanup_are_not_retried_and_restore_budget(self):
        port, device, _ = fixtures.TransportTests().setup_port()
        calls=[]
        def fail(request,timeout):
            calls.append((request.full_url,request.data,timeout))
            raise TimeoutError('lost response')
        port.opener.open=fail
        self.assertFalse(port._stop_and_clear(restore_default=True,emergency=True))
        self.assertEqual(port.timeout,.8)
        self.assertEqual([c[1] for c in calls],[b'cmd=stop',None,b'action=clear',b'action=default&speed=100',None])
        self.assertTrue(calls[0][2]<=.8)
        self.assertTrue(all(0<c[2]<=2 for c in calls[1:]))
        self.assertFalse(port.off_verified)
