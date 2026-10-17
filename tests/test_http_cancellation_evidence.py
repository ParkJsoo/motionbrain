"""Cancellation inside an HTTP read retains the attempt and observed bytes."""
import base64
import hashlib
import json
import sys
import unittest
from unittest.mock import Mock,patch

from learning.controller_latency import direct_get,collect as latency_collect
from learning.serial_m4_probe import collect as serial_collect,audit as serial_audit
from learning.mac_controller_probe import run_probe,audit_probe
from tests import test_mac_controller_probe as fixture


class HTTPCancellationTest(unittest.TestCase):
    def setUp(self):
        self.f=fixture.ProbeTest();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.response=Mock(status=200);self.response.read1.side_effect=[b'{"state":',KeyboardInterrupt()]
        self.connection=Mock();self.connection.getresponse.return_value=self.response
        self.patch=patch('learning.controller_latency.http.client.HTTPConnection',return_value=self.connection)
        self.patch.start();self.addCleanup(self.patch.stop)
    def test_transport_returns_cancelled_receipt_with_raw_prefix(self):
        r=direct_get('http://192.0.2.1','status',clock=self.f.clock)
        self.assertTrue(r['cancelled']);self.assertFalse(r['transportCompleted'])
        self.assertEqual(base64.b64decode(r['responseBodyBase64']),b'{"state":')
        self.assertEqual(r['bodySha256'],hashlib.sha256(b'{"state":').hexdigest())
        self.connection.close.assert_called_once()
    def test_serial_collector_saves_before_stopping_no_port_open(self):
        opener=Mock()
        r=serial_collect(self.f.root,'http://192.0.2.1','fake','fake',open_port=opener,clock=self.f.clock)
        self.assertTrue(r['cancelled']);self.assertEqual((r['httpCount'],r['serialCount']),(1,0))
        opener.assert_not_called();self.assertTrue(serial_audit(self.f.root)['ok'])
        row=json.loads((self.f.root/'http/01.json').read_text());self.assertTrue(row['cancelled'])
        self.assertEqual(row['responseBody'],'{"state":')
    def test_camera_http_collector_seals_cancelled_attempt_before_launch(self):
        r=run_probe(self.f.root,controller_url='http://192.0.2.1',helper=sys.executable,
                    device_id='fake',view_id='new-view',read=direct_get,launch=self.f.launch,
                    stop=self.f.stop,clock=self.f.clock)
        self.assertTrue(r['cancelled']);self.assertEqual(r['requestCount'],1)
        self.f.launch.assert_not_called();self.assertTrue(audit_probe(self.f.root)['ok'])
        self.assertTrue((self.f.root/'controller/01-status.json').exists())
    def test_latency_collector_seals_cancelled_attempt_and_stops(self):
        r=latency_collect(self.f.root,'http://192.0.2.1',clock=self.f.clock)
        self.assertTrue(r['cancelled']);self.assertEqual(r['requestCount'],1)
        self.assertEqual(r['rows'][0]['responseBody'],'{"state":')
        self.assertTrue((self.f.root/'events/00000001.json').exists())

    def test_serial_diagnostic_cancellation_preserves_separate_denominator(self):
        from tests.test_compact_m4_probe import CompactSerial
        device=CompactSerial(self.f.clock)
        def http(url,endpoint,*,clock):
            if endpoint=='m4_diagnostics':return direct_get(url,endpoint,clock=clock)
            return self.f.read(url,endpoint,clock=clock)
        r=serial_collect(self.f.root,'http://192.0.2.1','fake','fake',open_port=lambda *_:device,
                         http=http,clock=self.f.clock,compact_sample=True,firmware_timing=True)
        self.assertTrue(r['cancelled']);self.assertEqual((r['httpCount'],r['diagnosticCount'],r['serialCount']),(4,1,0))
        self.assertTrue(device.closed);self.assertTrue(serial_audit(self.f.root)['ok'])
        self.assertTrue(json.loads((self.f.root/'firmware/before.json').read_text())['cancelled'])


if __name__=='__main__':unittest.main()
