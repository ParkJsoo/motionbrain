"""Loopback and synthetic tests; no controller or camera access."""
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from unittest.mock import patch

from learning.controller_latency import collect,direct_get,deadline,summarize
from learning.mac_controller_probe import _check,ProbeRejected
from tests import test_mac_controller_probe as fixture


class ComparisonTest(unittest.TestCase):
    def setUp(self):
        self.fixture=fixture.ProbeTest();self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.root=self.fixture.root
    def direct(self,*a,**kw):
        row=self.fixture.read(*a,**kw);row.pop('curlExit');row.update(transport='python_http_client',transportCompleted=True)
        return row
    def test_fixed_budget_interleaved_transports(self):
        f=self.fixture;r=collect(self.root,'http://192.0.2.1',direct=self.direct,curl=f.read,clock=f.clock)
        self.assertEqual(r['status'],'recorded');self.assertEqual(r['requestCount'],14)
        summary=summarize(r)
        self.assertEqual(summary['transports']['curl']['samples'],3)
        self.assertEqual(summary['transports']['python_http_client']['samples'],3)
        self.assertFalse(summary['trainingEligible']);self.assertFalse(summary['sensorPhysicalSampleTimeVerified'])
        self.assertTrue((self.root/'events/00000001.json').exists())
    def test_output_change_aborts(self):
        def mutate(e,v):
            if e=='action_outputs' and len(self.fixture.calls)>10:v['dropped']+=1
        self.fixture.mutate=mutate;f=self.fixture
        r=collect(self.root,'http://192.0.2.1',direct=self.direct,curl=f.read,clock=f.clock)
        self.assertEqual(r['failure'],'controller_boot_or_output_changed')
    def test_existing_path_never_restarts(self):
        f=self.fixture;collect(self.root,'http://192.0.2.1',direct=self.direct,curl=f.read,clock=f.clock)
        with self.assertRaises(Exception):collect(self.root,'http://192.0.2.1',direct=self.direct,curl=f.read,clock=f.clock)
        self.assertEqual(len(f.calls),14)
    def test_timed_out_attempt_remains_in_denominator_and_raw_prefix(self):
        f=self.fixture
        def fail(*a,**kw):raise subprocess.TimeoutExpired('curl',1.2,output=b'partial')
        r=collect(self.root,'http://192.0.2.1',direct=self.direct,curl=fail,clock=f.clock)
        self.assertEqual(r['requestCount'],5)
        self.assertEqual(r['rows'][-1]['outputBase64'],'cGFydGlhbA==')
        self.assertEqual(summarize(r)['transports']['curl']['samples'],1)
        self.assertEqual(summarize(r)['transports']['curl']['validSamples'],0)


class DirectHTTPTest(unittest.TestCase):
    def setUp(self):
        self.calls=[];self.mode='normal';outer=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*a):pass
            def do_GET(self):
                outer.calls.append((self.command,self.path))
                if outer.mode=='redirect':
                    self.send_response(302);self.send_header('Location','/arm');self.end_headers();return
                self.send_response(200);self.send_header('Content-Length','22' if outer.mode=='short' else '2');self.end_headers()
                if outer.mode=='drip':time.sleep(1.1)
                try:self.wfile.write(b'\xff\xfe' if outer.mode=='invalid_utf8' else b'{}')
                except BrokenPipeError:pass
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.server.server_close);self.addCleanup(lambda:self.thread.join(timeout=2));self.addCleanup(self.server.shutdown)
        self.url=f'http://127.0.0.1:{self.server.server_port}'
    def test_no_subprocess_get_and_phase_stamps(self):
        with patch('subprocess.run') as proc:r=direct_get(self.url,'status')
        proc.assert_not_called();self.assertTrue(r['transportCompleted'])
        self.assertEqual(self.calls,[('GET','/status')])
        stamps=[r['requestStartedMonotonic'],*r['timing'].values(),r['receivedMonotonic']]
        self.assertEqual(stamps,sorted(stamps));self.assertEqual(signal.getitimer(signal.ITIMER_REAL),(0.,0.))
    def test_control_and_query_rejected_before_connect(self):
        for e in ('arm','command','status?x=1','actions/cancel'):
            with self.assertRaises(ValueError):direct_get(self.url,e)
        self.assertEqual(self.calls,[])
    def test_redirect_never_followed(self):
        self.mode='redirect';r=direct_get(self.url,'status')
        self.assertEqual(r['httpStatus'],'302') if 'httpStatus' in r else self.assertFalse(r['transportCompleted'])
        self.assertEqual(self.calls,[('GET','/status')])
        with self.assertRaises(ProbeRejected):_check(r,'status')
    def test_drip_stops_at_total_deadline_and_restores_handler(self):
        self.mode='drip';before=signal.getsignal(signal.SIGALRM);start=time.monotonic()
        r=direct_get(self.url,'status')
        self.assertLess(time.monotonic()-start,1.0);self.assertFalse(r['transportCompleted'])
        self.assertEqual(signal.getsignal(signal.SIGALRM),before)
        self.assertEqual(len(self.calls),1)
    def test_short_body_is_not_a_valid_latency_sample(self):
        self.mode='short';r=direct_get(self.url,'status')
        self.assertFalse(r['transportCompleted']);self.assertEqual(r['responseBodyBase64'],'e30=')
        self.assertEqual(r['httpStatus'],'200')
    def test_non_utf8_body_preserved_as_bytes(self):
        self.mode='invalid_utf8';r=direct_get(self.url,'status')
        self.assertFalse(r['transportCompleted']);self.assertEqual(r['responseBodyBase64'],'//4=')
    def test_existing_alarm_not_replaced(self):
        signal.setitimer(signal.ITIMER_REAL,10)
        try:
            with self.assertRaises(ValueError):
                with deadline(.8):pass
            self.assertGreater(signal.getitimer(signal.ITIMER_REAL)[0],9)
        finally:signal.setitimer(signal.ITIMER_REAL,0)

if __name__=='__main__':unittest.main()
