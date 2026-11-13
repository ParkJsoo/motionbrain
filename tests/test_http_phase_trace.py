"""한 요청의 phase 구분·기한·비밀 비기록·메모리 상한을 로컬 서버로 검사한다."""
import http.server
import json
import socket
import sys
import threading
import time
import unittest
from unittest.mock import patch
import urllib.request
import urllib.error

from learning.controller_latency import deadline
from learning.device_adapter import _NoRedirect
from learning.http_phase_trace import HttpPhaseTrace, validate_initial_phase_report
from learning.supervised_transport import SequencePort
from tests.test_supervised_transport import Store


class LocalServer:
    def __init__(self, mode='normal'):
        self.mode = mode
        self.requests = 0
        self.accepted = threading.Event()
        self.release = threading.Event()
        owner = self

        class Handler(http.server.BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def log_message(self, *args): pass

            def do_POST(self):
                self.rfile.read(int(self.headers.get('Content-Length',0)))
                self.do_GET()

            def do_GET(self):
                owner.requests += 1
                owner.accepted.set()
                if owner.mode == 'headers':
                    owner.release.wait(3)
                    return
                body = (b'{"bootId":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","success":true,"detail":"private-response-body"}'
                        if owner.mode == 'json' else
                        b'private-response-body' if owner.mode == 'normal' else b'x'*512)
                self.send_response(200)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('X-Private-Header', 'private-response-header')
                self.send_header('Connection', 'close')
                self.end_headers()
                if owner.mode == 'body':
                    owner.release.wait(3)
                    return
                try:
                    if owner.mode == 'drip':
                        for value in body:
                            self.wfile.write(bytes([value])); self.wfile.flush()
                            if owner.release.wait(.04): break
                    else:
                        self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever,
                                       kwargs=dict(poll_interval=.01), daemon=True)
        self.thread.start()
        self.url = 'http://127.0.0.1:'+str(self.server.server_port)

    def close(self):
        self.release.set()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(2)


class HttpPhaseTraceTests(unittest.TestCase):
    def opener(self, trace):
        return urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect(), trace.handler())

    def server(self, mode='normal'):
        server = LocalServer(mode)
        self.addCleanup(server.close)
        return server

    def test_constructor_has_no_IO_and_limits_configuration(self):
        with patch('socket.socket', side_effect=AssertionError('unexpected socket')), \
                patch('socket.getaddrinfo', side_effect=AssertionError('unexpected DNS')):
            trace = HttpPhaseTrace(include_tcp_info=True)
            trace.handler()
            self.assertEqual(trace.events, [])
            self.assertEqual(trace.phase, 'not_started')
        for value in (0, 129, -1, True, 1.5):
            with self.subTest(value=value), self.assertRaises(ValueError): HttpPhaseTrace(max_events=value)
        with self.assertRaises(ValueError): HttpPhaseTrace(include_tcp_info=1)

    def test_nested_send_error_preserves_original_connect_failure_phase(self):
        trace=HttpPhaseTrace()
        with patch('socket.create_connection',side_effect=TimeoutError('private-connection-message')) as connect:
            with self.assertRaises(urllib.error.URLError):
                self.opener(trace).open('http://connection.invalid',timeout=2)
        self.assertEqual(connect.call_count,1)
        self.assertEqual(trace.phase,'connect')
        self.assertTrue(any(e['phase']=='connect' and e['event']=='error' for e in trace.events))
        self.assertTrue(any(e['phase']=='send' and e['event']=='error' for e in trace.events))
        self.assertNotIn('private-connection-message',json.dumps(trace.events))

    def test_normal_request_keeps_payload_and_headers_out_of_trace_and_snapshot_is_detached(self):
        server = self.server()
        trace = HttpPhaseTrace()
        request = urllib.request.Request(server.url+'/private-url-fragment', headers={
            'Authorization':'Bearer private-authentication', 'X-Private':'private-request-header'})
        with self.opener(trace).open(request, timeout=2) as response:
            self.assertEqual(response.read(), b'private-response-body')
        self.assertEqual(server.requests, 1)
        events = trace.events
        for phase in ('connect', 'send', 'headers', 'body'):
            self.assertTrue(any(e['phase']==phase and e['event']=='begin' for e in events))
            self.assertTrue(any(e['phase']==phase and e['event']=='end' for e in events))
        self.assertTrue(all(b['at']>=a['at'] for a,b in zip(events,events[1:])))
        self.assertFalse(any(e['event']=='error' for e in events))
        self.assertEqual(trace.phase, 'body')
        serialized=json.dumps(events)
        for private in ('private-authentication','private-request-header','private-response-header',
                        'private-response-body','private-url-fragment','Authorization'):
            self.assertNotIn(private, serialized)
        events[0]['phase']='tampered';events.append({})
        self.assertNotEqual(trace.events[0]['phase'],'tampered')
        self.assertNotEqual(len(trace.events),len(events))

    def assert_stall_phase(self, mode, expected_phase):
        server = self.server(mode)
        trace = HttpPhaseTrace()
        began = time.monotonic()
        with self.assertRaises(TimeoutError):
            # Socket timeout stays 2s. The existing caller alarm alone ends this request.
            with deadline(.15):
                with self.opener(trace).open(server.url, timeout=2) as response:
                    response.read()
        elapsed = time.monotonic()-began
        self.assertGreaterEqual(elapsed, .10)
        self.assertLess(elapsed, .65)
        self.assertTrue(server.accepted.is_set())
        self.assertEqual(server.requests, 1)
        self.assertEqual(trace.phase, expected_phase)
        self.assertTrue(any(e['phase']==expected_phase and e['event']=='error'
                            and e['errorType']=='TimeoutError' for e in trace.events))
        return trace

    def test_header_stall_preserves_original_absolute_deadline(self):
        self.assert_stall_phase('headers', 'headers')

    def test_sequence_port_initial_two_second_deadline_is_unchanged(self):
        server=self.server('headers')
        trace=HttpPhaseTrace()
        store=Store()
        port=SequencePort(server.url,'private-runtime-token','discovering',store,timeout=2.)
        port.opener=self.opener(trace)
        began=time.monotonic()
        with self.assertRaises(TimeoutError):port._call('/actions')
        elapsed=time.monotonic()-began
        self.assertGreaterEqual(elapsed,1.85)
        self.assertLess(elapsed,2.65)
        self.assertEqual(port.serial,1)
        self.assertEqual(server.requests,1)
        self.assertEqual(store.rows['http-000001-intent.json']['timeoutSeconds'],2.)
        self.assertEqual(store.rows['http-000001-error.json']['error'],'TimeoutError')
        self.assertEqual(trace.phase,'headers')
        self.assertNotIn('private-runtime-token',json.dumps(trace.events))

    def test_default_initial_actions_GET_automatically_saves_phase_and_keeps_original_opener(self):
        server=self.server('json');store=Store()
        port=SequencePort(server.url,'private-runtime-token','discovering',store,timeout=2.)
        original=port.opener
        self.assertEqual(port._call('/actions')['bootId'],'a'*32)
        self.assertIs(port.opener,original)
        self.assertEqual(server.requests,1)
        proof=store.rows['http-000001-phase.json']
        self.assertEqual(set(proof),{'phase','events','elapsedSeconds'})
        validate_initial_phase_report(proof,store.rows['http-000001-intent.json'],
                                      store.rows['http-000001-response.json'])
        self.assertEqual(proof['phase'],'body')
        self.assertGreaterEqual(proof['elapsedSeconds'],0.)
        self.assertTrue(any(e['phase']=='connect' and e['event']=='end' for e in proof['events']))
        for private in ('private-runtime-token','private-response-body','private-response-header',
                        'rawUtf8','parsed','/actions','Authorization'):
            self.assertNotIn(private,json.dumps(proof))
        self.assertEqual(port._call('/actions')['bootId'],'a'*32)
        self.assertNotIn('http-000002-phase.json',store.rows)
        self.assertEqual(server.requests,2)

    def test_default_initial_actions_header_timeout_saves_phase_with_original_two_seconds(self):
        server=self.server('headers');store=Store()
        port=SequencePort(server.url,'private-runtime-token','discovering',store,timeout=2.)
        began=time.monotonic()
        with self.assertRaises(TimeoutError):port._call('/actions')
        elapsed=time.monotonic()-began
        self.assertGreaterEqual(elapsed,1.85);self.assertLess(elapsed,2.65)
        self.assertEqual(server.requests,1)
        self.assertEqual(store.rows['http-000001-intent.json']['timeoutSeconds'],2.)
        proof=store.rows['http-000001-phase.json']
        self.assertEqual(proof['phase'],'headers')
        self.assertTrue(any(e['event']=='error' and e['errorType']=='TimeoutError' for e in proof['events']))
        self.assertNotIn('private-runtime-token',json.dumps(proof))

    def test_first_non_actions_lookup_and_motor_HTTP_are_not_automatically_traced(self):
        server=self.server('json')
        for endpoint,payload,run in [('/status',None,False),('/sequence',{'action':'run'},True)]:
            with self.subTest(endpoint=endpoint):
                store=Store();port=SequencePort(server.url,'private-token','discovering',store,timeout=2.)
                with patch('learning.supervised_transport.HttpPhaseTrace',side_effect=AssertionError('unexpected trace')):
                    self.assertTrue(port._call(endpoint,payload,run=run)['success'])
                self.assertFalse(any(name.endswith('-phase.json') for name in store.rows))

    def test_injected_fake_opener_is_preserved_and_does_not_create_a_trace(self):
        store=Store();port=SequencePort('http://fake.invalid','private-token','discovering',store,timeout=2.)
        class Response:
            status=200
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def read(self,maximum):return b'{"bootId":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"}'
        class Fake:
            def __init__(self):self.calls=0
            def open(self,request,timeout):self.calls+=1;return Response()
        fake=Fake();port.opener=fake
        with patch('learning.supervised_transport.HttpPhaseTrace',side_effect=AssertionError('unexpected trace')), \
                patch('learning.supervised_transport.urllib.request.build_opener',side_effect=AssertionError('fake bypass')):
            self.assertEqual(port._call('/actions')['bootId'],'a'*32)
        self.assertIs(port.opener,fake);self.assertEqual(fake.calls,1)
        self.assertNotIn('http-000001-phase.json',store.rows)

    def test_phase_persistence_failure_aborts_successful_startup_but_preserves_existing_timeout(self):
        class BrokenPhaseStore(Store):
            def write_json(self,name,value):
                if name.endswith('-phase.json'):raise OSError('phase disk failed')
                return super().write_json(name,value)
        for mode,expected in [('json',OSError),('headers',TimeoutError)]:
            with self.subTest(mode=mode):
                server=self.server(mode);store=BrokenPhaseStore()
                port=SequencePort(server.url,'private-token','discovering',store,timeout=.15)
                with self.assertRaises(expected):port._call('/actions')
                self.assertTrue(port.evidence_failed)
                self.assertFalse(port.may_have_run);self.assertFalse(port.writes_possible)
                self.assertEqual(port._last_http_failure_serial,1)
                self.assertEqual(server.requests,1)

    def test_body_stall_preserves_original_absolute_deadline(self):
        self.assert_stall_phase('body', 'body')

    def test_dripping_body_cannot_restart_callers_absolute_deadline(self):
        self.assert_stall_phase('drip', 'body')

    def test_single_trace_never_retries_or_opens_a_second_request(self):
        server = self.server()
        trace = HttpPhaseTrace()
        opener = self.opener(trace)
        with opener.open(server.url,timeout=2) as response: response.read()
        with self.assertRaisesRegex(ValueError,'single_request'):
            self.opener(trace).open(server.url,timeout=2)
        self.assertEqual(server.requests,1)

    def test_event_budget_caps_repeated_reads_and_keeps_error_phase_after_overflow(self):
        server = self.server()
        trace = HttpPhaseTrace(max_events=4)
        with self.opener(trace).open(server.url,timeout=2) as response:
            while response.read(1): pass
        self.assertEqual(len(trace.events),4)
        self.assertGreater(trace.dropped_events,20)
        self.assertLess(len(json.dumps(trace.events)),4096)
        self.assertEqual(trace.phase,'body')
        # Even when all later events are discarded, first failure classification is kept.
        trace = HttpPhaseTrace(max_events=1)
        server = self.server('body')
        with self.assertRaises(TimeoutError):
            with deadline(.15):
                with self.opener(trace).open(server.url,timeout=2) as response: response.read()
        self.assertEqual(len(trace.events),1)
        self.assertEqual(trace.phase,'body')
        self.assertGreater(trace.dropped_events,0)

    def test_optin_TCP_metadata_uses_existing_connection_without_extra_socket_probe(self):
        server=self.server()
        trace=HttpPhaseTrace(include_tcp_info=True)
        original=socket.create_connection
        with patch('socket.create_connection',wraps=original) as connect:
            with self.opener(trace).open(server.url,timeout=2) as response:response.read()
        self.assertEqual(connect.call_count,1)
        self.assertEqual(server.requests,1)
        metadata=[e['socket'] for e in trace.events if 'socket' in e]
        self.assertTrue(metadata)
        self.assertTrue(all(item['family']==int(socket.AF_INET) for item in metadata))
        if sys.platform=='darwin':
            self.assertTrue(any('tcp' in item for item in metadata))
            for item in metadata:
                if 'tcp' in item:self.assertTrue(all(type(v) is int for v in item['tcp'].values()))

    def test_optional_socket_metadata_does_not_swallow_callers_deadline(self):
        trace=HttpPhaseTrace(include_tcp_info=True)
        sock=type('Socket',(),dict(family=socket.AF_INET,type=socket.SOCK_STREAM,
            gettimeout=lambda self:2.,getsockopt=lambda *args:(_ for _ in ()).throw(TimeoutError('request_deadline'))))()
        with patch('learning.http_phase_trace.sys.platform','darwin'):
            with self.assertRaises(TimeoutError): trace._socket_metadata(sock)


if __name__=='__main__':unittest.main()
