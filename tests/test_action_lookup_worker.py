import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Event, Thread
import time
import unittest
from unittest.mock import Mock
import socket
from types import SimpleNamespace

from learning.action_lookup_worker import ActionLookupWorker, _lookup, _Pending
from learning.device_adapter import DeviceReadError


class ActionLookupWorkerTest(unittest.TestCase):
    def test_worker_warms_and_closes_without_network(self):
        worker = ActionLookupWorker('http://unused.invalid')
        with worker:
            self.assertIsNotNone(worker.process)
            with self.assertRaisesRegex(ValueError, 'already_started'):
                worker.__enter__()
        with self.assertRaisesRegex(ValueError, 'not_ready'):
            with worker.reserve({'bootId':'a'*32,'commandId':'offline'}, 'http://unused.invalid'):
                self.fail('closed worker admitted a command')
        with self.assertRaisesRegex(ValueError, 'timeout'):
            ActionLookupWorker('http://unused.invalid', timeout=10)

    def test_worker_reads_only_one_named_action(self):
        from test_action_execution import FakeClock, FakePort, request
        req = request()
        record = FakePort(FakeClock()).submit(req)['record']
        reader = Mock()
        reader.lookup_action.return_value = dict(lookupStatus='matched_record',record=dict(record,noise='x'*10000))
        self.assertEqual(_lookup(reader, req, .16), record)
        reader.lookup_action.assert_called_once_with(req, propagate_transport=True)
        reader.lookup_action.return_value = {'lookupStatus':'unverified'}
        with self.assertRaises(DeviceReadError):
            _lookup(reader, req, .16)
        with self.assertRaises(ValueError):
            ActionLookupWorker('http://user@host/path')

    def test_pending_datagram_read_cannot_block_or_accept_worker_death(self):
        receiver, sender = socket.socketpair(type=socket.SOCK_DGRAM)
        with receiver, sender:
            receiver.setblocking(False)
            process = Mock()
            process.is_alive.return_value=True
            pending = _Pending(SimpleNamespace(channel=receiver,process=process),('a'*32,'id'))
            self.assertFalse(pending.done())
            with self.assertRaises(BlockingIOError):
                pending.result(timeout=0)
            process.is_alive.return_value=False
            self.assertTrue(pending.done())
            with self.assertRaises(DeviceReadError):
                pending.result(timeout=0)

    def test_obsolete_get_is_drained_before_next_command(self):
        from test_action_execution import FakeClock, FakePort, request
        entered, release = Event(), Event()
        req = request()
        record = FakePort(FakeClock()).submit(req)['record']
        paths = []
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass
            def do_GET(self):
                paths.append(self.path)
                entered.set()
                if not release.wait(1):
                    return
                body = json.dumps(record).encode()
                self.send_response(200)
                self.send_header('Content-Type','application/json')
                self.send_header('Content-Length',str(len(body)))
                self.end_headers()
                self.wfile.write(body)
        with ThreadingHTTPServer(('127.0.0.1',0), Handler) as server:
            thread = Thread(target=server.serve_forever,daemon=True)
            thread.start()
            try:
                origin = 'http://127.0.0.1:' + str(server.server_port)
                with ActionLookupWorker(origin,timeout=.2) as worker:
                    with worker.reserve(req,origin) as lease:
                        lease.check_ready(origin)
                        pending = lease.begin()
                        self.assertTrue(entered.wait(1))
                        with self.assertRaisesRegex(ValueError,'reserved'):
                            worker.__exit__(None,None,None)
                    with self.assertRaisesRegex(ValueError,'in_flight'):
                        with worker.reserve(request('second'),origin):
                            self.fail('old GET was still in flight')
                    release.set()
                    until=time.monotonic()+1
                    while not pending.done() and time.monotonic()<until:
                        time.sleep(.002)
                    self.assertTrue(pending.done())
                    with worker.reserve(request('second'),origin) as lease:
                        lease.check_ready(origin)
                self.assertEqual(paths,['/actions?bootId='+req['bootId']+'&commandId=first'])
            finally:
                release.set()
                server.shutdown()
                thread.join()
