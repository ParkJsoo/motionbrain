"""Read HTTP timing from an actual loopback socket, including an HTTP error."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.error
import urllib.request

from tools.record_controller_ui import make_server


class Upstream(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(409)
        self.send_header("X-MB-Handler-Ms", "4294967295")
        self.send_header("X-MB-LeaseCheck-Ms", "0")
        self.send_header("X-MB-WorkDone-Ms", "invalid")
        self.end_headers()
        self.wfile.write(b"upstream response")

    def log_message(self, *args):
        pass


class ControllerTiming(unittest.TestCase):
    def serve(self, server):
        self.addCleanup(server.server_close)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        self.addCleanup(thread.join)
        self.addCleanup(server.shutdown)
        return f"http://127.0.0.1:{server.server_port}"

    def test_409_body_and_per_request_clock_domains(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        log_dir = Path(temp.name) / "log"
        upstream = self.serve(ThreadingHTTPServer(("127.0.0.1", 0), Upstream))
        proxy = make_server(upstream, log_dir, 0)
        self.addCleanup(proxy.capture_log.close)
        url = self.serve(proxy)
        with self.assertRaises(urllib.error.HTTPError) as result:
            urllib.request.urlopen(url + "/manual_leases", timeout=3)
        with result.exception as response:
            self.assertEqual(response.code, 409)
            self.assertEqual(response.read(), b"upstream response")
        rows = [json.loads(line) for line in (log_dir / "ui-events.jsonl").read_text().splitlines()]
        row, = [r for r in rows if r["kind"] == "proxy_response"]
        self.assertEqual(row["deviceTiming"], {"HandlerMs": 4294967295, "LeaseCheckMs": 0})
        marks = [row["hostTiming"][k] for k in ("connectStart", "connected", "requestSent",
                                               "firstByteObserved", "bodyRead")]
        self.assertEqual(marks, sorted(marks))
