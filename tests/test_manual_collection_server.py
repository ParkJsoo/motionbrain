"""Loopback HTTP and replay integrity tests; no controller or camera access."""
import hashlib
import http.client
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch

from tools.manual_collection_server import CollectionServer, frame_png, strict_json


class ServerTests(unittest.TestCase):
    def setUp(self):
        self.manager = Mock()
        self.manager.status.return_value = {'phase': 'idle'}
        self.manager.start.return_value = {'phase': 'recording'}
        self.server = CollectionServer(('127.0.0.1', 0), self.manager)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': .01})
        self.thread.start()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.server.shutdown()
        self.thread.join(2)
        self.server.server_close()

    def request(self, path, *, method='GET', body=None, headers=None):
        client = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=2)
        try:
            client.request(method, path, body, headers or {})
            response = client.getresponse()
            return response.status, json.loads(response.read())
        finally:
            client.close()

    def test_open_and_read_state_never_starts_capture(self):
        self.assertEqual(self.request('/api/state'), (200, {'phase': 'idle'}))
        self.manager.start.assert_not_called()
        self.manager.pulse.assert_not_called()

    def test_start_needs_local_origin_and_separate_token(self):
        headers = {'Content-Type': 'application/json', 'X-Collection-Token': self.server.ui_token}
        self.assertEqual(self.request('/api/start', method='POST', body='{}')[0], 403)
        self.assertEqual(self.request('/api/start', method='POST', body='{}', headers={**headers, 'Origin': 'https://evil.example'})[0], 403)
        self.assertEqual(self.request('/api/start', method='POST', body='{}', headers={**headers, 'Host': 'evil.example'})[0], 403)
        self.manager.start.assert_not_called()
        self.assertEqual(self.request('/api/start', method='POST', body='{}', headers=headers)[0], 200)
        self.manager.start.assert_called_once_with({})

    def test_cross_site_cannot_read_token_or_recordings(self):
        for path in ('/api/config', '/api/sessions', '/api/frame?session=a&index=0'):
            self.assertEqual(self.request(path, headers={'Sec-Fetch-Site': 'cross-site'})[0], 403)

    def test_readonly_recovery_requires_auth_and_empty_body_then_delegates_once(self):
        self.manager.recover_readonly.return_value = {'sessionId': 'failed', 'offVerified': True, 'noWrites': True}
        headers = {'Content-Type': 'application/json', 'X-Collection-Token': self.server.ui_token}
        self.assertEqual(self.request('/api/recover-readonly', method='POST', body='{}')[0], 403)
        self.assertEqual(self.request('/api/recover-readonly', method='POST', body='{}',
                                     headers={**headers, 'Origin': 'https://evil.example'})[0], 403)
        self.assertEqual(self.request('/api/recover-readonly', method='POST', body='{"sessionId":"other"}', headers=headers)[0], 409)
        self.manager.recover_readonly.assert_not_called()
        code, result = self.request('/api/recover-readonly', method='POST', body='{}', headers=headers)
        self.assertEqual((code, result['noWrites']), (200, True))
        self.manager.recover_readonly.assert_called_once_with()
        self.manager.start.assert_not_called()
        self.manager.pulse.assert_not_called()

    def test_bad_payload_and_unknown_routes_never_dispatch(self):
        headers = {'Content-Type': 'application/json', 'X-Collection-Token': self.server.ui_token}
        for payload in ('{"joint":1,"joint":2}', '{"value":NaN}', '[]', 'x'*8193):
            self.assertEqual(self.request('/api/pulse', method='POST', body=payload, headers=headers)[0], 409)
        self.assertEqual(self.request('/api/unknown', method='POST', body='{}', headers=headers)[0], 404)
        self.manager.pulse.assert_not_called()

    def test_motion_failure_is_returned_once_without_retry(self):
        self.manager.pulse.side_effect = ValueError('output_history_gap')
        code, result = self.request('/api/pulse', method='POST', body='{}', headers={
            'Content-Type': 'application/json', 'X-Collection-Token': self.server.ui_token})
        self.assertEqual(code, 409)
        self.assertFalse(result['automaticRetry'])
        self.manager.pulse.assert_called_once()

    def test_stop_and_finish_have_distinct_terminal_intent(self):
        self.manager.stop.return_value = {'phase': 'aborted'}
        headers = {'Content-Type': 'application/json', 'X-Collection-Token': self.server.ui_token}
        self.assertEqual(self.request('/api/stop', method='POST', body='{}', headers=headers)[1]['phase'], 'aborted')
        self.manager.stop.assert_called_once()
        self.manager.finish.assert_not_called()


class FrameReplayTests(unittest.TestCase):
    def test_original_hash_verified_before_decode(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            camera = root/'session'/'native-camera'
            camera.mkdir(parents=True)
            meta = json.dumps({'width': 1, 'height': 1, 'rawByteLength': 4, 'pixelSha256': 'unused'}).encode()
            (camera/'frame-0000.json').write_bytes(meta)
            (camera/'frame-0000.bgra.lzbitmap').write_bytes(b'compressed')
            frame = {'frameIndex': 0, 'metadataFile': 'frame-0000.json',
                     'metadataSha256': hashlib.sha256(meta).hexdigest(),
                     'imageFile': 'frame-0000.bgra.lzbitmap',
                     'imageSha256': hashlib.sha256(b'compressed').hexdigest()}
            manager = Mock(root=root)
            manager.session.return_value = {'frames': [frame]}
            with patch('tools.manual_collection_server.decode', return_value=bytes([0, 0, 255, 255])) as decoder:
                self.assertTrue(frame_png(manager, 'session', 0).startswith(b'\x89PNG'))
                (camera/'frame-0000.bgra.lzbitmap').write_bytes(b'corrupt')
                with self.assertRaisesRegex(ValueError, 'image_changed'):
                    frame_png(manager, 'session', 0)
                self.assertEqual(decoder.call_count, 1)
            (camera/'frame-0000.json').write_bytes(b'{}')
            with self.assertRaisesRegex(ValueError, 'metadata_changed'):
                frame_png(manager, 'session', 0)

    def test_json_duplicate_nested_and_infinity_rejected(self):
        for raw in ('{"a":{"x":1,"x":2}}', '{"a":Infinity}'):
            with self.assertRaises(ValueError):
                strict_json(raw)



class ServerRestartTests(unittest.TestCase):
    def test_clean_restart_reuses_port_but_active_listener_is_exclusive(self):
        manager = Mock()
        manager.status.return_value = {'phase': 'idle'}
        first = CollectionServer(('127.0.0.1', 0), manager)
        port = first.server_port
        thread = threading.Thread(target=first.serve_forever, kwargs={'poll_interval': .01})
        thread.start()
        try:
            with self.assertRaises(OSError):
                CollectionServer(('127.0.0.1', port), manager)
            client = http.client.HTTPConnection('127.0.0.1', port, timeout=2)
            try:
                client.request('GET', '/api/state')
                response = client.getresponse()
                self.assertEqual(response.status, 200)
                response.read()
            finally:
                client.close()
        finally:
            first.shutdown(); thread.join(2); first.server_close()
        second = CollectionServer(('127.0.0.1', port), manager)
        try:
            self.assertNotEqual(first.ui_token, second.ui_token)
            manager.start.assert_not_called()
            manager.pulse.assert_not_called()
        finally:
            second.server_close()


if __name__ == '__main__':
    unittest.main()
