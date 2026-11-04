#!/usr/bin/env python3
"""Local attended collection console. Starting the server never starts capture."""
from __future__ import annotations

import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
from pathlib import Path
import secrets
import signal
import struct
import sys
import urllib.parse
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.artifacts import _read
from learning.lossless_frame import decode
from learning.supervised_collection import PROFILES


def strict_json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate_json_key')
            result[key] = value
        return result
    def invalid(value):
        raise ValueError('nonfinite_json')
    value = json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid)
    if not isinstance(value, dict):
        raise ValueError('json_object_required')
    return value


def frame_png(manager, session_id, index):
    """Verify original persisted bytes before producing a disposable PNG view."""
    session = manager.session(session_id)
    if type(index) is not int or not 0 <= index < len(session['frames']):
        raise ValueError('frame_not_available')
    frame = session['frames'][index]
    if frame['frameIndex'] != index:
        raise ValueError('frame_index_mismatch')
    camera_root = manager.root / session_id / 'native-camera'
    meta_raw = _read(camera_root, frame['metadataFile'])
    if hashlib.sha256(meta_raw).hexdigest() != frame['metadataSha256']:
        raise ValueError('frame_metadata_changed')
    meta = strict_json(meta_raw)
    stored = _read(camera_root, frame['imageFile'])
    if hashlib.sha256(stored).hexdigest() != frame['imageSha256']:
        raise ValueError('frame_image_changed')
    width, height = meta['width'], meta['height']
    if (type(width) is not int or type(height) is not int
            or not 1 <= width <= 1920 or not 1 <= height <= 1080
            or meta['rawByteLength'] != width * height * 4):
        raise ValueError('frame_dimensions_invalid')
    raw = decode(stored, meta['rawByteLength'], meta['pixelSha256'])
    # Native source is BGRA. PNG is RGB, no optional imaging dependency.
    rgb = bytearray(width * height * 3)
    rgb[0::3], rgb[1::3], rgb[2::3] = raw[2::4], raw[1::4], raw[0::4]
    rows = b''.join(b'\x00' + rgb[y*width*3:(y+1)*width*3] for y in range(height))
    def chunk(kind, data):
        return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!IIBBBBB', width, height, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(rows, 1)) + chunk(b'IEND', b''))


class CollectionServer(HTTPServer):
    allow_reuse_address = False

    def __init__(self, address, manager):
        if address[0] != '127.0.0.1':
            raise ValueError('loopback_only')
        self.manager = manager
        self.ui_token = secrets.token_urlsafe(32)
        super().__init__(address, CollectionHandler)

    def service_actions(self):
        self.manager.tick()


class CollectionHandler(BaseHTTPRequestHandler):
    # Single main-thread handler preserves SequencePort's hard SIGALRM deadlines.
    # Each motion is finite; a queued stop is served after that pulse completes.
    timeout = 2

    def log_message(self, *_):
        pass

    def reply(self, code, value, content_type='application/json; charset=utf-8'):
        body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' blob:; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
        self.send_header('Connection', 'close')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def local_request(self):
        port = self.server.server_port
        allowed = {f'127.0.0.1:{port}', f'localhost:{port}'}
        host = self.headers.get('Host', '')
        if host not in allowed:
            raise ValueError('local_host_required')
        origin = self.headers.get('Origin')
        if origin is not None and origin != 'http://' + host:
            raise ValueError('same_origin_required')
        if self.headers.get('Sec-Fetch-Site') not in (None, 'none', 'same-origin'):
            raise ValueError('cross_site_request_rejected')

    def do_GET(self):
        try:
            self.local_request()
        except ValueError as exc:
            self.reply(403, {'error': str(exc)})
            return
        target = urllib.parse.urlsplit(self.path)
        query = urllib.parse.parse_qs(target.query)
        manager = self.server.manager
        try:
            if target.path == '/':
                return self.reply(200, Path(__file__).with_name('manual_collection.html').read_bytes(), 'text/html; charset=utf-8')
            if target.path == '/api/config':
                return self.reply(200, {'token': self.server.ui_token, 'profiles': PROFILES, 'maxSessionSeconds': 60})
            if target.path == '/api/state':
                return self.reply(200, manager.status())
            if target.path == '/api/sessions':
                return self.reply(200, manager.sessions())
            if target.path == '/api/session':
                return self.reply(200, manager.session(query.get('id', [''])[0]))
            if target.path == '/api/frame':
                session = query.get('session', [''])[0]
                index = int(query.get('index', ['-1'])[0])
                # Do not cache verification across requests: replay detects changed evidence.
                return self.reply(200, frame_png(manager, session, index), 'image/png')
            self.reply(404, {'error': 'not_found'})
        except (ValueError, KeyError, OSError, RuntimeError) as exc:
            self.reply(400, {'error': str(exc) if isinstance(exc, ValueError) else 'artifact_unavailable'})

    def do_POST(self):
        try:
            self.local_request()
            if not secrets.compare_digest(self.headers.get('X-Collection-Token', ''), self.server.ui_token):
                raise ValueError('collection_token_required')
        except ValueError as exc:
            self.reply(403, {'error': str(exc)})
            return
        try:
            if self.headers.get('Transfer-Encoding') or self.headers.get_content_type() != 'application/json':
                raise ValueError('json_content_type_required')
            length = int(self.headers.get('Content-Length', '-1'))
            if not 0 < length <= 8192:
                raise ValueError('bounded_body_required')
            payload = strict_json(self.rfile.read(length))
            manager = self.server.manager
            route = urllib.parse.urlsplit(self.path).path
            if route == '/api/start':
                result = manager.start(payload)
            elif route == '/api/pulse':
                result = manager.pulse(payload)
            elif route in ('/api/finish', '/api/stop'):
                if payload:
                    raise ValueError('empty_body_required')
                result = manager.finish() if route == '/api/finish' else manager.stop()
            else:
                return self.reply(404, {'error': 'not_found'})
            self.reply(200, result)
        except (ValueError, RuntimeError, OSError) as exc:
            self.reply(409, {'error': str(exc) if isinstance(exc, ValueError) else 'operation_failed_preserve_evidence',
                             'state': self.server.manager.status(), 'automaticRetry': False})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-url', required=True)
    parser.add_argument('--device-id', required=True)
    parser.add_argument('--helper', type=Path, required=True)
    parser.add_argument('--view-id', default='manual')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--output-dir', type=Path, default=ROOT/'.local/manual-collection')
    args = parser.parse_args(argv)
    output = args.output_dir.resolve()
    if not output.is_relative_to((ROOT/'.local').resolve()):
        parser.error('output-dir must be inside repository .local/')
    token = os.environ.get('MOTIONBRAIN_HTTP_TOKEN', '')
    from learning.manual_collection import ManualCollection
    manager = ManualCollection(output, args.helper, args.controller_url, token, args.device_id, args.view_id)
    server = None
    def terminate(signum, frame):
        # Run cleanup on the owning main thread; never dispatch motion on restart.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt()
    previous_term = signal.signal(signal.SIGTERM, terminate)
    try:
        server = CollectionServer(('127.0.0.1', args.port), manager)
        print(f'수집 화면: http://127.0.0.1:{server.server_port} (수집 시작 전에는 촬영·구동하지 않습니다)', flush=True)
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            manager.close()
        finally:
            signal.signal(signal.SIGTERM, previous_term)
            if server:
                server.server_close()


if __name__ == '__main__':
    main()
