"""Finite native camera spool and ordered host ingress. Explicit start only."""
from __future__ import annotations

from collections import deque
import copy
import hashlib
from pathlib import Path
import subprocess
import threading
import time

from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.action_execution import finite
from learning.mac_clock_bridge import MacHostClock, validate_sample


class FrameInbox:
    """One lock orders frame ingress and dispatch/result boundaries.

    at is host producer acceptance, before processing/storage in the collector.
    The native callback, PTS, stdout read time and storage timing remain in spool.
    """
    def __init__(self, *, clock=time, max_frames=600, max_age=.5, on_fault=lambda: None):
        self.clock, self.max_frames, self.max_age = clock, max_frames, max_age
        self.on_fault = on_fault
        self.lock = threading.RLock()
        self.pending = deque()
        self.latest = None
        self.count = 0
        self.fault = None
        self.done = False

    def fail(self, code):
        with self.lock:
            self.fault = self.fault or code
        self.on_fault()

    def accept(self, packet, *, at=None):
        # Called with native evidence already checked, within the ingress lock.
        with self.lock:
            if self.count >= self.max_frames:
                self.fail('camera_frame_budget'); return
            packet = copy.deepcopy(packet)
            packet['at'] = self.clock.monotonic() if at is None else at
            self.pending.append(packet)
            self.latest = packet
            self.count += 1

    def check_fresh(self):
        with self.lock:
            now = self.clock.monotonic()
            if self.fault or self.done or self.latest is None:
                raise ValueError(self.fault or 'camera_not_live')
            if not 0 <= now-self.latest['sourceEarliest'] <= self.max_age:
                raise ValueError('camera_stale')
            if not all(p['visible'] for p in self.latest['features']):
                raise ValueError('tracking_lost')

    def drain(self):
        with self.lock:
            items = list(self.pending)
            self.pending.clear()
            return items


class NativeCameraStream(FrameInbox):
    def __init__(self, root, helper, plan, tracker, *, host_clock_id, on_fault=lambda: None):
        super().__init__(max_frames=plan['maxFrames'], max_age=plan['maxFrameAgeSeconds'], on_fault=on_fault)
        self.root, self.helper = Path(root), Path(helper).resolve(strict=True)
        self.plan, self.tracker, self.host_clock_id = copy.deepcopy(plan), tracker, host_clock_id
        self.process = None
        self.stop_event = threading.Event()
        self.reader_thread = self.watch_thread = None
        self.previous_pts = None
        self.configuration = None
        self.total_bytes = 0
        self.clock_source = None
        self.clock_before = None
        self.offset_low = None
        self.started = None

    def start(self):
        if self.process is not None or self.started is not None:
            raise ValueError('camera_start_is_single_use')
        self.root.mkdir(mode=0o700, exist_ok=False)
        self.ingress_store = ArtifactStore(self.root/'host-ingress')
        self.clock_source = MacHostClock()
        self.clock_before = self.clock_source.sample(self.host_clock_id)
        self.offset_low, _ = validate_sample(self.clock_before)
        (self.root/'clock-before.json').write_text(__import__('json').dumps(self.clock_before))
        p = self.plan
        command = [str(self.helper), '--stream', p['deviceId'], str(self.root.resolve()), p['sessionId'], p['viewId'],
                   str(p['maxFrames']), str(p['seconds']), str(p['sampleFps']), str(p['maxBytes'])]
        self.stderr = (self.root/'helper.stderr').open('xb')
        self.stdout_log = (self.root/'helper.stdout').open('xb')
        self.started = time.monotonic()
        try:
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=self.stderr)
        except BaseException:
            self.stderr.close(); self.stdout_log.close(); raise
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.watch_thread = threading.Thread(target=self._watch, daemon=True)
        self.reader_thread.start(); self.watch_thread.start()

    def _packet(self, receipt, read_at, ingress_at):
        i = self.count
        name = f'frame-{i:04d}'
        from learning.lossless_frame import ENCODING, SUFFIX
        encoding = self.plan.get('storageEncoding', 'BGRA8')
        if encoding not in ('BGRA8', ENCODING):
            raise ValueError('camera_storage_encoding_invalid')
        image_name = name + (SUFFIX if encoding == ENCODING else '.bgra')
        if receipt.get('frameIndex') != i or receipt.get('metadataFile') != name+'.json' or receipt.get('imageFile') != image_name or receipt.get('durable') is not True:
            raise ValueError('camera_receipt_mismatch')
        meta_bytes = _read(self.root, name+'.json')
        m = _decode_json(meta_bytes)
        if hashlib.sha256(meta_bytes).hexdigest() != receipt.get('metadataSha256'):
            raise ValueError('camera_metadata_hash')
        p = self.plan
        if (m.get('schemaVersion') != ('motionbrain.mac_camera_frame.v2' if encoding == ENCODING else 'motionbrain.mac_camera_frame.v1') or m.get('sessionId') != p['sessionId']
                or m.get('viewConfigurationId') != p['viewId'] or m.get('deviceUniqueId') != p['deviceId']
                or m.get('sourceFrameId') != f"{p['sessionId']}:{i}" or m.get('frameIndex') != i
                or (m.get('width'), m.get('height'), m.get('rowBytes')) != (p['width'], p['height'], p['width']*4)
                or m.get('pixelFormat') != 'BGRA8' or m.get('videoMirrored') != p['videoMirrored']
                or m.get('videoRotationAngle') != p['videoRotationAngle']
                or m.get('timestampSemantics') != 'cmsamplebuffer_presentation_timestamp'
                or m.get('clockDomain') != 'coremedia_host_clock' or m.get('imageFile') != image_name):
            raise ValueError('camera_identity_or_format_changed')
        config = m.get('configurationSha256')
        encoded = m.get('actualConfigurationJson')
        if not isinstance(encoded, str) or hashlib.sha256(encoded.encode()).hexdigest() != config:
            raise ValueError('camera_configuration_invalid')
        if self.configuration is not None and config != self.configuration:
            raise ValueError('camera_configuration_changed')
        self.configuration = config
        pts = m.get('pts', {})
        if (any(type(pts.get(k)) is not int for k in ('value', 'timescale', 'epoch', 'flags'))
                or pts['timescale'] <= 0 or pts['value'] < 0 or pts['epoch'] != 0 or not pts['flags'] & 1 or pts['flags'] & ~3):
            raise ValueError('camera_pts_invalid')
        from fractions import Fraction
        current = Fraction(pts['value'], pts['timescale'])
        if self.previous_pts is not None and current <= self.previous_pts:
            raise ValueError('camera_pts_regressed')
        self.previous_pts = current
        callback, native_pts = m.get('callbackHostSeconds'), m.get('convertedHostPtsSeconds')
        if not finite(callback) or not finite(native_pts) or not 0 <= native_pts <= callback:
            raise ValueError('camera_host_clock_invalid')
        earliest = native_pts+self.offset_low
        if not 0 <= time.monotonic()-earliest <= p['maxFrameAgeSeconds']:
            raise ValueError('camera_stale')
        if not finite(receipt.get('persistenceMs')) or not 0 <= receipt['persistenceMs'] <= 250:
            raise ValueError('camera_storage_late')
        storage = {}
        if encoding == ENCODING:
            if m.get('storageEncoding') != encoding or m.get('rawByteLength') != p['width']*p['height']*4:
                raise ValueError('camera_compressed_format_invalid')
            storage = {k: m.get(k) for k in ('storageEncoding', 'rawByteLength', 'pixelSha256', 'storedByteLength')}
        pixels = self.pixels(dict(imageFile=image_name, sha256=m.get('imageSha256'), **storage))
        if receipt.get('imageSha256') != m.get('imageSha256'):
            raise ValueError('camera_receipt_image_hash')
        self.total_bytes += storage.get('storedByteLength', len(pixels))
        if self.total_bytes > p['maxBytes']:
            raise ValueError('camera_byte_budget')
        features = self.tracker.update(pixels)
        packet = dict(frameId=m['sourceFrameId'], imageFile=image_name, sha256=m['imageSha256'],
                      pts={k: pts[k] for k in ('value', 'timescale', 'epoch')}, features=features,
                      sourceEarliest=earliest, callbackHostSeconds=callback, stdoutReadAt=read_at, **storage)
        self.accept(packet, at=ingress_at)
        self.ingress_store.write_json(f'{i:04d}.json', self.latest)
        if self.tracker.fault:
            self.fail(self.tracker.fault)

    def pixels(self, packet):
        raw = _read(self.root, packet['imageFile'])
        from learning.lossless_frame import ENCODING, decode
        if self.plan.get('storageEncoding') == ENCODING:
            if (packet.get('storageEncoding') != ENCODING or packet.get('rawByteLength') != self.plan['width']*self.plan['height']*4
                    or type(packet.get('storedByteLength')) is not int or len(raw) != packet['storedByteLength']
                    or hashlib.sha256(raw).hexdigest() != packet['sha256']):
                raise ValueError('camera_compressed_pixels_invalid')
            return decode(raw, packet['rawByteLength'], packet.get('pixelSha256'))
        if len(raw) != self.plan['width']*self.plan['height']*4 or hashlib.sha256(raw).hexdigest() != packet['sha256']:
            raise ValueError('camera_pixels_invalid')
        return raw

    def _reader(self):
        try:
            terminal_seen = False
            while True:
                line = self.process.stdout.readline(65537)
                read_at = time.monotonic()
                if not line: break
                self.stdout_log.write(line); self.stdout_log.flush()
                if len(line) > 65536 or not line.endswith(b'\n'):
                    raise ValueError('camera_stdout_invalid')
                event = _decode_json(line)
                with self.lock:
                    if event.get('event') == 'frame':
                        if terminal_seen: raise ValueError('frame_after_terminal')
                        self._packet(event['receipt'], read_at, time.monotonic())
                    elif event.get('event') == 'terminal':
                        terminal = _decode_json(_read(self.root, 'terminal.json'))
                        if (terminal_seen or event.get('status') != 'completed' or event.get('reason') != 'duration_complete'
                                or terminal.get('status') != 'completed' or terminal.get('reason') != 'duration_complete'
                                or terminal.get('schemaVersion') != 'motionbrain.mac_camera_stream_terminal.v1'
                                or terminal.get('sessionId') != self.plan['sessionId']
                                or terminal.get('viewConfigurationId') != self.plan['viewId']
                                or terminal.get('recordedFrames') != self.count or self.count < 2
                                or terminal.get('droppedFrames') != 0):
                            raise ValueError('camera_terminal_failed')
                        terminal_seen = True
                    else:
                        raise ValueError('camera_event_unknown')
            if not terminal_seen or self.process.wait(timeout=2) != 0:
                raise ValueError('camera_ended_without_terminal')
            with self.lock:
                self.done = True
        except BaseException as exc:
            self.fail(str(exc) if isinstance(exc, ValueError) else 'camera_reader_failed')
        finally:
            self.stdout_log.close()

    def _watch(self):
        while not self.stop_event.wait(.02):
            with self.lock:
                if self.done: return
                if time.monotonic()-self.started > 47:
                    self.fail('camera_process_timeout')
                    self.process.terminate(); return
                if self.latest is not None and time.monotonic()-self.latest['sourceEarliest'] > self.max_age:
                    self.fail('camera_stream_gap')

    def close(self):
        self.stop_event.set()
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try: self.process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self.process.kill(); self.process.wait(timeout=1)
            self.reader_thread.join(timeout=2)
            self.watch_thread.join(timeout=2)
            self.stderr.close()
            if self.process.stdout: self.process.stdout.close()
            if self.clock_source:
                after = self.clock_source.sample(self.host_clock_id)
                from learning.mac_clock_bridge import bridge
                import json
                (self.root/'clock-after.json').write_text(json.dumps(after))
                bridge(self.clock_before, after)
