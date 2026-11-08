"""Finite recording without tracking or motor access; acquisition starts explicitly."""
import copy
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import subprocess
import threading
import time

from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.action_execution import finite
from learning.lossless_frame import ENCODING, SUFFIX
from learning.mac_clock_bridge import MacHostClock, validate_sample


def stream_helper_capabilities(helper, plan):
    """Query the local producer contract before opening a camera or recording."""
    try:
        result = subprocess.run([str(Path(helper).resolve(strict=True)), '--describe-capabilities'],
                                capture_output=True, timeout=3, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ValueError('recording_helper_capability_query_failed') from exc
    if result.returncode != 0 or not 0 < len(result.stdout) <= 65536:
        raise ValueError('recording_helper_capability_query_failed')
    capabilities = _decode_json(result.stdout)
    if type(capabilities) is not dict or type(capabilities.get('stream')) is not dict:
        raise ValueError('recording_helper_stream_protocol_incompatible')
    stream = capabilities.get('stream', {})
    if (capabilities.get('schemaVersion') != 'motionbrain.mac_camera_capabilities.v1'
            or stream.get('frameSchemaVersion') != 'motionbrain.mac_camera_frame.v2'
            or stream.get('storageEncoding') != ENCODING
            or stream.get('processingMethod') != 'lossless_record_only_v1'
            or stream.get('gracefulStop') is not True
            or stream.get('droppedFramePolicy') != 'fail_close'
            or type(stream.get('storageBudgetMs')) is not int
            or stream['storageBudgetMs'] != 250):
        raise ValueError('recording_helper_stream_protocol_incompatible')
    for advertised, requested in [('maxFrames', 'maxFrames'), ('maxSeconds', 'seconds'),
                                  ('maxSampleFps', 'sampleFps'), ('maxBytes', 'maxBytes')]:
        limit = stream.get(advertised)
        if type(limit) is not int or limit < plan[requested]:
            raise ValueError('recording_helper_stream_budget_incompatible')
    deadline = stream.get('overallDeadlineSeconds')
    # Reserve launch, warmup and shutdown time within the same clock bracket.
    if type(deadline) is not int or not plan['seconds']+50 <= deadline <= 170:
        raise ValueError('recording_helper_stream_deadline_incompatible')
    return capabilities


def recording_bridge(before, after):
    """Same-clock bracket for the recorder's bounded 120-second acquisition."""
    a, b = validate_sample(before), validate_sample(after)
    if any(before[k] != after[k] for k in ('hostClockId', 'hostBootId', 'pythonClock')):
        raise ValueError('recording_clock_identity_changed')
    start, end = before['pairs'][-1]['after'], after['pairs'][0]['before']
    low, high = max(a[0], b[0]), min(a[1], b[1])
    if not 0 <= end-start <= 170 or low > high:
        raise ValueError('recording_clock_bracket_invalid')
    return dict(hostClockId=before['hostClockId'], validPythonBounds=[start, end],
                offsetBoundsSeconds=[low, high], opticalExposureVerified=False,
                timestampSemantics='presentation_timestamp_not_optical_exposure')


class ActionRecorder:
    def __init__(self, root, helper, plan, *, clock=time):
        self.root, self.helper = Path(root), Path(helper)
        self.plan, self.clock = copy.deepcopy(plan), clock
        for key in ('deviceId', 'sessionId', 'viewId'):
            if not isinstance(plan.get(key), str) or not plan[key]:
                raise ValueError('recording_identity_required')
        for key, lo, hi in (('maxFrames', 2, 1000), ('seconds', 2, 120),
                            ('sampleFps', 1, 30), ('maxBytes', 4, 8400000000)):
            if type(plan.get(key)) is not int or not lo <= plan[key] <= hi:
                raise ValueError('recording_budget_invalid_'+key)
        if plan.get('storageEncoding', ENCODING) != ENCODING:
            raise ValueError('recording_encoding_invalid')
        self.host_clock_id = plan.get('hostClockId', plan['sessionId'])
        self.frames = []
        self.first_source_at = None
        self.fault = None
        self.done = False
        self.started = None
        self.process = None
        self.clock_source = self.clock_before = None
        self.offset_low = self.offset_high = None
        self.previous_pts = self.previous_native = None
        self.configuration = None
        self.total_bytes = 0
        self.lock = threading.RLock()
        self.closing = False
        self.closed = None
        self.reader_thread = None
        self.terminal = None

    def _fail(self, reason):
        with self.lock:
            self.fault = self.fault or reason

    def start(self):
        if self.started is not None:
            raise ValueError('recording_single_use')
        self.started = self.clock.monotonic()
        try:
            capabilities = stream_helper_capabilities(self.helper, self.plan)
            self.root.mkdir(mode=0o700, parents=True, exist_ok=False)
            self.store = ArtifactStore(self.root/'host-ingress')
            self.store.write_json('helper-capabilities.json', capabilities)
            self.clock_source = MacHostClock()
            self.clock_before = self.clock_source.sample(self.host_clock_id)
            self.offset_low, self.offset_high = validate_sample(self.clock_before)
            self.store.write_json('clock-before.json', self.clock_before)
            p = self.plan
            cmd = [str(self.helper.resolve(strict=True)), '--stream', p['deviceId'],
                   str(self.root.resolve()), p['sessionId'], p['viewId'],
                   str(p['maxFrames']), str(p['seconds']), str(p['sampleFps']), str(p['maxBytes'])]
            self.stderr = (self.root/'helper.stderr').open('xb')
            self.stdout_log = (self.root/'helper.stdout').open('xb')
            self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=self.stderr)
            self.reader_thread = threading.Thread(target=self._reader, daemon=True)
            self.reader_thread.start()
        except BaseException:
            self._fail('recording_start_failed')
            for name in ('stderr', 'stdout_log'):
                if hasattr(self, name): getattr(self, name).close()
            raise

    def wait_ready(self, timeout=15):
        if not finite(timeout) or not 0 < timeout <= 30 or self.started is None:
            raise ValueError('recording_ready_wait_invalid')
        deadline = self.clock.monotonic()+timeout
        while self.clock.monotonic() < deadline:
            with self.lock:
                if self.fault: raise ValueError(self.fault)
                if self.frames:
                    self.check()
                    return self.frames[-1]
                if self.done: raise ValueError('recording_ended_before_ready')
            self.clock.sleep(.02)
        self._fail('recording_startup_timeout')
        raise ValueError(self.fault)

    def check(self):
        with self.lock:
            if self.fault: raise ValueError(self.fault)
            if self.started is None or not self.frames:
                raise ValueError('recording_not_ready')
            if self.done or self.closing:
                raise ValueError('recording_ended')
            age = self.clock.monotonic()-self.frames[-1]['sourceEarliest']
            if not 0 <= age <= 2:
                self._fail('recording_source_stall')
                raise ValueError(self.fault)

    def _packet(self, receipt, read_at):
        i = len(self.frames)
        if i >= self.plan['maxFrames']: raise ValueError('recording_frame_budget')
        name = f'frame-{i:04d}'
        image_name = name+SUFFIX
        if (receipt.get('frameIndex') != i or receipt.get('metadataFile') != name+'.json'
                or receipt.get('imageFile') != image_name or receipt.get('durable') is not True):
            raise ValueError('recording_receipt_invalid')
        raw_meta = _read(self.root, name+'.json')
        if hashlib.sha256(raw_meta).hexdigest() != receipt.get('metadataSha256'):
            raise ValueError('recording_metadata_hash')
        m = _decode_json(raw_meta)
        p = self.plan
        if (m.get('schemaVersion') != 'motionbrain.mac_camera_frame.v2'
                or m.get('sessionId') != p['sessionId'] or m.get('viewConfigurationId') != p['viewId']
                or m.get('deviceUniqueId') != p['deviceId'] or m.get('frameIndex') != i
                or m.get('sourceFrameId') != f"{p['sessionId']}:{i}" or m.get('imageFile') != image_name
                or m.get('storageEncoding') != ENCODING or m.get('pixelFormat') != 'BGRA8'
                or m.get('clockDomain') != 'coremedia_host_clock'
                or m.get('timestampSemantics') != 'cmsamplebuffer_presentation_timestamp'):
            raise ValueError('recording_identity_changed')
        width, height = m.get('width'), m.get('height')
        if (type(width) is not int or type(height) is not int or not 1 <= width <= 1920
                or not 1 <= height <= 1080 or m.get('rowBytes') != width*4
                or m.get('rawByteLength') != width*height*4):
            raise ValueError('recording_dimensions_invalid')
        for key in ('width', 'height', 'videoMirrored', 'videoRotationAngle'):
            if key in p and m.get(key) != p[key]: raise ValueError('recording_format_changed')
        encoded = m.get('actualConfigurationJson')
        if not isinstance(encoded, str) or hashlib.sha256(encoded.encode()).hexdigest() != m.get('configurationSha256'):
            raise ValueError('recording_configuration_invalid')
        configuration = (m['configurationSha256'], width, height, m.get('videoMirrored'), m.get('videoRotationAngle'))
        if self.configuration is not None and configuration != self.configuration:
            raise ValueError('recording_configuration_changed')
        pts = m.get('pts', {})
        if (any(type(pts.get(k)) is not int for k in ('value','timescale','epoch','flags'))
                or pts['timescale'] <= 0 or pts['value'] < 0 or pts['epoch'] != 0
                or not pts['flags'] & 1 or pts['flags'] & ~3):
            raise ValueError('recording_pts_invalid')
        current = Fraction(pts['value'], pts['timescale'])
        native, callback = m.get('convertedHostPtsSeconds'), m.get('callbackHostSeconds')
        if (not finite(native) or not finite(callback) or not 0 <= native <= callback
                or (self.previous_pts is not None and current <= self.previous_pts)
                or (self.previous_native is not None and native <= self.previous_native)):
            raise ValueError('recording_pts_regressed')
        earliest, latest = native+self.offset_low, native+self.offset_high
        if latest > read_at+.01 or earliest < self.started-.01:
            raise ValueError('recording_pts_outside_lifecycle')
        persistence = receipt.get('persistenceMs')
        if not finite(persistence) or persistence < 0:
            raise ValueError('recording_persistence_invalid')
        stored = _read(self.root, image_name)
        sha = hashlib.sha256(stored).hexdigest()
        if (len(stored) != m.get('storedByteLength') or sha != m.get('imageSha256')
                or sha != receipt.get('imageSha256') or not isinstance(m.get('pixelSha256'), str)
                or len(m['pixelSha256']) != 64):
            raise ValueError('recording_image_hash')
        if self.total_bytes+len(stored) > p['maxBytes']: raise ValueError('recording_byte_budget')
        frame = dict(frameIndex=i, frameId=m['sourceFrameId'], metadataFile=name+'.json',
                     metadataSha256=receipt['metadataSha256'], imageFile=image_name, sha256=sha,
                     imageSha256=sha, pixelSha256=m['pixelSha256'], width=width, height=height,
                     storageEncoding=ENCODING, rawByteLength=m['rawByteLength'], storedByteLength=len(stored),
                     sourceEarliest=earliest, sourceLatest=latest, clockOffsetBoundsSeconds=[self.offset_low, self.offset_high], convertedHostPtsSeconds=native,
                     callbackHostSeconds=callback, stdoutReadAt=read_at, persistenceMs=persistence,
                     qualityFlags=['slow_save'] if persistence > 250 else [], opticalExposureVerified=False)
        # A single old frame is preserved; check() detects persistent lack of fresh source.
        if read_at-earliest > 2: frame['qualityFlags'].append('late_delivery')
        self.store.write_json(f'{i:04d}.json', frame)
        self.frames.append(frame)
        self.configuration, self.previous_pts, self.previous_native = configuration, current, native
        self.total_bytes += len(stored)
        if self.first_source_at is None: self.first_source_at = earliest

    def _read_terminal(self, event):
        t = _decode_json(_read(self.root, 'terminal.json'))
        if (self.terminal is not None or t.get('schemaVersion') != 'motionbrain.mac_camera_stream_terminal.v1'
                or t.get('sessionId') != self.plan['sessionId'] or t.get('viewConfigurationId') != self.plan['viewId']
                or t.get('deviceUniqueId') != self.plan['deviceId'] or t.get('recordedFrames') != len(self.frames)
                or event.get('status') != t.get('status') or event.get('reason') != t.get('reason')):
            raise ValueError('recording_terminal_invalid')
        self.terminal = t
        accepted = t.get('status') == 'completed' and t.get('reason') == 'duration_complete'
        accepted |= self.closing and t.get('status') == 'partial' and t.get('reason') == 'operator_stop'
        if not accepted: raise ValueError('recording_terminal_'+str(t.get('reason')))

    def _reader(self):
        try:
            for line in iter(lambda:self.process.stdout.readline(65537), b''):
                read_at = self.clock.monotonic()
                self.stdout_log.write(line);self.stdout_log.flush()
                if len(line)>65536 or not line.endswith(b'\n'): raise ValueError('recording_stdout_invalid')
                event = _decode_json(line)
                with self.lock:
                    if event.get('event') == 'frame' and self.terminal is None:
                        self._packet(event['receipt'], read_at)
                    elif event.get('event') == 'terminal': self._read_terminal(event)
                    else: raise ValueError('recording_event_invalid')
            if self.terminal is None or self.process.wait(timeout=2) != 0:
                raise ValueError('recording_missing_terminal')
        except BaseException as exc:
            self._fail(str(exc) if isinstance(exc, ValueError) else 'recording_reader_failed')
        finally:
            with self.lock: self.done = True
            self.stdout_log.close()

    def close(self):
        if self.closed is not None: return copy.deepcopy(self.closed)
        self.closing = True
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try: self.process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    self.process.kill();self.process.wait(timeout=2);self._fail('recording_forced_kill')
            if self.reader_thread:
                self.reader_thread.join(timeout=3)
                if self.reader_thread.is_alive(): self._fail('recording_reader_not_closed')
            self.stderr.close()
            if self.process.stdout: self.process.stdout.close()
        evidence = None
        if self.clock_source and self.clock_before:
            try:
                after = self.clock_source.sample(self.host_clock_id)
                self.store.write_json('clock-after.json', after)
                evidence = recording_bridge(self.clock_before, after)
                self.store.write_json('clock-bridge.json', evidence)
            except Exception as exc:
                self._fail(str(exc) if isinstance(exc, ValueError) else 'recording_clock_close_failed')
        self.closed = dict(status='failed' if self.fault else ('completed' if self.terminal and self.terminal.get('status')=='completed' else 'partial'),
                           reason=self.fault, recordedFrames=len(self.frames), savedBytes=self.total_bytes,
                           terminalStatus=self.terminal.get('status') if self.terminal else None,
                           clockBridge=evidence, frames=copy.deepcopy(self.frames))
        if hasattr(self, 'store'): self.store.write_json('recording-summary.json', self.closed)
        return copy.deepcopy(self.closed)
