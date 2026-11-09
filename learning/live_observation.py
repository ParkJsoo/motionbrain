"""읽기 전용 실시간 표시의 최신 값·프레임 검증. 제어·학습 권한 없음."""
from __future__ import annotations

import hashlib
import math
import threading

from learning.artifacts import _decode_json
from learning.lossless_frame import decode


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def number(value):
    return type(value) in (float, int) and math.isfinite(value)


class LatestFeed:
    """프레임 이력을 쌓지 않는다. 연결 실패는 명시적 재시작 전 해제하지 않는다."""
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.failure = None
        self.count = 0

    def publish(self, value):
        require(number(value['sourceAt']), 'source_time_required')
        with self.lock:
            require(self.failure is None, 'failed_feed_cannot_resume')
            self.count += 1
            self.latest = dict(value, ordinal=self.count)

    def fail(self, reason):
        with self.lock:
            self.failure = self.failure or str(reason)

    def snapshot(self, now, maximum_age):
        with self.lock:
            item, failure, count = self.latest, self.failure, self.count
        if failure:
            return dict(phase='failed', reason=failure, count=count), None
        if item is None:
            return dict(phase='waiting', count=count), None
        age = now-item['sourceAt']
        if not 0 <= age <= maximum_age:
            return dict(phase='stale', ageSeconds=age, count=count), None
        phase = 'rejected' if item.get('observationError') else 'live'
        return dict(phase=phase, ageSeconds=age, count=count,
                    reason=item.get('observationError')), item


def _exact(stream, length):
    chunks = bytearray()
    while len(chunks) < length:
        chunk = stream.read(length-len(chunks))
        require(bool(chunk), 'preview_payload_truncated')
        chunks.extend(chunk)
    return bytes(chunks)


def read_preview(stream, expected, previous, now, offset=0.):
    """크기를 확인한 뒤 읽는다. 카메라·영상 변환·설정 변경은 거절한다."""
    line = stream.readline(8193)
    require(line and len(line) <= 8192 and line.endswith(b'\n'), 'preview_header_missing_or_oversized')
    h = _decode_json(line)
    require(isinstance(h, dict) and h.get('event') == 'preview_frame'
            and h.get('trainingEligible') is False and h.get('controlEligible') is False,
            'preview_header_invalid')
    for key in ('deviceUniqueId', 'configurationSha256', 'width', 'height',
                'videoMirrored', 'videoRotationAngle'):
        require(h.get(key) == expected[key], 'preview_identity_changed:'+key)
    require(type(h.get('sequence')) is int and h['sequence'] == previous+1,
            'preview_sequence_changed')
    size = expected['width']*expected['height']*4
    require(type(h.get('storedByteLength')) is int and 0 < h['storedByteLength'] <= size+65536
            and h.get('rawByteLength') == size, 'preview_payload_size_invalid')
    require(all(isinstance(h.get(k), str) and len(h[k]) == 64
                and all(c in '0123456789abcdef' for c in h[k])
                for k in ('pixelSha256', 'imageSha256')), 'preview_hash_required')
    require(number(h.get('sourceHostSeconds')) and number(h.get('callbackHostSeconds'))
            and 0 <= h['sourceHostSeconds'] <= h['callbackHostSeconds'], 'preview_timestamp_invalid')
    source_at = h['sourceHostSeconds']+offset
    payload = _exact(stream, h['storedByteLength'])
    require(hashlib.sha256(payload).hexdigest() == h.get('imageSha256'), 'preview_payload_hash_changed')
    pixels = decode(payload, size, h.get('pixelSha256'))
    require(0 <= now()-source_at <= 1.5, 'preview_frame_stale')
    return h, pixels, source_at


def shoulder_sample(status):
    shoulder = status.get('shoulderAngle', {})
    require(shoulder.get('available') is True and shoulder.get('sensorConnected') is True
            and shoulder.get('sensorFresh') is True and shoulder.get('sensorReady') is True
            and number(shoulder.get('ageMs')) and 0 <= shoulder['ageMs'] <= 500
            and number(shoulder.get('angleDeg')), 'M4_sensor_not_fresh')
    require(type(status.get('uptimeMs')) is int and status['uptimeMs'] >= 0,
            'controller_uptime_invalid')
    return shoulder['angleDeg']
