"""한 HTTP 요청의 구간만 계측한다. 요청·재시도·전역 설정 변경은 만들지 않는다.

호출자가 기존 opener에 handler()를 주입하고 원래의 timeout/절대 기한을 적용한다.
본문·헤더·주소·인증값·예외 메시지를 수집하지 않는다. TCP 통계는 macOS에서
include_tcp_info=True일 때에만 기존 연결 소켓에서 읽으며 새 소켓을 열지 않는다.
"""
from __future__ import annotations

import copy
import ctypes
import http.client
import socket
import sys
import time
import urllib.request


class _DarwinTCPInfo(ctypes.Structure):
    # 기존 로컬 원인 계측에서 사용한 Darwin tcp_connection_info 배치다.
    _fields_ = ([(name, ctypes.c_uint8) for name in ('state', 'sndscale', 'rcvscale', 'pad')]
        + [(name, ctypes.c_uint32) for name in ('options', 'flags', 'rto', 'maxseg', 'ssthresh',
            'cwnd', 'sndwnd', 'sendBuffered', 'rcvwnd', 'rtt', 'srtt', 'rttvar', 'tfo')]
        + [(name, ctypes.c_uint64) for name in ('txpackets', 'txbytes', 'txRetransmitBytes',
            'rxpackets', 'rxbytes', 'rxOutOfOrderBytes', 'txRetransmitPackets')])


class HttpPhaseTrace:
    """생성 시 I/O 없음. 하나의 HTTP open만 허용하는 제한된 타이밍 수집기.

    events는 phase/connect·send·headers·body, event/begin·end·error, at/단조시각을
    갖는 스냅샷이다. phase는 최초 실패 구간 또는 마지막 관측 구간이며,
    max_events 이후에도 최초 실패 구간과 dropped_events는 유지된다.
    """
    def __init__(self, max_events=128, *, clock=time, include_tcp_info=False):
        if type(max_events) is not int or not 1 <= max_events <= 128:
            raise ValueError('http_phase_trace_event_budget')
        if type(include_tcp_info) is not bool or not callable(getattr(clock, 'monotonic', None)):
            raise ValueError('http_phase_trace_configuration')
        self.max_events, self.clock, self.include_tcp_info = max_events, clock, include_tcp_info
        self._events, self._dropped = [], 0
        self._last_phase, self._failed_phase = 'not_started', None
        self._claimed = False

    @property
    def events(self):
        return copy.deepcopy(self._events)

    @property
    def phase(self):
        return self._failed_phase or self._last_phase

    @property
    def dropped_events(self):
        return self._dropped

    def _socket_metadata(self, sock):
        if sock is None:
            return None
        try:
            metadata = dict(family=int(sock.family), type=int(sock.type), timeoutSeconds=sock.gettimeout())
            if self.include_tcp_info and sys.platform == 'darwin':
                raw = sock.getsockopt(socket.IPPROTO_TCP, 0x106, ctypes.sizeof(_DarwinTCPInfo))
                value = _DarwinTCPInfo.from_buffer_copy(raw)
                metadata['tcp'] = {name: getattr(value, name) for name in ('state', 'rto', 'rtt',
                    'srtt', 'rttvar', 'sendBuffered', 'txpackets', 'txbytes', 'txRetransmitBytes',
                    'rxpackets', 'rxbytes', 'rxOutOfOrderBytes', 'txRetransmitPackets')}
            return metadata
        except TimeoutError:
            # 호출자의 절대 기한 알람을 선택적 통계 수집이 삼키면 안 된다.
            raise
        except (OSError, ValueError, TypeError):
            return None

    def _mark(self, phase, event, sock=None, error=None):
        self._last_phase = phase
        if error is not None and self._failed_phase is None:
            self._failed_phase = phase
        if len(self._events) >= self.max_events:
            self._dropped += 1
            return
        entry = dict(phase=phase, event=event, at=self.clock.monotonic())
        if error is not None:
            # 메시지에는 URL·헤더·본문이 포함될 수 있으므로 예외 유형만 기록한다.
            entry['errorType'] = type(error).__name__
        metadata = self._socket_metadata(sock)
        if metadata is not None:
            entry['socket'] = metadata
        self._events.append(entry)

    def handler(self):
        """기존 urllib 전송 구현을 감싼 인스턴스 전용 HTTPHandler를 반환한다."""
        trace = self

        class Response(http.client.HTTPResponse):
            def __init__(self, sock, *args, **kwargs):
                self._phase_socket = sock
                super().__init__(sock, *args, **kwargs)

            def _traced(self, phase, operation, *args, **kwargs):
                trace._mark(phase, 'begin', self._phase_socket)
                try:
                    result = operation(*args, **kwargs)
                except BaseException as exc:
                    trace._mark(phase, 'error', self._phase_socket, exc)
                    raise
                trace._mark(phase, 'end', self._phase_socket)
                return result

            def begin(self):
                return self._traced('headers', super().begin)

            def read(self, *args, **kwargs):
                return self._traced('body', super().read, *args, **kwargs)

            def read1(self, *args, **kwargs):
                return self._traced('body', super().read1, *args, **kwargs)

            def readinto(self, *args, **kwargs):
                return self._traced('body', super().readinto, *args, **kwargs)

        class Connection(http.client.HTTPConnection):
            response_class = Response

            def connect(self):
                trace._mark('connect', 'begin', self.sock)
                try:
                    result = super().connect()
                except BaseException as exc:
                    trace._mark('connect', 'error', self.sock, exc)
                    raise
                trace._mark('connect', 'end', self.sock)
                return result

            def send(self, data):
                trace._mark('send', 'begin', self.sock)
                try:
                    result = super().send(data)
                except BaseException as exc:
                    trace._mark('send', 'error', self.sock, exc)
                    raise
                trace._mark('send', 'end', self.sock)
                return result

        class Handler(urllib.request.HTTPHandler):
            def http_open(self, request):
                if trace._claimed:
                    raise ValueError('http_phase_trace_single_request')
                trace._claimed = True
                return self.do_open(Connection, request)

        return Handler()
