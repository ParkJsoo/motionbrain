"""Pi SSH 연결과 LocalForward 설치를 원격 READY로 확인하는 단일 사용 터널.

준비 확인용 TCP 연결이나 HTTP 요청을 만들지 않는다. 인증값·장치 명령·재시도는
포함하지 않으며, 이 객체가 시작한 SSH 자식 하나만 종료하고 회수한다.
"""
from __future__ import annotations

import copy
import ipaddress
import math
import os
import re
import selectors
import shlex
import subprocess
import time
import uuid


class PiHttpTunnelError(RuntimeError):
    """제한된 SSH 준비 또는 수명 주기 검사 실패."""


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


class PiHttpTunnel:
    """명시한 Pi 별칭과 사설 IPv4 장치 두 개 이하만 loopback에 전달한다.

    생성자는 검증만 수행한다. start() 또는 with 진입 시 SSH를 시작하며,
    SSH 인증·포워드 설치 후 실행되는 원격 Python의 READY를 기다린다.
    준비 성공은 장치 HTTP 응답이나 실물 실행 준비가 확인됐다는 뜻이 아니다.
    """
    def __init__(self, pi_alias, forwards, *, startup_seconds=6., hold_seconds=120,
                 store=None, log_name='pi-http-tunnel-diagnostic.json',
                 max_log_bytes=65536, clock=time):
        if not isinstance(pi_alias, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,100}', pi_alias):
            raise ValueError('explicit_pi_ssh_alias_required')
        if not isinstance(forwards, list) or not 1 <= len(forwards) <= 2:
            raise ValueError('one_or_two_declared_http_forwards_required')
        ports = set()
        ranges = [ipaddress.IPv4Network(value) for value in ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16')]
        for forward in forwards:
            if not isinstance(forward, dict) or set(forward) != {'localPort', 'targetHost'}:
                raise ValueError('explicit_http_forward_fields_required')
            port = forward['localPort']
            if type(port) is not int or not 1024 <= port <= 65535 or port in ports:
                raise ValueError('unique_unprivileged_loopback_port_required')
            ports.add(port)
            try:
                address = ipaddress.IPv4Address(forward['targetHost'])
            except (ipaddress.AddressValueError, TypeError):
                raise ValueError('private_device_ipv4_required') from None
            if not isinstance(forward['targetHost'], str) or not any(address in network for network in ranges):
                raise ValueError('private_device_ipv4_required')
        if not _number(startup_seconds) or not 0 < startup_seconds <= 6:
            raise ValueError('bounded_tunnel_startup_required')
        if type(hold_seconds) is not int or not 1 <= hold_seconds <= 180:
            raise ValueError('bounded_remote_hold_required')
        if type(max_log_bytes) is not int or not 1024 <= max_log_bytes <= 65536:
            raise ValueError('bounded_tunnel_log_required')
        if not isinstance(log_name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*\.json', log_name):
            raise ValueError('safe_tunnel_log_name_required')
        self.pi_alias, self.forwards = pi_alias, copy.deepcopy(forwards)
        self.startup_seconds, self.hold_seconds = startup_seconds, hold_seconds
        self.store, self.log_name, self.max_log_bytes, self.clock = store, log_name, max_log_bytes, clock
        self._declaration = (pi_alias, copy.deepcopy(forwards), startup_seconds, hold_seconds,
                             max_log_bytes, log_name)
        self._ready_line = 'MOTIONBRAIN_PI_HTTP_TUNNEL_READY_' + uuid.uuid4().hex
        self.process = None
        self._selector = None
        self._used = self._closed = self._ready = self._ready_seen = self._logged = False
        self._started_at = self._ready_at = None
        self._output = {'stdout': bytearray(), 'stderr': bytearray()}
        self._failure = None
        self._cleanup_errors = []

    def _check_declaration(self):
        if (self.pi_alias, self.forwards, self.startup_seconds, self.hold_seconds,
                self.max_log_bytes, self.log_name) != self._declaration:
            raise PiHttpTunnelError('tunnel_declaration_changed')

    def _command(self):
        self._check_declaration()
        script = f'import time; print({self._ready_line!r}, flush=True); time.sleep({self.hold_seconds})'
        command = ['ssh', '-T', '-oBatchMode=yes', '-oConnectTimeout=3',
                   '-oExitOnForwardFailure=yes', '-oControlMaster=no', '-oControlPath=none',
                   '-oServerAliveInterval=5', '-oServerAliveCountMax=1']
        for forward in self.forwards:
            command += ['-L', f"127.0.0.1:{forward['localPort']}:{forward['targetHost']}:80"]
        command += [self.pi_alias, 'python3 -u -c ' + shlex.quote(script)]
        return command

    def _drain(self, timeout=0.):
        if self._selector is None:
            return
        for key, _ in self._selector.select(timeout):
            name = key.data
            chunk = os.read(key.fileobj.fileno(), 4096)
            if not chunk:
                self._selector.unregister(key.fileobj)
                continue
            remaining = self.max_log_bytes-len(self._output[name])
            self._output[name].extend(chunk[:max(0, remaining)])
            if len(chunk) > remaining:
                raise PiHttpTunnelError('tunnel_output_limit_exceeded')
        marker = self._ready_line.encode('ascii') + b'\n'
        if marker in bytes(self._output['stdout']).splitlines(keepends=True):
            self._ready_seen = True

    def start(self):
        if self._used or self._closed:
            raise PiHttpTunnelError('tunnel_single_use')
        self._used = True
        self._started_at = self.clock.monotonic()
        try:
            self.process = subprocess.Popen(self._command(), stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0, start_new_session=True)
            self._selector = selectors.DefaultSelector()
            for name in ('stdout', 'stderr'):
                stream = getattr(self.process, name)
                os.set_blocking(stream.fileno(), False)
                self._selector.register(stream, selectors.EVENT_READ, name)
            deadline = self._started_at+self.startup_seconds
            while True:
                now = self.clock.monotonic()
                if now >= deadline:
                    raise PiHttpTunnelError('tunnel_ready_timeout')
                self._drain(min(.05, deadline-now))
                if self.process.poll() is not None:
                    self._drain()
                    raise PiHttpTunnelError('ssh_exited_before_tunnel_ready')
                if self._ready_seen:
                    self._ready_at = self.clock.monotonic()
                    if self._ready_at >= deadline:
                        raise PiHttpTunnelError('tunnel_ready_timeout')
                    self._ready = True
                    return self
        except BaseException as exc:
            self._failure = str(exc) if isinstance(exc, PiHttpTunnelError) else type(exc).__name__
            self.close()
            raise

    def check(self):
        if not self._ready or self._closed or self.process is None:
            raise PiHttpTunnelError('tunnel_not_active')
        try:
            self._check_declaration()
            self._drain()
            if self.process.poll() is not None:
                raise PiHttpTunnelError('tunnel_ssh_exited')
            if self.clock.monotonic()-self._ready_at >= self.hold_seconds:
                raise PiHttpTunnelError('tunnel_hold_expired')
        except BaseException as exc:
            self._failure = str(exc) if isinstance(exc, PiHttpTunnelError) else type(exc).__name__
            self.close()
            raise

    @property
    def summary(self):
        return dict(ready=self._ready, closed=self._closed, failure=self._failure,
                    returnCode=None if self.process is None else self.process.poll(),
                    stdoutBytes=len(self._output['stdout']), stderrBytes=len(self._output['stderr']),
                    cleanupErrors=list(self._cleanup_errors), deviceRequests=0,
                    readinessTcpConnections=0, automaticRetry=False)

    def close(self):
        if self._closed:
            return
        self._closed = True
        process = self.process
        try:
            if process is not None:
                if process.poll() is None:
                    try:
                        process.terminate()
                    except OSError as exc:
                        self._cleanup_errors.append(type(exc).__name__)
                    try:
                        process.wait(timeout=.5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=.5)
                else:
                    process.wait(timeout=.5)
                try:
                    for _ in range(34):
                        if self._selector is None or not self._selector.get_map():
                            break
                        self._drain()
                except (OSError, PiHttpTunnelError) as exc:
                    self._cleanup_errors.append(type(exc).__name__)
        except BaseException as exc:
            self._cleanup_errors.append(type(exc).__name__)
            self._failure = self._failure or 'tunnel_cleanup_failed'
            raise PiHttpTunnelError('tunnel_cleanup_failed') from exc
        finally:
            if self._selector is not None:
                self._selector.close()
            if process is not None:
                for name in ('stdout', 'stderr'):
                    stream = getattr(process, name, None)
                    if stream is not None:
                        stream.close()
            if self._used and self.store is not None and not self._logged:
                self._logged = True
                self.store.write_json(self.log_name, dict(self.summary,
                    stdout=self._log_text('stdout'), stderr=self._log_text('stderr')))

    def _log_text(self, name):
        text = self._output[name].decode('utf-8', errors='replace')
        return text.encode('utf-8')[:self.max_log_bytes].decode('utf-8', errors='ignore')

    def __enter__(self):
        return self.start()

    def __exit__(self, *args):
        self.close()
