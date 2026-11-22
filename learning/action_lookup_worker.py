"""Optional process that receives GET identities over IPC and sends no authentication."""
from contextlib import contextmanager
from multiprocessing import get_context
from threading import Lock
import json
import select
import socket
from urllib.parse import urlsplit

from learning.action_execution import canonical_request, checked_record

from learning.controller_latency import deadline
from learning.device_adapter import DeviceReadAdapter, DeviceReadError, ReadOnlyHTTP


MAX_PACKET_BYTES = 2048


def _origin_key(origin):
    url = urlsplit(origin)
    if (url.scheme not in ('http','https') or not url.hostname or url.username is not None
            or url.password is not None or url.query or url.fragment or url.path not in ('','/')
            or url.port == 0 or any(c.isspace() for c in url.netloc)):
        raise ValueError('invalid_controller_origin')
    return (url.scheme, url.hostname.lower(), url.port or (443 if url.scheme=='https' else 80))


def _send(channel, value):
    payload = json.dumps(value, allow_nan=False, separators=(',',':')).encode()
    if len(payload) > MAX_PACKET_BYTES or channel.send(payload) != len(payload):
        raise ValueError('invalid_lookup_packet')


def _receive(channel):
    payload = channel.recv(MAX_PACKET_BYTES+1)
    if not payload or len(payload)>MAX_PACKET_BYTES:
        raise DeviceReadError('lookup_packet_unverified')
    return json.loads(payload)


def _lookup(reader, request, timeout):
    # The spawned main thread can impose a total HTTP deadline.
    with deadline(timeout):
        outcome = reader.lookup_action(request)
        if outcome.get('lookupStatus') != 'matched_record':
            raise DeviceReadError('action_lookup_unverified')
        record = checked_record(outcome['record'],request)
        if len(record['reason']) > 64:
            raise DeviceReadError('action_lookup_reason_too_long')
        return record


def _serve(channel, origin, timeout):
    with channel:
        transport = ReadOnlyHTTP(controller_url=origin, camera_url=origin,
                                 perception_url=origin, timeout=timeout)
        reader = DeviceReadAdapter(transport)
        _send(channel, ['ready',_origin_key(transport.bases['controller'])])
        while True:
            request = canonical_request(_receive(channel))
            try:
                record = _lookup(reader, request, timeout)
            except Exception:
                record = None
            _send(channel, [request['bootId'], request['commandId'], record])


class _Pending:
    def __init__(self, worker, binding):
        self.worker, self.binding = worker, binding

    def done(self):
        return bool(select.select([self.worker.channel],[],[],0)[0]) or not self.worker.process.is_alive()

    def result(self, timeout):
        if timeout != 0 or not self.worker.process.is_alive():
            raise DeviceReadError('lookup_worker_response_missing')
        boot, cid, record = _receive(self.worker.channel)
        if (boot, cid) != self.binding or record is None:
            raise DeviceReadError('lookup_worker_response_unverified')
        self.worker.pending = None
        return record


class _Lease:
    def __init__(self, worker, request):
        self.worker, self.request = worker, request
        self.binding = (request['bootId'],request['commandId'])

    def check_ready(self, owner_origin):
        w = self.worker
        if (w.owner != self.binding or w.pending or not w.process.is_alive()
                or w.bound_origin != _origin_key(owner_origin)):
            raise ValueError('lookup_worker_not_ready')

    def begin(self):
        with self.worker.lock:
            self.check_ready(self.worker.origin)
            _send(self.worker.channel,self.request)
            self.worker.pending = _Pending(self.worker,self.binding)
            return self.worker.pending


class ActionLookupWorker:
    """Reserve for one command before dispatch. Close after DISARM, outside the active loop."""
    def __init__(self, origin, timeout=.16):
        if type(timeout) not in (int, float) or not 0 < timeout <= .2:
            raise ValueError('invalid_lookup_timeout')
        self.origin, self.timeout = origin, timeout
        self.bound_origin = None
        _origin_key(origin)
        self.process = self.channel = self.pending = self.owner = None
        self.lock = Lock()

    def __enter__(self):
        with self.lock:
            if self.process is not None:
                raise ValueError('lookup_worker_already_started')
            context = get_context('spawn')
            # Message boundaries are preserved by the kernel. A partial pipe frame
            # can appear readable while its payload is still being sent.
            self.channel, child = socket.socketpair(type=socket.SOCK_DGRAM)
            self.channel.setblocking(False)
            self.process = context.Process(target=_serve, args=(child,self.origin,self.timeout), daemon=True)
            try:
                self.process.start()
                child.close()
                if (not select.select([self.channel],[],[],5)[0] or
                        _receive(self.channel) != ['ready',list(_origin_key(self.origin))]):
                    raise ValueError('lookup_worker_startup_failed')
                self.bound_origin = _origin_key(self.origin)
            except BaseException:
                child.close()
                self._shutdown()
                raise
        return self

    @contextmanager
    def reserve(self, request, owner_origin):
        binding = (request['bootId'], request['commandId'])
        with self.lock:
            if _origin_key(owner_origin) != self.bound_origin and self.process is not None:
                raise ValueError('lookup_controller_origin_mismatch')
            if self.owner or self.process is None or not self.process.is_alive():
                raise ValueError('lookup_worker_not_ready')
            if self.pending:
                if not self.pending.done():
                    raise ValueError('previous_lookup_still_in_flight')
                # A verified cancellation can leave one obsolete GET response queued.
                # Consume and validate it before assigning the pipe to another action.
                self.pending.result(timeout=0)
            self.owner = binding

        try:
            yield _Lease(self,request.copy())
        finally:
            with self.lock:
                self.owner = None

    def _shutdown(self):
        process = self.process
        try:
            if process and process.pid is not None and process.is_alive():
                # No controller write occurs in this process; in-flight GETs may be abandoned.
                process.terminate()
                process.join(.5)
                if process.is_alive():
                    process.kill()
                    process.join(.5)
                if process.is_alive():
                    raise RuntimeError('lookup_worker_shutdown_unconfirmed')
        finally:
            if self.channel:
                self.channel.close()
            if process and not process.is_alive():
                process.close()
            self.channel = self.process = self.pending = None

    def __exit__(self, *args):
        with self.lock:
            if self.owner:
                raise ValueError('lookup_worker_reserved')
            self._shutdown()
