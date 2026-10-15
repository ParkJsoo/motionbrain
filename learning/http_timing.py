"""Opt-in, per-request HTTP diagnostics. No global socket patches or retries.

Uses the same urllib/http.client path as the read-only adapter. DNS here means
getaddrinfo wall time (numeric IP parsing is included), not DNS wire latency.
TCP includes address fallback and local scheduling. No TLS/proxy support.
"""
import http.client
import socket
import urllib.request


SEND_START_SEMANTICS = "direct_http_first_send_before_socket_send_v1"


class TimedHTTPHandler(urllib.request.HTTPHandler):
    def http_open(self, req):
        # Request-local state keeps the reusable opener free of shared traces.
        clock, trace = req.motionbrain_clock, req.motionbrain_connection_timing

        class Connection(http.client.HTTPConnection):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self._create_connection = self.create_timed_connection

            def create_timed_connection(self, address, timeout, source_address):
                trace["resolveStartedMonotonic"] = clock.monotonic()
                addresses = socket.getaddrinfo(*address, 0, socket.SOCK_STREAM)
                trace["resolveCompletedMonotonic"] = clock.monotonic()
                trace["addressAttempts"] = 0
                last_error = OSError("getaddrinfo returns an empty list")
                for family, socktype, proto, _, target in addresses:
                    sock = None
                    trace["addressAttempts"] += 1
                    try:
                        sock = socket.socket(family, socktype, proto)
                        sock.settimeout(timeout)
                        if source_address:
                            sock.bind(source_address)
                        sock.connect(target)
                        trace["connectedMonotonic"] = clock.monotonic()
                        return sock
                    except OSError as exc:
                        last_error = exc
                        if sock is not None:
                            sock.close()
                    except BaseException:
                        if sock is not None:
                            sock.close()
                        raise
                raise last_error

            def send(self, data):
                if self.sock is None:
                    self.connect()
                if "requestSendStartedMonotonic" not in trace:
                    trace["requestSendStartedMonotonic"] = clock.monotonic()
                    trace["sendStartSemantics"] = SEND_START_SEMANTICS
                super().send(data)
                trace["requestSendCompletedMonotonic"] = clock.monotonic()

        return self.do_open(Connection, req)


TRACE_STAMPS = ("resolveStartedMonotonic", "resolveCompletedMonotonic", "connectedMonotonic",
                "requestSendStartedMonotonic", "requestSendCompletedMonotonic")


def connection_metrics(receipt):
    """Return disjoint client intervals; never narrow the source-age bound."""
    t = receipt["connectionTiming"]
    start, resolved, connected, sending, sent = (t[k] for k in TRACE_STAMPS)
    return {"clientSetupMs": (start-receipt["requestStartedMonotonic"])*1000,
            "addressResolutionMs": (resolved-start)*1000,
            "tcpConnectMs": (connected-resolved)*1000,
            "connectedToSendMs": (sending-connected)*1000,
            "requestSendMs": (sent-sending)*1000,
            "requestSentToHeadersMs": (receipt["headersAvailableMonotonic"]-sent)*1000}
