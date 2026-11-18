#!/usr/bin/env python3
"""동일 controller UI의 사용자 의도/전송 기록. 실행 자체로 모터 명령을 만들지 않는다.

두 카메라 passive recorder와 같은 호스트에서 병행 실행한다. 두 로그의
monotonic 시각은 같은 호스트 기준이며 브라우저 anchor는 왕복 구간이다.
전송 실패는 재시도하지 않는다. 로그는 실제 driver 출력/파지 성공을 뜻하지 않는다.
"""
import argparse
import copy
import math
import secrets
from contextlib import nullcontext
import hashlib
import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

GET_PATHS = {
    "/",
    "/status",
    "/events",
    "/vision_endpoint",
    "/actions",
    "/action_outputs",
    "/collection_status",
    "/collection_observation",
    "/m4_diagnostics",
    "/finite_profile",
    "/sequence",
    "/routine",
    "/favicon.ico",
}
POST_PATHS = {"/command", "/motor"}
QUERY_KEYS = {
    "id",
    "action",
    "percent",
    "cmd",
    "direction",
    "duration",
    "durationMs",
    "speed",
    "after",
    "limit",
}
EVENT_KEYS = {
    "kind",
    "pageId",
    "sequence",
    "browserPerformanceMs",
    "browserEpochMs",
    "eventType",
    "targetId",
    "pointerId",
    "clientX",
    "clientY",
    "value",
    "functionName",
    "motorId",
    "direction",
    "mode",
    "path",
    "query",
    "requestSequence",
    "success",
    "browserBeforeMs",
    "browserAfterMs",
    "hostReceivedMonotonic",
    "hostSentMonotonic",
    "instrumentationVersion",
    "isTrusted",
    "captureId",
    "anchorId",
}
JS = Path(__file__).with_name("controller_ui_capture.js")


def safe_target(target):
    u = urllib.parse.urlsplit(target)
    if (
        u.scheme
        or u.netloc
        or u.fragment
        or "\\" in target
        or "%" in u.path
        or ".." in u.path
    ):
        raise ValueError("relative target required")
    query = urllib.parse.parse_qs(u.query, keep_blank_values=True)
    return u.path, {k: v for k, v in query.items() if k in QUERY_KEYS}


class EventLog:
    def __init__(self, directory):
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True)
        self.file = (self.root / "ui-events.jsonl").open("x")
        self.lock = threading.RLock()
        self.sequence = 0
        self.seen = {}
        self.last_browser = 0.0
        self.has_anchor = False
        self.page_started = False
        self.degraded = False
        self.pages = {}
        self.dispatched = set()
        self.clocks = {}
        self.write(
            "capture_started",
            instrumentationSha256=hashlib.sha256(JS.read_bytes()).hexdigest(),
            agentMotorCommands=0,
            automaticRetry=False,
            datasetEligible=False,
        )

    def write(self, kind, **fields):
        with self.lock:
            self.sequence += 1
            try:
                self.file.write(
                    json.dumps(
                        dict(
                            schemaVersion="motionbrain.controller_ui_capture.v1",
                            sequence=self.sequence,
                            kind=kind,
                            hostMonotonic=time.monotonic(),
                            hostEpoch=time.time(),
                            **fields,
                        ),
                        ensure_ascii=False,
                        allow_nan=False,
                    )
                    + "\n"
                )
                self.file.flush()
            except OSError:
                self.degraded = True

    def issue_clock(self, page_id, received):
        if (
            not isinstance(page_id, str)
            or not page_id
            or len(page_id) > 64
            or not all(c.isascii() and (c.isalnum() or c == "-") for c in page_id)
        ):
            raise ValueError("clock page")
        sample = dict(
            anchorId=secrets.token_hex(16),
            hostReceivedMonotonic=received,
            hostSentMonotonic=time.monotonic(),
        )
        with self.lock:
            self.clocks[sample["anchorId"]] = (page_id, sample.copy())
            if len(self.clocks) > 4096:
                self.degraded = True
                raise ValueError("clock sample budget")
        return sample

    def browser(self, batch):
        # A delayed/reordered batch cannot refresh command authority merely by arriving.
        now = time.monotonic()
        try:
            if not isinstance(batch, dict) or set(batch) - {
                "events",
                "dropped",
                "unflushed",
            }:
                raise ValueError("batch")
            if type(batch.get("dropped", 0)) is not int or batch.get("dropped", 0) < 0:
                raise ValueError("drop count")
            if batch.get("dropped", 0):
                self.degraded = True
            events = batch.get("events")
            if not isinstance(events, list) or not 1 <= len(events) <= 64:
                raise ValueError("batch size")
            with self.lock:
                pages = copy.deepcopy(self.pages)
                seen = dict(self.seen)
                fresh = []
                for e in events:
                    if not isinstance(e, dict) or set(e) - EVENT_KEYS:
                        raise ValueError("event keys")
                    page_id = e.get("pageId")
                    seq = e.get("sequence")
                    if (
                        not isinstance(page_id, str)
                        or not page_id
                        or len(page_id) > 64
                        or not all(
                            c.isascii() and (c.isalnum() or c == "-") for c in page_id
                        )
                        or type(seq) is not int
                        or seq < 1
                    ):
                        raise ValueError("identity")
                    if e.get("query") is not None and (
                        not isinstance(e["query"], dict) or set(e["query"]) - QUERY_KEYS
                    ):
                        raise ValueError("query")
                    encoded = json.dumps(e, allow_nan=False, sort_keys=True)
                    if len(encoded) > 4096:
                        raise ValueError("event size")
                    stamp = e.get("browserPerformanceMs")
                    if (
                        type(stamp) not in (int, float)
                        or not math.isfinite(stamp)
                        or stamp < 0
                    ):
                        raise ValueError("event timestamp")
                    identity = (page_id, seq)
                    if identity in seen:
                        if seen[identity] != encoded:
                            raise ValueError("duplicate metadata changed")
                        continue
                    page = pages.setdefault(
                        page_id,
                        dict(
                            started=False,
                            heartbeatAt=0.0,
                            anchorAt=0.0,
                            lastEventSequence=0,
                            lastPerformanceMs=-1.0,
                        ),
                    )
                    if (
                        seq != page["lastEventSequence"] + 1
                        or stamp < page["lastPerformanceMs"]
                    ):
                        raise ValueError("event sequence or clock order")
                    if not page["started"] and not (
                        seq == 1 and e.get("kind") == "page_started"
                    ):
                        raise ValueError("page start required")
                    if e.get("kind") == "page_started":
                        if seq != 1:
                            raise ValueError("duplicate page start")
                        page["started"] = True
                        page["startedAt"] = now
                    if e.get("kind") == "clock_anchor":
                        keys = (
                            "browserBeforeMs",
                            "browserAfterMs",
                            "hostReceivedMonotonic",
                            "hostSentMonotonic",
                        )
                        if any(
                            type(e.get(k)) not in (int, float)
                            or not math.isfinite(e[k])
                            for k in keys
                        ):
                            raise ValueError("anchor timestamp")
                        before, after, received, sent = (e[k] for k in keys)
                        issued = self.clocks.get(e.get("anchorId"))
                        if (
                            not issued
                            or issued[0] != page_id
                            or any(issued[1][k] != e[k] for k in keys[2:])
                        ):
                            raise ValueError("unissued or different page anchor")
                        if (
                            not 0 <= before <= after <= stamp
                            or not received <= sent <= now
                            or after - before + 2 < (sent - received) * 1000
                        ):
                            raise ValueError("anchor causal order")
                        # 1ms browser timer quantization allowance; this is a clock mapping,
                        # never an optical-exposure synchronization claim.
                        lo = sent - after / 1000 - 0.001
                        hi = received - before / 1000 + 0.001
                        if lo > hi:
                            raise ValueError("anchor interval")
                        if sent >= page["anchorAt"]:
                            page["offsetBounds"] = [lo, hi]
                            page["anchorAt"] = sent
                    page["lastEventSequence"] = seq
                    page["lastPerformanceMs"] = stamp
                    seen[identity] = encoded
                    fresh.append(e)
                bounds = []
                for e in fresh:
                    page = pages[e["pageId"]]
                    if "offsetBounds" not in page:
                        continue
                    lo, hi = page["offsetBounds"]
                    source_lo = e["browserPerformanceMs"] / 1000 + lo - 0.001
                    source_hi = e["browserPerformanceMs"] / 1000 + hi + 0.001
                    if source_lo > now + 0.01:
                        raise ValueError("future event source")
                    bounds.append(
                        dict(
                            pageId=e["pageId"],
                            sequence=e["sequence"],
                            sourceEarliest=source_lo,
                            sourceLatest=source_hi,
                        )
                    )
                    if e["kind"] == "page_started":
                        page["startedAt"] = source_lo
                    if e["kind"] == "browser_heartbeat":
                        # A legitimate prompt/backlog delay is preserved, but never
                        # renews command authority. A later fresh sample can recover.
                        if 0 <= now - source_lo <= 2:
                            page["heartbeatAt"] = max(page["heartbeatAt"], source_lo)
                self.pages = pages
                self.seen = seen
                self.last_browser = now
                self.has_anchor = any(p["anchorAt"] for p in pages.values())
                self.page_started = any(p["started"] for p in pages.values())
                self.write(
                    "browser_batch",
                    events=fresh,
                    eventHostBounds=bounds,
                    dropped=batch.get("dropped", 0),
                    unflushed=batch.get("unflushed", 0),
                )
        except (ValueError, TypeError, KeyError):
            self.degraded = True
            raise

    def close(self):
        self.file.close()


def make_server(controller, output, port=8768, *, ready_file=None, opener=None):
    origin = urllib.parse.urlsplit(controller)
    if (
        origin.scheme != "http"
        or not origin.hostname
        or origin.path not in ("", "/")
        or origin.query
        or origin.fragment
        or origin.username
        or origin.password
    ):
        raise ValueError("plain controller HTTP origin required")
    base = controller.rstrip("/")
    log = EventLog(output)
    forward_lock = threading.Lock()
    control_lock = threading.RLock()
    control = dict(
        generation=0,
        stopPending=0,
        highest={},
        unknown=False,
        unownedStopAt=0.0,
        ownerPage=None,
    )

    def ready(page_id):
        try:
            if ready_file is None:
                return False
            raw = Path(ready_file).read_bytes()
            if len(raw) > 65536:
                return False
            state = json.loads(raw)
            age = time.monotonic() - state["at"]
            page = log.pages.get(page_id, {})
            return bool(
                state.get("recording") is True
                and not state.get("errors")
                and 0 <= age <= 2
                and not log.degraded
                and not control["unknown"]
                and not control["stopPending"]
                and control["ownerPage"] in (None, page_id)
                and page.get("started")
                and page.get("startedAt", 0) > control["unownedStopAt"]
                and 0 <= time.monotonic() - page.get("heartbeatAt", 0) <= 2
                and 0 <= time.monotonic() - page.get("anchorAt", 0) <= 10
            )
        except (OSError, ValueError, KeyError, TypeError):
            return False

    # No environment proxy, redirects, or retry for mutating requests.
    class NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *args, **kwargs):
            return None

    transport = (
        opener
        or urllib.request.build_opener(
            urllib.request.ProxyHandler({}), NoRedirect()
        ).open
    )

    class Handler(BaseHTTPRequestHandler):
        timeout = 5

        def log_message(self, *args):
            pass

        def reply(self, status, raw, content="application/json"):
            self.send_response(status)
            self.send_header("Content-Type", content)
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            self.handle_capture()

        def do_POST(self):
            self.handle_capture()

        def handle_capture(self):
            received = time.monotonic()
            try:
                expected = {
                    f"127.0.0.1:{self.server.server_port}",
                    f"localhost:{self.server.server_port}",
                }
                if self.headers.get("Host") not in expected:
                    raise ValueError("host")
                path, query = safe_target(self.path)
                if path == "/__capture/status" and self.command == "GET":
                    page_id = urllib.parse.parse_qs(
                        urllib.parse.urlsplit(self.path).query
                    ).get("pageId", [None])[0]
                    return self.reply(
                        200,
                        json.dumps(
                            dict(
                                recordingReady=ready(page_id),
                                browserConnected=page_id in log.pages,
                                clockAnchorStored=bool(
                                    log.pages.get(page_id, {}).get("anchorAt")
                                ),
                                pages=[
                                    dict(pageId=key, recordingReady=ready(key))
                                    for key in list(log.pages)
                                ],
                            )
                        ).encode(),
                    )
                if path == "/__capture/clock" and self.command == "GET":
                    page_id = urllib.parse.parse_qs(
                        urllib.parse.urlsplit(self.path).query
                    ).get("pageId", [None])[0]
                    return self.reply(
                        200, json.dumps(log.issue_clock(page_id, received)).encode()
                    )
                length = int(self.headers.get("Content-Length", "0"))
                if (
                    length < 0
                    or length > 65536
                    or self.headers.get("Transfer-Encoding")
                ):
                    raise ValueError("body size")
                raw = self.rfile.read(length) if length else None
                if path == "/__capture/events" and self.command == "POST":
                    if self.headers.get("Origin") not in (
                        None,
                        f"http://127.0.0.1:{self.server.server_port}",
                        f"http://localhost:{self.server.server_port}",
                    ):
                        raise ValueError("origin")
                    log.browser(json.loads(raw or b""))
                    return self.reply(200, b'{"stored":true}')
                allowed = GET_PATHS if self.command == "GET" else POST_PATHS
                if path not in allowed:
                    return self.reply(404, b'{"error":"route"}')
                # Same-origin browser commands retain original CSRF/token checks at the controller.
                if self.command == "POST" and self.headers.get("Origin") not in (
                    f"http://127.0.0.1:{self.server.server_port}",
                    f"http://localhost:{self.server.server_port}",
                ):
                    raise ValueError("command origin")
                if self.command == "POST":
                    params = urllib.parse.parse_qs(
                        urllib.parse.urlsplit(self.path).query, keep_blank_values=True
                    )
                    if raw or any(len(v) != 1 for v in params.values()):
                        raise ValueError("query-only commands required")
                    stopping = False
                    if path == "/command":
                        if set(params) != {"cmd"} or params["cmd"][0] not in (
                            "arm",
                            "disarm",
                            "stop",
                        ):
                            raise ValueError("command")
                        stopping = params["cmd"][0] in ("disarm", "stop")
                    elif path == "/motor":
                        if not {"action", "id"} <= set(params) or set(params) - {
                            "action",
                            "id",
                            "percent",
                        }:
                            raise ValueError("motor fields")
                        if params["id"][0] not in ("1", "2", "3", "4", "5") or params[
                            "action"
                        ][0] not in ("forward", "reverse", "stop"):
                            raise ValueError("motor command")
                        stopping = params["action"][0] == "stop"
                        if stopping and set(params) != {"action", "id"}:
                            raise ValueError("stop fields")
                        if not stopping and (
                            set(params) != {"action", "id", "percent"}
                            or params["percent"][0]
                            not in {str(n) for n in range(1, 101)}
                        ):
                            raise ValueError("motor percent")
                    capture_id = self.headers.get("X-UI-Capture-ID")
                    page_id = None
                    if capture_id is not None and not stopping:
                        parts = capture_id.rsplit(":", 1)
                        if (
                            len(parts) != 2
                            or not parts[0]
                            or len(parts[0]) > 64
                            or not all(c.isalnum() or c == "-" for c in parts[0])
                            or not parts[1].isascii()
                            or not parts[1].isdigit()
                            or not 0 < int(parts[1]) < 10**12
                            or str(int(parts[1])) != parts[1]
                        ):
                            raise ValueError("capture identity")
                        page_id = parts[0]
                    elif capture_id is not None:
                        try:
                            parts = capture_id.rsplit(":", 1)
                            if (
                                len(parts) == 2
                                and parts[1].isdigit()
                                and parts[0] in log.pages
                            ):
                                page_id = parts[0]
                            else:
                                capture_id = None
                        except (ValueError, TypeError):
                            capture_id = None
                    if not stopping and (capture_id is None or not ready(page_id)):
                        log.write(
                            "command_blocked_not_recording", path=path, query=query
                        )
                        return self.reply(
                            409,
                            b'{"success":false,"error":"dual recording/browser clock not ready"}',
                        )
                    with control_lock:
                        if stopping:
                            control["generation"] += 1
                            control["stopPending"] += 1
                            if page_id is None or control["ownerPage"] not in (
                                None,
                                page_id,
                            ):
                                control["unownedStopAt"] = time.monotonic()
                            elif int(parts[1]) > control["highest"].get(page_id, 0):
                                control["highest"][page_id] = int(parts[1])
                        else:
                            if control["stopPending"] or control["ownerPage"] not in (
                                None,
                                page_id,
                            ):
                                return self.reply(
                                    409,
                                    b'{"success":false,"error":"STOP pending or different controlling page"}',
                                )
                            if capture_id in log.dispatched or int(parts[1]) <= control[
                                "highest"
                            ].get(page_id, 0):
                                return self.reply(
                                    409,
                                    b'{"success":false,"error":"stale or duplicate capture identity; no retry"}',
                                )
                            control["ownerPage"] = page_id
                            control["highest"][page_id] = int(parts[1])
                            log.dispatched.add(capture_id)
                        generation = control["generation"]
                headers = {
                    key: self.headers[key]
                    for key in ("X-MotionBrain", "X-MotionBrain-Token", "Content-Type")
                    if key in self.headers
                }
                headers["Connection"] = "close"
                if self.command != "POST":
                    capture_id = None
                request = urllib.request.Request(
                    base + self.path, data=raw, headers=headers, method=self.command
                )
                log.write(
                    "proxy_request",
                    method=self.command,
                    path=path,
                    query=query,
                    captureId=capture_id,
                    requestReceivedMonotonic=received,
                    bodyPresent=raw is not None,
                )
                # Serialize actual controller writes. STOP admission cancels queued older
                # writes before this lock; an already in-flight write must finish/timeout
                # before STOP can be sent. urllib timeout=3 is an individual socket
                # I/O timeout, NOT a whole-request wall-clock bound. No instant STOP
                # latency or wire-level ordering after an unknown transport is claimed.
                try:
                    with forward_lock if self.command == "POST" else nullcontext():
                        if self.command == "POST" and not stopping:
                            with control_lock:
                                eligible = (
                                    generation == control["generation"]
                                    and not control["stopPending"]
                                    and control["highest"].get(page_id) == int(parts[1])
                                    and ready(page_id)
                                )
                            if not eligible:
                                return self.reply(
                                    409,
                                    b'{"success":false,"error":"superseded or recording ended before dispatch"}',
                                )
                        log.write(
                            "proxy_dispatch",
                            method=self.command,
                            path=path,
                            captureId=capture_id,
                        )
                        if self.command == "POST" and not stopping and log.degraded:
                            return self.reply(
                                409, b'{"success":false,"error":"capture log failed"}'
                            )
                        try:
                            try:
                                response = transport(request, timeout=3)
                            except urllib.error.HTTPError as e:
                                response = e
                            with response:
                                body = response.read(2_000_001)
                                status = response.code
                                content = response.headers.get(
                                    "Content-Type", "application/octet-stream"
                                )
                            if len(body) > 2_000_000:
                                raise ValueError("upstream size")
                        except Exception:
                            if self.command == "POST":
                                control["unknown"] = True
                            raise
                        log.write(
                            "proxy_response",
                            method=self.command,
                            path=path,
                            captureId=capture_id,
                            httpStatus=status,
                            requestReceivedMonotonic=received,
                        )
                finally:
                    if self.command == "POST" and stopping:
                        with control_lock:
                            control["stopPending"] -= 1
                if path == "/" and status == 200:
                    if b"</body>" not in body:
                        raise ValueError("HTML injection anchor missing")
                    body = body.replace(
                        b"</body>",
                        b"<script>" + JS.read_bytes() + b"</script></body>",
                        1,
                    )
                self.reply(status, body, content)
            except Exception as e:
                log.write(
                    "capture_or_proxy_error",
                    errorType=type(e).__name__,
                    requestReceivedMonotonic=received,
                    automaticRetry=False,
                )
                self.reply(
                    502,
                    b'{"error":"capture_or_upstream_failure","automaticRetry":false}',
                )

    try:
        server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    except BaseException:
        log.close()
        raise
    server.capture_log = log
    return server


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--controller", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--port", type=int, default=8768)
    p.add_argument(
        "--ready-file",
        required=True,
        help="Passive live-status.json: recording=true, at=host monotonic, errors empty; age <=2s",
    )
    p.add_argument("--prepare", action="store_true")
    a = p.parse_args()
    if a.prepare:
        print(
            json.dumps(
                dict(
                    hardwareRequests=0,
                    instrumentationSha256=hashlib.sha256(JS.read_bytes()).hexdigest(),
                    browserURL=f"http://127.0.0.1:{a.port}/",
                )
            )
        )
        return
    server = make_server(a.controller, a.output, a.port, ready_file=a.ready_file)
    print(
        f"UI_CAPTURE_READY http://127.0.0.1:{server.server_port}/ (open this UI; run dual-camera recorder separately)",
        flush=True,
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        server.capture_log.write("capture_ended", agentMotorCommands=0)
        server.capture_log.close()


if __name__ == "__main__":
    main()
