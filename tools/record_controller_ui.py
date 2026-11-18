#!/usr/bin/env python3
"""동일 controller UI의 사용자 의도/전송 기록. 실행 자체로 모터 명령을 만들지 않는다.

두 카메라 passive recorder와 같은 호스트에서 병행 실행한다. 두 로그의
monotonic 시각은 같은 호스트 기준이며 브라우저 anchor는 왕복 구간이다.
전송 실패는 재시도하지 않는다. 로그는 실제 driver 출력/파지 성공을 뜻하지 않는다.
"""
import argparse
import copy
import math
import re
import os
import tempfile
import secrets
from contextlib import contextmanager, nullcontext
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
    "/manual_leases",
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
POST_PATHS = {"/command", "/motor", "/joint"}
JOINT_ACTIONS = {
    "gripper": {"open", "close"},
    "wrist": {"up", "down"},
    "elbow": {"up", "down"},
    "shoulder": {"up", "down"},
    "base": {"left", "right"},
}
QUERY_KEYS = {
    "lease",
    "joint",
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
    "gestureId",
    "controllerForwarded",
    "uiOutcome",
    "pendingRequestSequence",
    "commandId",
    "joint",
    "action",
    "percent",
    "sourceFunction",
    "domEventSequence",
    "relatedFailedRequestSequence",
    "httpStatus",
    "reasonCode",
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


MANUAL_LEASE_FIELDS = {"lease", "leaseId", "leaseSeq", "expectedBootId", "expectedGeneration"}


def validate_manual_lease_params(params, *, stopping):
    """Validate the new manual protocol without logging opaque lease tokens."""
    fields = set(params) & MANUAL_LEASE_FIELDS
    if stopping:
        if fields:
            raise ValueError("stop has no lease fields")
        return
    mode = params.get("lease", [None])[0]
    if mode == "start":
        if fields != {"lease", "expectedBootId", "expectedGeneration"}:
            raise ValueError("manual start fields")
        if not re.fullmatch("[0-9a-f]{32}", params["expectedBootId"][0]):
            raise ValueError("manual start boot")
        number = params["expectedGeneration"][0]
        if not re.fullmatch("0|[1-9][0-9]{0,9}", number) or int(number) > 2**32-1:
            raise ValueError("manual start generation")
    elif mode == "renew":
        if fields != {"lease", "leaseId", "leaseSeq"}:
            raise ValueError("manual renew fields")
        if not re.fullmatch("[0-9a-f]{32}-[1-5]-[0-9a-f]{8}", params["leaseId"][0]):
            raise ValueError("manual renew token")
        number = params["leaseSeq"][0]
        if not re.fullmatch("[1-9][0-9]{0,9}", number) or int(number) > 2**32-1:
            raise ValueError("manual renew sequence")
    else:
        raise ValueError("manual lease=start|renew required")


class EventLog:
    def __init__(self, directory, *, instrumentation=None):
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
            instrumentationSha256=hashlib.sha256(
                JS.read_bytes() if instrumentation is None else instrumentation
            ).hexdigest(),
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
    # A running capture serves exactly the bytes whose hash was recorded.
    # Editing the source on disk only affects a newly started server.
    instrumentation = JS.read_bytes()
    log = EventLog(output, instrumentation=instrumentation)
    motion_lock = threading.Lock()
    control_lock = threading.RLock()
    arm_finished = threading.Condition(control_lock)
    arm_inflight = False

    @contextmanager
    def forward_order(path, stopping):
        nonlocal arm_inflight
        if stopping:
            # A motor/joint STOP addresses its own axis, even if another axis
            # is active. Firmware invalidates covered START/RENEW requests.
            # Only ARM is unversioned and must finish before STOP/DISARM.
            with arm_finished:
                arm_finished.wait_for(lambda: not arm_inflight)
            yield
        else:
            with motion_lock:
                if path == "/command":
                    with arm_finished:
                        arm_inflight = True
                try:
                    yield
                finally:
                    if path == "/command":
                        with arm_finished:
                            arm_inflight = False
                            arm_finished.notify_all()

    control = dict(
        generation=0,
        stopPending=0,
        highest={},
        unknown=False,
        unownedStopAt=0.0,
        ownerPage=None,
        startPageId=None,
        startMonotonic=None,
    )

    signal_file = (
        Path(ready_file).parent / "operator-start.json"
        if ready_file is not None
        else None
    )

    def status_for(page_id, *, ignore_stop_pending=False):
        page = log.pages.get(page_id, {})
        reason = "browser_preparing"
        text = "브라우저 준비 중"
        can_start = False
        ended_file = (
            Path(ready_file).parent / "capture-ended.json"
            if ready_file is not None
            else None
        )
        state = {}
        try:
            if ready_file is not None and Path(ready_file).exists():
                with Path(ready_file).open("rb") as file:
                    raw = file.read(65537)
                if len(raw) > 65536:
                    raise ValueError("status size")
                state = json.loads(raw)
        except (OSError, ValueError, TypeError):
            reason = "capture_status_invalid"
            text = "수집 상태 파일 오류"
            return dict(
                recordingReady=False,
                startAllowed=False,
                reason=reason,
                reasonText=text,
                startRequested=control["startPageId"] is not None,
                startPageId=control["startPageId"],
            )
        if ended_file is not None and ended_file.exists():
            try:
                ended = json.loads(ended_file.read_bytes())
                failed = bool(ended.get("errors"))
            except (OSError, ValueError, TypeError):
                failed = True
            reason = "capture_failed" if failed else "capture_ended"
            text = (
                "수집 실패로 종료 · 새 episode가 필요합니다"
                if failed
                else "수집 종료 · 새 episode가 필요합니다"
            )
        elif state.get("errors"):
            reason = "capture_failed"
            text = "카메라/출력 수집 실패 · 새 동작 차단"
        elif log.degraded:
            reason = "ui_log_failed"
            text = "UI 기록 오류 · 새 동작 차단"
        elif control["unknown"]:
            reason = "transport_unknown"
            text = "명령 전송 결과 불명 · 새 동작 차단"
        elif control["stopPending"] and not ignore_stop_pending:
            reason = "stop_pending"
            text = "STOP 처리 중 · 새 동작 차단"
        elif control["ownerPage"] not in (None, page_id):
            reason = "other_tab"
            text = "다른 탭이 이 수집의 조작 탭입니다"
        elif not page.get("started"):
            pass
        elif not page.get("anchorAt"):
            reason = "clock_preparing"
            text = "브라우저 시각 연결 중"
        elif not 0 <= time.monotonic() - page.get("heartbeatAt", 0) <= 2:
            reason = "browser_stale"
            text = "브라우저 연결 지연 · 새 동작 차단"
        elif not 0 <= time.monotonic() - page.get("anchorAt", 0) <= 10:
            reason = "clock_stale"
            text = "브라우저 시각 기준 지연 · 새 동작 차단"
        elif page.get("startedAt", 0) <= control["unownedStopAt"]:
            reason = "page_invalidated"
            text = "외부 STOP 후 기존 탭 무효 · 새 수집 세션이 필요합니다"
        elif control["startPageId"] is None:
            if signal_file is not None and signal_file.exists():
                reason = "start_exists"
                text = (
                    "이 episode에 시작 기록이 이미 있습니다 · 새 episode가 필요합니다"
                )
            else:
                reason = "waiting_operator_start"
                text = "브라우저 준비 완료 · 수집 시작을 누르세요"
                can_start = signal_file is not None
        elif control["startPageId"] != page_id:
            reason = "other_tab"
            text = "다른 탭이 수집을 시작했습니다"
        elif not state:
            reason = "capture_starting"
            text = "수집 시작 요청됨 · 두 카메라와 출력 기록 준비 중"
        elif (
            state.get("operatorStartPageId") != control["startPageId"]
            or state.get("operatorStartMonotonic") != control["startMonotonic"]
            or type(state.get("at")) not in (int, float)
            or state["at"] <= control["startMonotonic"]
        ):
            reason = "capture_starting"
            text = "시작 요청 이후 두 카메라와 출력 기록 준비를 기다립니다"
        elif state.get("recording") is not True:
            reason = "capture_not_recording"
            text = "수집 대기 또는 종료 · 새 동작 차단"
        elif (
            type(state.get("at")) not in (int, float)
            or not math.isfinite(state["at"])
            or not 0 <= time.monotonic() - state["at"] <= 2
        ):
            reason = "capture_stale"
            text = "카메라/출력 기록 갱신 지연 · 새 동작 차단"
        else:
            reason = "recording"
            text = "수집 연결됨 · 수동 조작 가능"
        return dict(
            recordingReady=reason == "recording",
            startAllowed=can_start,
            reason=reason,
            reasonText=text,
            startRequested=control["startPageId"] is not None,
            startPageId=control["startPageId"],
        )

    def ready(page_id):
        return status_for(page_id)["recordingReady"]

    def operator_start(page_id):
        with control_lock:
            state = status_for(page_id)
            if not state["startAllowed"]:
                return False, state
            payload = dict(
                schemaVersion="motionbrain.operator_capture_start.v1",
                pageId=page_id,
                hostMonotonic=time.monotonic(),
                hostEpoch=time.time(),
            )
            # Publish complete JSON atomically, while preserving O_EXCL semantics.
            signal_file.parent.mkdir(parents=True, exist_ok=True)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(
                    mode="w",
                    dir=signal_file.parent,
                    prefix=".operator-start-",
                    delete=False,
                ) as file:
                    temporary = Path(file.name)
                    json.dump(payload, file)
                    file.write("\n")
                    file.flush()
                    os.fsync(file.fileno())
                os.link(temporary, signal_file)
            except FileExistsError:
                return False, dict(
                    reason="start_exists",
                    reasonText="시작 기록이 이미 있습니다 · 새 episode가 필요합니다",
                )
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            control["startPageId"] = page_id
            control["startMonotonic"] = payload["hostMonotonic"]
            control["ownerPage"] = page_id
            log.write(
                "operator_start_requested",
                pageId=page_id,
                signalFile=str(signal_file),
                agentMotorCommands=0,
            )
            return True, dict(
                reason="capture_starting",
                reasonText="수집 시작 요청됨 · 두 카메라와 출력 기록 준비 중",
            )

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

        def reply(self, status, raw, content="application/json", *, disposition=None):
            self.send_response(status)
            self.send_header("Content-Type", content)
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            if disposition is not None:
                self.send_header("X-MotionBrain-UI-Proxy-Disposition", disposition)
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
                                **status_for(page_id),
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
                if path == "/__capture/start" and self.command == "POST":
                    if self.headers.get("Origin") not in (
                        f"http://127.0.0.1:{self.server.server_port}",
                        f"http://localhost:{self.server.server_port}",
                    ):
                        raise ValueError("start origin")
                    request_body = json.loads(raw or b"")
                    if (
                        not isinstance(request_body, dict)
                        or set(request_body) != {"pageId"}
                        or not isinstance(request_body["pageId"], str)
                    ):
                        raise ValueError("start body")
                    started, result = operator_start(request_body["pageId"])
                    return self.reply(
                        201 if started else 409,
                        json.dumps(result, ensure_ascii=False).encode(),
                    )
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
                    lease_params = {k: v for k, v in params.items() if k in MANUAL_LEASE_FIELDS}
                    if path in ("/motor", "/joint"):
                        params = {k: v for k, v in params.items() if k not in MANUAL_LEASE_FIELDS}
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
                    elif path == "/joint":
                        if not {"joint", "action"} <= set(params) or set(params) - {
                            "joint",
                            "action",
                            "percent",
                        }:
                            raise ValueError("joint fields")
                        joint = params["joint"][0]
                        action = params["action"][0]
                        stopping = action == "stop"
                        if joint not in JOINT_ACTIONS and not (
                            joint == "all" and stopping
                        ):
                            raise ValueError("joint name")
                        if stopping:
                            if set(params) != {"joint", "action"}:
                                raise ValueError("joint stop fields")
                        elif (
                            action not in JOINT_ACTIONS[joint]
                            or set(params) != {"joint", "action", "percent"}
                            or params["percent"][0]
                            not in {str(n) for n in range(1, 101)}
                        ):
                            raise ValueError("joint action or percent")
                    if path in ("/motor", "/joint"):
                        validate_manual_lease_params(lease_params, stopping=stopping)
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
                            json.dumps(
                                dict(
                                    success=False,
                                    error="dual recording/browser clock not ready",
                                    reason=status_for(page_id)["reason"],
                                    reasonText=status_for(page_id)["reasonText"],
                                )
                            ).encode(),
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
                try:
                    order = (
                        forward_order(path, stopping)
                        if self.command == "POST" else nullcontext()
                    )
                    with order:
                        if self.command == "POST" and not stopping:
                            with control_lock:
                                independent_state = status_for(
                                    page_id, ignore_stop_pending=True
                                )
                                superseded = generation != control[
                                    "generation"
                                ] or control["highest"].get(page_id) != int(parts[1])
                                eligible = (
                                    not superseded
                                    and not control["stopPending"]
                                    and independent_state["recordingReady"]
                                )
                                neutralizable = (
                                    superseded and independent_state["recordingReady"]
                                )
                            if not eligible:
                                if neutralizable:
                                    # Only this branch proves that a reserved unique
                                    # request never entered the controller transport.
                                    payload = dict(
                                        schemaVersion="motionbrain.ui_proxy_command_disposition.v1",
                                        success=False,
                                        outcome="superseded_without_dispatch",
                                        controllerForwarded=False,
                                        independentGuardsPassed=True,
                                        captureId=capture_id,
                                        reason="superseded",
                                        reasonText="후속 입력 또는 STOP으로 취소되어 이 요청은 제어기에 보내지 않았습니다",
                                    )
                                    log.write(
                                        "proxy_superseded",
                                        captureId=capture_id,
                                        path=path,
                                        controllerForwarded=False,
                                    )
                                    if not log.degraded:
                                        return self.reply(
                                            409,
                                            json.dumps(
                                                payload, ensure_ascii=False
                                            ).encode(),
                                            disposition="superseded_without_dispatch",
                                        )
                                    independent_state = status_for(
                                        page_id, ignore_stop_pending=True
                                    )
                                state = (
                                    independent_state
                                    if not independent_state["recordingReady"]
                                    else status_for(page_id)
                                )
                                log.write(
                                    "proxy_guard_failed",
                                    captureId=capture_id,
                                    path=path,
                                    reason=state["reason"],
                                )
                                return self.reply(
                                    409,
                                    json.dumps(
                                        dict(
                                            success=False,
                                            error="recording ended before dispatch",
                                            reason=state["reason"],
                                            reasonText=state["reasonText"],
                                        ),
                                        ensure_ascii=False,
                                    ).encode(),
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
                                with control_lock:
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
                        b"<script>" + instrumentation + b"</script></body>",
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
        help=(
            "별도 sampler의 live-status.json. 수집 시작 버튼만 부모 경로에 "
            "operator-start.json(pageId, hostMonotonic)을 exclusive 생성합니다. "
            "녹화 heartbeat는 recording=true, 오류 없음, at이 시작 이후이면서 age<=2s, "
            "operatorStartPageId=signal.pageId 및 operatorStartMonotonic=signal.hostMonotonic 필수. "
            "이 도구는 camera recorder를 직접 기동하지 않습니다."
        ),
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
