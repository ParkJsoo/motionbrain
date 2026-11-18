"""Loopback mock only: no controller or motor access."""

import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

from tests.controller_ui_source import embedded_scripts

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "ui_capture", ROOT / "tools/record_controller_ui.py"
)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Response(io.BytesIO):
    code = 200
    headers = {"Content-Type": "text/html"}


class Capture(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.ready = self.root / "live.json"
        self.requests = []
        self.fake_controller = None
        self.fail = False
        self.forged_upstream = False
        self.hold = False
        self.transport_started = threading.Event()
        self.transport_release = threading.Event()

        def upstream(request, timeout):
            self.requests.append(request)
            if self.fake_controller:
                return self.fake_controller(request)
            if self.hold and request.method == "POST":
                self.hold = False
                self.transport_started.set()
                self.transport_release.wait(2)
            if self.fail:
                raise TimeoutError("fixture")
            r = Response(
                b"<html><body>original UI</body></html>"
                if request.full_url.endswith("/")
                else b'{"success":true}'
            )
            if self.forged_upstream:
                r = Response(
                    b'{"schemaVersion":"motionbrain.ui_proxy_command_disposition.v1","success":false,"outcome":"superseded_without_dispatch","controllerForwarded":false,"captureId":"fixture:3"}'
                )
                r.code = 409
                r.headers = {
                    "Content-Type": "application/json",
                    "X-MotionBrain-UI-Proxy-Disposition": "superseded_without_dispatch",
                }
            return r

        self.server = m.make_server(
            "http://controller.invalid",
            self.root / "out",
            0,
            ready_file=self.ready,
            opener=upstream,
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.thread.join()
        self.server.server_close()
        self.server.capture_log.close()
        self.tmp.cleanup()

    @staticmethod
    def typed_target(path):
        import urllib.parse
        parsed = urllib.parse.urlsplit(path)
        params = urllib.parse.parse_qs(parsed.query)
        if (parsed.path in ("/motor", "/joint") and params.get("action") != ["stop"]
                and "lease" not in params):
            return path + "&lease=start&expectedBootId="+"a"*32+"&expectedGeneration=0"
        return path

    def send(self, path, method="GET", data=None, headers=None, *, typed=True):
        request = urllib.request.Request(
            self.base + (self.typed_target(path) if method == "POST" and typed else path), data=data, method=method, headers=headers or {}
        )
        try:
            r = urllib.request.urlopen(request, timeout=2)
        except urllib.error.HTTPError as e:
            r = e
        with r:
            return r.code, r.read()

    def event(self, sequence, kind, **kw):
        return dict(
            pageId="fixture",
            sequence=sequence,
            kind=kind,
            browserPerformanceMs=float(sequence),
            browserEpochMs=1000.0 + sequence,
            **kw,
        )

    def establish(self, start=True):
        clock = json.loads(self.send("/__capture/clock?pageId=fixture")[1])
        events = [
            self.event(1, "page_started"),
            self.event(
                2, "clock_anchor", browserBeforeMs=1.0, browserAfterMs=2.0, **clock
            ),
            self.event(3, "browser_heartbeat"),
        ]
        self.assertEqual(
            self.send(
                "/__capture/events",
                "POST",
                json.dumps(dict(events=events)).encode(),
                {"Origin": self.base},
            )[0],
            200,
        )
        if start:
            self.assertEqual(
                self.send(
                    "/__capture/start",
                    "POST",
                    json.dumps({"pageId": "fixture"}).encode(),
                    {"Origin": self.base},
                )[0],
                201,
            )
        signal = (
            json.loads((self.root / "operator-start.json").read_bytes())
            if start
            else {}
        )
        self.ready.write_text(
            json.dumps(
                dict(
                    recording=True,
                    at=time.monotonic(),
                    errors=[],
                    operatorStartPageId=signal.get("pageId"),
                    operatorStartMonotonic=signal.get("hostMonotonic"),
                )
            )
        )

    def post(self, path, **kw):
        return self.send(
            path,
            "POST",
            headers={
                "Origin": self.base,
                "X-MotionBrain": "1",
                "X-MotionBrain-Token": "PRIVATE-FIXTURE",
                "X-UI-Capture-ID": "fixture:3",
                **kw,
            },
        )

    def logs(self):
        return [
            json.loads(s)
            for s in (self.root / "out/ui-events.jsonl").read_text().splitlines()
        ]

    def test_snapshot_get_and_typed_renew_preserve_fields_without_logging_opaque_token(self):
        self.assertEqual(self.send("/manual_leases")[0], 200)
        self.assertEqual(self.requests[-1].full_url, "http://controller.invalid/manual_leases")
        self.establish()
        token = "a"*32+"-3-00000001"
        target = "/motor?action=forward&id=3&percent=70&lease=renew&leaseId="+token+"&leaseSeq=2"
        self.assertEqual(self.post(target)[0], 200)
        self.assertEqual(self.requests[-1].full_url, "http://controller.invalid"+target)
        rows = self.logs()
        request = next(r for r in rows if r["kind"] == "proxy_request" and r["path"] == "/motor")
        self.assertEqual(request["query"]["lease"], ["renew"])
        self.assertNotIn(token, json.dumps(rows))
        self.assertNotIn("PRIVATE-FIXTURE", json.dumps(rows))

    def test_untyped_legacy_motor_and_joint_commands_fail_closed(self):
        self.establish()
        before = len(self.requests)
        for target in ("/motor?action=forward&id=3&percent=60", "/joint?joint=elbow&action=down&percent=60"):
            with self.subTest(target=target):
                code, _ = self.send(target, "POST", headers={"Origin": self.base,
                    "X-UI-Capture-ID": "fixture:4", "X-MotionBrain": "1", "X-MotionBrain-Token": "PRIVATE-FIXTURE"}, typed=False)
                self.assertEqual(code, 502)  # Existing malformed-command response; never forwarded.
        self.assertEqual(len(self.requests), before)
        self.assertEqual(self.post("/motor?action=stop&id=3")[0], 200)

    def test_prepare_no_hardware_and_no_output(self):
        out = self.root / "prepare"
        r = subprocess.run(
            [
                "python3",
                str(ROOT / "tools/record_controller_ui.py"),
                "--controller",
                "http://controller.invalid",
                "--output",
                str(out),
                "--ready-file",
                str(self.ready),
                "--prepare",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertEqual(json.loads(r.stdout)["hardwareRequests"], 0)
        self.assertFalse(out.exists())
        self.assertFalse(self.requests)

    def test_original_html_instrumented_without_commands(self):
        status, body = self.send("/")
        self.assertEqual(status, 200)
        self.assertIn(b"original UI", body)
        self.assertIn(b"command_dispatch", body)
        self.assertEqual(len(self.requests), 1)
        self.assertEqual(self.requests[0].method, "GET")

    def test_no_recording_or_browser_blocks_motion_stop_survives(self):
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 409)
        self.assertFalse(self.requests)
        self.assertEqual(self.post("/motor?action=stop&id=3")[0], 200)
        self.assertEqual(len(self.requests), 1)
        self.assertEqual(self.post("/command?cmd=disarm")[0], 200)

    def test_exact_forward_auth_correlation_no_secret_log(self):
        self.establish()
        self.assertEqual(self.post("/motor?action=reverse&id=1&percent=70")[0], 200)
        r = self.requests[0]
        self.assertEqual(r.method, "POST")
        self.assertEqual(r.get_header("X-motionbrain-token"), "PRIVATE-FIXTURE")
        self.assertIsNone(r.get_header("X-ui-capture-id"))
        self.assertEqual(
            r.full_url, "http://controller.invalid"+self.typed_target("/motor?action=reverse&id=1&percent=70")
        )
        raw = (self.root / "out/ui-events.jsonl").read_text()
        self.assertNotIn("PRIVATE-FIXTURE", raw)
        self.assertEqual(
            [r for r in self.logs() if r["kind"] == "proxy_request"][0]["captureId"],
            "fixture:3",
        )

    def test_end_error_stale_future_and_browser_loss_block_run(self):
        self.establish()
        for value in [
            dict(recording=False, at=time.monotonic()),
            dict(recording=True, at=time.monotonic(), errors=["capture"]),
            dict(recording=True, at=time.monotonic() - 3),
            dict(recording=True, at=time.monotonic() + 3),
        ]:
            self.ready.write_text(json.dumps(value))
            self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 409)
        self.ready.write_text(json.dumps(dict(recording=True, at=time.monotonic())))
        self.server.capture_log.pages["fixture"]["heartbeatAt"] -= 3
        self.assertEqual(self.post("/command?cmd=arm")[0], 409)
        self.assertFalse(self.requests)
        self.assertEqual(self.post("/command?cmd=stop")[0], 200)

    def test_no_retry_on_unknown_upstream(self):
        self.establish()
        self.fail = True
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 502)
        self.assertEqual(len(self.requests), 1)
        self.assertEqual(self.logs()[-1]["automaticRetry"], False)

    def test_host_origin_allowlist_and_query_rejection(self):
        self.establish()
        for path, kw in [
            (
                "/motor?action=forward&id=3&percent=60",
                {"Origin": "http://evil.invalid"},
            ),
            ("/motor?action=stop&id=3", {"Host": "evil.invalid"}),
            ("/motor?action=forward&id=3&id=4&percent=60", {}),
            ("/motor?action=forward&id=3&percent=60&token=secret", {}),
            ("/command?cmd=reboot", {}),
            ("/motor?action=forward&id=3&percent=060", {}),
            ("/%2fmotor?action=stop&id=3", {}),
            ("/joint?action=down", {}),
        ]:
            self.assertNotEqual(self.post(path, **kw)[0], 200)
        self.assertFalse(self.requests)

    def test_capture_identity_required_page_bound_and_single_forward(self):
        self.establish()
        self.assertEqual(
            self.send(
                "/motor?action=forward&id=3&percent=60",
                "POST",
                headers={
                    "Origin": self.base,
                    "X-MotionBrain": "1",
                    "X-MotionBrain-Token": "PRIVATE-FIXTURE",
                },
            )[0],
            409,
        )
        for capture_id in ("", "otherpage:1", "fixture:0003"):
            self.assertNotEqual(
                self.post(
                    "/motor?action=forward&id=3&percent=60",
                    **{"X-UI-Capture-ID": capture_id},
                )[0],
                200,
            )
        self.assertEqual(len(self.requests), 0)
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 200)
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 409)
        self.assertEqual(len(self.requests), 1)
        self.assertEqual(
            self.send(
                "/motor?action=stop&id=3",
                "POST",
                headers={
                    "Origin": self.base,
                    "X-MotionBrain": "1",
                    "X-MotionBrain-Token": "PRIVATE-FIXTURE",
                },
            )[0],
            200,
        )

    def test_per_page_anchor_and_heartbeat_cannot_be_mixed(self):
        self.establish()
        self.server.capture_log.pages["fixture"]["anchorAt"] -= 11
        clock = json.loads(self.send("/__capture/clock?pageId=other")[1])
        batch = dict(
            events=[
                dict(self.event(1, "page_started"), pageId="other"),
                dict(
                    self.event(
                        2,
                        "clock_anchor",
                        browserBeforeMs=1.0,
                        browserAfterMs=2.0,
                        **clock,
                    ),
                    pageId="other",
                ),
                dict(self.event(3, "browser_heartbeat"), pageId="other"),
            ]
        )
        self.send("/__capture/events", "POST", json.dumps(batch).encode())
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 409)
        self.assertEqual(
            self.post(
                "/motor?action=forward&id=3&percent=60",
                **{"X-UI-Capture-ID": "other:4"},
            )[0],
            409,
        )
        self.server.capture_log.pages["other"]["heartbeatAt"] -= 3
        self.assertEqual(
            self.post(
                "/motor?action=forward&id=3&percent=60",
                **{"X-UI-Capture-ID": "other:5"},
            )[0],
            409,
        )

    def test_disk_failure_blocks_motion_but_stop_still_forwarded(self):
        self.establish()
        real = self.server.capture_log.file

        class Broken:
            def write(self, *args):
                raise OSError("disk fixture")

            def flush(self):
                pass

        self.server.capture_log.file = Broken()
        try:
            self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 409)
            self.assertEqual(len(self.requests), 0)
            self.assertEqual(self.post("/motor?action=stop&id=3")[0], 200)
            self.assertEqual(len(self.requests), 1)
        finally:
            self.server.capture_log.file = real

    def test_stop_bypasses_inflight_lease_and_discards_queued_lease(self):
        self.establish()
        self.hold = True
        answers = {}

        proofs = {}

        def request(label, path, capture):
            req = urllib.request.Request(
                self.base + self.typed_target(path),
                method="POST",
                headers={
                    "Origin": self.base,
                    "X-UI-Capture-ID": capture,
                    "X-MotionBrain": "1",
                    "X-MotionBrain-Token": "PRIVATE-FIXTURE",
                },
            )
            try:
                response = urllib.request.urlopen(req, timeout=2)
            except urllib.error.HTTPError as error:
                response = error
            with response:
                answers[label] = response.code
                proofs[label] = (
                    response.headers.get("X-MotionBrain-UI-Proxy-Disposition"),
                    json.loads(response.read()),
                )

        first = threading.Thread(
            target=request,
            args=("first", "/motor?action=forward&id=3&percent=60", "fixture:4"),
        )
        first.start()
        self.assertTrue(self.transport_started.wait(1))
        lease = threading.Thread(
            target=request,
            args=("lease", "/motor?action=forward&id=3&percent=70", "fixture:5"),
        )
        lease.start()
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            if any(
                r.get("captureId") == "fixture:5" and r["kind"] == "proxy_request"
                for r in self.logs()
            ):
                break
            time.sleep(0.005)
        stop = threading.Thread(
            target=request, args=("stop", "/motor?action=stop&id=3", "fixture:6")
        )
        stop.start()
        stop.join(1)
        self.assertEqual(answers.get("stop"), 200)
        self.assertTrue(first.is_alive())
        self.transport_release.set()
        for t in (first, lease, stop):
            t.join(2)
            self.assertFalse(t.is_alive())
        self.assertEqual(answers, {"first": 200, "lease": 409, "stop": 200})
        marker, body = proofs["lease"]
        self.assertEqual(marker, "superseded_without_dispatch")
        self.assertEqual(body["captureId"], "fixture:5")
        self.assertIs(body["controllerForwarded"], False)
        self.assertIs(body["success"], False)
        self.assertNotIn("commandId", body)
        self.assertTrue(
            any(
                row["kind"] == "proxy_superseded" and row["captureId"] == "fixture:5"
                for row in self.logs()
            )
        )
        self.assertFalse(
            any(
                row["kind"] == "proxy_dispatch" and row["captureId"] == "fixture:5"
                for row in self.logs()
            )
        )
        self.assertEqual(
            [r.full_url for r in self.requests],
            [
                "http://controller.invalid"+self.typed_target("/motor?action=forward&id=3&percent=60"),
                "http://controller.invalid/motor?action=stop&id=3",
            ],
        )
        self.assertEqual(
            self.post(
                "/motor?action=forward&id=3&percent=60",
                **{"X-UI-Capture-ID": "fixture:5"},
            )[0],
            409,
        )
        self.assertEqual(
            self.post(
                "/motor?action=forward&id=3&percent=60",
                **{"X-UI-Capture-ID": "fixture:7"},
            )[0],
            200,
        )

    def async_post(self, path, sequence):
        result = []
        thread = threading.Thread(target=lambda: result.append(
            self.post(path, **{"X-UI-Capture-ID": f"fixture:{sequence}"})[0]
        ))
        thread.start()
        return thread, result

    def test_release_before_slow_shoulder_start_is_processed(self):
        # Episode 07: release arrived 408ms after START; START took 1106ms.
        self.establish()
        started = threading.Event()
        generation = 1
        pwm_writes = []

        def slow_controller(request):
            nonlocal generation
            params = urllib.parse.parse_qs(urllib.parse.urlsplit(request.full_url).query)
            response = Response(b'{"success":true}')
            if params["action"] == ["stop"]:
                generation += 1
                pwm_writes.append(0)
            else:
                started.set()
                time.sleep(1.106)
                if int(params["expectedGeneration"][0]) != generation:
                    response = Response(b'{"success":false,"error":"generation mismatch"}')
                    response.code = 409
                else:
                    pwm_writes.append(179)
            return response

        self.fake_controller = slow_controller
        path = "/joint?joint=shoulder&action=up&percent=70&lease=start&expectedBootId=" + "a"*32 + "&expectedGeneration=1"
        thread, result = self.async_post(path, 4)
        self.assertTrue(started.wait(1))
        time.sleep(0.408)
        self.assertEqual(self.post("/joint?joint=shoulder&action=stop",
                                   **{"X-UI-Capture-ID": "fixture:5"})[0], 200)
        self.assertTrue(thread.is_alive())
        thread.join(2)
        self.assertEqual(result, [409])
        self.assertEqual(pwm_writes, [0])
        rows = self.logs()
        stop_sent = next(r["hostMonotonic"] for r in rows
                         if r["kind"] == "proxy_dispatch" and r.get("captureId") == "fixture:5")
        start_reply = next(r["hostMonotonic"] for r in rows
                           if r["kind"] == "proxy_response" and r.get("captureId") == "fixture:4")
        self.assertLess(stop_sent, start_reply)

    def test_other_axis_stop_and_disarm_bypass_slow_renewal(self):
        self.establish()
        self.hold = True
        path = "/motor?id=4&action=forward&percent=70&lease=renew&leaseId=" + "a"*32 + "-4-00000002&leaseSeq=1"
        renew, result = self.async_post(path, 4)
        self.assertTrue(self.transport_started.wait(1))
        try:
            for sequence, path in enumerate(("/motor?id=3&action=stop", "/command?cmd=disarm"), 5):
                stop, stopped = self.async_post(path, sequence)
                stop.join(0.8)
                self.assertEqual(stopped, [200])
                self.assertTrue(renew.is_alive())
        finally:
            self.transport_release.set()
            renew.join(2)
        self.assertEqual(result, [200])

    def test_disarm_orders_unversioned_arm_and_invalidates_queued_motion(self):
        self.establish()
        self.hold = True
        arm, armed = self.async_post("/command?cmd=arm", 4)
        self.assertTrue(self.transport_started.wait(1))
        queued, motion = self.async_post("/joint?joint=shoulder&action=up&percent=70", 5)
        def wait_admitted(sequence):
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                if any(r["kind"] == "proxy_request" and r.get("captureId") == f"fixture:{sequence}"
                       for r in self.logs()):
                    return
                time.sleep(0.005)
            raise AssertionError(f"request {sequence} did not arrive")
        wait_admitted(5)
        disarm, disarmed = self.async_post("/command?cmd=disarm", 6)
        wait_admitted(6)
        try:
            self.assertTrue(disarm.is_alive())
            self.assertEqual(len(self.requests), 1)
        finally:
            self.transport_release.set()
            for thread in (arm, queued, disarm):
                thread.join(2)
        self.assertEqual((armed, motion, disarmed), ([200], [409], [200]))
        self.assertEqual([r.full_url.split("?")[1] for r in self.requests],
                         ["cmd=arm", "cmd=disarm"])

    def test_unknown_transport_blocks_followup_run_but_not_stop(self):
        self.establish()
        self.fail = True
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 502)
        self.fail = False
        self.assertEqual(
            self.post(
                "/motor?action=forward&id=3&percent=60",
                **{"X-UI-Capture-ID": "fixture:4"},
            )[0],
            409,
        )
        self.assertEqual(self.post("/motor?action=stop&id=3")[0], 200)

    def test_unowned_stop_invalidates_existing_page(self):
        self.establish()
        self.assertEqual(
            self.send("/command?cmd=stop", "POST", headers={"Origin": self.base})[0],
            200,
        )
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 409)

    def test_bad_anchor_timestamp_and_unissued_identity_rejected(self):
        self.establish()
        clock = json.loads(self.send("/__capture/clock?pageId=fixture")[1])
        base = self.event(
            4, "clock_anchor", browserBeforeMs=3.0, browserAfterMs=4.0, **clock
        )
        from unittest.mock import patch

        variants = [
            {"browserBeforeMs": None},
            {"browserAfterMs": float("nan")},
            {"hostSentMonotonic": float("inf")},
            {"browserBeforeMs": 5.0},
            {"browserAfterMs": 6.0},
            {"anchorId": "notissued"},
        ]
        for change in variants:
            with self.subTest(change=change):
                with self.assertRaises((ValueError, TypeError)):
                    self.server.capture_log.browser(dict(events=[dict(base, **change)]))
        with patch.object(m.time, "monotonic", return_value=time.monotonic() + 11):
            self.server.capture_log.browser(dict(events=[base]))
        self.assertEqual(self.post("/motor?action=stop&id=3")[0], 200)

    def test_delayed_heartbeat_and_out_of_order_event_sequences_rejected(self):
        self.establish()
        log = self.server.capture_log
        for e in [
            self.event(5, "browser_heartbeat"),
            dict(self.event(4, "browser_heartbeat"), browserPerformanceMs=1.0),
        ]:
            with self.assertRaises(ValueError):
                log.browser(dict(events=[e]))
        from unittest.mock import patch

        now = time.monotonic()
        with patch.object(m.time, "monotonic", return_value=now + 3):
            log.browser(dict(events=[self.event(4, "browser_heartbeat")]))
        self.assertEqual(self.post("/motor?action=stop&id=3")[0], 200)

    def test_new_run_during_inflight_stop_is_rejected_at_admission(self):
        self.establish()
        self.hold = True
        answers = {}
        t = threading.Thread(
            target=lambda: answers.setdefault(
                "stop",
                self.post(
                    "/motor?action=stop&id=3", **{"X-UI-Capture-ID": "fixture:4"}
                )[0],
            )
        )
        t.start()
        self.assertTrue(self.transport_started.wait(1))
        self.assertEqual(
            self.post(
                "/motor?action=forward&id=3&percent=60",
                **{"X-UI-Capture-ID": "fixture:5"},
            )[0],
            409,
        )
        self.transport_release.set()
        t.join(2)
        self.assertEqual(answers["stop"], 200)
        self.assertEqual(len(self.requests), 1)

    def test_only_one_controlling_page_and_foreign_stop_invalidates_owner(self):
        self.establish()
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 200)
        clock = json.loads(self.send("/__capture/clock?pageId=other")[1])
        events = [
            dict(self.event(1, "page_started"), pageId="other"),
            dict(
                self.event(
                    2, "clock_anchor", browserBeforeMs=1.0, browserAfterMs=2.0, **clock
                ),
                pageId="other",
            ),
            dict(self.event(3, "browser_heartbeat"), pageId="other"),
        ]
        self.assertEqual(
            self.send(
                "/__capture/events", "POST", json.dumps(dict(events=events)).encode()
            )[0],
            200,
        )
        self.assertEqual(
            self.post(
                "/motor?action=forward&id=3&percent=60",
                **{"X-UI-Capture-ID": "other:4"},
            )[0],
            409,
        )
        self.assertEqual(
            self.post("/motor?action=stop&id=3", **{"X-UI-Capture-ID": "other:5"})[0],
            200,
        )
        self.assertEqual(
            self.post(
                "/motor?action=forward&id=3&percent=60",
                **{"X-UI-Capture-ID": "fixture:4"},
            )[0],
            409,
        )

    def test_delayed_valid_heartbeat_kept_without_authority_then_fresh_recovers(self):
        self.establish()
        log = self.server.capture_log
        old = log.pages["fixture"]["heartbeatAt"]
        from unittest.mock import patch

        later = time.monotonic() + 3
        with patch.object(m.time, "monotonic", return_value=later):
            log.browser(dict(events=[self.event(4, "browser_heartbeat")]))
            self.assertEqual(log.pages["fixture"]["heartbeatAt"], old)
            self.assertFalse(log.degraded)
            clock = log.issue_clock("fixture", later)
            log.browser(
                dict(
                    events=[
                        self.event(
                            5,
                            "clock_anchor",
                            browserBeforeMs=4.0,
                            browserAfterMs=5.0,
                            **clock,
                        ),
                        self.event(6, "browser_heartbeat"),
                    ]
                )
            )
        self.assertGreater(log.pages["fixture"]["heartbeatAt"], old + 2)
        self.assertFalse(log.degraded)
        self.assertTrue(
            any(
                e["sequence"] == 4
                for r in self.logs()
                if r["kind"] == "browser_batch"
                for e in r["events"]
            )
        )

    def test_page_visit_and_auth_ready_do_not_start_collection(self):
        self.establish(start=False)
        self.assertFalse((self.root / "operator-start.json").exists())
        state = json.loads(self.send("/__capture/status?pageId=fixture")[1])
        self.assertEqual(state["reason"], "waiting_operator_start")
        self.assertTrue(state["startAllowed"])
        self.assertFalse(state["recordingReady"])
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 409)
        self.assertEqual(
            self.send(
                "/__capture/start",
                "POST",
                json.dumps({"pageId": "fixture"}).encode(),
                {"Origin": self.base},
            )[0],
            201,
        )
        signal = self.root / "operator-start.json"
        before = signal.read_bytes()
        self.assertEqual(json.loads(before)["pageId"], "fixture")
        self.assertEqual(
            self.send(
                "/__capture/start",
                "POST",
                json.dumps({"pageId": "fixture"}).encode(),
                {"Origin": self.base},
            )[0],
            409,
        )
        self.assertEqual(signal.read_bytes(), before)
        self.assertFalse(self.requests)

    def test_start_requires_origin_fresh_browser_and_never_overwrites_ended_episode(
        self,
    ):
        body = json.dumps({"pageId": "fixture"}).encode()
        self.assertEqual(
            self.send(
                "/__capture/start", "POST", body, {"Origin": "http://evil.invalid"}
            )[0],
            502,
        )
        self.assertEqual(
            self.send("/__capture/start", "POST", body, {"Origin": self.base})[0], 409
        )
        self.establish(start=False)
        (self.root / "capture-ended.json").write_text(json.dumps({"errors": []}))
        self.assertEqual(
            self.send("/__capture/start", "POST", body, {"Origin": self.base})[0], 409
        )
        self.assertFalse((self.root / "operator-start.json").exists())
        self.assertEqual(
            json.loads(self.send("/__capture/status?pageId=fixture")[1])["reason"],
            "capture_ended",
        )
        (self.root / "capture-ended.json").write_text(
            json.dumps({"errors": ["camera fixture failure"]})
        )
        self.assertEqual(
            json.loads(self.send("/__capture/status?pageId=fixture")[1])["reason"],
            "capture_failed",
        )
        self.assertFalse(self.requests)

    def test_start_needs_new_matching_sampler_heartbeat(self):
        self.establish(start=False)
        self.assertEqual(
            self.send(
                "/__capture/start",
                "POST",
                json.dumps({"pageId": "fixture"}).encode(),
                {"Origin": self.base},
            )[0],
            201,
        )
        signal = json.loads((self.root / "operator-start.json").read_bytes())
        self.assertEqual(
            json.loads(self.send("/__capture/status?pageId=fixture")[1])["reason"],
            "capture_starting",
        )
        for page, stamp, at in [
            ("fixture", signal["hostMonotonic"], signal["hostMonotonic"] - 0.001),
            ("other", signal["hostMonotonic"], time.monotonic()),
            ("fixture", signal["hostMonotonic"] - 0.01, time.monotonic()),
        ]:
            self.ready.write_text(
                json.dumps(
                    dict(
                        recording=True,
                        at=at,
                        operatorStartPageId=page,
                        operatorStartMonotonic=stamp,
                    )
                )
            )
            self.assertFalse(
                json.loads(self.send("/__capture/status?pageId=fixture")[1])[
                    "recordingReady"
                ]
            )
        self.ready.write_text(
            json.dumps(
                dict(
                    recording=True,
                    at=time.monotonic(),
                    operatorStartPageId="fixture",
                    operatorStartMonotonic=signal["hostMonotonic"],
                )
            )
        )
        self.assertTrue(
            json.loads(self.send("/__capture/status?pageId=fixture")[1])[
                "recordingReady"
            ]
        )
        self.assertFalse(self.requests)

    def test_joint_explicit_action_matrix_and_all_stop_preserve_query(self):
        self.establish()
        sequence = 4
        for joint, actions in m.JOINT_ACTIONS.items():
            for action in sorted(actions):
                self.assertEqual(
                    self.post(
                        f"/joint?joint={joint}&action={action}&percent=60",
                        **{"X-UI-Capture-ID": f"fixture:{sequence}"},
                    )[0],
                    200,
                )
                sequence += 1
        self.assertEqual(len(self.requests), 10)
        rows = [
            r
            for r in self.logs()
            if r["kind"] == "proxy_request" and r["path"] == "/joint"
        ]
        self.assertEqual(len(rows), 10)
        self.assertEqual(rows[0]["query"]["joint"], ["gripper"])
        self.assertEqual(
            self.post(
                "/joint?joint=all&action=stop",
                **{"X-UI-Capture-ID": f"fixture:{sequence}"},
            )[0],
            200,
        )
        self.assertEqual(
            self.post(
                "/joint?joint=elbow&action=stop", **{"X-UI-Capture-ID": "fixture:999"}
            )[0],
            200,
        )

    def test_joint_invalid_combinations_missing_id_and_duplicate_rejected(self):
        self.establish()
        for query in [
            "joint=gripper&action=up&percent=60",
            "joint=elbow&action=open&percent=60",
            "joint=base&action=up&percent=60",
            "joint=all&action=left&percent=60",
            "joint=wrong&action=stop",
            "joint=elbow&action=down&percent=0",
            "joint=elbow&action=down&percent=101",
            "joint=elbow&action=down&percent=060",
            "joint=elbow&action=down",
            "joint=elbow&action=stop&percent=0",
            "joint=elbow&joint=wrist&action=stop",
        ]:
            self.assertNotEqual(self.post("/joint?" + query)[0], 200)
        self.assertFalse(self.requests)
        path = "/joint?joint=elbow&action=down&percent=60"
        self.assertEqual(self.send(path, "POST", headers={"Origin": self.base})[0], 409)
        self.assertEqual(self.post(path)[0], 200)
        self.assertEqual(self.post(path)[0], 409)
        self.assertEqual(len(self.requests), 1)

    def test_joint_stop_survives_not_ready_unknown_and_stop_pending_blocks_run(self):
        self.assertEqual(
            self.send(
                "/joint?joint=all&action=stop", "POST", headers={"Origin": self.base}
            )[0],
            200,
        )
        # Leave more than the declared browser timer-quantization interval.
        time.sleep(0.01)
        self.establish()
        self.hold = True
        answers = {}
        t = threading.Thread(
            target=lambda: answers.setdefault(
                "stop",
                self.post(
                    "/joint?joint=elbow&action=stop", **{"X-UI-Capture-ID": "fixture:4"}
                )[0],
            )
        )
        t.start()
        self.assertTrue(self.transport_started.wait(1))
        self.assertEqual(
            self.post(
                "/joint?joint=elbow&action=down&percent=60",
                **{"X-UI-Capture-ID": "fixture:5"},
            )[0],
            409,
        )
        self.transport_release.set()
        t.join(2)
        self.assertEqual(answers["stop"], 200)
        self.fail = True
        self.assertEqual(
            self.post(
                "/joint?joint=elbow&action=down&percent=60",
                **{"X-UI-Capture-ID": "fixture:6"},
            )[0],
            502,
        )
        self.fail = False
        self.assertEqual(
            self.post(
                "/joint?joint=all&action=stop", **{"X-UI-Capture-ID": "fixture:7"}
            )[0],
            200,
        )

    def test_blocked_joint_exposes_actual_recording_reason(self):
        self.establish(start=False)
        code, raw = self.post("/joint?joint=elbow&action=down&percent=60")
        self.assertEqual(code, 409)
        data = json.loads(raw)
        self.assertEqual(data["reason"], "waiting_operator_start")
        self.assertIn("수집 시작", data["reasonText"])
        self.assertFalse(self.requests)

    def test_injected_javascript_is_frozen_to_recorded_startup_hash(self):
        import hashlib
        from unittest.mock import patch

        original = m.JS.read_bytes()
        replacement = self.root / "changed-ui.js"
        replacement.write_bytes(b"CHANGED_AFTER_SERVER_START")
        with patch.object(m, "JS", replacement):
            code, body = self.send("/")
            self.assertEqual(code, 200)
            self.assertIn(b"<script>" + original + b"</script>", body)
            self.assertNotIn(replacement.read_bytes(), body)
            replacement.unlink()
            self.assertEqual(self.send("/")[0], 200)
        recorded = next(row for row in self.logs() if row["kind"] == "capture_started")
        self.assertEqual(
            recorded["instrumentationSha256"], hashlib.sha256(original).hexdigest()
        )

    def test_upstream_cannot_forge_local_supersession_header(self):
        self.establish()
        self.forged_upstream = True
        request = urllib.request.Request(
            self.base + self.typed_target("/joint?joint=elbow&action=down&percent=60"),
            method="POST",
            headers={
                "Origin": self.base,
                "X-UI-Capture-ID": "fixture:3",
                "X-MotionBrain": "1",
                "X-MotionBrain-Token": "PRIVATE-FIXTURE",
            },
        )
        try:
            response = urllib.request.urlopen(request, timeout=2)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            self.assertEqual(response.code, 409)
            self.assertIsNone(
                response.headers.get("X-MotionBrain-UI-Proxy-Disposition")
            )
            self.assertEqual(
                json.loads(response.read())["outcome"], "superseded_without_dispatch"
            )
        self.assertEqual(len(self.requests), 1)
        self.assertTrue(
            any(
                r["kind"] == "proxy_dispatch" and r["captureId"] == "fixture:3"
                for r in self.logs()
            )
        )

    def queued_supersession_with_guard_failure(self, kind):
        self.establish()
        self.hold = True
        answers = {}
        proofs = {}

        def request(label, sequence):
            request = urllib.request.Request(
                self.base + self.typed_target("/joint?joint=elbow&action=down&percent=60"),
                method="POST",
                headers={
                    "Origin": self.base,
                    "X-UI-Capture-ID": f"fixture:{sequence}",
                    "X-MotionBrain": "1",
                    "X-MotionBrain-Token": "PRIVATE-FIXTURE",
                },
            )
            try:
                response = urllib.request.urlopen(request, timeout=2)
            except urllib.error.HTTPError as error:
                response = error
            with response:
                answers[label] = response.code
                proofs[label] = (
                    response.headers.get("X-MotionBrain-UI-Proxy-Disposition"),
                    json.loads(response.read()),
                )

        first = threading.Thread(target=request, args=("first", 4))
        first.start()
        self.assertTrue(self.transport_started.wait(1))
        others = []
        for label, sequence in [("lease", 5), ("newer", 6)]:
            t = threading.Thread(target=request, args=(label, sequence))
            t.start()
            others.append(t)
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                if any(
                    row["kind"] == "proxy_request"
                    and row.get("captureId") == f"fixture:{sequence}"
                    for row in self.logs()
                ):
                    break
                time.sleep(0.005)
            self.assertTrue(
                any(
                    row["kind"] == "proxy_request"
                    and row.get("captureId") == f"fixture:{sequence}"
                    for row in self.logs()
                )
            )
        if kind == "capture":
            state = json.loads(self.ready.read_bytes())
            state["recording"] = False
            self.ready.write_text(json.dumps(state))
        elif kind == "log":
            self.server.capture_log.degraded = True
        elif kind == "unknown":
            self.fail = True
        elif kind == "log_at_disposition":
            original_write = self.server.capture_log.write

            def failing_write(event, **fields):
                original_write(event, **fields)
                if event == "proxy_superseded":
                    self.server.capture_log.degraded = True

            self.server.capture_log.write = failing_write
        self.transport_release.set()
        for t in [first, *others]:
            t.join(2)
            self.assertFalse(t.is_alive())
        self.assertEqual(answers["lease"], 409)
        self.assertEqual(answers["newer"], 409)
        self.assertEqual(len(self.requests), 1)
        self.assertIsNone(proofs["lease"][0])
        self.assertNotIn("outcome", proofs["lease"][1])
        self.assertNotIn("independentGuardsPassed", proofs["lease"][1])
        self.assertFalse(proofs["lease"][1]["success"])
        expected = {
            "capture": "capture_not_recording",
            "log": "ui_log_failed",
            "unknown": "transport_unknown",
            "log_at_disposition": "ui_log_failed",
        }[kind]
        self.assertEqual(proofs["lease"][1]["reason"], expected)
        if kind != "log_at_disposition":
            self.assertFalse(
                any(
                    row["kind"] == "proxy_superseded"
                    and row.get("captureId") == "fixture:5"
                    for row in self.logs()
                )
            )
        self.assertTrue(
            any(
                row["kind"] == "proxy_guard_failed"
                and row.get("captureId") == "fixture:5"
                and row["reason"] == expected
                for row in self.logs()
            )
        )
        self.fail = False
        self.assertEqual(
            self.post(
                "/joint?joint=elbow&action=stop", **{"X-UI-Capture-ID": "fixture:7"}
            )[0],
            200,
        )

    def test_capture_failure_takes_priority_over_supersession(self):
        self.queued_supersession_with_guard_failure("capture")

    def test_log_failure_takes_priority_over_supersession(self):
        self.queued_supersession_with_guard_failure("log")

    def test_unknown_transport_takes_priority_over_supersession(self):
        self.queued_supersession_with_guard_failure("unknown")

    def test_log_failure_while_recording_disposition_is_not_neutral(self):
        self.queued_supersession_with_guard_failure("log_at_disposition")

    def test_events_deduplicated_and_bad_metadata_rejected(self):
        self.establish()
        batch = dict(
            events=[
                self.event(
                    4, "dom_intent", eventType="pointerdown", targetId="btn-forward-3"
                )
            ]
        )
        for _ in range(2):
            self.assertEqual(
                self.send("/__capture/events", "POST", json.dumps(batch).encode())[0],
                200,
            )
        self.assertEqual(
            sum(len(e["events"]) for e in self.logs() if e["kind"] == "browser_batch"),
            4,
        )
        batch["events"][0]["token"] = "secret"
        self.assertEqual(
            self.send("/__capture/events", "POST", json.dumps(batch).encode())[0], 502
        )

    def test_drop_count_permanently_blocks_new_motion(self):
        self.establish()
        self.send(
            "/__capture/events",
            "POST",
            json.dumps(
                dict(events=[self.event(3, "browser_heartbeat")], dropped=1)
            ).encode(),
        )
        self.assertEqual(self.post("/motor?action=forward&id=3&percent=60")[0], 409)
        self.assertEqual(self.post("/motor?action=stop&id=3")[0], 200)

    def test_clock_bracket_and_original_query_guards(self):
        before = time.monotonic()
        code, raw = self.send("/__capture/clock?pageId=fixture")
        after = time.monotonic()
        d = json.loads(raw)
        self.assertEqual(code, 200)
        self.assertLessEqual(before, d["hostReceivedMonotonic"])
        self.assertLessEqual(d["hostReceivedMonotonic"], d["hostSentMonotonic"])
        self.assertLessEqual(d["hostSentMonotonic"], after)
        for p in [
            "http://evil.invalid/motor",
            "//evil.invalid/motor",
            "/../motor",
            "/mo%74or",
        ]:
            with self.assertRaises(ValueError):
                m.safe_target(p)


class BrowserSemantics(unittest.TestCase):
    def test_original_ui_inline_secret_start_press_change_release_and_invalid_token(
        self,
    ):
        import re

        names = (
            "function manualLease(",
            "function commandFetch(",
            "function commandHeaders(",
            "function updateTokenStatus(",
            "function motorStart(",
            "function motorStop(",
            "function refreshMotorLease(",
            "function updateSpeedValue(",
            "function handleGlobalDrag(",
            'document.addEventListener("keydown"',
            'document.addEventListener("keyup"',
        )
        functions = []
        for decoded in embedded_scripts((ROOT / "src/network/web_server.cpp").read_text()):
            if any(decoded.startswith(name) for name in names):
                functions.append(decoded)
        self.assertEqual(len(functions), 11)
        fixture = r"""
const vm=require('node:vm'),fs=require('node:fs');const config=JSON.parse(fs.readFileSync(0,'utf8'));
const events=[],commands=[],intervals=[],elements={},listeners={},created=[],starts=[];let prompts=0,invalidNext=false;
global.window=global;global.location={href:'http://localhost/',origin:'http://localhost'};global.crypto=require('node:crypto').webcrypto;
function element(id){return elements[id]??={id,value:'60',textContent:'',style:{},dataset:{},classList:{add(){},remove(){}},appendChild(){},setAttribute(){},matches(){return true;}};}
global.document={documentElement:element('root'),body:{appendChild(){}},getElementById:element,createElement:tag=>{const e=element('node'+created.length);e.tagName=tag.toUpperCase();created.push(e);return e;},addEventListener:(n,f,capture=false)=>{(listeners[n]??=[]).push({f,capture});},visibilityState:'visible'};
global.addEventListener=(n,f)=>{(listeners[n]??=[]).push({f,capture:false});};
function dispatch(type,target,extra={}){let stopped=false;const e={type,target,isTrusted:true,pointerId:1,clientX:1,clientY:2,preventDefault(){},stopImmediatePropagation(){stopped=true;},...extra};for(const entry of [...(listeners[type]||[])].sort((a,b)=>Number(b.capture)-Number(a.capture))){entry.f(e);if(stopped)break;}}
global.setInterval=(f,ms)=>{intervals.push({f,ms});return intervals.length;};global.clearInterval=()=>{};
global.prompt=()=>{prompts++;throw Error('prompt forbidden');};global.showMessage=()=>{};global.updateStatus=()=>{};
global.MANUAL_LEASE_REFRESH_MS=250;
vm.runInThisContext("let commandToken='';const COMMAND_TOKEN_CONFIGURED=true;let activeMotors={};let currentMode='button';let activeJoysticks={};let joystickActive={};");
global.fetch=async(url,options={})=>{
 if(url==='/__capture/events'){events.push(...JSON.parse(options.body).events);return{ok:true,json:async()=>({stored:true})};}
 if(url.startsWith('/__capture/clock'))return{ok:true,json:async()=>({hostReceivedMonotonic:1,hostSentMonotonic:1,anchorId:'fixtureanchor'})};
 if(url.startsWith('/__capture/status'))return{ok:true,json:async()=>({recordingReady:starts.length>0,startAllowed:!starts.length,reasonText:starts.length?'수집 연결됨':'수집 시작 대기'})};
 if(url==='/__capture/start'){starts.push(JSON.parse(options.body));return{ok:true,json:async()=>({reasonText:'수집 시작 요청됨'})};}
 if(url==='/manual_leases')return {ok:true,json:async()=>({schemaVersion:'motionbrain.manual_control_leases.v1',bootId:'a'.repeat(32),ttlMs:750,motors:[1,2,3,4,5].map(motor=>({motor,generation:0,active:false}))})};
 const params=new URL(url,'http://localhost').searchParams;
 commands.push({url,method:options.method,headers:Object.fromEntries(new Headers(options.headers)),driverPWM:params.get('action')==='stop'?0:Math.round(Number(params.get('percent'))*255/100)});
 const invalid=invalidNext;invalidNext=false;return{ok:true,json:async()=>invalid?({success:false,error:'invalid X-MotionBrain-Token'}):({success:true,manualLeaseMs:750,leaseBootId:'a'.repeat(32),leaseGeneration:1,leaseId:'a'.repeat(32)+'-'+params.get('id')+'-00000001'})};
};
for(const fn of config.functions)vm.runInThisContext(fn);vm.runInThisContext(config.instrumentation);
const settle=async()=>{for(let i=0;i<36;i++)await Promise.resolve();};
(async()=>{
 await settle();if(starts.length||commands.length)throw Error('page visit started collection or motor');
 const password=created.find(e=>e.id==='intent-capture-password'),form=created.find(e=>e.tagName==='FORM'),start=created.find(e=>e.id==='intent-capture-start');
 for(const code of ['KeyQ','KeyA','KeyW','KeyS','KeyE','KeyD','KeyR','KeyF','KeyT','KeyG','Enter','Tab']){dispatch('keydown',password,{code});dispatch('keyup',code==='Tab'?element('btn-forward-3'):password,{code});}
 for(const type of ['beforeinput','input','paste','compositionstart','compositionupdate','compositionend','click'])dispatch(type,password,{data:'SECRET-FIXTURE'});
 password.value='SECRET-FIXTURE';form.onsubmit({preventDefault(){}});
 if(password.value||commands.length||starts.length||!vm.runInThisContext("commandToken==='SECRET-FIXTURE'"))throw Error('secret auth semantics');
 await start.onclick();if(starts.length!==1||commands.length)throw Error('start button semantics');
 dispatch('pointerdown',element('btn-forward-3'));window.motorStart(3,'forward',{preventDefault(){}});await settle();
 element('speed-3').value='70';dispatch('input',element('speed-3'));window.updateSpeedValue(3);window.refreshMotorLease(3);await settle();
 dispatch('pointerup',element('btn-forward-3'));window.motorStop(3,{preventDefault(){}});await settle();dispatch('pointercancel',element('btn-forward-3'));dispatch('blur',null);
 // Lost key-up during blur cannot suppress a later new physical press/release.
 dispatch('keydown',password,{code:'KeyQ'});dispatch('blur',null);
 dispatch('keydown',element('btn-forward-1'),{code:'KeyQ',repeat:false});await settle();dispatch('keyup',element('btn-forward-1'),{code:'KeyQ'});await settle();
 // A held secret key repeated after focus leaves the field must never RUN.
 const beforeRepeat=commands.length;dispatch('keydown',password,{code:'KeyE'});dispatch('keydown',element('btn-forward-3'),{code:'KeyE',repeat:true});dispatch('keyup',element('btn-forward-3'),{code:'KeyE'});await settle();if(commands.length!==beforeRepeat)throw Error('held secret key leaked to motor');
 // A motor key pressed outside must still STOP when released inside the secret field.
 dispatch('keydown',element('btn-forward-1'),{code:'KeyQ'});await settle();dispatch('keyup',password,{code:'KeyQ'});await settle();
 // Original joystick mouse release also reaches its handler through the secret field.
 vm.runInThisContext("activeJoysticks[3]={area:document.getElementById('joystick-3'),handle:document.getElementById('handle-3'),motorId:3,touchId:null};joystickActive[3]={action:'forward',percent:60,cancel:()=>{delete joystickActive[3];commandFetch('/motor?action=stop&id=3',{method:'POST'},false);}};");
 dispatch('mouseup',password);await settle();
 invalidNext=true;await window.commandFetch('/command?cmd=arm',{method:'POST'});await settle();
 if(!vm.runInThisContext("commandToken.length===0"))throw Error('invalid token not cleared');
 try{await window.commandFetch('/command?cmd=arm',{method:'POST'});}catch(e){}
 if(commands.length!==9)throw Error('missing token used prompt or sent command');
 intervals.find(x=>x.ms===100).f();await settle();console.log(JSON.stringify({events,commands,starts,prompts}));
})().catch(e=>{console.error(e);process.exit(1);});
"""
        raw = subprocess.run(
            ["node", "-e", fixture],
            input=json.dumps(
                dict(functions=functions, instrumentation=m.JS.read_text())
            ),
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        data = json.loads(raw)
        commands = data["commands"]
        events = data["events"]
        self.assertEqual(data["prompts"], 0)
        self.assertEqual(len(data["starts"]), 1)
        self.assertEqual(set(data["starts"][0]), {"pageId"})
        self.assertEqual([c["driverPWM"] for c in commands[:3]], [153, 179, 0])
        self.assertEqual([c["method"] for c in commands], ["POST"] * 9)
        self.assertTrue(
            all(
                c["headers"]["x-motionbrain-token"] == "SECRET-FIXTURE"
                for c in commands
            )
        )
        self.assertEqual(
            [e["eventType"] for e in events if e["kind"] == "dom_intent"],
            ["pointerdown", "input", "pointerup", "pointercancel"],
        )
        self.assertEqual(
            [e["direction"] for e in events if e["kind"] == "keyboard_intent"],
            ["KeyQ", "KeyQ", "KeyQ"],
        )
        self.assertEqual([c["driverPWM"] for c in commands[3:8]], [153, 0, 153, 0, 0])
        self.assertNotIn("SECRET-FIXTURE", json.dumps(events))
        self.assertNotIn("intent-capture-password", json.dumps(events))
        self.assertEqual(len([e for e in events if e["kind"] == "command_dispatch"]), 9)
        with tempfile.TemporaryDirectory() as directory:
            log = m.EventLog(directory)
            anchor = next(e for e in events if e["kind"] == "clock_anchor")
            log.clocks[anchor["anchorId"]] = (
                anchor["pageId"],
                {
                    k: anchor[k]
                    for k in ("anchorId", "hostReceivedMonotonic", "hostSentMonotonic")
                },
            )
            from unittest.mock import patch

            with patch.object(m.time, "monotonic", return_value=1.1):
                log.browser(dict(events=events))
            log.close()

    def test_original_joint_handlers_dom_identity_errors_and_command_ids(self):
        import re

        names = (
            "function manualLease(",
            "function commandFetch(",
            "function commandHeaders(",
            "function jointStart(",
            "function jointStop(",
            "function jointStopNow(",
            "function refreshJointLease(",
        )
        functions = []
        for decoded in embedded_scripts((ROOT / "src/network/web_server.cpp").read_text()):
            if any(decoded.startswith(name) for name in names):
                functions.append(decoded)
        self.assertEqual(len(functions), 7)
        result = subprocess.run(
            ["node", str(ROOT / "tests/controller_ui_joint_fixture.cjs")],
            input=json.dumps(
                dict(functions=functions, instrumentation=m.JS.read_text())
            ),
            capture_output=True,
            text=True,
            check=True,
        )
        data = json.loads(result.stdout)
        events = data["events"]
        commands = data["commands"]
        expected = [
            ("elbow", "down"),
            ("elbow", "down"),
            ("elbow", "stop"),
            ("wrist", "up"),
            ("wrist", "stop"),
            ("gripper", "stop"),
            ("elbow", "down"),
            ("elbow", "stop"),
            ("elbow", "down"),
            ("elbow", "stop"),
            ("elbow", "up"),
            ("elbow", "stop"),
            ("elbow", "stop"),
            ("elbow", "down"),
            ("elbow", "stop"),
            ("elbow", "down"),
            ("elbow", "up"),
            ("elbow", "down"),
            ("elbow", "up"),
            ("elbow", "down"),
        ]
        import urllib.parse

        parsed = [
            urllib.parse.parse_qs(urllib.parse.urlsplit(c["url"]).query)
            for c in commands
        ]
        self.assertEqual([(q["joint"][0], q["action"][0]) for q in parsed], expected)
        ids = [c["headers"].get("x-ui-capture-id") for c in commands]
        self.assertTrue(all(ids))
        self.assertEqual(len(set(ids)), 20)
        self.assertEqual(parsed[1]["percent"], ["70"])
        starts = [
            e
            for e in events
            if e["kind"] == "ui_function" and e["functionName"] == "jointStart"
        ]
        self.assertTrue(all(e["domEventSequence"] is not None for e in starts))
        self.assertTrue(all(e["joint"] for e in starts))
        stops = [
            e
            for e in events
            if e["kind"] == "ui_function" and e["functionName"] == "jointStopNow"
        ]
        self.assertTrue(any(e["domEventSequence"] is not None for e in stops))
        self.assertTrue(any(e["domEventSequence"] is None for e in stops))
        self.assertTrue(
            any(
                e["kind"] == "command_intent"
                and e["sourceFunction"] == "jointStopNow"
                and e["relatedFailedRequestSequence"] is not None
                for e in events
            )
        )
        self.assertTrue(
            any(
                e["kind"] == "ui_function"
                and e["functionName"] == "jointStop"
                and e["eventType"] == "touchcancel"
                for e in events
            )
        )
        self.assertTrue(
            any(
                e["kind"] == "command_intent"
                and e["sourceFunction"] == "refreshJointLease"
                and e["query"]["percent"] == "70"
                for e in events
            )
        )
        command_ids = {c["commandId"] for c in commands}
        self.assertEqual(
            {e["commandId"] for e in events if e["kind"] == "command_result"},
            command_ids,
        )
        self.assertFalse(data["failedDownAfterStop"]["hidden"])
        self.assertIn("[비밀 제외]", data["failedDownAfterStop"]["text"])
        self.assertIn("elbow/down", data["failedDownAfterStop"]["text"])
        self.assertTrue(data["downClearedAfterRun"]["hidden"])
        self.assertEqual(data["downClearedAfterRun"]["display"], "none")
        self.assertTrue(all(not n["hidden"] for n in data["upAndStopFailures"]))
        self.assertEqual(data["upAndStopFailures"][0]["anchorId"], "elbow-up-button")
        self.assertEqual(data["explicitStopFailure"]["anchorId"], "elbow-stop-button")
        self.assertIn("HTTP 403", data["explicitStopFailure"]["text"])
        self.assertTrue(
            all(n["hidden"] and n["display"] == "none" for n in data["clearedByNewRun"])
        )
        self.assertFalse(data["lateSuccessDidNotClear"]["hidden"])
        self.assertIn("newer failure", data["lateSuccessDidNotClear"]["text"])
        self.assertTrue(data["lateFailureDidNotReturn"]["hidden"])
        self.assertNotIn("SECRET-FIXTURE", json.dumps(events))
        self.assertNotIn(
            "SECRET-FIXTURE",
            json.dumps(
                {k: v for k, v in data.items() if k not in ("commands", "events")}
            ),
        )
        with tempfile.TemporaryDirectory() as directory:
            log = m.EventLog(directory)
            anchor = next(e for e in events if e["kind"] == "clock_anchor")
            log.clocks[anchor["anchorId"]] = (
                anchor["pageId"],
                {
                    k: anchor[k]
                    for k in ("anchorId", "hostReceivedMonotonic", "hostSentMonotonic")
                },
            )
            from unittest.mock import patch

            with patch.object(m.time, "monotonic", return_value=1.5):
                for offset in range(0, len(events), 64):
                    log.browser(dict(events=events[offset : offset + 64]))
            log.close()

    def test_lease_supersession_is_neutral_only_with_exact_no_dispatch_proof(self):
        import re

        names = (
            "function manualLease(",
            "function commandFetch(",
            "function commandHeaders(",
            "function jointStart(",
            "function jointStop(",
            "function jointStopNow(",
            "function refreshJointLease(",
            "function motorStart(",
            "function motorStop(",
            "function refreshMotorLease(",
        )
        functions = []
        for decoded in embedded_scripts((ROOT / "src/network/web_server.cpp").read_text()):
            if any(decoded.startswith(name) for name in names):
                functions.append(decoded)
        self.assertEqual(len(functions), 10)
        result = subprocess.run(
            ["node", str(ROOT / "tests/controller_ui_joint_fixture.cjs")],
            input=json.dumps(
                dict(
                    functions=functions,
                    instrumentation=m.JS.read_text(),
                    scenario="lease",
                )
            ),
            capture_output=True,
            text=True,
            check=True,
        )
        data = json.loads(result.stdout)
        snap = data["snapshots"]
        events = data["events"]
        commands = data["commands"]
        self.assertEqual(snap["coalescedIssued"], 0)
        self.assertEqual(snap["openReleaseExtraStops"], 0)
        self.assertEqual(snap["oldGestureExtraRequests"], 0)
        self.assertEqual(snap["openFailureNotice"], 0)
        self.assertIn("미전송", snap["openInfo"])
        self.assertEqual(
            snap["motorExtra"], 1
        )  # release STOP is not held behind a pending browser lease
        for failure in snap["failures"]:
            self.assertEqual(failure["stopAttempts"], 1)
            if failure["plan"].get("status") == 403:
                self.assertEqual(len(failure["requests"]), 1)
            else:
                self.assertEqual(len(failure["requests"]), 2)
                self.assertIn("action=stop", failure["requests"][-1])
        self.assertEqual(len(snap["initialCanceled"]), 2)
        self.assertIn("action=stop", snap["initialCanceled"][-1])
        neutral = [
            e
            for e in events
            if e["kind"] == "command_result"
            and e.get("uiOutcome") == "lease_superseded_without_dispatch"
        ]
        self.assertEqual(len(neutral), 3)
        self.assertTrue(
            all(
                e["success"] is False
                and e["commandId"] is None
                and e["controllerForwarded"] is False
                and e["gestureId"]
                for e in neutral
            )
        )
        self.assertEqual(len({e["gestureId"] for e in neutral}), 3)
        self.assertTrue(
            any(
                e["kind"] == "lease_coalesced"
                and e["sourceFunction"] == "refreshJointLease"
                for e in events
            )
        )
        self.assertTrue(
            any(
                e["kind"] == "lease_coalesced"
                and e["sourceFunction"] == "refreshMotorLease"
                for e in events
            )
        )
        self.assertEqual(
            commands[0]["url"], "/joint?joint=gripper&action=open&percent=60&lease=start&expectedBootId="+"a"*32+"&expectedGeneration=0"
        )
        self.assertTrue(
            next(
                e
                for e in events
                if e["kind"] == "command_result"
                and e["commandId"] == commands[0]["commandId"]
            )["success"]
        )
        self.assertNotIn("SECRET-FIXTURE", json.dumps(events))


if __name__ == "__main__":
    unittest.main()
