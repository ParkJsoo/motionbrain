"""Default-disabled HTTP port for the experimental bounded-action API.

Only the explicit pilot CLI enables this port. Runtime credentials stay in headers and are not stored
in receipts. No redirects, proxies, retries, legacy /shoulder fallback or arm API.
"""
from __future__ import annotations

import urllib.error
import urllib.parse
import urllib.request
from contextlib import nullcontext

from learning.action_execution import canonical_request, finite
from learning.artifacts import _decode_json
from learning.device_adapter import DeviceReadAdapter, DeviceReadError, ReadOnlyHTTP, _NoRedirect


class BoundedActionHTTP:
    def __init__(self, controller_url, *, token_provider=None, writes_enabled=False, timeout=1.0, clock=None,
                 camera_url=None, perception_url=None, required_profile=None, total_deadline=False):
        if type(writes_enabled) is not bool:
            raise ValueError("writes_enabled_must_be_boolean")
        options = {} if clock is None else {"clock": clock}
        self.transport = ReadOnlyHTTP(controller_url=controller_url, camera_url=camera_url or controller_url,
                                      perception_url=perception_url or controller_url, timeout=timeout, **options)
        self.reader = DeviceReadAdapter(self.transport)
        self.clock, self.host_clock_id = self.transport.clock, self.transport.host_clock_id
        self.writes_enabled, self.token_provider = writes_enabled, token_provider
        self.required_profile = required_profile
        if type(total_deadline) is not bool:
            raise ValueError("total_deadline_must_be_boolean")
        self.total_deadline = total_deadline
        self._prepared = None
        self._prepared_cancel = None
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect())

    def preflight(self, request):
        with self._deadline(3):
            return self._preflight(request)

    def _deadline(self, requests=1):
        # The live executor runs on the main thread. Bound the whole transaction,
        # including a peer that keeps a socket alive by dripping response bytes.
        from learning.controller_latency import deadline
        return deadline(self.transport.timeout*requests) if self.total_deadline else nullcontext()

    def _preflight(self, request):
        self._prepared = None
        r = canonical_request(request)
        if not self.writes_enabled:
            raise DeviceReadError("bounded_action_writes_disabled")
        # Resolve credentials before all freshness gates; no user callback may
        # delay submission after the executor's final dispatch guard.
        token = self._token()
        prepared = self._request("/actions", r, token)
        prepared_cancel = self._request("/actions/cancel", {k:r[k] for k in ("bootId", "commandId")}, token)
        capability = self.reader.capability()
        if self.required_profile is not None and capability.get("profile") != self.required_profile:
            raise DeviceReadError("controller_profile_mismatch")
        if (capability["bootId"] != r["bootId"] or capability["enabled"] is not True
                or capability["active"] is not False
                or type(capability.get("recordCount")) is not int
                or type(capability.get("recordCapacity")) is not int
                or not 0 <= capability["recordCount"] < capability["recordCapacity"]):
            raise DeviceReadError("controller_not_ready")
        status = self.reader.status()
        sensor, teleop, motors = status.get("sensor", {}), status.get("teleop", {}), status.get("motors", {})
        if (status.get("state") != "ARMED" or sensor.get("connected") is not True
                or sensor.get("imuOk") is not True or sensor.get("faultLatched") is not False
                or sensor.get("blocked") is not False or sensor.get("simulated") is not False
                or teleop.get("controlActive") is not False or teleop.get("deadman") is not False
                or set(motors) != {f"M{i}" for i in range(1,6)}
                or any(not isinstance(m,dict) or m.get("enabled") is not False
                       or type(m.get("speed")) is not int or m["speed"] != 0 for m in motors.values())):
            raise DeviceReadError("controller_safety_or_idle_unverified")
        if r["kind"] == "shoulder":
            m4 = status.get("shoulderAngle", {})
            if (any(m4.get(k) is not True for k in ("available", "sensorConnected", "sensorFresh", "sensorReady"))
                    or m4.get("active") is not False or not finite(m4.get("ageMs")) or not 0 <= m4["ageMs"] <= 20
                    or not finite(m4.get("angleDeg")) or not finite(m4.get("softMinDeg")) or not finite(m4.get("softMaxDeg"))
                    or not m4["softMinDeg"] <= r["targetDegrees"] <= m4["softMaxDeg"]):
                raise DeviceReadError("m4_state_unverified")
        output = self.reader.outputs()
        if output["dropped"] != 0:
            raise DeviceReadError("output_history_dropped")
        self._prepared = (r, prepared)
        self._prepared_cancel = (r, prepared_cancel)
        return output

    def _token(self):
        token = self.token_provider() if callable(self.token_provider) else None
        if not isinstance(token, str) or not token or any(ord(c) < 33 or ord(c) > 126 for c in token):
            raise DeviceReadError("runtime_token_required")
        return token

    def _request(self, path, payload, token):
        return urllib.request.Request(self.transport.bases["controller"]+path,
            data=urllib.parse.urlencode(payload).encode("ascii"), method="POST",
            headers={"X-MotionBrain": "1", "X-MotionBrain-Token": token,
                     "Content-Type": "application/x-www-form-urlencoded"})

    def _post(self, request):
        with self._deadline():
            return self._post_with_socket_timeout(request)

    def _post_with_socket_timeout(self, request):
        if not self.writes_enabled:
            raise DeviceReadError("bounded_action_writes_disabled")
        start = self.clock.monotonic()
        try:
            try:
                response = self.opener.open(request, timeout=self.transport.timeout)
            except urllib.error.HTTPError as exc:
                response = exc
            with response:
                body = response.read(65537)
                code = response.code
            end = self.clock.monotonic()
            if (not finite(start) or not finite(end) or not 0 <= end-start <= self.transport.timeout
                    or len(body) > 65536 or code != 200):
                raise DeviceReadError("post_response_unverified")
            value = _decode_json(body)
            if not isinstance(value, dict):
                raise DeviceReadError("post_response_not_object")
            return value
        except Exception:
            # Do not propagate URLs, response bodies or credential-bearing
            # exception text into the command journal.
            raise DeviceReadError("bounded_post_unverified") from None

    def submit(self, request):
        r = canonical_request(request)
        prepared, self._prepared = self._prepared, None
        if not self.writes_enabled or prepared is None or prepared[0] != r:
            raise DeviceReadError("matching_preflight_required")
        return self._post(prepared[1])

    def lookup(self, request):
        with self._deadline():
            result = self.reader.lookup_action(request)
        if result.get("lookupStatus") != "matched_record":
            raise DeviceReadError("action_lookup_unverified")
        return result["record"]

    def outputs(self):
        with self._deadline():
            return self.reader.outputs()

    def cancel(self, request):
        r = canonical_request(request)
        if not self.writes_enabled:
            raise DeviceReadError("bounded_action_writes_disabled")
        prepared = self._prepared_cancel
        if prepared is not None and prepared[0] == r:
            return self._post(prepared[1])
        # Explicit cancellation after a host restart has no prepared credentials.
        return self._post(self._request("/actions/cancel", {k:r[k] for k in ("bootId", "commandId")}, self._token()))
