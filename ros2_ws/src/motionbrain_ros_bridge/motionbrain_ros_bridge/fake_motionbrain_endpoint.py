#!/usr/bin/env python3

import argparse
import json
import sys
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from typing import Any

from motionbrain_ros_bridge.m4_write_contract import M4ContractError
from motionbrain_ros_bridge.m4_write_contract import rejection_payload
from motionbrain_ros_bridge.m4_write_contract import ros_rad_from_sensor_deg
from motionbrain_ros_bridge.m4_write_contract import validate_m4_request


SCENARIOS = {
    "ready",
    "controller_fault",
    "malformed_status",
    "policy_mismatch",
    "stale_detection",
    "stale_shoulder",
    "timeout_status",
    "m4_ready",
    "m4_target_missed",
    "m4_timeout",
}


def compact_json(payload: dict[str, Any]) -> bytes:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")


def base_status_payload(*, now_ms: int | None = None) -> dict[str, Any]:
    timestamp_ms = int(time.time() * 1000) if now_ms is None else int(now_ms)
    return {
        "schemaVersion": "phase3.v1",
        "messageType": "status",
        "uptimeMs": 120000,
        "state": "IDLE",
        "motorEnabled": False,
        "motors": {
            "M1": {"name": "Gripper", "speed": 0, "enabled": False, "direction": "stopped"},
            "M2": {"name": "Wrist", "speed": 0, "enabled": False, "direction": "stopped"},
            "M3": {"name": "Elbow", "speed": 0, "enabled": False, "direction": "stopped"},
            "M4": {"name": "Shoulder", "speed": 0, "enabled": False, "direction": "stopped"},
            "M5": {"name": "Base", "speed": 0, "enabled": False, "direction": "stopped"},
        },
        "light": False,
        "sensor": {
            "source": "fake_endpoint",
            "connected": True,
            "simulated": True,
            "simulationMode": "FAULT_INJECTION",
            "lastUpdateMs": 10,
            "packetsReceived": 42,
            "parseErrors": 0,
            "imuOk": True,
            "rangeOk": True,
            "sourceTimestampMs": timestamp_ms,
            "gyroX": 0.0,
            "gyroY": 0.0,
            "gyroZ": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "distCm": 0.0,
            "vibe": 0.0,
            "obstacleSafetyEnabled": False,
            "vibrationSafetyEnabled": False,
            "imuStatus": 1,
            "imuAddress": 104,
            "imuError": 0,
            "i2cSclHigh": True,
            "i2cSdaHigh": True,
            "blocked": False,
            "blockReason": "NONE",
            "faultLatched": False,
            "faultReason": "NONE",
        },
        "baseAngle": {
            "active": False,
            "direction": "left",
            "targetDeg": 0.0,
            "currentDeg": 0.0,
            "remainingDeg": 0.0,
            "percent": 40,
            "elapsedMs": 0,
            "timeoutMs": 0,
            "processedSamples": 0,
            "lastRateDps": 0.0,
            "lastStopReason": "NONE",
            "lastTransitionMs": timestamp_ms,
        },
        "shoulderAngle": {
            "available": True,
            "sensorConnected": True,
            "sensorFresh": True,
            "sensorReady": True,
            "magnetDetected": True,
            "magnetTooWeak": False,
            "magnetTooStrong": False,
            "active": False,
            "correctionActive": False,
            "manualGuardBlocked": False,
            "correctionAttempts": 0,
            "maxCorrectionAttempts": 4,
            "ageMs": 10,
            "agc": 128,
            "magnitude": 2048,
            "rawDeg": 247.15,
            "angleDeg": 222.8,
            "mountOffsetDeg": -24.35,
            "targetDeg": 222.8,
            "errorDeg": 0.0,
            "softMinDeg": 122.08,
            "softMaxDeg": 301.02,
            "targetToleranceDeg": 0.5,
            "settledSuccessToleranceDeg": 0.4,
            "manualDownBoundaryDeg": 123.58,
            "manualUpBoundaryDeg": 300.12,
            "lastStopReason": "NONE",
        },
        "teleop": {
            "connected": True,
            "deadman": False,
            "controlActive": False,
            "lastFrameAgeMs": 10,
            "packetsReceived": 42,
            "parseErrors": 0,
            "session": 0,
            "seq": 1,
            "reach": 0.0,
            "lift": 0.0,
            "twist": 0.0,
            "gripOpen": False,
            "gripClose": False,
            "ledToggleSeq": 0,
            "embeddedSafety": True,
            "embeddedSafetyAgeMs": 10,
            "embeddedSafetyPackets": 42,
            "lastStopReason": "DEADMAN_RELEASE",
        },
        "recovery": {
            "action": "none",
            "canRecoverToIdle": False,
            "requiresFaultClear": False,
            "requiresMotionClear": False,
            "detail": "Fake endpoint is in a stable read-only state.",
        },
        "lastCommand": {
            "seen": False,
            "id": 0,
            "type": "",
            "source": "fake_endpoint",
            "executedAtMs": 0,
            "ageMs": 0,
            "success": False,
            "message": "",
        },
    }


def base_routine_payload() -> dict[str, Any]:
    return {
        "schemaVersion": "routine.v0",
        "state": "IDLE",
        "dryRunOnly": True,
        "executeImplemented": False,
        "executor": {
            "enabled": False,
            "mode": "dry_run_only",
            "abortSupported": True,
            "timeoutSupported": True,
            "materializationGateSupported": True,
            "queueApplyAllowed": False,
            "status": {
                "state": "idle",
                "routineName": "",
                "currentStep": 0,
                "totalSteps": 0,
                "motionStepCount": 0,
                "remainingMs": 0,
                "lastResult": "none",
                "lastDetail": "fake endpoint idle",
            },
        },
        "diagnostics": {
            "sensor": {"connected": True, "fresh": True, "ageMs": 10},
            "teleop": {
                "connected": True,
                "deadman": False,
                "controlActive": False,
                "ageMs": 10,
            },
            "safety": {
                "motionBlocked": False,
                "blockReason": "NONE",
                "faultLatched": False,
                "faultReason": "NONE",
            },
        },
        "feedback": {
            "schemaVersion": "feedback.v0",
            "selectedClosureTarget": "base_yaw_reference",
            "physicalRoutineExecutionAllowed": False,
            "readyForRoutineExecution": False,
            "blockReason": "feedback_required",
            "detail": "base_yaw_reference feedback not installed; physical routine execution disabled",
            "baseYaw": {
                "installed": False,
                "available": False,
                "connected": False,
                "fresh": False,
                "referenced": False,
                "faulted": True,
                "hardwareReady": False,
                "readyForRoutineExecution": False,
                "signalActive": False,
                "pin": 36,
                "activeLow": True,
                "ageMs": 0,
                "lastUpdateMs": 0,
                "positionDeg": 0.0,
                "velocityDps": 0.0,
                "lastStopReason": "NOT_INSTALLED",
                "fault": "not_installed",
            },
        },
        "recovery": {"action": "none"},
        "lastCommand": {
            "seen": False,
            "success": False,
            "type": "",
            "source": "fake_endpoint",
            "message": "",
        },
        "routines": [{"name": "inspect", "steps": 0, "motionStepCount": 0}],
    }


def status_payload_for_scenario(scenario: str) -> dict[str, Any]:
    payload = base_status_payload()
    if scenario.startswith("m4_"):
        payload["state"] = "ARMED"
    if scenario == "controller_fault":
        payload["state"] = "FAULT"
        payload["sensor"]["faultLatched"] = True
        payload["sensor"]["faultReason"] = "FAKE_CONTROLLER_FAULT"
        payload["recovery"]["action"] = "clear_fault"
        payload["recovery"]["requiresFaultClear"] = True
    elif scenario == "stale_shoulder":
        shoulder = payload["shoulderAngle"]
        shoulder["available"] = True
        shoulder["sensorConnected"] = True
        shoulder["sensorFresh"] = False
        shoulder["sensorReady"] = False
        shoulder["ageMs"] = 5000
        shoulder["lastStopReason"] = "SENSOR_STALE"
    return payload


def routine_payload_for_scenario(scenario: str) -> dict[str, Any]:
    payload = base_routine_payload()
    if scenario == "policy_mismatch":
        feedback = payload["feedback"]
        feedback["physicalRoutineExecutionAllowed"] = True
        feedback["readyForRoutineExecution"] = False
        feedback["blockReason"] = "policy_mismatch"
        feedback["detail"] = "fault injection: physical execution allowed without feedback readiness"
        payload["executor"]["queueApplyAllowed"] = True
    return payload


def detection_payload_for_scenario(scenario: str) -> dict[str, Any]:
    if scenario == "stale_detection":
        return {
            "available": False,
            "detected": False,
            "targetType": "object",
            "label": "cup",
            "confidence": 0.0,
            "alignment": "LOST",
            "commandSuggestion": "none",
            "reason": "fault injection: stale detection",
            "ageMs": 60000,
            "ts": time.time() - 60.0,
        }
    return {
        "available": True,
        "detected": True,
        "targetType": "object",
        "label": "cup",
        "classId": 41,
        "confidence": 0.88,
        "alignment": "CENTER",
        "commandSuggestion": "hold",
        "areaRatio": 0.12,
        "pixels": 9216,
        "width": 320,
        "height": 240,
        "centerX": 160.0,
        "centerY": 120.0,
        "offsetX": 0.0,
        "offsetY": 0.0,
        "alignDeadband": 0.15,
        "ts": time.time(),
    }


def events_payload_for_scenario(scenario: str, limit: int) -> dict[str, Any]:
    events = [
        {
            "id": 1,
            "tsMs": 1,
            "severity": "INFO",
            "category": "system",
            "code": "FAKE_ENDPOINT_READY",
            "detail": f"scenario={scenario}",
        }
    ]
    return {"events": events[: max(0, limit)]}


class FakeMotionBrainRequestHandler(BaseHTTPRequestHandler):
    server_version = "MotionBrainFakeEndpoint/0.1"

    @property
    def scenario(self) -> str:
        return self.server.scenario  # type: ignore[attr-defined]

    @property
    def delay_sec(self) -> float:
        return self.server.delay_sec  # type: ignore[attr-defined]

    def log_message(self, format: str, *args: Any) -> None:
        if self.server.quiet:  # type: ignore[attr-defined]
            return
        super().log_message(format, *args)

    def send_json(self, payload: dict[str, Any], status: int = 200) -> None:
        body = compact_json(payload)
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            return

    def send_text(self, body: bytes, status: int = 200, content_type: str = "text/plain") -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            return

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"
        if path == "/health":
            self.send_json({"ok": True, "scenario": self.scenario})
            return
        if path == "/status":
            if self.scenario == "timeout_status":
                time.sleep(self.delay_sec)
            if self.scenario == "malformed_status":
                self.send_text(b'{"state":"IDLE"', content_type="application/json")
                return
            self.send_json(status_payload_for_scenario(self.scenario))
            return
        if path == "/routine":
            self.send_json(routine_payload_for_scenario(self.scenario))
            return
        if path == "/events":
            query = urllib.parse.parse_qs(parsed.query)
            try:
                limit = int(query.get("limit", ["8"])[0])
            except ValueError:
                limit = 8
            self.send_json(events_payload_for_scenario(self.scenario, limit))
            return
        if path == "/api/detection":
            self.send_json(detection_payload_for_scenario(self.scenario))
            return
        self.send_json({"ok": False, "error": "not_found", "path": path}, status=404)

    def do_POST(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"
        if path.startswith("/light"):
            self.send_json(
                {
                    "success": True,
                    "action": urllib.parse.parse_qs(parsed.query).get("action", [""])[0],
                    "forwarded": False,
                    "message": "fake endpoint accepted light command only",
                }
            )
            return
        if path.startswith("/routine"):
            self.send_json(
                {
                    "success": False,
                    "action": "rejected",
                    "routineName": "",
                    "result": "blocked",
                    "error": "fake_endpoint_read_only",
                    "forwarded": False,
                    "message": "fake endpoint never forwards physical routine commands",
                }
            )
            return
        if path == "/m4/target":
            self.handle_m4_target()
            return
        self.send_json({"ok": False, "error": "not_found", "path": path}, status=404)

    def handle_m4_target(self) -> None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            request = json.loads(self.rfile.read(length).decode("utf-8")) if length else {}
            if not isinstance(request, dict):
                raise M4ContractError("json_object_required")
            command_id = str(request.get("commandId", ""))
            if command_id in self.server.m4_command_ids:  # type: ignore[attr-defined]
                raise M4ContractError("duplicate_command_id")
            validated = validate_m4_request(request, status_payload_for_scenario(self.scenario))
            self.server.m4_command_ids.add(command_id)  # type: ignore[attr-defined]
            if self.scenario == "m4_timeout":
                self.send_json(
                    {
                        **validated,
                        "accepted": True,
                        "executed": False,
                        "forwarded": False,
                        "simulated": True,
                        "reason": "TIMEOUT",
                        "stopReason": "TIMEOUT",
                    },
                    status=504,
                )
                return
            final_deg = validated["requestedSensorDeg"]
            stop_reason = "TARGET_REACHED"
            executed = True
            if self.scenario == "m4_target_missed":
                final_deg += 1.0
                stop_reason = "TARGET_MISSED"
                executed = False
            self.send_json(
                {
                    **validated,
                    "accepted": True,
                    "executed": executed,
                    "forwarded": False,
                    "simulated": True,
                    "finalSensorDeg": final_deg,
                    "finalPositionRad": ros_rad_from_sensor_deg(final_deg, config=None),
                    "errorDeg": final_deg - validated["requestedSensorDeg"],
                    "stopReason": stop_reason,
                    "correctionAttempts": 0,
                },
                status=200 if executed else 409,
            )
        except M4ContractError as exc:
            self.send_json(
                rejection_payload(exc, str(locals().get("command_id", ""))),
                status=409,
            )
        except (json.JSONDecodeError, ValueError):
            self.send_json(rejection_payload(M4ContractError("invalid_json")), status=400)


def make_server(
    host: str,
    port: int,
    *,
    scenario: str,
    delay_sec: float = 3.0,
    quiet: bool = False,
) -> ThreadingHTTPServer:
    if scenario not in SCENARIOS:
        raise ValueError(f"scenario must be one of: {', '.join(sorted(SCENARIOS))}")
    server = ThreadingHTTPServer((host, int(port)), FakeMotionBrainRequestHandler)
    server.scenario = scenario  # type: ignore[attr-defined]
    server.delay_sec = max(float(delay_sec), 0.0)  # type: ignore[attr-defined]
    server.quiet = bool(quiet)  # type: ignore[attr-defined]
    server.m4_command_ids = set()  # type: ignore[attr-defined]
    return server


def strip_ros_args(argv: list[str]) -> list[str]:
    if "--ros-args" not in argv:
        return argv
    return argv[: argv.index("--ros-args")]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Run a fake MotionBrain HTTP/perception endpoint.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="ready")
    parser.add_argument("--delay-sec", type=float, default=3.0)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(strip_ros_args(sys.argv[1:] if argv is None else argv))

    server = make_server(
        args.host,
        args.port,
        scenario=args.scenario,
        delay_sec=args.delay_sec,
        quiet=args.quiet,
    )
    host, port = server.server_address
    print(f"fake MotionBrain endpoint listening on http://{host}:{port} scenario={args.scenario}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
