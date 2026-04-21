#!/usr/bin/env python3

import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def fetch_json(base_url: str, path: str, timeout: float) -> dict:
    request = urllib.request.Request(f"{base_url}{path}")
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def format_status(status: dict) -> str:
    state = status.get("state", "UNKNOWN")
    sensor = status.get("sensor", {})
    base = status.get("baseAngle", {})

    sensor_connected = "up" if sensor.get("connected") else "down"
    simulated = sensor.get("simulated", False)
    simulation_mode = sensor.get("simulationMode", "OFF")
    block_reason = sensor.get("blockReason", "NONE")
    fault_reason = sensor.get("faultReason", "NONE")
    dist_cm = sensor.get("distCm")
    vibe = sensor.get("vibe")

    if base.get("active"):
        base_summary = (
            f"active dir={base.get('direction', '?')}"
            f" current={base.get('currentDeg', 0):.1f}"
            f"/{base.get('targetDeg', 0):.1f}deg"
        )
    else:
        base_summary = f"idle last={base.get('lastStopReason', 'NONE')}"

    parts = [
        f"state={state}",
        f"sensor={sensor_connected}",
        f"sim={simulation_mode}" if simulated else "sim=OFF",
        f"block={block_reason}",
        f"fault={fault_reason}",
        f"base={base_summary}",
    ]
    if isinstance(dist_cm, (int, float)):
        parts.append(f"dist={dist_cm:.1f}cm")
    if isinstance(vibe, (int, float)):
        parts.append(f"vibe={vibe:.2f}")
    return " ".join(parts)


def print_status(status: dict) -> None:
    timestamp = time.strftime("%H:%M:%S")
    print(f"[{timestamp}] {format_status(status)}", flush=True)


def print_new_events(events_payload: dict, last_event_id: int) -> int:
    max_seen = last_event_id
    for event in events_payload.get("events", []):
        event_id = int(event.get("id", 0))
        if event_id <= last_event_id:
            continue
        timestamp = time.strftime("%H:%M:%S")
        severity = event.get("severity", "INFO")
        category = event.get("category", "event")
        code = event.get("code", "UNKNOWN")
        detail = event.get("detail", "")
        print(f"[{timestamp}] EVENT {event_id} {severity} {category}.{code} {detail}", flush=True)
        if event_id > max_seen:
            max_seen = event_id
    return max_seen


def run(args: argparse.Namespace) -> int:
    base_url = f"http://{args.host}:{args.port}"
    last_event_id = 0

    while True:
        try:
            status = fetch_json(base_url, "/status", args.timeout)
            print_status(status)

            if args.events_limit > 0:
                events = fetch_json(base_url, f"/events?limit={args.events_limit}", args.timeout)
                last_event_id = print_new_events(events, last_event_id)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            timestamp = time.strftime("%H:%M:%S")
            print(f"[{timestamp}] ERROR {exc}", file=sys.stderr, flush=True)

        if args.once:
            return 0

        time.sleep(args.interval)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Poll MotionBrain /status and /events for host-side diagnostics."
    )
    parser.add_argument("--host", default="192.168.4.1", help="MotionBrain host or IP")
    parser.add_argument("--port", type=int, default=80, help="MotionBrain HTTP port")
    parser.add_argument("--interval", type=float, default=1.0, help="Poll interval in seconds")
    parser.add_argument("--timeout", type=float, default=2.0, help="HTTP timeout in seconds")
    parser.add_argument("--events-limit", type=int, default=8, help="How many recent events to request")
    parser.add_argument("--once", action="store_true", help="Fetch one sample and exit")
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
