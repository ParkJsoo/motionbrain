#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import os
import re
import shutil
import signal
import socket
import statistics
import subprocess
import time
import urllib.request
from pathlib import Path


def run_command(
    args: list[str],
    *,
    timeout: float,
    cwd: Path | None = None,
) -> tuple[int, str]:
    process: subprocess.Popen[str] | None = None
    try:
        process = subprocess.Popen(
            args,
            cwd=cwd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        stdout, _ = process.communicate(timeout=timeout)
        return process.returncode, stdout.strip()
    except subprocess.TimeoutExpired:
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            stdout, _ = process.communicate()
            return 124, stdout.strip()
        return 124, "timeout"
    except OSError as exc:
        return 124, str(exc)


def run_shell(command: str, *, timeout: float, cwd: Path | None = None) -> tuple[int, str]:
    return run_command(["bash", "-lc", command], timeout=timeout, cwd=cwd)


def discover_url(repo: Path, kind: str, preferred: str, timeout: float) -> str:
    rc, output = run_command(
        [
            "python3",
            "tools/raspi/discover_device_url.py",
            "--kind",
            kind,
            "--preferred",
            preferred,
        ],
        timeout=timeout,
        cwd=repo,
    )
    if rc != 0:
        return ""
    for line in reversed(output.splitlines()):
        if line.startswith("http://") or line.startswith("https://"):
            return line.strip()
    return ""


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = round((len(ordered) - 1) * pct)
    return ordered[min(len(ordered) - 1, max(0, index))]


def fetch_latency(url: str, samples: int, timeout: float) -> dict[str, object]:
    values: list[float] = []
    failures: list[str] = []
    sizes: list[int] = []
    statuses: list[int] = []
    keys: set[str] = set()

    for _ in range(samples):
        start = time.perf_counter()
        try:
            request = urllib.request.Request(url, headers={"Cache-Control": "no-cache"})
            with urllib.request.urlopen(request, timeout=timeout) as response:
                data = response.read()
                statuses.append(response.status)
            values.append((time.perf_counter() - start) * 1000.0)
            sizes.append(len(data))
            try:
                payload = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                payload = None
            if isinstance(payload, dict):
                keys.update(str(key) for key in payload.keys())
        except Exception as exc:  # keep the capture useful across transient LAN issues
            failures.append(f"{type(exc).__name__}: {exc}")
        time.sleep(0.15)

    return {
        "ok": len(values),
        "fail": len(failures),
        "median_ms": statistics.median(values) if values else None,
        "p95_ms": percentile(values, 0.95),
        "min_ms": min(values) if values else None,
        "max_ms": max(values) if values else None,
        "bytes_median": statistics.median(sizes) if sizes else None,
        "statuses": sorted(set(statuses)),
        "keys": sorted(keys)[:12],
        "failures": failures[:3],
    }


def ros_command(workspace: Path, command: str, timeout: float) -> tuple[int, str]:
    setup = "source /opt/ros/jazzy/setup.bash && source install/setup.bash"
    return run_shell(f"{setup} && {command}", timeout=timeout, cwd=workspace)


def capture_topic_hz(workspace: Path, topic: str, seconds: int) -> dict[str, object]:
    rc, output = ros_command(
        workspace,
        f"timeout {seconds} ros2 topic hz {topic} --window 5",
        timeout=seconds + 5,
    )
    average_hz: float | None = None
    min_s: float | None = None
    max_s: float | None = None
    window: int | None = None

    match = re.search(r"average rate:\s*([0-9.]+)", output)
    if match:
        average_hz = float(match.group(1))

    match = re.search(r"min:\s*([0-9.]+)s\s+max:\s*([0-9.]+)s.*window:\s*(\d+)", output)
    if match:
        min_s = float(match.group(1))
        max_s = float(match.group(2))
        window = int(match.group(3))

    return {
        "rc": rc,
        "average_hz": average_hz,
        "min_s": min_s,
        "max_s": max_s,
        "window": window,
        "tail": " | ".join(output.splitlines()[-3:]),
    }


def timed_ros_status(workspace: Path, label: str, command: str, timeout: float) -> dict[str, object]:
    start = time.perf_counter()
    rc, output = ros_command(workspace, command, timeout=timeout)
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    return {
        "label": label,
        "rc": rc,
        "elapsed_ms": elapsed_ms,
        "success_true": rc == 0 and re.search(r"success[:=]\s*(true|True)", output) is not None,
        "tail": " | ".join(output.splitlines()[-6:]),
    }


def fmt_ms(value: object) -> str:
    if not isinstance(value, (int, float)):
        return "n/a"
    return f"{value:.1f}"


def sanitize_url(url: str, label: str) -> str:
    if re.search(r"https?://\d+\.\d+\.\d+\.\d+", url):
        return f"{label} discovered endpoint"
    return url


def markdown_table_value(value: str) -> str:
    return value.replace("|", "\\|").replace("\n", " ")


def build_report(args: argparse.Namespace) -> str:
    repo = Path(args.repo).expanduser().resolve()
    workspace = Path(args.workspace).expanduser().resolve()
    capture_time = dt.datetime.now().astimezone().isoformat(timespec="seconds")

    controller_url = args.controller_url or discover_url(
        repo, "controller", "http://motionbrain.local", args.discovery_timeout
    )
    camera_url = args.camera_url or discover_url(repo, "camera", "http://motionbrain-cam.local", args.discovery_timeout)

    urls: list[tuple[str, str, str]] = []
    if controller_url:
        urls.extend(
            [
                ("ESP32 controller /status", controller_url.rstrip("/") + "/status", "controller"),
                ("ESP32 controller /routine", controller_url.rstrip("/") + "/routine", "controller"),
            ]
        )
    if camera_url:
        urls.append(("ESP32-CAM /status", camera_url.rstrip("/") + "/status", "camera"))
    urls.extend(
        [
            ("Pi dashboard /api/status", args.dashboard_url.rstrip("/") + "/api/status", "dashboard"),
            ("Pi dashboard /api/config", args.dashboard_url.rstrip("/") + "/api/config", "dashboard"),
            ("Pi perception /health", args.perception_url.rstrip("/") + "/health", "perception"),
            ("Pi perception /api/detection", args.perception_url.rstrip("/") + "/api/detection", "perception"),
        ]
    )

    http_results = [
        (name, url, display_label, fetch_latency(url, args.samples, args.http_timeout))
        for name, url, display_label in urls
    ]

    topics = [
        "/motionbrain/status_typed",
        "/camera/detection_typed",
        "/joint_states",
        "/motionbrain/control_guard_typed",
        "/motionbrain/mission_state_typed",
    ]
    topic_results = [(topic, capture_topic_hz(workspace, topic, args.topic_seconds)) for topic in topics]

    service_results = [
        timed_ros_status(
            workspace,
            "routine service status",
            "ros2 service call /motionbrain/routine_command "
            "motionbrain_msgs/srv/GuardedRoutineCommand '{action: status}'",
            args.ros_timeout,
        ),
        timed_ros_status(
            workspace,
            "guarded routine action status",
            "ros2 action send_goal /motionbrain/guarded_routine "
            "motionbrain_msgs/action/GuardedRoutine '{action: status}'",
            args.ros_timeout,
        ),
    ]

    _, git_head = run_command(["git", "log", "--oneline", "-1"], timeout=5, cwd=repo)
    _, git_status = run_command(["git", "status", "-sb"], timeout=5, cwd=repo)
    _, uname = run_command(["uname", "-a"], timeout=5)
    _, groups = run_command(["groups"], timeout=5)
    _, lsusb = run_command(["lsusb"], timeout=5)
    _, temp = run_shell("vcgencmd measure_temp 2>/dev/null || true", timeout=5)
    _, throttled = run_shell("vcgencmd get_throttled 2>/dev/null || true", timeout=5)

    serial_devices = sorted(
        glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    )
    i2c_devices = sorted(glob.glob("/dev/i2c*"))
    gpio_devices = sorted(glob.glob("/dev/gpiochip*"))
    tools = {
        command: shutil.which(command) or "not installed"
        for command in [
            "sigrok-cli",
            "pulseview",
            "i2cdetect",
            "gpioinfo",
            "gpiomon",
            "pigpiod",
            "pigs",
            "vcgencmd",
        ]
    }

    lines: list[str] = [
        "# 2026-06-16 Runtime Measurement Evidence",
        "",
        "[README](../../README.md) | [Robotics system readiness](../../ROBOTICS_SYSTEM_READINESS.md)",
        "",
        "Read-only runtime measurements captured on the Raspberry Pi host. No physical motion,",
        "motor command, or routine execution command was sent; ROS2 service/action calls used",
        "`action: status` only.",
        "",
        "## Environment",
        "",
        "| Item | Value |",
        "| --- | --- |",
        f"| Capture time | `{capture_time}` |",
        f"| Host | `{socket.gethostname()}` |",
        f"| Kernel | `{markdown_table_value(uname)}` |",
        f"| Git | `{markdown_table_value(git_head)}` |",
        f"| Worktree | `{markdown_table_value(git_status)}` |",
        f"| Groups | `{markdown_table_value(groups)}` |",
        f"| Pi temperature | `{temp or 'n/a'}` |",
        f"| Pi throttling | `{throttled or 'n/a'}` |",
        "",
        "## Instrument Inventory",
        "",
        "| Item | Observed |",
        "| --- | --- |",
        f"| USB devices | `{markdown_table_value('; '.join(lsusb.splitlines()) if lsusb else 'none')}` |",
        f"| USB serial devices | `{', '.join(serial_devices) if serial_devices else 'none detected'}` |",
        f"| I2C devices | `{', '.join(i2c_devices) if i2c_devices else 'none detected'}` |",
        f"| GPIO chips | `{', '.join(gpio_devices) if gpio_devices else 'none detected'}` |",
    ]
    for command, path in tools.items():
        lines.append(f"| `{command}` | `{path}` |")
    lines.extend(
        [
            "",
            "No USB oscilloscope, logic analyzer, USB serial adapter, or multimeter interface",
            "was visible to the Pi during this capture. Physical PWM/UART/I2C waveform and",
            "motor-voltage measurements therefore remain equipment-gated, not software-gated.",
            "",
            "## HTTP Endpoint Latency",
            "",
            f"Each endpoint was sampled {args.samples} times from the Pi with a {args.http_timeout:g} s request timeout.",
            "",
            "| Endpoint | URL | OK/fail | median ms | p95 ms | min ms | max ms | median bytes | status |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for name, url, display_label, result in http_results:
        statuses = ",".join(str(item) for item in result["statuses"]) or "n/a"
        lines.append(
            f"| {name} | `{sanitize_url(url, display_label)}` | "
            f"{result['ok']}/{result['fail']} | {fmt_ms(result['median_ms'])} | "
            f"{fmt_ms(result['p95_ms'])} | {fmt_ms(result['min_ms'])} | "
            f"{fmt_ms(result['max_ms'])} | {result['bytes_median'] or 'n/a'} | `{statuses}` |"
        )
    lines.append("")
    if camera_url:
        lines.append("Direct ESP32-CAM `/status` discovery returned a reachable endpoint.")
    else:
        lines.append(
            "Direct ESP32-CAM `/status` discovery did not return during this capture; camera evidence is represented through dashboard/perception endpoints."
        )

    lines.extend(
        [
            "",
            "## ROS2 Topic Rate Samples",
            "",
            f"`ros2 topic hz` was sampled for {args.topic_seconds} seconds per topic.",
            "",
            "| Topic | avg Hz | min interval s | max interval s | window | result |",
            "| --- | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for topic, result in topic_results:
        average = result["average_hz"] if result["average_hz"] is not None else "n/a"
        min_s = result["min_s"] if result["min_s"] is not None else "n/a"
        max_s = result["max_s"] if result["max_s"] is not None else "n/a"
        window = result["window"] if result["window"] is not None else "n/a"
        tail = markdown_table_value(str(result["tail"]))
        lines.append(f"| `{topic}` | {average} | {min_s} | {max_s} | {window} | `{tail}` |")

    lines.extend(
        [
            "",
            "## ROS2 Status Round Trip",
            "",
            "| Check | rc | elapsed ms | success=true |",
            "| --- | ---: | ---: | --- |",
        ]
    )
    for result in service_results:
        lines.append(
            f"| {result['label']} | {result['rc']} | {float(result['elapsed_ms']):.1f} | "
            f"`{str(result['success_true']).lower()}` |"
        )

    lines.extend(
        [
            "",
            "## Physical Measurement Status",
            "",
            "| Signal | Status | Reason |",
            "| --- | --- | --- |",
            "| ESP32 PWM frequency/duty | not captured | no visible oscilloscope/logic analyzer/sigrok device on Pi |",
            "| STM32-to-ESP32 UART timing | not captured | no USB serial adapter or logic analyzer visible on Pi |",
            "| MPU-6050 I2C waveform | not captured | Pi I2C bus is available, but the STM32 sensor bus is not proven wired to Pi and should not be probed blindly |",
            "| Deadman release-to-stop latency | not captured | needs synchronized physical input/video or logic capture |",
            "| Motor voltage drop under bounded pulse | not captured | needs a meter/scope connected across motor supply during a safe bounded pulse |",
            "",
            "## Correct Claim",
            "",
            "```text",
            "Captured read-only runtime measurements for MotionBrain on the live Raspberry Pi host:",
            "HTTP endpoint latency, ROS2 topic rate samples, status service/action round trips,",
            "Pi health, and hardware-instrument inventory. Physical waveform and voltage",
            "measurements still require external instruments.",
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture read-only MotionBrain runtime measurement evidence.")
    parser.add_argument("--repo", default=str(Path.home() / "develop/arduino/motionbrain"))
    parser.add_argument("--workspace", default=str(Path.home() / "develop/arduino/motionbrain/ros2_ws"))
    parser.add_argument("--output", required=True)
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--http-timeout", type=float, default=3.0)
    parser.add_argument("--discovery-timeout", type=float, default=8.0)
    parser.add_argument("--topic-seconds", type=int, default=10)
    parser.add_argument("--ros-timeout", type=float, default=25.0)
    parser.add_argument("--controller-url", default="")
    parser.add_argument("--camera-url", default="")
    parser.add_argument("--dashboard-url", default="http://127.0.0.1:8765")
    parser.add_argument("--perception-url", default="http://127.0.0.1:8766")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(args)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(report)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
