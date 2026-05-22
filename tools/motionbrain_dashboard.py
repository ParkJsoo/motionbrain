#!/usr/bin/env python3

import argparse
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


TARGET_RATIO_THRESHOLD = 0.02
ALIGN_DEADBAND = 0.15


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MotionBrain Ops Dashboard</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f6f7f9;
      --panel: #ffffff;
      --text: #172026;
      --muted: #66737f;
      --border: #d7dde3;
      --ok: #16835f;
      --warn: #b36b00;
      --bad: #b42318;
      --accent: #2457c5;
      --shadow: 0 1px 3px rgba(20, 30, 40, 0.08);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      font-size: 14px;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 16px 20px;
      border-bottom: 1px solid var(--border);
      background: var(--panel);
      position: sticky;
      top: 0;
      z-index: 2;
    }
    h1 {
      margin: 0;
      font-size: 20px;
      font-weight: 700;
      letter-spacing: 0;
    }
    .topline {
      display: flex;
      align-items: center;
      gap: 10px;
      flex-wrap: wrap;
      justify-content: flex-end;
      color: var(--muted);
      font-size: 13px;
    }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1.15fr) minmax(320px, 0.85fr);
      gap: 16px;
      padding: 16px;
      max-width: 1440px;
      margin: 0 auto;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 8px;
      box-shadow: var(--shadow);
      overflow: hidden;
    }
    section h2 {
      margin: 0;
      padding: 12px 14px;
      border-bottom: 1px solid var(--border);
      font-size: 14px;
      letter-spacing: 0;
      background: #fbfcfd;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
      padding: 12px;
    }
    .metric {
      min-height: 82px;
      border: 1px solid var(--border);
      border-radius: 6px;
      padding: 10px;
      background: #fff;
    }
    .label {
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 8px;
    }
    .value {
      font-size: 18px;
      font-weight: 700;
      overflow-wrap: anywhere;
    }
    .subvalue {
      color: var(--muted);
      font-size: 12px;
      margin-top: 6px;
      overflow-wrap: anywhere;
    }
    .ok { color: var(--ok); }
    .warn { color: var(--warn); }
    .bad { color: var(--bad); }
    .row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
      margin-top: 16px;
    }
    .panel-body { padding: 12px; }
    .camera-frame {
      width: 100%;
      aspect-ratio: 4 / 3;
      object-fit: contain;
      background: #20262d;
      border: 1px solid var(--border);
      border-radius: 6px;
    }
    .controls {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 12px;
    }
    button {
      height: 34px;
      padding: 0 12px;
      border: 1px solid var(--border);
      border-radius: 6px;
      background: #fff;
      color: var(--text);
      font: inherit;
      cursor: pointer;
    }
    button.primary {
      background: var(--accent);
      border-color: var(--accent);
      color: #fff;
    }
    button:disabled {
      color: #9aa5ae;
      cursor: not-allowed;
    }
    table {
      width: 100%;
      border-collapse: collapse;
    }
    th, td {
      padding: 8px 10px;
      border-bottom: 1px solid var(--border);
      text-align: left;
      vertical-align: top;
      font-size: 13px;
    }
    th {
      color: var(--muted);
      font-weight: 600;
      background: #fbfcfd;
    }
    .log {
      height: 260px;
      overflow: auto;
      background: #111820;
      color: #dce6ef;
      padding: 10px;
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
      line-height: 1.45;
    }
    .stack {
      display: flex;
      flex-direction: column;
      gap: 16px;
    }
    @media (max-width: 980px) {
      main, .row {
        grid-template-columns: 1fr;
      }
      .grid {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }
    @media (max-width: 560px) {
      header {
        align-items: flex-start;
        flex-direction: column;
      }
      .topline {
        justify-content: flex-start;
      }
      .grid {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <header>
    <h1>MotionBrain Ops Dashboard</h1>
    <div class="topline">
      <span id="motionTarget">motion: -</span>
      <span id="cameraTarget">camera: -</span>
      <span id="lastRefresh">last: -</span>
    </div>
  </header>
  <main>
    <div class="stack">
      <section>
        <h2>Status</h2>
        <div class="grid">
          <div class="metric">
            <div class="label">State</div>
            <div class="value" id="stateValue">-</div>
            <div class="subvalue" id="uptimeValue">uptime -</div>
          </div>
          <div class="metric">
            <div class="label">Safety</div>
            <div class="value" id="safetyValue">-</div>
            <div class="subvalue" id="safetyReason">-</div>
          </div>
          <div class="metric">
            <div class="label">Sensor</div>
            <div class="value" id="sensorValue">-</div>
            <div class="subvalue" id="sensorDetail">-</div>
          </div>
          <div class="metric">
            <div class="label">Light</div>
            <div class="value" id="lightValue">-</div>
            <div class="subvalue" id="motorValue">motor -</div>
          </div>
        </div>
      </section>

      <div class="row">
        <section>
          <h2>Base Angle</h2>
          <div class="grid">
            <div class="metric">
              <div class="label">Active</div>
              <div class="value" id="baseActive">-</div>
              <div class="subvalue" id="baseDir">-</div>
            </div>
            <div class="metric">
              <div class="label">Progress</div>
              <div class="value" id="baseProgress">-</div>
              <div class="subvalue" id="baseStop">-</div>
            </div>
          </div>
        </section>
        <section>
          <h2>Teleop</h2>
          <div class="grid">
            <div class="metric">
              <div class="label">Connection</div>
              <div class="value" id="teleopConn">-</div>
              <div class="subvalue" id="teleopDeadman">-</div>
            </div>
            <div class="metric">
              <div class="label">Axes</div>
              <div class="value" id="teleopAxes">-</div>
              <div class="subvalue" id="teleopGrip">-</div>
            </div>
          </div>
        </section>
      </div>

      <section>
        <h2>Events</h2>
        <div class="panel-body">
          <table>
            <thead>
              <tr><th>ID</th><th>Severity</th><th>Category</th><th>Code</th><th>Detail</th></tr>
            </thead>
            <tbody id="eventsBody"></tbody>
          </table>
        </div>
      </section>
    </div>

    <div class="stack">
      <section>
        <h2>Camera Detection</h2>
        <div class="panel-body">
          <img id="cameraFrame" class="camera-frame" alt="ESP32-CAM capture">
          <div class="grid">
            <div class="metric">
              <div class="label">Detected</div>
              <div class="value" id="detectedValue">-</div>
              <div class="subvalue" id="detectionRatio">-</div>
            </div>
            <div class="metric">
              <div class="label">Frame</div>
              <div class="value" id="frameValue">-</div>
              <div class="subvalue" id="detectionReason">-</div>
            </div>
            <div class="metric">
              <div class="label">Target</div>
              <div class="value" id="targetValue">-</div>
              <div class="subvalue" id="targetOffset">-</div>
            </div>
            <div class="metric">
              <div class="label">Alignment</div>
              <div class="value" id="alignmentValue">-</div>
              <div class="subvalue" id="alignmentSuggestion">-</div>
            </div>
          </div>
          <div class="controls">
            <button class="primary" onclick="sendLight('toggle')">Toggle Light</button>
            <button onclick="sendLight('on')">Light On</button>
            <button onclick="sendLight('off')">Light Off</button>
          </div>
          <div class="controls">
            <button id="alignNudgeButton" class="primary" onclick="sendAlignNudge()" disabled>Nudge Once</button>
            <button onclick="refresh()">Refresh</button>
          </div>
          <div class="subvalue" id="alignActionState">alignment action unavailable</div>
        </div>
      </section>

      <section>
        <h2>Action Log</h2>
        <div class="log" id="actionLog"></div>
      </section>
    </div>
  </main>

  <script>
    const logLines = [];
    let lastStatus = null;
    let lastDetection = null;
    let dashboardConfig = null;

    function setText(id, value, className) {
      const el = document.getElementById(id);
      el.textContent = value;
      el.className = className ? `value ${className}` : el.className.replace(/\\b(ok|warn|bad)\\b/g, "");
    }

    function fmtBool(value) {
      return value ? "YES" : "NO";
    }

    function fmtNum(value, digits = 1) {
      return typeof value === "number" ? value.toFixed(digits) : "-";
    }

    function pushLog(line) {
      const stamp = new Date().toLocaleTimeString();
      logLines.unshift(`[${stamp}] ${line}`);
      if (logLines.length > 80) logLines.pop();
      document.getElementById("actionLog").textContent = logLines.join("\\n");
    }

    async function getJson(url) {
      const response = await fetch(url, { cache: "no-store" });
      const data = await response.json();
      if (!response.ok || data.ok === false) {
        throw new Error(data.error || response.statusText);
      }
      return data;
    }

    function updateStatus(status) {
      lastStatus = status;
      const sensor = status.sensor || {};
      const base = status.baseAngle || {};
      const teleop = status.teleop || {};
      const state = status.state || "UNKNOWN";
      const blocked = Boolean(sensor.blocked);
      const fault = Boolean(sensor.faultLatched);

      setText("stateValue", state, state === "FAULT" ? "bad" : state === "ARMED" ? "ok" : "");
      document.getElementById("uptimeValue").textContent = `uptime ${fmtNum((status.uptimeMs || 0) / 1000, 0)}s`;
      setText("safetyValue", fault ? "FAULT" : blocked ? "BLOCKED" : "CLEAR", fault ? "bad" : blocked ? "warn" : "ok");
      document.getElementById("safetyReason").textContent = `${sensor.blockReason || "NONE"} / ${sensor.faultReason || "NONE"}`;
      setText("sensorValue", sensor.connected ? "UP" : "DOWN", sensor.connected ? "ok" : "warn");
      document.getElementById("sensorDetail").textContent = `dist ${fmtNum(sensor.distCm)}cm, vibe ${fmtNum(sensor.vibe, 2)}`;
      setText("lightValue", status.light ? "ON" : "OFF", status.light ? "ok" : "");
      document.getElementById("motorValue").textContent = `motor ${status.motorEnabled ? "enabled" : "disabled"}`;

      setText("baseActive", base.active ? "ACTIVE" : "IDLE", base.active ? "ok" : "");
      document.getElementById("baseDir").textContent = base.active ? `${base.direction || "-"} @ ${base.percent || "-"}%` : "no active base command";
      setText("baseProgress", `${fmtNum(base.currentDeg)} / ${fmtNum(base.targetDeg)} deg`);
      document.getElementById("baseStop").textContent = base.lastStopReason || "NONE";

      setText("teleopConn", teleop.connected ? "UP" : "DOWN", teleop.connected ? "ok" : "warn");
      document.getElementById("teleopDeadman").textContent = `deadman ${fmtBool(teleop.deadman)} active ${fmtBool(teleop.controlActive)}`;
      setText("teleopAxes", `R ${fmtNum(teleop.reach, 2)} L ${fmtNum(teleop.lift, 2)} T ${fmtNum(teleop.twist, 2)}`);
      document.getElementById("teleopGrip").textContent = `grip open ${fmtBool(teleop.gripOpen)} close ${fmtBool(teleop.gripClose)}`;
      updateAlignActionState();
    }

    function updateEvents(payload) {
      const events = payload.events || [];
      const body = document.getElementById("eventsBody");
      body.innerHTML = "";
      for (const event of events.slice().reverse()) {
        const tr = document.createElement("tr");
        const severity = event.severity || "INFO";
        const cls = severity === "ERROR" ? "bad" : severity === "WARN" ? "warn" : "";
        tr.innerHTML = `<td>${event.id || ""}</td><td class="${cls}">${severity}</td><td>${event.category || ""}</td><td>${event.code || ""}</td><td>${event.detail || ""}</td>`;
        body.appendChild(tr);
      }
    }

    function updateDetection(payload) {
      lastDetection = payload;
      const detected = Boolean(payload.detected);
      setText("detectedValue", detected ? "YES" : "NO", detected ? "ok" : "");
      const areaRatio = typeof payload.areaRatio === "number" ? payload.areaRatio : payload.ratio;
      document.getElementById("detectionRatio").textContent = typeof areaRatio === "number" ? `area ${(areaRatio * 100).toFixed(2)}%` : "area -";
      setText("frameValue", payload.frameBytes ? `${payload.frameBytes} B` : "-");
      document.getElementById("detectionReason").textContent = payload.reason || `${payload.width || "-"}x${payload.height || "-"}`;
      const centerX = typeof payload.centerX === "number" ? payload.centerX : payload.centroidX;
      const centerY = typeof payload.centerY === "number" ? payload.centerY : payload.centroidY;
      const centroid = typeof centerX === "number" && typeof centerY === "number"
        ? `${centerX.toFixed(0)}, ${centerY.toFixed(0)}`
        : "-";
      setText("targetValue", centroid);
      document.getElementById("targetOffset").textContent = typeof payload.offsetX === "number"
        ? `x ${payload.offsetX >= 0 ? "+" : ""}${payload.offsetX.toFixed(2)}`
        : "x -";
      const alignment = payload.alignment || "-";
      setText("alignmentValue", alignment, alignment === "CENTER" ? "ok" : detected ? "warn" : "");
      const suggestion = payload.commandSuggestion || "none";
      const deadband = typeof payload.alignDeadband === "number" ? `db ${payload.alignDeadband.toFixed(2)}` : "db -";
      document.getElementById("alignmentSuggestion").textContent = `${suggestion} / ${deadband}`;
      updateAlignActionState();
    }

    function alignActionEligibility() {
      const status = lastStatus || {};
      const sensor = status.sensor || {};
      const base = status.baseAngle || {};
      const detection = lastDetection || {};
      const alignment = detection.alignment || "LOST";
      if (!dashboardConfig || !dashboardConfig.hasHttpToken) {
        return { ok: false, reason: "token missing" };
      }
      if (status.state !== "ARMED") {
        return { ok: false, reason: `state ${status.state || "-"}` };
      }
      if (sensor.blocked || sensor.faultLatched) {
        return { ok: false, reason: sensor.blockReason || sensor.faultReason || "safety block" };
      }
      if (base.active) {
        return { ok: false, reason: "base busy" };
      }
      if (!detection.detected || !["LEFT", "RIGHT"].includes(alignment)) {
        return { ok: false, reason: `alignment ${alignment}` };
      }
      return { ok: true, reason: `${alignment.toLowerCase()} nudge ready` };
    }

    function updateAlignActionState() {
      const button = document.getElementById("alignNudgeButton");
      const state = document.getElementById("alignActionState");
      if (!button || !state) return;
      const eligibility = alignActionEligibility();
      button.disabled = !eligibility.ok;
      const cfg = dashboardConfig || {};
      const nudge = cfg.alignNudgeMs ? `${cfg.alignNudgeMs}ms` : "-";
      const percent = cfg.alignPercent ? `${cfg.alignPercent}%` : "-";
      state.textContent = `${eligibility.reason} / nudge ${nudge} @ ${percent}`;
    }

    async function refresh() {
      try {
        const config = await getJson("/api/config");
        dashboardConfig = config;
        document.getElementById("motionTarget").textContent = `motion: ${config.motionBaseUrl}`;
        document.getElementById("cameraTarget").textContent = `camera: ${config.cameraUrl || "disabled"}`;
        updateAlignActionState();
      } catch (err) {
        pushLog(`config error: ${err.message}`);
      }

      try {
        updateStatus(await getJson("/api/status"));
      } catch (err) {
        pushLog(`status error: ${err.message}`);
      }

      try {
        updateEvents(await getJson("/api/events?limit=12"));
      } catch (err) {
        pushLog(`events error: ${err.message}`);
      }

      try {
        updateDetection(await getJson("/api/detection"));
        const img = document.getElementById("cameraFrame");
        img.src = `/api/capture?t=${Date.now()}`;
      } catch (err) {
        updateDetection({ detected: false, reason: err.message });
      }

      document.getElementById("lastRefresh").textContent = `last: ${new Date().toLocaleTimeString()}`;
    }

    async function sendLight(action) {
      try {
        const response = await fetch("/api/light", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ action }),
        });
        const data = await response.json();
        if (!response.ok || data.ok === false || data.success === false) {
          throw new Error(data.error || data.message || response.statusText);
        }
        pushLog(`light ${action}: ${data.message || "ok"}`);
        refresh();
      } catch (err) {
        pushLog(`light ${action} error: ${err.message}`);
      }
    }

    async function sendAlignNudge() {
      const eligibility = alignActionEligibility();
      if (!eligibility.ok) {
        pushLog(`align nudge blocked: ${eligibility.reason}`);
        return;
      }

      const alignment = lastDetection.alignment;
      try {
        const response = await fetch("/api/align_nudge", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ alignment }),
        });
        const data = await response.json();
        if (!response.ok || data.ok === false || data.success === false || data.stopped === false) {
          throw new Error(data.error || data.message || "nudge failed");
        }
        pushLog(`align ${alignment}: ${data.message || "ok"} stopped=${data.stopped}`);
        refresh();
      } catch (err) {
        pushLog(`align ${alignment} error: ${err.message}`);
      }
    }

    refresh();
    setInterval(refresh, 1500);
  </script>
</body>
</html>
"""


def fetch_json(url: str, timeout: float) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def fetch_bytes(url: str, timeout: float) -> tuple[bytes, str]:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        content_type = response.headers.get("Content-Type", "application/octet-stream")
        return response.read(), content_type


def post_motionbrain(base_url: str, path: str, timeout: float, token: str = "") -> dict[str, Any]:
    headers = {"X-MotionBrain": "1"}
    if token:
        headers["X-MotionBrain-Token"] = token
    request = urllib.request.Request(
        f"{base_url}{path}",
        method="POST",
        headers=headers,
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def classify_alignment(offset_x: float | None, deadband: float = ALIGN_DEADBAND) -> str:
    if offset_x is None:
        return "LOST"
    if offset_x < -deadband:
        return "LEFT"
    if offset_x > deadband:
        return "RIGHT"
    return "CENTER"


def command_suggestion_for_alignment(alignment: str) -> str:
    if alignment == "LEFT":
        return "base_left"
    if alignment == "RIGHT":
        return "base_right"
    if alignment == "CENTER":
        return "hold"
    return "none"


def execute_base_nudge(
    motion_base_url: str,
    direction: str,
    percent: int,
    nudge_ms: int,
    timeout: float,
    token: str,
) -> dict[str, Any]:
    start_path = f"/joint?joint=base&action={urllib.parse.quote(direction)}&percent={percent}"
    stop_path = "/joint?joint=base&action=stop"
    start_result: dict[str, Any] = {}
    stopped = False
    stop_result: dict[str, Any] = {}
    try:
        start_result = post_motionbrain(motion_base_url, start_path, timeout, token)
        if start_result.get("success"):
            time.sleep(nudge_ms / 1000.0)
    finally:
        try:
            stop_result = post_motionbrain(motion_base_url, stop_path, timeout, token)
            stopped = bool(stop_result.get("success"))
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError):
            pass

    return {
        "ok": bool(start_result.get("success")) and stopped,
        "success": bool(start_result.get("success")),
        "stopped": stopped,
        "direction": direction,
        "nudgeMs": nudge_ms,
        "percent": percent,
        "message": start_result.get("message", ""),
        "start": start_result,
        "stop": stop_result,
    }


def detect_colored_target(frame: bytes, color: str) -> dict[str, Any]:
    try:
        import cv2  # type: ignore
        import numpy as np  # type: ignore
    except ImportError:
        return {
            "detected": False,
            "available": False,
            "color": color,
            "reason": "opencv_unavailable",
            "frameBytes": len(frame),
            "alignment": "LOST",
            "commandSuggestion": "none",
        }

    data = np.frombuffer(frame, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        return {
            "detected": False,
            "available": True,
            "color": color,
            "reason": "decode_failed",
            "frameBytes": len(frame),
            "alignment": "LOST",
            "commandSuggestion": "none",
        }

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if color == "red":
        mask1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 80, 80), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
    elif color == "green":
        mask = cv2.inRange(hsv, (40, 70, 70), (85, 255, 255))
    elif color == "blue":
        mask = cv2.inRange(hsv, (95, 70, 70), (130, 255, 255))
    else:
        return {
            "detected": False,
            "available": True,
            "color": color,
            "reason": "unsupported_color",
            "frameBytes": len(frame),
            "alignment": "LOST",
            "commandSuggestion": "none",
        }

    pixels = int(cv2.countNonZero(mask))
    height, width = image.shape[:2]
    area = max(height * width, 1)
    ratio = pixels / area
    detected = ratio >= TARGET_RATIO_THRESHOLD
    centroid_x: float | None = None
    centroid_y: float | None = None
    offset_x: float | None = None
    offset_y: float | None = None

    if detected and pixels > 0:
        moments = cv2.moments(mask)
        if moments["m00"] != 0:
            centroid_x = float(moments["m10"] / moments["m00"])
            centroid_y = float(moments["m01"] / moments["m00"])
            center_x = (width - 1) / 2.0
            center_y = (height - 1) / 2.0
            offset_x = (centroid_x - center_x) / max(center_x, 1.0)
            offset_y = (centroid_y - center_y) / max(center_y, 1.0)

    alignment = classify_alignment(offset_x) if detected else "LOST"
    command_suggestion = command_suggestion_for_alignment(alignment)
    return {
        "detected": detected,
        "available": True,
        "color": color,
        "ratio": ratio,
        "areaRatio": ratio,
        "pixels": pixels,
        "width": width,
        "height": height,
        "frameBytes": len(frame),
        "centerX": centroid_x,
        "centerY": centroid_y,
        "centroidX": centroid_x,
        "centroidY": centroid_y,
        "offsetX": offset_x,
        "offsetY": offset_y,
        "alignDeadband": ALIGN_DEADBAND,
        "alignment": alignment,
        "commandSuggestion": command_suggestion,
    }


class DashboardHandler(BaseHTTPRequestHandler):
    server: "DashboardServer"

    def log_message(self, fmt: str, *args: object) -> None:
        sys.stderr.write(f"[dashboard] {self.address_string()} {fmt % args}\n")

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        try:
            if parsed.path == "/":
                self.send_html(INDEX_HTML)
            elif parsed.path == "/api/config":
                self.send_json(
                    {
                        "ok": True,
                        "motionBaseUrl": self.server.motion_base_url,
                        "cameraUrl": self.server.camera_url,
                        "detectColor": self.server.detect_color,
                        "hasHttpToken": bool(self.server.http_token),
                        "alignMode": "nudge",
                        "alignNudgeMs": self.server.align_nudge_ms,
                        "alignPercent": self.server.align_percent,
                    }
                )
            elif parsed.path == "/api/status":
                self.send_json(fetch_json(f"{self.server.motion_base_url}/status", self.server.timeout))
            elif parsed.path == "/api/events":
                query = urllib.parse.parse_qs(parsed.query)
                limit = query.get("limit", [str(self.server.events_limit)])[0]
                self.send_json(fetch_json(f"{self.server.motion_base_url}/events?limit={urllib.parse.quote(limit)}", self.server.timeout))
            elif parsed.path == "/api/capture":
                self.handle_capture()
            elif parsed.path == "/api/detection":
                self.handle_detection()
            else:
                self.send_error_json(HTTPStatus.NOT_FOUND, "not_found")
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            self.send_error_json(HTTPStatus.BAD_GATEWAY, str(exc))

    def do_POST(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/light":
            self.handle_light()
        elif parsed.path == "/api/align_nudge":
            self.handle_align_nudge()
        else:
            self.send_error_json(HTTPStatus.NOT_FOUND, "not_found")
            return

    def read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw) if raw else {}

    def handle_light(self) -> None:
        try:
            body = self.read_json_body()
            action = str(body.get("action", "")).strip().lower()
            if action not in {"on", "off", "toggle"}:
                self.send_error_json(HTTPStatus.BAD_REQUEST, "invalid_action")
                return

            path = f"/light?action={urllib.parse.quote(action)}"
            result = post_motionbrain(self.server.motion_base_url, path, self.server.timeout, self.server.http_token)
            result["requestedAction"] = action
            self.send_json(result)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError, OSError) as exc:
            self.send_error_json(HTTPStatus.BAD_GATEWAY, str(exc))

    def handle_align_nudge(self) -> None:
        try:
            if not self.server.http_token:
                self.send_error_json(HTTPStatus.FORBIDDEN, "http_token_required")
                return

            body = self.read_json_body()
            alignment = str(body.get("alignment", "")).strip().upper()
            if alignment not in {"LEFT", "RIGHT"}:
                self.send_error_json(HTTPStatus.BAD_REQUEST, "alignment_must_be_LEFT_or_RIGHT")
                return
            if not self.server.camera_url:
                self.send_error_json(HTTPStatus.BAD_REQUEST, "camera_url_not_configured")
                return

            frame, _ = self.server.get_camera_frame()
            detection = detect_colored_target(frame, self.server.detect_color)
            if not detection.get("detected") or detection.get("alignment") != alignment:
                detected_alignment = str(detection.get("alignment", "LOST"))
                self.send_error_json(HTTPStatus.CONFLICT, f"alignment_changed:{detected_alignment}")
                return

            status = fetch_json(f"{self.server.motion_base_url}/status", self.server.timeout)
            allowed, reason = self.server.status_allows_align_nudge(status)
            if not allowed:
                self.send_error_json(HTTPStatus.CONFLICT, f"alignment_not_allowed:{reason}")
                return

            direction = alignment.lower()
            result = execute_base_nudge(
                self.server.motion_base_url,
                direction,
                self.server.align_percent,
                self.server.align_nudge_ms,
                self.server.timeout,
                self.server.http_token,
            )
            result["alignment"] = alignment
            result["detection"] = detection
            self.send_json(result)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError, OSError) as exc:
            self.send_error_json(HTTPStatus.BAD_GATEWAY, str(exc))

    def handle_capture(self) -> None:
        if not self.server.camera_url:
            self.send_error_json(HTTPStatus.BAD_REQUEST, "camera_url_not_configured")
            return

        frame, content_type = self.server.get_camera_frame()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def handle_detection(self) -> None:
        if not self.server.camera_url:
            self.send_error_json(HTTPStatus.BAD_REQUEST, "camera_url_not_configured")
            return

        frame, _ = self.server.get_camera_frame()
        payload = detect_colored_target(frame, self.server.detect_color)
        payload["cameraUrl"] = self.server.camera_url
        payload["ts"] = time.time()
        self.send_json(payload)

    def send_html(self, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_error_json(self, status: HTTPStatus, error: str) -> None:
        self.send_json({"ok": False, "error": error}, status)


class DashboardServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        motion_base_url: str,
        camera_url: str,
        detect_color: str,
        timeout: float,
        events_limit: int,
        http_token: str,
        align_nudge_ms: int,
        align_percent: int,
    ) -> None:
        super().__init__(server_address, handler_class)
        self.motion_base_url = motion_base_url
        self.camera_url = camera_url.rstrip("/")
        self.detect_color = detect_color
        self.timeout = timeout
        self.events_limit = events_limit
        self.http_token = http_token
        self.align_nudge_ms = align_nudge_ms
        self.align_percent = align_percent
        self.camera_cache_lock = threading.Lock()
        self.camera_cache: tuple[float, bytes, str] | None = None
        self.camera_cache_seconds = 1.0

    def status_allows_align_nudge(self, status: dict[str, Any]) -> tuple[bool, str]:
        sensor = status.get("sensor", {})
        base = status.get("baseAngle", {})
        if status.get("state") != "ARMED":
            return False, f"state_{status.get('state', 'UNKNOWN')}"
        if sensor.get("faultLatched", False):
            return False, str(sensor.get("faultReason", "fault"))
        if sensor.get("blocked", False):
            return False, str(sensor.get("blockReason", "blocked"))
        if base.get("active", False):
            return False, "base_busy"
        return True, "ok"

    def get_camera_frame(self) -> tuple[bytes, str]:
        if not self.camera_url:
            raise ValueError("camera_url_not_configured")

        now = time.monotonic()
        with self.camera_cache_lock:
            if self.camera_cache is not None:
                fetched_at, frame, content_type = self.camera_cache
                if now - fetched_at <= self.camera_cache_seconds:
                    return frame, content_type

            frame, content_type = fetch_bytes(f"{self.camera_url}/capture", self.timeout)
            self.camera_cache = (time.monotonic(), frame, content_type)
            return frame, content_type


def run(args: argparse.Namespace) -> int:
    motion_base_url = f"http://{args.motion_host}:{args.motion_port}"
    server = DashboardServer(
        (args.host, args.port),
        DashboardHandler,
        motion_base_url,
        args.camera_url,
        args.detect_color,
        args.timeout,
        args.events_limit,
        args.http_token,
        args.align_nudge_ms,
        args.align_percent,
    )
    print(f"MotionBrain ops dashboard: http://{args.host}:{args.port}")
    print(f"motion={motion_base_url}")
    print(f"camera={args.camera_url or 'disabled'}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard")
    finally:
        server.server_close()
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Local web dashboard for MotionBrain status, events, camera detection, and light control."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Dashboard bind host")
    parser.add_argument("--port", type=int, default=8765, help="Dashboard bind port")
    parser.add_argument("--motion-host", default="192.168.4.1", help="MotionBrain controller IP")
    parser.add_argument("--motion-port", type=int, default=80, help="MotionBrain HTTP port")
    parser.add_argument("--camera-url", default="", help="ESP32-CAM base URL, for example http://192.168.4.2")
    parser.add_argument("--detect-color", choices=("red", "green", "blue"), default="red")
    parser.add_argument("--timeout", type=float, default=2.0, help="HTTP timeout in seconds")
    parser.add_argument("--events-limit", type=int, default=12, help="Default event query limit")
    parser.add_argument("--http-token", default=os.environ.get("MOTIONBRAIN_HTTP_TOKEN", ""), help="Optional X-MotionBrain-Token for controller POST endpoints")
    parser.add_argument("--align-nudge-ms", type=int, default=250, help="Dashboard vision nudge duration in milliseconds")
    parser.add_argument("--align-percent", type=int, default=25, help="Dashboard vision nudge base speed percent")
    args = parser.parse_args()
    if args.align_nudge_ms < 50 or args.align_nudge_ms > 2000:
        parser.error("--align-nudge-ms must be between 50 and 2000")
    if args.align_percent < 1 or args.align_percent > 100:
        parser.error("--align-percent must be between 1 and 100")
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
