#!/usr/bin/env python3

import argparse
import json
import os
import secrets
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

ROS_BRIDGE_SRC = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "motionbrain_ros_bridge"
MISSION_SRC = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "motionbrain_mission"
if str(ROS_BRIDGE_SRC) not in sys.path:
    sys.path.insert(0, str(ROS_BRIDGE_SRC))
if str(MISSION_SRC) not in sys.path:
    sys.path.insert(0, str(MISSION_SRC))

from motionbrain_ros_bridge.vision_detection import detect_colored_target  # noqa: E402
from motionbrain_mission.policy_proposal import PolicyConfig  # noqa: E402
from motionbrain_mission.policy_proposal import PolicyDetectionSnapshot  # noqa: E402
from motionbrain_mission.policy_proposal import PolicyGuardSnapshot  # noqa: E402
from motionbrain_mission.policy_proposal import PolicyStatusSnapshot  # noqa: E402
from motionbrain_mission.policy_proposal import propose_policy_action  # noqa: E402


DEFAULT_GRASP_SEQUENCE = [
    {"joint": "gripper", "action": "open", "percent": 35, "ms": 300},
    {"joint": "gripper", "action": "stop", "percent": 0, "ms": 0},
    {"joint": "gripper", "action": "close", "percent": 35, "ms": 450},
    {"joint": "gripper", "action": "stop", "percent": 0, "ms": 0},
]


def build_dashboard_policy_proposal(
    status: dict[str, Any],
    detection: dict[str, Any],
    *,
    instruction: str,
    target_label: str = "cup",
    min_confidence: float = 0.5,
) -> dict[str, Any]:
    sensor = status.get("sensor") if isinstance(status.get("sensor"), dict) else {}
    base = status.get("baseAngle") if isinstance(status.get("baseAngle"), dict) else {}
    state = str(status.get("state", "UNKNOWN"))
    detection_fresh = bool(detection.get("fresh", False))
    status_snapshot = PolicyStatusSnapshot(
        available=bool(status) and not bool(status.get("degraded", False)),
        state=state,
        moving=bool(status.get("moving", False)) or any(
            bool(motor.get("enabled", False))
            for motor in (status.get("motors", {}) or {}).values()
            if isinstance(motor, dict)
        ),
        faulted=state == "FAULT",
        base_active=bool(base.get("active", False)),
        safety_blocked=bool(sensor.get("blocked", False)),
        fault_latched=bool(sensor.get("faultLatched", False)),
    )
    detection_snapshot = PolicyDetectionSnapshot(
        available=bool(detection.get("available", False)),
        detected=bool(detection.get("detected", False)),
        fresh=detection_fresh,
        held=bool(detection.get("held", False)),
        alignment=str(detection.get("alignment", "LOST")),
        command_suggestion=str(detection.get("commandSuggestion", "none")),
        label=str(detection.get("label", "")),
        color=str(detection.get("color", "")),
        confidence=detection.get("confidence") if isinstance(detection.get("confidence"), (int, float)) else None,
        area_ratio=float(detection.get("areaRatio", detection.get("ratio", 0.0)) or 0.0),
    )
    guard = PolicyGuardSnapshot(
        ready=status_snapshot.available and detection_fresh,
        reason="ready" if status_snapshot.available and detection_fresh else "inputs_not_ready",
        status_fresh=status_snapshot.available,
        detection_fresh=detection_fresh,
    )
    proposal = propose_policy_action(
        instruction=instruction,
        status=status_snapshot,
        detection=detection_snapshot,
        guard=guard,
        config=PolicyConfig(target_label=target_label, min_confidence=min_confidence),
    ).to_dict()
    proposal["ok"] = True
    proposal["source"] = "dashboard_read_only"
    proposal["executionAvailable"] = False
    return proposal


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MotionBrain Ops Dashboard</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #080b10;
      --panel: #111821;
      --panel-2: #151e29;
      --panel-3: #0e141c;
      --line: #263241;
      --line-soft: #1b2532;
      --text: #e6edf3;
      --muted: #8fa1b4;
      --faint: #5e7084;
      --ok: #86efac;
      --warn: #fcd34d;
      --bad: #fca5a5;
      --accent: #38bdf8;
      --shadow: 0 18px 42px rgba(0, 0, 0, 0.28);
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
      align-items: flex-end;
      justify-content: space-between;
      gap: 16px;
      padding: 18px 20px 14px;
      border-bottom: 1px solid var(--line-soft);
      background: #080b10;
      position: sticky;
      top: 0;
      z-index: 2;
    }
    .brand-kicker {
      color: var(--accent);
      font-size: 11px;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 1.8px;
      margin-bottom: 5px;
    }
    h1 {
      margin: 0;
      font-size: 30px;
      line-height: 1.05;
      font-weight: 800;
      letter-spacing: 0;
    }
    .topline {
      display: flex;
      align-items: center;
      gap: 8px;
      flex-wrap: wrap;
      justify-content: flex-end;
      color: #b8c7d6;
      font-size: 11px;
      font-weight: 800;
      text-transform: uppercase;
    }
    .topline span {
      max-width: 360px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
      border: 1px solid var(--line);
      border-radius: 999px;
      padding: 7px 10px;
      background: #101721;
    }
    main {
      display: grid;
      grid-template-columns: minmax(0, 1.15fr) minmax(320px, 0.85fr);
      gap: 14px;
      padding: 16px;
      max-width: 1360px;
      margin: 0 auto;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      box-shadow: var(--shadow);
      overflow: hidden;
    }
    section h2 {
      margin: 0;
      padding: 14px 16px;
      border-bottom: 1px solid var(--line-soft);
      font-size: 12px;
      color: #c9d6e2;
      letter-spacing: 1.2px;
      text-transform: uppercase;
      background: var(--panel);
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
      padding: 12px;
    }
    .metric {
      min-height: 82px;
      border: 1px solid var(--line-soft);
      border-radius: 8px;
      padding: 12px;
      background: var(--panel-3);
    }
    .label {
      color: var(--muted);
      font-size: 11px;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 0.9px;
      margin-bottom: 8px;
    }
    .value {
      font-size: 18px;
      font-weight: 850;
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
    .row .grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }
    .panel-body { padding: 12px; }
    .camera-stage {
      position: relative;
      width: 100%;
      aspect-ratio: 4 / 3;
      background: #05080d;
      border: 1px solid var(--line-soft);
      border-radius: 8px;
      overflow: hidden;
    }
    .camera-frame {
      display: block;
      width: 100%;
      height: 100%;
      object-fit: contain;
      background: #05080d;
    }
    .vision-overlay {
      position: absolute;
      inset: 0;
      pointer-events: none;
      overflow: hidden;
    }
    .target-box {
      position: absolute;
      display: none;
      border: 2px solid rgba(134, 239, 172, 0.95);
      border-radius: 4px;
      box-shadow: 0 0 0 1px rgba(4, 120, 87, 0.7), 0 0 24px rgba(34, 197, 94, 0.34);
    }
    .target-box.visible {
      display: block;
    }
    .target-box::before,
    .target-box::after {
      content: "";
      position: absolute;
      width: 13px;
      height: 13px;
      border-color: #eafff2;
      border-style: solid;
    }
    .target-box::before {
      left: -4px;
      top: -4px;
      border-width: 2px 0 0 2px;
    }
    .target-box::after {
      right: -4px;
      bottom: -4px;
      border-width: 0 2px 2px 0;
    }
    .target-label {
      position: absolute;
      left: -2px;
      top: -28px;
      padding: 4px 7px;
      border-radius: 4px;
      background: rgba(8, 11, 16, 0.82);
      color: #bbf7d0;
      border: 1px solid rgba(134, 239, 172, 0.5);
      font-size: 10px;
      font-weight: 900;
      letter-spacing: 1px;
      text-transform: uppercase;
      white-space: nowrap;
    }
    .target-dot {
      position: absolute;
      display: none;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      border: 2px solid #ecfeff;
      background: rgba(34, 197, 94, 0.82);
      box-shadow: 0 0 16px rgba(34, 197, 94, 0.8);
      transform: translate(-50%, -50%);
    }
    .target-dot.visible {
      display: block;
    }
    .lock-state {
      position: absolute;
      left: 12px;
      top: 12px;
      padding: 6px 9px;
      border-radius: 999px;
      border: 1px solid rgba(148, 163, 184, 0.35);
      background: rgba(8, 11, 16, 0.74);
      color: #cbd5e1;
      font-size: 10px;
      font-weight: 900;
      letter-spacing: 1px;
      text-transform: uppercase;
    }
    .lock-state.lock {
      color: #bbf7d0;
      border-color: rgba(34, 197, 94, 0.58);
      background: rgba(20, 83, 45, 0.42);
    }
    .lock-state.track {
      color: #fef3c7;
      border-color: rgba(245, 158, 11, 0.55);
      background: rgba(120, 53, 15, 0.35);
    }
    .controls {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
      margin-top: 12px;
    }
    button {
      min-height: 38px;
      padding: 9px 12px;
      border: 1px solid #334155;
      border-radius: 6px;
      background: #1d2836;
      color: #dbeafe;
      font: inherit;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 0.3px;
      cursor: pointer;
    }
    button.primary {
      background: #0b3b54;
      border-color: rgba(56, 189, 248, 0.45);
      color: #d9f3ff;
    }
    button:disabled {
      opacity: 0.48;
      cursor: not-allowed;
    }
    input {
      min-height: 38px;
      min-width: 220px;
      padding: 9px 10px;
      border: 1px solid #334155;
      border-radius: 6px;
      background: #0b111a;
      color: var(--text);
      font: inherit;
    }
    input::placeholder {
      color: var(--faint);
    }
    table {
      width: 100%;
      border-collapse: collapse;
    }
    th, td {
      padding: 8px 10px;
      border-bottom: 1px solid var(--line-soft);
      text-align: left;
      vertical-align: top;
      font-size: 13px;
    }
    th {
      color: var(--muted);
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 0.8px;
      background: var(--panel-3);
    }
    .log {
      height: 260px;
      overflow: auto;
      background: #05080d;
      color: #dce6ef;
      padding: 10px;
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
      line-height: 1.45;
      border-top: 1px solid var(--line-soft);
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
    <div>
      <div class="brand-kicker">Operations Console</div>
      <h1>MotionBrain</h1>
    </div>
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
        <h2>M4 Shoulder Feedback</h2>
        <div class="grid">
          <div class="metric">
            <div class="label">Joint Angle</div>
            <div class="value" id="m4Angle">-</div>
            <div class="subvalue" id="m4Raw">-</div>
          </div>
          <div class="metric">
            <div class="label">AS5600</div>
            <div class="value" id="m4Sensor">-</div>
            <div class="subvalue" id="m4Magnet">-</div>
          </div>
          <div class="metric">
            <div class="label">Closed Loop</div>
            <div class="value" id="m4Control">-</div>
            <div class="subvalue" id="m4Target">-</div>
          </div>
          <div class="metric">
            <div class="label">Guard</div>
            <div class="value" id="m4Guard">-</div>
            <div class="subvalue" id="m4Stop">-</div>
          </div>
        </div>
      </section>

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
        <h2>Vision Feed</h2>
        <div class="panel-body">
          <div class="camera-stage" id="cameraStage">
            <img id="cameraFrame" class="camera-frame" alt="ESP32-CAM capture">
            <div class="vision-overlay" id="visionOverlay">
              <div class="target-box" id="targetBox"><span class="target-label" id="targetLabel">TARGET</span></div>
              <div class="target-dot" id="targetDot"></div>
              <div class="lock-state" id="lockState">SEARCHING</div>
            </div>
          </div>
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
            <input id="dashboardTokenInput" type="password" autocomplete="off" placeholder="Dashboard token">
            <button onclick="saveDashboardToken()">Set Token</button>
            <button onclick="clearDashboardToken()">Clear</button>
          </div>
          <div class="subvalue" id="dashboardAuthState">dashboard token not set</div>
          <div class="controls">
            <button class="primary" onclick="sendLight('toggle')">Toggle Light</button>
            <button onclick="sendLight('on')">Light On</button>
            <button onclick="sendLight('off')">Light Off</button>
          </div>
          <div class="controls">
            <button id="alignNudgeButton" class="primary" onclick="sendAlignNudge()" disabled>Nudge Once</button>
            <button id="cupPlanButton" onclick="sendCupGraspPlan()" disabled>Confirm Cup Dry Run</button>
            <button onclick="refresh()">Refresh</button>
          </div>
          <div class="subvalue" id="alignActionState">alignment action unavailable</div>
          <div class="subvalue" id="cupPlanState">cup dry run unavailable</div>
        </div>
      </section>

      <section>
        <h2>Policy Proposal · Read Only</h2>
        <div class="panel-body">
          <div class="controls">
            <input id="policyInstruction" type="text" value="center cup" placeholder="Policy instruction">
            <button onclick="refreshPolicyProposal()">Evaluate</button>
          </div>
          <div class="grid">
            <div class="metric">
              <div class="label">Action</div>
              <div class="value" id="policyAction">-</div>
              <div class="subvalue" id="policyConfidence">confidence -</div>
            </div>
            <div class="metric">
              <div class="label">Decision</div>
              <div class="value" id="policyConfirm">-</div>
              <div class="subvalue" id="policyReason">-</div>
            </div>
          </div>
          <div class="subvalue" id="policyPreconditions">preconditions -</div>
          <div class="subvalue">Proposal evaluation cannot execute controller commands.</div>
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
    let lastPolicyProposal = null;
    let dashboardConfig = null;
    let dashboardAuthToken = sessionStorage.getItem("motionbrainDashboardToken") || "";

    function hydrateDashboardTokenFromHash() {
      if (!window.location.hash) return;
      const params = new URLSearchParams(window.location.hash.slice(1));
      const token = params.get("token") || params.get("dashboardToken");
      if (!token) return;
      dashboardAuthToken = token;
      sessionStorage.setItem("motionbrainDashboardToken", token);
      history.replaceState(null, "", window.location.pathname + window.location.search);
    }

    function dashboardHeaders() {
      if (!dashboardAuthToken) {
        throw new Error("dashboard token required");
      }
      return {
        "Content-Type": "application/json",
        "X-Dashboard-Token": dashboardAuthToken,
      };
    }

    function updateDashboardAuthState() {
      const input = document.getElementById("dashboardTokenInput");
      const state = document.getElementById("dashboardAuthState");
      if (input && dashboardAuthToken) input.placeholder = "Dashboard token stored";
      if (input && !dashboardAuthToken) input.placeholder = "Dashboard token";
      if (!state) return;
      const cfg = dashboardConfig || {};
      if (cfg.dashboardAuthConfigured === false) {
        state.textContent = "server dashboard token not configured; POST endpoints disabled";
      } else if (dashboardAuthToken) {
        state.textContent = "dashboard token stored for this browser session";
      } else {
        state.textContent = "dashboard token required for POST endpoints";
      }
    }

    function saveDashboardToken() {
      const input = document.getElementById("dashboardTokenInput");
      const token = input ? input.value.trim() : "";
      if (!token) {
        pushLog("dashboard token unchanged: empty input");
        updateDashboardAuthState();
        return;
      }
      dashboardAuthToken = token;
      sessionStorage.setItem("motionbrainDashboardToken", token);
      if (input) input.value = "";
      updateDashboardAuthState();
      updateAlignActionState();
      updateCupPlanState();
      pushLog("dashboard token stored for this session");
    }

    function clearDashboardToken() {
      dashboardAuthToken = "";
      sessionStorage.removeItem("motionbrainDashboardToken");
      const input = document.getElementById("dashboardTokenInput");
      if (input) input.value = "";
      updateDashboardAuthState();
      updateAlignActionState();
      updateCupPlanState();
      pushLog("dashboard token cleared");
    }

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

    async function getJson(url, options = {}) {
      const response = await fetch(url, { cache: "no-store" });
      const data = await response.json();
      if (options.allowDegraded && data.degraded) {
        return data;
      }
      if (!response.ok || data.ok === false) {
        throw new Error(data.error || response.statusText);
      }
      return data;
    }

    async function postJson(url, payload) {
      const response = await fetch(url, {
        method: "POST",
        headers: dashboardHeaders(),
        body: JSON.stringify(payload),
      });
      const data = await response.json();
      if (!response.ok || data.ok === false || data.success === false) {
        throw new Error(data.error || data.message || response.statusText);
      }
      return data;
    }

    function updateStatus(status) {
      lastStatus = status;
      if (status.degraded) {
        const dependency = status.dependency || "controller";
        const reason = status.errorClass || status.error || "unavailable";
        setText("stateValue", "UNAVAILABLE", "bad");
        document.getElementById("uptimeValue").textContent = `dependency ${dependency}`;
        setText("safetyValue", "DEGRADED", "warn");
        document.getElementById("safetyReason").textContent = `${dependency}: ${reason}`;
        setText("sensorValue", "DOWN", "bad");
        document.getElementById("sensorDetail").textContent = status.dependencyUrl || "";
        setText("lightValue", "UNKNOWN", "warn");
        document.getElementById("motorValue").textContent = "motion-ready false";
        setText("baseActive", "UNAVAILABLE", "warn");
        document.getElementById("baseDir").textContent = "controller status unavailable";
        setText("baseProgress", "- / - deg");
        document.getElementById("baseStop").textContent = "UNKNOWN";
        setText("m4Angle", "- deg", "warn");
        document.getElementById("m4Raw").textContent = "controller status unavailable";
        setText("m4Sensor", "UNAVAILABLE", "bad");
        document.getElementById("m4Magnet").textContent = "no live controller sample";
        setText("m4Control", "UNAVAILABLE", "warn");
        document.getElementById("m4Target").textContent = "motion-ready false";
        setText("m4Guard", "BLOCKED", "bad");
        document.getElementById("m4Stop").textContent = status.message || reason;
        setText("teleopConn", "UNAVAILABLE", "warn");
        document.getElementById("teleopDeadman").textContent = "deadman unavailable active false";
        setText("teleopAxes", "R - L - T -");
        document.getElementById("teleopGrip").textContent = "grip unavailable";
        updateAlignActionState();
        updateCupPlanState();
        return;
      }
      const sensor = status.sensor || {};
      const base = status.baseAngle || {};
      const shoulder = status.shoulderAngle || {};
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

      setText("m4Angle", `${fmtNum(shoulder.angleDeg, 2)} deg`, shoulder.sensorReady ? "ok" : "warn");
      document.getElementById("m4Raw").textContent = `raw ${fmtNum(shoulder.rawDeg, 2)} deg, offset ${fmtNum(shoulder.mountOffsetDeg, 2)} deg`;
      setText("m4Sensor", shoulder.sensorReady ? "READY" : shoulder.sensorConnected ? "NOT READY" : "DOWN", shoulder.sensorReady ? "ok" : "bad");
      document.getElementById("m4Magnet").textContent = `magnet ${fmtBool(shoulder.magnetDetected)} AGC ${shoulder.agc ?? "-"} MAG ${shoulder.magnitude ?? "-"} age ${shoulder.ageMs ?? "-"}ms`;
      setText("m4Control", shoulder.active ? "ACTIVE" : "IDLE", shoulder.active ? "ok" : "");
      document.getElementById("m4Target").textContent = `target ${fmtNum(shoulder.targetDeg, 2)} deg, error ${fmtNum(shoulder.errorDeg, 2)} deg, output ${shoulder.appliedPercent ?? 0}%, correction ${shoulder.correctionAttempts ?? 0}/${shoulder.maxCorrectionAttempts ?? 0}`;
      setText("m4Guard", shoulder.manualGuardBlocked ? "BLOCKED" : "CLEAR", shoulder.manualGuardBlocked ? "bad" : "ok");
      document.getElementById("m4Stop").textContent = `${shoulder.lastStopReason || "NONE"}; limits ${fmtNum(shoulder.softMinDeg, 1)}-${fmtNum(shoulder.softMaxDeg, 1)} deg, acceptance ±${fmtNum(shoulder.targetToleranceDeg, 2)} deg, success ±${fmtNum(shoulder.settledSuccessToleranceDeg, 2)} deg`;

      setText("teleopConn", teleop.connected ? "UP" : "DOWN", teleop.connected ? "ok" : "warn");
      document.getElementById("teleopDeadman").textContent = `deadman ${fmtBool(teleop.deadman)} active ${fmtBool(teleop.controlActive)}`;
      setText("teleopAxes", `R ${fmtNum(teleop.reach, 2)} L ${fmtNum(teleop.lift, 2)} T ${fmtNum(teleop.twist, 2)}`);
      document.getElementById("teleopGrip").textContent = `grip open ${fmtBool(teleop.gripOpen)} close ${fmtBool(teleop.gripClose)}`;
      updateAlignActionState();
      updateCupPlanState();
    }

    function updateEvents(payload) {
      const events = payload.events || [];
      const body = document.getElementById("eventsBody");
      body.innerHTML = "";
      for (const event of events.slice().reverse()) {
        const tr = document.createElement("tr");
        const severity = event.severity || "INFO";
        const cls = severity === "ERROR" ? "bad" : severity === "WARN" ? "warn" : "";
        for (const [value, className] of [
          [event.id || "", ""],
          [severity, cls],
          [event.category || "", ""],
          [event.code || "", ""],
          [event.detail || "", ""],
        ]) {
          const td = document.createElement("td");
          if (className) td.className = className;
          td.textContent = value;
          tr.appendChild(td);
        }
        body.appendChild(tr);
      }
    }

    function frameViewport(stage, frameWidth, frameHeight) {
      const width = Number(frameWidth) || 320;
      const height = Number(frameHeight) || 240;
      const rect = stage.getBoundingClientRect();
      const scale = Math.min(rect.width / width, rect.height / height);
      const drawWidth = width * scale;
      const drawHeight = height * scale;
      return {
        left: (rect.width - drawWidth) / 2,
        top: (rect.height - drawHeight) / 2,
        scale,
        frameWidth: width,
        frameHeight: height,
      };
    }

    function setTargetOverlay(payload) {
      const stage = document.getElementById("cameraStage");
      const box = document.getElementById("targetBox");
      const dot = document.getElementById("targetDot");
      const label = document.getElementById("targetLabel");
      const lockState = document.getElementById("lockState");
      if (!stage || !box || !dot || !label || !lockState) return;

      const detected = Boolean(payload.detected);
      const alignment = payload.alignment || "LOST";
      if (!detected) {
        box.classList.remove("visible");
        dot.classList.remove("visible");
        lockState.textContent = payload.reason ? "NO SIGNAL" : "SEARCHING";
        lockState.className = "lock-state";
        return;
      }

      const centerX = typeof payload.centerX === "number" ? payload.centerX : payload.centroidX;
      const centerY = typeof payload.centerY === "number" ? payload.centerY : payload.centroidY;
      if (typeof centerX !== "number" || typeof centerY !== "number") {
        box.classList.remove("visible");
        dot.classList.remove("visible");
        lockState.textContent = "SEARCHING";
        lockState.className = "lock-state";
        return;
      }

      const viewport = frameViewport(stage, payload.width, payload.height);
      const targetBox = payload.targetBox || {};
      const fallbackSize = Math.max(34, Math.min(96, Math.sqrt(Math.max(payload.pixels || 0, 1)) * viewport.scale * 1.7));
      const x = typeof targetBox.x === "number" ? viewport.left + targetBox.x * viewport.scale : viewport.left + centerX * viewport.scale - fallbackSize / 2;
      const y = typeof targetBox.y === "number" ? viewport.top + targetBox.y * viewport.scale : viewport.top + centerY * viewport.scale - fallbackSize / 2;
      const width = typeof targetBox.width === "number" ? Math.max(28, targetBox.width * viewport.scale) : fallbackSize;
      const height = typeof targetBox.height === "number" ? Math.max(28, targetBox.height * viewport.scale) : fallbackSize;
      box.style.left = `${x}px`;
      box.style.top = `${y}px`;
      box.style.width = `${width}px`;
      box.style.height = `${height}px`;
      box.classList.add("visible");

      dot.style.left = `${viewport.left + centerX * viewport.scale}px`;
      dot.style.top = `${viewport.top + centerY * viewport.scale}px`;
      dot.classList.add("visible");

      const lockText = alignment === "CENTER" ? "LOCK" : `TRACK ${alignment}`;
      const targetName = payload.label || payload.color || "target";
      label.textContent = `${lockText} ${String(targetName).toUpperCase()}`;
      lockState.textContent = lockText;
      lockState.className = `lock-state ${alignment === "CENTER" ? "lock" : "track"}`;
    }

    function updateDetection(payload) {
      lastDetection = payload;
      const detected = Boolean(payload.detected);
      setTargetOverlay(payload);
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
      updateCupPlanState();
    }

    function detectionLabel(payload) {
      return String((payload && (payload.label || payload.color)) || "").trim().toLowerCase();
    }

    function alignActionEligibility() {
      const status = lastStatus || {};
      const sensor = status.sensor || {};
      const base = status.baseAngle || {};
      const detection = lastDetection || {};
      const alignment = detection.alignment || "LOST";
      if (!dashboardAuthToken) {
        return { ok: false, reason: "dashboard token missing" };
      }
      if (!dashboardConfig || !dashboardConfig.hasHttpToken) {
        return { ok: false, reason: "controller token missing" };
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
      if (detection.held) {
        return { ok: false, reason: "held target" };
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

    function cupPlanEligibility() {
      const status = lastStatus || {};
      const sensor = status.sensor || {};
      const base = status.baseAngle || {};
      const detection = lastDetection || {};
      const cfg = dashboardConfig || {};
      const requiredTarget = String(cfg.graspTargetLabel || "cup").trim().toLowerCase();
      const confidence = typeof detection.confidence === "number" ? detection.confidence : null;
      const minConfidence = typeof cfg.graspMinConfidence === "number" ? cfg.graspMinConfidence : 0.5;
      if (!dashboardAuthToken) {
        return { ok: false, reason: "dashboard token missing" };
      }
      if (!cfg.hasHttpToken) {
        return { ok: false, reason: "controller token missing" };
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
      if (detection.held) {
        return { ok: false, reason: "held target" };
      }
      if (!detection.detected) {
        return { ok: false, reason: "target not detected" };
      }
      if (detectionLabel(detection) !== requiredTarget) {
        return { ok: false, reason: `target ${detectionLabel(detection) || "-"}` };
      }
      if (confidence === null || confidence < minConfidence) {
        return { ok: false, reason: `confidence ${confidence === null ? "-" : confidence.toFixed(2)}` };
      }
      if (detection.alignment !== "CENTER") {
        return { ok: false, reason: `alignment ${detection.alignment || "LOST"}` };
      }
      return { ok: true, reason: `${requiredTarget} dry-run ready` };
    }

    function updateCupPlanState() {
      const button = document.getElementById("cupPlanButton");
      const state = document.getElementById("cupPlanState");
      if (!button || !state) return;
      const eligibility = cupPlanEligibility();
      button.disabled = !eligibility.ok;
      const cfg = dashboardConfig || {};
      const target = cfg.graspTargetLabel || "cup";
      const confidence = typeof cfg.graspMinConfidence === "number" ? cfg.graspMinConfidence.toFixed(2) : "0.50";
      state.textContent = `${eligibility.reason} / target ${target} >= ${confidence}`;
    }

    async function refresh() {
      try {
        const config = await getJson("/api/config");
        dashboardConfig = config;
        document.getElementById("motionTarget").textContent = `motion: ${config.motionBaseUrl}`;
        document.getElementById("cameraTarget").textContent = `camera: ${config.cameraUrl || "disabled"}`;
        updateDashboardAuthState();
        updateAlignActionState();
        updateCupPlanState();
      } catch (err) {
        pushLog(`config error: ${err.message}`);
      }

      try {
        updateStatus(await getJson("/api/status", { allowDegraded: true }));
      } catch (err) {
        pushLog(`status error: ${err.message}`);
      }

      try {
        updateEvents(await getJson("/api/events?limit=12"));
      } catch (err) {
        pushLog(`events error: ${err.message}`);
      }

      document.getElementById("lastRefresh").textContent = `last: ${new Date().toLocaleTimeString()}`;
    }

    async function refreshVision() {
      try {
        updateDetection(await getJson("/api/detection"));
        const img = document.getElementById("cameraFrame");
        if (img) img.src = `/api/capture?t=${Date.now()}`;
      } catch (err) {
        updateDetection({ detected: false, reason: err.message });
      }
    }

    function updatePolicyProposal(proposal) {
      lastPolicyProposal = proposal;
      const action = proposal.action || "hold";
      const physical = proposal.physicalMotionCandidate === true;
      setText("policyAction", action, physical ? "warn" : "ok");
      document.getElementById("policyConfidence").textContent = `confidence ${fmtNum(proposal.confidence, 2)}`;
      setText("policyConfirm", proposal.requiresOperatorConfirm ? "CONFIRM" : "NO EXEC", proposal.requiresOperatorConfirm ? "warn" : "ok");
      document.getElementById("policyReason").textContent = proposal.reason || "-";
      const preconditions = Array.isArray(proposal.preconditions) ? proposal.preconditions.join(", ") : "-";
      document.getElementById("policyPreconditions").textContent = `preconditions ${preconditions || "none"}`;
    }

    async function refreshPolicyProposal() {
      const input = document.getElementById("policyInstruction");
      const instruction = input ? input.value.trim() : "";
      try {
        const proposal = await getJson(`/api/policy_proposal?instruction=${encodeURIComponent(instruction)}`);
        updatePolicyProposal(proposal);
      } catch (err) {
        pushLog(`policy proposal error: ${err.message}`);
      }
    }

    async function sendLight(action) {
      try {
        const data = await postJson("/api/light", { action });
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
        const data = await postJson("/api/align_nudge", { alignment });
        if (!data.ok || !data.success || !data.stopped) {
          const start = data.start && (data.start.message || data.start.error);
          const stop = data.stop && (data.stop.message || data.stop.error);
          throw new Error(data.error || data.message || start || stop || "nudge failed");
        }
        pushLog(`align ${alignment}: ${data.message || "ok"} ${data.nudgeMs}ms @ ${data.percent}% stopped=${data.stopped}`);
        refresh();
      } catch (err) {
        pushLog(`align ${alignment} error: ${err.message}`);
      }
    }

    async function sendCupGraspPlan() {
      const eligibility = cupPlanEligibility();
      if (!eligibility.ok) {
        pushLog(`cup dry-run blocked: ${eligibility.reason}`);
        return;
      }

      try {
        const data = await postJson("/api/cup_grasp_plan", { confirmDryRun: true });
        const sequence = (data.plannedSequence || []).map((step) => `${step.joint}.${step.action}:${step.ms}ms`).join(", ");
        pushLog(`cup dry-run ready: ${data.target} ${data.alignment} conf=${fmtNum(data.confidence, 2)} sequence=${sequence}`);
        refresh();
      } catch (err) {
        pushLog(`cup dry-run error: ${err.message}`);
      }
    }

    document.addEventListener("visibilitychange", () => {
      if (!document.hidden) {
        refresh();
        refreshVision();
      }
    });
    window.addEventListener("resize", () => {
      if (lastDetection) setTargetOverlay(lastDetection);
    });
    hydrateDashboardTokenFromHash();
    updateDashboardAuthState();
    refresh();
    refreshVision();
    refreshPolicyProposal();
    setInterval(() => {
      if (!document.hidden) refresh();
    }, 2500);
    setInterval(() => {
      if (!document.hidden) refreshVision();
    }, 350);
    setInterval(() => {
      if (!document.hidden) refreshPolicyProposal();
    }, 2500);
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


def classify_dependency_error(exc: BaseException) -> str:
    text = str(exc)
    if isinstance(exc, json.JSONDecodeError):
        return "invalid_json"
    if isinstance(exc, TimeoutError) or "timed out" in text.lower():
        return "timeout"
    if isinstance(exc, urllib.error.HTTPError):
        return f"http_{exc.code}"
    if isinstance(exc, urllib.error.URLError):
        reason = str(getattr(exc, "reason", exc))
        lower_reason = reason.lower()
        if "name resolution" in lower_reason or "nodename nor servname" in lower_reason:
            return "name_resolution_failed"
        if "connection refused" in lower_reason:
            return "connection_refused"
        if "no route" in lower_reason:
            return "network_unreachable"
        return "url_error"
    if isinstance(exc, OSError):
        lower_text = text.lower()
        if "name resolution" in lower_text or "nodename nor servname" in lower_text:
            return "name_resolution_failed"
        if "connection refused" in lower_text:
            return "connection_refused"
        if "no route" in lower_text:
            return "network_unreachable"
        return "os_error"
    return "unknown_error"


def dependency_error_payload(
    dependency: str,
    dependency_url: str,
    exc: BaseException,
    *,
    last_success_at: float | None = None,
) -> dict[str, Any]:
    error_class = classify_dependency_error(exc)
    payload: dict[str, Any] = {
        "ok": False,
        "degraded": True,
        "serviceReady": True,
        "motionReady": False,
        "dependency": dependency,
        "dependencyUrl": dependency_url,
        "error": f"{dependency}_unavailable",
        "errorClass": error_class,
        "detail": str(exc),
        "message": f"{dependency} unavailable: {error_class}",
        "ts": time.time(),
    }
    if last_success_at is not None:
        payload["lastSuccessfulAt"] = last_success_at
    return payload


def normalized_target_label(value: Any) -> str:
    return str(value or "").strip().lower()


def detection_label(payload: dict[str, Any]) -> str:
    return normalized_target_label(payload.get("label") or payload.get("color"))


def build_grasp_dry_run_plan(
    detection: dict[str, Any],
    *,
    target_label: str = "cup",
    min_confidence: float = 0.5,
) -> dict[str, Any]:
    required_label = normalized_target_label(target_label) or "cup"
    label = detection_label(detection)
    confidence = detection.get("confidence")
    alignment = str(detection.get("alignment", "LOST")).upper()
    if not detection.get("detected"):
        return {"ok": False, "error": "target_not_detected", "target": required_label}
    if detection.get("held"):
        return {"ok": False, "error": "held_detection", "target": required_label}
    if label != required_label:
        return {"ok": False, "error": f"target_mismatch:{label or '-'}", "target": required_label}
    if not isinstance(confidence, (int, float)) or float(confidence) < min_confidence:
        return {"ok": False, "error": "confidence_below_threshold", "target": required_label}
    if alignment != "CENTER":
        return {"ok": False, "error": f"alignment_not_center:{alignment}", "target": required_label}

    return {
        "ok": True,
        "success": True,
        "dryRun": True,
        "target": required_label,
        "label": label,
        "confidence": float(confidence),
        "alignment": alignment,
        "reason": "operator_confirmed_dry_run",
        "plannedSequence": [dict(step) for step in DEFAULT_GRASP_SEQUENCE],
        "detection": detection,
    }


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
        "startSuccess": bool(start_result.get("success")),
        "stopSuccess": stopped,
        "direction": direction,
        "nudgeMs": nudge_ms,
        "percent": percent,
        "message": start_result.get("message", ""),
        "start": start_result,
        "stop": stop_result,
    }


class DashboardHandler(BaseHTTPRequestHandler):
    server: "DashboardServer"

    def log_message(self, fmt: str, *args: object) -> None:
        status = str(args[1]) if len(args) >= 2 else ""
        parsed = urllib.parse.urlparse(getattr(self, "path", ""))
        path = parsed.path or "-"
        if parsed.query:
            path = f"{path}?<redacted>"
        if getattr(self, "command", "") == "GET" and status == "200":
            return
        method = getattr(self, "command", "-")
        sys.stderr.write(f"[dashboard] {self.address_string()} {method} {path} {status}\n")

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        try:
            if parsed.path == "/":
                self.send_html(INDEX_HTML)
            elif parsed.path == "/api/config":
                payload = {
                    "ok": True,
                    "motionBaseUrl": self.server.motion_base_url,
                    "cameraUrl": self.server.camera_url,
                    "perceptionUrl": self.server.perception_url,
                    "detectColor": self.server.detect_color,
                    "hasHttpToken": bool(self.server.http_token),
                    "alignMode": "nudge",
                    "alignNudgeMs": self.server.align_nudge_ms,
                    "alignPercent": self.server.align_percent,
                    "graspTargetLabel": self.server.grasp_target_label,
                    "graspMinConfidence": self.server.grasp_min_confidence,
                    "dashboardAuthConfigured": bool(self.server.dashboard_token),
                    "dashboardAuthRequired": True,
                }
                self.send_json(payload, allow_cross_origin=True)
            elif parsed.path == "/api/status":
                self.handle_status()
            elif parsed.path == "/api/events":
                query = urllib.parse.parse_qs(parsed.query)
                limit = query.get("limit", [str(self.server.events_limit)])[0]
                events_url = f"{self.server.motion_base_url}/events?limit={urllib.parse.quote(limit)}"
                try:
                    self.send_json(fetch_json(events_url, self.server.timeout))
                except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
                    self.send_dependency_error("controller", events_url, exc)
            elif parsed.path == "/api/capture":
                self.handle_capture()
            elif parsed.path == "/api/vision_frame":
                self.handle_capture(allow_cross_origin=True)
            elif parsed.path == "/api/detection":
                self.handle_detection()
            elif parsed.path == "/api/policy_proposal":
                query = urllib.parse.parse_qs(parsed.query)
                instruction = query.get("instruction", [""])[0]
                self.handle_policy_proposal(instruction)
            else:
                self.send_error_json(HTTPStatus.NOT_FOUND, "not_found")
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            self.send_error_json(HTTPStatus.BAD_GATEWAY, str(exc))

    def do_POST(self) -> None:
        if not self.require_dashboard_auth():
            return

        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/light":
            self.handle_light()
        elif parsed.path == "/api/align_nudge":
            self.handle_align_nudge()
        elif parsed.path == "/api/cup_grasp_plan":
            self.handle_cup_grasp_plan()
        else:
            self.send_error_json(HTTPStatus.NOT_FOUND, "not_found")
            return

    def require_dashboard_auth(self) -> bool:
        expected = self.server.dashboard_token
        provided = self.headers.get("X-Dashboard-Token", "")
        if not expected or not provided or not secrets.compare_digest(provided, expected):
            self.send_error_json(HTTPStatus.FORBIDDEN, "dashboard_auth_required")
            return False
        return True

    def read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length).decode("utf-8")
        payload = json.loads(raw) if raw else {}
        if not isinstance(payload, dict):
            raise ValueError("json_object_required")
        return payload

    def handle_status(self) -> None:
        status_url = f"{self.server.motion_base_url}/status"
        try:
            payload = fetch_json(status_url, self.server.timeout)
            self.server.last_controller_status_success_at = time.time()
            self.send_json(payload)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
            self.send_dependency_error(
                "controller",
                status_url,
                exc,
                last_success_at=self.server.last_controller_status_success_at,
            )

    def handle_light(self) -> None:
        try:
            if not self.server.http_token:
                self.send_error_json(HTTPStatus.FORBIDDEN, "http_token_required")
                return

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
            if not self.server.camera_url and not self.server.perception_url:
                self.send_error_json(HTTPStatus.BAD_REQUEST, "camera_url_not_configured")
                return

            detection = self.server.get_detection()
            if not detection.get("detected") or detection.get("alignment") != alignment:
                detected_alignment = str(detection.get("alignment", "LOST"))
                self.send_error_json(HTTPStatus.CONFLICT, f"alignment_changed:{detected_alignment}")
                return
            if detection.get("held"):
                self.send_error_json(HTTPStatus.CONFLICT, "alignment_held_detection")
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

    def handle_cup_grasp_plan(self) -> None:
        try:
            if not self.server.http_token:
                self.send_error_json(HTTPStatus.FORBIDDEN, "http_token_required")
                return

            body = self.read_json_body()
            if body.get("confirmDryRun") is not True:
                self.send_error_json(HTTPStatus.BAD_REQUEST, "confirmDryRun_required")
                return
            if not self.server.camera_url and not self.server.perception_url:
                self.send_error_json(HTTPStatus.BAD_REQUEST, "camera_url_not_configured")
                return

            detection = self.server.get_detection()
            plan = build_grasp_dry_run_plan(
                detection,
                target_label=self.server.grasp_target_label,
                min_confidence=self.server.grasp_min_confidence,
            )
            if not plan.get("ok"):
                self.send_json(plan, HTTPStatus.CONFLICT)
                return

            status = fetch_json(f"{self.server.motion_base_url}/status", self.server.timeout)
            allowed, reason = self.server.status_allows_grasp_plan(status)
            if not allowed:
                self.send_error_json(HTTPStatus.CONFLICT, f"grasp_plan_not_allowed:{reason}")
                return

            plan["status"] = {
                "state": status.get("state", "UNKNOWN"),
                "baseActive": bool(status.get("baseAngle", {}).get("active", False)),
            }
            self.send_json(plan)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError, OSError) as exc:
            self.send_error_json(HTTPStatus.BAD_GATEWAY, str(exc))

    def handle_capture(self, allow_cross_origin: bool = False) -> None:
        if not self.server.camera_url and not self.server.perception_url:
            self.send_error_json(HTTPStatus.BAD_REQUEST, "camera_url_not_configured")
            return

        try:
            frame, content_type = self.server.get_camera_frame()
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError, OSError) as exc:
            dependency, url = self.server.camera_dependency("frame")
            self.send_dependency_error(dependency, url, exc)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        if allow_cross_origin:
            self.send_cross_origin_headers()
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def handle_detection(self) -> None:
        if not self.server.camera_url and not self.server.perception_url:
            self.send_error_json(HTTPStatus.BAD_REQUEST, "camera_url_not_configured")
            return

        try:
            payload = self.server.get_detection()
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError, OSError) as exc:
            dependency, url = self.server.camera_dependency("detection")
            self.send_dependency_error(dependency, url, exc)
            return
        self.send_json(payload, allow_cross_origin=True)

    def handle_policy_proposal(self, instruction: str) -> None:
        if len(instruction) > 240:
            self.send_error_json(HTTPStatus.BAD_REQUEST, "instruction_too_long")
            return
        try:
            status = fetch_json(f"{self.server.motion_base_url}/status", self.server.timeout)
            detection = self.server.get_detection()
            payload = build_dashboard_policy_proposal(
                status,
                detection,
                instruction=instruction,
                target_label=self.server.grasp_target_label,
                min_confidence=self.server.grasp_min_confidence,
            )
            self.send_json(payload)
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, ValueError, OSError) as exc:
            self.send_dependency_error("policy_inputs", "dashboard_status_and_detection", exc)

    def send_html(self, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_json(
        self,
        payload: dict[str, Any],
        status: HTTPStatus = HTTPStatus.OK,
        allow_cross_origin: bool = False,
    ) -> None:
        body = json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        if allow_cross_origin:
            self.send_cross_origin_headers()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def send_cross_origin_headers(self) -> None:
        origin = self.headers.get("Origin", "")
        if origin and origin in self.server.cors_origins:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")

    def send_error_json(self, status: HTTPStatus, error: str) -> None:
        self.send_json({"ok": False, "error": error}, status)

    def send_dependency_error(
        self,
        dependency: str,
        dependency_url: str,
        exc: BaseException,
        *,
        last_success_at: float | None = None,
    ) -> None:
        payload = dependency_error_payload(
            dependency,
            dependency_url,
            exc,
            last_success_at=last_success_at,
        )
        self.send_json(payload, HTTPStatus.BAD_GATEWAY, allow_cross_origin=True)


class DashboardServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BaseHTTPRequestHandler],
        motion_base_url: str,
        camera_url: str,
        perception_url: str,
        detect_color: str,
        timeout: float,
        events_limit: int,
        http_token: str,
        dashboard_token: str,
        cors_origins: set[str],
        align_nudge_ms: int,
        align_percent: int,
        grasp_target_label: str,
        grasp_min_confidence: float,
    ) -> None:
        super().__init__(server_address, handler_class)
        self.motion_base_url = motion_base_url
        self.camera_url = camera_url.rstrip("/")
        self.perception_url = perception_url.rstrip("/")
        self.detect_color = detect_color
        self.timeout = timeout
        self.events_limit = events_limit
        self.http_token = http_token
        self.dashboard_token = dashboard_token
        self.cors_origins = set(cors_origins)
        self.align_nudge_ms = align_nudge_ms
        self.align_percent = align_percent
        self.grasp_target_label = normalized_target_label(grasp_target_label) or "cup"
        self.grasp_min_confidence = grasp_min_confidence
        self.camera_cache_lock = threading.Lock()
        self.camera_cache: tuple[float, bytes, str] | None = None
        self.camera_cache_seconds = 0.25
        self.last_controller_status_success_at: float | None = None

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

    def status_allows_grasp_plan(self, status: dict[str, Any]) -> tuple[bool, str]:
        return self.status_allows_align_nudge(status)

    def get_camera_frame(self) -> tuple[bytes, str]:
        if self.perception_url:
            return fetch_bytes(f"{self.perception_url}/api/vision_frame", self.timeout)
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

    def get_detection(self) -> dict[str, Any]:
        if self.perception_url:
            payload = fetch_json(f"{self.perception_url}/api/detection", self.timeout)
            if not isinstance(payload, dict):
                raise ValueError("perception_detection_not_object")
            return payload

        frame, _ = self.get_camera_frame()
        payload = detect_colored_target(frame, self.detect_color)
        payload["cameraUrl"] = self.camera_url
        payload["ts"] = time.time()
        return payload

    def camera_dependency(self, purpose: str) -> tuple[str, str]:
        if self.perception_url:
            path = "api/detection" if purpose == "detection" else "api/vision_frame"
            return "perception", f"{self.perception_url}/{path}"
        return "camera", f"{self.camera_url}/capture"


def parse_cors_origins(value: str) -> set[str]:
    return {origin.strip().rstrip("/") for origin in value.split(",") if origin.strip()}


def dashboard_cors_origins(motion_base_url: str, configured_origins: set[str]) -> set[str]:
    origins = {origin.rstrip("/") for origin in configured_origins if origin}
    motion_origin = motion_base_url.rstrip("/")
    if motion_origin:
        origins.add(motion_origin)
    if motion_origin.endswith(":80"):
        origins.add(motion_origin[:-3])
    origins.add("http://motionbrain.local")
    return origins


def is_loopback_host(host: str) -> bool:
    normalized = host.strip().lower()
    return normalized == "localhost" or normalized == "::1" or normalized.startswith("127.")


def is_placeholder_token(token: str) -> bool:
    stripped = token.strip()
    return not stripped or stripped.startswith("CHANGE_ME")


def run(args: argparse.Namespace) -> int:
    motion_base_url = f"http://{args.motion_host}:{args.motion_port}"
    server = DashboardServer(
        (args.host, args.port),
        DashboardHandler,
        motion_base_url,
        args.camera_url,
        args.perception_url,
        args.detect_color,
        args.timeout,
        args.events_limit,
        args.http_token,
        args.dashboard_token,
        dashboard_cors_origins(motion_base_url, args.cors_origins),
        args.align_nudge_ms,
        args.align_percent,
        args.grasp_target_label,
        args.grasp_min_confidence,
    )
    print(f"MotionBrain ops dashboard: http://{args.host}:{args.port}")
    print(f"motion={motion_base_url}")
    print(f"camera={args.camera_url or 'disabled'}")
    print(f"perception={args.perception_url or 'local'}")
    auth_state = "enabled" if args.dashboard_token else "disabled (POST endpoints reject requests)"
    print(f"dashboard_auth={auth_state}")
    if args.cors_origins:
        print(f"cors_origins={','.join(sorted(args.cors_origins))}")
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
    parser.add_argument("--perception-url", default="", help="Optional MotionBrain perception service base URL")
    parser.add_argument("--detect-color", choices=("red", "green", "blue"), default="red")
    parser.add_argument("--timeout", type=float, default=2.0, help="HTTP timeout in seconds")
    parser.add_argument("--events-limit", type=int, default=12, help="Default event query limit")
    parser.add_argument("--http-token", default=os.environ.get("MOTIONBRAIN_HTTP_TOKEN", ""), help="Optional X-MotionBrain-Token for controller POST endpoints")
    parser.add_argument(
        "--dashboard-token",
        default=os.environ.get("MOTIONBRAIN_DASHBOARD_TOKEN", ""),
        help="Local dashboard POST token; required when binding outside loopback",
    )
    parser.add_argument(
        "--cors-origins",
        default=os.environ.get("MOTIONBRAIN_DASHBOARD_CORS_ORIGINS", ""),
        help="Comma-separated Origin allowlist for read-only dashboard proxy responses",
    )
    parser.add_argument("--align-nudge-ms", type=int, default=250, help="Dashboard vision nudge duration in milliseconds")
    parser.add_argument("--align-percent", type=int, default=25, help="Dashboard vision nudge base speed percent")
    parser.add_argument(
        "--grasp-target-label",
        default=os.environ.get("MOTIONBRAIN_GRASP_TARGET_LABEL", "cup"),
        help="Required selected target label for the dry-run grasp plan",
    )
    parser.add_argument(
        "--grasp-min-confidence",
        type=float,
        default=float(os.environ.get("MOTIONBRAIN_GRASP_MIN_CONFIDENCE", "0.5")),
        help="Minimum selected-target confidence for the dry-run grasp plan",
    )
    args = parser.parse_args()
    args.dashboard_token = args.dashboard_token.strip()
    args.cors_origins = parse_cors_origins(args.cors_origins)
    if not is_loopback_host(args.host) and is_placeholder_token(args.dashboard_token):
        parser.error("--dashboard-token must be set to a non-placeholder value when --host is not loopback")
    if args.align_nudge_ms < 50 or args.align_nudge_ms > 2000:
        parser.error("--align-nudge-ms must be between 50 and 2000")
    if args.align_percent < 1 or args.align_percent > 100:
        parser.error("--align-percent must be between 1 and 100")
    if args.grasp_min_confidence < 0.0 or args.grasp_min_confidence > 1.0:
        parser.error("--grasp-min-confidence must be between 0 and 1")
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
