#!/usr/bin/env bash
set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SERVICE_NAME="${MOTIONBRAIN_SERVICE_NAME:-motionbrain-ros-bridge.service}"
CHECK_SERVICE="${CHECK_SERVICE:-1}"
SAMPLE_TIMEOUT_SECONDS="${SAMPLE_TIMEOUT_SECONDS:-20}"
CURL_TIMEOUT_SECONDS="${CURL_TIMEOUT_SECONDS:-5}"
CAPTURE_FULL_ROS2_EVIDENCE="${CAPTURE_FULL_ROS2_EVIDENCE:-1}"

MOTION_URL="${MOTIONBRAIN_CONTROLLER_URL:-${MOTIONBRAIN_MOTION_URL:-http://motionbrain.local}}"
DASHBOARD_URL="${MOTIONBRAIN_DASHBOARD_URL:-http://127.0.0.1:8765}"

EXPECTED_FEEDBACK_READY="${EXPECTED_FEEDBACK_READY:-false}"
EXPECTED_PHYSICAL_ROUTINE_ALLOWED="${EXPECTED_PHYSICAL_ROUTINE_ALLOWED:-false}"
EXPECTED_BASE_YAW_FEEDBACK_FAULT="${EXPECTED_BASE_YAW_FEEDBACK_FAULT:-not_installed}"
EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY="${EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY:-false}"
EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE="${EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE:-false}"
EXPECTED_BASE_YAW_FEEDBACK_REFERENCED="${EXPECTED_BASE_YAW_FEEDBACK_REFERENCED:-false}"
EXPECTED_BASE_YAW_FEEDBACK_PIN="${EXPECTED_BASE_YAW_FEEDBACK_PIN:-36}"
EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW="${EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW:-true}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${MOTIONBRAIN_REPO:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
WORKSPACE="${MOTIONBRAIN_ROS_WS:-${REPO_DIR}/ros2_ws}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${MOTIONBRAIN_BASE_YAW_EVIDENCE_DIR:-/tmp/motionbrain_base_yaw_reference_${STAMP}}"
LOG="${OUTPUT_DIR}/capture.txt"

failures=0

MOTION_URL="${MOTION_URL%/}"
DASHBOARD_URL="${DASHBOARD_URL%/}"
mkdir -p "${OUTPUT_DIR}"
exec > >(tee "${LOG}") 2>&1

section() {
  echo
  echo "## $*"
}

run_step() {
  local label="$1"
  local rc=0
  shift
  section "${label}"
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  set +e
  "$@"
  rc=$?
  set -e
  if (( rc != 0 )); then
    echo "FAIL ${label}: exit ${rc}"
    failures=$((failures + 1))
  fi
}

capture_json() {
  local label="$1"
  local url="$2"
  local output="$3"

  run_step "${label}" curl -fsS --max-time "${CURL_TIMEOUT_SECONDS}" "${url}" -o "${output}"
  if [[ -f "${output}" ]]; then
    python3 -m json.tool "${output}" > "${output}.pretty" || {
      echo "FAIL ${label}: invalid JSON"
      failures=$((failures + 1))
    }
  fi
}

validate_feedback_json() {
  local label="$1"
  local file="$2"
  local rc=0

  section "Validate ${label}"
  printf '+ python3 validate_feedback_json %q\n' "${file}"
  set +e
  EXPECTED_FEEDBACK_READY="${EXPECTED_FEEDBACK_READY}" \
  EXPECTED_PHYSICAL_ROUTINE_ALLOWED="${EXPECTED_PHYSICAL_ROUTINE_ALLOWED}" \
  EXPECTED_BASE_YAW_FEEDBACK_FAULT="${EXPECTED_BASE_YAW_FEEDBACK_FAULT}" \
  EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY="${EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY}" \
  EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE="${EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE}" \
  EXPECTED_BASE_YAW_FEEDBACK_REFERENCED="${EXPECTED_BASE_YAW_FEEDBACK_REFERENCED}" \
  EXPECTED_BASE_YAW_FEEDBACK_PIN="${EXPECTED_BASE_YAW_FEEDBACK_PIN}" \
  EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW="${EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW}" \
  python3 - "${file}" "${label}" <<'PY'
import json
import os
import sys

path = sys.argv[1]
label = sys.argv[2]

with open(path, "r", encoding="utf-8") as handle:
    payload = json.load(handle)


def env_bool(name: str) -> bool | None:
    value = os.environ.get(name, "")
    if value == "":
        return None
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise SystemExit(f"invalid boolean expectation {name}={value!r}")


def require(path_parts: list[str], expected) -> None:
    current = payload
    for part in path_parts:
        if not isinstance(current, dict) or part not in current:
            raise SystemExit(f"{label}: missing {'.'.join(path_parts)}")
        current = current[part]
    if expected is not None and current != expected:
        raise SystemExit(
            f"{label}: {'.'.join(path_parts)} expected {expected!r}, got {current!r}"
        )
    print(f"OK {label}: {'.'.join(path_parts)}={current!r}")


require(["feedback", "selectedClosureTarget"], "base_yaw_reference")
require(["feedback", "readyForRoutineExecution"], env_bool("EXPECTED_FEEDBACK_READY"))
require(
    ["feedback", "physicalRoutineExecutionAllowed"],
    env_bool("EXPECTED_PHYSICAL_ROUTINE_ALLOWED"),
)
require(["feedback", "baseYaw", "fault"], os.environ["EXPECTED_BASE_YAW_FEEDBACK_FAULT"])
require(
    ["feedback", "baseYaw", "hardwareReady"],
    env_bool("EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY"),
)
require(
    ["feedback", "baseYaw", "signalActive"],
    env_bool("EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE"),
)
require(
    ["feedback", "baseYaw", "referenced"],
    env_bool("EXPECTED_BASE_YAW_FEEDBACK_REFERENCED"),
)
require(["feedback", "baseYaw", "pin"], int(os.environ["EXPECTED_BASE_YAW_FEEDBACK_PIN"]))
require(
    ["feedback", "baseYaw", "activeLow"],
    env_bool("EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW"),
)
PY
  rc=$?
  set -e
  if (( rc != 0 )); then
    echo "FAIL Validate ${label}: exit ${rc}"
    failures=$((failures + 1))
  fi
}

section "MotionBrain Base Yaw Reference Evidence"
date --iso-8601=seconds
hostname
uname -a
echo "MOTION_URL=${MOTION_URL}"
echo "DASHBOARD_URL=${DASHBOARD_URL}"
echo "OUTPUT_DIR=${OUTPUT_DIR}"
echo "EXPECTED_FEEDBACK_READY=${EXPECTED_FEEDBACK_READY}"
echo "EXPECTED_PHYSICAL_ROUTINE_ALLOWED=${EXPECTED_PHYSICAL_ROUTINE_ALLOWED}"
echo "EXPECTED_BASE_YAW_FEEDBACK_FAULT=${EXPECTED_BASE_YAW_FEEDBACK_FAULT}"
echo "EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY=${EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY}"
echo "EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE=${EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE}"
echo "EXPECTED_BASE_YAW_FEEDBACK_REFERENCED=${EXPECTED_BASE_YAW_FEEDBACK_REFERENCED}"
echo "EXPECTED_BASE_YAW_FEEDBACK_PIN=${EXPECTED_BASE_YAW_FEEDBACK_PIN}"
echo "EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW=${EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW}"

run_step "Git HEAD" git -C "${REPO_DIR}" log --oneline -1
run_step "Git status" git -C "${REPO_DIR}" status --short

if [[ "${CHECK_SERVICE}" == "1" ]]; then
  run_step "Bridge service active" systemctl is-active "${SERVICE_NAME}"
  run_step "Dashboard health" "${REPO_DIR}/tools/raspi/check_dashboard_health.sh"
fi

capture_json "ESP32 status" "${MOTION_URL}/status" "${OUTPUT_DIR}/01-esp32-status.json"
capture_json "ESP32 routine" "${MOTION_URL}/routine" "${OUTPUT_DIR}/02-esp32-routine.json"
capture_json "Dashboard status" "${DASHBOARD_URL}/api/status" "${OUTPUT_DIR}/03-dashboard-status.json"

for captured in \
  "${OUTPUT_DIR}/01-esp32-status.json" \
  "${OUTPUT_DIR}/02-esp32-routine.json" \
  "${OUTPUT_DIR}/03-dashboard-status.json"; do
  if [[ -f "${captured}" ]]; then
    validate_feedback_json "$(basename "${captured}")" "${captured}"
  fi
done

run_step "ROS2 bridge health with base yaw expectations" env \
  CHECK_SERVICE="${CHECK_SERVICE}" \
  ROS_DISTRO="${ROS_DISTRO}" \
  MOTIONBRAIN_ROS_WS="${WORKSPACE}" \
  SAMPLE_TIMEOUT_SECONDS="${SAMPLE_TIMEOUT_SECONDS}" \
  EXPECTED_FEEDBACK_READY="${EXPECTED_FEEDBACK_READY}" \
  EXPECTED_PHYSICAL_ROUTINE_ALLOWED="${EXPECTED_PHYSICAL_ROUTINE_ALLOWED}" \
  EXPECTED_BASE_YAW_FEEDBACK_FAULT="${EXPECTED_BASE_YAW_FEEDBACK_FAULT}" \
  EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY="${EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY}" \
  EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE="${EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE}" \
  EXPECTED_BASE_YAW_FEEDBACK_PIN="${EXPECTED_BASE_YAW_FEEDBACK_PIN}" \
  EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW="${EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW}" \
  "${REPO_DIR}/tools/raspi/check_ros_bridge_health.sh"

if [[ "${CAPTURE_FULL_ROS2_EVIDENCE}" == "1" ]]; then
  run_step "Full ROS2 read-only evidence with base yaw expectations" env \
    CHECK_SERVICE="${CHECK_SERVICE}" \
    ROS_DISTRO="${ROS_DISTRO}" \
    MOTIONBRAIN_ROS_WS="${WORKSPACE}" \
    MOTIONBRAIN_EVIDENCE_OUTPUT="${OUTPUT_DIR}/04-ros2-evidence.txt" \
    SAMPLE_TIMEOUT_SECONDS="${SAMPLE_TIMEOUT_SECONDS}" \
    CAPTURE_COMPAT_JSON=0 \
    EXPECTED_FEEDBACK_READY="${EXPECTED_FEEDBACK_READY}" \
    EXPECTED_PHYSICAL_ROUTINE_ALLOWED="${EXPECTED_PHYSICAL_ROUTINE_ALLOWED}" \
    EXPECTED_BASE_YAW_FEEDBACK_FAULT="${EXPECTED_BASE_YAW_FEEDBACK_FAULT}" \
    EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY="${EXPECTED_BASE_YAW_FEEDBACK_HARDWARE_READY}" \
    EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE="${EXPECTED_BASE_YAW_FEEDBACK_SIGNAL_ACTIVE}" \
    EXPECTED_BASE_YAW_FEEDBACK_PIN="${EXPECTED_BASE_YAW_FEEDBACK_PIN}" \
    EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW="${EXPECTED_BASE_YAW_FEEDBACK_ACTIVE_LOW}" \
    "${REPO_DIR}/tools/raspi/capture_ros2_evidence.sh"
fi

section "Summary"
echo "Output directory: ${OUTPUT_DIR}"
echo "Log: ${LOG}"
if (( failures > 0 )); then
  echo "Result: FAIL (${failures} step(s) failed)"
  exit 1
fi

echo "Result: OK"
