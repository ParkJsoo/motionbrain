#!/usr/bin/env bash
set -euo pipefail

DASHBOARD_URL="${MOTIONBRAIN_DASHBOARD_URL:-http://127.0.0.1:8765}"
PERCEPTION_URL="${MOTIONBRAIN_PERCEPTION_URL:-http://127.0.0.1:8766}"
CHECK_SERVICE="${CHECK_SERVICE:-0}"
ALLOW_DASHBOARD_DEGRADED="${ALLOW_DASHBOARD_DEGRADED:-0}"

DASHBOARD_URL="${DASHBOARD_URL%/}"
PERCEPTION_URL="${PERCEPTION_URL%/}"

if [[ "${CHECK_SERVICE}" == "1" ]]; then
  systemctl is-active --quiet motionbrain-perception.service
  echo "OK service active: motionbrain-perception.service"
  systemctl is-active --quiet motionbrain-dashboard.service
  echo "OK service active: motionbrain-dashboard.service"
fi

curl -fsS "${PERCEPTION_URL}/health" >/dev/null
echo "OK perception health: ${PERCEPTION_URL}/health"

curl -fsS "${PERCEPTION_URL}/api/detection" >/dev/null
echo "OK perception detection: ${PERCEPTION_URL}/api/detection"

curl -fsS "${DASHBOARD_URL}/api/config" >/dev/null
echo "OK dashboard config: ${DASHBOARD_URL}/api/config"

status_payload="$(mktemp)"
cleanup() {
  rm -f "${status_payload}"
}
trap cleanup EXIT

status_code="$(
  curl -sS -o "${status_payload}" -w "%{http_code}" \
    "${DASHBOARD_URL}/api/status"
)"
if [[ "${status_code}" =~ ^2 ]]; then
  echo "OK dashboard status: ${DASHBOARD_URL}/api/status"
  exit 0
fi

if python3 - "${status_payload}" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as handle:
    payload = json.load(handle)
if not payload.get("degraded"):
    sys.exit(1)
dependency = payload.get("dependency", "unknown")
error_class = payload.get("errorClass", payload.get("error", "unknown"))
motion_ready = payload.get("motionReady")
print(f"DEGRADED dashboard status: dependency={dependency} error={error_class} motionReady={motion_ready}")
PY
then
  if [[ "${ALLOW_DASHBOARD_DEGRADED}" == "1" ]]; then
    echo "OK dashboard degraded status accepted: ${DASHBOARD_URL}/api/status"
    exit 0
  fi
  echo "FAIL dashboard status is degraded; set ALLOW_DASHBOARD_DEGRADED=1 for read-only observability checks" >&2
  exit 22
fi

cat "${status_payload}" >&2
echo "FAIL dashboard status HTTP ${status_code}: ${DASHBOARD_URL}/api/status" >&2
exit 22
