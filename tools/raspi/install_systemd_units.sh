#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${MOTIONBRAIN_REPO:-$(cd "${SCRIPT_DIR}/../.." && pwd)}"
SERVICE_USER="${MOTIONBRAIN_SERVICE_USER:-$(id -un)}"
SERVICE_HOME="${MOTIONBRAIN_SERVICE_HOME:-$(getent passwd "${SERVICE_USER}" 2>/dev/null | cut -d: -f6)}"
SERVICE_HOME="${SERVICE_HOME:-${HOME}}"
SERVICE_GROUP="${MOTIONBRAIN_SERVICE_GROUP:-$(id -gn "${SERVICE_USER}" 2>/dev/null || true)}"
SERVICE_GROUP="${SERVICE_GROUP:-${SERVICE_USER}}"
ENV_DIR="${MOTIONBRAIN_ENV_DIR:-/etc/motionbrain}"
SYSTEMD_DIR="${MOTIONBRAIN_SYSTEMD_DIR:-/etc/systemd/system}"
LIBEXEC_DIR="${MOTIONBRAIN_LIBEXEC_DIR:-/usr/local/libexec/motionbrain}"

tmp_dir="$(mktemp -d)"
cleanup() {
  rm -rf "${tmp_dir}"
}
trap cleanup EXIT

render_unit() {
  local src="$1"
  local dst="${tmp_dir}/$(basename "${src}")"
  sed \
    -e "s#User=motionbrain#User=${SERVICE_USER}#g" \
    -e "s#/home/motionbrain/develop/arduino/motionbrain#${REPO_DIR}#g" \
    -e "s#/usr/local/libexec/motionbrain#${LIBEXEC_DIR}#g" \
    "${src}" > "${dst}"
  sudo install -o root -g root -m 0644 "${dst}" "${SYSTEMD_DIR}/$(basename "${src}")"
}

render_env_example() {
  local src="$1"
  local base
  local rendered
  local target
  base="$(basename "${src}" .example)"
  rendered="${tmp_dir}/${base}.example"
  target="${ENV_DIR}/${base}"
  sed \
    -e "s#/home/motionbrain/develop/arduino/motionbrain#${REPO_DIR}#g" \
    -e "s#/home/motionbrain#${SERVICE_HOME}#g" \
    "${src}" > "${rendered}"
  sudo install -o root -g "${SERVICE_GROUP}" -m 0640 "${rendered}" "${target}.example"
  if [[ ! -e "${target}" ]]; then
    sudo install -o root -g "${SERVICE_GROUP}" -m 0640 "${rendered}" "${target}"
  else
    sudo chown root:"${SERVICE_GROUP}" "${target}"
    sudo chmod 0640 "${target}"
  fi
}

sudo install -d -o root -g "${SERVICE_GROUP}" -m 0750 "${ENV_DIR}"
sudo install -d -o root -g root -m 0755 "${LIBEXEC_DIR}"
sudo install \
  -o root \
  -g root \
  -m 0755 \
  "${REPO_DIR}/tools/raspi/reconcile_dashboard_services.sh" \
  "${LIBEXEC_DIR}/reconcile_dashboard_services.sh"
for helper in discover_device_url.py apply_camera_profile.py; do
  sudo install \
    -o root \
    -g root \
    -m 0644 \
    "${REPO_DIR}/tools/raspi/${helper}" \
    "${LIBEXEC_DIR}/${helper}"
done

for unit in "${REPO_DIR}"/deploy/systemd/*.service "${REPO_DIR}"/deploy/systemd/*.timer; do
  [[ -e "${unit}" ]] || continue
  render_unit "${unit}"
done

for env_file in "${REPO_DIR}"/deploy/systemd/*.env.example; do
  [[ -e "${env_file}" ]] || continue
  render_env_example "${env_file}"
done

sudo systemctl daemon-reload

cat <<EOF
Installed MotionBrain systemd units.

Repo: ${REPO_DIR}
User: ${SERVICE_USER}
Env:  ${ENV_DIR}
Libexec: ${LIBEXEC_DIR}

Review ${ENV_DIR}/*.env before enabling services.
EOF
