#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_DIR="${ROOT_DIR}/firmware/stm32/MotionBrainSensor"
WORKSPACE_DIR="${STM32_WORKSPACE_DIR:-/tmp/motionbrain-stm32-workspace}"
CUBEIDE="${STM32_CUBEIDE:-/Applications/STM32CubeIDE.app/Contents/MacOS/STM32CubeIDE}"
PROJECT_NAME="${STM32_PROJECT_NAME:-MotionBrainSensor}"
CONFIG="${STM32_BUILD_CONFIG:-Debug}"

if [[ ! -x "${CUBEIDE}" ]]; then
  echo "STM32CubeIDE executable not found: ${CUBEIDE}" >&2
  exit 1
fi

"${CUBEIDE}" \
  -nosplash \
  -data "${WORKSPACE_DIR}" \
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
  -import "${PROJECT_DIR}" \
  -cleanBuild "${PROJECT_NAME}/${CONFIG}" \
  -no-indexer \
  -printErrorMarkers

echo "Built: ${PROJECT_DIR}/${CONFIG}/${PROJECT_NAME}.elf"
