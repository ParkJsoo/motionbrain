#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_NAME="${STM32_PROJECT_NAME:-MotionBrainSensor}"
CONFIG="${STM32_BUILD_CONFIG:-Debug}"
ELF="${STM32_ELF:-${ROOT_DIR}/firmware/stm32/${PROJECT_NAME}/${CONFIG}/${PROJECT_NAME}.elf}"
PROGRAMMER="${STM32_PROGRAMMER_CLI:-/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/Resources/bin/STM32_Programmer_CLI}"

if [[ ! -f "${ELF}" ]]; then
  echo "ELF not found: ${ELF}" >&2
  echo "Run tools/stm32_build.sh first." >&2
  exit 1
fi

if [[ ! -x "${PROGRAMMER}" ]]; then
  echo "STM32_Programmer_CLI not found: ${PROGRAMMER}" >&2
  exit 1
fi

# On this Mac, native arm64 STM32_Programmer_CLI fails with:
# "Incompatible processor. This Qt build requires the following features: neon".
# Forcing the x86_64 slice via Rosetta works.
ARCH_PREFIX=()
if [[ "$(uname -m)" == "arm64" ]]; then
  ARCH_PREFIX=(arch -x86_64)
fi

"${ARCH_PREFIX[@]}" "${PROGRAMMER}" \
  -c port=SWD mode=UR reset=HWrst \
  -w "${ELF}" \
  -v \
  -rst
