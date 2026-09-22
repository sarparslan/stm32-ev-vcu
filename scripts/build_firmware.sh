#!/usr/bin/env bash
#
# Command-line firmware build (used by CI).
#
# STM32CubeIDE remains the main way to build and flash; this script
# compiles and links the same sources with arm-none-eabi-gcc so every
# push is checked to still build, with all warnings in the
# application code treated as errors.
#
#   scripts/build_firmware.sh [build-dir]
#
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/build/firmware}"
CC="${CC:-arm-none-eabi-gcc}"
SIZE="${SIZE:-arm-none-eabi-size}"

CPU=(-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
DEFS=(-DUSE_HAL_DRIVER -DSTM32F407xx)
INCS=(
  -I"$ROOT/Core/Inc"
  -I"$ROOT/Drivers/STM32F4xx_HAL_Driver/Inc"
  -I"$ROOT/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"
  -I"$ROOT/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
  -I"$ROOT/Drivers/CMSIS/Include"
  -I"$ROOT/Middlewares/Third_Party/FreeRTOS/Source/include"
  -I"$ROOT/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
  -I"$ROOT/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
)
CFLAGS=(-std=gnu11 -Os -g -ffunction-sections -fdata-sections --specs=nano.specs -Wall)
APP_FLAGS=(-Wextra -Werror)

# Application modules get the strict flags; vendor code (HAL, CMSIS,
# FreeRTOS, CubeMX system files) is compiled as-is.
APP_SOURCES=(app bms_can can_bus charger_can freertos led_diag main motor_can vehicle_control)

rm -rf "$OUT"
mkdir -p "$OUT/obj"

is_app_source() {
  local name
  name="$(basename "$1" .c)"
  for app in "${APP_SOURCES[@]}"; do
    [[ "$name" == "$app" && "$1" == Core/Src/* ]] && return 0
  done
  return 1
}

objects=()
cd "$ROOT"
while IFS= read -r src; do
  obj="$OUT/obj/$(echo "$src" | tr '/' '_').o"
  extra=()
  if is_app_source "$src"; then
    extra=("${APP_FLAGS[@]}")
  fi
  echo "CC  $src"
  "$CC" "${CPU[@]}" "${DEFS[@]}" "${INCS[@]}" "${CFLAGS[@]}" ${extra[@]+"${extra[@]}"} -c "$src" -o "$obj"
  objects+=("$obj")
done < <(find Core Drivers Middlewares -name '*.c' | sort)

echo "AS  Core/Startup/startup_stm32f407vgtx.s"
"$CC" "${CPU[@]}" -x assembler-with-cpp -c Core/Startup/startup_stm32f407vgtx.s -o "$OUT/obj/startup.o"

echo "LD  $OUT/stm32-ev-vcu.elf"
"$CC" "${CPU[@]}" -T STM32F407VGTX_FLASH.ld --specs=nosys.specs --specs=nano.specs \
  -Wl,--gc-sections -Wl,-Map="$OUT/stm32-ev-vcu.map" \
  -o "$OUT/stm32-ev-vcu.elf" "${objects[@]}" "$OUT/obj/startup.o" -lc -lm

"$SIZE" "$OUT/stm32-ev-vcu.elf"
