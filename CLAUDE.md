# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for an ESP32-C6 acting as a remote sensor node: reads tilt (ADXL345
accelerometer) and temperature (MLX90614 IR thermometer) over I2C, and transmits them
over CAN (native TWAI) to an Arduino Nano base unit, sharing the same physical bus the
collimator already uses. Written in **pure ESP-IDF** (not Arduino), built with
PlatformIO (`framework = espidf`).

## Commands

```bash
pio run                                  # build
pio run --target upload                  # flash
pio device monitor                        # serial console (115200 baud)
pio run --target upload --target monitor  # build + flash + monitor in one step
pio run --target clean                    # clean build artifacts
```

There is no unit test suite (`test/` only has PlatformIO's placeholder README) and no
linter configured. Target board: `esp32-c6-devkitc-1`.

## Architecture

- [src/main.c](src/main.c) — `app_main()`: init sequence, then a single non-blocking
  `while(1)` loop gated by `TickType_t` timestamps (no separate FreeRTOS tasks). Reads
  keyboard input via USB Serial/JTAG driver for manual calibration triggering.
- [src/Functions.c](src/Functions.c) — all hardware logic: I2C bus + probing, ADXL345,
  MLX90614, TWAI (CAN) init/send/receive, WS2812 status LED, CAN frame assembly, and the
  accelerometer calibration state machine. Public signatures declared in
  [include/Functions.h](include/Functions.h).
- [src/NVS.c](src/NVS.c) — persists calibration offsets/gains to NVS (namespace
  `calib`), loaded back at boot in `main.c`.
- [include/Globals.h](include/Globals.h) — pin assignments, I2C addresses/registers,
  CAN IDs/commands, and `extern` globals shared between `main.c` and `Functions.c`
  (calibration state: `CalibracionValida`, `offsetX/Y/Z`, `gainX/Y/Z`). The real
  (storage-backing) definitions of those globals live in `Functions.c`, not in the
  header, to avoid duplicate symbols across translation units.

### Main loop (`app_main`, `src/main.c`)

Per iteration: drain pending CAN commands (`twai_recibir_comando`) → poll USB
Serial/JTAG for `s`/`f` keypresses (start/finish calibration) → if not calibrating, send
tilt every 1000 ms and temperature every 2000 ms → check/recover CAN bus every 2000 ms →
if calibrating, run one calibration step every 200 ms → `vTaskDelay(10 ms)`.

Deliberately uses `usb_serial_jtag_read_bytes(..., timeout=0)` instead of
`fgetc(stdin)`: with the default VFS (no USB Serial/JTAG driver installed), the first
read with no data leaves stdio in a permanent error state that `clearerr()` doesn't fix.

### Accelerometer calibration

Two-phase state machine in `Functions.c` (`calibracion_iniciar` / `calibracion_paso` /
`calibracion_finalizar`), triggerable either via USB keyboard (`s`/`f`) or a CAN command
frame on `CAN_ID_COMANDO` (`0x210`) sent by the Nano. While in progress, periodic
tilt/temperature sending is suspended and raw min/max per-axis accelerometer readings are
streamed instead (over CAN on `0x202`/`0x203`, and to the USB console). On finish,
offset/gain are computed from the accumulated extremes and validated against a plausible
gain range (`ADXL345_GAIN_MIN_ESPERADO`/`MAX_ESPERADO` in `Globals.h`) before being
accepted, persisted to NVS, and applied — an implausible gain means an axis wasn't
actually swept through ±1g, so the whole calibration is rejected without touching any
previously valid one. Tilt (pitch/roll) is only ever sent over CAN when
`CalibracionValida` is true.

### CAN protocol

Full frame-by-frame spec (IDs, byte layout, units) is in
[docs/PROTOCOLO_CAN.md](docs/PROTOCOLO_CAN.md). Summary: `0x200` tilt, `0x201`
temperature, `0x202`/`0x203` calibration min/max extremes (raw counts, sent only during
calibration), `0x210` commands from the Nano (`0x01` start / `0x02` stop calibration).
All multi-byte fields are little-endian; standard 11-bit IDs; bus runs at 100 kbit/s
(`TWAI_TIMING_CONFIG_100KBITS()` in `Functions.c` — other bitrates are present but
commented out, change there if the shared bus with the collimator uses a different one).

### Status LED (WS2812, GPIO8)

Colors are set directly at the call site in `Functions.c`/`main.c`, not centralized:
rainbow at boot, dim green = normal, amber = no valid calibration loaded, blue = ADXL345
read failure, red = MLX90614 read failure or a rejected calibration.

## Comments and language

Source comments and log messages are in Spanish and explain *why*, not what — this
codebase leans on comments to record non-obvious constraints (e.g. the stdio/USB Serial
JTAG quirk in `main.c`, the raw-domain math note in `enviar_inclinacion`, the plausible
gain range rationale). Match that style when adding new comments: explain the
non-obvious reason, not the mechanics.

## Hardware reference

Pins and addresses are centralized in `include/Globals.h` (I2C SDA=GPIO2, SCL=GPIO3;
TWAI TX=GPIO4, RX=GPIO5; LED=GPIO8; ADXL345 at `0x53`/`0x1D`, MLX90614 at `0x5A`). See
[README.md](README.md) for the full wiring table and calibration usage instructions.
