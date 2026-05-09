# Callandor

Fixed-wing flight computer for the RP2040, written in Zig & C, using the [MicroZig](https://github.com/ZigEmbeddedGroup/microzig) framework.

> **Status:** Milestone M1 complete — CRSF decoded, all 16 RC channels & link statistics printed over UART.

---

## What it does

Reads pilot input from an ELRS receiver over CRSF/UART, fuses IMU attitude data through a complementary filter, runs PID controllers for roll and pitch, and drives 4 servos + 1 ESC via 50 Hz PWM. A failsafe watchdog neutralizes all outputs if the RC link goes silent.

```
ELRS Receiver → CRSF Parser → Channel Values → [PID correction from IMU] → Mixer → PWM Outputs
                                                                                  ├─ Aileron ×2
                                                                                  ├─ Elevator
                                                                                  ├─ Rudder
                                                                                  └─ ESC (throttle)
```

---

## Hardware

| Component | Part |
|---|---|
| MCU | Raspberry Pi Pico (RP2040) |
| IMU | MPU6050 — 3-axis gyro + accelerometer (I²C) |
| RC receiver | RadioMaster RPv3, CRSF protocol over UART |
| Outputs | 4 servos (ailerons ×2, elevator, rudder) + 1 ESC using PWM|

---

## Building

Requires **Zig 0.15.1** and **MicroZig 0.15.1** (fetched automatically via `build.zig.zon`).

```sh
# Fetch dependencies (first time / after zon changes)
zig build --fetch

# Build firmware (.uf2 + .elf)
zig build

# Run unit tests (no hardware needed)
zig build test
```

Flash the resulting `.uf2` from `zig-out/firmware/` by holding BOOTSEL on the Pico while plugging in USB, then copying the file to the mass storage device that appears.

### Build options

| Flag | Default | Description |
|---|---|---|
| `-Dcalibrate=true` | `false` | Run ESC throttle-range calibration routine on boot |

> **Note:** Always run ESC calibration with propellers removed. After calibration, build without `-Dcalibrate=false` and re-flash to skip the calibration routine on boot.

---

## Architecture

### Main loop (non-blocking, polled)

| Task | Rate |
|---|---|
| UART / CRSF parsing | Every iteration (opportunistic) |
| IMU read + complementary filter | ~1 kHz |
| Failsafe check + PID + mixer + PWM update | ~200 Hz |

Timing uses the RP2040's 64-bit hardware timer at 1 µs resolution.

### Key modules

**CRSF parser** — state-machine frame parser over UART at 420 000 baud (8N1). Validates CRC-8 (poly `0xD5`) and decodes 16 RC channels packed as 11-bit values from frame type `0x16`. Channel range: 172–1811, center 992.

**IMU driver** — wakes MPU6050, configures gyro (±500 °/s) and accel (±4 g), reads 14-byte samples at ~1 kHz. Performs boot-time gyro bias calibration.

**Complementary filter** — fuses accelerometer angles (stable, noisy) with gyro integration (smooth, drifting):
```
angle = α × (prev_angle + gyro_rate × dt) + (1 − α) × accel_angle
```
Typical α: 0.95–0.99.

**PID controller** — independent loops for roll and pitch; yaw passes through directly. Output clamped to servo range; integrator frozen during saturation (anti-windup).

**Mixer** — translates roll/pitch/yaw/throttle commands to per-servo positions. Ailerons are mirrored; elevator, rudder, and throttle are direct. PID correction is applied before mixing.

**PWM output** — 50Hz signal, 1000–2000 µs pulse width (1500 µs = neutral). RP2040 config: divider = 125, wrap = 19_999 (1 µs resolution).

**Failsafe** — tracks time since last valid CRSF frame. If no valid frame arrives within 250–500 ms, overrides all channels with safe defaults (throttle minimum, surfaces neutral).

---

## Milestones

| | Title | Exit condition |
|---|---|---|
| ✅ M0 | Foundation | Toolchain flashes; PWM outputs stable; ESC arms |
| ✅ M1 | RC link | CRSF decoded; 16 channels visible over USB |
| ⬜ M2 | Manual flight | RC pass-through flyable; failsafe tested in the field |
| ⬜ M3 | IMU & sensor fusion | Stable pitch/roll angles, no drift |
| ⬜ M4 | Stabilization | PID active; surfaces resist tilt on bench |
| ⬜ M5 | Tuning & flight testing | Aircraft flies stably in stabilized mode |

---

## Safety

- **Propellers off** during all bench development with ESC connected.
- Test failsafe by physically cutting receiver power - do this before every flight session.
- ESC requires a throttle-low arming sequence at boot; the firmware handles this automatically.

---

## License

MIT
