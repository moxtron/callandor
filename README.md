# Callandor

Fixed-wing flight computer on the RP2040, written in Zig and C with MicroZig.

> **Status:** Early development — basic servo functionality in place. PWM interupts are next.

---

## What it does

Takes pilot input from an ELRS receiver (CRSF over UART), reads attitude from an MPU6050 IMU, runs a complementary filter and PID controller, and drives 4 servos + 1 ESC via PWM. A failsafe watchdog neutralizes outputs if the RC link goes silent.

## Hardware

- **MCU:** Raspberry Pi RP2040
- **IMU:** MPU6050 (I²C)
- **RC link:** ELRS receiver, CRSF protocol
- **Outputs:** 4 servos + 1 ESC

## Building

Requires Zig 0.15.x.

```sh
zig build        # build firmware
zig build test   # run unit tests (no hardware needed)
```

## Milestones

| | Title | Exit condition |
|-|-------|----------------|
| M0 | Foundation | Toolchain flashes; PWM outputs stable; ESC arms |
| M1 | RC link | CRSF decoded; 16 channels visible over USB |
| M2 | Manual flight | Pass-through flyable; failsafe tested |
| M3 | IMU & sensor fusion | Stable pitch/roll angles, no drift |
| M4 | Stabilization | PID active; surfaces resist tilt on bench |
| M5 | Tuning & flight testing | Aircraft flies stably in stabilized mode |

## License

MIT
