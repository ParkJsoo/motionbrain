# MotionBrain Homing and Feedback Plan

Last updated: 2026-06-11 KST

## Current Position

MotionBrain does not currently support true automatic homing.

The arm uses DC motors without per-joint encoders, limit switches, or absolute
position sensors. The firmware can command direction, PWM percent, and bounded
time, but it cannot prove the absolute joint position after boot.

Current supported routine:

- `soft_home_reference`

This is an operator-confirmed software reference procedure. It has zero motion
steps. The operator places the arm in the agreed reference pose, then the
firmware records the procedure boundary in the routine interface. It does not
seek hard stops, infer absolute pose, or calibrate encoder-grade position.

## What Counts As Real Homing

Real homing needs a repeatable physical signal that defines a reference point.

Acceptable signals:

- Limit switch per homed axis.
- Hall sensor plus magnet per homed axis.
- Absolute magnetic encoder per measured axis.
- Other mechanically repeatable position sensor with explicit failure states.

Not acceptable on current hardware:

- Timed motion alone.
- Driving into mechanical hard stops without current or force feedback.
- Treating IMU/base gyro feedback as full joint homing.

## Recommended Hardware Paths

### Path A: Limit Switch Home Sensors

Use one normally-closed or normally-open switch per homed axis.

Pros:

- Simple firmware model.
- Cheap.
- Clear home/not-home signal.
- Good first step for one or two axes.

Cons:

- Needs careful mechanical mounting.
- Switch bounce requires debounce.
- Only gives a reference point, not continuous pose.

Recommended first target:

- Base or gripper only, read-only status first.

### Path B: Hall Sensor Home Sensors

Use one digital Hall sensor and magnet per homed axis.

Pros:

- Contactless.
- Cheap.
- Less mechanical wear than microswitches.

Cons:

- Magnet alignment matters.
- Detection threshold and hysteresis need bench validation.
- Still only gives a reference point unless multiple magnets or analog sensing
  are added.

### Path C: Absolute Magnetic Encoders

Use an angle sensor such as AS5600-class modules with a diametric magnet mounted
to a rotating joint.

Pros:

- Provides absolute angle after boot.
- Enables better routine validation than limit switches alone.
- Better long-term path for pose-aware operation.

Cons:

- Mechanical mounting is the hard part.
- Multiple identical I2C sensors need a mux, separate buses, or non-I2C output
  strategy.
- Cable routing and magnet centering matter.
- More expensive than simple home sensors.

Recommended first target:

- One accessible axis only, as read-only telemetry.

## Pin and Bus Notes

The ESP32 motor controller is already pin-constrained by five DC motors, UART
teleop input, and existing safety/status paths.

Prefer one of these architectures:

- STM32 sensor/teleop node reads homing sensors and forwards debounced status.
- I2C GPIO expander reads limit/Hall sensors and exposes them to ESP32.
- I2C mux or separate sensor node handles multiple magnetic encoders.

Do not add homing sensors directly to random unused ESP32 pins without updating
`PIN_MAP.md`, boot-strapping constraints, and failure behavior.

## Firmware Contract Before Motion

Add any homing hardware in this order:

1. Read-only telemetry.
2. Explicit failure states: disconnected, stale, noisy, uncalibrated.
3. Status/API documentation.
4. Safety/preflight blocking.
5. Only then consider bounded low-speed homing motion.

Suggested future status shape:

```json
{
  "homing": {
    "mode": "soft_reference",
    "supported": false,
    "referenced": false,
    "hardware": "none",
    "detail": "operator-confirmed software reference only"
  }
}
```

For hardware-backed homing:

```json
{
  "homing": {
    "mode": "limit_switch",
    "supported": true,
    "referenced": true,
    "hardware": "base_limit_switch",
    "lastReferenceMs": 123456,
    "fault": "none"
  }
}
```

## Acceptance Boundary

A homing implementation is not complete until:

- It is visible in `GET /status`.
- It is documented in `MESSAGE_INTERFACE.md`.
- Fault and stale states are visible.
- It blocks unsafe routine execution when required.
- It has a non-motion read-only smoke test.
- Physical homing motion, if added, is low-speed, bounded, abortable, and
  operator-confirmed.

