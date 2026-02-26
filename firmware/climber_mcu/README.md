# Climber MCU Firmware

PlatformIO-based firmware for the ESP32 MCUs controlling each arm of the
cylinder-climbing robot.

## Architecture (Option C — Hybrid)

Each MCU runs a local control loop with safety overrides while accepting
high-level commands from the central ROS 2 instance via micro-ROS:

```
ROS 2 (ArmCommand) ──► micro-ROS ──► MCU
                                       │
                                 ┌─────┴─────┐
                                 │ Fast PID   │ ← ToF + encoder
                                 │ Safety     │
                                 └─────┬─────┘
                                       │
                              ┌────────┼────────┐
                              ▼        ▼        ▼
                           Wheel    Actuator   ToF
                           Motor    Motor      Sensors
```

## Building -- ignore this section

```bash
# Install PlatformIO CLI
pip install platformio

# Build for NE arm (ARM_ID=0)
cd firmware/climber_mcu
pio run

# Build for a different arm (override in platformio.ini or):
pio run --environment esp32 -DARM_ID=1   # NW
pio run --environment esp32 -DARM_ID=2   # SW
pio run --environment esp32 -DARM_ID=3   # SE

# Upload
pio run --target upload
```

## Hardware Setup -- ignore

| Function | Pin | Notes |
|----------|-----|-------|
| Wheel PWM | 25 | H-bridge speed |
| Wheel DIR | 26 | H-bridge direction |
| Wheel Enc A | 34 | Quadrature channel A (interrupt) |
| Wheel Enc B | 35 | Quadrature channel B |
| Actuator PWM | 27 | Linear actuator speed |
| Actuator DIR | 14 | Linear actuator direction |
| Act Limit IN | 32 | Limit switch — fully retracted |
| Act Limit OUT | 33 | Limit switch — fully extended |
| I2C SDA | 21 | ToF sensor array |
| I2C SCL | 22 | ToF sensor array |
| ToF XSHUT 0 | 4 | Individual sensor enable |
| ToF XSHUT 1 | 16 | Individual sensor enable |
| ToF XSHUT 2 | 17 | Individual sensor enable |

## Firmware State Machine

```
INIT → IDLE → NORMAL ←→ EMERGENCY_GRIP
                ↓
              FAULT (latched, needs CLEAR_FAULT from ROS 2)
```

## Tuning

PID gains in `main.cpp`:
- `KP_ACTUATOR`, `KI_ACTUATOR`, `KD_ACTUATOR` — actuator position control
- `ENCODER_TICKS_PER_REV` — must match your encoder

Safety thresholds:
- `COMMS_TIMEOUT_MS` — time before emergency grip on ROS 2 silence
- ToF contact loss threshold (currently 50mm)
