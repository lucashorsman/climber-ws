# Climber MCU Firmware

PlatformIO-based firmware for the Arduino Nano ESP32 MCUs controlling each arm of the
cylinder-climbing robot.

## Architecture

Each MCU runs a local control loop while accepting high-level commands from the
central ROS 2 instance via micro-ROS:

```
ROS 2 (ArmCommand) ──► micro-ROS ──► Arduino Nano ESP32
                                             │
                                       ┌─────┴─────┐
                                       │ PID Loop  │ ← RoboClaw encoder ticks
                                       │ Safety*   │
                                       └─────┬─────┘
                                             │
                       ┌─────────────────────┼─────────────────────┐
                       ▼                     ▼                     ▼
                  Wheel Motor          Actuator Motor          ToF Sensor
              (ODrive over CAN)     (RoboClaw over UART)   (1x on TCA9548A mux)
```
*\* Note: Safety override checks are currently flagged to be reviewed and re-enabled.*

## Building

```bash
# Install PlatformIO CLI
pip install platformio

# Build for configured arm (set ARM_ID in platformio.ini or override):
cd firmware/climber_mcu
pio run

# Build for a specific arm (override):
pio run --environment esp32 -DARM_ID=0   # N
pio run --environment esp32 -DARM_ID=1   # W
pio run --environment esp32 -DARM_ID=2   # S
pio run --environment esp32 -DARM_ID=3   # E

# Upload
pio run --target upload
```

## Hardware Setup (Arduino Nano ESP32)

| Function | Pin (Nano ESP32) | Notes |
|----------|-------------------|-------|
| CAN TWAI TX | D8 (GPIO 18) | CAN transceiver TX to ODrive |
| CAN TWAI RX | D9 (GPIO 17) | CAN transceiver RX from ODrive |
| RoboClaw RX | D7 | Hardware UART RX from RoboClaw TX |
| RoboClaw TX | D6 | Hardware UART TX to RoboClaw S2 (38400 baud) |
| Act Limit IN | D5 | Limit switch — fully retracted (INPUT_PULLUP) |
| Act Limit OUT | D3 | Limit switch — fully extended (INPUT_PULLUP) |
| I2C SDA | A4 | ToF sensor array & IMU |
| I2C SCL | A5 | ToF sensor array & IMU |
| ToF Sensor | TCA9548A Ch 2 | 1x VL53L0X on I2C mux ch 2 (*Note: Had issues with the multiplexor, so went down to 1 for now*) |

## Loop Structure & Rates

- **Control Loop**: **50 Hz** (*FLAG: Running at 50 Hz for now; should be running this faster once the system is proven*)
- **ToF Sensor Read**: **50 Hz**
- **micro-ROS State Publish**: **50 Hz** (`/mcu_{n,w,s,e}/arm_state`)
- **micro-ROS Command Subscribe**: Event-driven on receive (`/mcu_{n,w,s,e}/arm_cmd`)

## Firmware State Machine

```
INIT → IDLE → NORMAL ←→ EMERGENCY_GRIP
                ↓
              FAULT (latched, needs CLEAR_FAULT from ROS 2)
```

## Tuning

PID gains in `main.cpp`:
- `KP_ACTUATOR`, `KI_ACTUATOR`, `KD_ACTUATOR` — linear actuator position control (using RoboClaw encoder feedback)
- Wheel speed and position are tracked by ODrive encoder feedback over CAN.

Safety thresholds:
- `COMMS_TIMEOUT_MS` — time before emergency grip on ROS 2 silence (200ms)
- ToF contact loss threshold (200mm)

## Actuator Coordinates & Limits

The linear actuator uses physical stroke coordinates referenced to the mechanical limit switch:
- `0.00 m` (`ACTUATOR_MIN`) — Fully retracted at `PIN_ACT_LIMIT_IN` (maximum chassis clearance for installation/removal).
- `0.02 m` (`ACTUATOR_RELEASE`) — Clearance position when commanded to release.
- `0.155 m` (`EMERGENCY_GRIP`) — Gripping preload extension clamping against the cylinder surface.
- `0.16 m` (`ACTUATOR_MAX`) — Fully extended limit switch (`PIN_ACT_LIMIT_OUT`).
