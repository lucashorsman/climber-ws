# Climber Robot — Hardware Bringup Guide

This document provides a step-by-step procedure for bringing up and testing the 4-arm cylinder-climbing robot on real hardware.

---

## 1. Prerequisites & Host Configuration

### 1.1 Install Udev Rules
Persistent symlinks ensure MCUs map to predictable device paths (`/dev/climber_{n,w,s,e}`) regardless of USB enumeration order:

```bash
cd ~/climber-ws/src/climber_base/config/udev
sudo cp 99-climber-mcus.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify all 4 MCUs are recognized:
```bash
ls -la /dev/climber_*
# Expected output:
# /dev/climber_n -> ttyACM0 (or similar)
# /dev/climber_w -> ttyACM1
# /dev/climber_s -> ttyACM2
# /dev/climber_e -> ttyACM3
```

### 1.2 Build ROS 2 Workspace
```bash
cd ~/climber-ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 2. Firmware Flashing (Arduino Nano ESP32)

Each MCU runs the same firmware with a unique `ARM_ID` compile flag (0=N, 1=W, 2=S, 3=E).

```bash
cd ~/climber-ws/firmware/climber_mcu

# Build and flash each arm MCU (plug in one by one, or specify upload-port):
pio run --environment esp32 -DARM_ID=0 --target upload  # North (N)
pio run --environment esp32 -DARM_ID=1 --target upload  # West  (W)
pio run --environment esp32 -DARM_ID=2 --target upload  # South (S)
pio run --environment esp32 -DARM_ID=3 --target upload  # East  (E)
```

### Pre-Power Hardware Check
- **CAN Bus**: ODrive CAN bus terminated with 120Ω resistors at each end; TX to D8, RX to D9.
- **RoboClaw**: Hardware serial connected (MCU D6 TX → RoboClaw S2 RX, MCU D7 RX → RoboClaw S1 TX) at 38400 baud.
- **Limit Switches**: Normally open switches connected between pin and GND (active LOW with internal pull-up):
  - `D5`: Retracted limit switch (`PIN_ACT_LIMIT_IN`)
  - `D3`: Extended limit switch (`PIN_ACT_LIMIT_OUT`)
- **ToF Sensor**: VL53L0X connected to I2C multiplexer channel 2 (SDA=A4, SCL=A5).

---

## 3. Step-by-Step Bringup Procedure

### Step 1: Actuator Homing Sanity Check
Upon power-up or reset, each MCU executes `initialize_actuator_motion()`:
1. Actuator moves backward toward the retracted switch (`D5`).
2. Upon contact, it breaks away forward until the switch opens.
3. This release point is calibrated as **`0.00 m`** (`actuator_home_ticks`).
4. If the switch fails to trigger within 5 seconds or fails to break away, the MCU enters `STATE_FAULT` and marks `actuator_ready = false`.

### Step 2: Launch micro-ROS Agents
Start the 4 serial bridge nodes:
```bash
ros2 launch climber_base micro_ros_agents.launch.py
```

Verify telemetry from each MCU:
```bash
ros2 topic echo /mcu_n/arm_state --once
ros2 topic echo /mcu_w/arm_state --once
ros2 topic echo /mcu_s/arm_state --once
ros2 topic echo /mcu_e/arm_state --once
```

Checklist for valid `ArmState`:
- `actuator_position` ≈ `0.00` m (retracted home)
- `tof_distances[0]` > 0.0 m (not negative error codes like -10 or -20)
- `contact_state`: `2` (`RELEASED`)

---

### Step 3: Launch Full Real-Robot Stack
In a new terminal:
```bash
source ~/climber-ws/install/setup.bash
ros2 launch climber_base real_robot.launch.py use_rviz:=true launch_agents:=false
```
*(Note: Set `launch_agents:=false` if agents were already started in Step 2).*

Verify controller manager status:
```bash
ros2 control list_controllers
# Expected active controllers:
#   joint_state_broadcaster    [active]
#   velocity_controller        [active]
#   position_controller        [active]
```

---

### Step 4: Actuator Motion & Grip Verification

Test the linear actuators using the `/grip_cmd` override topic:

1. **Retracted Clearance (Release)**:
   ```bash
   ros2 topic pub /grip_cmd std_msgs/msg/Float64 "{data: 0.02}" --once
   ```
   *Expectation*: Actuators hold position 2 cm away from retracted limit switch.

2. **Surface Approach (Touch)**:
   ```bash
   ros2 topic pub /grip_cmd std_msgs/msg/Float64 "{data: 0.15}" --once
   ```
   *Expectation*: Actuators extend 15 cm inward toward cylinder surface.

3. **Grip Preload**:
   ```bash
   ros2 topic pub /grip_cmd std_msgs/msg/Float64 "{data: 0.155}" --once
   ```
   *Expectation*: Actuators press 5 mm into tire/surface compliance. Robot state transitions to `GRIPPING`.

---

### Step 5: Wheel Drive & Velocity Verification

Once grip is confirmed, command wheel motions via `/cmd_vel`:

1. **Vertical Climb (Upward crawl)**:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.05}}" -r 10
   ```
   *Expectation*: All 4 wheels spin forward with equal sign. Robot climbs up cylinder.

2. **Orbit (Circumferential rotation)**:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.1}}" -r 10
   ```
   *Expectation*: N/S wheels spin opposite to W/E wheels. Robot orbits around cylinder.

3. **Stop**:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
   ```

---

## 4. Emergency & Fault Handling

### Manual Mode Overrides (`/climber_arm_mode`)
- **Emergency Grip (1)**: Immediate wheel stop and clamp actuators to `0.155 m`:
  ```bash
  ros2 topic pub /climber_arm_mode std_msgs/msg/UInt8 "{data: 1}" --once
  ```
- **Release (2)**: Retract actuators to `0.02 m` clearance:
  ```bash
  ros2 topic pub /climber_arm_mode std_msgs/msg/UInt8 "{data: 2}" --once
  ```
- **Clear Fault (3)**: Reset PID integrals and return MCU to `IDLE`:
  ```bash
  ros2 topic pub /climber_arm_mode std_msgs/msg/UInt8 "{data: 3}" --once
  ```
- **Normal (0)**: Resume tracking ROS 2 setpoints:
  ```bash
  ros2 topic pub /climber_arm_mode std_msgs/msg/UInt8 "{data: 0}" --once
  ```

### Common Troubleshooting

| Symptom | Probable Cause | Remedy |
|---|---|---|
| `MCU comms timeout` in ROS log | Serial disconnected or micro-ROS agent died | Verify `/dev/climber_*` symlinks and restart `micro_ros_agents.launch.py`. |
| Actuator reports `contact_state = 3 (FAULT)` | Homing failed or limit switch disconnected | Inspect limit switch wiring (`D5`). Clear fault with mode `3`. |
| ToF distance is negative (-10 or -20) | I2C communication or sensor ranging failure | Verify multiplexer wiring and pull-ups on SDA (`A4`) / SCL (`A5`). |
| ODrive closed-loop fails on boot | ODrive in error state or CAN missing 120Ω | Power cycle ODrive or run `odrv0.clearErrors()`. Verify CAN bit rate (1 Mbps). |
