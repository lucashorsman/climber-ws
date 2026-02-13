# Climber Robot — Cylinder-Climbing Mecanum Platform

A ROS 2 (Jazzy) robot that wraps around a vertical cylinder (pole/tree) and climbs it using 4 mecanum wheels in an X formation. Each wheel is mounted on a linear actuator that controls radial grip pressure, with ToF sensors providing surface distance feedback.

```
        NW (135°)       NE (45°)
            \           /
             [  pole  ]
            /           \
        SW (225°)       SE (315°)
```

## Repository Structure

```
climber-ws/
├── src/
│   ├── climber_base/                   # Main robot package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── climber_base_hardware.xml   # pluginlib registration for HW interface
│   │   ├── ARCHITECTURE.md             # Legacy architecture doc
│   │   ├── config/
│   │   │   ├── climber_controllers.yaml    # ros2_control controller config
│   │   │   ├── mecanum_controllers.yaml    # Legacy (unused)
│   │   │   ├── view_robot.rviz            # RViz config
│   │   │   └── udev/
│   │   │       └── 99-climber-mcus.rules  # Stable serial symlinks for MCUs
│   │   ├── include/climber_base/
│   │   │   └── climber_hardware_interface.hpp
│   │   ├── launch/
│   │   │   ├── display.launch.py           # RViz + joint sliders (no sim)
│   │   │   ├── gazebo_sim.launch.py        # Full Gazebo simulation
│   │   │   ├── real_robot.launch.py        # Real hardware launch
│   │   │   └── micro_ros_agents.launch.py  # 4x serial micro-ROS agents
│   │   ├── src/
│   │   │   ├── climber_hardware_interface.cpp  # ros2_control ↔ micro-ROS bridge
│   │   │   └── cylinder_climb_controller.py    # Central climb controller node
│   │   └── urdf/
│   │       ├── climber_robot.urdf.xacro    # Main robot description
│   │       ├── ros2_control.xacro          # Hardware interface definitions
│   │       ├── gazebo.xacro                # Gazebo materials/friction/plugin
│   │       ├── materials.xacro             # RViz colors
│   │       ├── cylinder_climb.world        # Gazebo world with 5m pole
│   │       └── empty.world                 # Legacy
│   └── climber_msgs/                   # Custom message definitions
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── msg/
│           ├── ArmState.msg            # MCU → ROS 2 sensor feedback
│           ├── ArmCommand.msg          # ROS 2 → MCU setpoints + mode
│           └── ClimberState.msg        # Aggregated robot state
└── firmware/
    └── climber_mcu/                    # ESP32 PlatformIO firmware
        ├── platformio.ini
        ├── README.md
        └── src/
            └── main.cpp                # Firmware skeleton
```

---

## System Architecture

### Option C — Hybrid Control

Global decision-making in ROS 2 with fast local safety loops on each MCU.

```
                     ┌────────────────────────────────┐
   /cmd_vel ────────►│   cylinder_climb_controller    │
   /grip_cmd ───────►│                                │
                     │  State machine + grip manager   │
                     │  Reads: 4x /mcu_*/arm_state     │
                     │  Publishes: /climber_state       │
                     └──────┬───────┬───────┬───────┬──┘
                            │       │       │       │
                  ┌─────────▼──┐ ┌──▼──┐ ┌──▼──┐ ┌──▼─────────┐
                  │ velocity_  │ │     │ │     │ │ position_   │
                  │ controller │ │     │ │     │ │ controller  │
                  └─────┬──────┘ │     │ │     │ └──────┬──────┘
                        └────────┴──┬──┴─┴──┬──┘        │
                                    │       │           │
                     ┌──────────────▼───────▼───────────▼──┐
                     │     ClimberHardwareInterface         │
                     │  (ros2_control SystemInterface)      │
                     │                                      │
                     │  Subscribes: /mcu_*/arm_state        │
                     │  Publishes:  /mcu_*/arm_cmd          │
                     └──────┬───────┬───────┬───────┬──────┘
                            │       │       │       │  micro-ROS
                       ┌────▼┐ ┌────▼┐ ┌────▼┐ ┌────▼┐
                       │ NE  │ │ NW  │ │ SW  │ │ SE  │  MCUs
                       │ PID │ │ PID │ │ PID │ │ PID │  (ESP32)
                       │ ToF │ │ ToF │ │ ToF │ │ ToF │
                       │safe │ │safe │ │safe │ │safe │
                       └─────┘ └─────┘ └─────┘ └─────┘
```

### Data Flow

| Path | Topic | Message Type | Rate |
|------|-------|-------------|------|
| Teleop → Controller | `/cmd_vel` | `geometry_msgs/Twist` | 10 Hz |
| Manual grip | `/grip_cmd` | `std_msgs/Float64` | on demand |
| Controller → Wheels | `/velocity_controller/commands` | `Float64MultiArray` | 20 Hz |
| Controller → Actuators | `/position_controller/commands` | `Float64MultiArray` | 20 Hz |
| HW Interface → MCU | `/mcu_{ne,nw,sw,se}/arm_cmd` | `climber_msgs/ArmCommand` | 100 Hz |
| MCU → HW Interface | `/mcu_{ne,nw,sw,se}/arm_state` | `climber_msgs/ArmState` | 50 Hz |
| Controller → Monitor | `/climber_state` | `climber_msgs/ClimberState` | 20 Hz |
| JSB → TF | `/joint_states` | `sensor_msgs/JointState` | 100 Hz |

---

## Messages (`climber_msgs`)

### ArmState (MCU → ROS 2)
| Field | Type | Description |
|-------|------|-------------|
| `tof_distances` | `float32[]` | ToF sensor array — distances to surface (m) |
| `actuator_position` | `float32` | Current actuator position (m). 0 = touching |
| `actuator_velocity` | `float32` | Current actuator velocity (m/s) |
| `wheel_position` | `float32` | Accumulated wheel angle (rad) |
| `wheel_velocity` | `float32` | Wheel angular velocity (rad/s) |
| `contact_state` | `uint8` | 0=UNKNOWN, 1=GRIPPING, 2=RELEASED, 3=FAULT |

### ArmCommand (ROS 2 → MCU)
| Field | Type | Description |
|-------|------|-------------|
| `actuator_setpoint` | `float32` | Desired actuator position (m) |
| `wheel_velocity` | `float32` | Desired wheel velocity (rad/s) |
| `mode` | `uint8` | 0=NORMAL, 1=EMERGENCY_GRIP, 2=RELEASE, 3=CLEAR_FAULT |

### ClimberState (aggregated)
| Field | Type | Description |
|-------|------|-------------|
| `arms` | `ArmState[4]` | Per-arm states [NE, NW, SW, SE] |
| `robot_state` | `uint8` | 0=IDLE, 1=APPROACHING, 2=GRIPPING, 3=CLIMBING, 4=ORBITING, 5=STOPPED, 6=FAULT |

---

## URDF Model

### Kinematic Chain (per arm)

```
base_link (cylinder axis)
 └── {prefix}_frame_node (fixed) ─── outer frame attachment
 └── {prefix}_anchor (fixed) ─────── actuator mount point
      └── {prefix}_carriage (prismatic: {prefix}_actuator_joint)
           └── {prefix}_wheel (continuous: {prefix}_wheel_joint)
```

Plus 4 arc links connecting frame nodes into an outer ring.

### Key Parameters

| Property | Value | Description |
|----------|-------|-------------|
| `cylinder_diameter` | 0.15 m | Pole diameter (change this for different trees) |
| `wheel_radius` | 0.06 m | Mecanum wheel radius |
| `wheel_width` | 0.05 m | Wheel width along axle |
| `actuator_stroke` | 0.05 m | Total actuator travel |
| `actuator_min` | -0.01 m | Max press-in (grip) |
| `actuator_max` | 0.04 m | Max pull-out (release) |

### Joint Conventions

**Wheel joints** (`{ne,nw,sw,se}_wheel_joint`):
- Type: continuous
- Axis: `0 -1 0` (negative tangential Y)
- Positive velocity = climb UP

**Actuator joints** (`{ne,nw,sw,se}_actuator_joint`):
- Type: prismatic
- Axis: `-1 0 0` (radially inward)
- Position 0.0 = wheel touching cylinder
- Position < 0 = pressing into cylinder (grip)
- Position > 0 = pulling away (release)

---

## ros2_control

### Hardware Modes

Controlled by xacro args `use_sim` and `sim_gz`:

| Mode | `use_sim` | `sim_gz` | Plugin |
|------|-----------|----------|--------|
| Gazebo | `true` | `true` | `gz_ros2_control/GazeboSimSystem` |
| Real hardware | `false` | `false` | `climber_base/ClimberHardwareInterface` |

### Controllers

| Controller | Type | Joints | Topic |
|------------|------|--------|-------|
| `joint_state_broadcaster` | JointStateBroadcaster | all 8 | `/joint_states` |
| `velocity_controller` | JointGroupVelocityController | 4 wheel | `/velocity_controller/commands` |
| `position_controller` | JointGroupPositionController | 4 actuator | `/position_controller/commands` |

### Hardware Interface (`ClimberHardwareInterface`)

A C++ `SystemInterface` plugin that bridges ros2_control to the MCUs:

- Creates an internal ROS 2 node added to the controller_manager's executor
- Subscribes to `/mcu_*/arm_state` — updates joint state variables (wheel encoder + actuator position) read by controller_manager
- Publishes to `/mcu_*/arm_cmd` — forwards velocity/position commands from controllers
- Monitors per-arm communication timeout (configurable via `comms_timeout` URDF parameter)

---

## Climb Controller (`cylinder_climb_controller.py`)

The central decision-making node. Operates in two modes:

### Simulation Mode (`use_real_hardware:=false`)
- Subscribes to `/cmd_vel` and `/grip_cmd`
- Publishes to velocity and position controller command topics
- No MCU topic interaction

### Real Hardware Mode (`use_real_hardware:=true`)
- All of the above, plus:
- Subscribes to `/mcu_*/arm_state` for ToF and contact monitoring
- Publishes `/climber_state` with aggregated robot state
- Runs auto-grip adjustment using ToF feedback
- Monitors MCU comms health, triggers FAULT on timeout

### State Machine

```
IDLE → APPROACHING → GRIPPING → CLIMBING ←→ ORBITING
                                    ↓
                                 STOPPED → IDLE
                          (any) → FAULT
```

- **FAULT**: Latched on comms timeout or MCU fault. Commands emergency grip (all actuators press in) and stops wheels.
- **GRIPPING → CLIMBING**: Triggered when `/cmd_vel` has nonzero `linear.z`
- **CLIMBING → STOPPED**: Triggered on `/cmd_vel` timeout
- Safety timeout stops wheels if no `/cmd_vel` received for 0.5s

### Grip Manager

When running on real hardware with ToF data:
- Reads minimum ToF distance per arm
- Applies proportional correction to maintain `grip_distance_target` (3mm default)
- Per-arm independent — handles asymmetric tree bark / varying diameters
- Step-limited to ±2mm per cycle to prevent overshoot

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_radius` | 0.06 | Must match URDF |
| `cylinder_radius` | 0.075 | Must match URDF `cylinder_diameter/2` |
| `max_wheel_vel` | 10.0 | Clamp for wheel velocity (rad/s) |
| `cmd_vel_timeout` | 0.5 | Safety stop timeout (s) |
| `default_grip_position` | -0.005 | Default actuator position (m) |
| `grip_distance_target` | 0.003 | Target ToF reading when gripped (m) |
| `grip_tolerance` | 0.002 | Acceptable ToF deviation (m) |
| `grip_press_position` | -0.005 | Actuator position for grip (m) |
| `release_position` | 0.03 | Actuator position for release (m) |
| `max_climb_speed` | 0.5 | Speed limit during climb (m/s) |
| `use_real_hardware` | false | Enable MCU topic subscriptions |
| `comms_timeout` | 0.5 | Fault trigger on MCU silence (s) |

### Mecanum Kinematics

```
v_orbit = cylinder_radius × angular.z

NE = (vz + v_orbit) / wheel_radius
NW = (vz - v_orbit) / wheel_radius
SW = (vz + v_orbit) / wheel_radius
SE = (vz - v_orbit) / wheel_radius
```

| Motion | NE | NW | SW | SE |
|--------|----|----|----|----|
| Climb up | + | + | + | + |
| Orbit CCW | + | − | + | − |

---

## MCU Firmware (`firmware/climber_mcu/`)

PlatformIO project targeting ESP32. Each MCU controls one arm assembly.

### Firmware State Machine

```
INIT → IDLE → NORMAL ←→ EMERGENCY_GRIP
                ↓
              FAULT (latched — needs CLEAR_FAULT from ROS 2)
```

### Loop Structure

| Loop | Rate | Function |
|------|------|----------|
| Actuator PID | 1 kHz | Position control using sensor feedback |
| Wheel motor | 1 kHz | Velocity passthrough to H-bridge |
| ToF sensors | 100 Hz | Read VL53L0X/VL53L1X array |
| micro-ROS publish | 50 Hz | Send `ArmState` to ROS 2 |
| micro-ROS spin | ~1 kHz | Process incoming `ArmCommand` |

### Safety Features

- **Comms timeout** (200ms): If no `ArmCommand` received from ROS 2, firmware enters EMERGENCY_GRIP — clamps actuator in, stops wheel.
- **Contact loss detection**: If all ToF sensors read > 50mm (lost surface), triggers EMERGENCY_GRIP.
- **Limit switches**: Actuator PID respects physical limit switches at both ends of travel.
- **FAULT latch**: Once in FAULT, MCU holds position until ROS 2 sends explicit CLEAR_FAULT.

### Pin Mapping

| Function | GPIO | Notes |
|----------|------|-------|
| Wheel PWM | 25 | H-bridge speed |
| Wheel DIR | 26 | H-bridge direction |
| Wheel Enc A | 34 | Quadrature A (ISR) |
| Wheel Enc B | 35 | Quadrature B |
| Actuator PWM | 27 | Linear actuator |
| Actuator DIR | 14 | Actuator direction |
| Limit IN | 32 | Fully retracted switch |
| Limit OUT | 33 | Fully extended switch |
| I2C SDA | 21 | ToF sensor array |
| I2C SCL | 22 | ToF sensor array |
| XSHUT 0–2 | 4, 16, 17 | Individual ToF enable |

### Building

```bash
cd firmware/climber_mcu

# Build for NE arm (default ARM_ID=0)
pio run

# Override arm ID for other MCUs:
# Edit platformio.ini: -DARM_ID=1 (NW), 2 (SW), 3 (SE)

# Upload
pio run --target upload
```

---

## Getting Started

### Prerequisites

- ROS 2 Jazzy
- Gazebo Sim (Harmonic)
- PlatformIO (for firmware)

```bash
# Install ROS 2 dependencies
sudo apt install \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-micro-ros-agent
```

### Build

```bash
cd ~/GIT/climber-ws
colcon build
source install/setup.bash   # or setup.zsh
```

### Launch — RViz Only (inspect URDF)

No simulation, no controllers. Joint slider GUI lets you move all 8 joints manually.

```bash
ros2 launch climber_base display.launch.py
```

Drag the `*_actuator_joint` sliders to see wheels move radially. Drag `*_wheel_joint` sliders to spin wheels.

### Launch — Gazebo Simulation

Full stack with ros2_control, all controllers, and the climb controller node.

```bash
ros2 launch climber_base gazebo_sim.launch.py
```

Then in separate terminals:

```bash
# Verify controllers are active
ros2 control list_controllers

# Climb up
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}" -r 10

# Grip (press wheels in 5mm)
ros2 topic pub --once /grip_cmd std_msgs/msg/Float64 "{data: -0.005}"

# Release
ros2 topic pub --once /grip_cmd std_msgs/msg/Float64 "{data: 0.03}"

# Monitor joint states
ros2 topic echo /joint_states
```

### Launch — Real Robot

Requires 4 MCUs connected via USB serial.

**1. Set up udev rules:**

```bash
# Find each MCU's serial number
udevadm info -a -n /dev/ttyACM0 | grep '{serial}'

# Edit the rules file with actual serial numbers
nano src/climber_base/config/udev/99-climber-mcus.rules

# Install
sudo cp src/climber_base/config/udev/99-climber-mcus.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Verify
ls -la /dev/climber_*
```

**2. Flash firmware to each MCU:**

```bash
cd firmware/climber_mcu
# Edit platformio.ini: set ARM_ID per MCU (0=NE, 1=NW, 2=SW, 3=SE)
pio run --target upload
```

**3. Launch:**

```bash
ros2 launch climber_base real_robot.launch.py
```

This starts:
- Robot State Publisher (URDF with `ClimberHardwareInterface`)
- `ros2_control_node` (loads the hardware interface plugin)
- Joint State Broadcaster + Velocity Controller + Position Controller
- Cylinder Climb Controller (with `use_real_hardware:=true`)
- 4 micro-ROS serial agents
- RViz

**4. Command:**

```bash
# Grip the pole
ros2 topic pub --once /grip_cmd std_msgs/msg/Float64 "{data: -0.005}"

# Climb
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.3}}" -r 10

# Monitor robot state
ros2 topic echo /climber_state

# Check MCU health
ros2 topic hz /mcu_ne/arm_state
ros2 topic hz /mcu_nw/arm_state
ros2 topic hz /mcu_sw/arm_state
ros2 topic hz /mcu_se/arm_state
```

### Launch Arguments

| Launch File | Argument | Default | Description |
|-------------|----------|---------|-------------|
| `display.launch.py` | `use_gui` | `true` | Joint slider GUI |
| `gazebo_sim.launch.py` | `use_sim_time` | `true` | Gazebo clock |
| | `use_rviz` | `true` | Launch RViz |
| `real_robot.launch.py` | `use_rviz` | `true` | Launch RViz |
| | `launch_agents` | `true` | Start micro-ROS agents |
| `micro_ros_agents.launch.py` | `{ne,nw,sw,se}_serial_dev` | `/dev/climber_*` | Serial device path |
| | `{ne,nw,sw,se}_baud` | `921600` | Serial baud rate |

---

## Bring-Up Sequence (Real Robot)

Follow this staged testing procedure:

| Stage | What | How |
|-------|------|-----|
| 1 | Single MCU on bench | Flash one MCU, connect motor + ToF. Verify topics with `ros2 topic echo /mcu_ne/arm_state`. Send manual commands. |
| 2 | 4 MCUs connected | All 4 agents up. `ros2 topic list \| grep mcu` should show 8 topics. Check rates with `ros2 topic hz`. |
| 3 | Open-loop control | Send ArmCommand via CLI. Confirm each arm moves correctly, wheels spin right direction. |
| 4 | Grip on static pole | Robot on pole, send grip command. Verify all 4 arms achieve contact, ToF converges. |
| 5 | Climb with teleop | Joystick/keyboard → `/cmd_vel`. Climb up/down with manual grip. |
| 6 | Auto-grip | Enable grip manager (`use_real_hardware:=true`). Test on varying diameter. |
| 7 | Fault injection | Kill one MCU mid-climb. Verify safety clamp on remaining 3. Robot must not fall. |

---

## Customization

### Different Pole Diameter

1. Change `cylinder_diameter` in `climber_robot.urdf.xacro`
2. Update `cylinder_radius` parameter in launch files (or rely on defaults matching URDF)
3. Update pole radius in `cylinder_climb.world` to match

### Different Wheel Size

1. Change `wheel_radius` and `wheel_width` in xacro
2. Update `wheel_radius` parameter in launch files
3. Update `WHEEL_RADIUS` in firmware

### Different MCU Platform

The firmware skeleton targets ESP32 but the structure is portable:
1. Update `platformio.ini` with your board
2. Adapt pin definitions in `main.cpp`
3. Change micro-ROS transport if not using serial (WiFi, Ethernet, etc.)

---

## Troubleshooting

| Problem | Diagnosis | Fix |
|---------|-----------|-----|
| Robot falls off pole in Gazebo | No mechanical constraint in sim | Use `/grip_cmd` with negative value; increase friction in `gazebo.xacro` |
| Controllers not loading | `ros2 control list_controllers` shows inactive | Check `climber_controllers.yaml` joint names match URDF |
| MCU topic not appearing | micro-ROS agent not connecting | Check serial port path, baud rate, udev rules; run agent manually |
| FAULT state won't clear | Latched fault | Send `CLEAR_FAULT` mode: `ros2 topic pub --once /mcu_ne/arm_cmd climber_msgs/msg/ArmCommand "{mode: 3}"` |
| Wheels spin wrong direction | Encoder wiring or axis sign | Swap encoder A/B wires or invert `PIN_WHEEL_DIR` logic |
| Actuator overshoots | PID gains too aggressive | Reduce `KP_ACTUATOR` in firmware, increase `KD_ACTUATOR` |
| `climber_msgs` import error in controller | Package not sourced | `source install/setup.bash` after building |
