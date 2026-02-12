# climber_base — Cylinder-Climbing Mecanum Robot

## Overview

`climber_base` is a ROS 2 (Jazzy) package that defines a 4-mecanum-wheel robot designed to wrap around a vertical cylinder (pole/tree) and climb it. The robot uses an X-formation wheel layout where all four wheels press inward against the cylinder surface. Driving all wheels together moves the robot vertically; driving diagonal pairs in opposite directions orbits the robot around the cylinder.

```
        NW (135°)       NE (45°)
            \           /
             [  pole  ]
            /           \
        SW (225°)       SE (315°)
```

The package provides:
- A parametric URDF/xacro model of the robot
- A kinematic controller node (`cylinder_climb_controller.py`)
- ros2_control integration (velocity-controlled wheels)
- Gazebo Sim launch with a world containing a climbable pole
- RViz visualization launch

---

## File Structure

```
src/climber_base/
├── CMakeLists.txt                    # Build system configuration
├── package.xml                       # Package manifest & dependencies
├── config/
│   ├── climber_controllers.yaml      # ros2_control controller config (active)
│   ├── mecanum_controllers.yaml      # Legacy flat-ground mecanum config (unused)
│   └── view_robot.rviz              # RViz display config
├── launch/
│   ├── display.launch.py            # RViz-only visualization (no sim)
│   └── gazebo_sim.launch.py         # Full Gazebo simulation launch
├── src/
│   ├── __init__.py
│   └── cylinder_climb_controller.py # Kinematic controller (cmd_vel → wheel vels)
└── urdf/
    ├── climber_robot.urdf.xacro     # Main robot description
    ├── ros2_control.xacro           # Hardware interface definitions
    ├── gazebo.xacro                 # Gazebo material/friction/plugin config
    ├── materials.xacro              # RViz color definitions
    ├── cylinder_climb.world         # Gazebo world with a 5m climbing pole
    └── empty.world                  # Bare world (legacy)
```

---

## URDF Model (`urdf/climber_robot.urdf.xacro`)

### Concept

The robot's `base_link` is at the center of the cylinder being climbed. The Z-axis of `base_link` is aligned with the cylinder's vertical axis. Four mecanum wheels are arranged at 45°, 135°, 225°, and 315° around the cylinder, each pressing its rolling surface against the pole.

### Key Properties (top of file)

| Property | Default | Description |
|---|---|---|
| `cylinder_diameter` | `0.15` (m) | Diameter of the pole being climbed. **This is the primary value to change when targeting a different pole size.** |
| `wheel_radius` | `0.06` (m) | Mecanum wheel radius |
| `wheel_width` | `0.05` (m) | Mecanum wheel width (along axle) |
| `wheel_mass` | `0.4` (kg) | Mass per wheel |
| `arm_length` | `0.08` (m) | Length of radial structural arm from wheel to outer frame |
| `arm_thickness` | `0.02` (m) | Cross-section of structural members |
| `base_mass` | `3.0` (kg) | Mass of the chassis/frame structure |

### Derived Properties (computed automatically)

- `cylinder_radius` = `cylinder_diameter / 2`
- `wheel_radial_distance` = `cylinder_radius + wheel_width / 2` — the wheel center is placed so its inner face touches the cylinder surface
- `frame_ring_radius` = `wheel_radial_distance + wheel_width/2 + arm_length` — outer frame ring

### Link/Joint Structure

```
base_link (center, on cylinder axis)
├── chassis (fixed) — inertial-only, no visual (to not occlude wheels)
├── {ne,nw,sw,se}_arm (fixed) — radial structural bars
├── {ne,nw,sw,se}_frame_node (fixed) — outer frame attachment points
├── {ne,nw,sw,se}_wheel (continuous) — mecanum wheels
└── arc_{ne_nw, nw_sw, sw_se, se_ne} (fixed) — outer ring segments
```

### Wheel Joint Convention

Each wheel joint is placed at `(wheel_radial_distance * cos(angle), wheel_radial_distance * sin(angle), 0)` with `rpy="0 0 angle"`. This means:
- **Child X-axis** points radially outward (away from cylinder center)
- **Child Y-axis** is tangential to the cylinder

The rotation axis is `<axis xyz="0 -1 0"/>` (negative child-Y). This means:
- **Positive joint velocity → robot climbs UP** (positive Z)
- **Negative joint velocity → robot climbs DOWN** (negative Z)

The wheel cylinder geometry inside the link uses `rpy="pi/2 0 0"` to align the visual cylinder with the Y-axis (the axle).

### Customization

- **Different pole size**: Change `cylinder_diameter`. Everything else adapts automatically.
- **Different wheel size**: Change `wheel_radius`, `wheel_width`, `wheel_mass`. Also update `climber_controllers.yaml` and `cylinder_climb_controller.py` parameters to match.
- **Frame geometry**: Adjust `arm_length` and `arm_thickness` to change how far the outer ring sits from the wheels.
- **Wheel count or angles**: The angles `angle_ne`, `angle_nw`, `angle_sw`, `angle_se` are defined as properties and the `climbing_wheel_assembly` macro instantiated 4 times. To change to 3 or 6 wheels, add/remove instantiations and adjust angles. The controller would also need updating.

---

## ros2_control Hardware Interface (`urdf/ros2_control.xacro`)

Defines the `ros2_control` `<hardware>` and `<joint>` tags.

### Modes

Controlled by xacro args `use_sim` and `sim_gz`:

| `use_sim` | `sim_gz` | Hardware Plugin |
|---|---|---|
| `true` | `true` | `gz_ros2_control/GazeboSimSystem` (Gazebo sim) |
| `false` | any | `mock_components/GenericSystem` (mock/real hardware) |

### Joint Interfaces

All four wheel joints (`ne_wheel_joint`, `nw_wheel_joint`, `sw_wheel_joint`, `se_wheel_joint`) expose:
- **Command**: `velocity` (range: -10 to 10 rad/s)
- **State**: `position`, `velocity`

### Customization

- **Real hardware**: Replace `mock_components/GenericSystem` with your actual hardware interface plugin (e.g., a CAN bus motor driver plugin). Update `<param>` tags as needed.
- **Velocity limits**: Change the `min`/`max` params under `<command_interface name="velocity">`.

---

## Gazebo Configuration (`urdf/gazebo.xacro`)

### Materials & Friction

Sets Gazebo-specific visual materials and surface friction for each wheel:
- `mu1=0.8`, `mu2=0.4` — primary and secondary friction coefficients
- `kp=1000000`, `kd=100` — contact stiffness/damping
- `minDepth=0.001` — minimum penetration depth

### ros2_control Plugin

Loads `gz_ros2_control::GazeboSimROS2ControlPlugin` pointing at `config/climber_controllers.yaml`.

### Customization

- **Friction tuning**: Adjust `mu1`/`mu2` to change how much grip the wheels have on the pole. Higher values = more grip.
- **Contact parameters**: `kp`/`kd` affect how "hard" or "soft" the contact feels.

---

## Controller Configuration (`config/climber_controllers.yaml`)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

velocity_controller:
  ros__parameters:
    joints: [ne_wheel_joint, nw_wheel_joint, sw_wheel_joint, se_wheel_joint]
```

Uses `JointGroupVelocityController` which accepts a `Float64MultiArray` on topic `/velocity_controller/commands`. The array order matches the `joints` list: `[ne, nw, sw, se]`.

### Customization

- **Update rate**: Change `update_rate` (Hz). Higher = smoother but more CPU.
- **Joint order**: If you rename or add wheels, update the `joints` list here AND in the controller node.

> **Note**: `config/mecanum_controllers.yaml` is a legacy file from an earlier flat-ground mecanum design. It references old joint names (`front_left_wheel_joint`, etc.) and the `mecanum_drive_controller` plugin. It is **not used** by the current robot.

---

## Kinematic Controller (`src/cylinder_climb_controller.py`)

A ROS 2 Python node that converts `geometry_msgs/Twist` on `/cmd_vel` into individual wheel velocity commands.

### Input

- **Topic**: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- **Used fields**:
  - `linear.z` — climb velocity in m/s (positive = up)
  - `angular.z` — orbit angular velocity in rad/s (positive = CCW when viewed from above)

All other Twist fields are ignored.

### Output

- **Topic**: `/velocity_controller/commands` (`std_msgs/msg/Float64MultiArray`)
- **Data order**: `[ne, nw, sw, se]` wheel angular velocities in rad/s

### Kinematics

```
v_orbit = cylinder_radius * angular.z

ne = (vz + v_orbit) / wheel_radius
nw = (vz - v_orbit) / wheel_radius
sw = (vz + v_orbit) / wheel_radius
se = (vz - v_orbit) / wheel_radius
```

| Motion | NE | NW | SW | SE |
|---|---|---|---|---|
| Climb (linear.z > 0) | + | + | + | + |
| Orbit (angular.z > 0) | + | − | + | − |

This follows the mecanum strafing principle: NE/SW form one diagonal pair, NW/SE form the other. When diagonal pairs spin in opposite directions, the net force is tangential (orbiting), not vertical.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `wheel_radius` | `0.06` | Must match URDF `wheel_radius` |
| `cylinder_radius` | `0.075` | Must match URDF `cylinder_diameter / 2` |
| `max_wheel_vel` | `10.0` | Clamp value for wheel angular velocity (rad/s) |
| `cmd_vel_timeout` | `0.5` | Seconds of no `/cmd_vel` before wheels are stopped (safety) |

### Safety

A 20 Hz timer checks if `/cmd_vel` has been received within `cmd_vel_timeout` seconds. If not, all wheels are commanded to zero.

### Customization

- **Invert orbit direction**: Swap the `+v_orbit` / `-v_orbit` signs if the orbit direction is wrong for your mecanum roller orientation.
- **Add more DOFs**: The Twist message has 6 fields. You could map `linear.x`/`linear.y` to lateral/forward motion if your application requires it.
- **Odometry**: Currently there is no odometry publisher. To add one, subscribe to `/joint_states`, integrate wheel velocities, and publish `nav_msgs/Odometry` + a TF `odom → base_link`.

---

## Launch Files

### `launch/display.launch.py` — RViz Visualization

Loads the URDF (with `use_sim:=false`, `sim_gz:=false` → mock hardware) and opens RViz with `joint_state_publisher_gui` for manually dragging wheel joints.

```bash
ros2 launch climber_base display.launch.py
```

**Arguments:**
| Arg | Default | Description |
|---|---|---|
| `use_gui` | `true` | Launch joint_state_publisher_gui |

### `launch/gazebo_sim.launch.py` — Gazebo Simulation

Full simulation stack: Gazebo Sim with the `cylinder_climb.world`, ros2_control, velocity controller, and the climb controller node.

```bash
ros2 launch climber_base gazebo_sim.launch.py
```

**Arguments:**
| Arg | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use Gazebo clock |
| `use_rviz` | `true` | Also launch RViz |
| `world` | `empty.sdf` | Gazebo world (overridden internally to `cylinder_climb.world`) |

**Startup sequence:**
1. Robot State Publisher (publishes URDF to `/robot_description`)
2. Gazebo Sim (loads `cylinder_climb.world`)
3. Spawn robot at (0, 0, 0.3)
4. ROS-Gazebo bridge (clock sync)
5. Joint State Broadcaster spawner
6. After JSB exits → Velocity Controller spawner + Climb Controller node
7. RViz (if enabled)

**Commanding the robot in simulation:**
```bash
# Climb up
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.5}}" -r 10

# Climb down
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: -0.5}}" -r 10

# Orbit around cylinder
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}" -r 10

# Combined climb + orbit
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {z: 0.3}, angular: {z: 0.5}}" -r 10
```

---

## Gazebo World (`urdf/cylinder_climb.world`)

An SDF world containing:
- Ground plane with moderate friction
- Sun directional light
- **Climbing pole**: A static cylinder at (0, 0, 2.5), radius 0.075m, length 5m, brown-colored

The pole radius (0.075m) matches `cylinder_radius` in the URDF (i.e., `cylinder_diameter=0.15`).

### Customization
- **Pole size**: Change the `<radius>` and `<length>` in the `climbing_pole` model. Keep the radius in sync with the URDF's `cylinder_diameter / 2`.
- **Pole friction**: Adjust `<mu>` / `<mu2>` in the pole's `<surface>` element.
- **Pole position**: Change the `<pose>` if you want the pole elsewhere.

---

## Build & Dependencies

### System Requirements
- ROS 2 Jazzy
- Gazebo Sim (Harmonic recommended)

### Key Dependencies

| Package | Purpose |
|---|---|
| `ros2_control` | Hardware abstraction framework |
| `velocity_controllers` | `JointGroupVelocityController` for wheel commands |
| `joint_state_broadcaster` | Publishes joint states |
| `gz_ros2_control` | Bridges Gazebo ↔ ros2_control |
| `ros_gz_sim` | Gazebo Sim launch integration |
| `ros_gz_bridge` | ROS ↔ Gazebo message bridging (clock) |
| `robot_state_publisher` | URDF → TF publisher |
| `xacro` | URDF macro processor |

### Build

```bash
cd ~/GIT/climber-ws
colcon build
source install/setup.bash
```

---

## Known Limitations & Next Steps

1. **No pole constraint in sim**: The robot spawns near the pole but is not mechanically attached. Gravity pulls it down and it falls off. To test kinematics properly, either:
   - Add a prismatic (Z) + revolute (yaw) joint chain between the world and the robot in the SDF world file, constraining it to the pole
   - Add spring/clamp joints in the URDF that press wheels against the pole

2. **No odometry**: The controller publishes wheel commands but does not compute/publish odometry (`nav_msgs/Odometry` or TF `odom → base_link`).

3. **Orbit direction may need inversion**: Depending on actual mecanum roller orientation, the `+v_orbit` / `-v_orbit` mapping on diagonal pairs may need swapping.

4. **Legacy files**: `config/mecanum_controllers.yaml` and `urdf/empty.world` are from an older flat-ground design and are not used. They can be safely removed.

5. **Real hardware**: The `ros2_control.xacro` has a placeholder `mock_components/GenericSystem` for non-sim mode. Replace this with your actual motor driver hardware interface plugin.
