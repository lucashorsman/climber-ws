#!/usr/bin/env python3
"""
Cylinder Climb Controller — Full Version (Option C)

Central decision-making node for a 4-mecanum-wheel cylinder-climbing robot.

Operation modes:
  SIM   — publishes to ros2_control Float64MultiArray topics (velocity_controller,
           position_controller). No MCU topics involved.
  REAL  — ros2_control hardware interface handles MCU comms. This node still
           publishes to the same controller topics. Additionally subscribes to
           /mcu_*/arm_state for ToF data and contact state monitoring.

Subscriptions:
  /cmd_vel           (Twist)        — climb & orbit velocity commands
  /grip_cmd          (Float64)      — manual actuator override (metres)
  /mcu_*/arm_state   (ArmState)     — [REAL only] per-arm sensor feedback

Publications:
  /velocity_controller/commands  (Float64MultiArray) — wheel spin [ne,nw,sw,se]
  /position_controller/commands  (Float64MultiArray) — actuator pos [ne,nw,sw,se]
  /climber_state                 (ClimberState)       — aggregated robot state

State Machine:
  IDLE → APPROACHING → GRIPPING → CLIMBING ←→ ORBITING
                                      ↓
                                   STOPPED → IDLE
                            (any) → FAULT

Wheel layout (looking down at the cylinder from above):
  NW (135°)   NE (45°)
       \\       /
        [cyl]
       /       \\
  SW (225°)   SE (315°)

Mecanum kinematic mapping:
  Climb (vz):  all wheels same sign → vertical motion
  Orbit (wz):  NE/SW vs NW/SE opposite → circumferential motion

Actuator convention (prismatic, radial inward):
  position  0.0  → wheel surface just touching cylinder
  position <0    → pressing into cylinder (grip)
  position >0    → pulling away (release)
  limits: -0.01 m … +0.04 m
"""

import enum
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

# climber_msgs may not be available in sim-only builds — guard the import
try:
    from climber_msgs.msg import ArmState, ClimberState
    HAS_CLIMBER_MSGS = True
except ImportError:
    HAS_CLIMBER_MSGS = False


# ── Arm indices (must match ros2_control joint order) ───────────────
NE, NW, SW, SE = 0, 1, 2, 3
ARM_NAMES = ['ne', 'nw', 'sw', 'se']


class RobotState(enum.IntEnum):
    IDLE = 0
    APPROACHING = 1
    GRIPPING = 2
    CLIMBING = 3
    ORBITING = 4
    STOPPED = 5
    FAULT = 6


class ContactState(enum.IntEnum):
    UNKNOWN = 0
    GRIPPING = 1
    RELEASED = 2
    FAULT = 3


class CylinderClimbController(Node):
    def __init__(self):
        super().__init__('cylinder_climb_controller')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('cylinder_radius', 0.075)
        self.declare_parameter('max_wheel_vel', 10.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('default_grip_position', -0.005)
        self.declare_parameter('grip_distance_target', 0.003)   # 3mm ToF target
        self.declare_parameter('grip_tolerance', 0.002)         # ±2mm acceptable
        self.declare_parameter('grip_press_position', -0.005)   # actuator pos for grip
        self.declare_parameter('release_position', 0.03)        # actuator pos for release
        self.declare_parameter('approach_speed', 0.05)          # m/s approach crawl
        self.declare_parameter('max_climb_speed', 0.5)          # m/s limit during climb
        self.declare_parameter('use_real_hardware', False)       # enable MCU topics
        self.declare_parameter('comms_timeout', 0.5)            # fault if MCU silent

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.cylinder_radius = self.get_parameter('cylinder_radius').value
        self.max_wheel_vel = self.get_parameter('max_wheel_vel').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.default_grip_pos = self.get_parameter('default_grip_position').value
        self.grip_distance_target = self.get_parameter('grip_distance_target').value
        self.grip_tolerance = self.get_parameter('grip_tolerance').value
        self.grip_press_pos = self.get_parameter('grip_press_position').value
        self.release_pos = self.get_parameter('release_position').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.max_climb_speed = self.get_parameter('max_climb_speed').value
        self.use_real_hw = self.get_parameter('use_real_hardware').value
        self.comms_timeout = self.get_parameter('comms_timeout').value

        # ── State ───────────────────────────────────────────────────
        self.robot_state = RobotState.IDLE
        self.last_cmd_time = self.get_clock().now()
        self.grip_positions = [self.default_grip_pos] * 4
        self.cmd_vel_active = False

        # Per-arm feedback (populated from MCU topics or defaults)
        self.arm_contact = [ContactState.UNKNOWN] * 4
        self.arm_tof = [[] for _ in range(4)]          # list of float arrays
        self.arm_actuator_pos = [0.0] * 4
        self.arm_wheel_vel = [0.0] * 4
        self.arm_last_rx = [self.get_clock().now()] * 4

        # ── Subscribers ─────────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.grip_sub = self.create_subscription(
            Float64, '/grip_cmd', self.grip_callback, 10)

        # MCU state subscribers (real hardware only)
        self.arm_state_subs = []
        if self.use_real_hw and HAS_CLIMBER_MSGS:
            for i, name in enumerate(ARM_NAMES):
                sub = self.create_subscription(
                    ArmState, f'/mcu_{name}/arm_state',
                    lambda msg, idx=i: self.arm_state_callback(idx, msg), 10)
                self.arm_state_subs.append(sub)
            self.get_logger().info('Real hardware mode — subscribed to MCU topics')
        else:
            self.get_logger().info('Simulation mode — no MCU topic subscriptions')

        # ── Publishers ──────────────────────────────────────────────
        self.wheel_vel_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        self.actuator_pos_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)

        self.climber_state_pub = None
        if HAS_CLIMBER_MSGS:
            self.climber_state_pub = self.create_publisher(
                ClimberState, '/climber_state', 10)

        # ── Timer ───────────────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.periodic_update)  # 20 Hz

        self.get_logger().info(
            f'Cylinder climb controller started '
            f'(state={self.robot_state.name}, '
            f'wheel_r={self.wheel_radius}, cyl_r={self.cylinder_radius})')

    # ════════════════════════════════════════════════════════════════
    #  Callbacks
    # ════════════════════════════════════════════════════════════════

    def cmd_vel_callback(self, msg: Twist):
        """Process climb/orbit velocity commands."""
        self.last_cmd_time = self.get_clock().now()
        self.cmd_vel_active = True

        vz = msg.linear.z
        wz = msg.angular.z

        # Speed-limit during climb if grip is marginal
        if self.robot_state == RobotState.CLIMBING:
            vz = max(-self.max_climb_speed, min(self.max_climb_speed, vz))

        v_orbit = self.cylinder_radius * wz

        # Mecanum kinematics — joint order: [ne, nw, sw, se]
        wheels = [
            (vz + v_orbit) / self.wheel_radius,   # NE
            (vz - v_orbit) / self.wheel_radius,    # NW
            (vz + v_orbit) / self.wheel_radius,    # SW
            (vz - v_orbit) / self.wheel_radius,    # SE
        ]

        # Clamp
        max_abs = max(abs(w) for w in wheels) if any(wheels) else 0.0
        if max_abs > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_abs
            wheels = [w * scale for w in wheels]

        cmd = Float64MultiArray()
        cmd.data = wheels
        self.wheel_vel_pub.publish(cmd)

        # State transitions based on activity
        if self.robot_state == RobotState.GRIPPING and (abs(vz) > 0.01 or abs(wz) > 0.01):
            if abs(wz) > 0.01 and abs(vz) < 0.01:
                self._transition(RobotState.ORBITING)
            else:
                self._transition(RobotState.CLIMBING)
        elif self.robot_state == RobotState.STOPPED and (abs(vz) > 0.01 or abs(wz) > 0.01):
            self._transition(RobotState.CLIMBING)

    def grip_callback(self, msg: Float64):
        """Manual grip override — set all actuators to the same position."""
        pos = max(-0.01, min(0.04, msg.data))
        self.grip_positions = [pos] * 4
        self._publish_actuators()

        # State transitions for grip
        if pos < 0:
            if self.robot_state in (RobotState.IDLE, RobotState.APPROACHING):
                self._transition(RobotState.GRIPPING)
        elif pos > 0.02:
            if self.robot_state in (RobotState.GRIPPING, RobotState.STOPPED):
                self._transition(RobotState.IDLE)

    def arm_state_callback(self, idx: int, msg):
        """Receive state feedback from one MCU (real hardware only)."""
        self.arm_contact[idx] = ContactState(msg.contact_state)
        self.arm_tof[idx] = list(msg.tof_distances)
        self.arm_actuator_pos[idx] = msg.actuator_position
        self.arm_wheel_vel[idx] = msg.wheel_velocity
        self.arm_last_rx[idx] = self.get_clock().now()

    # ════════════════════════════════════════════════════════════════
    #  Periodic update (20 Hz)
    # ════════════════════════════════════════════════════════════════

    def periodic_update(self):
        """Safety checks, state machine updates, actuator re-publish."""

        now = self.get_clock().now()

        # ── Safety: stop wheels on cmd_vel timeout ──────────────────
        dt_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if dt_cmd > self.cmd_vel_timeout:
            if self.cmd_vel_active:
                self.cmd_vel_active = False
                cmd = Float64MultiArray()
                cmd.data = [0.0] * 4
                self.wheel_vel_pub.publish(cmd)
                if self.robot_state in (RobotState.CLIMBING, RobotState.ORBITING):
                    self._transition(RobotState.STOPPED)

        # ── Safety: MCU comms timeout (real hardware only) ─────────
        if self.use_real_hw:
            for i in range(4):
                dt = (now - self.arm_last_rx[i]).nanoseconds / 1e9
                if dt > self.comms_timeout:
                    self.get_logger().warn(
                        f'MCU {ARM_NAMES[i]} comms timeout ({dt:.1f}s)',
                        throttle_duration_sec=2.0)
                    if self.robot_state != RobotState.FAULT:
                        self._transition(RobotState.FAULT)

            # ── Safety: any MCU reporting FAULT contact state ──────
            for i in range(4):
                if self.arm_contact[i] == ContactState.FAULT:
                    if self.robot_state != RobotState.FAULT:
                        self.get_logger().error(
                            f'MCU {ARM_NAMES[i]} reports FAULT contact state')
                        self._transition(RobotState.FAULT)

        # ── Grip manager: auto-adjust per-arm grip if real HW ─────
        if self.use_real_hw and self.robot_state in (
            RobotState.GRIPPING, RobotState.CLIMBING,
            RobotState.ORBITING, RobotState.STOPPED
        ):
            self._auto_grip_adjust()

        # ── Always re-publish actuator positions ───────────────────
        self._publish_actuators()

        # ── Publish aggregated state ───────────────────────────────
        self._publish_climber_state()

    # ════════════════════════════════════════════════════════════════
    #  Grip Manager
    # ════════════════════════════════════════════════════════════════

    def _auto_grip_adjust(self):
        """
        Adjust each arm's actuator setpoint based on its ToF reading.
        Goal: maintain grip_distance_target ± grip_tolerance for each arm.
        """
        for i in range(4):
            if not self.arm_tof[i]:
                continue  # no ToF data yet for this arm

            # Use the minimum ToF reading as the surface distance
            min_dist = min(self.arm_tof[i])

            error = min_dist - self.grip_distance_target
            if abs(error) > self.grip_tolerance:
                # Proportional adjustment: positive error = too far → move in
                # Clamp the step to avoid overshoot
                step = max(-0.002, min(0.002, error * 0.5))
                new_pos = self.grip_positions[i] - step  # minus because inward is negative
                self.grip_positions[i] = max(-0.01, min(0.04, new_pos))

    # ════════════════════════════════════════════════════════════════
    #  State Machine
    # ════════════════════════════════════════════════════════════════

    def _transition(self, new_state: RobotState):
        """Transition the robot state machine with logging."""
        if new_state == self.robot_state:
            return
        old = self.robot_state.name
        self.robot_state = new_state
        self.get_logger().info(f'State: {old} → {new_state.name}')

        # On entering FAULT, command emergency grip
        if new_state == RobotState.FAULT:
            self.grip_positions = [self.grip_press_pos] * 4
            self._publish_actuators()
            # Stop wheels
            cmd = Float64MultiArray()
            cmd.data = [0.0] * 4
            self.wheel_vel_pub.publish(cmd)

    # ════════════════════════════════════════════════════════════════
    #  Helpers
    # ════════════════════════════════════════════════════════════════

    def _publish_actuators(self):
        """Publish the current grip positions to all four actuators."""
        cmd = Float64MultiArray()
        cmd.data = list(self.grip_positions)
        self.actuator_pos_pub.publish(cmd)

    def _publish_climber_state(self):
        """Publish aggregated ClimberState message."""
        if self.climber_state_pub is None:
            return

        msg = ClimberState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_state = self.robot_state.value

        for i in range(4):
            arm = ArmState()
            arm.header.stamp = msg.header.stamp
            arm.tof_distances = self.arm_tof[i] if self.arm_tof[i] else []
            arm.actuator_position = float(self.arm_actuator_pos[i])
            arm.actuator_velocity = 0.0
            arm.wheel_position = 0.0
            arm.wheel_velocity = float(self.arm_wheel_vel[i])
            arm.contact_state = self.arm_contact[i].value
            msg.arms[i] = arm

        self.climber_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CylinderClimbController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
