#!/usr/bin/env python3
"""
Cylinder Climb Controller

Kinematic controller for a 4-mecanum-wheel cylinder-climbing robot.

Subscriptions:
  /cmd_vel  (Twist)          - climb & orbit commands
  /grip_cmd (Float64)        - actuator position command (metres)
                                0.0 = wheels touching cylinder (home)
                               <0   = pressing in (grip)
                               >0   = pulling away (release)

Publications:
  /velocity_controller/commands  (Float64MultiArray) - wheel spin [ne,nw,sw,se]
  /position_controller/commands  (Float64MultiArray) - actuators  [ne,nw,sw,se]

DOFs:
  - linear.z  → climb velocity (all wheels same direction)
  - angular.z → orbit velocity around the cylinder (diagonal pairs opposite)

Wheel layout (looking down at the cylinder from above):
  NW (135°)   NE (45°)
       \\       /
        [cyl]
       /       \\
  SW (225°)   SE (315°)

Mecanum roller convention:
  - NE and SW have one roller orientation (+45° to axle)
  - NW and SE have the other roller orientation (-45° to axle)

For climbing (vz): all wheels spin the same → robot moves along Z
For orbiting (wz): diagonal pairs spin opposite → robot orbits around cylinder

Actuator convention (prismatic, radial axis pointing inward):
  position  0.0  → wheel surface just touching cylinder
  position <0    → pressing wheel into cylinder (grip)
  position >0    → pulling wheel away from cylinder (release)
  limits: -0.01 m (max grip) to +0.04 m (fully retracted)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray


class CylinderClimbController(Node):
    def __init__(self):
        super().__init__('cylinder_climb_controller')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.06)
        self.declare_parameter('cylinder_radius', 0.075)
        self.declare_parameter('max_wheel_vel', 10.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('default_grip_position', -0.005)  # slight press-in

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.cylinder_radius = self.get_parameter('cylinder_radius').value
        self.max_wheel_vel = self.get_parameter('max_wheel_vel').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.default_grip_pos = self.get_parameter('default_grip_position').value

        # ── State ───────────────────────────────────────────────────
        self.last_cmd_time = self.get_clock().now()
        self.grip_position = self.default_grip_pos  # current actuator target

        # ── Subscribers ─────────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.grip_sub = self.create_subscription(
            Float64, '/grip_cmd', self.grip_callback, 10
        )

        # ── Publishers ──────────────────────────────────────────────
        self.wheel_vel_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10
        )
        self.actuator_pos_pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10
        )

        # ── Timer ───────────────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.periodic_update)  # 20 Hz

        self.get_logger().info(
            f'Cylinder climb controller started '
            f'(wheel_r={self.wheel_radius}, cyl_r={self.cylinder_radius}, '
            f'default_grip={self.default_grip_pos})'
        )

    # ── Callbacks ───────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        vz = msg.linear.z    # Climb velocity (m/s)
        wz = msg.angular.z   # Orbit angular velocity (rad/s)

        v_orbit = self.cylinder_radius * wz

        # Joint order: [ne, nw, sw, se]
        ne = (vz + v_orbit) / self.wheel_radius
        nw = (vz - v_orbit) / self.wheel_radius
        sw = (vz + v_orbit) / self.wheel_radius
        se = (vz - v_orbit) / self.wheel_radius

        # Clamp to max velocity
        wheels = [ne, nw, sw, se]
        max_abs = max(abs(w) for w in wheels) if any(wheels) else 0.0
        if max_abs > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_abs
            wheels = [w * scale for w in wheels]

        cmd = Float64MultiArray()
        cmd.data = wheels
        self.wheel_vel_pub.publish(cmd)

    def grip_callback(self, msg: Float64):
        """Set all actuators to the same radial position (metres)."""
        self.grip_position = max(-0.01, min(0.04, msg.data))
        self._publish_actuators()

    # ── Periodic ────────────────────────────────────────────────────

    def periodic_update(self):
        """Safety timeout for wheels + continuous actuator command."""
        # Stop wheels if no cmd_vel within timeout
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_vel_timeout:
            cmd = Float64MultiArray()
            cmd.data = [0.0, 0.0, 0.0, 0.0]
            self.wheel_vel_pub.publish(cmd)

        # Always re-publish actuator positions so controller stays active
        self._publish_actuators()

    # ── Helpers ─────────────────────────────────────────────────────

    def _publish_actuators(self):
        """Publish the current grip position to all four actuators."""
        cmd = Float64MultiArray()
        cmd.data = [self.grip_position] * 4
        self.actuator_pos_pub.publish(cmd)


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
