#!/usr/bin/env python3.10
"""
Actuator Keyboard Teleop
========================
Manually jog all four prismatic actuators (N, W, S, E) from the keyboard.

Controls
--------
  q / a  — North  actuator  extend / retract
  w / s  — West   actuator  extend / retract
  e / d  — South  actuator  extend / retract
  r / f  — East   actuator  extend / retract

  z       — ALL actuators → grip (press_pos)
  x       — ALL actuators → release (release_pos)
  c       — ALL actuators → zero (0.0)

  +/-     — increase / decrease step size
  ?       — print this help
  Ctrl-C  — quit

Actuator convention (matches cylinder_climb_controller):
  0.0  → wheel touching surface
  < 0  → pressing into cylinder (gripping)
  > 0  → pulling away (releasing)
  limits: [-0.01 m … +0.15 m]

Topic published: /position_controller/commands  (Float64MultiArray, order [N,W,S,E])
"""

import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ── Arm indices ──────────────────────────────────────────────────────
N, W, S, E = 0, 1, 2, 3
ARM_NAMES   = ['N (north)', 'W (west)', 'S (south)', 'E (east)']

# ── Limits (metres) ──────────────────────────────────────────────────
ACT_MIN      = -0.010   # max grip
ACT_MAX      =  0.150   # max release
GRIP_POS     = -0.005   # default gripping position
RELEASE_POS  =  0.030   # default release position
DEFAULT_STEP =  0.001   # 1 mm per key-press

KEYBINDINGS = """
  q / a  →  North  extend / retract
  w / s  →  West   extend / retract
  e / d  →  South  extend / retract
  r / f  →  East   extend / retract

  z      →  ALL grip   ({:.3f} m)
  x      →  ALL release ({:.3f} m)
  c      →  ALL zero   (0.000 m)

  +/-    →  increase / decrease step size
  ?      →  show this help
  Ctrl-C →  quit
"""


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def get_key(timeout: float = 0.05) -> str:
    """Return a single character from stdin, or '' on timeout."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class ActuatorKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('actuator_keyboard_teleop')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('step_size', DEFAULT_STEP)

        rate = self.get_parameter('publish_rate_hz').value
        self.step = self.get_parameter('step_size').value

        self.positions = [0.0, 0.0, 0.0, 0.0]   # [N, W, S, E]

        self.pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)

        self.timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('Actuator keyboard teleop started.')
        self._print_help()

    # ── Helpers ──────────────────────────────────────────────────────

    def _print_help(self):
        print(KEYBINDINGS.format(GRIP_POS, RELEASE_POS))
        self._print_status()

    def _print_status(self):
        bar_width = 20
        print('\033[2K\r', end='')   # clear line
        parts = []
        for i, name in enumerate(['N', 'W', 'S', 'E']):
            pos = self.positions[i]
            # Normalise to [0,1] for display (0=grip end, 1=release end)
            norm = (pos - ACT_MIN) / (ACT_MAX - ACT_MIN)
            filled = int(norm * bar_width)
            bar = '█' * filled + '░' * (bar_width - filled)
            parts.append(f'{name}:[{bar}] {pos:+.4f}m')
        print('  '.join(parts) + f'  step={self.step*1000:.1f}mm', end='\r', flush=True)

    def _set_all(self, pos: float):
        self.positions = [clamp(pos, ACT_MIN, ACT_MAX)] * 4

    def _jog(self, idx: int, direction: int):
        self.positions[idx] = clamp(
            self.positions[idx] + direction * self.step, ACT_MIN, ACT_MAX)

    def _publish(self):
        msg = Float64MultiArray()
        msg.data = list(self.positions)
        self.pub.publish(msg)

    # ── Main loop ────────────────────────────────────────────────────

    def run(self):
        KEY_MAP = {
            # (arm_index, direction)
            'q': (N, +1),  'a': (N, -1),
            'w': (W, +1),  's': (W, -1),
            'e': (S, +1),  'd': (S, -1),
            'r': (E, +1),  'f': (E, -1),
        }
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                key = get_key(timeout=0.05)

                if key in KEY_MAP:
                    idx, direction = KEY_MAP[key]
                    self._jog(idx, direction)
                    self._print_status()

                elif key == 'z':
                    self._set_all(GRIP_POS)
                    print(f'\n  → ALL grip ({GRIP_POS:.3f} m)')
                    self._print_status()

                elif key == 'x':
                    self._set_all(RELEASE_POS)
                    print(f'\n  → ALL release ({RELEASE_POS:.3f} m)')
                    self._print_status()

                elif key == 'c':
                    self._set_all(0.0)
                    print('\n  → ALL zero (0.000 m)')
                    self._print_status()

                elif key in ('+', '='):
                    self.step = min(self.step * 2, 0.010)
                    print(f'\n  step → {self.step*1000:.1f} mm')
                    self._print_status()

                elif key == '-':
                    self.step = max(self.step / 2, 0.0001)
                    print(f'\n  step → {self.step*1000:.2f} mm')
                    self._print_status()

                elif key == '?':
                    self._print_help()

                elif key == '\x03':   # Ctrl-C
                    break

        except KeyboardInterrupt:
            pass
        finally:
            # Zero actuators on exit
            print('\n\nExiting — zeroing all actuators.')
            self.positions = [0.0] * 4
            self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorKeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
