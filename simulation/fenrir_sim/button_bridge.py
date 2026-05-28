"""button_bridge — keyboard → /bpc_prp_robot/buttons.

Real Fenrir buttons (`software/raspberry_pi/.../buttons_handler.py`):
* Three GPIO buttons on pins 5 / 6 / 13 → ids 0 / 1 / 2.
* The handler polls GPIO at 10 Hz and publishes a single UInt8 **only
  on the falling edge** (transition from not-pressed to pressed).
* In the student `solution/` package, buttons 0 / 1 / 2 enable the
  Line / Corridor / Maze loops respectively.

This node emulates that contract from a keyboard:

    key '1'  →  publish UInt8(0)   (Line loop)
    key '2'  →  publish UInt8(1)   (Corridor loop)
    key '3'  →  publish UInt8(2)   (Maze loop)
    key 'q'  →  quit
    Ctrl+C   →  quit

Each keypress publishes exactly one message — edge-triggered, like
the real robot.

**Interactive-only — run from a TTY, not from `ros2 launch`.** Launch
files detach stdin, so the node refuses to start when stdin is not a
TTY. Typical usage:

    # terminal 1: start the sim
    ros2 launch fenrir_sim line.launch.py

    # terminal 2: start the button bridge interactively
    ros2 run fenrir_sim button_bridge

For scripted / headless use (e.g. CI tests), publish directly to the
contract topic instead:

    ros2 topic pub --once /bpc_prp_robot/buttons std_msgs/msg/UInt8 \\
        "{data: 0}"
"""

from __future__ import annotations

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8


KEY_TO_BUTTON = {
    "1": 0,  # Line loop
    "2": 1,  # Corridor loop
    "3": 2,  # Maze loop
}

QUIT_KEYS = {"q", "\x03"}  # 'q' or Ctrl+C


class ButtonBridge(Node):
    """Keyboard-driven publisher for /bpc_prp_robot/buttons."""

    def __init__(self) -> None:
        super().__init__("button_bridge")
        # queue depth 1 matches the real buttons_handler
        self.pub = self.create_publisher(UInt8, "/bpc_prp_robot/buttons", 1)
        # 20 Hz poll — fast enough that keypress latency is sub-frame.
        self.timer = self.create_timer(0.05, self._drain_stdin)

        self.get_logger().info(
            "button_bridge ready: press 1/2/3 to publish button 0/1/2 "
            "(Line / Corridor / Maze). 'q' or Ctrl+C to quit."
        )

    def _drain_stdin(self) -> None:
        """Consume all pending keypresses since the last tick."""
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if not ready:
                return
            ch = sys.stdin.read(1)
            if not ch:
                return
            if ch in QUIT_KEYS:
                self.get_logger().info("quit requested")
                raise KeyboardInterrupt
            btn = KEY_TO_BUTTON.get(ch)
            if btn is None:
                continue  # silently ignore unrecognised keys
            msg = UInt8()
            msg.data = btn
            self.pub.publish(msg)
            self.get_logger().info(f"button {btn} pressed (key {ch!r})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ButtonBridge()

    if not sys.stdin.isatty():
        node.get_logger().error(
            "button_bridge needs a TTY. Run it from an interactive "
            "shell (`ros2 run fenrir_sim button_bridge`), not from "
            "`ros2 launch`. For scripted presses, publish directly: "
            "`ros2 topic pub --once /bpc_prp_robot/buttons "
            "std_msgs/msg/UInt8 \"{data: 0}\"`."
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        # cbreak (not raw): single-char reads, but Ctrl+C still raises
        # SIGINT so rclpy.spin returns cleanly via KeyboardInterrupt.
        tty.setcbreak(fd)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
