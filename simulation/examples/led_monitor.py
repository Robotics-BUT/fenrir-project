#!/usr/bin/env python3
"""led_monitor — show the 4 RGB LEDs in a plain terminal (no GUI needed).

Subscribes /bpc_prp_robot/rgb_leds and renders the four LEDs (front,
right, back, left) as 24-bit ANSI colour blocks, refreshing in place.
Use it to watch LED state over SSH on a machine with no display — the
headless alternative to the RViz markers.

    ros2 launch fenrir_sim line.launch.py headless:=true   # terminal 1
    python3 simulation/examples/rainbow_leds.py            # terminal 2
    python3 simulation/examples/led_monitor.py             # terminal 3

Needs a terminal that supports 24-bit colour (most do, incl. ssh).
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


NAMES = ["front", "right", "back", "left"]
NUM_LEDS = len(NAMES)


class LedMonitor(Node):
    """Render /bpc_prp_robot/rgb_leds as ANSI colour blocks."""

    def __init__(self) -> None:
        super().__init__("led_monitor")
        self.last = [(0, 0, 0)] * NUM_LEDS
        self.create_subscription(
            UInt8MultiArray, "/bpc_prp_robot/rgb_leds", self._on_leds, 10)
        self.create_timer(0.1, self._draw)  # 10 Hz refresh
        self.get_logger().info(
            "led_monitor: watching /bpc_prp_robot/rgb_leds "
            "(front/right/back/left). Ctrl+C to quit.")

    def _on_leds(self, msg: UInt8MultiArray) -> None:
        d = list(msg.data)
        for i in range(NUM_LEDS):
            if i * 3 + 2 < len(d):
                self.last[i] = (d[i * 3], d[i * 3 + 1], d[i * 3 + 2])

    def _draw(self) -> None:
        cells = []
        for i, (r, g, b) in enumerate(self.last):
            block = "\x1b[48;2;%d;%d;%dm      \x1b[0m" % (r, g, b)
            cells.append("%5s %s %3d,%3d,%3d" % (NAMES[i], block, r, g, b))
        sys.stdout.write("\r" + "   ".join(cells) + "  ")
        sys.stdout.flush()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LedMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
