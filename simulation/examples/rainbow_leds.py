#!/usr/bin/env python3
"""rainbow_leds — drive /bpc_prp_robot/rgb_leds with a rotating rainbow.

Demo for the 4 directional RGB LEDs (front, right, back, left). Each LED
gets a hue 90° apart, and all hues rotate over time, so the colour sweeps
around the robot. Uses only the /bpc_prp_robot/* contract, so the same
node would drive the LEDs on the real robot.

Publishes the 12-value contract topic (std_msgs/UInt8MultiArray):
    [front_r, front_g, front_b,
     right_r, right_g, right_b,
     back_r,  back_g,  back_b,
     left_r,  left_g,  left_b]   each 0–255

Run it against any sim world:
    ros2 launch fenrir_sim line.launch.py        # terminal 1 (opens RViz)
    python3 simulation/examples/rainbow_leds.py   # terminal 2

Then watch the 4 LED markers cycle colour in RViz (MarkerArray display on
/sim/rgb_leds_markers). Parameters: rate_hz, period_s (seconds per full
rotation), brightness (0–1).
"""

import colorsys

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


NUM_LEDS = 4  # front, right, back, left


class RainbowLeds(Node):
    """Publish a rotating rainbow on the 4 RGB LEDs."""

    def __init__(self) -> None:
        super().__init__("rainbow_leds")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("period_s", 4.0)     # one full hue rotation
        self.declare_parameter("brightness", 1.0)   # 0..1

        rate = float(self.get_parameter("rate_hz").value)
        self.period = float(self.get_parameter("period_s").value)
        self.value = max(0.0, min(1.0, float(self.get_parameter("brightness").value)))

        self.pub = self.create_publisher(
            UInt8MultiArray, "/bpc_prp_robot/rgb_leds", 10)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            "rainbow_leds: rotating rainbow on 4 LEDs (front/right/back/left), "
            "%.1f s/turn, brightness %.2f" % (self.period, self.value)
        )

    def _tick(self) -> None:
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        base = (t / self.period) % 1.0 if self.period > 0 else 0.0
        data = []
        for i in range(NUM_LEDS):
            hue = (base + i / NUM_LEDS) % 1.0
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, self.value)
            data += [int(round(r * 255)), int(round(g * 255)), int(round(b * 255))]
        msg = UInt8MultiArray()
        msg.data = data
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RainbowLeds()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
