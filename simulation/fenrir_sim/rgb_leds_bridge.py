"""rgb_leds_bridge — /bpc_prp_robot/rgb_leds → RViz MarkerArray + log.

Closes the LED side of the /bpc_prp_robot/* contract in sim. There is no
gz "RGB LED" actuator, so the bridge is a sink: it subscribes to the
contract topic, logs the new triplets on change, and republishes 4
SPHERE markers in `base_link` so the LED state is visible in RViz while
developing.

Real /bpc_prp_robot/rgb_leds
(software/raspberry_pi/.../rgb_leds_handler/rgb_leds_handler.py):
* `UInt8MultiArray`, data layout `[R0,G0,B0, R1,G1,B1, R2,G2,B2, R3,G3,B3]`
* Iteration: `data[i*3 : i*3+3]` per LED i. Short arrays are accepted —
  only the LEDs covered by the array are updated.
* Hardware: 4-element NeoPixel strip on Pi GPIO 18.

Sim positions the 4 markers in a row along the back edge of the chassis
top (just below the lidar height). The mapping LED-i → y position
mirrors the L-to-R convention used elsewhere in the codebase.
"""

from __future__ import annotations

from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from visualization_msgs.msg import Marker, MarkerArray


NUM_LEDS = 4

# 4 LEDs along the back edge of the 15 cm chassis top, z=0.155
# (chassis top is at z=0.150; lidar at 0.170). y from +0.060 (left) to
# −0.060 (right) — L→R matches the [0..3] LED indexing.
LED_POSITIONS: List[Tuple[float, float, float]] = [
    (-0.060, +0.060, 0.155),  # LED 0 — back-left
    (-0.060, +0.020, 0.155),  # LED 1
    (-0.060, -0.020, 0.155),  # LED 2
    (-0.060, -0.060, 0.155),  # LED 3 — back-right
]
MARKER_SPHERE_DIAM = 0.020  # m


class RgbLedsBridge(Node):
    """Subscribe /bpc_prp_robot/rgb_leds; emit RViz MarkerArray on change."""

    def __init__(self) -> None:
        super().__init__("rgb_leds_bridge")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("marker_topic", "/sim/rgb_leds_markers")
        self.frame_id: str = str(self.get_parameter("frame_id").value)
        marker_topic: str = str(self.get_parameter("marker_topic").value)

        self.sub = self.create_subscription(
            UInt8MultiArray, "/bpc_prp_robot/rgb_leds", self._on_leds, 1
        )
        self.pub = self.create_publisher(MarkerArray, marker_topic, 1)

        # Track the latest committed RGB per LED. Short messages from the
        # robot only update the covered LEDs; uncovered slots keep their
        # last value (matches real rgb_leds_handler.py behaviour).
        self.last: List[Tuple[int, int, int]] = [(0, 0, 0)] * NUM_LEDS

        # Seed RViz with the all-off state so 4 dark markers appear
        # immediately, before any student message arrives.
        self._publish_markers(self.last)

        self.get_logger().info(
            "rgb_leds_bridge ready: /bpc_prp_robot/rgb_leds -> %s "
            "(%d sphere markers in frame %r)"
            % (marker_topic, NUM_LEDS, self.frame_id)
        )

    def _on_leds(self, msg: UInt8MultiArray) -> None:
        data = list(msg.data)
        leds: List[Tuple[int, int, int]] = []
        for i in range(NUM_LEDS):
            r = data[i * 3]     if i * 3     < len(data) else self.last[i][0]
            g = data[i * 3 + 1] if i * 3 + 1 < len(data) else self.last[i][1]
            b = data[i * 3 + 2] if i * 3 + 2 < len(data) else self.last[i][2]
            leds.append((r, g, b))

        if leds == self.last:
            return  # no change, skip publish + log

        self.get_logger().info("rgb_leds = %s" % (leds,))
        self.last = leds
        self._publish_markers(leds)

    def _publish_markers(self, leds: List[Tuple[int, int, int]]) -> None:
        out = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (r, g, b) in enumerate(leds):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = "rgb_leds"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            x, y, z = LED_POSITIONS[i]
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = MARKER_SPHERE_DIAM
            m.scale.y = MARKER_SPHERE_DIAM
            m.scale.z = MARKER_SPHERE_DIAM
            m.color.r = r / 255.0
            m.color.g = g / 255.0
            m.color.b = b / 255.0
            m.color.a = 1.0
            out.markers.append(m)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RgbLedsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
