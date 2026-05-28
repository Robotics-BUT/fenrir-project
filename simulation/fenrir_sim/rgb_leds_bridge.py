"""rgb_leds_bridge — /bpc_prp_robot/rgb_leds → RViz MarkerArray + log.

Closes the LED side of the /bpc_prp_robot/* contract in sim. The sim
robot carries 4 RGB LEDs on top of the chassis, one at each edge
midpoint, so each faces a compass direction:

    front (+x), right (-y), back (-x), left (+y)

The contract topic is a UInt8MultiArray of 12 values (4 LEDs × RGB,
each 0–255), ordered left-to-right per LED:

    data[0:3]   front  R,G,B
    data[3:6]   right  R,G,B
    data[6:9]   back   R,G,B
    data[9:12]  left   R,G,B

gz-sim 8.11 has no runtime material-colour system, so this node renders
the live colours as 4 RViz SPHERE markers on `/sim/rgb_leds_markers`,
placed in `base_link` at the same positions as the LED bodies in the
URDF. Add a MarkerArray display on that topic in RViz to watch them.

Real /bpc_prp_robot/rgb_leds
(software/raspberry_pi/.../rgb_leds_handler/rgb_leds_handler.py):
* `UInt8MultiArray`, layout `[R0,G0,B0, R1,G1,B1, R2,G2,B2, R3,G3,B3]`.
* Short arrays are accepted — only the LEDs covered by the array update;
  uncovered LEDs keep their last value.
* Hardware: 4-element NeoPixel strip on Pi GPIO 18. The front/right/back/
  left mapping above is the sim's physical placement of strip indices 0–3.
"""

from __future__ import annotations

from typing import List, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, UInt8MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from ros_gz_interfaces.msg import Entity, MaterialColor


# LED index -> compass direction. Order matches the contract data layout.
LED_NAMES = ["front", "right", "back", "left"]
NUM_LEDS = len(LED_NAMES)

# Marker positions in base_link — must match the led_*_link origins in
# description/fenrir.urdf.xacro (edge midpoints of the 15 cm chassis top,
# led_z = chassis_height + led_height/2 = 0.150 + 0.006 = 0.156).
LED_POSITIONS: List[Tuple[float, float, float]] = [
    (+0.060, 0.000, 0.156),  # front (+x)
    (0.000, -0.060, 0.156),  # right (-y)
    (-0.060, 0.000, 0.156),  # back  (-x)
    (0.000, +0.060, 0.156),  # left  (+y)
]
MARKER_SPHERE_DIAM = 0.022  # m — slightly larger than the 20 mm LED body


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
        # Per-LED colour for the Gazebo cubes: ros_gz_bridge forwards this to
        # gz.msgs.MaterialColor, which the LedMaterialColor system applies to
        # the led_* cube visuals on the robot model.
        self.color_pub = self.create_publisher(MaterialColor, "/led_colors", 10)

        # Track the latest committed RGB per LED. Short messages from the
        # robot only update the covered LEDs; uncovered slots keep their
        # last value (matches real rgb_leds_handler.py behaviour).
        self.last: List[Tuple[int, int, int]] = [(0, 0, 0)] * NUM_LEDS

        # Seed the all-off state so the markers and cubes start dark.
        self._publish_markers(self.last)
        self._publish_material_colors(self.last)

        self.get_logger().info(
            "rgb_leds_bridge ready: /bpc_prp_robot/rgb_leds -> %s "
            "(%d LEDs %s in frame %r)"
            % (marker_topic, NUM_LEDS, "/".join(LED_NAMES), self.frame_id)
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

        self.get_logger().info(
            "rgb_leds = %s"
            % ", ".join("%s=(%d,%d,%d)" % (LED_NAMES[i], *leds[i])
                        for i in range(NUM_LEDS))
        )
        self.last = leds
        self._publish_markers(leds)
        self._publish_material_colors(leds)

    def _publish_material_colors(self, leds: List[Tuple[int, int, int]]) -> None:
        for i, (r, g, b) in enumerate(leds):
            c = ColorRGBA(r=r / 255.0, g=g / 255.0, b=b / 255.0, a=1.0)
            m = MaterialColor()
            m.entity.name = "led_" + LED_NAMES[i]
            m.entity.type = Entity.VISUAL
            m.entity_match = MaterialColor.ALL
            m.ambient = c
            m.diffuse = c
            m.emissive = c
            self.color_pub.publish(m)

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
