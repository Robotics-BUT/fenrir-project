"""line_sensor_bridge — /internal/floor_camera  →  /bpc_prp_robot/line_sensors.

The real Fenrir has 2 analog IR reflectance sensors pointing down. There
is no native Gazebo "line sensor" sensor type, so we approximate it with
a small downward-facing camera (declared in the URDF as `floor_camera`)
and sample its image at two horizontal positions corresponding to the
left and right physical sensor mounting points.

Output contract — matches MODERNIZATION_ROADMAP.md Appendix B:
    /bpc_prp_robot/line_sensors   std_msgs/UInt16MultiArray, length 2
                                  data[0] = left sensor,  0 .. 1023
                                  data[1] = right sensor, 0 .. 1023
    Polarity (Adam, 2026-05-20): white floor → LOW reading (≈0),
                                  black line  → HIGH reading (≈1023).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray


class LineSensorBridge(Node):
    """Convert downward-camera image samples into /bpc_prp_robot/line_sensors."""

    def __init__(self) -> None:
        super().__init__("line_sensor_bridge")

        # Sample column positions inside the camera image (fraction of width).
        # 0.25 / 0.75 puts the two virtual sensors lateral-symmetric around
        # the robot's centerline, mimicking the real left+right pair.
        self.declare_parameter("left_col_frac",  0.25)
        self.declare_parameter("right_col_frac", 0.75)
        self.declare_parameter("row_frac",       0.5)
        self.declare_parameter("sample_radius",  2)        # pixels each side
        self.declare_parameter("max_reading",    1023)     # match real ADC

        self.left_frac  = float(self.get_parameter("left_col_frac").value)
        self.right_frac = float(self.get_parameter("right_col_frac").value)
        self.row_frac   = float(self.get_parameter("row_frac").value)
        self.radius     = int(self.get_parameter("sample_radius").value)
        self.max_reading = int(self.get_parameter("max_reading").value)

        self.sub = self.create_subscription(
            Image, "/internal/floor_camera", self._on_image, 10)
        self.pub = self.create_publisher(
            UInt16MultiArray, "/bpc_prp_robot/line_sensors", 10)

        self.get_logger().info(
            "line_sensor_bridge ready: left=%.2f right=%.2f row=%.2f r=%d max=%d"
            % (self.left_frac, self.right_frac, self.row_frac,
               self.radius, self.max_reading)
        )

    def _on_image(self, img: Image) -> None:
        # We expect rgb8 / R8G8B8 (URDF declares R8G8B8). Each pixel = 3 bytes.
        if img.encoding not in ("rgb8", "R8G8B8"):
            self.get_logger().warning_once(
                f"unexpected floor-camera encoding '{img.encoding}'; "
                "expected rgb8 / R8G8B8"
            )

        h, w, step = img.height, img.width, img.step
        if h == 0 or w == 0:
            return

        cx_left  = max(0, min(w - 1, int(self.left_frac  * w)))
        cx_right = max(0, min(w - 1, int(self.right_frac * w)))
        cy       = max(0, min(h - 1, int(self.row_frac   * h)))

        left  = self._sample_brightness(img.data, step, cx_left,  cy)
        right = self._sample_brightness(img.data, step, cx_right, cy)

        msg = UInt16MultiArray()
        msg.data = [
            self._brightness_to_reading(left),
            self._brightness_to_reading(right),
        ]
        self.pub.publish(msg)

    def _sample_brightness(self, buf, step: int, cx: int, cy: int) -> float:
        """Mean brightness in a (2*radius+1)² window around (cx, cy)."""
        total = 0
        n = 0
        for dy in range(-self.radius, self.radius + 1):
            for dx in range(-self.radius, self.radius + 1):
                y, x = cy + dy, cx + dx
                if x < 0 or y < 0:
                    continue
                base = y * step + x * 3
                if base + 2 >= len(buf):
                    continue
                r, g, b = buf[base], buf[base + 1], buf[base + 2]
                total += (r + g + b) / 3.0
                n += 1
        return total / n if n else 0.0

    def _brightness_to_reading(self, brightness: float) -> int:
        # Linear inverted: pixel brightness (0..255) → ADC reading
        # (max_reading..0). White floor → low reading, black line → high
        # reading. Matches Fenrir hardware (Adam 2026-05-20).
        v = int(round((1.0 - brightness / 255.0) * self.max_reading))
        return max(0, min(self.max_reading, v))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
