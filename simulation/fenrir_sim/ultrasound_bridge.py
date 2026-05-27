"""ultrasound_bridge — 3× /internal/us_*  →  /bpc_prp_robot/ultrasounds.

Three Gazebo `gpu_lidar` sensors (each a ±30° fan of rays) model the
3 SRF-05-like ultrasounds on Fenrir. For each sensor this node takes
the **minimum** finite distance across the fan — mirroring real-US
echo behaviour: the sensor reports the closest reflector inside its
beam.

Output format matches Appendix B of the modernization roadmap:
    /bpc_prp_robot/ultrasounds : std_msgs/UInt8MultiArray, 5 Hz
        - 3 values, each a uint8 distance in cm (range 2 – 255)
        - 255 = "no echo" / max range / nothing detected
        - Order: parameter `channel_names`, default
          [us_left, us_front, us_right] — matches the encoder_bridge
          left-to-right convention. If the real firmware wires
          US_0/US_1/US_2 to a different physical layout, override the
          parameter from the launch file.

Pre-publish behaviour: until each channel has received at least one
scan, its slot is reported as 255 (out-of-range / no echo) — same as a
real ultrasound that has not yet returned a measurement.
"""

from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8MultiArray


# Real SRF-05-like spec (Adam, 2026-05-27): 2 cm minimum, 255 cm maximum.
US_MIN_CM = 2
US_MAX_CM = 255

DEFAULT_CHANNEL_NAMES = ["us_left", "us_front", "us_right"]


class UltrasoundBridge(Node):
    """Aggregate 3 narrow-cone gpu_lidar scans into /bpc_prp_robot/ultrasounds."""

    def __init__(self) -> None:
        super().__init__("ultrasound_bridge")

        self.declare_parameter("channel_names", DEFAULT_CHANNEL_NAMES)
        self.declare_parameter("publish_period", 0.2)  # 5 Hz per Appendix B

        names = list(self.get_parameter("channel_names").value)
        if len(names) != 3:
            raise ValueError(
                f"channel_names must have 3 elements, got {len(names)}: {names}"
            )
        self.channel_names: List[str] = names
        self.period: float = float(self.get_parameter("publish_period").value)

        # Latest min-of-fan per channel, in meters. None = no scan yet.
        self.latest: List[Optional[float]] = [None, None, None]

        # One subscription per channel; the `i=idx` default binds the index.
        self.subs = []
        for idx, name in enumerate(self.channel_names):
            topic = f"/internal/{name}"
            sub = self.create_subscription(
                LaserScan, topic,
                lambda msg, i=idx: self._on_scan(i, msg),
                10,
            )
            self.subs.append(sub)

        self.pub = self.create_publisher(
            UInt8MultiArray, "/bpc_prp_robot/ultrasounds", 10
        )
        self.timer = self.create_timer(self.period, self._publish)

        rate_hz = 1.0 / self.period if self.period > 0 else float("inf")
        self.get_logger().info(
            "ultrasound_bridge ready: %s -> /bpc_prp_robot/ultrasounds @ %.1f Hz"
            % (self.channel_names, rate_hz)
        )

    @staticmethod
    def _min_valid_range(msg: LaserScan) -> Optional[float]:
        """Return min finite range within [range_min, range_max], or None."""
        lo = msg.range_min
        hi = msg.range_max
        best: Optional[float] = None
        for r in msg.ranges:
            if not math.isfinite(r):
                continue
            if r < lo or r > hi:
                continue
            if best is None or r < best:
                best = r
        return best

    def _on_scan(self, idx: int, msg: LaserScan) -> None:
        self.latest[idx] = self._min_valid_range(msg)

    def _publish(self) -> None:
        out = UInt8MultiArray()
        out.data = [self._to_cm(d) for d in self.latest]
        self.pub.publish(out)

    @staticmethod
    def _to_cm(distance_m: Optional[float]) -> int:
        """Convert meters → clamped uint8 cm. None/no-echo → max-range cap."""
        if distance_m is None:
            return US_MAX_CM
        cm = int(round(distance_m * 100.0))
        if cm < US_MIN_CM:
            return US_MIN_CM
        if cm > US_MAX_CM:
            return US_MAX_CM
        return cm


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasoundBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
