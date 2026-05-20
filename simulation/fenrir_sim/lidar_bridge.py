"""lidar_bridge — /internal/lidar  →  /bpc_prp_robot/lidar.

The real Fenrir RPLiDAR is mounted **180° rotated about z** (its 0°
mark points BACKWARD relative to the robot) and the scan rotates
**clockwise** (viewed from above).  So the published scan obeys:

    ranges[0]              = distance behind the robot
    ranges[N/4]            = distance to the robot's LEFT
    ranges[N/2]            = distance ahead of the robot
    ranges[3N/4]           = distance to the robot's RIGHT
    angle_min              =  +π
    angle_max              ≈  −π                 (one step away)
    angle_increment        =  −2π / N            (negative → CW)

Gazebo's gpu_lidar produces the standard ROS REP-103 layout: CCW,
angle_min = −π, angle_max = +π, angle_increment = +2π/N. With the
lidar link aligned with the robot's base, gz already puts sample 0 at
angle = −π, which is the robot's backward direction.  So sample 0 is
already correctly placed; we only need to **reverse the remainder of
the array** to flip CCW → CW and rewrite the angle metadata so a
consumer using `angle_min + i * angle_increment` gets the right
direction for each sample.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarBridge(Node):
    """gz LaserScan (CCW from backward) → Fenrir LaserScan (CW from backward)."""

    def __init__(self) -> None:
        super().__init__("lidar_bridge")

        self.sub = self.create_subscription(
            LaserScan, "/internal/lidar", self._on_scan, 10
        )
        self.pub = self.create_publisher(
            LaserScan, "/bpc_prp_robot/lidar", 10
        )

        self.get_logger().info(
            "lidar_bridge ready: gz /internal/lidar -> /bpc_prp_robot/lidar "
            "(reverse + CW metadata)"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n == 0:
            return

        # Reverse ranges[1:] in place (preserves sample 0 = backward).
        # ranges_new[i] = ranges_old[(n - i) % n]
        ranges_new = [msg.ranges[0]] + list(reversed(msg.ranges[1:]))

        intens_new: list[float] = []
        if len(msg.intensities) == n:
            intens_new = [msg.intensities[0]] + list(reversed(msg.intensities[1:]))

        out = LaserScan()
        out.header = msg.header
        out.angle_min       = math.pi
        out.angle_increment = -2.0 * math.pi / n   # CW
        out.angle_max       = out.angle_min + (n - 1) * out.angle_increment
        out.time_increment  = msg.time_increment
        out.scan_time       = msg.scan_time
        out.range_min       = msg.range_min
        out.range_max       = msg.range_max
        out.ranges          = ranges_new
        out.intensities     = intens_new
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
