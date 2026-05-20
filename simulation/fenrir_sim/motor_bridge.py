"""motor_bridge — /bpc_prp_robot/set_motor_speeds  ↔  /cmd_vel.

Translates the Fenrir contract (UInt8MultiArray[2]: [left, right],
0..255 with 127 = stop) into geometry_msgs/Twist for the Gazebo
DiffDrive plugin, AND enforces the 1-second motor watchdog that the
real robot's I2C firmware implements.

Topic-ordering convention (Adam, 2026-05-20): every multi-element
/bpc_prp_robot/* topic is ordered LEFT-to-RIGHT. So:
    set_motor_speeds : [left,  right]
    line_sensors     : [left,  right]
    ultrasounds      : [left,  center, right]   (T4.3 follow-up)

Byte encoding (per MODERNIZATION_ROADMAP.md Appendix B):
    raw byte 127         -> stop  (0 m/s for that wheel)
    raw byte 255         -> +1.27 m/s (forward)
    raw byte   0         -> -1.27 m/s (reverse)
    everything in between is linear.

Differential-drive kinematics:
    v     = (v_right + v_left) / 2
    omega = (v_right - v_left) / wheel_separation
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


# Wheel-tangential speed corresponding to one raw byte unit, computed so
# that byte=255 → +1.27 m/s. Keep in sync with the real firmware.
M_PER_S_PER_BYTE = 1.27 / 128.0


class MotorBridge(Node):
    """Bridge the /bpc_prp_robot/set_motor_speeds topic to /cmd_vel."""

    def __init__(self) -> None:
        super().__init__("motor_bridge")

        self.declare_parameter("wheel_separation", 0.20)
        self.declare_parameter("watchdog_seconds", 1.0)
        self.declare_parameter("publish_period",   0.05)  # 20 Hz

        self.wheel_separation: float = float(
            self.get_parameter("wheel_separation").value)
        self.watchdog: float = float(
            self.get_parameter("watchdog_seconds").value)
        self.period: float = float(
            self.get_parameter("publish_period").value)

        self.v_right_mps: float = 0.0
        self.v_left_mps: float = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.sub = self.create_subscription(
            UInt8MultiArray,
            "/bpc_prp_robot/set_motor_speeds",
            self._on_set_motor_speeds,
            10,
        )
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(self.period, self._publish_cmd)

        self.get_logger().info(
            "motor_bridge ready: wheel_separation=%.3f m, watchdog=%.1f s"
            % (self.wheel_separation, self.watchdog)
        )

    def _on_set_motor_speeds(self, msg: UInt8MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warning(
                "set_motor_speeds expects [left, right]; got %d bytes"
                % len(msg.data)
            )
            return

        # Left-to-right convention: data[0] = left wheel, data[1] = right wheel.
        left, right = int(msg.data[0]), int(msg.data[1])
        self.v_left_mps  = (left  - 127) * M_PER_S_PER_BYTE
        self.v_right_mps = (right - 127) * M_PER_S_PER_BYTE
        self.last_cmd_time = self.get_clock().now()

    def _publish_cmd(self) -> None:
        # Watchdog: if no command for `watchdog` seconds, drive zero.
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.watchdog:
            v_right, v_left = 0.0, 0.0
        else:
            v_right, v_left = self.v_right_mps, self.v_left_mps

        twist = Twist()
        twist.linear.x  = (v_right + v_left) / 2.0
        twist.angular.z = (v_right - v_left) / self.wheel_separation
        self.pub.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
