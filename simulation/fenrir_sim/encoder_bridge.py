"""encoder_bridge — /joint_states  ↔  /bpc_prp_robot/encoders.

Translates Gazebo wheel-joint positions (radians) into the Fenrir
encoder contract: UInt32MultiArray[2] = [left_ticks, right_ticks] at
576 pulses per wheel revolution, wrapped to a uint32 counter.

Topic-ordering convention (Adam, 2026-05-20): every multi-element
/bpc_prp_robot/* topic is ordered LEFT-to-RIGHT. So:
    encoders : [left, right]

Real-firmware behaviour we mirror (per `software/arduino_nano/main/main.ino`
and `software/raspberry_pi/.../i2c_handler.py`):
    * 32-bit unsigned counter — wraps at 2**32
    * 576 pulses per full wheel rotation (MOTOR_PULSES_PER_ROT)
    * Published from the on_arduino_fast loop at 100 Hz
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt32MultiArray


TWO_PI = 2.0 * math.pi
UINT32_MOD = 1 << 32


class EncoderBridge(Node):
    """Bridge /joint_states wheel positions to /bpc_prp_robot/encoders."""

    def __init__(self) -> None:
        super().__init__("encoder_bridge")

        self.declare_parameter("left_joint_name",  "left_wheel_joint")
        self.declare_parameter("right_joint_name", "right_wheel_joint")
        self.declare_parameter("pulses_per_rev",   576)
        self.declare_parameter("publish_period",   0.01)  # 100 Hz

        self.left_name:  str = str(self.get_parameter("left_joint_name").value)
        self.right_name: str = str(self.get_parameter("right_joint_name").value)
        self.pulses_per_rev: int = int(self.get_parameter("pulses_per_rev").value)
        self.period: float = float(self.get_parameter("publish_period").value)

        self.left_pos:  float = 0.0
        self.right_pos: float = 0.0
        self.have_data: bool  = False

        self.sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10)
        self.pub = self.create_publisher(
            UInt32MultiArray, "/bpc_prp_robot/encoders", 10)
        self.timer = self.create_timer(self.period, self._publish)

        self.get_logger().info(
            "encoder_bridge ready: %d pulses/rev, %s + %s -> /bpc_prp_robot/encoders"
            % (self.pulses_per_rev, self.left_name, self.right_name)
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name == self.left_name:
                self.left_pos = float(pos)
                self.have_data = True
            elif name == self.right_name:
                self.right_pos = float(pos)
                self.have_data = True

    def _ticks(self, position_rad: float) -> int:
        # Real firmware exposes an unsigned 32-bit counter; Python's % always
        # returns a non-negative remainder for a positive modulus, so reverse
        # rotation underflow lands at 0xFFFFFFFF as it would on the real bot.
        return int(round(position_rad / TWO_PI * self.pulses_per_rev)) % UINT32_MOD

    def _publish(self) -> None:
        if not self.have_data:
            return
        msg = UInt32MultiArray()
        msg.data = [self._ticks(self.left_pos), self._ticks(self.right_pos)]
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
