"""Tiny PID line-following demo.

NOT part of the simulation package itself — this is the "what students
would write" sample, used to prove that the /bpc_prp_robot/* contract
the sim exposes is good enough to close a real line-following control
loop. Subscribes only to /bpc_prp_robot/line_sensors and publishes only
to /bpc_prp_robot/set_motor_speeds. No sim-specific assumptions.

Algorithm:
    err   = left_sensor - right_sensor      # > 0 when line is to the right
    u     = kp*err + kd*(err - prev_err)
    right_motor = base_speed - u
    left_motor  = base_speed + u

Run alongside the sim:
    ros2 launch fenrir_sim line.launch.py headless:=true   # one terminal
    python3 simulation/examples/line_follower.py            # another

Or inside the same launch — the example is intentionally standalone
so it can be swapped out for the bpc-prp-devel solution package once
that integration test happens.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, UInt16MultiArray


class LineFollower(Node):
    """Closed-loop differential PID over the two line sensors."""

    def __init__(self) -> None:
        super().__init__("line_follower_demo")

        # Tunables (all bytes / dimensionless).
        self.declare_parameter("base_byte_above_stop", 22)   # ~0.22 m/s
        self.declare_parameter("kp",                    0.10)
        self.declare_parameter("kd",                    0.05)
        self.declare_parameter("publish_hz",            30.0)
        self.declare_parameter("min_byte",              50)
        self.declare_parameter("max_byte",              205)

        self.base = int(self.get_parameter("base_byte_above_stop").value)
        self.kp   = float(self.get_parameter("kp").value)
        self.kd   = float(self.get_parameter("kd").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.min_byte = int(self.get_parameter("min_byte").value)
        self.max_byte = int(self.get_parameter("max_byte").value)

        self.left  = 0
        self.right = 0
        self.prev_err = 0
        self.have_reading = False

        self.create_subscription(
            UInt16MultiArray,
            "/bpc_prp_robot/line_sensors",
            self._on_line,
            10,
        )
        self.pub = self.create_publisher(
            UInt8MultiArray,
            "/bpc_prp_robot/set_motor_speeds",
            10,
        )
        self.create_timer(1.0 / self.publish_hz, self._tick)

        self.get_logger().info(
            "line_follower_demo: base=%d kp=%.3f kd=%.3f @ %.0f Hz"
            % (self.base, self.kp, self.kd, self.publish_hz)
        )

    def _on_line(self, msg: UInt16MultiArray) -> None:
        if len(msg.data) < 2:
            return
        self.left  = int(msg.data[0])
        self.right = int(msg.data[1])
        self.have_reading = True

    def _tick(self) -> None:
        if not self.have_reading:
            return

        err = self.left - self.right
        d_err = err - self.prev_err
        self.prev_err = err
        u = int(self.kp * err + self.kd * d_err)

        right_byte = 127 + self.base - u
        left_byte  = 127 + self.base + u
        right_byte = max(self.min_byte, min(self.max_byte, right_byte))
        left_byte  = max(self.min_byte, min(self.max_byte, left_byte))

        msg = UInt8MultiArray()
        msg.data = [right_byte, left_byte]
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Safety: send a stop command before we let go.
        try:
            stop = UInt8MultiArray()
            stop.data = [127, 127]
            node.pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
