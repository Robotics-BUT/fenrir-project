"""Tiny PID line-following demo.

NOT part of the simulation package itself — this is the "what students
would write" sample, used to prove that the /bpc_prp_robot/* contract
the sim exposes is good enough to close a real line-following control
loop. Subscribes only to /bpc_prp_robot/line_sensors and publishes only
to /bpc_prp_robot/set_motor_speeds. No sim-specific assumptions.

Topic-ordering convention: multi-element topics are LEFT-to-RIGHT.
    line_sensors     = [left,  right]
    set_motor_speeds = [left,  right]

Algorithm (polarity: white floor → low, black line → HIGH):

    Two states.

    TRACKING — at least one sensor reads above `line_threshold`:
        err   = right_sensor - left_sensor      # > 0 when line is to the right
        u     = kp*err + kd*(err - prev_err)
        right_motor = base_speed - u            # slow right wheel → turn right
        left_motor  = base_speed + u

    LOST    — both sensors below `line_threshold` (white floor only):
        Rotate in place toward the last-known line direction (sign of last
        non-zero err), with a slow forward bias. The robot keeps moving and
        rotating until at least one sensor re-acquires the line.

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
        self.declare_parameter("base_byte_above_stop", 9)    # ~0.09 m/s forward
        self.declare_parameter("kp",                    0.075)
        self.declare_parameter("kd",                    0.0)
        self.declare_parameter("publish_hz",            30.0)
        self.declare_parameter("min_byte",              50)
        self.declare_parameter("max_byte",              205)
        # Search-mode tunables (LOST state).
        self.declare_parameter("line_threshold",        400)  # reading above this = sensor on line
        self.declare_parameter("search_base",            8)   # very slow crawl while searching
        self.declare_parameter("search_turn",           55)   # strong fixed turn (byte units)

        self.base        = int(self.get_parameter("base_byte_above_stop").value)
        self.kp          = float(self.get_parameter("kp").value)
        self.kd          = float(self.get_parameter("kd").value)
        self.publish_hz  = float(self.get_parameter("publish_hz").value)
        self.min_byte    = int(self.get_parameter("min_byte").value)
        self.max_byte    = int(self.get_parameter("max_byte").value)
        self.threshold   = int(self.get_parameter("line_threshold").value)
        self.search_base = int(self.get_parameter("search_base").value)
        self.search_turn = int(self.get_parameter("search_turn").value)

        self.left  = 0
        self.right = 0
        self.prev_err = 0
        self.last_err = 0
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

        left_on  = self.left  > self.threshold
        right_on = self.right > self.threshold

        if left_on or right_on:
            # TRACKING: at least one sensor over the line, run PID.
            # Polarity: black line → HIGH reading. If right_sensor > left_sensor,
            # the right side is more over the line → robot needs to steer right.
            err = self.right - self.left
            d_err = err - self.prev_err
            self.prev_err = err
            self.last_err = err
            u = int(self.kp * err + self.kd * d_err)
            base = self.base
        else:
            # LOST: both sensors over white. Rotate toward the last-known line
            # direction at a slow forward crawl until a sensor reacquires.
            sign = 1 if self.last_err >= 0 else -1
            u = sign * self.search_turn
            base = self.search_base

        right_byte = 127 + base - u
        left_byte  = 127 + base + u
        right_byte = max(self.min_byte, min(self.max_byte, right_byte))
        left_byte  = max(self.min_byte, min(self.max_byte, left_byte))

        # Convention: set_motor_speeds is [left, right].
        msg = UInt8MultiArray()
        msg.data = [left_byte, right_byte]
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
