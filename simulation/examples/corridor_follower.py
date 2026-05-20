"""Tiny lidar-based corridor follower demo.

NOT part of the simulation package — this is the "what students would write"
sample for Lab 12 (corridor following midterm). Drives forward while keeping
the robot centered between the two corridor walls, using just a 2D lidar.

Lidar layout (Fenrir-specific — see lidar_bridge.py):
    ranges[0]            distance directly behind
    ranges[N/4]          distance to robot's LEFT
    ranges[N/2]          distance directly ahead
    ranges[3N/4]         distance to robot's RIGHT
    samples increment    clockwise viewed from above

State machine:

    1. forward < turn_trigger (wall ahead) → turn IN PLACE toward the
       more open side until forward clears.
    2. forward clear, both side walls close (within wall_limit) → drive
       forward, balance the two side distances (PID on right − left).
    3. forward clear, only ONE side wall close → drive forward, keep
       `side_target_dist` from that wall.
    4. forward clear, NO side walls close (open space) → drive straight.

The key trick (per Adam, 2026-05-20): **ignore any side reading farther
than `wall_limit` = 1 cell = 0.40 m**. Without this clamp the balancer
will steer toward a far-open corner trying to "balance" against a 2 m
distance, which gets the robot stuck.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8MultiArray


def _min_in_window(ranges, center: int, half: int, fallback: float) -> float:
    """Min of ranges[center-half .. center+half], filtering NaN / inf / 0."""
    n = len(ranges)
    if n == 0:
        return fallback
    best = math.inf
    for i in range(center - half, center + half + 1):
        v = ranges[i % n]
        if v is None:
            continue
        if math.isnan(v) or math.isinf(v) or v <= 0.0:
            continue
        if v < best:
            best = v
    return best if math.isfinite(best) else fallback


class CorridorFollower(Node):
    """Centers the robot between the two corridor walls."""

    def __init__(self) -> None:
        super().__init__("corridor_follower_demo")

        self.declare_parameter("base_byte_above_stop", 15)     # ~0.15 m/s forward
        self.declare_parameter("kp",                    60.0)   # bytes per m of L-R diff
        self.declare_parameter("window_half_samples",   3)     # ±3 samples averaged
        self.declare_parameter("wall_limit",            0.40)   # m — ignore walls beyond
        self.declare_parameter("side_target_dist",      0.18)   # m — desired dist when one wall
        self.declare_parameter("turn_trigger_dist",     0.35)   # m — forward < this → turn
        self.declare_parameter("turn_byte",             30)     # byte units, in-place turn
        self.declare_parameter("min_byte",              50)
        self.declare_parameter("max_byte",              205)

        self.base       = int(self.get_parameter("base_byte_above_stop").value)
        self.kp         = float(self.get_parameter("kp").value)
        self.half       = int(self.get_parameter("window_half_samples").value)
        self.wall_limit = float(self.get_parameter("wall_limit").value)
        self.side_target= float(self.get_parameter("side_target_dist").value)
        self.turn_trig  = float(self.get_parameter("turn_trigger_dist").value)
        self.turn_byte  = int(self.get_parameter("turn_byte").value)
        self.min_byte   = int(self.get_parameter("min_byte").value)
        self.max_byte   = int(self.get_parameter("max_byte").value)

        self.create_subscription(
            LaserScan, "/bpc_prp_robot/lidar", self._on_scan, 10
        )
        self.pub = self.create_publisher(
            UInt8MultiArray, "/bpc_prp_robot/set_motor_speeds", 10
        )

        self._log_count = 0

        self.get_logger().info(
            "corridor_follower_demo: base=%d kp=%.1f wall_limit=%.2f m "
            "turn_trigger=%.2f m"
            % (self.base, self.kp, self.wall_limit, self.turn_trig)
        )

    def _on_scan(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n < 8:
            return

        idx_left  = n // 4
        idx_front = n // 2
        idx_right = (3 * n) // 4

        left  = _min_in_window(msg.ranges, idx_left,  self.half, msg.range_max)
        front = _min_in_window(msg.ranges, idx_front, self.half, msg.range_max)
        right = _min_in_window(msg.ranges, idx_right, self.half, msg.range_max)

        # "Close" walls = within one cell.  Anything farther is treated
        # as open space (no wall on that side).
        l_close = left  < self.wall_limit
        r_close = right < self.wall_limit
        f_block = front < self.turn_trig

        state = "?"
        if f_block:
            # Forward blocked: turn in place toward the WIDER side.
            # (Always picking the wider side handles inside-of-corner cases
            # where both side walls are close but one side is slightly more
            # open — that is the way out.)
            base = 0
            if left > right:
                u = -self.turn_byte         # turn LEFT (left wheel slower)
                state = "TURN-LEFT"
            else:
                u = +self.turn_byte         # turn RIGHT (right wheel slower)
                state = "TURN-RIGHT"
        else:
            # Forward clear: drive forward, choose steering rule from walls.
            base = self.base
            if l_close and r_close:
                err = right - left          # balance the two
                state = "BALANCE"
            elif l_close:
                # Only LEFT wall close — keep target distance from it.
                # If left < target → too close → drift right (u > 0).
                err = self.side_target - left
                state = "HUG-LEFT"
            elif r_close:
                # Only RIGHT wall close — symmetric.
                # If right < target → too close → drift left (u < 0).
                err = right - self.side_target
                state = "HUG-RIGHT"
            else:
                err = 0.0
                state = "OPEN"
            u = int(round(self.kp * err))

        left_byte  = 127 + base + u           # drift toward more-open side
        right_byte = 127 + base - u
        left_byte  = max(self.min_byte, min(self.max_byte, left_byte))
        right_byte = max(self.min_byte, min(self.max_byte, right_byte))

        # Convention: set_motor_speeds is [left, right].
        out = UInt8MultiArray()
        out.data = [left_byte, right_byte]
        self.pub.publish(out)

        # Throttled log so the user can see what's happening.
        self._log_count += 1
        if self._log_count % 15 == 0:
            self.get_logger().info(
                "L=%.2f F=%.2f R=%.2f  state=%-12s  base=%d  u=%+d  motors=[%d,%d]"
                % (left, front, right, state, base, u, left_byte, right_byte)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CorridorFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
