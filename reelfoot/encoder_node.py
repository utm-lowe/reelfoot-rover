"""ROS 2 node that publishes raw encoder tick counts for all four wheels.

The low‑level :pymod:`reelfoot.encoder` module maintains global counters:
    FL, FR, RL, RR
These correspond to the four wheels. This node reads each counter and publishes
them on ``/front_left/ticks``, ``/front_right/ticks``, ``/rear_left/ticks`` and
``/rear_right/ticks`` as ``std_msgs/msg/Int64`` messages.

Typical usage (via the console‑script entry point defined in ``setup.py``)::

    ros2 run reelfoot encoder_node --ros-args -p publish_rate_hz:=50.0
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float64

# Import the low‑level encoder module which holds the global counters.
from . import encoder as _enc


class EncoderNode(Node):
    """Publishes the tick count for a single wheel.

    Parameters
    ----------
    wheel: str (default: "front_left")
        Identifier used to select the global counter and to form the topic
        name. Accepted values are ``front_left``, ``front_right``, ``rear_left``,
        ``rear_right``.
    publish_rate_hz: float (default: 20.0)
        How often the node reads the counter and publishes.
    """

    _VALID_WHEELS = {
        "front_left": "FL",
        "front_right": "FR",
        "rear_left": "RL",
        "rear_right": "RR",
    }

    def __init__(self) -> None:
        super().__init__("encoder_node")

        # Declare parameters with defaults.
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("ticks_per_rev", 20)
        self.declare_parameter("wheel_diameter_mm", 68.0)

        # Resolve publish rate and other parameters.
        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._ticks_per_rev = self.get_parameter("ticks_per_rev").get_parameter_value().integer_value
        wheel_diameter_mm = self.get_parameter("wheel_diameter_mm").get_parameter_value().double_value
        self._wheel_radius_m = (wheel_diameter_mm / 2.0) / 1000.0  # convert mm to meters

        # Map wheel names to encoder attributes and topics.
        self._wheel_map = {
            "front_left": ("FL", "/front_left/ticks"),
            "front_right": ("FR", "/front_right/ticks"),
            "rear_left": ("RL", "/rear_left/ticks"),
            "rear_right": ("RR", "/rear_right/ticks"),
        }
        # Create publishers for each wheel (ticks, angular speed, linear speed).
        self._pubs = {}
        self._ang_speed_pubs = {}
        self._lin_speed_pubs = {}
        for name, (_, _) in self._wheel_map.items():
            self._pubs[name] = self.create_publisher(Int64, f"/{name}/ticks", 10)
            self._ang_speed_pubs[name] = self.create_publisher(Float64, f"/{name}/angular_speed", 10)
            self._lin_speed_pubs[name] = self.create_publisher(Float64, f"/{name}/linear_speed", 10)
            self.get_logger().info(f"Publishing encoder, angular speed, and linear speed topics for '{name}'")

        # Store previous counts for speed calculation.
        self._prev_counts = {wheel: 0 for wheel in self._wheel_map}
        # Timer to periodically publish all wheels.
        timer_period = 1.0 / max(rate_hz, 0.001)
        self._dt = timer_period
        self._timer = self.create_timer(timer_period, self._publish_all_ticks)
    def _publish_all_ticks(self) -> None:
        """Read each global counter and publish its value on the corresponding topic."""
        for wheel_name, (attr, _) in self._wheel_map.items():
            # Current tick count
            count = getattr(_enc, attr, None)
            if count is None:
                self.get_logger().error(f"Counter attribute '{attr}' not found in encoder module.")
                continue
            # Publish tick count
            tick_msg = Int64()
            tick_msg.data = int(count)
            self._pubs[wheel_name].publish(tick_msg)
            self.get_logger().debug(f"Published {tick_msg.data} on /{wheel_name}/ticks")

            # Compute speed (delta ticks since last callback)
            prev = self._prev_counts.get(wheel_name, 0)
            delta_ticks = count - prev
            # Update stored count for next interval
            self._prev_counts[wheel_name] = count
            # Revolutions per second
            rev_per_sec = delta_ticks / self._ticks_per_rev / self._dt if self._dt > 0 else 0.0
            # Angular speed (rad/s)
            angular_speed = rev_per_sec * 2.0 * 3.141592653589793
            # Linear speed (m/s) = angular * radius
            linear_speed = angular_speed * self._wheel_radius_m
            # Publish angular speed
            ang_msg = Float64()
            ang_msg.data = angular_speed
            self._ang_speed_pubs[wheel_name].publish(ang_msg)
            # Publish linear speed
            lin_msg = Float64()
            lin_msg.data = linear_speed
            self._lin_speed_pubs[wheel_name].publish(lin_msg)
            self.get_logger().debug(
                f"Wheel {wheel_name}: angular={angular_speed:.3f} rad/s, linear={linear_speed:.3f} m/s"
            )

    # Deprecated single‑wheel method retained for compatibility (no‑op).
    def _publish_tick(self) -> None:
        pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EncoderNode()
        rclpy.spin(node)
    finally:
        # Ensure a clean shutdown.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
