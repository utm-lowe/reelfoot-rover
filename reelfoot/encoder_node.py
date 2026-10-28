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
from std_msgs.msg import Int64

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

        # Resolve publish rate.
        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        # Map wheel names to encoder attributes and topics.
        self._wheel_map = {
            "front_left": ("FL", "/front_left/ticks"),
            "front_right": ("FR", "/front_right/ticks"),
            "rear_left": ("RL", "/rear_left/ticks"),
            "rear_right": ("RR", "/rear_right/ticks"),
        }
        # Create a publisher for each wheel.
        self._pubs = {}
        for name, (_, topic) in self._wheel_map.items():
            self._pubs[name] = self.create_publisher(Int64, topic, 10)
            self.get_logger().info(f"Publishing encoder ticks on topic '{topic}'")

        # Timer to periodically publish all wheels.
        timer_period = 1.0 / max(rate_hz, 0.001)
        self._timer = self.create_timer(timer_period, self._publish_all_ticks)

    def _publish_all_ticks(self) -> None:
        """Read each global counter and publish its value on the corresponding topic."""
        for wheel_name, (attr, topic) in self._wheel_map.items():
            count = getattr(_enc, attr, None)
            if count is None:
                self.get_logger().error(f"Counter attribute '{attr}' not found in encoder module.")
                continue
            msg = Int64()
            msg.data = int(count)
            self._pubs[wheel_name].publish(msg)
            self.get_logger().debug(f"Published {msg.data} on {topic}")

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
