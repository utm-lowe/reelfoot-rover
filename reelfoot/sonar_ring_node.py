#!/usr/bin/env python3
"""Reelfoot Rover sonar ring: publishes a sensor_msgs/Range for each sonar."""
import math
import threading
import time

import lgpio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from . import pins as p

SPEED_OF_SOUND = 343.0  # m/s at about 20 C


class SonarRing:
    """Sequentially fired sonars whose echo lines are OR'd onto one GPIO."""

    def __init__(self, triggers, echo, chip=0):
        self.triggers = triggers
        self.echo = echo
        self.done = threading.Event()
        self.start = None
        self.end = None

        self.handle = lgpio.gpiochip_open(chip)
        for t in triggers:
            lgpio.gpio_claim_output(self.handle, t, 0)
        time.sleep(0.2)   # let any echo from a claim-glitch ping finish
        lgpio.gpio_claim_alert(self.handle, echo, lgpio.BOTH_EDGES,
                               lgpio.SET_PULL_NONE)
        time.sleep(0.1)
        self.cb = lgpio.callback(self.handle, echo, lgpio.BOTH_EDGES, self._edge)
        time.sleep(0.1)   # give lgpio's callback thread time to come up

    def _edge(self, chip, gpio, level, tick):
        """lgpio alert callback; tick is in nanoseconds."""
        if level == 1:
            self.start = tick
        elif self.start is not None:      # ignore orphan falling edges
            self.end = tick
            self.done.set()

    def ping(self, idx, timeout=0.04):
        """Fire sonar idx; return distance in meters, or inf on timeout."""
        # don't fire while a previous echo is still holding the OR'd line high
        deadline = time.monotonic() + 0.25
        while (lgpio.gpio_read(self.handle, self.echo)
               and time.monotonic() < deadline):
            time.sleep(0.001)
        self.start = self.end = None
        self.done.clear()
        lgpio.tx_pulse(self.handle, self.triggers[idx], 10, 10, 0, 1)
        if not self.done.wait(timeout):
            return math.inf
        return (self.end - self.start) / 1e9 * SPEED_OF_SOUND / 2

    def close(self):
        self.cb.cancel()
        for t in self.triggers:
            lgpio.gpio_write(self.handle, t, 0)
        lgpio.gpiochip_close(self.handle)


class SonarRingNode(Node):
    def __init__(self):
        super().__init__("sonar_ring")
        self.declare_parameter("timeout", 0.04)        # s; 0.04 covers ~4 m
        self.declare_parameter("min_range", 0.02)      # m
        self.declare_parameter("max_range", 4.0)       # m
        self.declare_parameter("field_of_view", 0.26)  # rad, about 15 degrees

        self.timeout = self.get_parameter("timeout").value
        self.min_range = self.get_parameter("min_range").value
        self.max_range = self.get_parameter("max_range").value
        self.fov = self.get_parameter("field_of_view").value

        triggers = (p.SO1_Trig, p.SO2_Trig, p.SO3_Trig)
        self.ring = SonarRing(triggers, p.SO_Echo)

        # names match the pin labels: sonar_1, sonar_2, sonar_3
        # (ROS name tokens can't start with a digit, so no sonar/1)
        self.names = [f"sonar_{i + 1}" for i in range(len(triggers))]
        self.pubs = [self.create_publisher(Range, n, qos_profile_sensor_data)
                     for n in self.names]

        self.running = True
        self.thread = threading.Thread(target=self.scan_loop, daemon=True)
        self.thread.start()

    def scan_loop(self):
        while self.running and rclpy.ok():
            for i, (name, pub) in enumerate(zip(self.names, self.pubs)):
                if not self.running:
                    return
                stamp = self.get_clock().now().to_msg()
                d = self.ring.ping(i, self.timeout)

                msg = Range()
                msg.header.stamp = stamp
                msg.header.frame_id = name
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = self.fov
                msg.min_range = self.min_range
                msg.max_range = self.max_range
                # REP 117: -inf = too close, +inf = nothing detected
                if d < self.min_range:
                    msg.range = -math.inf
                elif d > self.max_range:
                    msg.range = math.inf
                else:
                    msg.range = d
                pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)   # finish the current ping before closing GPIO
        self.ring.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SonarRingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
