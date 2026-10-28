#!/usr/bin/env python3
"""
Reelfoot Rover drive node.

Subscribes to geometry_msgs/Twist on cmd_vel and turns it into mecanum
wheel commands for the four motors in reelfoot.motors.

ROS conventions (REP 103): linear.x forward, linear.y left, angular.z
counterclockwise. Speeds are open loop: max_linear and max_angular set
which velocities map to full power.
"""
import rclpy
from std_msgs.msg import Int8
from rclpy.node import Node
from geometry_msgs.msg import Twist

from . import motors as m


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive')
        self.declare_parameter('max_linear', 0.5)    # m/s mapped to full power
        self.declare_parameter('max_angular', 2.0)   # rad/s mapped to full power
        self.declare_parameter('min_pwm', 25.0)      # % duty where the motors start to turn
        self.declare_parameter('timeout', 1.0)       # s without cmd_vel before stopping
        self.declare_parameter('invert', [False, False, False, False])  # FL FR RL RR

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.timeout = self.get_parameter('timeout').value
        self.invert = list(self.get_parameter('invert').value)

        self.motors = [m.FL, m.FR, m.RL, m.RR]
        self.moving = False
        self.last_cmd = self.get_clock().now()

        # Declare publish rate parameter (same default as encoder node)
        self.declare_parameter('publish_rate_hz', 20.0)
        rate_hz = self.get_parameter('publish_rate_hz').value
        timer_period = 1.0 / max(rate_hz, 0.001)
        self._timer = self.create_timer(timer_period, self._publish_directions)
        self._last_dir_vals = [0, 0, 0, 0]
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.pub_fl_dir = self.create_publisher(Int8, '/front_left/direction', 10)
        self.pub_fr_dir = self.create_publisher(Int8, '/front_right/direction', 10)
        self.pub_rl_dir = self.create_publisher(Int8, '/rear_left/direction', 10)
        self.pub_rr_dir = self.create_publisher(Int8, '/rear_right/direction', 10)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog)

    def on_cmd(self, msg):
        """Mecanum inverse kinematics, normalized to [-1, 1] per wheel."""
        vx = msg.linear.x / self.max_linear
        vy = msg.linear.y / self.max_linear
        wz = msg.angular.z / self.max_angular

        wheels = [
            vx - vy - wz,   # FL
            vx + vy + wz,   # FR
            vx + vy - wz,   # RL
            vx - vy + wz,   # RR
        ]
        # if any wheel would exceed full power, scale all of them down
        # together so the direction of motion is preserved
        peak = max(1.0, *(abs(w) for w in wheels))
        self.drive([w / peak for w in wheels])
        self.last_cmd = self.get_clock().now()

    def drive(self, wheels):
        self.moving = any(abs(w) > 0.01 for w in wheels)
        # Store last direction values for publishing at configured rate.
        self._last_dir_vals = [
            0 if abs(w) < 0.01 else (1 if w > 0 else -1) for w in wheels
        ]
        for motor, w, inv in zip(self.motors, wheels, self.invert):
            if inv:
                w = -w
            if abs(w) < 0.01:
                motor.set_speed(0)
                motor.set_direction(m.BRAKE)
            else:
                motor.set_direction(m.FORWARD if w > 0 else m.REVERSE)
                motor.set_speed(self.min_pwm + (100 - self.min_pwm) * abs(w))

    def _publish_directions(self):
        # Publish the most recent direction values at the configured rate.
        dirs = self._last_dir_vals
        self.pub_fl_dir.publish(Int8(data=dirs[0]))
        self.pub_fr_dir.publish(Int8(data=dirs[1]))
        self.pub_rl_dir.publish(Int8(data=dirs[2]))
        self.pub_rr_dir.publish(Int8(data=dirs[3]))

    def stop(self):
        self.drive([0.0, 0.0, 0.0, 0.0])

    def watchdog(self):
        """Stop if the commands stop coming (dropped teleop, crashed planner)."""
        idle = (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9
        if self.moving and idle > self.timeout:
            self.get_logger().warning('cmd_vel timed out, stopping')
            self.stop()

    def destroy_node(self):
        self.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
