#!/usr/bin/env python3
"""cliff_guard: a small safety filter, the simulator's stand-in for the ESP32 cliff stop.

    teleop or Nav2 --/cmd_vel_in--> cliff_guard --/cmd_vel--> Gazebo DiffDrive (plays the motor firmware)

Rules (old-designs/DESIGN_RULES.md, cliff sensors):
  - a front cliff sensor reading more than floor + margin blocks forward motion;
  - a rear cliff sensor blocks reverse;
  - a bumper contact blocks forward motion;
  - forward speed is capped at 0.3 m/s (the limit near edges) and reverse at 0.1 m/s;
  - no command for 0.5 s -> send zero (a watchdog).
The ramp-down itself (0.5 m/s^2) is done by the DiffDrive plugin's deceleration limit.
On the real robot this logic runs on the ESP32, not in ROS; here it is a ROS node so you can read and change it.

Change a limit while it runs:  ros2 param set /cliff_guard max_forward 0.2
Run it with the simulation:   ros2 launch roamview_sim sim.launch.py guard:=true
and point teleop at its input: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args
                                   -p stamped:=true -r cmd_vel:=cmd_vel_in
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

SENSORS = ('front_left', 'front_right', 'rear_left', 'rear_right')


class CliffGuard(Node):
    def __init__(self):
        super().__init__('cliff_guard')
        self.floor = self.declare_parameter('floor_range', 0.087).value      # m, window to floor
        self.margin = self.declare_parameter('cliff_margin', 0.030).value    # m above floor_range = cliff
        self.max_fwd = self.declare_parameter('max_forward', 0.3).value      # m/s
        self.max_rev = self.declare_parameter('max_reverse', 0.1).value      # m/s
        self.timeout = self.declare_parameter('timeout', 0.5).value          # s
        rate = self.declare_parameter('rate', 50.0).value                    # Hz

        self.cliff = {k: False for k in SENSORS}
        self.range = {k: float('nan') for k in SENSORS}
        self.bump_time = {'left': None, 'right': None}   # Gazebo only publishes contacts while touching
        self.cmd, self.cmd_time = None, None
        self.last_status = ''

        for k in SENSORS:
            self.create_subscription(LaserScan, f'cliff/{k}', lambda m, k=k: self.on_cliff(k, m),
                                     qos_profile_sensor_data)
        for side in self.bump_time:
            self.create_subscription(Contacts, f'bumper/{side}', lambda m, s=side: self.on_bump(s, m), 10)
        self.create_subscription(TwistStamped, 'cmd_vel_in', self.on_cmd, 10)
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'cliff_guard/status', 10)
        self.create_timer(1.0 / rate, self.tick)
        self.add_on_set_parameters_callback(self.on_params)   # lets `ros2 param set` change limits live
        self.get_logger().info(f'cliff_guard ready: cliff if a sensor reads more than '
                               f'{self.floor + self.margin:.3f} m; forward <= {self.max_fwd} m/s, '
                               f'reverse <= {self.max_rev} m/s')

    def on_params(self, params):
        for p in params:
            if p.name == 'max_forward':
                self.max_fwd = float(p.value)
            elif p.name == 'max_reverse':
                self.max_rev = float(p.value)
            elif p.name == 'cliff_margin':
                self.margin = float(p.value)
            elif p.name == 'floor_range':
                self.floor = float(p.value)
            elif p.name == 'timeout':
                self.timeout = float(p.value)
            else:
                continue
            self.get_logger().info(f'{p.name} is now {p.value}')
        return SetParametersResult(successful=True)

    def on_cliff(self, name, msg):
        # The sensor sees floor while any part of its cone does, so take the nearest valid ray.
        valid = [r for r in msg.ranges if math.isfinite(r)]
        r = min(valid) if valid else float('inf')     # nothing within 1.3 m = a drop
        self.range[name] = r
        self.cliff[name] = r > self.floor + self.margin

    def on_bump(self, side, msg):
        if msg.contacts:
            self.bump_time[side] = self.get_clock().now()

    def pressed(self, now):
        # a half counts as pressed if it reported a contact in the last 0.15 s
        return any(t is not None and (now - t).nanoseconds * 1e-9 < 0.15 for t in self.bump_time.values())

    def on_cmd(self, msg):
        self.cmd, self.cmd_time = msg, self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        v = w = 0.0
        if self.cmd is not None and (now - self.cmd_time).nanoseconds * 1e-9 < self.timeout:
            v, w = self.cmd.twist.linear.x, self.cmd.twist.angular.z

        front = self.cliff['front_left'] or self.cliff['front_right']
        rear = self.cliff['rear_left'] or self.cliff['rear_right']
        bump = self.pressed(now)
        reasons = []
        if front:
            reasons.append('front cliff')
        if rear:
            reasons.append('rear cliff')
        if bump:
            reasons.append('bumper')
        if (front or bump) and v > 0.0:
            v = 0.0
        if rear and v < 0.0:
            v = 0.0
        v = max(-self.max_rev, min(self.max_fwd, v))

        out = TwistStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x, out.twist.angular.z = v, w
        self.pub.publish(out)

        status = 'clear' if not reasons else 'BLOCKED: ' + ', '.join(reasons)
        if status != self.last_status:
            ranges = ', '.join(f'{k} {self.range[k]:.3f}' for k in SENSORS)
            if reasons:
                self.get_logger().warn(f'{status}  (cliff m: {ranges})')
            else:
                self.get_logger().info(f'{status}  (cliff m: {ranges})')
            self.last_status = status
        self.status_pub.publish(String(data=status))


def main():
    rclpy.init()
    node = CliffGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
