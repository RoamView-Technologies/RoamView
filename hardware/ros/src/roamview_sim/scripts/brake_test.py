#!/usr/bin/env python3
"""brake_test: drive straight, stop, and measure what really happened (from Gazebo's ground truth).

  mode:=stop  speed up to `speed`, cruise for `cruise` s, command zero; prints stopping distance and time,
              peak deceleration and pitch. Publishes to /cmd_vel.
  mode:=edge  drive forward at `speed` into /cmd_vel_in until the robot stops by itself (run the sim with
              guard:=true and spawn facing the stairwell); prints how far the axle travelled after the first
              cliff reading, whether the front caster and the drive wheels went over, the pitch, and whether
              the robot fell.

Examples:
  ros2 run roamview_sim brake_test.py --ros-args -p use_sim_time:=true -p speed:=0.3
  ros2 launch roamview_sim sim.launch.py x:=2.9 y:=-1.0 yaw:=-1.5708 guard:=true
  ros2 run roamview_sim brake_test.py --ros-args -p use_sim_time:=true -p mode:=edge -p speed:=0.3

Design numbers it compares against (old-designs/DESIGN_RULES.md): front cliff sensors 131 mm ahead of the axle,
front caster contact 98 mm ahead (so it goes over ~33 mm after the sensors see the edge), stop ramp 0.5 m/s^2.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

SENSOR_AHEAD = 0.131   # front cliff sensors ahead of the axle (m)
CASTER_AHEAD = 0.098   # front caster contact ahead of the axle while driving forward (m)


def pitch_of(q):
    s = 2.0 * (q.w * q.y - q.z * q.x)
    return math.degrees(math.asin(max(-1.0, min(1.0, s))))   # + = nose down


class BrakeTest(Node):
    def __init__(self):
        super().__init__('brake_test')
        self.mode = self.declare_parameter('mode', 'stop').value
        self.speed = float(self.declare_parameter('speed', 0.3).value)
        self.cruise = float(self.declare_parameter('cruise', 1.0).value)
        self.limit = float(self.declare_parameter('timeout', 20.0).value)
        default_topic = 'cmd_vel_in' if self.mode == 'edge' else 'cmd_vel'
        topic = self.declare_parameter('cmd_topic', default_topic).value
        self.cliff_at = 0.087 + 0.030

        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.create_subscription(Odometry, 'ground_truth/odom', self.on_truth, 10)
        for k in ('front_left', 'front_right'):
            self.create_subscription(LaserScan, f'cliff/{k}', self.on_cliff, qos_profile_sensor_data)

        self.phase, self.t_phase = 'wait', None
        self.pos = self.prev = None
        self.v = 0.0
        self.z0 = None
        self.start = self.brake_pos = self.cliff_pos = None
        self.v_brake = self.t_brake = None
        self.peak_decel = self.peak_pitch = 0.0
        self.min_z = float('inf')
        self.still_since = self.t_stop = None
        self.tip = None
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f'brake test, mode {self.mode}, {self.speed} m/s, commands on /{topic}')

    # ---------- inputs ----------
    def on_truth(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p = msg.pose.pose.position
        pos = (p.x, p.y, p.z, t)
        if self.prev is not None and t > self.prev[3]:
            dt = t - self.prev[3]
            v_new = math.hypot(p.x - self.prev[0], p.y - self.prev[1]) / dt
            a = (v_new - self.v) / dt
            self.v += 0.3 * (v_new - self.v)                     # light smoothing
            if self.phase in ('brake', 'drive') and self.brake_pos is not None:
                self.peak_decel = max(self.peak_decel, -a * 0.3)
        self.prev = pos
        self.pos = pos
        if self.z0 is None:
            self.z0 = p.z
        self.min_z = min(self.min_z, p.z)
        if self.phase not in ('wait', 'done'):
            pitch = pitch_of(msg.pose.pose.orientation)
            self.peak_pitch = max(self.peak_pitch, pitch)
            if pitch > 5.0 and self.tip is None:
                ref = self.cliff_pos or self.brake_pos or self.start
                self.tip = (self.travelled(ref, pos) if ref else 0.0, self.v)

    def on_cliff(self, msg):
        valid = [r for r in msg.ranges if math.isfinite(r)]
        r = min(valid) if valid else float('inf')
        if r > self.cliff_at and self.cliff_pos is None and self.phase in ('accel', 'drive') and self.pos:
            self.cliff_pos = self.pos
            self.brake_pos, self.v_brake, self.t_brake = self.pos, self.v, self.pos[3]
            self.get_logger().info(f'cliff seen at {self.v:.3f} m/s')

    # ---------- state machine ----------
    def send(self, v):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = v
        self.pub.publish(m)

    def elapsed(self):
        return (self.get_clock().now() - self.t_phase).nanoseconds * 1e-9

    def go(self, phase):
        self.phase, self.t_phase = phase, self.get_clock().now()

    def travelled(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def tick(self):
        if self.phase == 'wait':
            if self.pos is not None:
                self.start = self.pos
                self.go('accel')
            return
        if self.phase == 'done':
            return
        if self.phase == 'accel':
            self.send(self.speed)
            if self.v >= 0.97 * self.speed or self.elapsed() > 5.0:
                self.go('cruise' if self.mode == 'stop' else 'drive')
            return
        if self.phase == 'cruise':
            self.send(self.speed)
            if self.elapsed() >= self.cruise:
                self.brake_pos, self.v_brake, self.t_brake = self.pos, self.v, self.pos[3]
                self.peak_decel = 0.0
                self.go('brake')
            return
        if self.phase == 'drive':
            self.send(self.speed)                   # cliff_guard decides whether this gets through
            if self.cliff_pos is None and self.elapsed() > self.limit:
                self.report(no_cliff=True)
                return
        if self.phase == 'brake':
            self.send(0.0)
        # stopped?
        if self.brake_pos is not None:
            if self.v < 0.005:
                if self.still_since is None:
                    self.still_since, self.t_stop = self.get_clock().now(), self.pos[3]
                if (self.get_clock().now() - self.still_since).nanoseconds * 1e-9 > 0.5:
                    self.report()
            else:
                self.still_since = None

    def report(self, no_cliff=False):
        self.send(0.0)
        self.go('done')
        log = self.get_logger().info
        drop = self.min_z - self.z0
        fell = drop < -0.05
        log('=' * 60)
        if no_cliff:
            log(f'no cliff reading within {self.limit:.0f} s - is the robot facing the stairwell?')
        elif self.mode == 'stop':
            d = self.travelled(self.brake_pos, self.pos)
            t = self.t_stop - self.t_brake
            ideal = self.v_brake ** 2 / (2 * 0.5)
            log(f'speed when the stop was commanded : {self.v_brake:.3f} m/s')
            log(f'stopping distance                 : {d * 1000:.0f} mm in {t:.2f} s '
                f'(v^2 / 2a at 0.5 m/s^2 = {ideal * 1000:.0f} mm)')
            log(f'peak deceleration (ground truth)  : {self.peak_decel:.2f} m/s^2 ({self.peak_decel / 9.81:.2f} g)')
        else:
            d = self.travelled(self.cliff_pos, self.pos)
            log(f'speed at the first cliff reading  : {self.v_brake:.3f} m/s')
            log(f'axle travel after that reading    : {d * 1000:.0f} mm (sensors are {SENSOR_AHEAD * 1000:.0f} mm '
                f'ahead of the axle)')
            log(f'front caster went over the edge   : {"yes" if d > SENSOR_AHEAD - CASTER_AHEAD else "no"}')
            log(f'drive wheels reached the edge     : {"YES" if d > SENSOR_AHEAD else "no"}')
        log(f'peak pitch                        : {self.peak_pitch:.1f} deg nose down')
        if self.tip:
            log(f'pitch passed 5 deg                : {self.tip[0] * 1000:.0f} mm after the '
                f'{"cliff reading" if self.mode == "edge" else "stop command"}, at {self.tip[1]:.2f} m/s')
        log(f'lowest height change              : {drop * 1000:.0f} mm -> '
            f'{"FELL" if fell else "stayed on the floor"}')
        log('=' * 60)
        raise SystemExit


def main():
    rclpy.init()
    node = BrakeTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
