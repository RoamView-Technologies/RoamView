#!/usr/bin/env python3
"""Patrol the warehouse club with Nav2 (nav2_simple_commander): drive through the waypoints in
config/warehouse_club_patrol.yaml and report each one.

    ros2 run roamview_sim patrol.py --ros-args -p use_sim_time:=true
    ros2 run roamview_sim patrol.py --ros-args -p use_sim_time:=true -p loops:=0            # forever
    ros2 run roamview_sim patrol.py --ros-args -p use_sim_time:=true -p first:=0 -p last:=4  # a short test

Parameters
  waypoints   path to the patrol YAML (default: the one installed with the package)
  loops       how many times to go round (0 = until Ctrl+C)
  first, last use only waypoints first..last (inclusive; -1 = to the end)
  start       dock | entrance | receiving: tell AMCL where the robot is before starting (default: none)
"""
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    rclpy.init(args=sys.argv)
    nav = BasicNavigator('roamview_patrol')
    default = os.path.join(get_package_share_directory('roamview_sim'), 'config', 'warehouse_club_patrol.yaml')
    path = nav.declare_parameter('waypoints', default).value
    loops = int(nav.declare_parameter('loops', 1).value)
    first = int(nav.declare_parameter('first', 0).value)
    last = int(nav.declare_parameter('last', -1).value)
    start = str(nav.declare_parameter('start', 'none').value)
    with open(path) as f:
        cfg = yaml.safe_load(f)
    wps = cfg['waypoints'][first:(None if last < 0 else last + 1)]

    if start != 'none':
        x, y, yaw = cfg['spawns'][start]
        nav.setInitialPose(pose(nav, x, y, yaw))
        nav.get_logger().info(f'initial pose set to {start} ({x}, {y})')
    nav.waitUntilNav2Active(localizer='amcl')

    k = 0
    while rclpy.ok() and (loops == 0 or k < loops):
        k += 1
        goals = [pose(nav, w['x'], w['y'], w['yaw']) for w in wps]
        nav.get_logger().info(f'patrol {k}: {len(goals)} waypoints')
        t0 = time.monotonic()
        nav.followWaypoints(goals)
        last_idx = -1
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb and fb.current_waypoint != last_idx:
                last_idx = fb.current_waypoint
                w = wps[last_idx]
                nav.get_logger().info(f'  -> waypoint {first + last_idx} ({w["x"]:.1f}, {w["y"]:.1f})')
            time.sleep(0.2)
        result = nav.getResult()
        res = nav.result_future.result().result if nav.result_future else None
        missed = list(getattr(res, 'missed_waypoints', []) or [])
        missed = [getattr(m, 'index', m) for m in missed]
        took = time.monotonic() - t0
        if result == TaskResult.SUCCEEDED and not missed:
            nav.get_logger().info(f'patrol {k} done in {took:.0f} s (wall clock)')
        else:
            nav.get_logger().warn(f'patrol {k} ended: {result}, missed waypoints {missed}')
            if result == TaskResult.CANCELED:
                break
    nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
