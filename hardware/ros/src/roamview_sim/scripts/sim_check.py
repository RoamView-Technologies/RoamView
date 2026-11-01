#!/usr/bin/env python3
"""Is the running RoamView simulation healthy? One line per check: PASS, WARN or FAIL.

    ros2 run roamview_sim sim_check.py            # watches only; the robot is not moved
    ros2 run roamview_sim sim_check.py --drive    # also drives 0.4 m forward and back on /cmd_vel
    ros2 run roamview_sim sim_check.py --map      # warehouse club: also compares the LiDAR with the map

Start the simulation first (sim.launch.py), in another terminal with the same ROS_DOMAIN_ID.

  clock   /clock advances: real-time factor = simulated seconds per wall-clock second
  lidar   /scan arrives at about 10 Hz of simulated time, and not every beam reads its minimum range (that is
          what a broken render path does: see "Rendering on WSL" in hardware/ros/README.md)
  cliff   the four /cliff/* sensors see the floor about 87 mm below them (the sensors' mounting height)
  camera  /oak/image arrives and is not one flat colour (skipped with camera:=false)
  odom    /odom (wheel odometry from Gazebo) arrives
  map     (--map) LiDAR hits, placed with the ground-truth pose, land on obstacles drawn in maps/warehouse_club.pgm
  drive   (--drive) 0.2 m/s for 2 s of simulated time moves the robot about 0.35 m, then it comes back

The exit code is 0 when nothing FAILed, so scripts can use it.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan

CLIFFS = ('front_left', 'front_right', 'rear_left', 'rear_right')
CLIFF_HEIGHT = 0.087              # m, sensor to floor on flat ground
CLIFF_TRIGGER = 0.117             # m, what cliff_guard.py calls a drop


class Probe(Node):
    """Subscribes to everything it checks and keeps the last message and a count per topic."""

    def __init__(self, cmd_topic):
        super().__init__('sim_check')
        self.last, self.count = {}, {}
        topics = [('/clock', Clock), ('/scan', LaserScan), ('/odom', Odometry), ('/ground_truth/odom', Odometry),
                  ('/oak/image', Image)] + [(f'/cliff/{c}', LaserScan) for c in CLIFFS]
        for topic, msg_type in topics:
            self.count[topic] = 0
            self.create_subscription(msg_type, topic, lambda m, t=topic: self._store(t, m), qos_profile_sensor_data)
        self.cmd = self.create_publisher(TwistStamped, cmd_topic, 10)

    def _store(self, topic, msg):
        self.last[topic] = msg
        self.count[topic] += 1

    def sim_time(self):
        c = self.last.get('/clock')
        return None if c is None else c.clock.sec + c.clock.nanosec * 1e-9

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, vx, sim_seconds, wall_limit=30.0):
        """Publish a forward speed at 20 Hz until sim_seconds of simulated time have passed."""
        start, wall_end, nxt = self.sim_time(), time.time() + wall_limit, 0.0
        while self.sim_time() - start < sim_seconds and time.time() < wall_end:
            if time.time() >= nxt:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.linear.x = vx
                self.cmd.publish(msg)
                nxt = time.time() + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)


results = []


def report(name, status, detail):
    results.append(status)
    print(f'{status:<4}  {name:<6}  {detail}', flush=True)


def position(odom):
    p = odom.pose.pose.position
    return p.x, p.y


def yaw_of(odom):
    q = odom.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_pgm(path):
    """Binary PGM (P5) as a numpy array: the map format Nav2's map_server reads."""
    import numpy as np
    with open(path, 'rb') as f:
        data = f.read()
    fields, pos = [], 0
    while len(fields) < 4:                       # magic, width, height, max value; skip comments
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    w, h = int(fields[1]), int(fields[2])
    return np.frombuffer(data[pos + 1:pos + 1 + w * h], np.uint8).reshape(h, w)


def check_map(n):
    gt, scan = n.last.get('/ground_truth/odom'), n.last.get('/scan')
    if gt is None or scan is None:
        return report('map', 'FAIL', 'needs /ground_truth/odom and /scan')
    try:
        import numpy as np
        import yaml
        from ament_index_python.packages import get_package_share_directory
    except ImportError as e:
        return report('map', 'WARN', f'skipped: {e.name} is not installed (sudo apt install python3-numpy)')
    map_yaml = os.path.join(get_package_share_directory('roamview_sim'), 'maps', 'warehouse_club.yaml')
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(os.path.dirname(map_yaml), meta['image']))
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    x0, y0 = position(gt)
    if not (ox <= x0 < ox + img.shape[1] * res and oy <= y0 < oy + img.shape[0] * res):
        return report('map', 'WARN', f'the robot ({x0:.1f}, {y0:.1f}) is outside the warehouse club map (test world?)')
    occupied, k = img < 50, 4                     # grow obstacles by 4 cells = 0.2 m
    padded, near = np.pad(occupied, k), np.zeros_like(occupied)
    for dy in range(-k, k + 1):
        for dx in range(-k, k + 1):
            near |= padded[k + dy:k + dy + occupied.shape[0], k + dx:k + dx + occupied.shape[1]]
    r = np.array(scan.ranges, float)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment + yaw_of(gt)
    ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max * 0.98)
    col = ((x0 + r[ok] * np.cos(a[ok]) - ox) / res).astype(int)
    row = img.shape[0] - 1 - ((y0 + r[ok] * np.sin(a[ok]) - oy) / res).astype(int)
    inside = (col >= 0) & (col < img.shape[1]) & (row >= 0) & (row < img.shape[0])
    hits = near[row[inside], col[inside]]
    if hits.size == 0:
        return report('map', 'WARN', 'no LiDAR hits to compare')
    pct = 100.0 * hits.mean()
    detail = f'{pct:.0f} % of {hits.size} LiDAR hits within 0.2 m of an obstacle on the map'
    if pct >= 90:
        report('map', 'PASS', detail)
    elif pct >= 70:
        report('map', 'WARN', detail + ' (shoppers or clutter in view lower it)')
    else:
        report('map', 'FAIL', detail + ': the scan does not match the map')


def check_drive(n, topic):
    if '/odom' not in n.last:
        return report('drive', 'FAIL', 'no /odom to measure with')
    x0, y0 = position(n.last['/odom'])
    n.drive(0.2, 2.0)                              # forward 2 s of simulated time
    n.drive(0.0, 0.8)                              # stop and settle
    x1, y1 = position(n.last['/odom'])
    n.drive(-0.2, 2.0)                             # and back
    n.drive(0.0, 0.8)
    x2, y2 = position(n.last['/odom'])
    forward, back = math.hypot(x1 - x0, y1 - y0), math.hypot(x2 - x0, y2 - y0)
    detail = f'forward {forward:.2f} m (expected about 0.4), then back to {back:.2f} m from the start'
    if forward < 0.05:
        report('drive', 'FAIL', f'the robot moved {forward:.2f} m: commands on {topic} do not reach it '
                                f'(with guard:=true use --topic /cmd_vel_in)')
    elif 0.28 <= forward <= 0.5:
        report('drive', 'PASS', detail)
    else:
        report('drive', 'WARN', detail)


def check_live(n):
    """Watch for 5 s of wall time; returns False when simulated time is not moving (nothing else can be judged)."""
    before, s0, w0 = dict(n.count), n.sim_time(), time.time()
    n.spin_for(5.0)
    ds, dw = n.sim_time() - s0, time.time() - w0
    rtf = ds / dw
    if ds < 0.05:
        report('clock', 'FAIL', 'simulated time is not moving: Gazebo is paused (press play in its window) or '
                                'stuck loading')
        return False
    if rtf < 0.5:
        report('clock', 'WARN', f'real-time factor {rtf:.2f}: the PC is struggling (README, "It is slow")')
    else:
        report('clock', 'PASS', f'real-time factor {rtf:.2f}')
    rate = lambda topic: (n.count[topic] - before[topic]) / ds          # messages per simulated second

    scan = n.last.get('/scan')
    if scan is None:
        report('lidar', 'FAIL', 'no /scan: the robot was not spawned or the bridge is not running (launch log)')
    else:
        r = list(scan.ranges)
        at_min = sum(1 for v in r if math.isfinite(v) and v <= scan.range_min + 0.005)
        hits = [v for v in r if math.isfinite(v) and scan.range_min < v < scan.range_max]
        hz = rate('/scan')
        if r and at_min > 0.9 * len(r):
            report('lidar', 'FAIL', f'{at_min} of {len(r)} beams read the minimum range ({scan.range_min:.2f} m): '
                                    'the sensors render on a broken path (README, "Rendering on WSL")')
        elif hz < 7:
            report('lidar', 'WARN', f'{hz:.1f} scans per simulated second (expected 10)')
        elif not hits:
            report('lidar', 'WARN', f'{hz:.1f} Hz, but nothing within {scan.range_max:.0f} m')
        else:
            report('lidar', 'PASS', f'{hz:.1f} Hz, {len(r)} beams, {len(hits)} hits, nearest {min(hits):.2f} m')

    floor, missing = {}, []
    for c in CLIFFS:
        m = n.last.get(f'/cliff/{c}')
        if m is None:
            missing.append(c)
            continue
        vals = sorted(v for v in m.ranges if not math.isnan(v))
        floor[c] = vals[len(vals) // 2] if vals else float('inf')
    if len(missing) == len(CLIFFS):
        report('cliff', 'FAIL', 'no /cliff/* topics')
    elif any(v <= 0.02 for v in floor.values()):
        report('cliff', 'FAIL', 'the sensors read their minimum range: broken render path (README, '
                                '"Rendering on WSL")')
    elif any(v > CLIFF_TRIGGER for v in floor.values()):
        drop = ', '.join(f'{c} {v:.3f} m' for c, v in floor.items() if v > CLIFF_TRIGGER)
        report('cliff', 'WARN', f'a drop under {drop}: is the robot at an edge?')
    elif missing:
        report('cliff', 'WARN', f'no reading from {", ".join(missing)}')
    else:
        report('cliff', 'PASS', f'floor {min(floor.values()):.3f}-{max(floor.values()):.3f} m below '
                                f'(mounted {CLIFF_HEIGHT:.3f} m up)')

    img = n.last.get('/oak/image')
    if img is None:
        report('camera', 'WARN', 'no /oak/image (started with camera:=false?)')
    else:
        data = img.data
        sample = [int(b) for b in data[::max(1, len(data) // 5000)]]
        mean = sum(sample) / len(sample)
        spread = math.sqrt(sum((b - mean) ** 2 for b in sample) / len(sample))
        hz = rate('/oak/image')
        if spread < 3:
            report('camera', 'FAIL', f'the image is one flat colour (level {mean:.0f} of 255): nothing is rendered')
        elif hz < 10:
            report('camera', 'WARN', f'{hz:.1f} images per simulated second (expected 15)')
        else:
            report('camera', 'PASS', f'{img.width} x {img.height} {img.encoding}, {hz:.1f} Hz')

    if '/odom' in n.last:
        report('odom', 'PASS', f'{rate("/odom"):.0f} Hz')
    else:
        report('odom', 'FAIL', 'no /odom: the DiffDrive plugin or the bridge is missing')
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--drive', action='store_true', help='also drive 0.4 m forward and back')
    ap.add_argument('--map', action='store_true', help='also compare the LiDAR with the warehouse club map')
    ap.add_argument('--topic', default='/cmd_vel', help='where --drive sends speeds (/cmd_vel_in with guard:=true)')
    ap.add_argument('--wait', type=float, default=60.0, help='seconds to wait for the simulation (default 60)')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    n = Probe(args.topic)
    print(f'RoamView sim check (ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")})', flush=True)
    try:
        t0 = time.time()
        while n.sim_time() is None and time.time() - t0 < args.wait:
            n.spin_for(0.2)
        if n.sim_time() is None:
            report('clock', 'FAIL', f'no /clock after {args.wait:.0f} s: is sim.launch.py running, with the same '
                                    'ROS_DOMAIN_ID in both terminals?')
        else:
            while ('/scan' not in n.last or '/odom' not in n.last) and time.time() - t0 < args.wait:
                n.spin_for(0.2)                    # the robot spawns a few seconds after the world
            if check_live(n):
                if args.map:
                    check_map(n)
                if args.drive:
                    check_drive(n, args.topic)
    except KeyboardInterrupt:
        pass
    finally:
        if args.drive:
            n.cmd.publish(TwistStamped())          # never leave the robot driving
        fails, warns = results.count('FAIL'), results.count('WARN')
        print(f'{len(results) - fails - warns} passed, {warns} warnings, {fails} failed', flush=True)
        n.destroy_node()
        rclpy.try_shutdown()
    sys.exit(1 if fails else 0)


if __name__ == '__main__':
    main()
