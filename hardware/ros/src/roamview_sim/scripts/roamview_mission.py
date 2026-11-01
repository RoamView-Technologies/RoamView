#!/usr/bin/env python3
"""roamview_mission.py — full autonomous mission controller for RoamView (Gazebo simulation).

STATE MACHINE
─────────────
  INIT      → wait for Nav2 to be ready (background thread)
  EXPLORE   → frontier-based exploration while slam_toolbox builds the map
  AWAIT     → map ready; idle, waiting for user commands
  NAVIGATING→ driving to a user-supplied goal via Nav2
  INSPECTING→ arrived; pauses for INSPECT_DURATION seconds, then back to AWAIT
  RECOVERY  → cliff or bumper event; backs up, re-sends the interrupted goal
  DOCKING   → low battery or "dock" command; navigates to the charging dock
  DOCKED    → at dock; simulates charging until battery is full, then back to AWAIT

CONDITIONS HANDLED AUTOMATICALLY
──────────────────────────────────
  • Cliff sensor (any of the 4) or bumper contact → RECOVERY (back up + retry)
  • Nav2 goal failure → retry up to MAX_WAYPOINT_RETRIES, then skip and return to AWAIT
  • Person near (/people_near) → pause Nav2, wait PERSON_CLEAR_WAIT s after they leave
  • Battery ≤ BATTERY_DOCK_THRESHOLD → cancel current task, go DOCK

USER INTERFACE (publish from another terminal)
──────────────────────────────────────────────
  # Send the robot to a specific location
  ros2 topic pub --once /mission/goal geometry_msgs/msg/PoseStamped \\
    "{header: {frame_id: map}, pose: {position: {x: 50.0, y: 20.0}, orientation: {w: 1.0}}}"

  # Commands: explore | dock | cancel | save_map
  ros2 topic pub --once /mission/command std_msgs/msg/String "data: explore"
  ros2 topic pub --once /mission/command std_msgs/msg/String "data: dock"
  ros2 topic pub --once /mission/command std_msgs/msg/String "data: cancel"

  # Simulate a person nearby (for testing; real use: plug in OAK-D detection node)
  ros2 topic pub /people_near std_msgs/msg/Bool "data: true"

MONITOR
───────
  ros2 topic echo /mission/state      # current state name
  ros2 topic echo /mission/status     # human-readable one-liner with battery %

FULL STARTUP SEQUENCE
──────────────────────
  # Terminal 1 — simulation (guard routes velocity through safety filter)
  ros2 launch roamview_sim sim.launch.py world:=warehouse_club guard:=true rviz:=true

  # Terminal 2 — SLAM (builds the map during EXPLORE phase)
  ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

  # Terminal 3 — Nav2 with slam_toolbox as localizer (no pre-built map needed)
  ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

  # Terminal 4 — this script
  ros2 run roamview_sim roamview_mission.py --ros-args -p use_sim_time:=true

  After the map is built, send commands on /mission/goal or /mission/command.
  To save the map at any time: ros2 topic pub --once /mission/command std_msgs/msg/String "data: save_map"

NOTES
──────
  • vel_topic parameter: use /cmd_vel (default) or /cmd_vel_in when guard:=true so cliff_guard
    filters the recovery back-up velocity too.
  • Person detection: /people_near (std_msgs/Bool) is published by a separate OAK-D detection
    node. In simulation you can publish it manually to test the pause-and-wait behaviour.
  • Battery is simulated internally (distance + time). It resets to 100 % when docked.
  • The EXPLORE state requires slam_toolbox; if you already have a map and start Nav2 with AMCL
    instead, skip to AWAIT by sending: ros2 topic pub --once /mission/command std_msgs/msg/String "data: cancel"
"""
import math
import subprocess
import threading
import time
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

# ── tuneable constants ─────────────────────────────────────────────────────────
DOCK_POSE         = (1.6, 92.0, 0.0)    # (x, y, yaw) — matches CLUB_SPAWNS['dock']
BATTERY_FULL      = 100.0
BATTERY_DOCK_THR  = 20.0    # % — go dock below this
BATTERY_M_DRAIN   = 0.05    # % per metre driven
BATTERY_S_DRAIN   = 0.002   # % per second (idle / turning)
BATTERY_CHARGE    = 0.1     # % per 10 Hz tick (100 s to full)
INSPECT_DURATION  = 5.0     # s — pause at each goal for inspection
BACKUP_DIST       = 0.25    # m — how far to reverse in RECOVERY
BACKUP_SPEED      = -0.08   # m/s (negative = reverse)
MAX_WP_RETRIES    = 2       # Nav2 retries before skipping a goal
FRONTIER_MIN_CELLS= 3       # minimum cluster size to be a valid frontier
FRONTIER_REVISIT  = 1.5     # m — ignore frontiers within this radius of a visited one
PERSON_CLEAR_WAIT = 3.0     # s — wait after /people_near goes False before resuming
CLIFF_THRESHOLD   = 0.117   # m — floor_range(0.087) + cliff_margin(0.030), from cliff_guard.py


class State(Enum):
    INIT       = auto()
    EXPLORE    = auto()
    AWAIT      = auto()
    NAVIGATING = auto()
    INSPECTING = auto()
    RECOVERY   = auto()
    DOCKING    = auto()
    DOCKED     = auto()


def _make_pose(nav: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


# ── main node ──────────────────────────────────────────────────────────────────

class RoamViewMission(Node):
    def __init__(self):
        super().__init__('roamview_mission')

        # configurable
        vel_topic   = self.declare_parameter('vel_topic',  '/cmd_vel').value
        slam_mode   = self.declare_parameter('slam_mode',  True).value   # False → assume AMCL + existing map
        map_save_path = self.declare_parameter('map_save_path', '/tmp/roamview_map').value
        self._slam_mode    = bool(slam_mode)
        self._map_save_path = str(map_save_path)

        # ── state ──────────────────────────────────────────────────────────
        self._state      = State.INIT
        self._prev_state = State.AWAIT

        # pending navigation goal (NAVIGATING / DOCKING)
        self._goal: PoseStamped | None = None
        self._wp_retries = 0
        self._dock_goal_sent = False

        # timing
        self._inspect_t0: float | None   = None
        self._recovery_t0: float | None  = None
        self._person_clear_t0: float | None = None

        # ── safety flags ───────────────────────────────────────────────────
        self._cliff  = {k: False for k in ('front_left', 'front_right', 'rear_left', 'rear_right')}
        self._bumper = {'left': False, 'right': False}
        self._person_near = False

        # ── odometry / battery ─────────────────────────────────────────────
        self._pos:      tuple[float, float] | None = None
        self._prev_pos: tuple[float, float] | None = None
        self._battery   = BATTERY_FULL
        self._last_tick = self.get_clock().now()

        # ── frontier exploration ────────────────────────────────────────────
        self._map:               OccupancyGrid | None = None
        self._frontier_goal:     PoseStamped | None   = None
        self._visited_frontiers: list[tuple[float, float]] = []
        self._explore_done       = False

        # ── Nav2 ───────────────────────────────────────────────────────────
        self._nav = BasicNavigator('roamview_mission_nav')
        self._nav_ready = False
        threading.Thread(target=self._wait_for_nav, daemon=True).start()

        # ── subscriptions ──────────────────────────────────────────────────
        for k in self._cliff:
            self.create_subscription(
                LaserScan, f'cliff/{k}',
                lambda m, k=k: self._on_cliff(k, m), qos_profile_sensor_data)
        for side in self._bumper:
            self.create_subscription(
                Contacts, f'bumper/{side}',
                lambda m, s=side: self._on_bump(s, m), 10)
        self.create_subscription(OccupancyGrid, '/map',        self._on_map,    1)
        self.create_subscription(Odometry,      '/odom',       self._on_odom,   10)
        self.create_subscription(String,    '/mission/command', self._on_cmd,   10)
        self.create_subscription(PoseStamped,'/mission/goal',   self._on_goal,  10)
        self.create_subscription(Bool,       '/people_near',    self._on_people, 10)

        # ── publishers ─────────────────────────────────────────────────────
        self._vel_pub    = self.create_publisher(TwistStamped, vel_topic,         10)
        self._state_pub  = self.create_publisher(String,       '/mission/state',  10)
        self._status_pub = self.create_publisher(String,       '/mission/status', 10)

        # ── 10 Hz control loop ─────────────────────────────────────────────
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'RoamView mission controller ready  '
            f'(vel_topic={vel_topic}, slam_mode={slam_mode})')

    # ──────────────────────────────────────────────────────────────────────────
    # Nav2 startup
    # ──────────────────────────────────────────────────────────────────────────

    def _wait_for_nav(self):
        localizer = 'slam_toolbox' if self._slam_mode else 'amcl'
        self.get_logger().info(f'waiting for Nav2 (localizer={localizer})...')
        self._nav.waitUntilNav2Active(localizer=localizer)
        self._nav_ready = True
        self.get_logger().info('Nav2 is ready')

    # ──────────────────────────────────────────────────────────────────────────
    # Sensor callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _on_cliff(self, name: str, msg: LaserScan):
        valid = [r for r in msg.ranges if math.isfinite(r)]
        r = min(valid) if valid else float('inf')
        self._cliff[name] = r > CLIFF_THRESHOLD

    def _on_bump(self, side: str, msg: Contacts):
        self._bumper[side] = bool(msg.contacts)

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pos = (p.x, p.y)

    def _on_people(self, msg: Bool):
        self._person_near = msg.data

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'command received: {cmd!r}')
        if cmd == 'explore':
            self._explore_done = False
            self._go(State.EXPLORE)
        elif cmd == 'dock':
            self._nav.cancelTask()
            self._dock_goal_sent = False
            self._go(State.DOCKING)
        elif cmd == 'cancel':
            self._nav.cancelTask()
            self._goal = None
            self._go(State.AWAIT)
        elif cmd == 'save_map':
            self._save_map()
        else:
            self.get_logger().warn(f'unknown command: {cmd!r}')

    def _on_goal(self, msg: PoseStamped):
        self._goal = msg
        self._wp_retries = 0
        self._go(State.NAVIGATING)
        self._nav.goToPose(self._goal)
        self.get_logger().info(
            f'new goal → ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})')

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _go(self, new_state: State):
        self.get_logger().info(f'{self._state.name} → {new_state.name}')
        self._state = new_state

    def _status(self, text: str):
        self._state_pub.publish(String(data=self._state.name))
        self._status_pub.publish(String(data=text))

    def _safety_triggered(self) -> str | None:
        """Return a reason string if any safety sensor is active, else None."""
        reasons = []
        if self._cliff['front_left'] or self._cliff['front_right']:
            reasons.append('front cliff')
        if self._cliff['rear_left'] or self._cliff['rear_right']:
            reasons.append('rear cliff')
        if any(self._bumper.values()):
            reasons.append('bumper')
        return ', '.join(reasons) if reasons else None

    def _send_vel(self, linear: float = 0.0, angular: float = 0.0):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x  = linear
        m.twist.angular.z = angular
        self._vel_pub.publish(m)

    def _save_map(self):
        try:
            subprocess.Popen(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self._map_save_path])
            self.get_logger().info(f'map save started → {self._map_save_path}')
        except Exception as e:
            self.get_logger().error(f'map save failed: {e}')

    def _drain_battery(self, dt: float):
        self._battery -= BATTERY_S_DRAIN * dt
        if self._pos and self._prev_pos:
            d = math.hypot(self._pos[0] - self._prev_pos[0], self._pos[1] - self._prev_pos[1])
            self._battery -= BATTERY_M_DRAIN * d
        self._battery  = max(0.0, self._battery)
        self._prev_pos = self._pos

    # ──────────────────────────────────────────────────────────────────────────
    # Frontier exploration (pure Python, no numpy required)
    # ──────────────────────────────────────────────────────────────────────────

    def _find_frontiers(self) -> list[tuple[float, float]]:
        """Return world (x, y) centres of unknown-space frontier clusters."""
        if self._map is None:
            return []
        data = self._map.data
        w = self._map.info.width
        h = self._map.info.height
        res = self._map.info.resolution
        ox  = self._map.info.origin.position.x
        oy  = self._map.info.origin.position.y

        # mark frontier cells: free (0) adjacent to unknown (−1)
        frontier_set: set[tuple[int, int]] = set()
        for row in range(1, h - 1):
            for col in range(1, w - 1):
                if data[row * w + col] != 0:
                    continue
                for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                    if data[(row + dr) * w + (col + dc)] == -1:
                        frontier_set.add((col, row))
                        break

        # cluster connected frontier cells (4-connected flood fill)
        visited: set[tuple[int, int]] = set()
        clusters: list[tuple[float, float]] = []
        for seed in frontier_set:
            if seed in visited:
                continue
            cluster: list[tuple[int, int]] = []
            stack = [seed]
            while stack:
                cell = stack.pop()
                if cell in visited:
                    continue
                visited.add(cell)
                if cell in frontier_set:
                    cluster.append(cell)
                    cx, cy = cell
                    for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
                        nb = (cx + dc, cy + dr)
                        if nb not in visited:
                            stack.append(nb)
            if len(cluster) < FRONTIER_MIN_CELLS:
                continue
            # world-frame centroid
            fx = ox + (sum(c[0] for c in cluster) / len(cluster) + 0.5) * res
            fy = oy + (sum(c[1] for c in cluster) / len(cluster) + 0.5) * res
            # skip frontiers too close to ones we already visited
            if not any(math.hypot(fx - vx, fy - vy) < FRONTIER_REVISIT
                       for vx, vy in self._visited_frontiers):
                clusters.append((fx, fy))

        return clusters

    def _nearest_frontier(self) -> tuple[float, float] | None:
        frontiers = self._find_frontiers()
        if not frontiers or self._pos is None:
            return None
        return min(frontiers, key=lambda f: math.hypot(f[0] - self._pos[0], f[1] - self._pos[1]))

    # ──────────────────────────────────────────────────────────────────────────
    # Main 10 Hz loop
    # ──────────────────────────────────────────────────────────────────────────

    def _tick(self):
        now = self.get_clock().now()
        dt  = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        self._drain_battery(dt)

        # ── global: safety interrupt (skip for states that handle it themselves)
        if self._state not in (State.INIT, State.RECOVERY, State.DOCKED):
            reason = self._safety_triggered()
            if reason:
                self.get_logger().warn(f'safety event ({reason}) → RECOVERY')
                self._prev_state = self._state
                self._nav.cancelTask()
                self._recovery_t0 = time.monotonic()
                self._go(State.RECOVERY)
                return

        # ── global: low battery (skip when already heading to or at dock)
        if self._state not in (State.DOCKING, State.DOCKED, State.INIT, State.RECOVERY):
            if self._battery <= BATTERY_DOCK_THR:
                self.get_logger().warn(
                    f'battery {self._battery:.0f}% ≤ {BATTERY_DOCK_THR}% → DOCKING')
                self._nav.cancelTask()
                self._dock_goal_sent = False
                self._go(State.DOCKING)
                return

        # ── global: person near → pause navigation
        if self._state == State.NAVIGATING:
            if self._person_near:
                if self._person_clear_t0 is None:
                    self.get_logger().info('person nearby — pausing navigation')
                    self._nav.cancelTask()
                    self._send_vel(0.0, 0.0)
                    self._person_clear_t0 = -1.0  # sentinel: person is still here
                self._status(f'[PAUSED] person nearby | battery {self._battery:.0f}%')
                return
            else:
                if self._person_clear_t0 is not None:
                    if self._person_clear_t0 < 0.0:
                        # person just left; start the clear-wait timer
                        self._person_clear_t0 = time.monotonic()
                    elif time.monotonic() - self._person_clear_t0 >= PERSON_CLEAR_WAIT:
                        self.get_logger().info('path clear — resuming')
                        self._person_clear_t0 = None
                        if self._goal:
                            self._nav.goToPose(self._goal)
                    else:
                        self._status(f'[WAIT] path just cleared | battery {self._battery:.0f}%')
                        return

        # ── dispatch to per-state handler
        {
            State.INIT:       self._tick_init,
            State.EXPLORE:    self._tick_explore,
            State.AWAIT:      self._tick_await,
            State.NAVIGATING: self._tick_navigating,
            State.INSPECTING: self._tick_inspecting,
            State.RECOVERY:   self._tick_recovery,
            State.DOCKING:    self._tick_docking,
            State.DOCKED:     self._tick_docked,
        }[self._state]()

    # ──────────────────────────────────────────────────────────────────────────
    # Per-state handlers
    # ──────────────────────────────────────────────────────────────────────────

    def _tick_init(self):
        self._status('waiting for Nav2...')
        if self._nav_ready:
            self.get_logger().info('Nav2 ready')
            self._go(State.EXPLORE if self._slam_mode else State.AWAIT)

    def _tick_explore(self):
        if self._explore_done:
            self.get_logger().info('no more frontiers — exploration complete')
            self._save_map()
            self._go(State.AWAIT)
            return

        # if there is no active nav goal (or the last one finished), pick the next frontier
        if self._frontier_goal is None or self._nav.isTaskComplete():
            if self._frontier_goal is not None:
                # record the frontier we just tried (succeeded or not)
                self._visited_frontiers.append(
                    (self._frontier_goal.pose.position.x,
                     self._frontier_goal.pose.position.y))
                self._frontier_goal = None

            nxt = self._nearest_frontier()
            if nxt is None:
                self._explore_done = True
                return
            self._frontier_goal = _make_pose(self._nav, nxt[0], nxt[1], 0.0)
            self._nav.goToPose(self._frontier_goal)
            self.get_logger().info(
                f'frontier goal ({nxt[0]:.1f}, {nxt[1]:.1f}) '
                f'| {len(self._visited_frontiers)} visited')

        self._status(
            f'[EXPLORE] frontiers visited: {len(self._visited_frontiers)} '
            f'| battery {self._battery:.0f}%')

    def _tick_await(self):
        self._status(
            f'[AWAIT] send /mission/goal or /mission/command '
            f'| battery {self._battery:.0f}%')

    def _tick_navigating(self):
        if self._goal is None:
            self._go(State.AWAIT)
            return

        if self._nav.isTaskComplete():
            result = self._nav.getResult()
            gx, gy = self._goal.pose.position.x, self._goal.pose.position.y

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'reached ({gx:.1f}, {gy:.1f}) → INSPECTING')
                self._inspect_t0 = time.monotonic()
                self._go(State.INSPECTING)

            elif result == TaskResult.CANCELED:
                self._go(State.AWAIT)

            else:  # FAILED
                self._wp_retries += 1
                if self._wp_retries <= MAX_WP_RETRIES:
                    self.get_logger().warn(
                        f'Nav2 failed for ({gx:.1f}, {gy:.1f}), '
                        f'retry {self._wp_retries}/{MAX_WP_RETRIES}')
                    self._nav.goToPose(self._goal)
                else:
                    self.get_logger().error(
                        f'goal ({gx:.1f}, {gy:.1f}) unreachable after '
                        f'{MAX_WP_RETRIES} retries — skipping')
                    self._goal = None
                    self._go(State.AWAIT)
        else:
            gx, gy = self._goal.pose.position.x, self._goal.pose.position.y
            self._status(
                f'[NAVIGATING] → ({gx:.1f}, {gy:.1f}) retry {self._wp_retries} '
                f'| battery {self._battery:.0f}%')

    def _tick_inspecting(self):
        elapsed   = time.monotonic() - self._inspect_t0
        remaining = max(0.0, INSPECT_DURATION - elapsed)
        self._send_vel(0.0, 0.0)   # hold still
        self._status(f'[INSPECTING] {remaining:.1f}s remaining | battery {self._battery:.0f}%')

        if elapsed >= INSPECT_DURATION:
            if self._goal:
                self.get_logger().info(
                    f'inspection done at ({self._goal.pose.position.x:.1f}, '
                    f'{self._goal.pose.position.y:.1f})')
            self._goal = None
            self._go(State.AWAIT)

    def _tick_recovery(self):
        elapsed     = time.monotonic() - self._recovery_t0
        backup_time = BACKUP_DIST / abs(BACKUP_SPEED)

        if elapsed < backup_time:
            self._send_vel(BACKUP_SPEED, 0.0)
            self._status('[RECOVERY] backing up...')
            return

        # back-up done — stop and resume previous state
        self._send_vel(0.0, 0.0)
        self.get_logger().info(f'recovery done → {self._prev_state.name}')

        if self._prev_state == State.NAVIGATING and self._goal:
            self._nav.goToPose(self._goal)
        elif self._prev_state == State.EXPLORE and self._frontier_goal:
            self._nav.goToPose(self._frontier_goal)

        resume = self._prev_state if self._prev_state not in (State.RECOVERY, State.INIT) \
            else State.AWAIT
        self._go(resume)

    def _tick_docking(self):
        if not self._dock_goal_sent:
            dock = _make_pose(self._nav, *DOCK_POSE)
            self._nav.goToPose(dock)
            self._dock_goal_sent = True
            self.get_logger().info(f'heading to dock at {DOCK_POSE[:2]}')

        if self._nav.isTaskComplete():
            result = self._nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('docked successfully → DOCKED')
                self._dock_goal_sent = False
                self._go(State.DOCKED)
            else:
                self.get_logger().warn('dock nav failed — retrying')
                self._dock_goal_sent = False   # will retry next tick

        self._status(f'[DOCKING] heading to dock | battery {self._battery:.0f}%')

    def _tick_docked(self):
        self._battery = min(BATTERY_FULL, self._battery + BATTERY_CHARGE)
        self._status(f'[DOCKED] charging {self._battery:.0f}%')
        if self._battery >= BATTERY_FULL * 0.95:
            self.get_logger().info('battery full → AWAIT')
            self._go(State.AWAIT)


# ── entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    mission = RoamViewMission()
    executor = MultiThreadedExecutor()
    executor.add_node(mission)
    executor.add_node(mission._nav)   # BasicNavigator is its own Node; needs its own spin
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        mission.destroy_node()
        mission._nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
