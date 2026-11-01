> [!WARNING]
> This markdown is written with Claude's help.

# roamview_sim

RoamView Rev B in Gazebo with ROS 2 Lyrical: the robot model, a two-room test world with a stairwell drop, a
130 × 100 m warehouse club, the Gazebo ↔ ROS 2 bridge, Nav2 with a map and keepout zones, a patrol, and tools to
check that a simulation is healthy.

**The guide is [`hardware/ros/README.md`](../../README.md)**: setup (native WSL or Docker), how the parts fit
together, every launch argument and topic, test results and troubleshooting. This page is only a map of the
package.

```bash
ros2 launch roamview_sim sim.launch.py                                   # test world
ros2 launch roamview_sim sim.launch.py world:=warehouse_club people:=8   # the store, with shoppers
ros2 run roamview_sim sim_check.py                                       # is it healthy? (second terminal)
```

| Folder | What is in it |
|---|---|
| `launch/` | `sim.launch.py` (Gazebo, robot, bridge, EKF, RViz) and `nav.launch.py` (Nav2 for the warehouse club) |
| `urdf/` | The robot: `roamview.urdf.xacro` (links, joints, masses), `roamview_gazebo.xacro` (plugins, sensors), `roamview_body.xacro` (shapes generated from CAD) |
| `worlds/` | `roamview_test.sdf` and two ready-made warehouse club worlds for plain `gz sim` |
| `models/warehouse_club/` | The store's meshes (glTF), textures, `model.sdf` and `manifest.json` |
| `maps/` | Nav2 map of the store and its keepout mask |
| `config/` | `bridge.yaml` (which topics cross between Gazebo and ROS), `ekf.yaml`, Nav2 settings, patrol waypoints |
| `rviz/` | The RViz view |
| `scripts/` | Nodes and tools you run with `ros2 run`: `cliff_guard.py`, `brake_test.py`, `patrol.py`, `club_world.py`, `sim_check.py`, `watch_memory.sh` |
| `tools/` | Generators for the robot shapes, the test world and the warehouse club (run by hand, not installed) |
| `docs/` | Store plan and renders used by the guide |
