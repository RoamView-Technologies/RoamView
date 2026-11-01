# hardware

All hardware-side files for RoamView.

- `firmware/` — ESP32 MCU code: motor control, encoder polling, e-stop interlock, UART link to the Pi
- `ros/` — ROS 2 workspace: the RoamView simulation (`src/roamview_sim`: Gazebo, Nav2, RViz) and the Docker dev container (Dockerfile, compose, entrypoint) — see `ros/README.md`
- `docs/` — documentation as needed
- `RobotCAD/` — mechanical CAD of the robot (SOLIDWORKS, parametric, built by Python scripts) — see `RobotCAD/00_Admin/README.md`. The CAD files are stored with Git LFS, so run `git lfs install` once.
