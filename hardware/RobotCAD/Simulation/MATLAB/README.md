# RoamView MATLAB simulations

These scripts cover what SolidWorks here cannot do: a motion study of the whole robot, the motor and battery behaviour, a sweep of the front caster spring, and a second FEA to check `fea.py`. They need MATLAB R2024a or later with Simulink, Simscape, Simscape Multibody and the PDE Toolbox. No C compiler is needed.

## Running them

Open MATLAB in this folder and type `run_all`. From a terminal the same thing is

```bash
matlab -batch "run_all"
```

A full run takes about five minutes. Each piece also runs on its own: `run_dynamics`, `run_drivetrain_electrical`, `sweep_caster_spring` and `fea_pde` (or `fea_pde("bracket")`, `"housing"`, `"guide"`). Reports and plots go to `results/`.

The scripts read the robot from the RobotCAD folder: `Admin/RV_globals.txt`, `Simulation/MassProps/mass_table.csv`, `Simulation/MassProps/mass_inertia.csv` and the parts in `Exports/STEP`. If the folder is somewhere unusual, set the environment variable `RV_ROBOTCAD` to its path. After a design change, refresh the inputs first. With SolidWorks open, `python validate.py massprops` rewrites the mass, centre of mass and inertia, and `python export_all.py` re-exports the STEP files. Then rerun `run_all`; it rebuilds the Simulink model from the new numbers.

## Watching a run

The batch scripts run without graphics. To see the robot move, open MATLAB in this folder and type, for example,

```matlab
watch_case("threshold")              % current spring: the front caster stalls at the 15 mm edge
watch_case("threshold_new_spring")   % 0.5 N/mm and 6 N per spring: it crosses
watch_case list                      % every case
```

Mechanics Explorer opens and animates the run with the SolidWorks parts, and a figure of wheel speed, contact loads and spring compression opens beside it. The playback bar at the bottom replays the run, the speed box gives slow motion, and the toolbar has front, side and top views. The side skins are drawn see-through so the caster carriage is visible. Any `rv_tunables` option can follow the case name and overrides it, such as `watch_case("threshold", caster_d=0.075)` for the 3-inch caster or `watch_case("ramp", ramp_deg=10)`.

To watch the whole validation suite instead, type `run_all(animate=true)`. Every multibody run plays in Mechanics Explorer at real time, and the command window names each one as it starts (which script, stop time, ramp, threshold, spring and drawer). That takes about ten minutes. `run_all(animate=true, pace=0.5)` plays at half speed, and `sweep=false` leaves out the 36 runs of the spring sweep. Reports and plots are the same as a normal run. On a ramp the floor is drawn level and gravity is tilted instead, so the robot looks like it is on flat ground while it climbs.

The part meshes come from `Simulation/FOV/parts`, the per-part STLs that `export_fov_stl.py` writes. `make_rv_visuals.m` groups them by the body they ride on and saves thinned copies in `visuals/` the first time the model is built; after a design change, re-export the STLs and run `make_rv_visuals(rv_params(), true)` before rebuilding. The meshes carry no mass, so they don't change any result. If the STLs are missing, the model falls back to boxes and balls, and `build_rv_dynamics(rv_params(), "simple")` asks for that look on purpose.

## Driving the test course

`run_arena` puts the robot in a two-room course and lets it drive itself through, the way it would move around a home. The course has a 5 degree ramp up to a 50 mm platform and back down, a 12 mm cable cover, a slalom through three table legs, a sofa and a bin to pass close by, and a 0.8 m doorway with a 15 mm threshold into the second room, where it parks. A pure pursuit controller (`rv_pursuit.m`) steers it along the route at up to 0.4 m/s, with the same motors, duty cap, spring and contacts as the validation runs.

```matlab
run_arena                                  % current spring, animated in Mechanics Explorer
run_arena(spring="recommended")            % 0.5 N/mm with 6 N preload per spring
run_arena(video=true)                      % also writes results/matlab_arena_design.mp4
run_arena(animate=false, video=true)       % no live window, just the report and the video
```

It reports whether the robot finishes, and where it gets stuck if it doesn't. It also reports any collision of its footprint with walls or furniture, any scraping of its underside on the ramp, cable cover or threshold, how close it comes to the obstacles, and the tilt, wheel and caster loads, motor current and battery energy section by section (`results/matlab_arena_<spring>.txt` and `.png`). The collision box covers the robot's plan extents from the CAD, tyres and charge pads included, from the lowest chassis point (the charge pad holder, 30 mm up) to the deck. The video is drawn from the simulated motion with the SolidWorks parts and a chase camera, so it doesn't need Mechanics Explorer and can go straight into a presentation.

The course takes about ten times real time to compute (some 20 minutes for the 110 s route) because of its forty contact pairs. To change it, edit `rv_arena_layout.m`, where every object is one line with its size, position, colour and role and the route is a list of waypoints, then run `run_arena(rebuild=true)`.

## What each file does

`rv_params.m` gathers every number the simulations use: globals, masses, the inertia from SolidWorks, the Pololu motor data, the driver limit, the battery and the spring. The battery capacity (5 Ah) and the electronics load (18 W) are placeholders until those parts are chosen, and the runtime estimate inherits that.

`build_rv_dynamics.m` writes `rv_dynamics.slx`. Don't edit the model by hand, because the next build overwrites it; change the script instead. The chassis rides on a 6-DOF joint and carries the measured mass and inertia, minus the wheels and casters, which are separate bodies. Each drive wheel turns on a revolute joint and touches the floor through a contact force with friction. The front caster sits on a sprung carriage (prismatic joint with preload, end stops and damping) and the rear caster is rigid. Both casters are modelled as balls at the point where the swivel wheel meets the floor, so the trail is set per run and swivel flutter is left out. Each motor is an averaged PWM model referred to the gearbox output, driven by a PI wheel speed loop with the 71 % duty cap. The floor can be tilted, and a door threshold block can be placed anywhere.

`rv_tunables.m` holds the per-run settings (ramp angle, threshold position, drawer pulled out, spring, duty cap, battery voltage, caster size and travel). `rv_sim.m` runs one case and returns the signals on a 1 ms grid; a fourth argument of `true` animates it. `watch_case.m` wraps the validation cases for viewing, and `make_rv_visuals.m` prepares the part meshes. To try your own case:

```matlab
P  = rv_params();
RV = rv_tunables(P, ramp_deg=8, drawer_out=0.15);
R  = rv_sim(RV, [0 0 0; 0.5 0 0; 1.1 0.3 0; 99 0.3 0], 4);   % [time speed yaw_rate]
plot(R.t, R.Tload)
```

`run_dynamics.m` drives the robot through the validation cases and compares each with `hand_calcs_drivetrain.txt`. `run_drivetrain_electrical.m` covers stall current, the threshold climb at full and empty battery, and runtime. `sweep_caster_spring.m` tries a grid of spring rates and preloads. `fea_pde.m` repeats the three `fea.py` cases in the PDE Toolbox.

## Results

From the run on 5 November 2026 (8.68 kg, COM 1.9 mm behind the axle and 146 mm up). The full numbers are in `results/`.

Standing still, the preloaded front spring puts 14.9 N on the front caster and 12.1 N on the rear one, leaving 29.1 N on each drive wheel. The hand calc assumed 42 N per wheel because it ignored the preload, and that difference matters for traction.

Driving matches the hand calc closely. Cruising at 0.5 m/s takes 0.06 N·m per motor (hand calc 0.07), accelerating at 0.5 m/s² takes 0.27 (0.24), and the 5° ramp takes 0.33 cruising and 0.54 accelerating (0.36 and 0.54). Braking at 1 m/s² stops in 155 mm from the command; the extra 30 mm over the hand calc is the speed loop's lag. While braking, the load comes off the rigid rear caster (12 N down to 3 N) rather than compressing the front spring, which moves less than 1 mm.

The 15 mm door threshold is the one real problem. With the current spring (1.5 N/mm and 6 N preload, per spring) the robot stalls at a square edge going forwards and backwards; the drive wheels slip at 2.4 N·m. The 3-inch caster and extra carriage travel don't fix it alone. A softer spring does: at 0.4 to 0.5 N/mm with 3 to 6 N preload, the robot crosses both ways. The sweep picks 0.5 N/mm with 6 N preload per spring as the firmest one that crosses both ways, keeps 9 mm of bump travel while braking and keeps the rear caster on the floor. Springs with no preload let the rear caster lift under braking. The edge of the passing region is sensitive (some neighbouring grid points fail one direction), so test the chosen spring on a real threshold.

In a full-reverse emergency stop from 0.5 m/s the robot decelerates at about 5.9 m/s², near the traction limit. The rear caster lifts for about 0.3 s and the nose settles 4.5° onto the front carriage stop; nothing tips over. With the drawer pulled out 250 mm, a full-power launch unloads the drive wheels for about 20 ms as the robot rocks back onto the rear caster, which caps the acceleration at about 1.7 m/s². Turning at 0.5 m/s up to 3 rad/s (1.5 m/s² sideways) keeps at least 21.6 N on the inner wheel. A spin in place with a fast reversal puts at most 0.49 N·m of bending on each gearbox shaft, under the 0.87 N·m of the 2 g bump case.

On the electrical side, the 71 % duty cap holds stall current at 5.5 A per motor at full charge (7.7 A without it), which matches the 12 V rating. At an empty pack the capped stall torque drops to 1.98 N·m and top speed to 0.85 m/s. With the placeholder 5 Ah pack and 18 W of electronics, a mixed mission runs about 3.2 h, and the electronics use 91 % of the energy, so the pack choice depends far more on the compute load than on the motors.

The PDE Toolbox FEA agrees with `fea.py`. The caster housing peaks at 3.1 MPa (fea.py 5.1) and the drawer guide at 14.8 MPa with 0.13 mm deflection (19.5 MPa and 0.12 mm). For the motor bracket, holding the whole leg underside gives 39.8 MPa and leaving it free gives 114 MPa at the bolt holes (48.6 MPa for 99.5 % of the part); fea.py's partial seat on the rail, 62.2 MPa, sits between the two as it should. Every part passes its safety factor in the bulk.

## Limits of the model

The casters are balls, so the swivel and its side scrub are not there. That makes the spin-in-place side load on the gearboxes a lower bound. Tyres are rigid spheres with a penalty contact, so there is no tyre deflection or contact patch. The battery is a fixed voltage per run, which is fine for runs of a few seconds. The motor model comes from the 12 V datasheet points (stall and no load), so it will be off by the usual few percent between those points. The PDE Toolbox applies loads and supports to whole CAD faces, so where `fea.py` used a patch of a face the PDE result is bracketed instead of matched; `fea_pde.m` explains each case.
