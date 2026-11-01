
# What my robot do
Autonomous and Teleoperated Robot for Remote Facility Inspection and Virtual Tours
RoamView is a dual-mode robotic platform that gives remote stakeholders accurate, live, and interactive access to any physical space without requiring anyone to be there in person.
In autonomous mode, the robot can navigate a space on a set schedule, performing inspections and site scans without human guidance. In remote or steered mode, a user connects through a web dashboard, takes control of the robot, explores the space in real time, and communicates directly with anyone on-site. Every session, regardless of mode, generates a timestamped 3D map stored in the cloud and accessible through the client dashboard.
Key Features
- Autonomous scheduled inspections and site scans
- Remote-controlled live walkthroughs through a web interface
- Real-time HD video streaming and two-way communication
- Post-session 3D map generation with real-world accurate measurements
- Progress comparison and historical site tracking
- Interactive client dashboards with annotation tools and report generation


## Components List 

### Computer and sensing
- OAK-D Pro W together with OAK Y adapter
- D500 LiDAR 
- Raspiberry Pi 5 
- Cliff sensors VL53L4CD x4. [STEMMAQT VL53L4CD TIME OF FLIGHT](https://www.digikey.ca/short/vbh8220v)
- ESP32 board carrier 

### Power, battery, and protection
- K2 relay + RC receiver ??? Together these two make a wireless kill switch
- K1 Relay, power switch to the motor. [Panasonic CB1AF-P-12V automotive relay](https://www.digikey.ca/short/13wqvhnz)
- 3 × INA260
- 2 × 5 V bucks
- 3 × ideal diodes
- 4700 µF capacitor.
- Wiring

### Drivetrain
- 1x [70:1 Metal Gearmotor 37Dx70L mm 12V with 64 CPR Encoder (Helical Pinion)](https://www.pololu.com/product/4754)
- 1x [DUAL-CHANNEL DC MOTOR DRIVER-12A](https://www.digikey.ca/short/55dnc85p)
- 1x [goBILDA 1309 Series Sonic Hub 6mm Bore](https://ca.robotshop.com/products/gobilda-1309-series-sonic-hub-6mm-bore)
- 2x Wheels on the sides
- 2x caster wheels at front and back 


### 3D Printed Components
- 


### Docking system
**How docking works**
1. **Stage:** Nav2 drives the robot to a staging pose about 0.7 m in front of the dock. The dock’s position is saved in the map.
2. **Detect:** The OAK detects the dock’s tag on the camera itself, using DepthAI’s AprilTag node, so the Pi does no image processing. Use the global-shutter mono cameras with the OAK-D Pro’s IR flood light, which works in the dark and while moving.
3. **Approach:** Nav2’s docking server (opennav_docking) steers onto the tag. It slows to about 0.05 m/s for the last 150 mm.
4. **Confirm:** When the contacts touch, the robot sees charger voltage on its charge input and stops. If it doesn’t within a few seconds, it backs off 0.3 m and retries, up to 3 times, then reports a fault.
5. **Undock:** Reverse 0.3 m slowly, then turn. The dock area is known to be safe, so the robot doesn’t need rear cliff sensors for this.


**Three things to check:**
1. **Close-range tag detection:** At the docked position, the lens is only about 0.25 m from the tag. Test that the fixed-focus OAK still decodes it there. If not, add a small second tag or finish the last few centimetres on odometry.
2. **Pad height:** It has to hold steady with or without suspension. That’s one more reason to settle the suspension question.
3. **Dock power when idle:** The dock’s pins are bare. I recommend switching the charger output on only when a robot is present (a magnet in the robot plus a reed switch or Hall sensor in the dock). Otherwise, a coin or tool across the pins shorts the charger.


## Therefore, the followings are design rules: 
- What if the dimensions change?
- Fully define every sketch.
- Interconnection of components. i need flexibility to change components around if i want. Just like how the car is interconnected
- Power, battery, and protection >> these components will go a PCB which is to be designed later. leave space for where to put the PCB board and assume it will having a casing for protection
- keep in mind of the cable that will be connected to raspi pi and the MCU
- MMGS standard.
- Placements of components
    - Battery should be easily accessible for exchange
    - 
- Create a structured folder tree separating custom 3D-printed parts, sheet metal/weldments, purchased off-the-shelf (COTS) hardware (motors, bearings, sensors), and assembly files.
- Use top-level layout sketches or master sketches to define overall wheel base, center of gravity, and sensor field-of-view before building individual subassemblies. Ref: https://blog.cadsoftwaredirect.com/build-a-autonomous-robot-car-in-solidworks/
- **Chassis & Frame Modeling:**
	- Use Weldments for structural aluminum extrusions or steel roll cages.
	- Use Sheet Metal features (base flange, edge flange, hems) for bent-metal electronics trays and battery enclosures.
- **Component integration:** Download or model standard electronic boards (e.g., ESP32, microcontrollers, motor drivers), LiDAR/ultrasonic units, and gearmotors.
- Design custom snap-fit or threaded-insert mounting brackets (using brass heat-set inserts) to house your electronics safely.
- **Kinematics & coordinate frames:** Remember that this is an autonomous navigating robot, so set up explicit coordinate systems and reference geometries at each joint rotation axis.
- Mate the drivetrain, wheels, sensors, and structural panels using standard, advanced, and mechanical mates.
- Run Interference Detection and Collision Detection to ensure wires, servos, and rotating links do not collide during movement.


**Simulation & Validation**
- Motion Studies: Use SOLIDWORKS Motion to simulate wheel-terrain interactions, actuator speeds, or arm trajectories under gravity and payload forces.
- Center of Mass (COM) Analysis: Evaluate your mass properties early to make sure your autonomous mobile robot (AMR) or rover does not tip over on inclines.
- Static Simulation: Run finite element analysis (FEA) on custom suspension arms, brackets, or load-bearing chassis components to verify safety factors.


**Manufacturing & Exporting**
- 3D Printing / CNC Prep: Export parts directly to STL, STEP, or DXF/DWG flat patterns for laser cutting or 3D printing.
- BOM & Drawings: Generate automated Bills of Materials (BOM) and production drawings to track purchased hardware versus manufactured items.


**Useful Resources & Guides**
- Explore community robot models and reference assemblies via the GrabCAD SOLIDWORKS Robot Library.
- Check out structured workflows like the SOLIDWORKS Robotics Design Guide for step-by-step chassis and leg-linkage challenges.