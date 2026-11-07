# RoamView robot CAD

This folder holds the mechanical design of the RoamView robot in SOLIDWORKS 2026, in MMGS units. The model is parametric: every sketch is fully defined, and every design dimension comes from one file, `Admin/RV_globals.txt`. Python scripts in `Admin/build_scripts/` build the whole model from that file, including the parts, assemblies, drawings, BOM and exports.

> [!NOTE]
> The CAD files are stored with Git LFS (see `.gitattributes`). Run `git lfs install` once on your machine before you clone or pull, or you will get small pointer files instead of the parts.

## Folders

```
Admin/                    RV_globals.txt and build_scripts/
Layout/                   RV-000 master layout (skeleton), RV-001 reference dock
Assemblies/               RV-1xx top level and modules
3DPrinted/                printed parts (PETG, bumper TPU)
SheetMetal_Weldments/     Weldments/ (2020 frame, tower), SheetMetal/
COTS/                     Motors, Wheels_Casters, Sensors, Compute, Electronics
Drawings/                 .SLDDRW drawings, PDF/, BOM (.csv, .xlsx)
Exports/                  STEP, STL, DXF
Simulation/               validation report, interference, mass, FEA, motion, sensor sight lines
```

Part numbers follow `RV-<module><item>`. Each part carries the custom properties `PartNo`, `Description`, `Source` (LAYOUT, REFERENCE, COTS, PRINT, SHEET, WELD or ENVELOPE) and `Revision` (A). The BOM groups parts by `Source`.

## Changing a dimension

Edit `Admin/RV_globals.txt`. Values are in ROS terms: x forward, y left, z up, with the origin on the floor under the drive axle. Parts are linked to that file, so opening a part or assembly and rebuilding picks up the change (Tools > Equations shows the link). You can also rebuild everything from the scripts, as described below.

Positions on the negative side of an axis are written as `abs(...)` in the equations. If you move something across the centreline, for example `POST_X` from −40 to +40, re-run the build script instead of only editing the value.

A global may only reference globals defined above it in the file. SolidWorks evaluates a linked file in stored order, and a part only picks up globals added after it was made once it is re-linked with `update_from_globals.py`.

## Building the model

Run these with SolidWorks open, from `Admin/build_scripts/`, using a Python that has pywin32:

| Step | Script | Output |
|---|---|---|
| 1 | `build_000_master_layout.py` | RV-000 master layout |
| 2 | `build_001_reference_dock.py` | RV-001 reference dock |
| 3 | `build_500_cots.py` | bought parts and envelopes (`COTS`) |
| 4 | `build_400_weldments.py` | 2020 frame and sensor tower |
| 5 | `build_430_sheetmetal.py` | sheet-metal parts |
| 6 | `build_300_printed.py` | 3D-printed parts |
| 7 | `set_masses.py` | realistic mass on every part |
| 8 | `build_100_assemblies.py` | modules, RV-100 top level, RV-190 docking check |
| 9 | `validate.py interference` / `sweeps` / `massprops`, `hand_calcs.py 8.63 -2.1 147.3` | checks (see `Simulation`) |
| 10 | `export_all.py`, `drawings.py`, `bom.py` | exports, drawings, BOM |

To rebuild one part, pass its number, for example `python build_300_printed.py 307` or `python drawings.py RV-307`. After editing the globals, `python update_from_globals.py "<folder>/*.SLDPRT"` re-links, rebuilds and re-checks the linked parts. Run it after moving the folder as well, because the link to the globals file is stored as an absolute path. Assemblies and drawings find their files by relative path, so they keep working as long as the whole folder moves together.

The scripts check their own work. A build fails unless every sketch, read back from the reopened file, reports fully constrained, and every save reopens the file and requires zero feature errors. Points are given in ROS millimetres and converted with each sketch's own transform. Plane offsets, extrude directions and frame axes are verified by reading the geometry back.

## Axes and frames

| SolidWorks | ROS (REP-103) | Meaning |
|---|---|---|
| +X | +y | robot left |
| +Y | +z | up |
| +Z | +x | forward |

The SolidWorks Front view looks at the robot's front. Use the Left view for the side profile, with the front to the right.

The master layout, RV-000, holds the planes, axes and sketches the rest of the model is built on:

- Planes: `PL_Floor`, `PL_Axle`, `PL_CasterMount`, `PL_Deck`, `PL_Scan`, `PL_Lens`, `PL_WheelL/R`, `PL_CasterF/R`, `PL_Lidar`, `PL_Posts`.
- Axes: `AX_Fwd`, `AX_Left`, `AX_Up`, `AX_Wheel`, `AX_CasterF_Swivel`, `AX_CasterR_Swivel`, `AX_Lidar_Spin`.
- ROS frames (coordinate systems): `base_footprint`, `base_link`, `wheel_left_joint`, `wheel_right_joint`, `caster_front_swivel`, `caster_rear_swivel`, `laser`, and `oak_link`, pitched `OAK_TILT` down.
- Module frames: one `IF_*` coordinate system for each module position.

| Sketch | Contents |
|---|---|
| `SK_Side_Chassis` | ground, body box, bumper, charge pad, E-stop height |
| `SK_Side_Running` | drive wheel, casters, caster mount height |
| `SK_Side_Tower` | 2020 post, rear strut, head profile with `SCREEN_ANG` |
| `SK_Side_Sensors` | OAK lens, optical axis (5° down), depth and RGB FOV rays, LiDAR spin axis and scan plane, cliff beams, dock tag |
| `SK_Top_Footprint` | body, wheels, posts, caster swivels, LiDAR, E-stop, 4 cliff sensors, charge pads |
| `SK_Top_Zones` | battery, power PCB enclosure, motor driver, compute bay |
| `SK_Front_Section` | body section, wheels on the ground, posts, head |

Modules mate to these frames and never to each other's geometry. To swap a component, give the replacement the same frame names.

The reference dock, RV-001, has its origin at the robot's `base_footprint` when docked. Its AprilTag face sits at `LENS_X + DOCK_TAG_DIST`, the tag is `DOCK_TAG_SIZE` square at `DOCK_TAG_Z`, and the contacts sit under `CONTACT_X/Y` up to `CONTACT_Z`.

## Parts

### Bought parts and envelopes (`COTS`)

These are simplified bodies with real interfaces. Each part's origin is its mounting point, marked by an `IF_Mount` frame, and some parts add working frames such as `IF_Shaft`, `oak_rgb_lens`, `laser` and `range`. The `DimSource` property on each part says where its numbers came from.

| Part | Status | Interface |
|---|---|---|
| RV-501 Pololu 37D 70:1 gearmotor with encoder | datasheet; hole clocking approx | 6×M3 on Ø31; shaft Ø6 offset 7, 22 out |
| RV-502 goBILDA 1309 Sonic Hub | OD approx | Ø6 bore, 4×M4 on a 16 mm square |
| RV-503 goBILDA Wasteland wheel Ø144 × 52 (3616-0014-0144) | vendor drawing | 4×M4 on a 16 mm square, deep recess toward the motor |
| RV-504 goBILDA swivel caster 3621-0001-0001 | vendor STEP, `COTS/vendor/3621-0001-0001 assembly.STEP` | M8 × 15 stud through the carriage (nyloc nut) or into a captive nut in the rear riser; hex face 67.0 above the floor (`CASTER_H`), wheel Ø52.6 × 23.1, trail 21.9 |
| RV-511 OAK-D Pro W | datasheet; VESA height and USB approx | VESA 2×M4 75 apart; USB-C receptacle in the bottom face, `IF_USB` frame at its mouth |
| RV-512 LiDAR STL-19P | datasheet; hole x ±1 | 3×Ø2.5; laser plane 27.2 above the mount face |
| RV-513 VL53L4CD ToF (Adafruit 5396) | holes and connector side approx | 25.5×17.7 board, `range` frame along +z |
| RV-514 Luxonis OAK Y-adapter (DM6010) | vendor STEP, `COTS/vendor/y-adapter_20211129.stp` | plugs into the OAK at `IF_USB`; keep-outs for 2 right-angle USB-C plugs |
| RV-521 Raspberry Pi 5 + active cooler | datasheet; heights approx | 4×Ø2.7 on 58×49 |
| RV-522 ESP32-S 38-pin expansion board + WROOM-32E dev board | board approx | 77×54, 4×M4 |
| RV-531 DFR0601 motor driver | datasheet | 50×50×12.5, 4×Ø3 on 45×45 |
| RV-532 power PCB envelope | follows `PCB_*` globals | reserved volume |
| RV-541 4S battery envelope | follows `BAT_*` globals | XT60 at the front (blind-mate end) |
| RV-551 Excel Cell ECS-22RE1RM2Q-020Q E-stop (DigiKey 5916-ECS-22RE1RM2Q-020Q-ND) | datasheet, `COTS/vendor/ECS-22RE1RM2Q-020Q.pdf`; collar split approx | Ø22.3 cutout in the deck; contact unit 35.7×50.8 hangs 47.4 below |
| RV-552 Touch Display 2 (7") | thickness and holes approx | 189.32×120.24 |
| RV-553 Omron D2F-01L-D bumper switch (DigiKey SW1054-ND) | vendor STEP, `COTS/vendor/D2F-01L-D.STEP`, and datasheet | 2 × M2 holes 6.5 apart; lever free 10, operates at 6.8 ± 1.5 above the hole line |
| RV-554 Lee Spring LC 032E 05 S compression spring, used twice | leespring.com datasheet; a swept helix at its installed length, ground end flats not modelled | 9.14 OD, 0.81 wire, 19.05 free, 1.751 N/mm, solid 4.7 |

### Frame and tower (`SheetMetal_Weldments/Weldments`)

RV-410, the chassis frame, is a weldment of 17 2020 members kept as separate bodies:
- a bottom ring (z 36.5 to 56.5) and a top ring (z 158 to 178), with rails at x ±198 and y ±140;
- 4 corner uprights;
- 2 mid cross members (z 71.5 to 91.5) that carry the drawer guides and the cliff cradles;
- 3 top cross members under the tower posts, the strut feet and the LiDAR.

The members are simplified to 20×20 with a Ø4.2 bore and no T-slots. Cut lengths are 4 × 416, 9 × 260 and 4 × 101.5 mm.

RV-420, the sensor tower, has 2 posts on 5 mm foot plates, a brace at `BRACE_Z` and 2 rear struts. Cut lengths are 2 × 215, 1 × 100, and about 108 mm for each of the 2 struts.

### Sheet metal (`SheetMetal_Weldments/SheetMetal`, 5052-H32)

- RV-430 deck: 2 mm, with a 21 × 17 grid of M4 holes at 20 mm and 3 cable ports.
- RV-431 floor pan: 1.5 mm, with Ø84 holes for the caster sweeps.
- RV-432 drawer tray: 1.5 mm U with 2 bends.
- RV-433 drawer guide: 1.5 mm L, used on both sides.
- RV-434 motor bracket: 3 mm L with an R1.5 inside bend. The face plate sits inside the wheel's Ø76 recess, and the leg bolts onto the bottom side rail.
- RV-435 charge pad, in brass.

### 3D-printed parts (`3DPrinted`, PETG, bumper in TPU 95A)

Heat-set insert holes are M2.5 Ø3.6 × 4.5, M3 Ø4.0 × 5.7 and M4 Ø5.6 × 8.1.

The head is RV-301, a 3 mm shell open at the front. RV-302 is the bezel that closes it (snap tabs not modelled), and RV-303 is the 5° wedge that tilts the OAK camera down. RV-304 is the LiDAR pedestal; its height is `LIDAR_MOUNT_Z − DECK_Z`.

At the front, RV-305 is the bumper, with two microswitch pockets and two windows for the cliff sensors. RV-306 holds the charge pads and has a pocket for the dock-detect magnet.

RV-307 is the cliff-sensor cradle, used four times. It clamps the end skin to the outer T-slot of the mid cross member with two M4 bolts. The VL53L4CD board lies flush on a face tilted `CLIFF_ANG` from vertical. Its STEMMA plugs sit in side pockets, and the cable drops to a hole in the skin below the cross member. The front pair sit in the bumper windows, 2 mm behind the bumper face.

The rest are RV-308 (sprung front caster housing), RV-309 (its carriage) and RV-310 (rigid rear caster riser). They are followed by RV-311 (compute tray), RV-312 (motor driver mount), RV-313 (side skin with a wheel notch), RV-314 (front skin with a light slot), RV-315 (rear skin with the drawer opening) and RV-316 (drawer face with a handle). The front and rear skins carry the bolt and cable holes for the cliff cradles.

The two front caster springs sit in Ø10 seats cut `SPRING_SEAT` (6 mm) into the underside of the RV-308 top plate and on Ø7 pilots on top of the RV-309 carriage, which grew to 44 × 48 so the coils have full support. That gives an installed length of 16 mm, 5.3 N of preload per spring, and 1.3 mm short of solid at the full 10 mm travel. The springs are modelled rigid at their rest length, so the +10 mm sweep ignores their overlap with the lifted carriage; the hand calcs check the compression instead.

RV-317 is the bumper-switch bracket, used twice. One M4 clamps it through the front skin into the mid cross member slot, and the Omron switch sits on its tab with the lever toward the bumper. The M2 holes are slots, so the switch can slide 1.5 mm either way along the lever travel. The bumper floats `BUMPER_GAP` (4 mm) proud of the skin on its two guide pins. At rest the pocket floor just touches the lever at its free position; pushed home, it brings the lever to 6 mm above the hole line, inside the operating window and short of bottoming. At assembly, push the bumper in about 2 mm and slide each switch until it clicks.

### Assemblies (`Assemblies`)

The modules are:
- RV-110 drive module, used on both sides
- RV-120 sprung front caster (housing, carriage, caster and two RV-554 springs)
- RV-125 rigid rear caster
- RV-130 battery drawer
- RV-140 sensor tower
- RV-150 compute bay
- RV-160 driver bay
- RV-170 LiDAR module
- RV-180 cliff sensor, used 4 times
- RV-182 bumper switch (bracket + Omron D2F), used twice
- RV-185 charge pads

RV-100 is the top-level assembly. It fixes RV-000, which is hidden and excluded from the BOM, and coordinate-mates each module's `IF_Mount` to its frame in the layout. RV-190 places the robot on the reference dock to check docking.

The wheels spin and the casters swivel freely. The sprung caster travels up to 10 mm and the drawer slides up to 250 mm, each limited by a limit mate.

## Design decisions

Change these in the globals if you disagree.

| # | Decision | Why |
|---|---|---|
| 1 | Wheel: goBILDA Wasteland Ø144 × 52, deep recess toward the motor (`WHEEL_WEB_OFF` 6) | the wheel centre sits only 9 mm outboard of the gearbox face, so the shaft bending load is about 3.5× lower than with the recess facing out |
| 2 | The goBILDA "16 mm pattern" is a 16 mm square (±8, ±8) | vendor drawing |
| 3 | `CASTER_X` 136.5 | the goBILDA caster's 21.9 mm trail sweeps a 49.6 mm radius at full spring travel; this leaves 1.9 mm to the bottom front rail |
| 4 | `PCB_X` 33.5, `COMPUTE_X` −49, compute tray 66 × 180 with the Pi and ESP32 turned 90° | between the two caster sweeps there is 173.8 mm; the 95 mm PCB envelope stays as it was and the shorter tray takes the cut |
| 5 | Rear struts end at `STRUT_Z` = SCAN_Z − 20 | keeps them out of the LiDAR plane, so only the 2 posts shadow the scan |
| 6 | `HEAD_Z0` = POST_TOP | the head sits on the posts, fixed with M5 into the post bores |
| 7 | `CONTACT_Z` 30, with the pads in a holder under the pan | clears the pan and the front rail; the dock follows |
| 8 | `LENS_X` = 45.6 (head front + bezel + wedge + OAK) | the real lens position; `update_from_globals.py` warns if the stack-up drifts |
| 9 | Side skins at y = ±150, inside the wheels, with the 340 mm deck overhanging as fenders | 4.5 mm between skin and wheel |
| 10 | Front caster sprung by a carriage on 2 guide pins and 2 Lee Spring LC 032E 05 S springs (10 mm travel, 10.7 N total preload); rear caster rigid | the rear tripod keeps the charge-pad height set; braking at 1 m/s² compresses the springs about 3 mm and leaves about 7 mm for bumps |
| 11 | ESP32 on an "ESP32-S Expansion Board 38-Pin" (77 × 54, 4 × M4) | a header carrier, so no breadboard on the robot |
| 12 | Motor bracket inside bend R1.5 | clears the gearbox; confirm with the fabricator |
| 13 | Rear skin opening and drawer face sized to `BAT_H` + 4 | the battery clears the skin through the whole drawer stroke |
| 14 | Cliff sensors in RV-307 cradles on the end skins, with the die at `CLIFF_X` 218 / `CLIFF_Y` ±105 / `CLIFF_Z` 72 looking `CLIFF_ANG` 50° from vertical; the same cradle front and rear | the floor spot lands 140 mm ahead of the caster tyre, against 17 mm with the sensors under the floor pan |
| 15 | `CLIFF_Y` 105 | keeps the outer cradle T-nut clear of the corner upright |
| 16 | Bumper microswitches at ±55 and guide pins at ±150 | leaves room for the cliff-sensor windows |
| 17 | The Y-adapter plugs straight into the OAK's USB-C port and hangs below the camera, outside the bezel | that is how Luxonis says to fit it; the old head-floor spot had it on the end of a cable |
| 18 | E-stop panel-mounted through the deck at `ESTOP_X` −100 / `ESTOP_Y` −100, top at `ESTOP_TOP` = `DECK_Z` + 37.5 | the cutout swallows one grid hole and leaves 6.7 mm webs, and the contact block below clears the top ring, the tower cross members and the drawer at every stroke; the old spot put the block into the side rail |
| 19 | The caster stud goes straight through the carriage and into the rear riser, without goBILDA's 2814 plate mount | the hex face then sits at 67.0, 1 mm from the old `CASTER_H`, so the frame and dock heights stay put; the plate mount would add 5.4 mm |
| 20 | Floor-pan caster holes sized from the caster globals (Ø102 now) | the wheel rises through the pan when it swivels |
| 21 | Bumper floats 4 mm (`BUMPER_GAP`) on its guide pins; Omron D2F-01L-D switches at `BSW_Y` ±55 on slotted RV-317 brackets | the D2F operating point varies ±1.5 mm between parts, so a fixed mount cannot promise a click inside a short travel; the slot lets each switch be set at assembly |

## Validation

The full report is `Simulation/Validation_Report.md`. To re-run the checks yourself, follow `Simulation/HOW_TO_RUN_SIMULATIONS.md`.

| Check | Result |
|---|---|
| Mass, centre of mass | 8.67 kg; COM (−2.0, 0.1, 145.9) mm |
| Static tip-over (≥ 20°) | 33.2° with the drawer closed, 24.3° with it 250 mm out (casters further in, with a longer trail) |
| Interference at rest | none that matter. There are 16 zero-volume face contacts, 10 of them between the Y-adapter's own vendor bodies. Two more are 0.3 mm³, where a 0.07 mm lip at the root of the vendor caster stud sits just proud of its hex face. The check runs in two passes, RV-100 with the tower hidden and then RV-140 alone, because one full pass with the vendor STEP bodies crashes SolidWorks |
| Collision sweeps | all 28 positions clear (drawer stroke under the E-stop, caster swivel, spring travel); the above-deck tower is hidden during the sweeps |
| Strength (hand calcs and 3D FEA, 2 g bump) | all pass; lowest FS 3.1, on the motor bracket in FEA; the longer caster housing peaks at 5.1 MPa (FS 5.8) |
| Drivetrain | 55 % of continuous torque on a 5° ramp while accelerating |
| Sensor sight lines | the tower posts block 17.3° of the LiDAR scan; the OAK sees none of the robot (Y-adapter included) and sees the floor from 452 mm; the cliff cones are clear, and their floor spots land 164 mm past the caster tyre |

This SolidWorks install has only SimulationXpress and no Motion add-in. Hand calculations stand in for motion analysis, and Python runs the 3D FEA (`fea.py`) and the sight-line check (`fov_check.py`).

## Outputs

`export_all.py` writes STEP files for every part and for RV-100 and RV-190, binary STL files in mm for the printed parts, and DXF flat patterns for the sheet metal, all into `Exports`.

`drawings.py` makes 24 part drawings on ISO A3 sheets in third-angle projection, plus the RV-100 assembly drawing on A2 with 4 views, a BOM table and balloons. Each drawing is saved as `.SLDDRW` with a PDF copy in `Drawings/PDF`.

`bom.py` writes `Drawings/RV-100_BOM.csv`, grouped into purchased, manufactured, reserved and hardware items; the hardware quantities are estimates. Its Purpose column says in a sentence or two what each part does and why it was chosen. That text lives in `bom_purpose.py`, and `bom.py` warns about any part that has none. `bom_xlsx.py` turns the CSV into `RV-100_BOM.xlsx` and needs openpyxl.

## Open items

- The ESP32 cliff stop has to brake at 1 m/s² or harder. With the casters moved in, the cliff sensors see 164 mm ahead of the caster tyre, and stopping from 0.5 m/s with 50 ms latency needs 150 mm.
- On a 15 mm threshold the sprung front caster needs about 22 N of push against 58 N of traction, now that the preload is 10.7 N. That passes, though a 2-inch caster still meets the edge at a steep 65°.
- The rear cliff cradles stick out 13 mm from the rear skin, and there is no bumper in front of them.
- The bumper guide pins (shoulder bolts, 4 mm travel, light return springs) are in the BOM as hardware but are not modelled.
- The goBILDA caster's load rating is not on its product page. Each caster carries little (the drive wheels take about 42 N each), but confirm the rating with goBILDA.
- Board hole positions marked APPROX in `DimSource` need measuring before printing.
- On the part drawings, the hole-pattern dimensions are crowded and need tidying by hand before release. The weldment cut lists show quantities only (the cut lengths are listed above), and the A2 sheet-format border did not load on the assembly drawing.
- `Simulation/Validation_Report.md` lists further recommendations on the caster spring, door thresholds, battery voltage, a drawer latch, gearbox side load, bumper mass and the bracket bend.

### Parts still to choose

The bumper switch is the Omron D2F-01L-D. DigiKey lists a 26-week factory lead time but holds about 300 in stock. If that runs out, the Omron SSG-01L1, the Panasonic AV4 with gold contacts, or an E-Switch SS lever switch with gold contacts would do, with a new bracket tab.

The front caster springs are the Lee Spring LC 032E 05 S (about $8 each in small quantities). Lee Spring offers a STEP behind a download form; the model here is built from the datasheet instead, and the only thing it leaves out is the ground end flats.
