# RoamView robot CAD

The mechanical design of the RoamView robot in SOLIDWORKS (MMGS units).
- **Parametric:** every sketch is fully defined, and every design dimension comes from one file, `00_Admin/RV_globals.txt`.
- **Scripted:** the whole model, from parts to assemblies, drawings, BOM and exports, is built by the Python scripts in `00_Admin/build_scripts/`.

**Git:** the CAD files are stored with Git LFS (`hardware/RobotCAD/.gitattributes`). Run `git lfs install` once on your machine before cloning or pulling.

## Change a dimension

1. Edit `00_Admin/RV_globals.txt`. Values are in ROS terms: x forward, y left, z up, origin on the floor under the drive axle.
2. Open the part or assembly. It is linked to that file, so a rebuild picks up the change (Tools > Equations shows the link).
3. Or rebuild from scratch with the scripts (see "Build the model").

Positions on the negative side of an axis are written as `abs(...)` in the equations. If you move something across the centreline (e.g. `POST_X` from −40 to +40), re-run the build script instead of only editing the value.

**Rule for globals:** a global may only reference globals defined *above* it. SolidWorks evaluates a linked file in stored order, and a part only imports globals added after it was made when it is re-linked (`update_from_globals.py`).

## Build the model

With SolidWorks open, from `00_Admin/build_scripts/` (system Python with pywin32):

| Step | Script | Output |
|---|---|---|
| 1 | `build_000_master_layout.py` | RV-000 master layout |
| 2 | `build_001_reference_dock.py` | RV-001 reference dock |
| 3 | `build_500_cots.py` | bought parts and envelopes (`05_COTS`) |
| 4 | `build_400_weldments.py` | 2020 frame and sensor tower |
| 5 | `build_430_sheetmetal.py` | sheet-metal parts |
| 6 | `build_300_printed.py` | 3D-printed parts |
| 7 | `set_masses.py` | realistic mass on every part |
| 8 | `build_100_assemblies.py` | modules, RV-100 top level, RV-190 docking check |
| 9 | `validate.py interference` / `sweeps` / `massprops`, `hand_calcs.py 8.63 -2.1 147.3` | checks (see `08_Simulation`) |
| 10 | `export_all.py`, `drawings.py`, `bom.py` | exports, drawings, BOM |

Single parts can be rebuilt by number, e.g. `python build_300_printed.py 307` or `python drawings.py RV-307`.
After editing the globals, `python update_from_globals.py "<folder>/*.SLDPRT"` re-links, rebuilds and re-audits the linked parts. The globals link is stored as an absolute path, so run it after moving the folder too. Assemblies and drawings find their files by relative path as long as the whole folder moves together.

How the scripts work:
- **Fully-defined check:** a build fails unless every sketch, read back from the reopened file, reports fully constrained.
- **Clean-file check:** each save reopens the file and requires zero feature errors.
- **Points:** points are given in ROS mm and converted with each sketch's own transform.
- **Directions:** plane offsets, extrude directions and frame axes are verified by reading geometry back.

## Axes

| SolidWorks | ROS (REP-103) | Meaning |
|---|---|---|
| +X | +y | robot left |
| +Y | +z | up |
| +Z | +x | forward |

The SolidWorks **Front** view looks at the robot's front. Use the **Left** view for the side profile (front to the right).

## Folder tree

```
00_Admin/                 this README, RV_globals.txt, build_scripts/
01_Layout/                RV-000 master layout (skeleton), RV-001 reference dock
02_Assemblies/            RV-1xx top level and modules
03_Custom_3DPrinted/      printed parts (PETG, bumper TPU)
04_SheetMetal_Weldments/  Weldments/ (2020 frame, tower), SheetMetal/
05_COTS/                  Motors, Wheels_Casters, Sensors, Compute, Electronics
06_Drawings/              .SLDDRW drawings, PDF/, BOM (.csv, .xlsx)
07_Exports/               STEP, STL, DXF
08_Simulation/            validation report, interference, mass, FEA, motion, sensor sight lines
design_rules_november.md  the design rules this model follows
```

Part numbers follow `RV-<module><item>`. Every part carries the custom properties `PartNo`, `Description`, `Source` (LAYOUT / REFERENCE / COTS / PRINT / SHEET / WELD / ENVELOPE) and `Revision` (A). The BOM groups by `Source`.

## RV-000 master layout

- **Planes:** `PL_Floor`, `PL_Axle`, `PL_CasterMount`, `PL_Deck`, `PL_Scan`, `PL_Lens`, `PL_WheelL/R`, `PL_CasterF/R`, `PL_Lidar`, `PL_Posts`.
- **Axes:** `AX_Fwd`, `AX_Left`, `AX_Up`, `AX_Wheel`, `AX_CasterF_Swivel`, `AX_CasterR_Swivel`, `AX_Lidar_Spin`.
- **Sketches** (all fully defined):

| Sketch | Contents |
|---|---|
| `SK_Side_Chassis` | ground, body box, bumper, charge pad, E-stop height |
| `SK_Side_Running` | drive wheel, casters, caster mount height |
| `SK_Side_Tower` | 2020 post, rear strut, head profile with `SCREEN_ANG` |
| `SK_Side_Sensors` | OAK lens, optical axis (5° down), depth and RGB FOV rays, LiDAR spin axis and scan plane, cliff beams, dock tag |
| `SK_Top_Footprint` | body, wheels, posts, caster swivels, LiDAR, E-stop, 4 cliff sensors, charge pads |
| `SK_Top_Zones` | battery, power PCB enclosure, motor driver, compute bay |
| `SK_Front_Section` | body section, wheels on the ground, posts, head |

- **ROS frames** (coordinate systems): `base_footprint`, `base_link`, `wheel_left_joint`, `wheel_right_joint`, `caster_front_swivel`, `caster_rear_swivel`, `laser`, `oak_link` (pitched `OAK_TILT` down).
- **Module frames:** `IF_*` coordinate systems, one per module position.

Every module mates to these frames; modules never mate to each other's geometry. To swap a component, give the replacement the same frame names.

**RV-001 reference dock:** its origin is the robot's `base_footprint` when docked. The AprilTag face sits at `LENS_X + DOCK_TAG_DIST`, the tag is `DOCK_TAG_SIZE` square at `DOCK_TAG_Z`, and the contacts sit under `CONTACT_X/Y` up to `CONTACT_Z`.

## Parts

### Bought parts and envelopes (`05_COTS`)
Simplified bodies with real interfaces: each part's origin is its mounting interface (`IF_Mount`), plus working frames such as `IF_Shaft`, `oak_rgb_lens`, `laser` and `range`. The `DimSource` property says where the numbers came from.

| Part | Status | Key interface |
|---|---|---|
| RV-501 Pololu 37D 70:1 gearmotor with encoder | datasheet; hole clocking approx | 6×M3 on Ø31; shaft Ø6 offset 7, 22 out |
| RV-502 goBILDA 1309 Sonic Hub | OD approx | Ø6 bore, 4×M4 on a 16 mm square |
| RV-503 goBILDA Wasteland wheel Ø144 × 52 (3616-0014-0144) | vendor drawing | 4×M4 on a 16 mm square, deep recess toward the motor |
| RV-504 2" swivel caster | **model TBD** | 40×40 plate, 4 holes 30×30, height 68, trail 10 |
| RV-511 OAK-D Pro W | datasheet; VESA height and USB approx | VESA 2×M4 75 apart; USB-C keep-out below |
| RV-512 LiDAR STL-19P | datasheet; hole x ±1 | 3×Ø2.5; laser plane 27.2 above the mount face |
| RV-513 VL53L4CD ToF (Adafruit 5396) | holes and connector side approx | 25.5×17.7 board, `range` frame along +z |
| RV-514 OAK Y-adapter | envelope approx | 34×24×10 |
| RV-521 Raspberry Pi 5 + active cooler | datasheet; heights approx | 4×Ø2.7 on 58×49 |
| RV-522 ESP32-S 38-pin expansion board + WROOM-32E dev board | board approx | 77×54, 4×M4 |
| RV-531 DFR0601 motor driver | datasheet | 50×50×12.5, 4×Ø3 on 45×45 |
| RV-532 power PCB envelope | follows `PCB_*` globals | reserved volume |
| RV-541 4S battery envelope | follows `BAT_*` globals | XT60 at the front (blind-mate end) |
| RV-551 E-stop, 22 mm in a 45×45 box | **model TBD** | top at `ESTOP_TOP` |
| RV-552 Touch Display 2 (7") | thickness and holes approx | 189.32×120.24 |

### Weldments (`04_SheetMetal_Weldments/Weldments`)
- **RV-410 chassis frame:** 17 2020 members as separate weldment bodies:
  - bottom ring (z 36.5–56.5) and top ring (z 158–178), rails at x ±198 / y ±140
  - 4 corner uprights
  - 2 mid cross members (z 71.5–91.5) that carry the drawer guides and the cliff cradles
  - 3 top cross members under the tower posts, the strut feet and the LiDAR

  Members are simplified to 20×20 with a Ø4.2 bore and no T-slots. Cut lengths: 4 × 416, 9 × 260, 4 × 101.5 mm.
- **RV-420 sensor tower:** 2 posts on 5 mm foot plates, a brace at `BRACE_Z`, and 2 rear struts. Cut lengths: 2 × 215, 1 × 100, 2 struts ≈ 108 mm.

### Sheet metal (`04_SheetMetal_Weldments/SheetMetal`, 5052-H32)
- **RV-430 deck:** 2 mm, 21 × 17 M4 grid at 20 mm, 3 cable ports.
- **RV-431 floor pan:** 1.5 mm, caster sweep holes Ø84.
- **RV-432 drawer tray:** 1.5 mm U, 2 bends.
- **RV-433 drawer guide:** 1.5 mm L, used left and right.
- **RV-434 motor bracket:** 3 mm L, inside bend R1.5. The face plate sits inside the wheel's Ø76 deep recess, and the leg bolts onto the bottom side rail.
- **RV-435 brass charge pad.**

### Printed (`03_Custom_3DPrinted`, PETG, bumper TPU 95A)
Heat-set insert holes: M2.5 Ø3.6 × 4.5, M3 Ø4.0 × 5.7, M4 Ø5.6 × 8.1.
- **Head and camera:** RV-301 head (3 mm shell, front-open), RV-302 bezel (snap tabs not modelled), RV-303 5° OAK wedge.
- **LiDAR:** RV-304 pedestal, height = `LIDAR_MOUNT_Z − DECK_Z`.
- **Front end:** RV-305 bumper with 2 microswitch pockets and 2 cliff-sensor windows. RV-306 charge-pad holder with a dock-detect magnet pocket.
- **Cliff sensors:** RV-307 cradle (×4).
  - **Mounting:** clamps the end skin to the mid cross member's outer T-slot with 2 × M4.
  - **Board:** the VL53L4CD lies flush on a face tilted `CLIFF_ANG` from vertical. The STEMMA plugs sit in side pockets, and the cable drops to a skin hole below the mid cross member.
  - **Front pair:** they sit in the bumper windows, 2 mm behind the bumper face.
- **Casters:** RV-308 sprung front caster housing, RV-309 carriage, RV-310 rigid rear riser.
- **Electronics:** RV-311 compute tray, RV-312 driver mount.
- **Skins:** RV-313 side (with wheel notch), RV-314 front (light slot), RV-315 rear (drawer opening). The end skins carry the cliff-cradle bolt and cable holes.
- **Drawer:** RV-316 drawer face and handle.

### Assemblies (`02_Assemblies`)
- **Modules:**
  - RV-110 drive module (used L and R)
  - RV-120 sprung front caster, RV-125 rigid rear caster
  - RV-130 battery drawer
  - RV-140 sensor tower
  - RV-150 compute bay, RV-160 driver bay
  - RV-170 LiDAR module
  - RV-180 cliff sensor (×4)
  - RV-185 charge pads
- **Top level:** RV-100 fixes RV-000 (hidden, excluded from the BOM) and coordinate-mates each module's `IF_Mount` to its layout frame. RV-190 docks the robot on the reference dock.
- **Free joints:** wheel spin, caster swivel, sprung caster travel 0–10 mm (limit mate), drawer slide 0–250 mm (limit mate).

## Design decisions (override in the globals if you disagree)
| # | Decision | Why |
|---|---|---|
| 1 | Wheel: goBILDA Wasteland Ø144 × 52, deep recess toward the motor (`WHEEL_WEB_OFF` 6) | the wheel centre sits only 9 mm outboard of the gearbox face, so the shaft bending load is about 3.5× lower than the other way round |
| 2 | The goBILDA "16 mm pattern" is a 16 mm **square** (±8, ±8) | vendor drawing |
| 3 | `CASTER_X` 150 | further out, the swivel sweep hits the bottom front rail |
| 4 | `PCB_X` 62.5, `COMPUTE_X` −65 | clear of the caster swivel sweeps |
| 5 | Rear struts end at `STRUT_Z` = SCAN_Z − 20 | keeps them out of the LiDAR plane; only the 2 posts shadow the scan |
| 6 | `HEAD_Z0` = POST_TOP | the head sits on the posts, M5 into the post bores |
| 7 | `CONTACT_Z` 30; pads in a holder under the pan | clears the pan and front rail; the dock follows |
| 8 | `LENS_X` = 45.6 (head front + bezel + wedge + OAK) | the real lens position; `update_from_globals.py` warns if the stack-up drifts |
| 9 | Side skins at y = ±150, inside the wheels; the 340 mm deck overhangs as fenders | 4.5 mm skin-to-wheel clearance |
| 10 | Front caster sprung by a carriage on 2 guide pins + 2 springs (10 mm); rear caster rigid | keeps the charge-pad height set by the rear tripod |
| 11 | ESP32 on an "ESP32-S Expansion Board 38-Pin" (77 × 54, 4 × M4) | header carrier, no breadboard |
| 12 | Motor bracket inside bend R1.5 | gearbox clearance; confirm with the fabricator |
| 13 | Rear skin opening and drawer face sized to `BAT_H` + 4 | the battery clears the skin through the whole drawer stroke |
| 14 | Cliff sensors in RV-307 cradles on the end skins: die at `CLIFF_X` 218 / `CLIFF_Y` ±105 / `CLIFF_Z` 72, looking `CLIFF_ANG` 50° from vertical; same cradle front and rear | the floor spot lands 140 mm ahead of the caster tyre. Under the floor pan it would only be 17 mm ahead |
| 15 | `CLIFF_Y` 105 | keeps the outer cradle T-nut clear of the corner upright |
| 16 | Bumper microswitches at ±55 and guide pins at ±150 | room for the cliff-sensor windows |

## Validation
Full report: `08_Simulation/Validation_Report.md`. To re-run the checks yourself, see `08_Simulation/HOW_TO_RUN_SIMULATIONS.md`.

| Check | Result |
|---|---|
| Mass, centre of mass | 8.63 kg; COM (−2.1, −0.6, 147.3) mm |
| Static tip-over (≥ 20°) | 36.5° drawer closed, 28.9° drawer out 250 mm |
| Interference at rest | none (6 zero-volume face contacts) |
| Collision sweeps | 28/28 positions clear (drawer stroke, caster swivel, spring travel) |
| Strength (hand calcs + 3D FEA, 2 g bump) | all pass; lowest FS 3.1 (motor bracket, FEA) |
| Drivetrain | 55 % of continuous torque on a 5° ramp while accelerating |
| Sensor sight lines | LiDAR 17.3° blocked by the tower posts; OAK no self-occlusion, floor from 452 mm; cliff cones clear |

Tooling: this SolidWorks install has SimulationXpress only (no Motion add-in). Hand calculations cover FEA and motion, and Python covers 3D FEA (`fea.py`) and sight lines (`fov_check.py`).

## Outputs
- **Exports:** `07_Exports`, from `export_all.py`. Covers STEP (all parts, RV-100, RV-190), STL (printed parts, mm, binary) and DXF flat patterns (sheet metal).
- **Drawings:** `06_Drawings`, from `drawings.py`. 24 part drawings plus the RV-100 assembly drawing (A2, 4 views, BOM table and balloons), each as `.SLDDRW` with a PDF in `06_Drawings/PDF`. ISO A3, third-angle.
- **BOM:** `06_Drawings/RV-100_BOM.csv` and `.xlsx`, from `bom.py`. Grouped purchased / manufactured / reserved / hardware; hardware quantities are estimates.

## Open items
- **Cliff stop at 0.5 m/s:** forward at 0.5 m/s with a 1 m/s² hard stop needs 150 mm of look-ahead and the robot has 140 mm.
  - Fix: set the ESP32 cliff stop to at least 1.2 m/s², or cap speed at 0.45 m/s unless the OAK depth stream confirms the floor.
  - Context: the 140 mm is measured to the caster tyre's leading edge; the wheel only drops about 25 mm later.
- **Rear cradles:** they stick out 13 mm from the rear skin with no bumper in front.
- **Placeholder models:** the caster (RV-504) and E-stop (RV-551) models are still TBD.
- **Approximate interfaces:** board hole positions marked APPROX in `DimSource` need measuring before printing.
- **Drawings:**
  - Hole-pattern dimensions on part drawings are crowded; tidy them by hand before release.
  - Weldment cut lists show quantities only (cut lengths are above).
  - The A2 sheet-format border did not load on the assembly drawing.
- **Recommendations:** see `08_Simulation/Validation_Report.md`. They cover the caster spring, thresholds, battery voltage, drawer latch, gearbox side load, bumper mass and the bracket bend.
