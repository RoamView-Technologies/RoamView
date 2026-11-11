# RoamView robot CAD

The mechanical design of the RoamView robot, modelled in SOLIDWORKS 2026 (MMGS units). This folder holds every part, assembly, drawing and export, plus the scripts that generate them.

The CAD files are stored with Git LFS. Run `git lfs install` once on your machine before you clone or pull. Without it you get small pointer files instead of parts.

## Folder structure

```
RobotCAD/
├── Admin/
│   ├── RV_globals.txt          every design dimension, in one file
│   └── build_scripts/          Python scripts that build, check and export the model
├── Layout/
│   ├── RV-000_MasterLayout.SLDPRT
│   └── RV-001_ReferenceDock.SLDPRT
├── Assemblies/                 RV-100 robot and its modules
├── 3DPrinted/                  RV-301 to RV-317, printed parts
├── SheetMetal_Weldments/
│   ├── Weldments/              RV-410 chassis frame, RV-420 sensor tower
│   └── SheetMetal/             RV-430 to RV-435
├── COTS/                       bought parts (RV-5xx)
│   ├── Motors/  Wheels_Casters/  Sensors/  Compute/  Electronics/  Hardware/
│   └── vendor/                 vendor STEP files and datasheets
├── Drawings/                   .SLDDRW files, PDF/ copies, BOM
├── Exports/                    STEP/, STL/, DXF/
└── Simulation/                 checks and results (gitignored, local only)
```

## Part numbering

Every part and assembly is named `RV-<number>_<Name>`. The hundreds digit says what kind of item it is.

| Range | What it is |
|---|---|
| RV-000, RV-001 | master layout and reference dock |
| RV-100 to RV-190 | assemblies. RV-100 is the whole robot |
| RV-300s | 3D printed parts (PETG, bumper in TPU) |
| RV-400s | 2020 frame, sensor tower and sheet metal |
| RV-500s | purchased parts, plus placeholder envelopes for the power PCB (RV-532) and battery (RV-541) |
| HW-01 to HW-21 | fasteners and small hardware. Listed in the BOM, not modelled |

Each part carries four custom properties: `PartNo`, `Description`, `Source` and `Revision`. `Source` is one of LAYOUT, REFERENCE, COTS, PRINT, SHEET, WELD or ENVELOPE, and the BOM groups parts by it.

## How the files connect

**Globals.** `Admin/RV_globals.txt` holds every dimension (wheel size, caster position, deck height, sensor angles and so on). Parts link to it through SOLIDWORKS equations (Tools > Equations). Change a value there, rebuild, and every linked part follows. A global can only refer to globals above it in the file.

**Master layout.** `Layout/RV-000_MasterLayout.SLDPRT` is a skeleton part with no solid geometry. It holds the planes, axes, coordinate systems and layout sketches the rest of the model is built on. Dimensions in it come from the globals.

**Interface frames.** Each module has an `IF_*` coordinate system in the layout. Each module's own parts have an `IF_Mount` frame at their mounting point. RV-100 mates the two frames together. Modules never mate to each other's geometry, so a component can be swapped for another that uses the same frame names.

**Coordinates.** The model uses ROS axes (REP-103): x forward, y left, z up, origin on the floor under the drive axle. SOLIDWORKS stores these as +Z forward, +X left, +Y up. Named ROS frames (`base_footprint`, `base_link`, `laser`, `oak_link` and others) live in the layout.

**Paths.** Parts store an absolute link to the globals file. If you move the folder, re-link with `update_from_globals.py`. Assemblies and drawings use relative paths, so they survive a move as long as the folder moves as a whole.

## What is in each folder

### Layout
- RV-000 master layout. Planes (`PL_Floor`, `PL_Axle`, `PL_Deck`, `PL_Scan` and others), axes, ROS frames and the side, top and front layout sketches (`SK_Side_Chassis`, `SK_Top_Footprint` and others).
- RV-001 reference dock. The charging dock with the AprilTag and charge contacts. Its origin is the robot's `base_footprint` when docked.

### Assemblies
RV-100 is the top-level robot. It fixes RV-000 (hidden, excluded from the BOM) and mates each module to its frame.

| Assembly | Contents |
|---|---|
| RV-110 | drive module, used on both sides |
| RV-120 | sprung front caster: housing, carriage, caster, two springs |
| RV-125 | rigid rear caster |
| RV-130 | battery drawer |
| RV-140 | sensor tower |
| RV-150 | compute bay |
| RV-160 | motor driver bay |
| RV-170 | LiDAR module |
| RV-180 | cliff sensor, used four times |
| RV-182 | bumper switch, used twice |
| RV-185 | charge pads |
| RV-190 | robot placed on the reference dock, for the docking check |

`RV-100_RoamView.urdf.json` sits beside the top assembly. It is the data export for the ROS URDF.

### 3DPrinted
| Part | What it is |
|---|---|
| RV-301 to RV-303 | head shell, head bezel, OAK camera wedge |
| RV-304 | LiDAR pedestal |
| RV-305 | bumper (TPU) |
| RV-306 | charge pad holder |
| RV-307 | cliff sensor mount, used four times |
| RV-308, RV-309, RV-310 | front caster housing, front caster carriage, rear caster riser |
| RV-311, RV-312 | compute tray, motor driver mount |
| RV-313 to RV-315 | side, front and rear skins |
| RV-316 | drawer face |
| RV-317 | bumper switch bracket, used twice |

### SheetMetal_Weldments
- `Weldments/`: RV-410 chassis frame (2020 aluminium extrusion, members kept as separate bodies) and RV-420 sensor tower.
- `SheetMetal/`, 5052-H32: RV-430 deck plate, RV-431 floor pan, RV-432 battery drawer tray, RV-433 drawer guide, RV-434 motor bracket, RV-435 charge pad (brass).

### COTS
Simplified models of bought parts, with real mounting interfaces. The `DimSource` property on each part records where its dimensions came from (datasheet, vendor STEP or approximate).

| Folder | Parts |
|---|---|
| `Motors/` | RV-501 Pololu 37D gearmotor |
| `Wheels_Casters/` | RV-502 goBILDA hub, RV-503 goBILDA wheel, RV-504 goBILDA caster |
| `Sensors/` | RV-511 OAK-D Pro W, RV-512 LiDAR STL-19P, RV-513 ToF sensor, RV-514 OAK Y-adapter |
| `Compute/` | RV-521 Raspberry Pi 5, RV-522 ESP32 board, RV-552 touch display |
| `Electronics/` | RV-531 motor driver, RV-532 power PCB envelope, RV-541 battery envelope, RV-551 E-stop, RV-553 bumper microswitch |
| `Hardware/` | RV-554 compression spring, used twice |
| `vendor/` | vendor STEP files and datasheets the models are based on |

### Drawings
One `.SLDDRW` per manufactured part (RV-301 to RV-317, RV-410, RV-420, RV-430 to RV-435) on ISO A3, third-angle projection. `RV-100_RoamView.SLDDRW` is the assembly drawing on A2, with a BOM table and balloons. `PDF/` holds a PDF of each drawing.

The BOM is `RV-100_BOM.csv`, with an `.xlsx` copy. It groups items into purchased, manufactured, reserved and hardware. `RV-100_Electrical_BOM.md` lists the electrical parts with their supply, current and connector data.

### Exports
- `STEP/`: every part, RV-100 and RV-190.
- `STL/`: the printed parts, in mm.
- `DXF/`: flat patterns of the sheet-metal parts.

### Simulation
Not tracked in Git. It holds the validation report, interference and collision results, mass properties, FEA, drivetrain and sensor sight-line results, MATLAB models and presentation renders. `HOW_TO_RUN_SIMULATIONS.md` explains how to re-run them.

## The build scripts

`Admin/build_scripts/` rebuilds the model from `RV_globals.txt`. Run the scripts with SolidWorks open, using a Python with pywin32 (`requirements.txt` lists the packages).

| Group | Scripts | Output |
|---|---|---|
| Build, in this order | `build_000_master_layout.py`, `build_001_reference_dock.py`, `build_500_cots.py`, `build_400_weldments.py`, `build_430_sheetmetal.py`, `build_300_printed.py`, `set_masses.py`, `build_100_assemblies.py` | the parts and assemblies above |
| Update | `update_from_globals.py` | re-links, rebuilds and re-checks parts after a globals change |
| Documents | `drawings.py`, `bom.py`, `bom_xlsx.py`, `bom_purpose.py` | drawings, BOM and the Purpose text for each part |
| Export | `export_all.py`, `export_fov_stl.py`, `export_urdf.py`, `export_views.py` | STEP, STL, DXF, URDF data and review images |
| Checks | `validate.py`, `hand_calcs.py`, `fea.py`, `fov_check.py`, `verify_bodies.py`, `verify_cots.py` | interference, mass, hand calcs, FEA, sensor sight lines |
| Media | `bay_video.py`, `drive_video.py`, `presentation_views.py` | videos and presentation images |
| Shared code | `rv_sw.py`, `cots.py`, `asm.py` | SolidWorks API helpers, the COTS part builder, assembly helpers |

To rebuild one item, pass its number: `python build_300_printed.py 307` or `python drawings.py RV-307`.
