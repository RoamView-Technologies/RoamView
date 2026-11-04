# Running the RoamView simulations yourself

There are two ways to run each check:
- **A. SolidWorks SimulationXpress:** native SolidWorks, GUI, one part at a time.
- **B. The Python scripts** that produced the reference results: repeatable, and they also cover the sensor sight-line check.

All values below match what the scripts use, so your results can be compared directly. Units: mm, N, MPa.

---

## A. FEA in SolidWorks SimulationXpress

Open a part, then go to **Tools ▸ Xpress Products ▸ SimulationXpress**. Work through the wizard tabs in this order: Options (set units to SI/mm), Fixtures, Loads, Material, Run, Results.

### A1. Motor bracket: `SheetMetal_Weldments/SheetMetal/RV-434_MotorBracket.SLDPRT`
| Step | Set |
|---|---|
| Material | Aluminium alloy 5052-H32 (or *1060* if not listed, then compare stress to 193 MPa yield) |
| Fixture 1 | **Fixed geometry**: the 2 M4 hole faces in the horizontal leg (they bolt to the side rail) |
| Fixture 2 | Not available in Xpress (rail contact under the leg). Leave it out, which is slightly conservative |
| Load | **Force 84.4 N, downward (−Y)**, split evenly over the **6 small hole faces** (Ø3.4) in the vertical plate = wheel load at a 2 g bump |
| Mesh | Default, or slide toward *Fine* |
| Pass if | Max von Mises ≤ 193 / 3 = **64 MPa** |

Reference result (`fea.py`, same setup plus the rail contact): **peak 62.2 MPa (FS 3.1)**, 99.5 % of the part below 36.7 MPa, max deflection 0.37 mm. Without the rail-contact fixture, expect somewhat higher stress near the bolt holes.

> Xpress can't apply the 633 N·mm overhang moment separately. Doing so would make the result slightly more severe; the hand calc and `fea.py` include it.

### A2. Front caster housing: `3DPrinted/RV-308_FrontCasterHousing.SLDPRT`
| Step | Set |
|---|---|
| Material | Custom: **PETG** E = 2100 MPa, ν = 0.38, yield 30 MPa (across layers). In Xpress, pick *ABS* and compare against 30 MPa |
| Fixture | **Fixed geometry**: the outer face of the back wall (the face that bolts to the front mid cross member) |
| Load | **Force 30 N upward (+Y)** on the underside of the top plate (where the carriage bottoms out on a 2 g bump) |
| Pass if | Max von Mises ≤ 30 / 2 = **15 MPa** |

Hand calculation: about 1.2 MPa (FS 26).

### A3. Drawer guide: `SheetMetal_Weldments/SheetMetal/RV-433_DrawerGuide.SLDPRT`
| Step | Set |
|---|---|
| Material | 5052-H32 (yield 193 MPa) |
| Fixture | **Fixed geometry**: the 2 M4 hole faces at each end (they sit on the mid cross members) |
| Load | **Force 11.8 N downward** on the top face of the horizontal leg (half the drawer + battery weight at 2 g) |
| Pass if | Max von Mises ≤ 64 MPa; deflection small (< 1 mm) |

Hand calculation: 11.7 MPa, mid-span deflection 0.17 mm.

Other parts worth trying in Xpress with the same approach: the LiDAR pedestal (RV-304), head shell (RV-301) and compute tray (RV-311).

---

## B. The Python scripts (same checks, repeatable)

Scripts live in `Admin/build_scripts`. One-time setup in a terminal (short path matters for VTK on Windows):

```bash
python -m venv C:\rv-venv
```
```bash
C:\rv-venv\Scripts\python -m pip install gmsh scikit-fem meshio pyamg trimesh rtree matplotlib pyvista
```

### B1. FEA (`fea.py`): bracket, caster housing, drawer guide
Uses the STEP files in `Exports/STEP`. Re-export them after any design change (`python export_all.py` with SolidWorks open).
```bash
C:\rv-venv\Scripts\python fea.py bracket
```
Replace `bracket` with `housing`, `guide` or `all`.

- **Outputs:** `Simulation/FEA/fea_<case>.txt` (loads, fixtures, stress, safety factor) and `fea_<case>.png` (von Mises on the deformed shape).
- **Mesh size:** set by the second argument of `mesh_step(...)` in each case (mm). Smaller means more accurate and slower. The guide and housing take several minutes each.

### B2. 3D sensor sight lines (`fov_check.py`)
First export one STL per component in the assembly frame. With SolidWorks open, run this with the normal Python:
```bash
python export_fov_stl.py
```
It sets the STL options for you (one file per component, not shifted to positive space, mm) and writes to `Simulation/FOV/parts/`.

Then run:
```bash
C:\rv-venv\Scripts\python fov_check.py
```
Outputs: `Simulation/FOV/fov_report.txt`, `lidar_occlusion.png`, `oak_occlusion.png`, `cliff_spots.png`.

What it checks:

| Sensor | Check | Result |
|---|---|---|
| LiDAR | 360° scan-plane self-occlusion | blocked 17.3° (both tower posts) |
| OAK | depth / RGB fields of view, and the near edge of visible floor | no self-occlusion; floor visible from 452 mm ahead of the lens |
| Cliff sensors | 18° cones, floor spots, stopping look-ahead | no rays blocked; floor spot 140 mm past the caster (see below) |
| Dock tag | line of sight when docked | clear |

### B3. Other checks (need SolidWorks running)
These scripts use the SolidWorks API through `pywin32`. Run them with the normal Python, not `C:\rv-venv`:

| Check | Command | Output |
|---|---|---|
| Interference at rest | `python validate.py interference` | `Simulation/Interference/rest_interference.txt` |
| Collision sweeps | `python validate.py sweeps` | `collision_sweeps.txt` (drawer stroke, caster swivel, spring travel) |
| Mass, COM, tip-over | `python validate.py massprops` | `Simulation/MassProps/mass_com_tipover.txt` |
| Strength and drivetrain hand calcs | `python hand_calcs.py 8.67 -1.1 146.7` | `Simulation/FEA` and `Simulation/Motion` reports |

---

## Cliff sensors: look-ahead (B2)
The four VL53L4CD sensors sit in RV-307 cradles on the end skins; the front pair look out through the bumper windows. Each sensor is 72 mm above the floor, looking 50° from vertical. The floor spot is **140 mm** ahead of the caster tyre, front and rear.

Stopping check, assuming a 1 m/s² hard stop and 50 ms latency:

| Case | Look-ahead needed | Result |
|---|---|---|
| Forward 0.3 m/s | 60 mm | OK, +80 mm margin |
| Reverse 0.3 m/s | 60 mm | OK, +80 mm margin |
| Forward 0.5 m/s | 150 mm | short by 10 mm |

To cover 0.5 m/s:
- set the ESP32 cliff stop to at least 1.2 m/s², or
- cap forward speed at 0.45 m/s unless the OAK depth stream confirms the floor. It sees the floor from 452 mm ahead.
