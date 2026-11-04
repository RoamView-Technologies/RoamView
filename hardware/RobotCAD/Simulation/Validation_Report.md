# RoamView validation report

**Result: all checks pass.** Open items and seven recommendations are listed at the end. To re-run any check yourself, see `HOW_TO_RUN_SIMULATIONS.md`.

## Tools and limits
- **In SolidWorks 2026 (through the API):** interference detection, collision sweeps and mass properties.
- **Not available:** this install has only the SimulationXpress engine (no full Simulation API) and no Motion add-in.
- **Substitutes:**
  - Closed-form engineering checks (`build_scripts/hand_calcs.py`), with conservative inputs; every number is printed with its inputs.
  - A 3D FEA of the three most loaded parts (`build_scripts/fea.py`).
  - A ray-traced sensor sight-line check (`build_scripts/fov_check.py`).

## Pass criteria
| Check | Criterion |
|---|---|
| Static tip-over | ≥ 20° in every direction, drawer closed and open |
| Strength | FS ≥ 3 aluminium/steel, FS ≥ 2 printed PETG |
| Clearance | no interference at rest; no collision through the drawer stroke, caster swivel (360°) and spring travel (10 mm) |
| Drivetrain | ≤ continuous motor torque for normal driving; ≤ instantaneous limit for thresholds |
| Cliff sensors | floor spot far enough ahead of the casters to stop before a drop |

## 1. Masses (`MassProps/mass_table.csv`)
| Source | Basis |
|---|---|
| Bought parts | datasheet masses, battery 900 g placeholder |
| 2020 frame and tower | 0.46 kg/m plus hardware allowance |
| Sheet metal | 5052 at 2.68 g/cm³ |
| PETG prints | 1.15 g/cm³ for shells and skins, 0.70 for infilled blocks |
| TPU bumper | 0.73 g/cm³ |

**Robot total: 8.63 kg.** Centre of mass: x −2.1, y −0.6, z 147.3 mm (ROS, floor under the axle).

## 2. Interference at rest (`Interference/rest_interference.txt`)
**0 volume interferences.** The 6 hits listed are 0.0 mm³ face contacts: drawer face/tray, bracket/gearbox face (×2), drawer guides/skins.

## 3. Collision sweeps (`Interference/collision_sweeps.txt`), 28 positions
| Motion | Positions | Result |
|---|---|---|
| Battery drawer stroke | 0 / 100 / 200 / 333 mm | CLEAR |
| Front caster swivel, spring at rest | 0–315° every 45° | CLEAR |
| Front caster swivel, spring +10 mm | 0–315° every 45° | CLEAR |
| Rear caster swivel | 0–315° every 45° | CLEAR |

## 4. Static tip-over (`MassProps/mass_com_tipover.txt`)
The support polygon is the two wheel contacts plus both caster contacts, with each caster's trail turned inward (worst case).

| Case | COM x | Min tip angle | Result |
|---|---|---|---|
| Drawer closed | −2.1 mm | **36.5°** | PASS |
| Drawer out 150 mm | −23.1 mm | 32.1° | PASS |
| Drawer out 250 mm | −37.1 mm | 28.9° | PASS |

Forward tip under braking would need 0.95 g, so braking at 1 m/s² (0.1 g) is safe.

## 5. Structural hand checks (`FEA/hand_calcs_structural.txt`), 2 g bump unless noted
| Item | Stress | FS | Result |
|---|---|---|---|
| Motor bracket leg (3 mm 5052), worst-case pivot at bolt line | 56.9 MPa | 3.4 | PASS |
| Motor bracket leg, pivot at rail edge | 43.5 MPa | 4.4 | PASS |
| 2× M4 rail bolts, prying | 137 N each | 44 | PASS |
| Gearmotor Ø6 output shaft, 15 mm overhang | 59.7 MPa | 5.9 | PASS |
| Tower posts, 1 g lateral on 0.785 kg head stack | 1.6 MPa | 92 | PASS |
| Front caster housing plate (PETG, Z strength) | 1.2 MPa | 26 | PASS |
| Drawer guides (1.5 mm L), span 396 mm | 11.7 MPa, 0.17 mm deflection | 17 | PASS |

### 5a. 3D FEA (`build_scripts/fea.py`, `FEA/fea_*.txt` + PNG)
A linear static analysis on the exported STEP geometry, using quadratic (P2) tetrahedra (gmsh + scikit-fem). Loads and fixtures are the same 2 g cases as above.
- **Peak:** the single highest element. It includes stress spikes at the fixtures, which are a meshing artefact, so it overstates the real stress.
- **99.5 %:** the value 99.5 % of the part stays below; the representative stress level.

| Part | Mesh | Peak | 99.5 % | Max deflection | FS (peak) | Hand calc | Result |
|---|---|---|---|---|---|---|---|
| RV-434 motor bracket (5052, 84.4 N + overhang moment) | 1.2 mm | 62.2 MPa | 36.7 MPa | 0.37 mm | 3.1 | 56.9 MPa | PASS |
| RV-308 caster housing (PETG, 30 N up) | 1.5 mm | 3.3 MPa | 1.1 MPa | 0.11 mm | 9.2 | 1.2 MPa | PASS |
| RV-433 drawer guide (5052, 11.8 N) | 1.2 mm | 19.5 MPa | 4.3 MPa | 0.12 mm | 9.9 | 11.7 MPa, 0.17 mm | PASS |

**How FEA compares with the hand calcs:**
- **Typical stress:** matches or sits below the hand calcs.
- **Drawer-guide deflection:** lower in FEA, because the real L-section with bolted ends is stiffer than the simple-beam model.
- **Peaks:** they sit where the part is held.

## 6. Drivetrain hand checks (`Motion/hand_calcs_drivetrain.txt`)
Motor: Pololu 4754 at 12 V. Stall 2.65 N·m, continuous limit 0.98 N·m, instantaneous limit 2.45 N·m, no-load 150 rpm.

| Case | Torque per motor | % of limit | Current |
|---|---|---|---|
| Flat cruise 0.5 m/s | 0.07 N·m | 7 % continuous | 0.34 A |
| Flat, 0.5 m/s² | 0.24 N·m | 25 % continuous | 0.68 A |
| 5° ramp, 0.3 m/s | 0.36 N·m | 37 % continuous | 0.93 A |
| 5° ramp + 0.5 m/s² | 0.54 N·m | 55 % continuous | 1.28 A |
| 15 mm threshold (drive wheel) | 1.86 N·m | 76 % instantaneous | brief |

- **Top speed:** about 1.0–1.1 m/s under load, well above Nav2's 0.3–0.5 m/s.
- **Threshold at the front caster: MARGINAL.** A 2-inch caster meets a 15 mm edge at 66°, needing 27–34 N of push against about 42–59 N of traction.

## 7. Sensor sight lines (`FOV/fov_report.txt`, PNG plots)
| Sensor | Result |
|---|---|
| LiDAR STL-19P | 17.3° of the 360° scan blocked, by the two tower posts (rear) |
| OAK-D Pro W | 0 % self-occlusion for the depth and RGB fields of view; depth sees the floor from 452 mm ahead of the lens |
| Dock AprilTag | in view and unobstructed when docked, 250–268 mm from the lens |
| Cliff sensors (×4) | 0 % of each 18° cone blocked; range 111 mm; floor spot 140 mm ahead of the front caster tyre (and behind the rear one) |

Cliff stopping check (hard stop at 1 m/s², 50 ms latency):

| Case | Look-ahead needed | Result |
|---|---|---|
| Forward 0.3 m/s | 60 mm | OK (+80 mm) |
| Reverse 0.3 m/s | 60 mm | OK (+80 mm) |
| Forward 0.5 m/s | 150 mm | short by 10 mm |

Close the 0.5 m/s case in firmware:
- set the ESP32 cliff stop to at least 1.2 m/s² (needs 129 mm), or
- cap forward speed at 0.45 m/s unless the OAK depth stream confirms the floor.

## Recommendations
1. **Front caster spring.** 2 × compression springs, ~1.5 N/mm each, 6 N preload each (12 N total).
   - Braking at 1 m/s² then compresses about 3 mm (≈1.2° pitch) and leaves ~6 mm for bumps.
   - The rigid rear caster keeps ~13 N, so the charging-pad height stays set.
2. **Thresholds.** If the building has 13–15 mm door thresholds, switch to 3-inch casters: `CASTER_D` 75, `CASTER_H` ≈ 95. That is a globals change plus a rebuild of the caster modules.
3. **Battery voltage.** A 4S pack (14.8–16.8 V) feeds 12 V motors. Cap PWM at 71 % or current-limit in firmware; otherwise stall current reaches 7.7 A at full charge. The DFR0601 is fine either way.
4. **Drawer retention.** The drawer is a removable cartridge, and the guides only support it from below. Add a closed-position latch or detent on the face.
5. **Gearbox side load.** Pololu gives no radial rating. The deep-recess wheel mounting keeps the overhang to 15 mm. Bench-test aggressive spins, or add an outboard bearing.
6. **Bumper mass.** The solid TPU bumper is 276 g. Printing it as a 2 mm shell saves most of that.
7. **Bracket bend radius.** Confirm the 1.5 mm inside radius on 3 mm 5052 with the fabricator; a machined bracket is the fallback.
