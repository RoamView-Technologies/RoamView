# RoamView CAD design rules

Rev B layout 2, 2026-09-28. These rules apply to `RoamView_Build.bas` (Rev A), the Rev B build script `old-designs/RevB/RoamView_Build_RevB.py`, and the Rev B interconnect model and cable router in `old-designs/Interconnect_RevB/` that feed it. If a drawing and a rule disagree, the rule wins until the team changes it here.

Rules marked **(enforced)** are checked automatically: part placement by `rules_b.py`, cables by `build_b.py`. Rules marked **(new)** were added at the 2026-09-27 review and apply from Rev B on. Numbers are Rev B layout 2 unless marked Rev A.

---

## Rev B drivetrain decisions (2026-09-27)

Chosen from the concept renders in `old-designs/Drivetrain_Options/`.

- **Wheels and casters (option A).** Differential drive stays. The drive wheels grow to 125 mm and the casters become low-profile 3-inch casters at X ±118.
- **Suspension (option B).** Each drive motor and wheel rides on a trailing arm that pivots behind the axle, with one spring per side and 10 mm of travel to a hard stop.
- **Bumper and handle.** A bumper face with two switches goes in front of the charge contacts, and a carry handle goes on the rear posts (see §6).
- **What changed** (confirmed by the Rev B layout on 2026-09-28):
  - The decks rise 21 mm. The lower deck underside is at 93 mm (Rev A 72) and the top deck top at 222 mm (Rev A 201): 12.5 mm for the larger wheels plus room for the motors to rise 10 mm, with 2 mm to spare.
  - Each drive module is a 5 mm aluminium plate carrying the motor and wheel. It pivots on a hanger under the lower deck, 66 mm behind the axle. Its spring sits 32 mm ahead of the axle and 72 mm either side of centre, rises through an 18 mm hole in the lower deck and seats in a cup on the deck top. The nut on the spring's guide bolt is the droop stop (5 mm).
  - The casters bolt to the lower deck underside at 93 mm: a 3-inch low-profile caster, about 89 mm tall, on a 4 mm spacer.
  - The LiDAR riser is cut to 40 mm (Rev A 60), so the scan plane stays at about 286 mm.
  - The motors need about 25 % more torque for the same acceleration. The 37 mm gearmotor stays a placeholder until that is checked.
  - The centre of gravity rises to about 126 mm and sits only 2–4 mm behind the axle (Rev A 9 mm). That breaks the cliff-stop rule in §6; see the open issue there.
- **Layout 2 moves**, made to meet the spacing and service rules:
  - The battery slides out forward, because the larger wheels block the sideways path.
  - The Pi 5 is turned so its USB and Ethernet ports face the rear edge and its USB-C and micro-HDMI ports face the left edge.
  - The e-stop moves to X −105 on the centre line, clear of the handle and the Pi's ports.
  - F1 moves onto the power board, and the lever splices drop to two: the pack + node and the star ground.
  - There is no junction board. The signal deck plug grows to 24 pins and carries 3.3 V from the DevKit.
  - The cliff sensors hang under the lower deck, and the compute and sensor power blocks hang under the top deck.

---

## 1. Frame, units and source of truth

- Units are millimetres. Frame: +X forward, +Y up, +Z robot right. Origin is the floor point under the drive-axle centre, which is ROS `base_footprint` (ros x = X, ros y = −Z, ros z = Y).
- Chassis geometry comes from the constants at the top of `old-designs/Interconnect_RevB/model_b.py` (Rev B) and `RoamView_Build.bas` (Rev A). Change a constant and rebuild; don't edit the generated parts by hand.
- Part placements and cable routes come from one interconnect model, `model_b.py`. That model drives the CAD model, the web animation and the harness cut list, so all three stay in step. The Rev B SOLIDWORKS model in `old-designs/RevB/` is generated from it: `cad_spec_b.py` exports `RoamView_RevB_spec.json`, and `RoamView_Build_RevB.py` builds every part, both decks, the harness and the assembly from that file (see `old-designs/RevB/README.md`).
- Once a part is chosen, it is modelled at its datasheet outline with its connectors in the right places. Until then its Description property says "placeholder".
- A plate change needs a new part number or revision. Reference plate volumes for rebuild checks, from the Rev B SOLIDWORKS build: lower deck RV-CH-001B 404,264 mm³ (1.092 kg) and top deck RV-CH-002B 238,649 mm³ (0.644 kg); Rev A 414,753 and 240,471 mm³.

## 2. Envelope and keep-out zones (enforced)

| Zone | Rule |
|---|---|
| Footprint | Nothing outside 320 × 260 mm, except the bumper face (to X 171, 11 mm proud of the front edge), the charge contacts (to X 163, behind the bumper's windows) and the caster wheels when they swing round (below the lower deck). |
| Ground clearance | No part below 38 mm except the wheels and casters. The drive modules are lowest: 42 mm at rest, 38 mm at full bump. No cable below 55 mm. |
| Wheel wells | No part within X ±75 mm and \|Z\| ≥ 88 mm below 140 mm (the wheel at full bump plus 5 mm). Cables keep another 3 mm away. |
| Suspension sweep **(new)** | Nothing within 2 mm of a drive module anywhere between full bump (wheel 10 mm up) and full droop (5 mm down), and the modules never touch the lower deck. |
| Caster swing **(new)** | Nothing within 2 mm of the space each caster sweeps as it swivels: about 60 mm around its axis at X ±118, from the floor up to the mount face. No cable below the lower deck within 64 mm of a caster axis. |
| LiDAR scan plane | Nothing on the robot between 271 and 301 mm above the floor, all the way round. The plane is at about 286 mm. Cables stay below 268 mm. |
| OAK-D field of view | Nothing ahead of X 146 above 230 mm. The stereo view is 127° wide and 79.5° tall. |
| Pi 5 fan inlet | The GPIO HAT has a 35 mm hole over the cooler fan (X −96 to −60, Z −86 to −50). Keep it and the space above it clear. |
| Holes that are not pass-throughs | Spring holes, the LiDAR cable hole, grid holes and fastener holes never carry cables. |
| Battery removal path **(new)** | The pack slides out forward. Keep its lane clear (X 55 to 175, \|Z\| ≤ 42, from the deck top up to 152 mm), and nothing above the pack (X −40 to 60, \|Z\| ≤ 42, from 144 mm up to the top deck) except its own XT60 lead. |

## 3. Serviceability **(new)**

- **Spacing (enforced).** Leave at least 10 mm between neighbouring parts, 15 mm around anything you unplug, and 25 mm of finger space in front of every connector's mating face.
- **One-part swaps.** The battery, Pi 5, ESP32 DevKit, both buck converters, the motor driver, the INA260 boards and every fuse can each be replaced without removing another part.
  - Battery: unplug the XT60, undo two straps, lift the pack over the tray's 6 mm front lip and slide it out forward between the decks.
  - Pi 5: unplug its cables, lift off the GPIO HAT, then four screws.
  - ESP32: it sits in sockets on its carrier, so it pulls straight out once its strap is off.
  - Fuses: F1, F3 and F4 pull straight up out of blade holders on the power board; F2 is in an inline holder on the charge lead.
- **Common parts.** Use the same part wherever a function repeats (both bucks are Pololu D36V50F5), so one spare covers both.
- **Buy built (added 2026-09-28).** Prefer a pre-built part or pack over assembling the same function from individual components whenever a suitable off-the-shelf one exists: the pre-built 4S2P pack, the DFR0601 driver, the INA260 modules and the Pololu ideal-diode boards already follow this. A custom or discrete solution needs a specific reason (cost, size, or no suitable part) to win.
- **Top deck removal.** All wiring between the decks goes through two locking plugs tied to the rear posts: a 6-pin Micro-Fit for power (compute feed, sensor feed, e-stop loop) at the rear-left, and a 24-pin Micro-Fit for signals (encoders, driver control, I²C, 3.3 V, 5 V, cliff-sensor XSHUT lines, bumper switches) at the rear-right. Unplugging those two frees the deck. There is no junction board.
- **MCU access.** The ESP32 carrier sits on the top deck at the rear-right, with the DevKit's two USB-C ports facing the rear edge so a laptop can plug in with the robot assembled. Every carrier connector sits on an outward-facing edge and is labelled with its cable ID.
- **Debug access with the robot assembled.** Reachable without removing anything:
  - the Pi's SD card (front edge), USB and Ethernet (rear edge), USB-C and micro-HDMI (left edge);
  - the ESP32 USB ports;
  - fuses F1 to F4 and the XT60;
  - the e-stop reset;
  - the terminals of each INA260 (every rail's test point).
- **Labels.** Every cable carries its ID (P01…, D01…) at both ends, and each board connector is marked with the same ID.

## 4. Harness

- **Pass-throughs (enforced).** Cables cross a deck only through the eight pass-through slots: three in the lower deck (rear, front-left, front-right) and five in the top deck (left, right, rear, front, centre). The LiDAR lead alone goes down the riser's bore and the 20 mm hole under it. The slots are lined with edge grommet: Essentra DSC-1 on the 3 mm top deck and DSC-3 on the 6 mm lower deck. Lined, a top-deck slot is about 8 mm wide and a lower-deck slot 10–12 mm, so only a bare-wire end or a narrow connector goes through. That fixes the assembly order.
- **Clearance (enforced).** Cables keep at least 3 mm from every part. They lie on a surface or rise vertically; no cable runs diagonally through open air.
- **Bend radius.** At least 5 × outside diameter for USB and multi-core cables, and 3 × for single hook-up wire.
- **Locking connectors only.** Use Micro-Fit, JST-XH, XT60, terminal blocks or lever splices. The Pi's GPIO connects through a HAT with locking connectors, not loose jumpers.
- **Pairs and noise.** Each power + and − run together. Motor leads are twisted. After the motor breakout, UART and I²C wires stay out of the motor-lead bundle.
- **Tie-downs.** A tie point at least every 100 mm, and one within 30 mm of every connector, so no cable hangs on its plug.
- **Cut length.** Cut length is the routed length plus 20 mm per end, rounded up to 10 mm. Rev B totals about 10.2 m over 45 cables.
- **Fill.** A pass-through is never more than 40 % full by cable cross-section. In Rev B the busiest is the top deck's rear slot, at 14 % with the 24-core deck signal cable.

## 5. Electrical

- **Star ground.** All returns meet at one lever splice (WAGO 221-415) on the lower deck beside the pack; a second splice is the pack + node. The frame is bonded to the star ground at one point only. Nylon standoffs go under every board, because the Pi's mounting holes (and many others) are tied to board ground.
- **Fuse the wire.** F1 is 15 A on the 14 AWG trunk. F3 and F4 are 3 A on the 18 AWG buck feeds. All three sit in PCB blade holders (Keystone 3557-2) on the power board. F2 is 5 A on the charge input, in an inline holder. Thin 5 V branches get a resettable fuse or wire of 22 AWG or thicker.
- **Runs ahead of the fuses (new).** The XT60 lead, P01 to the + splice and P14 to F1 (about 20 cm of 14 AWG together), and P17 from the charge guard back to the + splice, are protected only by the pack's BMS. Keep them short, sleeved and tied down. Moving D1 to the splice end of P17 would make that lead dead from the pack side.
- **High current stays on wire.** Anything over 3 A runs on wire with screw or lever terminals, never on prototype-board traces. On the power board, those wires solder straight to the relay and fuse-holder pins.
- **E-stop in hardware.** Contact NC1 opens K1's coil without any software involved. The RC relay K2's contact is in series in the same coil loop, so losing the radio link or flipping the transmitter switch also stops the motors. K1's coil runs through a 15 Ω 1 W resistor, which keeps it inside its 10–16 V range across the 12–16.8 V pack range. A diode and 24 V zener across the coil make K1 open quickly. Contact NC2 reports to the ESP32 for logging, and the robot will not restart until someone re-arms it.
- **Hold-up.** The 4700 µF capacitor goes on Buck A's 14.8 V input, behind ideal diode D2. On the 5.1 V side it would last under 1 ms.
- **Dead contacts.** The charge contacts are dead unless the dock drives them (ideal diode D1).
- **3.3 V logic.** No signal above 3.3 V reaches an ESP32 pin. The encoders and K2's status output (both 5 V) go through the 74LVC244 on the ESP32 carrier. The cliff sensors, INA260 #1 and the bumper switches run at 3.3 V from the DevKit's 3V3 pin through the signal deck plug, about 170 mA at most. The pin map avoids:
  - GPIO0, 3, 45 and 46 (strapping);
  - GPIO19 and 20 (USB);
  - GPIO33–37 (octal flash and PSRAM on the N32R16V);
  - GPIO38 (the RGB LED on DevKitC-1 v1.1);
  - GPIO43 and 44 (console);
  - GPIO47 and 48, which run at 1.8 V on the N32R16V.
- **Pin budget (new).** All 22 usable ESP32 pins are assigned; the map is in `model_b.py`. The rear-right cliff sensor has no XSHUT line, so the firmware gives it a new address before releasing the other three. The next signal needs an I²C expander rather than a GPIO.

## 6. Mechanical robustness **(new)**

- **Design loads.** Every part and its fasteners must hold at 3 g horizontal in any direction and 2 g vertical. That covers a collision at 0.5 m/s and bumps.
- **Hard braking.** The tyres can only produce about 0.6–0.8 g. With all four wheels on the floor and the Rev B estimate from `cog_b.py` (5.7 kg, centre of gravity 126 mm up and 2–4 mm behind the axle):
  - tipping forward over the front caster takes 0.81 g of braking;
  - tipping back over the rear caster takes 0.75 g of braking in reverse, or 1.06 g of forward acceleration;
  - tipping sideways over a drive wheel takes 0.89 g.

  In practice the tyres slide first, and the drive wheels lose grip as soon as the robot starts to pitch over a caster, so braking and turning alone should not tip it. Recheck once the robot is weighed.
- **Cliff-stop stability (new).** The front caster trails 20 mm behind its swivel axis, so while driving forward it rolls over a drop about 33 mm after the front cliff sensors see it. The robot then finishes stopping on its drive wheels and rear caster only. To stop at 0.5 m/s² without pitching forward, the centre of gravity must sit at least 10 mm behind the drive axle: 6.4 mm for the braking plus about 3 mm for the springs settling. `cog_b.py` checks this.
  - **Open issue, Rev B layout 2:** the estimate is 2–4 mm (Rev A had 9 mm), so this fails. Recommended fix: move the drive axle about 12 mm forward of the plate centre, with the LiDAR moving with it. The alternative is about 0.4 kg of ballast at the rear edge.
  - Either way, the firmware should ramp a cliff stop down at 0.5 m/s² or less and never brake hard (advice for the software team).
- **Suspension (new).** Each drive module pivots on an 8 mm shoulder bolt in bronze bushings. Travel is 10 mm up to a hard stop and 5 mm down to the nut on the spring's guide bolt. Set the spring rate and preload after weighing the robot, so the drive wheels keep enough load to climb a 5° ramp with a caster on a 15 mm bump.
- **Retention.**
  - Battery: in a tray with a rear stop and a 6 mm front lip, as well as two straps.
  - Microphone array and speaker (or the speakerphone, if the team keeps it): in brackets.
  - C1: glued with RTV.
  - K1: PC-board mount.
  - ESP32 DevKit: held in its sockets by a strap or clip.
  - Parts hanging under a deck (power blocks, K2, cliff sensors, deck plugs): screwed down or tied to a post, never held by tape or glue alone.
  - USB plugs: tied back within 30 mm.
  - Screws: every screw with a nylock nut or threadlocker.
  - Wheels: set screw on the D-flat plus threadlocker.
  - LiDAR riser: printed in PETG at 40 % infill or more, and through-bolted.
- **Front protection.** The charge contacts sit behind a 5 mm PETG bumper face, 250 mm wide and 56–100 mm above the floor, on two flexure brackets. Two Omron SS-5GL2 switches close when it moves 2 mm. Windows let the dock's pins through, and a centre notch clears the front caster's swing.
- **Handle.** A 16 mm grip across the rear, 38 mm above the top deck (top at 268 mm, under the LiDAR band), bolted through the deck into the rear posts. Lift the robot here, never by the LiDAR.

## 7. Sensing geometry

- **Cliff sensors.** Four VL53L4CD sensors hang under the lower deck at X ±131, 87 mm either side of centre, looking straight down 87 mm to the floor. The front pair is 131 mm ahead of the drive axle.
  - Stopping from 0.5 m/s at 0.5 m/s² with 50 ms of reaction takes about 275 mm.
  - Near edges, forward speed is limited to about 0.3 m/s (a 105 mm stop) unless the depth camera stops the robot first. At that speed the front caster goes over the edge before the robot stops, which is why §6 has a cliff-stop rule.
  - Reverse is limited to about 0.1 m/s. The rear caster then stays on the floor: it swings round to 98 mm behind the axle, 33 mm inside the rear sensors.
  - The cliff stop runs on the ESP32, not through ROS.
- **LiDAR.** Level within 1°, mounted directly over the drive axle.
- **Antennas.** Mounted vertically, with the tips below 271 mm or above the LiDAR head. At least 50 mm from the OAK-D's USB 3 cable. Prefer the 5 GHz band.

## 8. Operating limits (to confirm with the team)

- **Floors.** Flat indoor floors, thresholds up to about 15 mm (limited by the 3-inch casters) and ramps up to 5°. The robot cannot climb stairs; it only detects drops.
- **Turning.** Turning in place sweeps a circle about 425 mm across (the bumper's corners). Plan for aisles of at least 475 mm.
- **Charging temperature.** 0–45 °C. The BMS thermistor cuts off charging below 0 °C.

## 9. Release checks

- `python rules_b.py`: 0 issues. It also checks the plate holes: at least 1.5 mm of metal between any two holes, and half the hole's width (at least 1.5 mm, at most 5 mm) between a hole and the plate edge.
- `python build_b.py`: 0 clearance issues, 0 cable overlaps, every pass-through 40 % full or less.
- CAD build log (`RoamView_build_log_RevB.txt`): every part's bounding box matches the model, and the interference check lists only the intended overlaps: motor shafts in the wheel hubs, pivot pins in the hanger bushings and spring guide bolts through the module tabs (the envelopes have no bores).
- Plate volumes in SOLIDWORKS match the model to 0.01 % (the model draws circles as polygons, so it is a few mm³ off).
- Every part in the assembly has a BOM line with an MPN, or is marked "placeholder".
- Board mounting holes come from the maker's drawing, never from a centred guess (Rev B check: the Pi 5 pattern sits 3.5 mm from the SD-card end; the half-size Perma-Proto has two holes 73.7 mm apart; the VL53L4CD has four on 20.32 × 12.70 mm).
- `python cog_b.py`, then the weighed robot: centre of gravity at least 10 mm behind the drive axle (§6).
