# RoamView: Power Analysis (as of 2026-10-27)

This estimate is built from datasheets. Validate it against readings from the three INA260 monitors (0x40 motor, 0x41 compute, 0x44 sensor).

**Source codes:**

- **D** — manufacturer datasheet or spec page.
- **M** — third-party measurement.
- **A** — allowance, used where no figure is published. Replace it once the part is chosen or measured.

---

## 1. Power architecture
![alt text](<Power Arch. 2026-10-27.drawio.png>)

## 2. Component data

Components are grouped by the rail that supplies them.

> [!WARNING]
> The robot's mechanical parts are generic, and the team has not chosen the final parts yet. We assume the final parts will have **similar** electrical characteristics.

### 2.1 Buck A: compute rail (5 V)

| # | Component | Qty | Fed from | Supply | Current | Power | Src | Ref |
|---|---|---|---|---|---|---|---|---|
| 1 | Raspberry Pi 5, 8 GB | 1 | Buck A | 5 V (5.1 V / 5 A PD recommended) | — | idle 3.2–3.3 W; stress-ng 9.8 W | M | [1][2] |
| 2 | Pi 5 USB-A ports (total) | — | Buck A, via the Pi 5 | 5 V | 600 mA without a 5 A PD supply; 1.6 A with one | ≤ 3 W | D | [2] |
| 3 | Pi Camera Module 3 Wide (IMX708) | 1 | Buck A, via the Pi 5 | Pi CSI | not published | 0.5 W typ / 1.0 W max | A | [3] |

### 2.2 Buck B: sensor rail (5 V)

| # | Component | Qty | Fed from | Supply | Current | Power | Src | Ref |
|---|---|---|---|---|---|---|---|---|
| 4 | Luxonis OAK-D Pro W | 1 | Buck B | 5 V USB-C + Y-adapter | — | idle 2.5 W; 4 / 6 / 7.5 W (min / typ / max) | D | [4][5] |
| 5 | Waveshare D500 LiDAR | 1 | Buck B | 5 V (4.5–5.5 V) | 290 mA | 1.45 W | D | [6] |
| 6 | ESP32 MCU | 1 | Buck B | module 3.0–3.6 V; board fed 5 V | 30–68 mA at 240 MHz with radio off; 240 mA TX peak; supply ≥ 500 mA | 0.35 W typ / 0.5 W max | D | [7] |
| 7 | Hall encoder (on #4753) | 2 | Buck B | 3.5–20 V | ≤ 10 mA | ≤ 0.05 W each | D | [8] |
| 8 | VL53L4CD carrier | 4 | Buck B | 2.6–5.5 V | 25 mA typ / 40 mA peak each | 0.125 / 0.2 W each | D | [10] |
| 9 | DFRobot DFR0601 DC motor driver | 1 | Buck B (logic); pack (motor side) | motor input 6.5–37 V; logic 3.3 / 5 V | 12 A/ch continuous; 70 A for 100 ms; logic 15 mA; under-voltage cut-off 6.3 V | 0.075 W (logic) | D | [9] |
| 10 | INA260 | 3 | Buck B | 2.7–5.5 V | 310 µA typ | ≈ 0 | D | [11] |
| 11 | Pololu #2804 RC switch with relay (K2) | 1 | Buck B | 5 V | ≈ 100 mA with the relay on | 0.5 W | D | [15] |
| 12 | Recording and status LEDs | 2 | Buck B | 5 V | 20 mA each | 0.2 W | A | — |

### 2.3 Direct from the pack: no buck (14.4 V nominal, 10.6–16.8 V)

| # | Component | Qty | Fed from | Supply | Current | Power | Src | Ref |
|---|---|---|---|---|---|---|---|---|
| 13 | Pololu 37D 50:1 12 V gearmotor, #4753 | 2 | Pack, via the motor driver | 12 V nominal (PWM from the pack) | 0.2 A no-load; 5.5 A stall | 10 W max output; 51 % max efficiency at 2.2 kg·cm / 0.66 A | D | [8] |
| 14 | Panasonic CB1aF-P-12V relay (K1) | 1 | Pack | coil 12 V; must operate at 7 V | 117 mA (103 Ω coil) | 1.4 W rated | D | [14] |
| 15 | Pololu D36V50F5 buck | 2 | Pack | 5.5–50 V in; 5 V ±4 % out | 5.5 A (rated at 36 V in); 2–4 mA quiescent | efficiency 80–95 % | D | [12] |
| 16 | Pololu #5382 ideal diode | 2 | Pack | 4–60 V | 10 A | < 10 mΩ, so < 0.1 W at 3 A | D | [13] |

---

## 3. Load budget by mode

| Mode | Definition |
|---|---|
| Standby | Powered on; subsystems are up and idle, waiting for commands. |
| Autonomous | Autonomous scan: 0.3 m/s cruise, SLAM + Nav2 running, the OAK encodes video, live video and map data upload. Pi CPU at 75 %. |
| Teleop | Teleop walkthrough: up to 0.5 m/s, the Pi encodes H.264 720p30 in software. Pi CPU at 83 %. |
| Peak | Sizing case: every load at its maximum. Both motors climb a 5° ramp while accelerating at 0.5 m/s² at 0.5 m/s. K1 coil at 16.8 V. |

### Raspberry Pi 5 power

Pi power is modelled as a linear function of CPU load, anchored at the idle and stress-ng measurements in [1]:

$$P_{Pi}(u) = 3.3 + (9.8 - 3.3)\,u \;\; \text{W}, \quad u = \text{CPU load}$$

| Mode | $u$ | $P_{Pi}$ (W) |
|---|---|---|
| Standby | 0 (idle) | $3.3 + 6.5 \times 0 = 3.30$ |
| Autonomous | 0.75 | $3.3 + 6.5 \times 0.75 = 8.18$ |
| Teleop | 0.83 | $3.3 + 6.5 \times 0.83 = 8.70$ |
| Peak | 1.00 | $3.3 + 6.5 \times 1.00 = 9.80$ |

### Load table (power at each load)

Values are in W unless marked A (current) or % (loss). Drive motor figures come from section 4.

| Load | Rail | Standby (W) | Autonomous (W) | Teleop (W) | Peak (W) |
|---|---|---|---|---|---|
| Raspberry Pi 5 | A | 3.30 | 8.18 | 8.70 | 9.80 |
| Camera Module 3 Wide | A | 0 | 0.50 | 0.50 | 1.00 |
| **Buck A output** | | **3.30 (0.66 A)** | **8.68 (1.74 A)** | **9.20 (1.84 A)** | **10.80 (2.16 A)** |
| OAK-D Pro W | B | 2.50 | 6.00 | 6.00 | 7.50 |
| D500 LiDAR | B | 1.45 | 1.45 | 1.45 | 1.45 |
| ESP32 MCU | B | 0.35 | 0.35 | 0.35 | 0.50 |
| Encoders × 2 | B | 0.10 | 0.10 | 0.10 | 0.10 |
| VL53L4CD × 4 | B | 0.50 | 0.50 | 0.50 | 0.80 |
| INA260 × 3 | B | 0.01 | 0.01 | 0.01 | 0.01 |
| DFR0601 logic | B | 0.08 | 0.08 | 0.08 | 0.08 |
| LEDs | B | 0.20 | 0.20 | 0.20 | 0.20 |
| RC receiver | B | 0.30 | 0.30 | 0.30 | 0.30 |
| K2 relay (energised) | B | 0.50 | 0.50 | 0.50 | 0.50 |
| **Buck B output** | | **5.98 (1.20 A)** | **9.48 (1.90 A)** | **9.48 (1.90 A)** | **11.43 (2.29 A)** |
| Buck A + B input (efficiency 0.90) | pack | 10.31 | 20.17 | 20.75 | 24.70 |
| K1 coil + 15 Ω resistor | pack | 1.76 | 1.76 | 1.76 | 2.39 |
| Drive motors × 2 (via DFR0601, efficiency 0.95) | pack | 0 | 2.50 | 3.00 | 12.30 |
| Buck quiescent × 2 | pack | 0.12 | 0.12 | 0.12 | 0.12 |
| Wiring, fuse and diode loss | pack | +2 % | +2 % | +2 % | +2 % |
| **Total from the pack** | | **12.4** | **25.0** | **26.1** | **40.3** |
| Pack current at 14.4 V | | 0.86 A | 1.74 A | 1.82 A | 2.80 A |
| Pack current at 12.0 V (near empty) | | 1.04 A | 2.09 A | 2.18 A | 3.36 A |

Energy split in autonomous mode: the 5 V electronics take 82 %, the motors about 10 % and the K1 coil about 7 %. **The electronics, not the drive, set the battery size.**

---

## 4. Drive motor operating points (Pololu #4753 data)

$$F = C_{rr}\,m g + m a + m g \sin\theta,\qquad T = \frac{F\,r}{2},\qquad I = I_0 + \frac{T}{k_t},\qquad V = I R + k_e\,\omega,\qquad P_{pack} = \frac{2\,V I}{\eta_{drv}}$$

**Inputs:**

- Mass $m = 5.71$ kg, from the sim model (needs revision).
- Wheel radius $r = 0.0625$ m.
- Rolling resistance $C_{rr} = 0.02$ (A).
- Driver efficiency $\eta_{drv} = 0.95$ (A).

**Motor constants**, fitted to the #4753 no-load and stall points at 12 V:

- $R = 12/5.5 = 2.18\ \Omega$
- $k_t = 0.389$ N·m/A
- $k_e = 0.552$ V·s/rad

The fit gives 45 % efficiency at the point where Pololu quotes 51 %, so it is about 10 % conservative.

| Case | Wheel speed (rpm) | Torque per motor (kg·cm) | Current per motor (A) | $V_{eff}$ (V) | Pack power, both motors (W) |
|---|---|---|---|---|---|
| Cruise 0.3 m/s, flat | 46 | 0.36 | 0.29 | 3.3 | 2.0 |
| Cruise 0.5 m/s, flat | 76 | 0.36 | 0.29 | 5.1 | 3.1 |
| 0.5 m/s² acceleration at 0.5 m/s | 76 | 1.27 | 0.52 | 5.6 | 6.1 |
| 5° ramp at 0.3 m/s | 46 | 1.91 | 0.68 | 4.1 | 6.0 |
| 5° ramp at 0.5 m/s | 76 | 1.91 | 0.68 | 5.9 | 8.5 |
| 5° ramp + 0.5 m/s² at 0.5 m/s | 76 | 2.82 | 0.91 | 6.4 | 12.3 |
| Stall, PWM duty capped at 12 V equivalent | 0 | 21 | 5.5 | 12.0 | 132 (9.2 A at 14.4 V) |
| Stall, uncapped at 16.8 V | 0 | — | 7.7 | 16.8 | 258 (15.4 A) |

**Averages used in the load budget (A):**

- Autonomous: 2.0 W × 80 % of the time moving × 1.5 for turns, thresholds and caster scrub = **2.5 W**.
- Teleop: 3.1 W × 60 % of the time moving × 1.5 = **3.0 W**.

**Pololu limits [8]:** continuous load ≤ 10 kg·cm and current ≤ 25 % of stall (≈ 1.4 A). The worst normal case (2.82 kg·cm, 0.91 A) is within both.

**Against the proposal (spec 3):** the proposal assumed 12–40 W for the motors at 12 V. The datasheet-based figure is 2–3 W on average and 12.3 W at the 5° + acceleration limit. 40 W is reached only near stall.

---

## 5. Battery sizing vs runtime

### Baseline pack options [17][18]

| | 4S1P | 4S2P | 4S3P |
|---|---:|---:|---:|
| Cells | 4 | 8 | 12 |
| Voltage (nominal / full / cut-off) | 14.4 / 16.8 / 10.6 V | 14.4 / 16.8 / 10.6 V | 14.4 / 16.8 / 10.6 V |
| Capacity | 3.35–3.5 Ah | 6.7–7.0 Ah | 10.05–10.5 Ah |
| Energy | 48.2–50.4 Wh | 96.5–100.8 Wh | 144.7–151.2 Wh |
| Usable energy (85 %, spec 3 reserve) [19] | 41.0 Wh | 82.0 Wh | 123.0 Wh |
| Max continuous current [16][20] | 8 A | 16 A | 24 A |
| Max momentary current | 13 A | 26 A | 39 A |
| Standby, 12.4 W / 0.86 A: runtime · C-rate | 3.3 h · 0.25 C | 6.6 h · 0.12 C | 9.9 h · 0.08 C |
| Autonomous, 25.0 W / 1.74 A | 1.6 h · 0.50 C | 3.3 h · 0.25 C | 4.9 h · 0.17 C |
| Teleop, 26.1 W / 1.82 A | 1.6 h · 0.52 C | 3.1 h · 0.26 C | 4.7 h · 0.17 C |
| Peak, 40.3 W / 2.80 A | 1.0 h · 0.80 C | 2.0 h · 0.40 C | 3.0 h · 0.27 C |
| Cell mass (cells only) | ≈ 0.20 kg | ≈ 0.39 kg | ≈ 0.59 kg |
| Standard charge current [21] | 1.7 A | 3.4 A | 5.1 A |

### Required pack energy (Wh)

| Runtime | Autonomous, 25.0 W | Teleop, 26.1 W | Peak, 40.3 W sustained |
|---|---|---|---|
| 5 min | 2.5 | 2.6 | 4.0 |
| 10 min | 4.9 | 5.1 | 7.9 |
| 15 min | 7.4 | 7.7 | 11.9 |
| 30 min | 14.7 | 15.4 | 23.7 |

To convert to amp-hours: Ah = Wh ÷ 14.4 V.

---

## References

1. J. Geerling, measurements of Pi 5 idle and stress-ng power (C1 stepping, 4/8 GB: idle 3.2–3.3 W, stress-ng 9.8 W) — https://jeffgeerling.com/node/3401
2. Raspberry Pi, *USB Power Delivery on Raspberry Pi 5* (white paper RP-009856): 5 V / 5 A PDO, 600 mA / 1.6 A USB limits, 5 V / 3 A assumed without PD — https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf
3. Raspberry Pi Camera Module 3 product page (no power specification) — https://raspberrypi.com/products/camera-module-3 ; for scale, an Arducam IMX708 USB 3.0 module (which includes an ISP bridge) is rated 0.75–1.36 W — https://www.arducam.com/product/arducam-12mp-imx708-motorized-focus-usb-3-0-camera-module-b0474/
4. Luxonis, OAK-D Pro W hardware documentation (power consumption up to about 7.5 W; Y-adapter required for the IR projector and LED) — https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9098prow
5. Luxonis, OAK-D Pro datasheet (DigiKey copy): power 4 / 6 / 7.5 W, idle 2.5 W — https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/6222/OAK%20D%20Pro.pdf
6. Waveshare, D500 LiDAR Kit wiki and product page: 5 V, 290 mA, 1.45 W — https://waveshare.com/wiki/D500_LiDAR_Kit · https://www.waveshare.com/product/modules/sensors/d500-lidar-kit.htm
7. Espressif, ESP32-WROOM-32E/UE datasheet — https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf ; current table reproduced at https://mischianti.org/esp32-practical-power-saving-manage-wifi-and-cpu-1/
8. Pololu, 50:1 Metal Gearmotor 37Dx70L 12 V with 64 CPR encoder, #4753 (specs, encoder current, load limits) — https://www.pololu.com/product/4753 · https://www.pololu.com/product/4753/specs
9. DFRobot, DFR0601 wiki — https://wiki.dfrobot.com/DFR0601 ; protection thresholds — https://littlebirdelectronics.com.au/products/dual-channel-dc-motor-driver-12a
10. Pololu, VL53L4CD carrier #3692 — https://pololu.com/product/3692 ; ST VL53L4CD datasheet — https://pololu.com/file/0J2060/vl53l4cd.pdf
11. Texas Instruments, INA260 — https://www.ti.com/product/INA260
12. Pololu, D36V50F5 #4091 — https://www.pololu.com/product/4091 · https://www.pololu.com/product/4091/specs
13. Pololu, ideal diode #5382 — https://www.pololu.com/product/5382
14. Panasonic, CB1aF-P-12V (ACB32101) — https://industry.panasonic.com/ap/en/products/control/relay/vehicle/number/acb32101 ; must-operate voltage and switching voltage (DigiKey, CB series) — https://azcus.digikey.com/en/products/detail/panasonic-electric-works/CB1A-12V/647008
15. Pololu, RC Switch with Relay user's guide §5.1 — https://www.pololu.com/docs/0J60/5.1 ; 5 V relay 63 Ω / 80 mA — https://www.pololu.com/category/135/line-sensors
16. Samsung SDI INR18650-35E specifications — https://www.18650batterystore.com/products/samsung-35e-18650-3500mah-8a-battery · https://www.electrokit.com/en/batteri-li-ion-18650-3.7v-3500mah-inr18650-35e

**Further reading: Battery background**

17. Battery University, *BU-302: Series and Parallel Battery Configurations* (what "4S2P" means; links to BU-803a on cell matching and balancing) — https://batteryuniversity.com/article/bu-302-series-and-parallel-battery-configurations
18. Adafruit, *Li-Ion & LiPoly Batteries* (voltages, protection circuitry, multi-cell packs, charging) — https://learn.adafruit.com/li-ion-and-lipoly-batteries
19. G. Plett, University of Colorado Boulder, *Algorithms for Battery Management Systems* (Coursera, 5 courses; SOC and SOH estimation, balancing) — https://www.coursera.org/specializations/algorithms-for-battery-management-systems
20. lygte-info.dk, independent test of the Samsung INR18650-35E (discharge curves and cell temperature by current) — https://lygte-info.dk/review/batteries2012/Samsung%20INR18650-35E%203500mAh%20(Pink)%20UK.html
21. Texas Instruments, *Battery Management Deep Dive* (free videos: chargers, fuel gauging, protection, cell balancing) — https://training.ti.com/battery-management-deep-dive-technical-training

