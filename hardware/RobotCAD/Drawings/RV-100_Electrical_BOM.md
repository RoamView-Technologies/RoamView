# RoamView electrical BOM

This list covers everything electrical in the RV-100 CAD BOM (`RV-100_BOM.csv`, 2026-11-07), with the electrical data the schematic and the power PCB need. Quantities, part numbers and connector positions come from the CAD. Voltages, currents and pinouts come from the vendor datasheets linked at the end. Values the vendor does not publish are marked "measure".

## 1. Parts in the CAD

| CAD no. | Part | Qty | Supply | Current | Signals to the controller | Connector on the part | Connects to |
|---|---|---|---|---|---|---|---|
| RV-541 | 4S Li-ion pack (model not chosen; CAD envelope 240 × 90 × 58 mm, 900 g) | 1 | 12.0 to 16.8 V, 14.8 V nominal | sized by the motors, see section 2 | pack voltage and current through the INA260s | XT60 male at the drawer end | HW-15 XT60 panel socket, then the power PCB |
| RV-532 | Power PCB in its case (to be designed) | 1 | battery | all loads | I2C (INA260s), relay drive | to be defined | battery, motor driver, 5 V loads, dock contacts |
| RV-501 | Pololu 4754 gearmotor 37D 70:1, 12 V, 64 CPR encoder | 2 | 12 V rated; runs from the 4S pack | 0.2 A no load, 5.5 A stall at 12 V (about 7.7 A at 16.8 V) | encoder A and B, 64 CPR at the motor, 4480 counts per wheel turn | 1 × 6 female header, 2.54 mm, 20 cm leads | motor leads to RV-531; encoder to the ESP32 |
| RV-531 | DFRobot DFR0601 dual motor driver | 1 | 6.5 to 37 V motor supply; 3.3 or 5 V logic | 12 A per channel continuous, 70 A for 100 ms | PWM1, INA1, INB1, PWM2, INA2, INB2 (PWM 18 to 60 kHz) | P+ / P−, M1_A/B, M2_A/B terminals; 4 × Ø3 holes on 45 × 45 | battery through the E-stop relay; ESP32 |
| RV-522 | ESP32-S 38-pin expansion board + ESP32-WROOM-32E USB-C dev board | 1 | DC jack on the carrier (range: measure) or 5 V | about 0.25 A | 3.3 V GPIO only, not 5 V tolerant | 38-pin female headers, DC jack, USB-C | Pi 5 (UART or USB), all sensors and switches below |
| RV-521 | Raspberry Pi 5 + active cooler | 1 | 5 V USB-C PD, or 5 V on GPIO pins 2/4 | up to 5 A (27 W) including 1.6 A for its USB ports | UART, USB 3, DSI | USB-C power, 2 × USB 3, 2 × USB 2, 40-pin header, 2 × 22-pin MIPI FPC | 5 V buck; OAK; LiDAR; display; ESP32 |
| RV-511 | Luxonis OAK-D Pro W (USB) | 1 | 5 V | 2.5 to 3 W base, up to 15 W peak with IR dot projector and flood LED | USB 3 | USB-C | Pi 5 data through RV-514 |
| RV-514 | Luxonis OAK Y-adapter DM6010 | 1 | 5 V on its power port | passes the OAK's power | USB data passthrough | 3 × USB-C (camera, data, power) | Pi 5 USB 3 and a 5 V, 15 W supply from the PCB |
| RV-512 | LDROBOT STL-19P (D500) LiDAR | 1 | 4.5 to 5.5 V | 290 mA working | UART TX only, 230400 8N1, 3.3 V; PWM speed input (ground it if unused) | ZH1.5T-4P, 1.5 mm: 1 Tx, 2 PWM, 3 GND, 4 P5V | Pi 5 UART RX (or USB-UART), 5 V rail |
| RV-552 | Raspberry Pi Touch Display 2, 7 in | 1 | 5 V from the Pi GPIO header | measure | MIPI DSI, touch over DSI | 22-pin FPC + 2-pin power lead | Pi 5 |
| RV-513 | Adafruit 5396 VL53L4CD ToF (cliff sensors) | 4 | 3 to 5 V (on-board regulator and level shift) | measure (small) | I2C, fixed address 0x29 at power-up; XSHUT and GPIO1 pins | 2 × STEMMA QT (JST SH 4-pin) + header holes | ESP32 I2C; one XSHUT line each to give unique addresses |
| RV-553 | Omron D2F-01L-D microswitch (bumper) | 2 | signal level | 0.1 A 30 VDC max, gold contacts | COM, NO, NC | solder terminals | ESP32 GPIO with pull-up |
| RV-551 | Excel Cell ECS-22RE1RM2Q-020Q E-stop, 22 mm | 1 | in the power path | 10 A on sister models, confirm for -020Q | contacts (CAD says 2 NC; DigiKey lists it as DPST, confirm) | screw terminals on the contact block | motor relay coil (K1) and an ESP32 sense input |
| RV-435 | Charging contact pad, brass 20 × 26 | 2 | charger voltage (to be defined) | charge current (to be defined) | none | solder or ring lug, to be decided | power PCB charge input |
| HW-15 | XT60 panel-mount female (drawer blind mate) | 1 | battery | 60 A rated XT60 | none | XT60 | battery to power PCB |
| HW-17 | Addressable LED strip, about 260 mm | 1 | 5 V (assumes WS2812B, 60/m: about 15 LEDs) | up to about 0.9 A at full white | one data line (5 V logic level for WS2812B) | 3 wire | 5 V rail, ESP32 GPIO through a level shifter |
| HW-14 | Magnet Ø6 × 3, dock detect | 1 | none | none | sensed by the dock, not the robot | none | none |

The CAD also records what goes on the power PCB (RV-532 property `Contents`): K1 Panasonic CB1aF relay, K2 relay with RC receiver, 3 × INA260 current sensors, 2 × 5 V buck, 3 × ideal diode and a 4700 µF bulk capacitor.

## 2. Power budget (estimate)

| Rail | Loads | Normal | Peak |
|---|---|---|---|
| Battery, 12 to 16.8 V | 2 motors through RV-531 | 0.7 A cruising, 2.6 A on a 5° ramp while accelerating (validation report) | 15.4 A with both motors stalled at 16.8 V |
| 5 V, Pi branch | Pi 5 with display | about 3 A | 5 A (27 W) |
| 5 V, accessory branch | OAK (via Y-adapter) 3 A, LiDAR 0.29 A, LED strip 0.9 A, ESP32 board 0.25 A, 4 ToF | about 1.5 A | about 4.5 A |
| 3.3 V | ESP32 logic, encoders (power them from 3.3 V so A/B stay ESP32-safe), ToF logic | from the ESP32 board regulator | |

Two 5 V bucks match this split: one for the Pi alone, one for the OAK and the rest, each sized for 5 A. The OAK needs its own 15 W feed through the Y-adapter; the Pi's USB ports cannot carry its IR peaks.

## 3. Controller I/O the schematic must route

ESP32 (3.3 V logic), 22 pins in total:

| Function | Pins |
|---|---|
| Motor driver PWM1, INA1, INB1, PWM2, INA2, INB2 | 6 outputs |
| Encoder A/B, both motors | 4 inputs (interrupt capable) |
| I2C bus for 4 ToF sensors and 3 INA260s | 2 (SDA, SCL) |
| ToF XSHUT, one per sensor | 4 outputs |
| Bumper switches | 2 inputs |
| E-stop state | 1 input |
| LED strip data | 1 output |
| UART link to the Pi | 2 |

Raspberry Pi 5: USB 3 to the OAK, UART RX (GPIO15) from the LiDAR, DSI to the display, UART or USB to the ESP32.

## 4. What the CAD fixes for the PCB and wiring

- The power PCB and its case must fit a 95 × 120 × 40 mm envelope (globals `PCB_L`, `PCB_W`, `PCB_H`), centred at `PCB_X` 33.5 mm under the deck.
- The battery plugs in blind: its XT60 meets HW-15 at the inner end of the drawer when the drawer closes.
- The charging pads sit at y ±64 mm, 30 mm above the floor, at the front of the robot (`CONTACT_*`).
- The E-stop goes through the deck at (−100, −100) mm.
- Cables leave the body through deck grommets of Ø25 (tower) and Ø16 (LiDAR).
- The four cliff sensors sit at (±218, ±105) mm; their STEMMA cables drop through holes in the end skins below the MidCross.
- The motor driver sits on RV-312, and the Pi 5 and ESP32 board share the RV-311 compute tray.

## 5. CAD and the power schematic notes disagree

`hardware/datasheets/README.md` was written for an earlier power schematic. These lines differ from the CAD and need one answer before the schematic is drawn:

| Item | CAD | Schematic notes |
|---|---|---|
| Gearmotor | Pololu 4754, 70:1 | Pololu 4753, 50:1 |
| Cliff sensor | Adafruit 5396 (STEMMA QT) | Pololu 3692 carrier |
| ESP32 | 38-pin expansion board + WROOM-32E USB-C dev board | ESP32 DevKitC V4 |
| Ideal diodes | 3 | 2 (U1, U4) |
| E-stop | Excel Cell ECS-22RE1RM2Q-020Q | not chosen |
| Pi Camera Module 3 Wide | not in the CAD | listed |

## 6. Still to choose (not in the CAD)

Battery pack and BMS, charger circuit and its voltage at the dock contacts, fuses F1 to F4, main switch SW1, RC receiver, bucks and diodes by part number, wire gauges and connectors for each harness, and the drawer XT60 bracket.

## Sources

- [Pololu 4754 gearmotor](https://www.pololu.com/product/4754)
- [DFRobot DFR0601 wiki](https://wiki.dfrobot.com/DFR0601)
- LDROBOT STL-19P datasheet v1.0, `hardware/datasheets/LDROBOT_STL-19P_Datasheet_EN_v1.0.pdf`
- [Luxonis OAK-D Pro W](https://docs.luxonis.com/hardware/products/OAK-D%20Pro%20W), [Y-adapter](https://docs.luxonis.com/hardware/products/Y-Adapter), [USB deployment guide](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide)
- [Adafruit 5396 VL53L4CD](https://www.adafruit.com/product/5396)
- [Raspberry Pi Touch Display 2](https://www.raspberrypi.com/products/touch-display-2/)
- Raspberry Pi 5 product brief, `hardware/datasheets/RP-008348-DS-6-raspberry-pi-5-product-brief.pdf`
- [Excel Cell ECS-22RE1RM2Q-020Q on DigiKey](https://www.digikey.de/en/products/detail/excel-cell-electronic-co-ltd/ECS-22RE1RM2Q-020Q/25812873), [sister model ECS-22RE1RM1Q-010Q](https://www.digikey.ca/en/products/detail/excel-cell-electronic-co-ltd/ECS-22RE1RM1Q-010Q/25812945)
- Omron D2F datasheet D2F_0318 (used for RV-553)
- Motor currents in use: `Simulation/Validation_Report.md`, section 6
