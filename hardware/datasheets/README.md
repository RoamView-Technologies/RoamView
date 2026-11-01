# Datasheets

Manufacturer documents for every part on the power schematic (`hardware/KiCAD/RoamView/power.kicad_sch`).
Files downloaded for the schematic design live in `KiCAD/`; the module datasheets at this level were already here.
Each symbol's **Datasheet** field opens the local copy (`${KIPRJMOD}/../../datasheets/[KiCAD/]...`); the extra
**Schematic / Pinout / Drawing** fields point to the manufacturer's board schematic where one is published.

| Ref | Part | Manufacturer board schematic | Datasheet / other docs |
|---|---|---|---|
| U2, U3, U6 | Adafruit INA260 breakout (#4226) | `KiCAD/Adafruit_INA260_breakout_guide.pdf` (Downloads section), Eagle source `KiCAD/Adafruit_INA260_breakout_schematic.sch` | `KiCAD/TI_INA260_datasheet.pdf` |
| U1, U4 | Pololu #5382 ideal diode | `KiCAD/Pololu_5382_ideal_diode_schematic.png` | `KiCAD/TI_LM74700-Q1_ideal_diode_controller_datasheet.pdf` (controller IC), `KiCAD/Pololu_5382_ideal_diode_pinout.jpg` |
| U5, U7 | Pololu D36V50F5 buck (#4091) | not published by Pololu | `KiCAD/Pololu_D36V50Fx_regulator_dimensions.jpg`, https://www.pololu.com/product/4091 |
| U8 | DFRobot DFR0601 motor driver | `KiCAD/DFRobot_DFR0601_block_diagram.png` (block diagram only) | `KiCAD/DFRobot_DFR0601_dimensions.png`, https://wiki.dfrobot.com/DFR0601 |
| K2 | Pololu #2804 RC switch with relay | `KiCAD/Pololu_2804_RC_switch_relay_schematic.png` | `KiCAD/Pololu_2804_RC_switch_relay_pinout.jpg`, `KiCAD/..._wiring.jpg`, `KiCAD/..._dimensions.jpg`, `KiCAD/Omron_G5LE_relay_datasheet.pdf` |
| K1 | Panasonic CB1aF-P-12V (ACB32101) | n/a (relay) | not stored: Panasonic serves the CB catalog (ASCTB220E) only through its download portal, https://industry.panasonic.com/ap/en/products/control/relay/vehicle/number/acb32101 |
| A4 | ESP32 DevKitC V4 | `KiCAD/Espressif_ESP32-DevKitC-V4_schematic.pdf` | `esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf` |
| A1 | Raspberry Pi 5 | not published for Pi 5 | `RP-008348-DS-6-raspberry-pi-5-product-brief.pdf`, `KiCAD/RaspberryPi5_mechanical_drawing.pdf` |
| (A1) | Camera Module 3 Wide | not published | `RP-008151-DS-1-camera-module-3-product-brief.pdf` |
| A2 | Luxonis OAK-D Pro W (DM9098) | not open-sourced (luxonis/oak-hardware) | `KiCAD/Luxonis_OAK-D-Pro_DM9098_datasheet.pdf` |
| A3 | Waveshare D500 LiDAR (LDROBOT STL-19P) | not published | `LDROBOT_STL-19P_Datasheet_EN_v1.0.pdf` |
| J6–J9 | Pololu VL53L4CD carrier (#3692) | `KiCAD/Pololu_VL53Lx_carrier_schematic.pdf` | `KiCAD/ST_VL53L4CD_datasheet.pdf`, `KiCAD/Pololu_VL53L4CD_carrier_pinout.jpg` |
| J3, J4 | Pololu 37D 50:1 gearmotor + encoder (#4753) | n/a | `KiCAD/Pololu_37D_gearmotor_datasheet.pdf`, `KiCAD/Pololu_37D_gearmotor_dimensions.pdf` |
| Q1 | AO3400A | — | `KiCAD/AOS_AO3400A_datasheet.pdf` |
| Q2, Q3 | 2N7002 | — | `KiCAD/Diodes_2N7002_datasheet.pdf` |
| D1 | 1N4007 | — | `KiCAD/Diodes_1N4001-1N4007_datasheet.pdf` |
| D2 | SMBJ18CA TVS | — | `KiCAD/Vishay_SMBJ_TVS_datasheet.pdf` |

Not chosen yet (no datasheet stored): pack/BMS (cells: Samsung INR18650-35E), fuses F1–F4 (ATO/ATC blade),
SW1 main switch, SW2 E-stop, LEDs, resistors, capacitors, dock contacts, RC receiver.

## OAK-D Pro W (Luxonis)
- File: `KiCAD/Luxonis_OAK-D-Pro_DM9098_datasheet.pdf` (from https://github.com/luxonis/oak-hardware/tree/master/DM9098_OAK-D-Pro/Datasheet)
- Product: https://shop.luxonis.com/products/oak-d-pro-w
- Hardware docs: https://docs.luxonis.com/hardware/products/OAK-D%20Pro%20W
- DepthAI hardware reference: https://docs.luxonis.com/projects/hardware/en/latest/pages/DM9098prow.html
- All hardware docs: https://docs.luxonis.com/hardware

## ESP32-WROOM-32E / 32UE
- File: `esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf`
- DevKitC V4 schematic: `KiCAD/Espressif_ESP32-DevKitC-V4_schematic.pdf` (https://dl.espressif.com/dl/schematics/esp32_devkitc_v4-sch.pdf)
- Datasheet (HTML): https://documentation.espressif.com/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.html
- Datasheet (PDF): https://documentation.espressif.com/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf
- Product page: https://www.espressif.com/en/producttype/esp32-wroom-32e32ue
- Technical documents: https://www.espressif.com/en/support/documents/technical-documents/esp32-wroom

## Raspberry Pi Camera Module 3
- File: `RP-008151-DS-1-camera-module-3-product-brief.pdf`
- Docs: https://www.raspberrypi.com/documentation/accessories/camera.html
- Camera software: https://www.raspberrypi.com/documentation/computers/camera_software.html
- Product brief (PDF): https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf
- Product page: https://www.raspberrypi.com/products/camera-module-3/
- Design files & schematics: https://pip.raspberrypi.com/categories/1207-design-files

## Raspberry Pi 5
- File: `RP-008348-DS-6-raspberry-pi-5-product-brief.pdf`
- Mechanical drawing: `KiCAD/RaspberryPi5_mechanical_drawing.pdf` (https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-mechanical-drawing.pdf)
- Docs: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
- Product brief (PDF): https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-product-brief.pdf
- Product page: https://www.raspberrypi.com/products/raspberry-pi-5/

## LDROBOT STL-19P LiDAR (D500 Kit)
- File: `LDROBOT_STL-19P_Datasheet_EN_v1.0.pdf`
- Wiki & setup guide: https://www.waveshare.com/wiki/D500_LiDAR_Kit
- Datasheet (PDF, Waveshare mirror): https://files.waveshare.com/wiki/D500-LiDAR-Kit/LDROBOT_STL-19P_Datasheet_EN_v1.0(1).pdf
- ROS/ROS2 driver: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros

## Power path modules (Pololu, Adafruit, DFRobot)
- INA260 breakout: https://www.adafruit.com/product/4226 · PCB source https://github.com/adafruit/Adafruit-INA260-PCB · TI https://www.ti.com/product/INA260
- Ideal diode #5382: https://www.pololu.com/product/5382 · TI LM74700-Q1 https://www.ti.com/product/LM74700-Q1
- D36V50F5 buck: https://www.pololu.com/product/4091
- RC switch with relay #2804: https://www.pololu.com/product/2804 · user's guide https://www.pololu.com/docs/0J60
- DFR0601: https://wiki.dfrobot.com/DFR0601
- VL53L4CD carrier #3692: https://www.pololu.com/product/3692
- 37D gearmotor #4753: https://www.pololu.com/product/4753
