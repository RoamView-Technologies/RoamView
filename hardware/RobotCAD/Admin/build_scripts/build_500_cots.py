"""Build the RoamView COTS and envelope parts (COTS/...).

    python build_500_cots.py            # all parts
    python build_500_cots.py 501 511    # only these part numbers

Simplified bodies, real interfaces. `DimSource` on every part says where its numbers came from;
anything marked APPROX must be checked against the physical part before printing mounts for it.
Local frame: x forward, y left, z up (part's own sense), origin = mounting interface (IF_Mount).
"""
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(__file__))
from cots import Cots  # noqa: E402
from rv_sw import app  # noqa: E402

Q = lambda n: f'"{n}"'          # noqa: E731
BLACK, DARK, GREY, SILVER = (0.08, 0.08, 0.09), (0.18, 0.18, 0.2), (0.55, 0.56, 0.58), (0.78, 0.79, 0.8)


def p501_motor(sw):
    """Pololu #4754 37D 70:1 gearmotor, 12 V, 64 CPR encoder. Origin = gearbox face centre, shaft along +y."""
    c = Cots(sw, "RV-501", "Gearmotor_37D_70to1_Enc", r"COTS\Motors",
             "Pololu 37D 70:1 metal gearmotor 12V with 64 CPR encoder (helical pinion)", color=GREY,
             Vendor="Pololu", VendorPN="4754", Mass_g="215",
             DimSource="datasheet Pololu_37D_gearmotor_dimensions.pdf p2 (L=24 for 70:1); hole clocking APPROX")
    c.feature("Gearbox", "y", 0, [("circle", (0, 0), 36.8)], 24, -1)
    c.feature("Motor", "y", -24, [("circle", (0, 0), 34.8)], 30.7, -1)
    c.feature("Encoder", "y", -54.7, [("circle", (0, 0), 34.0)], 15.4, -1)
    c.feature("Bushing", "y", 0, [("circle", (0, 7), 12.0)], 6, +1)
    c.feature("Shaft", "y", 6, [("circle", (0, 7), 6.0)], 16, +1)
    c.feature("MountHoles_6xM3", "y", 0, c.bolt_circle((0, 0), 31.0, 6, 2.5), 3, -1, cut=True)
    c.frame("IF_Mount")
    c.frame("IF_Shaft", (0, 0, 7))
    return c.finish()


def p502_hub(sw):
    """goBILDA 1309-0016-0006 Sonic Hub, 6 mm round bore, M4 on the 16 mm square pattern.
    Origin = motor-side face, +y toward the wheel."""
    c = Cots(sw, "RV-502", "SonicHub_1309_6mm", r"COTS\Wheels_Casters",
             "goBILDA 1309 Series Sonic Hub 6mm bore (M4, 16mm square pattern)", color=SILVER,
             Vendor="goBILDA", VendorPN="1309-0016-0006", Mass_g="14",
             DimSource="vendor: 8mm thick, 6mm bore, 4xM4 on 16mm square (pattern per goBILDA 3616 drawing); "
                       "OD 32 APPROX")
    c.feature("Body", "y", 0, [("circle", (0, 0), 32.0)], 8, +1)
    c.feature("Bore", "y", 0, [("circle", (0, 0), 6.0)], 8, +1, cut=True)
    c.feature("Pattern16_4xM4", "y", 8, [("circle", (sx * 8, sz * 8), 3.3) for sx in (1, -1) for sz in (1, -1)],
              6, -1, cut=True)
    c.frame("IF_Mount")
    c.frame("IF_WheelFace", (0, 8, 0))
    return c.finish()


def p503_wheel(sw):
    """goBILDA 3616-0014-0144 Wasteland Wheel, 144 x 52. The 4 mm mounting web sits 16 mm from one face and
    32 mm from the other. Mounted deep-side toward the motor, so the hub sits inside the wheel and the wheel
    centre plane is only WHEEL_WEB_OFF (6 mm) inboard of the web.
    Origin = web face the hub bolts to (deep side), +y = outboard."""
    c = Cots(sw, "RV-503", "Wheel_Wasteland_144x52", r"COTS\Wheels_Casters",
             "goBILDA Wasteland Wheel 144mm dia x 52mm wide (14mm bore, 16/32mm square patterns)", color=BLACK,
             Vendor="goBILDA / ServoCity", VendorPN="3616-0014-0144", Mass_g="348", MaxLoad_kg="6",
             DimSource="goBILDA 3616-0014-0144 schematic: d144 x 52, bore d14, holes d4 on 16 and 32 square, "
                       "web 4 thick at 16 / 32 from the faces; recess d76 measured off the section (APPROX)")
    c.feature("Tyre", "y", -32, [("circle", (0, 0), 144.0)], 52, +1)
    c.feature("RecessDeep", "y", -32, [("circle", (0, 0), 76.0)], 32, +1, cut=True)
    c.feature("RecessShallow", "y", 20, [("circle", (0, 0), 76.0)], 16, -1, cut=True)
    c.feature("WebHoles", "y", 0, [("circle", (0, 0), 14.0)]
              + [("circle", (sx * 8, sz * 8), 4.0) for sx in (1, -1) for sz in (1, -1)]
              + [("circle", (sx * 16, sz * 16), 4.0) for sx in (1, -1) for sz in (1, -1)], 4, +1, cut=True)
    c.frame("IF_Mount")
    c.frame("IF_Center", (0, -6, 0))
    return c.finish()


def p504_caster(sw):
    """2-inch swivel caster placeholder. Origin = top of mounting plate, wheel below. Total height 68."""
    c = Cots(sw, "RV-504", "Caster_2in_Swivel", r"COTS\Wheels_Casters",
             "2 inch swivel caster, top plate 40x40 (model TBD)", color=DARK,
             Vendor="TBD", VendorPN="TBD", Mass_g="60",
             DimSource="APPROX: generic 2in caster, height = CASTER_H 68, trail 10")
    c.feature("Plate", "z", 0, [("rectc", (0, 0), 40, 40)], 3, -1)
    c.feature("PlateHoles_4x", "z", 0, [("circle", (sx * 15, sy * 15), 4.5) for sx in (1, -1) for sy in (1, -1)],
              3, -1, cut=True)
    c.feature("Swivel", "z", -3, [("circle", (0, 0), 30.0)], 12, -1)
    c.feature("Fork", "z", -15, [("rect", (-22, -13), (4, 13))], 20, -1)
    c.feature("Wheel", "y", -10, [("circle", (-10, -43), 50.0)], 20, +1)
    c.frame("IF_Mount")
    c.frame("IF_Axle", (-10, 0, -43))
    return c.finish()


def p511_oak(sw):
    """Luxonis OAK-D Pro W. Origin = back (VESA) face centre at lens height, +x = viewing direction."""
    c = Cots(sw, "RV-511", "OAK-D-Pro-W", r"COTS\Sensors",
             "Luxonis OAK-D Pro W (USB), with USB-C plug keep-out", color=BLACK,
             Vendor="Luxonis", VendorPN="OAK-D-PRO-W", Mass_g="91",
             DimSource="datasheet Luxonis DM9098 sec 9: 97x29.5x22.9, cams 19.5 from bottom, VESA 2xM4 75; "
                       "VESA height, USB-C and tripod positions APPROX")
    c.feature("Body", "x", 0, [("rect", (-48.5, -19.5), (48.5, 10))], 22.9, +1)
    c.feature("VESA_2xM4", "x", 0, [("circle", (sy * 37.5, -4.75), 3.3) for sy in (1, -1)], 8, +1, cut=True)
    c.feature("Lenses", "x", 22.9, [("circle", (37.5, 0), 9.0), ("circle", (-37.5, 0), 9.0),
                                    ("circle", (0, 0), 9.0), ("rectc", (17, 0), 14, 10)], 1, -1, cut=True)
    c.feature("KeepOut_USB_C", "z", -19.5, [("rectc", (11.45, 0), 8, 13)], 25, -1)
    c.frame("IF_Mount")
    c.frame("oak_rgb_lens", (22.9, 0, 0))
    return c.finish()


def p512_lidar(sw):
    """LDROBOT STL-19P (D500 kit). Origin = underside of the ears at the spin axis; laser plane 27.2 above."""
    c = Cots(sw, "RV-512", "LiDAR_STL-19P", r"COTS\Sensors",
             "LDROBOT STL-19P / D500 LiDAR", color=DARK,
             Vendor="LDROBOT", VendorPN="STL-19P", Mass_g="45",
             DimSource="datasheet LDROBOT STL-19P v1.0 sec 5.1: 54x46.29x35, dome 35.29x12.8, window 7.8 below top, "
                       "3x d2.5 holes; hole x-positions APPROX (+-1)")
    c.feature("Housing", "z", 0, [("rectc", (0, 0), 38.59, 38.59)], 22.2, +1)
    c.feature("Ears", "z", 0, [("rect", (1.33, 19.0), (8.53, 27.0)), ("rect", (1.33, -27.0), (8.53, -19.0)),
                               ("rect", (-27.0, -3.6), (-19.0, 3.6))], 2, +1)
    c.feature("Holes_3x", "z", 0, [("circle", (4.93, 23.4), 2.5), ("circle", (4.93, -23.4), 2.5),
                                   ("circle", (-23.39, 0), 2.5)], 2, +1, cut=True)
    c.feature("Dome", "z", 22.2, [("circle", (0, 0), 35.29)], 12.8, +1)
    c.frame("IF_Mount")
    c.frame("laser", (0, 0, 27.2))
    return c.finish()


def p513_tof(sw):
    """Adafruit 5396 VL53L4CD STEMMA QT. Origin = back of board centre, +z = sensing direction."""
    c = Cots(sw, "RV-513", "ToF_VL53L4CD_Adafruit5396", r"COTS\Sensors",
             "Adafruit VL53L4CD time-of-flight sensor, STEMMA QT", color=(0.1, 0.2, 0.55),
             Vendor="Adafruit", VendorPN="5396", Mass_g="1.8",
             DimSource="vendor: 25.5x17.7x4.6; mounting holes and connector side APPROX - measure before printing")
    c.feature("Board", "z", 0, [("rectc", (0, 0), 25.5, 17.7)], 1.6, +1)
    c.feature("Sensor", "z", 1.6, [("rectc", (0, 0), 4.4, 2.4)], 1.0, +1)
    c.feature("KeepOut_STEMMA", "z", 0, [("rectc", (9.0, 0), 6, 4.3), ("rectc", (-9.0, 0), 6, 4.3)], 3, -1)
    c.frame("IF_Mount")
    c.frame("range", (0, 0, 2.6), x="z", y="y")
    return c.finish()


def p514_yadapter(sw):
    c = Cots(sw, "RV-514", "OAK_Y_Adapter", r"COTS\Sensors",
             "Luxonis OAK Y-adapter (USB-C data + power split), envelope", color=DARK,
             Vendor="Luxonis", VendorPN="OAK-Y-ADAPTER", Mass_g="10",
             DimSource="APPROX envelope 34x24x10")
    c.feature("Envelope", "z", 0, [("rectc", (0, 0), 34, 24)], 10, +1)
    c.frame("IF_Mount")
    return c.finish()


def p521_pi5(sw):
    """Raspberry Pi 5 + active cooler. Origin = board underside centre; ports at +x, GPIO along +y."""
    c = Cots(sw, "RV-521", "RaspberryPi5_ActiveCooler", r"COTS\Compute",
             "Raspberry Pi 5 with active cooler", color=(0.1, 0.45, 0.2),
             Vendor="Raspberry Pi", VendorPN="SC1112 + SC1148", Mass_g="72",
             DimSource="datasheet RaspberryPi5_mechanical_drawing.pdf: 85x56, holes d2.7 58x49 inset 3.5; "
                       "component heights APPROX")
    c.feature("Board", "z", 0, [("rectc", (0, 0), 85, 56)], 1.6, +1)
    c.feature("MountHoles_4x", "z", 0, [("circle", (x, y), 2.7) for x in (-39, 19) for y in (-24.5, 24.5)],
              1.6, +1, cut=True)
    c.feature("Ports_USB_ETH", "z", 1.6, [("rect", (21.5, -26), (45.5, 26))], 16, +1)
    c.feature("ActiveCooler", "z", 1.6, [("rect", (-34, -22), (14, 17))], 9, +1)
    c.feature("GPIO", "z", 1.6, [("rectc", (-10, 24.5), 51, 5)], 8.5, +1)
    c.frame("IF_Mount")
    return c.finish()


def p522_esp32(sw):
    """ESP32-S 38-pin expansion board (header carrier) with the team's ESP32-WROOM-32E USB-C dev board plugged in.
    Origin = carrier underside centre, long side along x, dev-board USB-C toward +x."""
    c = Cots(sw, "RV-522", "ESP32_ExpansionBoard38P_DevKit", r"COTS\Compute",
             "ESP32-S Expansion Board 38-Pin (header carrier, DC jack) + ESP32-WROOM-32E USB-C dev board",
             color=(0.12, 0.22, 0.45), Vendor="ProtoSupplies (generic ESP32-S 38P expansion board)",
             VendorPN="ESP32-S Expansion Board 38-Pin", Mass_g="35",
             DimSource="carrier 77x54, 4x M4 (ProtoSupplies); dev board ~28x56, 1.0in row spacing, measured from "
                       "team photo; hole positions, header and jack placement APPROX")
    c.feature("Carrier", "z", 0, [("rectc", (0, 0), 77, 54)], 1.6, +1)
    c.feature("MountHoles_4xM4", "z", 0, [("circle", (sx * 34.5, sy * 23.5), 4.2) for sx in (1, -1) for sy in (1, -1)],
              1.6, +1, cut=True)
    c.feature("FemaleHeaders", "z", 1.6, [("rectc", (0, 12.7), 48.3, 5), ("rectc", (0, -12.7), 48.3, 5)], 8.5, +1)
    c.feature("GPIO_Headers", "z", 1.6, [("rectc", (0, 22.5), 48.3, 5), ("rectc", (0, -22.5), 48.3, 5)], 8.5, +1)
    c.feature("DC_Jack", "z", 1.6, [("rect", (-38.5, -26), (-24.5, -17))], 11, +1)
    c.feature("DevKit", "z", 10.1, [("rectc", (0, 0), 55.7, 28)], 1.6, +1)
    c.feature("WROOM_Module", "z", 11.7, [("rect", (-27.85, -9), (-2.35, 9))], 3.1, +1)
    c.feature("KeepOut_USB_C", "z", 11.7, [("rect", (27.85, -6), (47.85, 6))], 7, +1)
    c.frame("IF_Mount")
    return c.finish()


def p531_driver(sw):
    c = Cots(sw, "RV-531", "MotorDriver_DFR0601", r"COTS\Electronics",
             "DFRobot DFR0601 dual-channel 12A DC motor driver", color=(0.55, 0.12, 0.2),
             Vendor="DFRobot", VendorPN="DFR0601", Mass_g="30",
             DimSource="datasheet DFRobot_DFR0601_dimensions.png: 50x50x12.5, 4x d3 on 45x45")
    c.feature("Board", "z", 0, [("rectc", (0, 0), 50, 50)], 1.6, +1)
    c.feature("MountHoles_4x", "z", 0, [("circle", (sx * 22.5, sy * 22.5), 3.0) for sx in (1, -1) for sy in (1, -1)],
              1.6, +1, cut=True)
    c.feature("Components", "z", 1.6, [("rectc", (0, 0), 38, 44)], 9.3, +1)
    c.feature("Pins", "z", 0, [("rectc", (0, 0), 40, 40)], 1.6, -1)
    c.frame("IF_Mount")
    return c.finish()


def p532_pcb_env(sw):
    """Reserved volume for the future power PCB in its protective case. Size follows the PCB_* globals."""
    c = Cots(sw, "RV-532", "PowerPCB_Envelope", r"COTS\Electronics",
             "Power/battery/protection PCB in case - RESERVED ENVELOPE (PCB to be designed)",
             source="ENVELOPE", color=(0.48, 0.31, 0.69), transparency=0.4, link_globals=True,
             Mass_g="150", DimSource="globals PCB_L x PCB_W x PCB_H",
             Contents="K1 CB1AF relay, K2 relay + RC receiver, 3x INA260, 2x 5V buck, 3x ideal diode, 4700uF")
    from rv_sw import read_globals
    G = read_globals()
    c.feature("Envelope", "z", 0, [("rectc", (0, 0), G["PCB_L"], G["PCB_W"], {"w": Q("PCB_L"), "h": Q("PCB_W")})],
              G["PCB_H"], +1, depth_expr=Q("PCB_H"))
    c.frame("IF_Mount")
    return c.finish()


def p541_battery_env(sw):
    """4S pack envelope with an XT60 at the front (blind-mate end). Size follows the BAT_* globals."""
    from rv_sw import read_globals
    G = read_globals()
    c = Cots(sw, "RV-541", "Battery4S_Envelope", r"COTS\Electronics",
             "4S battery pack ENVELOPE with XT60 (pack model TBD)", source="ENVELOPE", color=(0.18, 0.43, 0.33),
             transparency=0.25, link_globals=True, Mass_g="900",
             DimSource="globals BAT_L x BAT_W x BAT_H; XT60 15.8x8.2x16; pack APPROX until chosen")
    c.feature("Pack", "z", 0, [("rectc", (0, 0), G["BAT_L"], G["BAT_W"], {"w": Q("BAT_L"), "h": Q("BAT_W")})],
              G["BAT_H"], +1, depth_expr=Q("BAT_H"))
    L2 = G["BAT_L"] / 2
    c.feature("XT60", "z", 0, [("rect", (L2, -4.1), (L2 + 16, 4.1), {"pu": f'{Q("BAT_L")} / 2'})], 15.8, +1)
    c.frame("IF_Mount")
    return c.finish()


def p551_estop(sw):
    c = Cots(sw, "RV-551", "EStop_22mm_Box", r"COTS\Electronics",
             "Emergency stop, 22mm mushroom in 45x45 enclosure (model TBD)", color=(0.95, 0.76, 0.19),
             Vendor="TBD", VendorPN="TBD", Mass_g="80", DimSource="APPROX: height 45 so top = ESTOP_TOP")
    c.feature("Box", "z", 0, [("rectc", (0, 0), 45, 45)], 18, +1)
    c.feature("Collar", "z", 18, [("circle", (0, 0), 24)], 14, +1)
    c.feature("Mushroom", "z", 32, [("circle", (0, 0), 32)], 13, +1)
    c.frame("IF_Mount")
    return c.finish()


def p552_display(sw):
    """Raspberry Pi Touch Display 2 (7in). Origin = back face centre, +z = viewing side, long side along y."""
    c = Cots(sw, "RV-552", "TouchDisplay2_7in", r"COTS\Compute",
             "Raspberry Pi Touch Display 2, 7in 720x1280", color=(0.1, 0.12, 0.16),
             Vendor="Raspberry Pi", VendorPN="SC1635", Mass_g="270",
             DimSource="vendor 189.32x120.24, active 155x88; thickness 12 and hole pattern APPROX - "
                       "use the official mechanical drawing before printing the head")
    c.feature("Body", "z", 0, [("rectc", (0, 0), 120.24, 189.32)], 12, +1)
    c.feature("ActiveArea", "z", 12, [("rectc", (0, 0), 88, 155)], 0.5, -1, cut=True)
    c.frame("IF_Mount")
    return c.finish()


PARTS = {"501": p501_motor, "502": p502_hub, "503": p503_wheel, "504": p504_caster, "511": p511_oak,
         "512": p512_lidar, "513": p513_tof, "514": p514_yadapter, "521": p521_pi5, "522": p522_esp32,
         "531": p531_driver, "532": p532_pcb_env, "541": p541_battery_env, "551": p551_estop,
         "552": p552_display}


def main():
    sw = app()
    wanted = sys.argv[1:] or list(PARTS)
    results = []
    for k in wanted:
        try:
            r = PARTS[k](sw)
            results.append(r)
            print(f"OK   {r['part']:40s} sketches={r['sketches']:2d} not_fully_defined={r['not_fully_defined']}")
        except Exception as e:
            traceback.print_exc()
            print(f"FAIL RV-{k}: {e}")
            results.append(dict(part=f"RV-{k}", error=str(e)))
            for d in (sw.GetDocuments() or []):            # leave nothing half-built open
                from rv_sw import cast
                doc = cast(d, "IModelDoc2")
                if doc.GetPathName() == "":
                    sw.QuitDoc(doc.GetTitle())
    bad = [r for r in results if r.get("error") or r.get("not_fully_defined")]
    print(f"\n{len(results) - len(bad)}/{len(results)} parts clean")


if __name__ == "__main__":
    main()
