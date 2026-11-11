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
from rv_sw import ROOT, app, read_globals  # noqa: E402

G = read_globals()

Q = lambda n: f'"{n}"'          # noqa: E731
VENDOR = os.path.join(ROOT, "COTS", "vendor")          # vendor CAD and datasheets the models are built from
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
    Origin = web face the hub bolts to (deep side), +y = outboard.
    Vendor STEP (rim + rubber tyre): axis along STEP X, web from X -2 to +2, shallow side toward -X (face at -18),
    deep side toward +X (face at +34). In ROS terms that is y = X, so turn 180 deg about z (deep side to -y) and
    shift +2 mm to put the deep web face on the origin."""
    c = Cots(sw, "RV-503", "Wheel_Wasteland_144x52", r"COTS\Wheels_Casters",
             "goBILDA Wasteland Wheel 144mm dia x 52mm wide (14mm bore, 16/32mm square patterns)", color=BLACK,
             step=os.path.join(VENDOR, "3616-0014-0144 Assembly.step"),
             Vendor="goBILDA / ServoCity", VendorPN="3616-0014-0144", Mass_g="348", MaxLoad_kg="6",
             DimSource="vendor STEP gobilda.com/content/step_files/3616-0014-0144.zip: d144 x 52, bore d14, "
                       "web 4 thick at 16 / 32 from the faces; deep recess d53 at the web to d69 at the face")
    c.move_bodies(rot=(("z", 180),), trans=(0.0, 2.0, 0.0))
    c.frame("IF_Mount")
    c.frame("IF_Center", (0, -6, 0))
    return c.finish()


def p504_caster(sw):
    """goBILDA swivel caster 3621-0001-0001, vendor STEP. Origin = hex-flange face (seats under the carriage or
    riser) on the swivel axis; M8 x 15 stud up +z, wheel below, trail toward -x like a trailing caster.
    In the STEP, up is -Z, the trail runs along +X and the axle along Y; swivel axis at (-56.855, 19.225),
    hex face at Z = -14, floor at Z = 53.02 (all mm)."""
    c = Cots(sw, "RV-504", "Caster_goBILDA_3621", r"COTS\Wheels_Casters",
             "goBILDA swivel caster, M8 x 1.25 male stud (3621-0001-0001)", color=DARK,
             step=os.path.join(VENDOR, "3621-0001-0001 assembly.STEP"),
             Vendor="goBILDA", VendorPN="3621-0001-0001", Mass_g="105",
             DimSource="vendor STEP gobilda.com/content/step_files/3621-0001-0001.zip: wheel d52.6 x 23.1, "
                       "trail 21.9, hex face to floor 67.0, M8 x 15 stud, hex 13.9 AF")
    c.move_bodies(rot=(("y", 90), ("z", 90)), trans=(-56.855, -19.225, -14.0))
    c.frame("IF_Mount")
    c.frame("IF_Axle", (-G["CASTER_TRAIL"], 0, G["CASTER_D"] / 2 - G["CASTER_H"]))
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
    c.feature("USB_C_Receptacle", "z", -19.5, [("rectc", (11.45, 0), 2.6, 8.4)], 7, +1, cut=True)
    c.frame("IF_Mount")
    c.frame("oak_rgb_lens", (22.9, 0, 0))
    c.frame("IF_USB", (11.45, 0, -19.5))                            # receptacle mouth, z into the camera
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
    """Luxonis Y-adapter (DM6010), vendor STEP. Plugs straight into the OAK's USB-C port; its two USB-C
    receptacles (data, power) then face down. STEP: plug along -Z, receptacles on the +Z face at X = +-9.
    IF_Mount: on the plug axis at the camera face (plug 6.6 mm inserted), z along the plug, y along its long side."""
    c = Cots(sw, "RV-514", "OAK_Y_Adapter", r"COTS\Sensors",
             "Luxonis OAK Y-adapter DM6010 (USB-C data + power split), plugs into the OAK", color=DARK,
             step=os.path.join(VENDOR, "y-adapter_20211129.stp"),
             Vendor="Luxonis", VendorPN="OAK-Y-ADAPTER", Mass_g="5",
             DimSource="vendor STEP github.com/luxonis/oak-hardware DM6010_Y-adapter/Mechanical/y-adapter_20211129.stp "
                       "(25.5 x 12.6 x 18.6); cable keep-outs assume right-angle USB-C plugs")
    c.feature("KeepOut_RA_USB_C", "x", 6.5, [("rectc", (sy * 9, 0), 6.5, 12.3) for sy in (1, -1)], 15, +1,
              merge=False)
    c.frame("IF_Mount", (-5.5, 0, 0), x="z", y="y")
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
    """Excel Cell ECS-22RE1RM2Q-020Q: big mushroom, push-lock turn-reset, 2 NC, panel-mounted through the deck.
    Origin = deck top face at the cutout centre, z up; the contact unit hangs below the deck."""
    from rv_sw import read_globals
    G = read_globals()
    T = G["DECK_T"]
    c = Cots(sw, "RV-551", "EStop_ECS-22RE1RM2Q-020Q", r"COTS\Electronics",
             "Emergency stop, 22 mm big mushroom, push-lock turn-reset, 2 NC, through the deck",
             color=(0.85, 0.1, 0.1), Vendor="Excel Cell Electronic", VendorPN="ECS-22RE1RM2Q-020Q",
             DigiKey="5916-ECS-22RE1RM2Q-020Q-ND", Mass_g="60",
             DimSource="datasheet ECS A31250 p9 (ECS-22RE1RM2Q-XXXQ): mushroom d41.5, 37.5 above the panel, "
                       "contact unit 35.7 x 50.8, 47.4 behind the panel face, cutout d22.3, panel 1-6 mm; "
                       "collar / stem split and mass APPROX")
    c.feature("Collar", "z", 0, [("circle", (0, 0), 30.0)], 10, +1)
    c.feature("Stem", "z", 10, [("circle", (0, 0), 20.0)], 5, +1)
    c.feature("Mushroom", "z", 15, [("circle", (0, 0), 41.5)], 22.5, +1)
    c.feature("Bushing", "z", 0, [("circle", (0, 0), 22.0)], T, -1)
    c.feature("ContactUnit", "z", -T, [("rectc", (0, 0), 35.7, 50.8)], 47.4 - T, -1)
    c.frame("IF_Mount")
    return c.finish()


def p553_bumper_switch(sw):
    """Omron D2F-01L-D (hinge lever, gold contacts, solder lugs), vendor STEP. In the STEP the body runs along X
    (-9.65..3.15), the lever rises along +Y (free position 10 above the hole line Y = 0), the M2 holes run along Z
    through the 5.8 thickness at X = 0 and -6.5. IF_Mount: on the +Z face at the middle of the hole line,
    x along the lever travel, z into the switch (so a bracket face carries it)."""
    c = Cots(sw, "RV-553", "Microswitch_D2F-01L-D", r"COTS\Electronics",
             "Omron D2F-01L-D subminiature switch, hinge lever, 0.1 A 30 VDC gold contacts (bumper)",
             color=BLACK, step=os.path.join(VENDOR, "D2F-01L-D.STEP"),
             Vendor="Omron (Aratas)", VendorPN="D2F-01L-D", DigiKey="SW1054-ND", Mass_g="0.6",
             DimSource="vendor STEP (DigiKey model 368442) + datasheet D2F_0318: FP 10 max, OP 6.8+-1.5, OT 0.55 min, "
                       "OF 0.78 N max, 2 x M2 holes 6.5 apart")
    c.frame("IF_Mount", (2.9, -3.25, 0), x="z", y="y")
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


def p554_caster_spring(sw):
    """Lee Spring LC 032E 05 S, stainless, squared and ground ends. Modelled at its installed length SPRING_L
    (carriage top to the bottom of the housing seat at rest) with the datasheet coil count; the ground end flats
    are not modelled. Origin = bottom end (on the carriage), +z up into the housing seat."""
    d, wire, L = G["SPRING_OD"] - G["SPRING_WIRE"], G["SPRING_WIRE"], G["SPRING_L"]
    c = Cots(sw, "RV-554", "Spring_LeeSpring_LC032E05S", r"COTS\Hardware",
             "Lee Spring LC 032E 05 S compression spring, 302 stainless (front caster, used x2)", color=SILVER,
             Vendor="Lee Spring", VendorPN="LC 032E 05 S", Mass_g="0.5",
             DimSource="leespring.com datasheet: OD 9.14, wire 0.81, free length 19.05, rate 1.751 N/mm, solid 4.7, "
                       "max load 23.34 N, 5.71 total coils; modelled at installed length")
    c.coil("Coil", d, wire, L, 5.71)
    c.frame("IF_Mount")
    c.frame("IF_Top", (0, 0, L))
    return c.finish()


PARTS = {"501": p501_motor, "502": p502_hub, "503": p503_wheel, "504": p504_caster, "511": p511_oak,
         "512": p512_lidar, "513": p513_tof, "514": p514_yadapter, "521": p521_pi5, "522": p522_esp32,
         "531": p531_driver, "532": p532_pcb_env, "541": p541_battery_env, "551": p551_estop,
         "552": p552_display, "553": p553_bumper_switch,
         "554": p554_caster_spring}


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
