"""Assemblies (Assemblies).

Every module is a subassembly whose own origin + `IF_Mount` frame is its interface. The top level fixes the
master layout and coordinate-mates each module's IF_Mount to the matching layout IF_* frame. Moving joints
keep their DOF: wheel spin (axis + face), caster swivel (axis + face), sprung front caster (limit distance
0..10 mm), battery drawer slide (limit distance 0..250 mm).

    python build_100_assemblies.py [110 120 ... 100 190]
"""
import os
import sys
import traceback

sys.path.insert(0, os.path.dirname(__file__))
from asm import Asm  # noqa: E402
from rv_sw import K, app, cast  # noqa: E402

A = "Assemblies"
COTS, SM, WELD, PR = "COTS", r"SheetMetal_Weldments\SheetMetal", r"SheetMetal_Weldments\Weldments", \
    "3DPrinted"
P = {
    "motor": rf"{COTS}\Motors\RV-501_Gearmotor_37D_70to1_Enc.SLDPRT",
    "hub": rf"{COTS}\Wheels_Casters\RV-502_SonicHub_1309_6mm.SLDPRT",
    "wheel": rf"{COTS}\Wheels_Casters\RV-503_Wheel_Wasteland_144x52.SLDPRT",
    "caster": rf"{COTS}\Wheels_Casters\RV-504_Caster_goBILDA_3621.SLDPRT",
    "oak": rf"{COTS}\Sensors\RV-511_OAK-D-Pro-W.SLDPRT",
    "lidar": rf"{COTS}\Sensors\RV-512_LiDAR_STL-19P.SLDPRT",
    "tof": rf"{COTS}\Sensors\RV-513_ToF_VL53L4CD_Adafruit5396.SLDPRT",
    "yad": rf"{COTS}\Sensors\RV-514_OAK_Y_Adapter.SLDPRT",
    "pi": rf"{COTS}\Compute\RV-521_RaspberryPi5_ActiveCooler.SLDPRT",
    "esp": rf"{COTS}\Compute\RV-522_ESP32_ExpansionBoard38P_DevKit.SLDPRT",
    "display": rf"{COTS}\Compute\RV-552_TouchDisplay2_7in.SLDPRT",
    "driver": rf"{COTS}\Electronics\RV-531_MotorDriver_DFR0601.SLDPRT",
    "pcb": rf"{COTS}\Electronics\RV-532_PowerPCB_Envelope.SLDPRT",
    "battery": rf"{COTS}\Electronics\RV-541_Battery4S_Envelope.SLDPRT",
    "estop": rf"{COTS}\Electronics\RV-551_EStop_ECS-22RE1RM2Q-020Q.SLDPRT",
    "frame": rf"{WELD}\RV-410_ChassisFrame_2020.SLDPRT",
    "tower": rf"{WELD}\RV-420_SensorTower_2020.SLDPRT",
    "deck": rf"{SM}\RV-430_DeckPlate.SLDPRT", "pan": rf"{SM}\RV-431_FloorPan.SLDPRT",
    "tray": rf"{SM}\RV-432_BatteryDrawerTray.SLDPRT", "guide": rf"{SM}\RV-433_DrawerGuide.SLDPRT",
    "bracket": rf"{SM}\RV-434_MotorBracket.SLDPRT", "pad": rf"{SM}\RV-435_ChargePad.SLDPRT",
    "head": rf"{PR}\RV-301_Head.SLDPRT", "bezel": rf"{PR}\RV-302_HeadBezel.SLDPRT",
    "wedge": rf"{PR}\RV-303_OAK_Wedge.SLDPRT", "pedestal": rf"{PR}\RV-304_LidarPedestal.SLDPRT",
    "bumper": rf"{PR}\RV-305_Bumper.SLDPRT", "padholder": rf"{PR}\RV-306_ChargePadHolder.SLDPRT",
    "cliffmount": rf"{PR}\RV-307_CliffSensorMount.SLDPRT", "housing": rf"{PR}\RV-308_FrontCasterHousing.SLDPRT",
    "carriage": rf"{PR}\RV-309_FrontCasterCarriage.SLDPRT", "riser": rf"{PR}\RV-310_RearCasterRiser.SLDPRT",
    "ctray": rf"{PR}\RV-311_ComputeTray.SLDPRT", "dmount": rf"{PR}\RV-312_DriverMount.SLDPRT",
    "skinside": rf"{PR}\RV-313_SkinSide.SLDPRT", "skinfront": rf"{PR}\RV-314_SkinFront.SLDPRT",
    "skinrear": rf"{PR}\RV-315_SkinRear.SLDPRT", "face": rf"{PR}\RV-316_DrawerFace.SLDPRT",
    "swbracket": rf"{PR}\RV-317_BumperSwitchBracket.SLDPRT",
    "bswitch": rf"{COTS}\Electronics\RV-553_Microswitch_D2F-01L-D.SLDPRT",
    "spring": rf"{COTS}\Hardware\RV-554_Spring_LeeSpring_LC032E05S.SLDPRT",
    "layout": r"Layout\RV-000_MasterLayout.SLDPRT", "dock": r"Layout\RV-001_ReferenceDock.SLDPRT",
}


def interface_frame(a):
    """IF_Mount at the assembly origin, x fwd / y left, so a parent can coordinate-mate this module."""
    pl = a.std_planes()
    fx = a.axis_from_planes("AX_Fwd", pl["top"], pl["right"])
    fy = a.axis_from_planes("AX_Left", pl["top"], pl["front"])
    a.coordinate_system("IF_Mount", "origin", fx, fy, (1, 0, 0), (0, 1, 0))


def finish(a, name, props, checks=()):
    interface_frame(a)
    a.doc.ForceRebuild3(False)
    a.set_props(**props)
    path = a.save_asm(rf"{A}\{name}.SLDASM")
    bad = a.unsolved()
    out = {"asm": name, "unsolved_mates": bad}
    for label, comp, cs in checks:
        out[label] = tuple(round(v, 1) for v in a.frame_world(comp, cs))
    a.sw.CloseDoc(a.title)
    a.close_sources()
    return out


def sub(sw, name, base_key, children, desc, flexible=False):
    """Generic subassembly: base part fixed at the origin; children = (key, child_cs, parent_key, parent_cs)."""
    a = Asm.new(sw)
    comps = {base_key: a.insert(P[base_key.split("#")[0]])}
    a.fix(comps[base_key])
    for key, ccs, pkey, pcs in children:
        comps[key] = a.insert(P[key.split("#")[0]])
        a.cs_mate(comps[key], ccs, comps[pkey], pcs, name=f"M_{key}_{pcs}")
    return a, comps


def a110_drive(sw):
    a, c = sub(sw, "RV-110", "bracket", [("motor", "IF_Mount", "bracket", "IF_Motor"),
                                          ("hub", "IF_Mount", "bracket", "IF_Hub")], "")
    c["wheel"] = a.insert(P["wheel"])
    a.axis_mate(c["wheel"], "AX_Y", c["hub"], "AX_Y", name="M_wheel_axis")
    a.plane_mate(c["wheel"], "Right Plane", c["hub"], "PL_Yp8", name="M_wheel_face")
    return finish(a, "RV-110_DriveModule", dict(PartNo="RV-110", Description="Drive module (used L and R)",
                                                 Source="ASSY", Revision="A"),
                  [("wheel_center", c["wheel"], "IF_Center"), ("shaft", c["motor"], "IF_Shaft")])


def a120_casterF(sw):
    a, c = sub(sw, "RV-120", "housing", [], "")
    c["carriage"] = a.insert(P["carriage"])
    a.plane_mate(c["carriage"], "Right Plane", c["housing"], "Right Plane", name="M_carriage_y")
    a.plane_mate(c["carriage"], "Front Plane", c["housing"], "Front Plane", name="M_carriage_x")
    a.limit_mate(c["carriage"], "Top Plane", c["housing"], "Top Plane", 0, 10, 0, name="M_spring_travel")
    for side in ("L", "R"):                       # rigid at installed length, hung from the housing seats
        c[f"spring{side}"] = a.insert(P["spring"])
        a.cs_mate(c[f"spring{side}"], "IF_Top", c["housing"], f"IF_Spring{side}", name=f"M_spring{side}")
    c["caster"] = a.insert(P["caster"])
    a.axis_mate(c["caster"], "AX_Z", c["carriage"], "AX_Z", name="M_swivel_axis")
    a.plane_mate(c["caster"], "Top Plane", c["carriage"], "Top Plane", name="M_swivel_face")
    return finish(a, "RV-120_CasterFront_Sprung", dict(PartNo="RV-120", Description="Front sprung caster module",
                                                        Source="ASSY", Revision="A"),
                  [("axle", c["caster"], "IF_Axle")])


def a125_casterR(sw):
    a, c = sub(sw, "RV-125", "riser", [], "")
    c["caster"] = a.insert(P["caster"])
    a.axis_mate(c["caster"], "AX_Z", c["riser"], "AX_Z", name="M_swivel_axis")
    a.plane_mate(c["caster"], "Top Plane", c["riser"], "Top Plane", name="M_swivel_face")
    return finish(a, "RV-125_CasterRear_Rigid", dict(PartNo="RV-125", Description="Rear rigid caster module",
                                                      Source="ASSY", Revision="A"),
                  [("axle", c["caster"], "IF_Axle")])


def simple(sw, name, pn, desc, base, children, checks=()):
    a, c = sub(sw, pn, base, children, desc)
    return finish(a, name, dict(PartNo=pn, Description=desc, Source="ASSY", Revision="A"),
                  [(lbl, c[k], cs) for lbl, k, cs in checks])


def a130_drawer(sw):
    return simple(sw, "RV-130_BatteryDrawer", "RV-130", "Battery drawer (tray + pack + face)", "tray",
                  [("battery", "IF_Mount", "tray", "IF_Battery"), ("face", "IF_Mount", "tray", "IF_Face")],
                  [("battery", "battery", "IF_Mount")])


def a140_tower(sw):
    return simple(sw, "RV-140_SensorTower", "RV-140", "Sensor tower: posts, head, bezel, OAK, display", "tower",
                  [("head", "IF_Mount", "tower", "IF_Head"), ("bezel", "IF_Mount", "head", "IF_Bezel"),
                   ("wedge", "IF_Mount", "bezel", "IF_Wedge"), ("oak", "IF_Mount", "wedge", "IF_OAK"),
                   ("display", "IF_Mount", "head", "IF_Display"), ("yad", "IF_Mount", "oak", "IF_USB")],
                  [("oak_lens", "oak", "oak_rgb_lens")])


def a150_compute(sw):
    return simple(sw, "RV-150_ComputeBay", "RV-150", "Compute bay: Pi 5 + ESP32 expansion board", "ctray",
                  [("pi", "IF_Mount", "ctray", "IF_Pi"), ("esp", "IF_Mount", "ctray", "IF_ESP32")])


def a160_driver(sw):
    return simple(sw, "RV-160_DriverBay", "RV-160", "Motor driver bay", "dmount",
                  [("driver", "IF_Mount", "dmount", "IF_Driver")])


def a170_lidar(sw):
    return simple(sw, "RV-170_LidarModule", "RV-170", "LiDAR on pedestal", "pedestal",
                  [("lidar", "IF_Mount", "pedestal", "IF_Lidar")], [("laser", "lidar", "laser")])


def a180_cliff(sw):
    return simple(sw, "RV-180_CliffSensor", "RV-180", "Cliff sensor module (x4)", "cliffmount",
                  [("tof", "IF_Mount", "cliffmount", "IF_Sensor")])


def a182_bumperswitch(sw):
    return simple(sw, "RV-182_BumperSwitch", "RV-182", "Bumper switch module (x2): bracket + Omron D2F", "swbracket",
                  [("bswitch", "IF_Mount", "swbracket", "IF_Switch")])


def a185_pads(sw):
    return simple(sw, "RV-185_ChargePads", "RV-185", "Charging pads in holder", "padholder",
                  [("pad#L", "IF_Mount", "padholder", "IF_PadL"), ("pad#R", "IF_Mount", "padholder", "IF_PadR")])


SUBS = {"drive": r"Assemblies\RV-110_DriveModule.SLDASM", "casterF": r"Assemblies\RV-120_CasterFront_Sprung.SLDASM",
        "casterR": r"Assemblies\RV-125_CasterRear_Rigid.SLDASM", "drawer": r"Assemblies\RV-130_BatteryDrawer.SLDASM",
        "towerA": r"Assemblies\RV-140_SensorTower.SLDASM", "compute": r"Assemblies\RV-150_ComputeBay.SLDASM",
        "driverbay": r"Assemblies\RV-160_DriverBay.SLDASM", "lidarmod": r"Assemblies\RV-170_LidarModule.SLDASM",
        "cliff": r"Assemblies\RV-180_CliffSensor.SLDASM", "pads": r"Assemblies\RV-185_ChargePads.SLDASM",
        "bswitch": r"Assemblies\RV-182_BumperSwitch.SLDASM"}


def a100_top(sw):
    a = Asm.new(sw)
    lay = a.insert(P["layout"])
    a.fix(lay)
    c = {}
    place = [  # (key, path, layout frame, flexible)
        ("frame", P["frame"], "base_footprint", False), ("pan", P["pan"], "IF_FloorPan", False),
        ("deck", P["deck"], "IF_Deck", False), ("guideL", P["guide"], "IF_GuideL", False),
        ("guideR", P["guide"], "IF_GuideR", False), ("skinL", P["skinside"], "IF_SkinL", False),
        ("skinR", P["skinside"], "IF_SkinR", False), ("skinF", P["skinfront"], "IF_SkinFront", False),
        ("skinB", P["skinrear"], "IF_SkinRear", False), ("bumper", P["bumper"], "IF_Bumper", False),
        ("pcb", P["pcb"], "IF_Power", False), ("estop", P["estop"], "IF_EStop", False),
        ("driveL", SUBS["drive"], "IF_DriveL", True), ("driveR", SUBS["drive"], "IF_DriveR", True),
        ("casterF", SUBS["casterF"], "IF_CasterF", True), ("casterR", SUBS["casterR"], "IF_CasterR", True),
        ("tower", SUBS["towerA"], "IF_Tower", False), ("lidar", SUBS["lidarmod"], "IF_LidarPedestal", False),
        ("compute", SUBS["compute"], "IF_Compute", False), ("driverbay", SUBS["driverbay"], "IF_MotorDriver", False),
        ("cliffFL", SUBS["cliff"], "IF_Cliff_FL", False), ("cliffFR", SUBS["cliff"], "IF_Cliff_FR", False),
        ("cliffRL", SUBS["cliff"], "IF_Cliff_RL", False), ("cliffRR", SUBS["cliff"], "IF_Cliff_RR", False),
        ("bswL", SUBS["bswitch"], "IF_BumperSwL", False), ("bswR", SUBS["bswitch"], "IF_BumperSwR", False),
        ("pads", SUBS["pads"], "IF_ChargePads", False),
    ]
    for key, path, frame, flex in place:
        c[key] = a.insert(path)
        if flex:
            a.flexible(c[key])
        a.cs_mate(c[key], "IF_Mount", lay, frame, name=f"M_{key}")
    c["drawer"] = a.insert(SUBS["drawer"])
    a.plane_mate(c["drawer"], "Top Plane", lay, "PL_IF_DRAWER_Z", name="M_drawer_height")
    a.plane_mate(c["drawer"], "Right Plane", lay, "Right Plane", name="M_drawer_centre")
    a.limit_mate(c["drawer"], "Front Plane", lay, "Front Plane", 0, 250, 0, name="M_drawer_slide")
    lay.Visible = K.swComponentHidden                      # reference geometry only: hidden, not in the BOM
    lay.ExcludeFromBOM = True
    return finish(a, "RV-100_RoamView", dict(PartNo="RV-100", Description="RoamView top-level assembly",
                                                   Source="ASSY", Revision="A"),
                  [("laser", c["lidar"], "IF_Mount")])


def a190_dock(sw):
    a = Asm.new(sw)
    top = a.insert(r"Assemblies\RV-100_RoamView.SLDASM")
    a.fix(top)
    dock = a.insert(P["dock"])
    for pl in ("Front Plane", "Top Plane", "Right Plane"):
        a.plane_mate(dock, pl, top, pl, name=f"M_dock_{pl.split()[0]}")
    return finish(a, "RV-190_DockingCheck", dict(PartNo="RV-190", Description="Robot docked on reference dock",
                                                  Source="ASSY", Revision="A"))


BUILD = {"110": a110_drive, "120": a120_casterF, "125": a125_casterR, "130": a130_drawer, "140": a140_tower,
         "150": a150_compute, "160": a160_driver, "170": a170_lidar, "180": a180_cliff, "182": a182_bumperswitch, "185": a185_pads,
         "100": a100_top, "190": a190_dock}

if __name__ == "__main__":
    sw = app()
    for k in sys.argv[1:] or list(BUILD):
        try:
            print("OK  ", BUILD[k](sw))
        except Exception as e:
            traceback.print_exc()
            print(f"FAIL RV-{k}: {e}")
            for d in (sw.GetDocuments() or []):
                doc = cast(d, "IModelDoc2")
                if doc.GetPathName() == "":
                    sw.QuitDoc(doc.GetTitle())
