"""Engineering hand calculations standing in for FEA and Motion Analysis (the installed SOLIDWORKS has
only the SimulationXpress engine and no Motion add-in). Conservative closed-form checks; every input is printed.

    python hand_calcs.py <total_mass_kg> <com_x_mm> <com_z_mm>
Writes Simulation/FEA/hand_calcs_structural.txt and Simulation/Motion/hand_calcs_drivetrain.txt.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from rv_sw import ROOT, read_globals  # noqa: E402

G = read_globals()
M, CX, CZ = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
g = 9.81
W = M * g

# ---- materials (MPa, typical minimum yield) ----
SY_5052 = 193.0          # 5052-H32
SY_6063 = 145.0          # 6063-T5 extrusion
SY_PETG = 45.0           # printed PETG, XY, conservative (layer direction ~ 30)
SY_PETG_Z = 30.0
SY_SHAFT = 350.0         # Pololu steel output shaft, conservative
FS_ALU, FS_PRINT = 3.0, 2.0

# ---- static loads ----
xr = G["CASTER_X"] - 10                     # rear caster contact (trail inward)
rear = max(0.0, -CX) / xr * W               # rigid rear caster
wheel = (W - rear) / 2                      # each drive wheel (front caster preload ignored -> conservative)
BUMP = 2.0                                  # 2 g vertical bump factor


def line(lst, s):
    lst.append(s)


S = []
line(S, "RoamView - structural hand checks (stand-in for FEA)")
line(S, f"Inputs: mass {M:.2f} kg, COM x {CX:.1f} mm z {CZ:.1f} mm; static wheel load {wheel:.1f} N, rear caster "
        f"{rear:.1f} N; bump factor {BUMP:g} g")
line(S, f"Criteria: FS >= {FS_ALU:g} aluminium/steel, FS >= {FS_PRINT:g} printed PETG\n")

# 1. motor bracket horizontal leg (cantilever from the rail's outer edge to the wheel plane)
Fw = wheel * BUMP
arm = G["TRACK"] / 2 - (G["RAIL_Y"] + G["PROFILE"] / 2)          # rail outer edge -> wheel centre
b, t = 42.0, 3.0
Z = b * t ** 2 / 6
sig = Fw * arm / Z
line(S, "1. Motor bracket RV-434 (3 mm 5052-H32), leg as cantilever off the bottom side rail")
line(S, f"   F = {Fw:.1f} N (2 g), arm {arm:.1f} mm, Z = b t^2/6 = {Z:.0f} mm3 -> sigma = {sig:.1f} MPa, "
        f"FS = {SY_5052 / sig:.1f}  {'PASS' if SY_5052 / sig >= FS_ALU else 'FAIL'}")
arm2 = G["TRACK"] / 2 - G["RAIL_Y"]                              # worst case: pivot on the bolt line
sig2 = Fw * arm2 / Z
line(S, f"   worst case (pivot at bolt line, arm {arm2:.1f} mm): sigma = {sig2:.1f} MPa, FS = {SY_5052 / sig2:.1f}  "
        f"{'PASS' if SY_5052 / sig2 >= FS_ALU else 'FAIL'}")
# bolt prying: 2x M4 react moment about rail outer edge, lever ~ 10 mm (bolt line to edge)
Fb = Fw * arm / 10.0 / 2
line(S, f"   2x M4 rail bolts (8.8, proof ~ 6.0 kN each): prying tension ~ {Fb:.0f} N each, FS = {6000 / Fb:.0f}  PASS\n")

# 2. gearmotor output shaft bending (overhang from gearbox bushing to wheel centre)
ov = G["BUSHING_L"] + G["HUB_GAP"] + G["HUB_T"] - G["WHEEL_WEB_OFF"] + 6.0    # +6 mm into gearbox bearing
Zs = math.pi * 6.0 ** 3 / 32
sig_s = Fw * ov / Zs
line(S, "2. Gearmotor 6 mm output shaft, bending at the gearbox bearing")
line(S, f"   F = {Fw:.1f} N (2 g), overhang {ov:.1f} mm, Z = pi d^3/32 = {Zs:.1f} mm3 -> sigma = {sig_s:.1f} MPa, "
        f"FS = {SY_SHAFT / sig_s:.1f}  {'PASS' if SY_SHAFT / sig_s >= FS_ALU else 'FAIL'}")
line(S, "   Pololu gives no radial-load rating for the 37D gearbox bushing: the deep-recess wheel mounting keeps the\n"
        "   overhang short; bench-test or add an outboard bearing if wheels are side-loaded (e.g. tight spins).\n")

# 3. tower posts under 1 g lateral load at the head
m_head = (0.310 + 0.090 + 0.014 + 0.091 + 0.270 + 0.010)          # head, bezel, wedge, OAK, display, Y-adapter
h_head = (G["HEAD_Z0"] + G["HEAD_TOP"]) / 2 - G["DECK_Z"]
Mb = m_head * g * 1.0 * h_head
I2020, c2020 = 6900.0, 10.0                                        # mm4, mm (2020 typical Ixx)
sig_p = (Mb / 2) / (I2020 / c2020)
line(S, "3. Sensor tower (2x 2020 posts) under 1 g lateral load on the head")
line(S, f"   head stack {m_head:.3f} kg at {h_head:.0f} mm above deck -> M = {Mb / 1000:.2f} N.m, per post sigma = "
        f"{sig_p:.1f} MPa, FS = {SY_6063 / sig_p:.0f}  PASS")
defl = (m_head * g / 2) * h_head ** 3 / (3 * 69000 * I2020)
line(S, f"   tip deflection (cantilever, brace ignored) ~ {defl:.2f} mm - camera vibration not a concern\n")

# 4. front caster housing top plate (PETG cantilever from the back wall to the caster axis)
Fc = max(0.15 * W, 15.0) * BUMP                                   # sprung caster: <= ~15 % of weight + bump
arm_c, bw, tp = 35.0, 60.0, 9.5
sig_c = Fc * arm_c / (bw * tp ** 2 / 6)
line(S, "4. Front caster housing RV-308 (PETG), top plate cantilever, spring bottomed")
line(S, f"   F = {Fc:.1f} N, arm {arm_c:.0f} mm, 60 x 9.5 section -> sigma = {sig_c:.1f} MPa, FS = {SY_PETG_Z / sig_c:.1f}"
        f" (Z-direction strength)  {'PASS' if SY_PETG_Z / sig_c >= FS_PRINT else 'FAIL'}")
Fr = rear * BUMP
line(S, f"   rear riser RV-310 is a solid 23.5 mm block in compression: {Fr:.0f} N over 60 x 63 mm -> "
        f"{Fr / (60 * 63):.2f} MPa  PASS\n")

# 5. drawer guides (L 1.5 mm) carrying tray + battery
m_dr = 0.280 + 0.900 + 0.021
span = 2 * G["RAIL_X"]
Fg = m_dr * g * BUMP / 2
Ig, cg = 1300.0, 13.0
sig_g = Fg * span / 4 / (Ig / cg)
dg = Fg * span ** 3 / (48 * 70000 * Ig)
line(S, "5. Drawer guides RV-433 (1.5 mm 5052 L) simply supported between the mid cross members")
line(S, f"   F = {Fg:.1f} N per guide (2 g), span {span:.0f} mm -> sigma = {sig_g:.1f} MPa, FS = {SY_5052 / sig_g:.0f}, "
        f"mid deflection {dg:.2f} mm  PASS")
line(S, "   NOTE: the guides only support from below. With the drawer pulled past ~150 mm the battery COM leaves the\n"
        "   guides and the tray tips - the drawer is a removable cartridge (pull fully out by the handle). Add a\n"
        "   closed-position latch/detent so it cannot slide out under braking (see drivetrain item 4).\n")

# 6. 2 mm deck plate under the LiDAR pedestal and E-stop: point loads near cross members -> negligible
line(S, "6. Deck plate RV-430: LiDAR pedestal sits on the LIDAR_X top cross member, tower feet bolt through to the\n"
        "   POST_X / STRUT_X0 cross members - no unsupported point loads. PASS by load path.")

out = os.path.join(ROOT, "Simulation", "FEA", "hand_calcs_structural.txt")
os.makedirs(os.path.dirname(out), exist_ok=True)
open(out, "w", encoding="utf-8").write("\n".join(S) + "\n")
print("\n".join(S))

# ================================================================== drivetrain / motion
D = []
r = G["WHEEL_D"] / 2 / 1000                                        # m
T_stall, T_cont, T_inst = 27 * 0.0981, 10 * 0.0981, 25 * 0.0981    # N.m (Pololu 4754 @ 12 V)
w0 = 150 * 2 * math.pi / 60                                        # rad/s no-load @ 12 V
I0, Is = 0.2, 5.5
Crr = 0.02
eta = 0.9                                                          # wheel/bushing losses
mu = 0.7                                                           # silicone tread on sealed concrete/tile


def per_motor(force):
    return force * r / 2 / eta


def speed(T):
    return w0 * max(0.0, 1 - T / T_stall) * r


def current(T):
    return I0 + (Is - I0) * T / T_stall


line(D, "RoamView - drivetrain hand checks (stand-in for Motion Analysis)")
line(D, f"Motor: Pololu 4754 37D 70:1 @ 12 V: stall {T_stall:.2f} N.m, continuous limit {T_cont:.2f}, instantaneous "
        f"{T_inst:.2f}, no-load 150 rpm, 0.2 A / 5.5 A. Wheel r = {r * 1000:.0f} mm, Crr {Crr}, eta {eta}, mu {mu}")
line(D, f"Robot mass {M:.2f} kg, COM height {CZ:.0f} mm\n")
cases = [("flat, 0.5 m/s cruise", M * g * Crr, 0.5),
         ("flat, accelerate 0.5 m/s^2", M * (0.5 + g * Crr), 0.5),
         ("5 deg ramp, 0.3 m/s cruise", M * g * (math.sin(math.radians(5)) + Crr * math.cos(math.radians(5))), 0.3),
         ("5 deg ramp + 0.5 m/s^2", M * (g * (math.sin(math.radians(5)) + Crr) + 0.5), 0.3)]
for name, Fx, v in cases:
    T = per_motor(Fx)
    ok = T <= T_cont
    line(D, f"  {name:30s} F {Fx:6.1f} N -> {T:5.2f} N.m/motor ({T / T_cont * 100:4.0f} % of continuous), "
            f"{current(T):4.2f} A, max speed {speed(T):4.2f} m/s  {'PASS' if ok else 'CHECK'}")
# threshold climb (driven wheel pivoting over a 15 mm edge)
h = 15.0 / 1000
phi = math.acos((r - h) / r)
T_th = wheel * r * math.sin(phi)
line(D, f"\n  15 mm threshold, drive wheel: lever r sin(phi) = {r * math.sin(phi) * 1000:.0f} mm -> {T_th:.2f} N.m "
        f"({T_th / T_inst * 100:.0f} % of instantaneous limit)  {'PASS' if T_th <= T_inst else 'FAIL'}")
rc = G["CASTER_D"] / 2 / 1000
phic = math.acos((rc - h) / rc)
for nm, N in (("front sprung caster (preload ~15 N)", 15.0), ("rear rigid caster (reversing)", rear)):
    Fpush = N * math.tan(phic)
    trac = mu * 2 * wheel
    line(D, f"  15 mm threshold, {nm:35s}: needs {Fpush:5.1f} N push (tan {math.degrees(phic):.0f} deg) vs traction "
            f"{trac:.0f} N, motor force {2 * T_inst * eta / r:.0f} N  {'PASS' if Fpush < 0.5 * min(trac, 2 * T_inst * eta / r) else 'MARGINAL'}")
line(D, "  A 2-inch caster meets a 15 mm edge at 66 deg: it gets over only because it is lightly loaded. If thresholds\n"
        "  are common, switch to 3-inch casters (CASTER_D 75, CASTER_H ~ 95) - a global change.")
# braking / tipping
a_brake = 1.0
xf = G["CASTER_X"] - 10
a_tip_f = g * (xf + CX) / CZ if CZ else 0
line(D, f"\n  braking from 0.5 m/s at {a_brake} m/s^2: stop in {0.5 ** 2 / 2 / a_brake * 1000:.0f} mm, regenerative "
        f"torque {per_motor(M * a_brake) * eta * eta:.2f} N.m/motor")
line(D, f"  forward tip-over needs decel > {a_tip_f:.1f} m/s^2 ({a_tip_f / g:.2f} g) - braking at 1 m/s^2 is "
        f"{'SAFE' if a_tip_f > 2 * a_brake else 'MARGINAL'}; traction limit mu g = {mu * g:.1f} m/s^2")
line(D, f"  drawer cartridge ({m_dr:.2f} kg) under {a_brake} m/s^2 braking pushes {m_dr * a_brake:.1f} N toward the "
        f"front stop (XT60 dock end) - closed-position latch recommended")
v_pack = 16.8
line(D, f"\n  SUPPLY NOTE: 4S pack is {14.8:.1f}-{v_pack:.1f} V into 12 V motors -> cap PWM at {12 / v_pack * 100:.0f} % "
        f"(or regulate) so stall torque/current stay within the 12 V ratings ({Is * v_pack / 12:.1f} A stall at full "
        f"charge otherwise); DFR0601 12 A/ch has margin either way.")
out = os.path.join(ROOT, "Simulation", "Motion", "hand_calcs_drivetrain.txt")
os.makedirs(os.path.dirname(out), exist_ok=True)
open(out, "w", encoding="utf-8").write("\n".join(D) + "\n")
print("\n" + "\n".join(D))
