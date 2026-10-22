#!/usr/bin/env python3
"""Builds the 1.2 m T-28 Trojan RC model for INAV HITL in X-Plane 11, tuned to the real aircraft's blackbox logs.

Starts from base_planemaker_blank.acf, the file Plane Maker 11.55 writes for File > New with nothing changed, and
writes every part of the aircraft from the numbers below: fuselage sections from the Blender model
(blender/t28_model.py), wings and tail of the T-28B at 1:10, tricycle retracts, electric motor and fixed-pitch prop,
masses, controls and flaps. Nothing comes from another aircraft. The look is the Blender OBJ (blender/out), copied to
objects/; Plane Maker's own parts stay invisible.

    python build_t28.py            # build and check
    python build_t28.py --check    # check the ACF already built
"""
import argparse
import ast
import math
import os
import re
import shutil
import struct
import sys
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
# the X-Plane 11 folder: XPLANE_DIR in the environment, or --xp
XP_DEFAULT = os.environ.get("XPLANE_DIR", r"C:\X-Plane 11")
BASE_ACF = os.path.join(HERE, "base_planemaker_blank.acf")
BLENDER_DIR = os.path.join(HERE, "blender")
OUT_REL = r"Aircraft\Extra Aircraft\T28_RC_1200"
OUT_NAME = "T28_RC"
OBJ_NAME = "t28_rc.obj"

FT = 0.3048
LB = 0.45359237
KT = 0.514444
HP = 745.7
G_FT = 32.174

# --- Geometry, metres. ACF reference point: spinner tip on the thrust line; x right, y up, z aft.
# T-28B at 1:10 (12.22 m span), length of the 1.2 m foam model.
SPAN_M = 1.224
LENGTH_M = 0.986
WING_CROOT, WING_CTIP = 0.27, 0.12   # straight leading edge
WING_LE_Z = 0.33
WING_ROOT_Y = -0.05                  # quarter chord at the root, low wing
WING_DIHED = 7.0
WING_INCID_DEG = 1.0                 # no washout: the foam wing is flat, and the real one drops a wing at the stall
WING_AIRFOIL = "NACA 2412 (popular).afl"
# Semispan stations of the three wing sections (flaps, ailerons, tip) and their elements; X-Plane computes the forces
# per element, and a flap or aileron sits on whole elements
WING_STATIONS = (0.0, 0.30, 0.56, SPAN_M / 2.0)
WING_ELS = (4, 5, 2)
WING_PARTS = ((8, 9), (10, 11), (12, 13))        # left, right
FLAP_ELEMENTS = {0: range(1, 4)}                 # 0.075-0.30 m: from the fuselage side to the ailerons
AILN_ELEMENTS = {1: range(0, 5)}                 # 0.30-0.56 m
HT_SPAN, HT_CROOT, HT_CTIP = 0.52, 0.16, 0.09
HT_QC_Z, HT_Y = 0.85, 0.0            # straight quarter-chord line
HT_PARTS = (16, 17)
HT_ELS = 8
FIN_HEIGHT, FIN_CROOT, FIN_CTIP = 0.16, 0.18, 0.09
FIN_TE_Z, FIN_ROOT_Y = 0.986, 0.035  # vertical trailing edge (rudder hinge)
FIN_PART = 18
FIN_ELS = 8
TAIL_AIRFOIL = "NACA 0009 (symmetrical).afl"
# spinner ahead of the cowl (blender/t28_model.py: spinner_length 0.07, spinner_radius 0.034)
SPINNER = ((0.0, 0.0), (0.012, 0.020), (0.028, 0.030), (0.040, 0.034))
FUSE_PART = 56
FUSE_RADII = 9                       # points per half section, Plane Maker's maximum
# Measured on the aircraft: 75-80 mm behind the wing saddle, 0.38 of the mean chord; manual 83 +/- 19 mm
CG_MAC, CG_FWD_MAC, CG_AFT_MAC = 0.42, 0.31, 0.55
PILOT_EYE = (0.0, 0.07, 0.38)        # over the canopy, where an FPV camera would sit
# Moments of inertia per kg (m2), for the record: X-Plane recomputes them from the drawn shape at every load whatever
# the ACF says (checked), 0.06 / 0.08 / 0.02. Roll and pitch agree with a count of the real masses (motor and
# battery in the nose, retracts in the wings, foam shell) and with the open-loop step response against the logs
# (step_shape.py); yaw is a quarter of what a near-planar airframe must have (Jzz close to Jxx + Jyy), so the HITL
# plugin writes 0.09 to sim/aircraft/weight/acf_Jzz_unitmass whenever the T-28 is loaded
J_UNITMASS = (0.06, 0.08, 0.09)

# --- Tuning to the real logs (docs/MODEL.md): effective travels and hinge chord ratios
# Travels measured on the aircraft (high rate): elevator 15/15 mm at the tip, ailerons 16/16 mm at the root
ELEV_UP_DEG, ELEV_DN_DEG = 11.0, 11.0
AILN_UP_DEG, AILN_DN_DEG = 17.0, 17.0
RUDD_DEG = 25.0
ELEV_CRAT = 0.37
AILN_CRAT = 0.25
RUDD_CRAT = 0.32                     # hinge at 0.68 of the fin chord (blender/control_surfaces.py)
FLAP_CRAT = 0.25                     # hinge at 0.75 of the wing chord
HT_INCID_DEG = 0.9                   # stabiliser incidence: the trim the logs show
# neutral to full travel of a quick digital servo; the logs' command-to-response delay (50 ms roll, 54 ms pitch) comes
# from the plugin's servo dead time (servo_response.py: with it these travel times change nothing measurable)
AILN_DEF_TIME_S, ELEV_DEF_TIME_S, RUDD_DEF_TIME_S = 0.04, 0.03, 0.03
# Flaps: half and full detents, and what the flap adds to the airfoil; the moment sets the elevator the flaps need
# (530 units per point of cm) and the lift the attitude change, almost apart: solved on a grid of live datarefs
# (flap_grid.py) for the logs' half -50 units / -5.9 deg and full -75 / -7.2
FLAP_DN_DEG = (20.0, 27.0)
FLAP_CL, FLAP_CD, FLAP_CM = 1.50, 0.03, -0.78
FLAP_TIME_S = 1.5

# --- Mass, motor, prop
# flying weight with battery: the logs give 2.0-2.4 kg with the manual's motor and prop (mass_from_logs.py and
# the ESC fit with the real motor) and the logged stall (10.5-11 m/s) the same; stock 1.7 kg with a 4S 2200
MASS_KG = 2.2
FUEL_LB = 0.02                       # an electric motor burns nothing; a valid tank stays
# The HITL plugin drives the prop with the T-28's own motor and ESC (EFLM4115 850 Kv, Avian 70 A; plugin menu Motor, on
# by default, R and I0 fitted to the logs). These are for X-Plane's electric motor, whose torque is throttle x power /
# redline: 0.74 N m at full throttle, the real motor's torque at flight speeds; redline = 850 Kv x 16.6 V
POWER_HP = 1.45                      # at the shaft
REDLINE_RPM = 14000.0
MINGREEN_RPM = 6000.0
PROP_DIAM_IN = 12.0                  # EFLP120703B, 12x7 3-blade (manual)
PROP_PITCH_IN = 7.0
PROP_BLADES = 3.0
PROP_CROOT_M, PROP_CTIP_M = 0.030, 0.0165   # blade chord fitted to the logged current and climb
PROP_AIRFOIL = "Clark-Y (good propeller).afl"
PROP_Z = 0.0524                      # prop plane behind the spinner tip
# right thrust as RC models have it: without it the prop wash swings the model 50 deg/s left at the start of the
# take-off run, while the real one in the logs runs straight with no rudder
THRUST_RIGHT_DEG, THRUST_DOWN_DEG = 1.5, 0.0
THROTTLE_TIME_S = 0.3
PROP_INERTIA_RAT = 0.5               # 12x7 prop and motor bell, ~3e-4 kg m2; X-Plane's own figure for the fitted blade chord
                                     # spun up in 0.9 s where the logged current says 0.3 (take-off run 1.65 s against 1.25)
BATTERY_WH = 500000.0                # never runs flat in a session

# --- Drag: the .afl polars already cover Re 0.1 M; the airframe's own drag (radial cowl, canopy, gear bays) goes on
# the fuselage, as a drag coefficient on its frontal area of 0.0165 m2, fitted with the real motor and prop to the
# logs' climb, glide and cruise (airframe CD0 about 0.03 with the prop's own drag apart)
FUSE_CD = 0.23
GEAR_FLATPLATE_M2 = 0.002            # extra with the gear down

# --- Gear: tricycle retracts, contacts chosen for GROUND_PITCH_DEG nose up and PROP_CLEARANCE_M under the prop
NOSE_WHEEL_Z, MAIN_WHEEL_X = 0.13, 0.15
# mains behind the effective CG (CG_MAC): with the 12 inch prop the tail touches at 21 deg nose up and the CG is 189 mm
# up; mains closer than ~73 mm let the model rest on its tail once a bounce lifts the nose that far, as it did after a
# nose-wheel touchdown and after the placing drop; 70 mm = 20.3 deg, 22 deg from the attitude at rest
MAIN_BEHIND_CG_M = 0.070
# wire legs: 8 mm let the prop touch under full power, 5 mm let the nose porpoise on the take-off run
TIRE_R, TIRE_W, GEAR_STAT_DEF = 0.025, 0.02, 0.003
NOSE_ATTACH_Y, MAIN_ATTACH_Y = -0.06, -0.04
# at rest the real one sits 1.4-2.2 deg nose down (board attitude in the logs; the board reads the model's own body
# angle of attack at 24 m/s, so it is aligned with the fuselage): 3 deg nose up made INAV push the nose wheel down
# all along the take-off run in ANGLE and the model porpoised
GROUND_PITCH_DEG, PROP_CLEARANCE_M = -1.8, 0.025
GEAR_ZETA = (0.40, 0.27, 0.27)       # damping ratios of the earlier model, whose take-offs and landings went well
GEAR_CYCLE_S = 2.5
NOSE_STEER_LO_DEG, NOSE_STEER_HI_DEG = 25.0, 10.0

G_POS, G_NEG = 12.0, 8.0
# m/s, for the indicators only (the stall itself is the airfoil's): logs show 11 clean and about 10 with flaps
VS_MS, VSO_MS = 11.0, 10.0
VFE1_MS, VFEM_MS, VNO_MS, VNE_MS, VLE_MS = 25.0, 20.0, 30.0, 55.0, 30.0

NAME = "T-28 RC 1.2m (INAV HITL)"
AUTHOR = "INAV HITL"
DESCRIP = ("1.2 m foam T-28 Trojan for INAV HITL, tuned to blackbox logs of the real aircraft: 1.224 m span, "
           "2.2 kg, electric, retracts, flaps.")


def f32(v):
    return struct.unpack("f", struct.pack("f", v))[0]


def fmt_float(v):
    v = f32(float(v))
    if v == int(v) and abs(v) < 1e9:
        return "%d.0" % int(v)
    return "%.9f" % v


def is_int(s):
    return re.fullmatch(r"-?\d+", s) is not None


def is_float(s):
    return re.fullmatch(r"-?\d+\.\d+(e-?\d+)?|-?nan\(ind\)", s) is not None


class Acf:
    """Text ACF: lines before, 'P name value' properties in order, lines after. CRLF read and written as LF."""

    def __init__(self, path):
        raw = open(path, "rb").read().replace(b"\r\n", b"\n")
        self.text = raw.decode("latin-1")
        lines = self.text.split("\n")
        i0 = lines.index("PROPERTIES_BEGIN")
        i1 = lines.index("PROPERTIES_END")
        self.head = lines[:i0 + 1]
        self.tail = lines[i1:]
        self.vals = {}
        for ln in lines[i0 + 1:i1]:
            if not ln.startswith("P "):
                raise SystemExit("%s: unexpected line among the properties: %r" % (path, ln))
            name, _, val = ln[2:].partition(" ")
            if name in self.vals:
                raise SystemExit("%s: property twice: %s" % (path, name))
            self.vals[name] = val

    @property
    def names(self):
        return sorted(self.vals)

    def get(self, name):
        return self.vals[name]

    def f(self, name):
        return float(self.vals[name])

    def part_eq(self, p):
        return self.vals.get("_part/%d/_part_specs_eq" % p) == "1"

    def set(self, name, value):
        """An existing property keeps its type (integer or float)."""
        if name not in self.vals:
            raise SystemExit("no such property in the base: %s" % name)
        old = self.vals[name]
        if isinstance(value, str):
            self.vals[name] = value
        elif is_int(old):
            if int(value) != value:
                raise SystemExit("%s is an integer, value %r" % (name, value))
            self.vals[name] = str(int(value))
        elif is_float(old):
            self.vals[name] = fmt_float(value)
        else:
            raise SystemExit("%s is not numeric (%r)" % (name, old))

    def add(self, name, value):
        """A property Plane Maker writes only once the part exists: the Python type says integer, float or text."""
        if isinstance(value, bool) or isinstance(value, int):
            self.vals[name] = str(int(value))
        elif isinstance(value, float):
            self.vals[name] = fmt_float(value)
        else:
            self.vals[name] = str(value)

    def put(self, name, value):
        if name in self.vals:
            self.set(name, value)
        else:
            self.add(name, value)

    def lines_out(self):
        return self.head + ["P %s %s" % (n, self.vals[n]) for n in self.names] + self.tail


def fuselage_stations():
    """(s, width, height, centre height, superellipse exponent) of blender/t28_model.py, read without Blender."""
    src = open(os.path.join(BLENDER_DIR, "t28_model.py"), encoding="utf-8").read()
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "STATIONS":
            return ast.literal_eval(node.value)
    raise SystemExit("STATIONS not found in blender/t28_model.py")


def superellipse_half(w, h, yc, n, k):
    """k points from the top down the right side to the bottom (Plane Maker's order), metres."""
    pts = []
    for j in range(k):
        phi = math.pi * j / (k - 1)
        s, c = math.sin(phi), math.cos(phi)
        x = (w / 2.0) * abs(s) ** (2.0 / n)
        y = yc + (h / 2.0) * math.copysign(abs(c) ** (2.0 / n), c)
        pts.append((0.0 if abs(x) < 1e-9 else x, y))
    return pts


def mac_trapezoid(cr, ct):
    lam = ct / cr
    return 2.0 / 3.0 * cr * (1 + lam + lam * lam) / (1 + lam)


# --- Plane Maker part template: the keys of a part are those the blank writes for its fuselage
def part_template(base):
    return {n.split("/", 2)[2]: v for n, v in base.vals.items() if n.startswith("_part/56/")}


def wing_keys():
    """The keys of one wing part (Plane Maker 11), with their neutral values."""
    k = {}
    for a in ("ailn1", "ailn2", "elev1", "elev2", "flap1", "flap2", "inc_ailn1", "inc_ailn2", "inc_elev1", "inc_elev2",
              "inc_rudd1", "inc_rudd2", "inc_trim", "inc_vect", "rudd1", "rudd2", "sbrk1", "sbrk2", "slat1", "slat2",
              "spoi1", "spoi2", "yawbr"):
        k["_%s/count" % a] = 10
        for i in range(10):
            k["_%s/%d" % (a, i)] = 0
    k["_incidence/count"] = 10
    for i in range(10):
        k["_incidence/%d" % i] = 0.0
    for a, v in (("c_off_for_RIB", 0.0), ("c_rat_for_RIB", 1.0), ("chord_for_RIB", 0.0), ("crib_x_arm", 0.0),
                 ("crib_y_arm", 0.0), ("crib_z_arm", 0.0)):
        k["_%s/count" % a] = 11
        for i in range(11):
            k["_%s/%d" % (a, i)] = v
    for a in ("Croot", "Ctip", "dihed_design", "dihed_full_up", "dihed_root_inc", "incid_chrd_inc", "incid_full_up",
              "retract_max", "semilen_JND", "semilen_SEG", "sweep_design", "sweep_full_aft", "sweep_root_inc"):
        k["_" + a] = 0.0
    for a in ("var_dihed", "var_incid", "var_retract", "var_sweep"):
        k["_" + a] = 0
    k["_els"] = 4
    k["_foil_interp_power"] = 1.0
    k["_is_right_mult"] = 1.0
    for s in ("R0", "R1", "T0", "T1"):
        k["_afl_file_" + s] = ""
    return k


def add_part(acf, tmpl, p, invisible=True, eq=1, s_dim=8, r_dim=14):
    for suffix, v in tmpl.items():
        name = "_part/%d/%s" % (p, suffix)
        if "_geo_xyz/" in suffix and not suffix.endswith("_count"):
            acf.vals[name] = "0.0"
        elif "_locked/" in suffix and not suffix.endswith("_count"):
            acf.vals[name] = "0"
        else:
            acf.vals[name] = v
    acf.set("_part/%d/_part_specs_eq" % p, eq)
    acf.set("_part/%d/_part_specs_invis" % p, 1 if invisible else 0)
    acf.set("_part/%d/_s_dim" % p, s_dim)
    acf.set("_part/%d/_r_dim" % p, r_dim)
    for a in ("x", "y", "z", "phi", "psi", "the"):
        acf.set("_part/%d/_part_%s" % (p, a), 0.0)


def add_wing(acf, p, els, airfoil, right):
    for suffix, v in wing_keys().items():
        acf.add("_wing/%d/%s" % (p, suffix), v)
    acf.set("_wing/%d/_els" % p, els)
    acf.set("_wing/%d/_is_right_mult" % p, 1.0 if right else -1.0)
    for s in ("R0", "R1", "T0", "T1"):
        acf.set("_wing/%d/_afl_file_%s" % (p, s), airfoil)


def set_surface(acf, p, root_m, semilen_m, croot_m, ctip_m, sweep_deg, dihed_deg, side):
    """One Plane Maker wing section: root at the quarter chord, semilength and sweep on the 25 % line (Plane Maker
    manual); the derived ribs too."""
    acf.set("_part/%d/_part_x" % p, root_m[0] / FT)
    acf.set("_part/%d/_part_y" % p, root_m[1] / FT)
    acf.set("_part/%d/_part_z" % p, root_m[2] / FT)
    acf.set("_wing/%d/_semilen_SEG" % p, semilen_m / FT)
    acf.set("_wing/%d/_Croot" % p, croot_m / FT)
    acf.set("_wing/%d/_Ctip" % p, ctip_m / FT)
    acf.set("_wing/%d/_sweep_design" % p, sweep_deg)
    acf.set("_wing/%d/_dihed_design" % p, dihed_deg)
    sw, di = math.radians(sweep_deg), math.radians(dihed_deg)
    d = (side * math.cos(sw) * math.cos(di), math.cos(sw) * math.sin(di), math.sin(sw))
    els = int(acf.get("_wing/%d/_els" % p))
    for i in range(els + 1):
        f = i / float(els)
        for k, ax in enumerate("xyz"):
            v = root_m[k] + f * semilen_m * d[k]
            acf.set("_wing/%d/_crib_%s_arm/%d" % (p, ax, i), 0.0 if abs(v) < 1e-9 else v / FT)
        acf.set("_wing/%d/_chord_for_RIB/%d" % (p, i), (croot_m + (ctip_m - croot_m) * f) / FT)


def wing_tip_x(acf, p):
    sweep = math.radians(acf.f("_wing/%d/_sweep_design" % p))
    dihed = math.radians(acf.f("_wing/%d/_dihed_design" % p))
    return abs(acf.f("_part/%d/_part_x" % p)) + acf.f("_wing/%d/_semilen_SEG" % p) * math.cos(sweep) * math.cos(dihed)


def wing_parts_present(acf):
    return [p for p in range(8, 16) if acf.part_eq(p)]


def span_ft(acf):
    return 2.0 * max(wing_tip_x(acf, p) for p in wing_parts_present(acf))


def wing_area_ft2(acf):
    s = 0.0
    for p in wing_parts_present(acf):
        sweep = math.radians(acf.f("_wing/%d/_sweep_design" % p))
        dihed = math.radians(acf.f("_wing/%d/_dihed_design" % p))
        b = acf.f("_wing/%d/_semilen_SEG" % p) * math.cos(sweep) * math.cos(dihed)
        s += b * 0.5 * (acf.f("_wing/%d/_Croot" % p) + acf.f("_wing/%d/_Ctip" % p))
    return s


def gear_contacts(acf):
    """Contact points at rest: axle minus tyre radius, plus the static deflection."""
    pts = []
    for g in range(10):
        if acf.get("_gear/%d/_gear_type" % g) == "0":
            continue
        lat = math.radians(acf.f("_gear/%d/_latE" % g))
        lon = math.radians(acf.f("_gear/%d/_lonE" % g))
        L = acf.f("_gear/%d/_leg_len" % g)
        x = acf.f("_gear/%d/_gear_x" % g) + L * math.cos(lon) * math.sin(lat)
        y = acf.f("_gear/%d/_gear_y" % g) - L * math.cos(lon) * math.cos(lat)
        z = acf.f("_gear/%d/_gear_z" % g) - L * math.sin(lon)
        y += -acf.f("_gear/%d/_tire_radius" % g) + acf.f("_gear/%d/_stat_def" % g)
        pts.append((g, x, y, z))
    return pts


def ground_attitude(acf):
    pts = gear_contacts(acf)
    nose = min(pts, key=lambda p: p[3])
    mains = [p for p in pts if p is not nose]
    ym = sum(p[2] for p in mains) / len(mains)
    zm = sum(p[3] for p in mains) / len(mains)
    th = math.atan2(ym - nose[2], zm - nose[3])

    def height(y, z):
        return (y - ym) * math.cos(th) - (z - zm) * math.sin(th)

    return th, height, pts


def prop_clearance_ft(acf, radius_ft):
    th, height, _ = ground_attitude(acf)
    return height(acf.f("_part/0/_part_y"), acf.f("_part/0/_part_z")) - radius_ft * math.cos(th)


def write_png_solid(path, rgb, size=64):
    row = b"\x00" + bytes(rgb) * size
    raw = row * size

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


def build(xp):
    out_dir = os.path.join(xp, OUT_REL)
    base = Acf(BASE_ACF)
    acf = Acf(BASE_ACF)
    if acf.vals.get("acf/_file_writer_version") != "115501" or any(acf.part_eq(p) for p in range(0, 56)):
        raise SystemExit("the base is not Plane Maker 11.55's empty aircraft")
    rep = []
    tmpl = part_template(base)

    # --- 1. fuselage: one body from the spinner tip to the rudder hinge, sections of the Blender model
    st = fuselage_stations()
    if abs(st[-1][0] - LENGTH_M) > 1e-6:
        raise SystemExit("the Blender fuselage ends at %.3f m, not %.3f" % (st[-1][0], LENGTH_M))
    sections = [(s, 2 * r, 2 * r, 0.0, 2.0) for s, r in SPINNER] + list(st)
    if len(sections) > 20:
        raise SystemExit("%d fuselage sections, Plane Maker holds 20" % len(sections))
    p = FUSE_PART
    add_part(acf, tmpl, p, s_dim=len(sections), r_dim=2 * FUSE_RADII)
    for i in range(20):
        s, w, h, yc, n = sections[min(i, len(sections) - 1)]
        half = superellipse_half(w, h, yc, n, FUSE_RADII)
        ring = half + [(-x, y) for x, y in reversed(half)]
        for j, (x, y) in enumerate(ring):
            for k, v in enumerate((x, y, s)):
                acf.set("_part/%d/_geo_xyz/%d,%d,%d" % (p, i, j, k), v / FT)
    acf.set("_part/%d/_part_rad" % p, max(max(w, h) for _, w, h, _, _ in sections) / 2.0 / FT)
    acf.set("_part/%d/_part_cd" % p, FUSE_CD)
    rep.append("fuselage: %d sections from %.3f to %.3f m, widest %.3f m, tallest %.3f m"
               % (len(sections), sections[0][0], sections[-1][0], max(x[1] for x in sections),
                  max(x[2] for x in sections)))

    # --- 2. wing, tailplane and fin
    b2 = SPAN_M / 2.0
    dy_w = b2 * math.tan(math.radians(WING_DIHED))
    dz_w = -(WING_CROOT - WING_CTIP) / 4.0          # straight leading edge: the 25 % line sweeps forward
    sw_w = math.degrees(math.atan2(dz_w, math.hypot(b2, dy_w)))
    L_w = math.sqrt(b2 * b2 + dy_w * dy_w + dz_w * dz_w)

    def chord(f):
        return WING_CROOT + (WING_CTIP - WING_CROOT) * f

    fr = [x / b2 for x in WING_STATIONS]
    for s, (pl, pr) in enumerate(WING_PARTS):
        fa, fb = fr[s], fr[s + 1]
        for pp, side in ((pl, -1), (pr, 1)):
            add_part(acf, tmpl, pp)
            add_wing(acf, pp, WING_ELS[s], WING_AIRFOIL, side > 0)
            root = (side * fa * b2, WING_ROOT_Y + fa * dy_w, WING_LE_Z + chord(fa) / 4.0)
            set_surface(acf, pp, root, L_w * (fb - fa), chord(fa), chord(fb), sw_w, WING_DIHED, side)
            for k in range(10):
                acf.set("_wing/%d/_incidence/%d" % (pp, k), WING_INCID_DEG)
                acf.set("_wing/%d/_flap1/%d" % (pp, k), 1 if k in FLAP_ELEMENTS.get(s, ()) else 0)
                acf.set("_wing/%d/_ailn1/%d" % (pp, k), 1 if k in AILN_ELEMENTS.get(s, ()) else 0)
    for pp, side in ((HT_PARTS[0], -1), (HT_PARTS[1], 1)):
        add_part(acf, tmpl, pp)
        add_wing(acf, pp, HT_ELS, TAIL_AIRFOIL, side > 0)
        set_surface(acf, pp, (0.0, HT_Y, HT_QC_Z), HT_SPAN / 2.0, HT_CROOT, HT_CTIP, 0.0, 0.0, side)
        for k in range(10):
            acf.set("_wing/%d/_incidence/%d" % (pp, k), HT_INCID_DEG)
            acf.set("_wing/%d/_elev1/%d" % (pp, k), 1 if k < HT_ELS else 0)
    add_part(acf, tmpl, FIN_PART)
    add_wing(acf, FIN_PART, FIN_ELS, TAIL_AIRFOIL, True)
    qc_root = FIN_TE_Z - 0.75 * FIN_CROOT
    qc_tip = FIN_TE_Z - 0.75 * FIN_CTIP
    sw_f = math.degrees(math.atan2(qc_tip - qc_root, FIN_HEIGHT))
    set_surface(acf, FIN_PART, (0.0, FIN_ROOT_Y, qc_root), FIN_HEIGHT / math.cos(math.radians(sw_f)),
                FIN_CROOT, FIN_CTIP, sw_f, 90.0, 1)
    for k in range(10):
        acf.set("_wing/%d/_rudd1/%d" % (FIN_PART, k), 1 if k < FIN_ELS else 0)
    rep.append("wing: quarter-chord sweep %.2f deg, sections at %s m of the semispan"
               % (sw_w, " ".join("%.3f" % x for x in WING_STATIONS)))

    # --- 3. mass and centre of gravity
    mac = mac_trapezoid(WING_CROOT, WING_CTIP)
    z_cg = WING_LE_Z + CG_MAC * mac
    m_max = MASS_KG / LB
    acf.set("acf/_m_max", m_max)
    acf.set("acf/_m_empty", m_max - FUEL_LB)
    acf.set("acf/_m_fuel_max_tot", FUEL_LB)
    acf.set("acf/_cgY", 0.0)
    acf.set("acf/_cgZ", z_cg / FT)
    acf.set("acf/_cgZ_fwd", (WING_LE_Z + CG_FWD_MAC * mac) / FT)
    acf.set("acf/_cgZ_aft", (WING_LE_Z + CG_AFT_MAC * mac) / FT)
    acf.set("acf/_tank_rat/0", 1.0)
    acf.set("acf/_tank_xyz/0,2", z_cg / FT)
    acf.set("acf/_tank_xyz_full/0,2", z_cg / FT)
    for i, v in enumerate(PILOT_EYE):
        acf.set("acf/_pe_xyz/%d" % i, v / FT)
    for ax, v in zip("xyz", J_UNITMASS):
        acf.set("acf/_J%s%s_unitmass" % (ax, ax), v)

    # --- 4. electric motor and fixed-pitch prop
    acf.set("acf/_num_engn", 1)
    acf.set("acf/_num_prop", 1)
    acf.set("acf/_num_xmsn", 1)
    acf.add("_engn/0/_type", "ELE")
    acf.add("_engn/0/_engn_clutch_EQ", 0)
    acf.add("_engn/0/_engn_freewheels", 0)
    acf.add("_engn/0/_engn_mass", 0.0)
    acf.add("_engn/0/_engn_mi_rpm", 0.0)
    acf.add("_engn/0/_exhaust_os_xyz/count", 3)
    for i in range(3):
        acf.add("_engn/0/_exhaust_os_xyz/%d" % i, 0.0)
    acf.set("acf/_power_max_limit", POWER_HP)
    acf.set("acf/_red_hi_pwr", POWER_HP)
    acf.set("acf/_RSC_redline_ENGN", REDLINE_RPM)
    acf.set("acf/_RSC_maxgreen_ENGN", REDLINE_RPM)
    acf.set("acf/_RSC_mingreen_ENGN", MINGREEN_RPM)
    acf.set("acf/_snd_rpm_eng", REDLINE_RPM)
    acf.set("acf/_snd_rpm_prp", REDLINE_RPM)
    acf.set("acf/_throt_time_prop", THROTTLE_TIME_S)
    acf.set("acf/_rotor_mi_rat", PROP_INERTIA_RAT)
    acf.set("acf/_fadec_EQ_mixture", 1)              # no mixture lever
    acf.set("acf/_starter_max_rpm_rat", 1.0)         # the starter is the motor itself
    acf.set("acf/_battery_watt_hr_max", BATTERY_WH)
    for n in ("acf/_SFC_full_lo_PRP", "acf/_SFC_full_hi_PRP", "acf/_SFC_half_lo_PRP", "acf/_SFC_half_hi_PRP"):
        acf.set(n, 0.0)
    R_m = PROP_DIAM_IN * 0.0254 / 2.0
    des_kts = PROP_PITCH_IN * 0.0254 * REDLINE_RPM / 60.0 / KT
    for name, v in (("_des_kts_acf", des_kts), ("_des_rpm_prp", REDLINE_RPM), ("_max_pitch", 0.0),
                    ("_min_pitch", 0.0), ("_num_blades", PROP_BLADES), ("_prop_clutch_EQ", 0), ("_prop_dir", 1.0),
                    ("_prop_ducted", 0), ("_prop_gear_rat", 1.0), ("_prop_type", 0), ("_vectarmY_engn", 0.0),
                    ("_vectarmZ_engn", 0.0)):
        acf.add("_prop/0/" + name, v)
    add_part(acf, tmpl, 0, eq=0, s_dim=20, r_dim=14)
    acf.set("_part/0/_part_z", PROP_Z / FT)
    acf.set("_part/0/_part_psi", THRUST_RIGHT_DEG)
    acf.set("_part/0/_part_the", -THRUST_DOWN_DEG)
    add_wing(acf, 0, 10, PROP_AIRFOIL, True)
    acf.set("_wing/0/_semilen_SEG", R_m / FT)
    acf.set("_wing/0/_Croot", PROP_CROOT_M / FT)
    acf.set("_wing/0/_Ctip", PROP_CTIP_M / FT)
    for i in range(11):
        acf.set("_wing/0/_chord_for_RIB/%d" % i, (PROP_CROOT_M + (PROP_CTIP_M - PROP_CROOT_M) * i / 10.0) / FT)
        acf.set("_wing/0/_crib_x_arm/%d" % i, R_m * i / 10.0 / FT)
    # fixed-pitch twist as Plane Maker computes it: the flow angle at the design point, so V/n is the pitch
    v_ms = des_kts * KT
    omega = REDLINE_RPM * 2.0 * math.pi / 60.0
    for i in range(10):
        r = (i + 0.5) * R_m / 10.0
        acf.set("_wing/0/_incidence/%d" % i, math.degrees(math.atan(v_ms / (omega * r))))

    # --- 5. gear
    th = math.radians(GROUND_PITCH_DEG)
    main_z = z_cg + MAIN_BEHIND_CG_M
    y_main = -(PROP_CLEARANCE_M + R_m * math.cos(th) + (PROP_Z - main_z) * math.sin(th)) / math.cos(th)
    y_nose = y_main - (main_z - NOSE_WHEEL_Z) * math.tan(th)
    nose_frac = (main_z - z_cg) / (main_z - NOSE_WHEEL_Z)
    legs = ((0, 0.0, NOSE_ATTACH_Y, NOSE_WHEEL_Z, y_nose, nose_frac),
            (1, -MAIN_WHEEL_X, MAIN_ATTACH_Y, main_z, y_main, (1 - nose_frac) / 2),
            (2, MAIN_WHEEL_X, MAIN_ATTACH_Y, main_z, y_main, (1 - nose_frac) / 2))
    for g, gx, gy, gz, y_contact, frac in legs:
        y_axle = y_contact + TIRE_R - GEAR_STAT_DEF
        acf.set("_gear/%d/_gear_type" % g, 2)
        acf.set("_gear/%d/_gear_x" % g, gx / FT)
        acf.set("_gear/%d/_gear_y" % g, gy / FT)
        acf.set("_gear/%d/_gear_z" % g, gz / FT)
        acf.set("_gear/%d/_leg_len" % g, (gy - y_axle) / FT)
        acf.set("_gear/%d/_tire_radius" % g, TIRE_R / FT)
        acf.set("_gear/%d/_tire_swidth" % g, TIRE_W / 2.0 / FT)
        acf.set("_gear/%d/_stat_def" % g, GEAR_STAT_DEF / FT)
        acf.set("_gear/%d/_gear_load_fraction" % g, frac)
        acf.set("_gear/%d/_gear_can_retract" % g, 1)
        acf.set("_gear/%d/_cyc_time" % g, GEAR_CYCLE_S)
        acf.set("_gear/%d/_gear_brakes" % g, 0 if g == 0 else 1)
        load = frac * m_max
        scon = load / (GEAR_STAT_DEF / FT)
        acf.set("_gear/%d/_scon" % g, scon)
        acf.set("_gear/%d/_damp" % g, GEAR_ZETA[g] * 2.0 * math.sqrt(scon * load / G_FT))
        acf.set("_gear/%d/_gear_def_empty" % g, GEAR_STAT_DEF / FT)
        acf.set("_gear/%d/_gear_def_gross" % g, GEAR_STAT_DEF / FT)
        acf.set("_part/%d/_part_specs_invis" % (85 + g), 1)   # the OBJ draws the gear, animated with the same datarefs
    for g in range(3, 10):
        acf.set("_gear/%d/_gear_type" % g, 0)
    acf.set("_gear/1/_latR", 90.0)     # mains fold inwards into the wing
    acf.set("_gear/2/_latR", -90.0)
    acf.set("_gear/0/_lonR", -90.0)    # nose wheel folds back
    acf.set("_gear/0/_steerdeg_lospeed", NOSE_STEER_LO_DEG)
    acf.set("_gear/0/_steerdeg_hispeed", NOSE_STEER_HI_DEG)
    acf.set("acf/_nw_side_k", 1.0)     # the nose wheel holds the aircraft sideways, as a tyre does
    # gear and flap actuators on bus 0, where the battery is
    for rel in ("rel_gear_act", "rel_flap_act"):
        acf.set("acf/_esys_bus_bit_add/" + rel, 1)
    th_c, height, _ = ground_attitude(acf)
    acf.set("acf/_the_eqlbm", math.degrees(th_c))
    acf.set("acf/_h_eqlbm", height(acf.f("acf/_cgY"), acf.f("acf/_cgZ")))

    # --- 6. controls and flaps
    acf.set("acf/_elev1_up", ELEV_UP_DEG)
    acf.set("acf/_elev1_dn", ELEV_DN_DEG)
    acf.set("acf/_elev1_cratR", ELEV_CRAT)
    acf.set("acf/_elev1_cratT", ELEV_CRAT)
    acf.set("acf/_ailn1_up", AILN_UP_DEG)
    acf.set("acf/_ailn1_dn", AILN_DN_DEG)
    acf.set("acf/_ailn1_cratR", AILN_CRAT)
    acf.set("acf/_ailn1_cratT", AILN_CRAT)
    acf.set("acf/_rudd1_lf", RUDD_DEG)
    acf.set("acf/_rudd1_rt", RUDD_DEG)
    acf.set("acf/_rudd1_cratR", RUDD_CRAT)
    acf.set("acf/_rudd1_cratT", RUDD_CRAT)
    acf.set("acf/_ailn_def_time", AILN_DEF_TIME_S)
    acf.set("acf/_elev_def_time", ELEV_DEF_TIME_S)
    acf.set("acf/_rudd_def_time", RUDD_DEF_TIME_S)
    # full deflection at any speed (Plane Maker's new aircraft leave the speed band at 0-0)
    for c in ("ailn1", "ailn2", "elev1", "elev2", "rudd1", "rudd2", "spoi1", "spoi2", "yawbr"):
        acf.set("acf/_%s_v2" % c, 999.0)
    acf.set("acf/_flap_EQ", 1)
    acf.set("acf/_flap1_type", 1)
    acf.set("acf/_flap_detents", len(FLAP_DN_DEG))
    for i, v in enumerate(FLAP_DN_DEG):
        acf.set("acf/_flap1_dn/%d" % (i + 1), v)
    acf.set("acf/_flap1_cratR", FLAP_CRAT)
    acf.set("acf/_flap1_cratT", FLAP_CRAT)
    acf.set("acf/_flap1_cl", FLAP_CL)
    acf.set("acf/_flap1_cd", FLAP_CD)
    acf.set("acf/_flap1_cm", FLAP_CM)
    acf.set("acf/_flap_ext_time", FLAP_TIME_S)
    acf.set("acf/_flap_ret_time", FLAP_TIME_S)
    acf.set("acf/_wing_mid_dihed_per_g", 0.0)        # a stiff wing, no flex

    # --- 7. drag and speeds
    acf.set("acf/_additonal_gear_flatplate", GEAR_FLATPLATE_M2 / FT ** 2)
    for n, v in (("Vso", VSO_MS), ("Vs", VS_MS), ("Vfe1", VFE1_MS), ("Vfem", VFEM_MS), ("Vno", VNO_MS),
                 ("Vne", VNE_MS), ("Vle", VLE_MS)):
        acf.set("acf/_%s_kts" % n, v / KT)
    acf.set("acf/_gear_warn_below_kias", 1.5 * VS_MS / KT)
    acf.set("acf/_snd_kias", VNO_MS / KT)
    acf.set("acf/_Gpos", G_POS)
    acf.set("acf/_Gneg", G_NEG)

    # --- 8. look: one OBJ for the whole aircraft, outside views
    acf.set("_obja/count", 1)
    for name, v in (("_obj_flags", 24), ("_v10_att_file_stl", OBJ_NAME), ("_v10_att_part", -1),
                    ("_v10_att_phi_ref", 0.0), ("_v10_att_psi_ref", 0.0), ("_v10_att_the_ref", 0.0),
                    ("_v10_att_x_acf_prt_ref", 0.0), ("_v10_att_y_acf_prt_ref", 0.0),
                    ("_v10_att_z_acf_prt_ref", 0.0), ("_v10_is_internal", 0), ("_v10_steers_with_gear", 0)):
        acf.add("_obja/0/" + name, v)

    # --- 9. names
    for n, v in (("_name", NAME), ("_author", AUTHOR), ("_descrip", DESCRIP), ("_ICAO", "T28"),
                 ("_callsign", "T28RC"), ("_manufacturer", "INAV HITL"), ("_studio", "INAV HITL")):
        acf.add("acf/" + n, v)
    acf.set("acf/_tailnum", "T28RC")
    acf.set("acf/_is_experimental", 1)

    # --- writing
    os.makedirs(os.path.join(out_dir, "objects"), exist_ok=True)
    out_acf = os.path.join(out_dir, OUT_NAME + ".acf")
    data = "\n".join(acf.lines_out())
    if "\u2014" in data:
        raise SystemExit("em dash in the ACF")
    with open(out_acf, "wb") as f:
        f.write(data.encode("latin-1"))
    for n in ("_paint.png", "_paint2.png"):
        write_png_solid(os.path.join(out_dir, OUT_NAME + n), (178, 180, 184))
    from PIL import Image
    img = Image.open(os.path.join(BLENDER_DIR, "out", "preview_three_quarter.png")).convert("RGB")
    w, h = img.size
    crop = img.crop((0, (h - w * 450 // 800) // 2, w, (h + w * 450 // 800) // 2))
    crop.resize((800, 450), Image.LANCZOS).save(os.path.join(out_dir, OUT_NAME + "_icon11.png"))
    crop.resize((174, 107), Image.LANCZOS).save(os.path.join(out_dir, OUT_NAME + "_icon11_thumb.png"))
    for n in (OBJ_NAME, "t28_rc.png"):
        shutil.copyfile(os.path.join(BLENDER_DIR, "out", n), os.path.join(out_dir, "objects", n))
    # the earlier model's 2D panel art belongs to no one here any more
    old_cockpit = os.path.join(out_dir, "cockpit")
    if os.path.isdir(old_cockpit):
        shutil.rmtree(old_cockpit)
        rep.append("removed the old cockpit/ folder")
    if not os.path.exists(os.path.join(out_dir, "plugins", "INAV-X-Plane-HITL", "64", "win.xpl")):
        rep.append("WARNING: no INAV HITL plugin in %s\\plugins" % out_dir)
    rep.append("written %s (%d properties)" % (out_acf, len(acf.vals)))
    return rep


def geometry_report(acf):
    lines = []
    for p in [9, 11, 13, 17, FIN_PART]:
        if not acf.part_eq(p):
            continue
        sw = math.radians(acf.f("_wing/%d/_sweep_design" % p))
        di = math.radians(acf.f("_wing/%d/_dihed_design" % p))
        L = acf.f("_wing/%d/_semilen_SEG" % p) * FT
        r = [acf.f("_part/%d/_part_%s" % (p, a)) * FT for a in "xyz"]
        t = [r[0] + L * math.cos(sw) * math.cos(di), r[1] + L * math.cos(sw) * math.sin(di), r[2] + L * math.sin(sw)]
        cr, ctp = acf.f("_wing/%d/_Croot" % p) * FT, acf.f("_wing/%d/_Ctip" % p) * FT
        lines.append("  part %2d: root x %.3f y %.3f LE z %.3f chord %.3f | tip x %.3f y %.3f LE z %.3f chord %.3f"
                     % (p, r[0], r[1], r[2] - cr / 4, cr, t[0], t[1], t[2] - ctp / 4, ctp))
    lines.append("  prop: centre (%.3f, %.3f, %.4f) m, diameter %.4f m, %d blades"
                 % (acf.f("_part/0/_part_x") * FT, acf.f("_part/0/_part_y") * FT, acf.f("_part/0/_part_z") * FT,
                    2 * acf.f("_wing/0/_semilen_SEG") * FT, int(acf.f("_prop/0/_num_blades"))))
    for g, x, y, z in gear_contacts(acf):
        lines.append("  gear %d: contact at rest (%.3f, %.4f, %.3f) m, load %.3f" % (
            g, x * FT, y * FT, z * FT, acf.f("_gear/%d/_gear_load_fraction" % g)))
    lines.append("  CG z %.4f m (limits %.4f / %.4f)" % (
        acf.f("acf/_cgZ") * FT, acf.f("acf/_cgZ_fwd") * FT, acf.f("acf/_cgZ_aft") * FT))
    return lines


def check(xp):
    out_dir = os.path.join(xp, OUT_REL)
    path = os.path.join(out_dir, OUT_NAME + ".acf")
    raw = open(path, "rb").read()
    out = Acf(path)
    base = Acf(BASE_ACF)
    ok = True
    print("Structure")
    print("  CR in the file: %s" % (b"\r" in raw))
    ok &= b"\r" not in raw
    lost = [n for n in base.vals if n not in out.vals]
    print("  properties of the base missing: %d | added: %d | total %d" % (
        len(lost), len(out.vals) - len(base.vals) + len(lost), len(out.vals)))
    ok &= not lost
    words = sorted({w for w in re.findall(r"[A-Za-z_]*(?:c400|olumbia|cockpit/|\.dds)\w*", out.text)})
    print("  references to other aircraft or panel art:", words or "none")
    ok &= not words
    print("Geometry (metres from the spinner tip)")
    so = span_ft(out) * FT
    ao = wing_area_ft2(out) * FT * FT
    print("  span %.4f m, wing area %.4f m2, aspect ratio %.2f, wing loading %.1f kg/m2"
          % (so, ao, so * so / ao, out.f("acf/_m_max") * LB / ao))
    for ln in geometry_report(out):
        print(ln)
    tho, ho, pts = ground_attitude(out)
    print("  on the ground %.2f deg nose up, CG %.1f mm above ground, prop clearance %.1f mm"
          % (math.degrees(tho), ho(out.f("acf/_cgY"), out.f("acf/_cgZ")) * FT * 1000,
             prop_clearance_ft(out, out.f("_wing/0/_semilen_SEG")) * FT * 1000))
    print("Mass %.3f kg, motor %.0f W at the shaft, prop %.1fx%.1f in, %d blades" % (
        out.f("acf/_m_max") * LB, out.f("acf/_power_max_limit") * HP, PROP_DIAM_IN, PROP_PITCH_IN, PROP_BLADES))
    print("Flaps %s deg, elevator %s/%s deg, ailerons %s/%s deg" % (
        [out.get("acf/_flap1_dn/%d" % i) for i in range(int(out.get("acf/_flap_detents")) + 1)],
        out.get("acf/_elev1_up"), out.get("acf/_elev1_dn"), out.get("acf/_ailn1_up"), out.get("acf/_ailn1_dn")))
    afl_dirs = [os.path.join(xp, "Airfoils"), os.path.join(out_dir, "airfoils")]
    afl = sorted({v for n, v in out.vals.items() if "_afl_file_" in n and v})
    missing_afl = [a for a in afl if not any(os.path.exists(os.path.join(d, a)) for d in afl_dirs)]
    print("Airfoils %s, missing %s" % (afl, missing_afl))
    shown = [p for p in range(95) if out.vals.get("_part/%d/_part_specs_eq" % p) == "1"
             and out.vals.get("_part/%d/_part_specs_invis" % p) != "1"]
    obj = os.path.join(out_dir, "objects", out.get("_obja/0/_v10_att_file_stl"))
    print("Parts drawn by Plane Maker: %s | object %s present: %s" % (shown, OBJ_NAME, os.path.exists(obj)))
    ok &= not missing_afl and not shown and os.path.exists(obj)
    print("RESULT:", "ok" if ok else "PROBLEMS")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xp", default=XP_DEFAULT)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    if not a.check:
        for ln in build(a.xp):
            print(ln)
    sys.exit(0 if check(a.xp) else 1)


if __name__ == "__main__":
    main()
