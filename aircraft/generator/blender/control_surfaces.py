"""Wing, tailplane and fin of the T-28 with their control surfaces as separate pieces (flaps, ailerons, elevator,
rudder), each with its hinge line for the X-Plane animation. Used by t28_model.py.
Blender frame: +X right, +Y forward, +Z up; s = -Y."""
import math

from mathutils import Vector

XH_WING = 0.75      # flap and aileron hinge, chord fraction
XH_TAIL = 0.68      # elevator and rudder hinge
FLAP = (0.0775, 0.4025)      # span stations of the flaps (physics: flaps on 0-0.411)
AILERON = (0.4225, 0.5525)   # ailerons (physics 0.411-0.560)
ELEVATOR = (0.03, 0.20)
RUDDER = (0.03, 0.13)        # rudder, heights above the fin root line (the fin is 0.16 tall, rounded above 0.136)
GAP = 0.0025                 # spanwise gap at the ends of a control surface


def naca_xz(thickness, camber, x):
    m, p = camber, 0.4
    yt = 5 * thickness * (0.2969 * math.sqrt(max(x, 0)) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3
                          - 0.1036 * x ** 4)
    if m:
        yc = m / p ** 2 * (2 * p * x - x * x) if x < p else m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x * x)
    else:
        yc = 0.0
    return yc, yt


def segment(thickness, camber, x0, x1, n=20):
    """Closed ring of the airfoil between chord fractions x0 and x1: upper surface x1 -> x0, lower x0 -> x1.
    Always 2 (n + 1) points, so pieces cut at different places still loft together."""
    xs = [x0 + (x1 - x0) * (1 - math.cos(math.pi * i / n)) / 2 for i in range(n + 1)]
    upper = [(x, naca_xz(thickness, camber, x)[0] + naca_xz(thickness, camber, x)[1]) for x in reversed(xs)]
    lower = [(x, naca_xz(thickness, camber, x)[0] - naca_xz(thickness, camber, x)[1]) for x in xs]
    return upper + lower


def ring_h(le, chord, thickness, camber, twist, x0, x1):
    ang = math.radians(twist)
    out = []
    for x, z in segment(thickness, camber, x0, x1):
        dy, dz = -x * chord, z * chord
        dy, dz = dy * math.cos(ang) - dz * math.sin(ang), dy * math.sin(ang) + dz * math.cos(ang)
        out.append(le + Vector((0, dy, dz)))
    return out


def hinge_h(le, chord, thickness, camber, twist, xh):
    ang = math.radians(twist)
    dy, dz = -xh * chord, naca_xz(thickness, camber, xh)[0] * chord
    dy, dz = dy * math.cos(ang) - dz * math.sin(ang), dy * math.sin(ang) + dz * math.cos(ang)
    return le + Vector((0, dy, dz))


def ring_v(le, chord, thickness, x0, x1):
    return [le + Vector((z * chord, -x * chord, 0)) for x, z in segment(thickness, 0.0, x0, x1)]


def wing_section(D, x):
    """Leading edge, chord, thickness, camber and twist of the wing at span |x| (rounded tip included)."""
    a = abs(x)
    t = a / D["half_span"]
    chord = D["root_chord"] + (D["tip_chord"] - D["root_chord"]) * t
    le_s = D["wing_le_s"] + D["wing_le_sweep"] * t
    z = D["wing_z"] + a * math.tan(math.radians(D["dihedral"]))
    thickness = D["wing_thick_root"] + (D["wing_thick_tip"] - D["wing_thick_root"]) * t
    if t > 0.92:
        f = (t - 0.92) / 0.08
        chord *= math.sqrt(max(0.02, 1 - f * f))
        le_s += (D["tip_chord"] - chord) * 0.35
    return Vector((x, -le_s, z)), chord, thickness, 0.02, 2.0 - 2.5 * t


def wing_with_surfaces(D, side, loft):
    """Main wing piece of one side ("r" or "l") plus its flap and aileron.
    Returns (main, {name: (obj, pivot, axis)})."""
    sg = 1 if side == "r" else -1
    f0, f1 = FLAP
    a0, a1 = AILERON
    stations = [(0.0, 1.0), (f0, 1.0), (f0, XH_WING), ((f0 + f1) / 2, XH_WING), (f1, XH_WING), (f1, 1.0),
                (a0, 1.0), (a0, XH_WING), ((a0 + a1) / 2, XH_WING), (a1, XH_WING), (a1, 1.0),
                (0.575, 1.0), (0.590, 1.0), (0.600, 1.0), (0.607, 1.0), (D["half_span"], 1.0)]
    rings = []
    for x, x1 in stations:
        le, c, th, cam, tw = wing_section(D, sg * x)
        rings.append(ring_h(le, c, th, cam, tw, 0.0, x1))
    main = loft("wing_" + side, rings, True, True)
    moving = {}
    for name, (u0, u1) in (("flap_" + side, FLAP), ("aileron_" + side, AILERON)):
        xs = [u0 + GAP, (u0 + u1) / 2, u1 - GAP]
        rings = []
        for x in xs:
            le, c, th, cam, tw = wing_section(D, sg * x)
            rings.append(ring_h(le, c, th, cam, tw, XH_WING + 0.004, 1.0))
        ob = loft(name, rings, True, True)
        h = [hinge_h(*wing_section(D, sg * x), XH_WING) for x in (u0, u1)]
        # axis along +X on both sides: a positive angle then moves the trailing edge down on both wings
        p_in, p_out = (h[0], h[1]) if sg > 0 else (h[1], h[0])
        moving[name] = (ob, p_in, (p_out - p_in).normalized())
    return main, moving


def tail_section(D, x):
    a = abs(x)
    t = a / D["tail_half_span"]
    chord = D["tail_root_chord"] + (D["tail_tip_chord"] - D["tail_root_chord"]) * t
    rounded = t > 0.85
    if rounded:
        chord *= math.sqrt(max(0.03, 1 - ((t - 0.85) / 0.15) ** 2))
    le_s = D["tail_le_s"] + D["tail_sweep"] * t + (D["tail_tip_chord"] - chord) * 0.4 * rounded
    return Vector((x, -le_s, D["tail_z"])), chord, 0.09, 0.0, 0.0


def tail_with_elevator(D, side, loft):
    sg = 1 if side == "r" else -1
    e0, e1 = ELEVATOR
    stations = [(0.0, 1.0), (e0, 1.0), (e0, XH_TAIL), ((e0 + e1) / 2, XH_TAIL), (e1, XH_TAIL), (e1, 1.0),
                (0.215, 1.0), (0.23, 1.0), (0.245, 1.0), (0.255, 1.0), (D["tail_half_span"], 1.0)]
    rings = [ring_h(*tail_section(D, sg * x), 0.0, x1) for x, x1 in stations]
    main = loft("tail_" + side, rings, True, True)
    xs = [e0 + GAP, (e0 + e1) / 2, e1 - GAP]
    ob = loft("elevator_" + side, [ring_h(*tail_section(D, sg * x), XH_TAIL + 0.004, 1.0) for x in xs], True, True)
    h = [hinge_h(*tail_section(D, sg * x), XH_TAIL) for x in (e0, e1)]
    p_in, p_out = (h[0], h[1]) if sg > 0 else (h[1], h[0])
    return main, ("elevator_" + side, ob, p_in, (p_out - p_in).normalized())


def fin_section(D, dz):
    t = dz / D["fin_height"]
    chord = D["fin_root_chord"] + (D["fin_tip_chord"] - D["fin_root_chord"]) * t
    if t > 0.85:
        chord *= math.sqrt(max(0.05, 1 - ((t - 0.85) / 0.15) ** 2))
    le_s = D["fin_le_s"] + D["fin_sweep"] * t
    return Vector((0, -le_s, 0.035 + dz)), chord


def fin_with_rudder(D, loft):
    r0, r1 = RUDDER
    height = D["fin_height"]
    rings = []
    for dz, x1 in [(0.0, 1.0), (r0, 1.0), (r0, XH_TAIL), ((r0 + r1) / 2, XH_TAIL), (r1, XH_TAIL), (r1, 1.0),
                   (0.875 * height, 1.0), (0.925 * height, 1.0), (0.9625 * height, 1.0), (0.9875 * height, 1.0),
                   (height, 1.0)]:
        le, c = fin_section(D, dz)
        rings.append(ring_v(le, c, 0.09, 0.0, x1))
    main = loft("fin", rings, True, True)
    xs = [r0 + GAP, (r0 + r1) / 2, r1 - GAP]
    rings = []
    for dz in xs:
        le, c = fin_section(D, dz)
        rings.append(ring_v(le, c, 0.09, XH_TAIL + 0.004, 1.0))
    ob = loft("rudder", rings, True, True)
    h = []
    for dz in (r0, r1):
        le, c = fin_section(D, dz)
        h.append(le + Vector((0, -XH_TAIL * c, 0)))
    return main, ("rudder", ob, h[0], (h[1] - h[0]).normalized())
