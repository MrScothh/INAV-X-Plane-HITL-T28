"""T-28 RC 1.2 m for X-Plane 11, built from scratch in Blender (headless) with the E-flite red and white U.S. Navy
trainer scheme, then written straight to an X-Plane OBJ8 with its texture. Prop, gear and control surfaces are
animated.
    blender -b --factory-startup -P t28_model.py -- OUT_DIR [--render]
OUT_DIR gets t28_rc.obj, t28_rc.png, t28_rc.blend and, with --render, preview PNGs.
Units metres. Blender frame: +X right, +Y forward, +Z up, origin at the spinner tip; X-Plane OBJ: +x right, +y up,
+z aft, origin at the ACF reference point (OFFSET below, from the ACF), so xp = (x, z, -y) - OFFSET."""
import math
import os
import subprocess
import sys

import bmesh
import bpy
from mathutils import Matrix, Vector

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
OUT = os.path.abspath(argv[0] if argv else ".")
RENDER = "--render" in argv
HERE = os.path.dirname(os.path.abspath(__file__))
os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, HERE)
import control_surfaces as CS  # noqa: E402
import t28_livery as LIV  # noqa: E402

# --- dimensions (T-28B scaled 0.1, E-flite 1224 mm span / 986 mm length); s = metres aft of the spinner tip
D = dict(
    length=0.986, half_span=0.612, root_chord=0.27, tip_chord=0.12, wing_le_s=0.33, wing_z=-0.050,
    dihedral=7.0, wing_le_sweep=0.0, wing_thick_root=0.14, wing_thick_tip=0.12,
    tail_le_s=0.81, tail_half_span=0.26, tail_root_chord=0.16, tail_tip_chord=0.09, tail_z=0.0,
    tail_sweep=0.0175, fin_le_s=0.806, fin_root_chord=0.18, fin_tip_chord=0.09,
    fin_height=0.16, fin_sweep=0.09,
    prop_radius=0.1397, prop_s=0.0524, spinner_length=0.07, spinner_radius=0.034,
    # legs as the ACF has them (build_t28.py --check: contact points at rest): nose 98 mm, mains 129 mm at 0.486 m
    nose_s=0.130, nose_z=-0.060, nose_leg=0.098, nose_wheel_radius=0.025,
    main_s=0.486, main_x=0.150, main_z=-0.040, main_leg=0.129, main_wheel_radius=0.025,
)
# The ACF reference point is the spinner tip on the thrust line, the origin of this frame too
OFFSET = Vector((0.0, 0.0, 0.0))

# fuselage stations: (s, width, height, centre z, superellipse exponent); build_t28.py reads this list for the ACF
STATIONS = [
    (0.045, 0.118, 0.118, 0.000, 2.0), (0.060, 0.142, 0.142, 0.000, 2.0), (0.110, 0.146, 0.146, 0.000, 2.0),
    (0.170, 0.142, 0.146, 0.002, 2.1), (0.250, 0.132, 0.156, 0.003, 2.3), (0.330, 0.124, 0.158, 0.002, 2.4),
    (0.450, 0.116, 0.152, 0.002, 2.4), (0.560, 0.102, 0.136, 0.005, 2.3), (0.680, 0.082, 0.112, 0.010, 2.2),
    (0.800, 0.060, 0.088, 0.012, 2.1), (0.900, 0.040, 0.062, 0.012, 2.0), (0.960, 0.024, 0.040, 0.012, 2.0),
    (0.986, 0.006, 0.012, 0.012, 2.0),
]


def superellipse(w, h, n, k):
    pts = []
    for i in range(k):
        t = 2 * math.pi * i / k
        c, s = math.cos(t), math.sin(t)
        x = (w / 2) * math.copysign(abs(c) ** (2 / n), c)
        z = (h / 2) * math.copysign(abs(s) ** (2 / n), s)
        pts.append((x, z))
    return pts


def mesh_from(name, verts, faces, smooth=True):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.validate()
    bm = bmesh.new()
    bm.from_mesh(me)
    # sections cut at a hinge collapse points onto each other: drop the zero-area faces before the normals
    bmesh.ops.dissolve_degenerate(bm, dist=1e-6, edges=bm.edges[:])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    for p in me.polygons:
        p.use_smooth = smooth
    if smooth:
        # smooth surfaces, but hinge walls, trailing edges and tips stay crisp
        me.set_sharp_from_angle(angle=math.radians(40))
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    return ob


def loft(name, rings, close_start=True, close_end=True, smooth=True):
    """Rings of equal length (lists of Vector) joined in order; ends capped with a fan."""
    k = len(rings[0])
    verts = [tuple(p) for ring in rings for p in ring]
    faces = []
    for r in range(len(rings) - 1):
        for i in range(k):
            a, b = r * k + i, r * k + (i + 1) % k
            faces.append((a, b, b + k, a + k))
    if close_start:
        c = Vector((0, 0, 0))
        for p in rings[0]:
            c += p
        verts.append(tuple(c / k))
        ci = len(verts) - 1
        faces += [(ci, (i + 1) % k, i) for i in range(k)]
    if close_end:
        c = Vector((0, 0, 0))
        for p in rings[-1]:
            c += p
        verts.append(tuple(c / k))
        ci = len(verts) - 1
        base = (len(rings) - 1) * k
        faces += [(ci, base + i, base + (i + 1) % k) for i in range(k)]
    return mesh_from(name, verts, faces, smooth)


def naca(thickness, camber, k):
    """NACA 4-digit style section, closed, k points, chord 1 from LE (0) to TE (1); returns (x, z) list."""
    m, p = camber, 0.4
    upper, lower = [], []
    for i in range(k // 2 + 1):
        beta = math.pi * i / (k // 2)
        x = (1 - math.cos(beta)) / 2
        yt = 5 * thickness * (0.2969 * math.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3
                              - 0.1036 * x ** 4)
        if m:
            yc = m / p ** 2 * (2 * p * x - x * x) if x < p else m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * x - x * x)
        else:
            yc = 0.0
        upper.append((x, yc + yt))
        lower.append((x, yc - yt))
    return upper[::-1] + lower[1:-1]


def fuselage():
    rings = []
    for s, w, h, zc, n in STATIONS:
        rings.append([Vector((x, -s, zc + z)) for x, z in superellipse(w, h, n, 48)])
    return loft("fuselage", rings, close_start=False, close_end=True)


def intake():
    # dark recessed face inside the cowl lip, with the engine's front showing through
    rings = [[Vector((x, -0.046, z)) for x, z in superellipse(0.116, 0.116, 2.0, 48)],
             [Vector((x, -0.062, z)) for x, z in superellipse(0.090, 0.090, 2.0, 48)]]
    return loft("intake", rings, close_start=False, close_end=True)


def canopy():
    rings = []
    s0, s1 = 0.30, 0.64
    for i in range(25):
        t = i / 24
        s = s0 + (s1 - s0) * t
        height, width = LIV.canopy_profile(t)
        base = 0.006 + 0.075
        ring = []
        for j in range(17):
            a = math.pi * j / 16
            ring.append(Vector((width / 2 * math.cos(a), -s, base + height * math.sin(a))))
        rings.append(ring)
    return loft("canopy", rings, True, True)


def spinner():
    rings = []
    for i in range(9):
        t = i / 8
        r = D["spinner_radius"] * math.sin(math.pi / 2 * t) ** 0.8
        rings.append([Vector((r * math.cos(a), -D["spinner_length"] * t, r * math.sin(a)))
                      for a in [2 * math.pi * j / 24 for j in range(24)]])
    return loft("spinner", rings, close_start=False, close_end=True)


def blades():
    verts, faces = [], []
    for b in range(3):
        ang = 2 * math.pi * b / 3
        rot = Matrix.Rotation(ang, 4, "Y")
        sections = []
        for i in range(7):
            t = i / 6
            r = 0.03 + (D["prop_radius"] - 0.03) * t
            chord = 0.026 * (1 - 0.55 * t) * (1 if t < 0.95 else 0.6)
            pitch = math.radians(55 - 38 * t)
            ring = []
            for x, z in naca(0.10, 0.03, 16):
                p = Vector(((x - 0.35) * chord * math.cos(pitch),
                            -D["prop_s"] + (x - 0.35) * chord * math.sin(pitch), 0))
                p = Vector((p.x, p.y + z * chord * 0.5, r))
                ring.append(rot @ p)
            sections.append(ring)
        base = len(verts)
        k = len(sections[0])
        for ring in sections:
            verts += [tuple(v) for v in ring]
        for r_ in range(len(sections) - 1):
            for i in range(k):
                a, b_ = base + r_ * k + i, base + r_ * k + (i + 1) % k
                faces.append((a, b_, b_ + k, a + k))
    return mesh_from("blades", verts, faces)


def wheel(name, centre, radius, width):
    rings = []
    for dx, rr in ((-width / 2, radius * 0.55), (-width / 2, radius * 0.92), (-width / 2 * 0.6, radius),
                   (width / 2 * 0.6, radius), (width / 2, radius * 0.92), (width / 2, radius * 0.55)):
        rings.append([centre + Vector((dx, rr * math.cos(a), rr * math.sin(a)))
                      for a in [2 * math.pi * j / 20 for j in range(20)]])
    return loft(name, rings, True, True)


def leg(name, top, bottom, r=0.004):
    axis = (bottom - top).normalized()
    perp = axis.orthogonal().normalized()
    perp2 = axis.cross(perp)
    rings = []
    for p in (top, bottom):
        rings.append([p + r * (math.cos(a) * perp + math.sin(a) * perp2)
                      for a in [2 * math.pi * j / 8 for j in range(8)]])
    return loft(name, rings, True, True)


def join(name, objects):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objects:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.join()
    objects[0].name = name
    return objects[0]


def subdivide(ob, levels):
    mod = ob.modifiers.new("sub", "SUBSURF")
    mod.levels = levels
    mod.render_levels = levels
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    ob.modifiers.clear()
    old = ob.data
    ob.data = me
    bpy.data.meshes.remove(old)
    for p in me.polygons:
        p.use_smooth = True


# --- build
bpy.ops.wm.read_factory_settings(use_empty=True)
pieces = {}
pieces["fuselage"] = fuselage()
subdivide(pieces["fuselage"], 1)
pieces["intake"] = intake()
pieces["canopy"] = canopy()
subdivide(pieces["canopy"], 1)
# wing, tailplane and fin with their control surfaces as separate animated pieces
MOVING = {}
halves = []
for side in ("r", "l"):
    body, moving = CS.wing_with_surfaces(D, side, loft)
    halves.append(body)
    for name, (ob, pivot, axis) in moving.items():
        pieces[name] = ob
        MOVING[name] = (pivot, axis)
pieces["wing"] = join("wing", halves)
tails = []
for side in ("r", "l"):
    body, (name, ob, pivot, axis) = CS.tail_with_elevator(D, side, loft)
    tails.append(body)
    pieces[name] = ob
    MOVING[name] = (pivot, axis)
body, (name, ob, pivot, axis) = CS.fin_with_rudder(D, loft)
tails.append(body)
pieces[name] = ob
MOVING[name] = (pivot, axis)
pieces["tail"] = join("tail", tails)
pieces["spinner"] = spinner()
pieces["blades"] = blades()
# gear: legs and wheels, each leg with its wheel is one animated group
ns, nz = D["nose_s"], D["nose_z"]
pieces["nose_gear"] = join("nose_gear", [
    leg("leg_n", Vector((0, -ns, nz)), Vector((0, -ns, nz - D["nose_leg"]))),
    wheel("wheel_n", Vector((0, -ns, nz - D["nose_leg"])), D["nose_wheel_radius"], 0.016)])
for side, sg in (("l", -1), ("r", 1)):
    top = Vector((sg * D["main_x"], -D["main_s"], D["main_z"]))
    bottom = top + Vector((0, 0, -D["main_leg"]))
    pieces["main_gear_" + side] = join("main_gear_" + side, [
        leg("leg_" + side, top, bottom),
        wheel("wheel_" + side, bottom + Vector((sg * 0.012, 0, 0)), D["main_wheel_radius"], 0.02)])

# --- UVs and texture
LIV.assign_uv(pieces, D)
png = os.path.join(OUT, "t28_rc.png")
# Blender's own Python has no PIL: the atlas is painted by the system Python
subprocess.run(["python", os.path.join(HERE, "t28_livery.py"), png], check=True)

mat = bpy.data.materials.new("livery")
mat.use_nodes = True
nt = mat.node_tree
tex = nt.nodes.new("ShaderNodeTexImage")
tex.image = bpy.data.images.load(png)
bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
bsdf.inputs["Roughness"].default_value = 0.45
for ob in pieces.values():
    ob.data.materials.append(mat)

# --- X-Plane OBJ8
ANIM = {
    "blades": ("rot", Vector((0, 0, 0)), Vector((0, -1, 0)), 0, 360, 0, 360,
               "sim/flightmodel2/engines/prop_rotation_angle_deg[0]"),
    "nose_gear": ("rot", Vector((0, -ns, nz)), Vector((1, 0, 0)), -95, 0, 0, 1,
                  "sim/flightmodel2/gear/deploy_ratio[0]"),
    "main_gear_l": ("rot", Vector((-D["main_x"], -D["main_s"], D["main_z"])), Vector((0, 1, 0)), -88, 0, 0, 1,
                    "sim/flightmodel2/gear/deploy_ratio[1]"),
    "main_gear_r": ("rot", Vector((D["main_x"], -D["main_s"], D["main_z"])), Vector((0, 1, 0)), 88, 0, 0, 1,
                    "sim/flightmodel2/gear/deploy_ratio[2]"),
}


# control surfaces: the deflection X-Plane's physics applies, in degrees, positive = trailing edge down (rudder: right);
# even wing indices are the left side, 8 the tailplane, 10 the fin. Drawn 5x (up to 9 degrees of real deflection,
# 45 drawn): in flight INAV moves them 1-5 degrees, which nobody sees from the chase view (3x was still too little).
# Flaps are drawn 1:1
DEFL = "sim/flightmodel2/wing/%s1_deg[%d]"
DRAW_GAIN = 5
LIMIT = 45.0 / DRAW_GAIN
for name, (pivot, axis) in MOVING.items():
    if name.startswith("flap"):
        ANIM[name] = ("rot", pivot, axis, 0, 25, 0, 1, "sim/flightmodel2/controls/flap1_deploy_ratio")
    elif name.startswith("aileron"):
        ANIM[name] = ("rot", pivot, axis, -45, 45, -LIMIT, LIMIT, DEFL % ("aileron", 1 if name == "aileron_r" else 0))
    elif name.startswith("elevator"):
        ANIM[name] = ("rot", pivot, axis, -45, 45, -LIMIT, LIMIT, DEFL % ("elevator", 8))
    elif name == "rudder":
        ANIM[name] = ("rot", pivot, axis, -45, 45, -LIMIT, LIMIT, DEFL % ("rudder", 10))


def to_xp(v):
    return Vector((v.x, v.z, -v.y)) - Vector((OFFSET.x, OFFSET.z, -OFFSET.y))


def write_obj(path):
    vt, idx, groups = [], [], []
    for name, ob in pieces.items():
        me = ob.data
        me.calc_loop_triangles()
        uv = me.uv_layers.active.data
        start = len(idx)
        for tri in me.loop_triangles:
            for li in reversed(tri.loops):  # X-Plane winding is the reverse of Blender's after the axis swap
                lp = me.loops[li]
                co = to_xp(me.vertices[lp.vertex_index].co)
                n = me.corner_normals[li].vector if hasattr(me, "corner_normals") else lp.normal
                nn = Vector((n.x, n.z, -n.y))
                u, v = uv[li].uv
                vt.append((co.x, co.y, co.z, nn.x, nn.y, nn.z, u, v))
                idx.append(len(vt) - 1)
        groups.append((name, start, len(idx) - start))
    lines = ["I", "800", "OBJ", "", "TEXTURE t28_rc.png", "POINT_COUNTS %d 0 0 %d" % (len(vt), len(idx)), ""]
    lines += ["VT %.5f %.5f %.5f %.4f %.4f %.4f %.5f %.5f" % t for t in vt]
    lines.append("")
    i = 0
    while i + 10 <= len(idx):
        lines.append("IDX10 " + " ".join(str(x) for x in idx[i:i + 10]))
        i += 10
    while i < len(idx):
        lines.append("IDX %d" % idx[i])
        i += 1
    lines.append("")
    lines.append("ATTR_no_cull")
    for name, start, n in groups:
        if name in ANIM:
            _, pivot, axis, r1, r2, v1, v2, dref = ANIM[name]
            p = to_xp(pivot)
            a = Vector((axis.x, axis.z, -axis.y))
            lines += ["ANIM_begin",
                      "ANIM_trans %.5f %.5f %.5f %.5f %.5f %.5f 0 0 none" % (p.x, p.y, p.z, p.x, p.y, p.z),
                      "ANIM_rotate %.4f %.4f %.4f %g %g %g %g %s" % (a.x, a.y, a.z, r1, r2, v1, v2, dref),
                      "ANIM_trans %.5f %.5f %.5f %.5f %.5f %.5f 0 0 none" % (-p.x, -p.y, -p.z, -p.x, -p.y, -p.z),
                      "TRIS %d %d" % (start, n), "ANIM_end"]
        else:
            lines.append("TRIS %d %d" % (start, n))
    open(path, "w", newline="\n").write("\n".join(lines) + "\n")
    return len(vt), len(idx)


nvt, nidx = write_obj(os.path.join(OUT, "t28_rc.obj"))
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, "t28_rc.blend"))
print("OBJ: %d vertices, %d indices" % (nvt, nidx))

if RENDER:
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_WORKBENCH"
    sc.display.shading.light = "STUDIO"
    sc.display.shading.color_type = "TEXTURE"
    sc.render.resolution_x, sc.render.resolution_y = 1280, 800
    # no stamp metadata in the PNGs: it would carry this file's path, the date and the time
    for field in ("filename", "date", "time", "frame", "frame_range", "camera", "lens", "scene", "render_time",
                  "memory", "hostname", "marker", "sequencer_strip", "note"):
        if hasattr(sc.render, "use_stamp_" + field):
            setattr(sc.render, "use_stamp_" + field, False)
    w = bpy.data.worlds.new("w")
    sc.world = w
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam
    centre = Vector((0, -0.45, 0.0))
    for name, pos in (("three_quarter", (1.1, 0.35, 0.45)), ("side", (1.4, -0.45, 0.02)), ("top", (0.001, -0.45, 1.6)),
                      ("rear", (-0.6, -1.7, 0.35)), ("below", (0.5, -0.2, -1.2))):
        cam.location = Vector(pos)
        cam.rotation_euler = (centre - cam.location).to_track_quat("-Z", "Y").to_euler()
        sc.render.filepath = os.path.join(OUT, "preview_%s.png" % name)
        bpy.ops.render.render(write_still=True)
    print("previews written")
