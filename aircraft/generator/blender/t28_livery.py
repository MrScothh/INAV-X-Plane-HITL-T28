"""Livery of the T-28 RC model: the E-flite red and white U.S. Navy trainer scheme painted with PIL on one 2048 px
atlas, and the matching UVs. Faces are projected by where they face: the two fuselage sides each get their own half
(so lettering is never mirrored), top and bottom share a plan view, small parts use solid swatches.
Blender frame as in t28_model.py: +X right, +Y forward, +Z up, s = -Y = metres aft of the spinner tip."""
import math
import sys

N = 2048
WHITE = (242, 242, 238)
RED = (200, 30, 36)
BLACK = (22, 22, 24)
BLUE = (28, 44, 96)
GLASS = (38, 58, 76)
YELLOW = (238, 196, 30)
RUBBER = (30, 30, 30)
METAL = (150, 152, 156)
INTERIOR = (52, 54, 58)
LINE = (205, 205, 200)

# side halves: s along u, z along v; plan halves: x along u, s along v (nose up)
S0, S1 = 0.0, 1.0
Z0, Z1 = -0.10, 0.26
X0, X1 = -0.63, 0.63
SIDE_R = (0.0, 0.5, 0.5, 1.0)     # u0, u1, v0, v1
SIDE_L = (0.5, 1.0, 0.5, 1.0)
TOP = (0.0, 0.5, 0.04, 0.5)
BOTTOM = (0.5, 1.0, 0.04, 0.5)
SWATCHES = ["glass", "black", "blades", "rubber", "spinner", "metal", "yellow", "white", "interior"]
COLOURS = dict(glass=GLASS, black=BLACK, blades=BLACK, rubber=RUBBER, spinner=RED, metal=METAL, yellow=YELLOW,
               white=WHITE, interior=INTERIOR)


def canopy_profile(t):
    """Canopy height and width (m) at fraction t of its length, shared by the mesh and the paint: a long tandem
    bubble, steeper at the windscreen than at the rear."""
    up = min(1.0, t / 0.24) ** 0.55
    down = min(1.0, (1.0 - t) / 0.30) ** 0.75
    hump = 1.0 + 0.06 * math.sin(math.pi * t)
    return (max(0.002, 0.056 * up * down * hump),
            max(0.006, 0.074 * min(1.0, t / 0.10) ** 0.5 * min(1.0, (1.0 - t) / 0.18) ** 0.5))


def uv_side(s, z, right):
    u0, u1, v0, v1 = SIDE_R if right else SIDE_L
    f = (s - S0) / (S1 - S0)
    u = u0 + (u1 - u0) * ((1 - f) if right else f)   # seen from the right the nose is on the viewer's right
    return u, v0 + (v1 - v0) * (z - Z0) / (Z1 - Z0)


def uv_plan(x, s, top):
    u0, u1, v0, v1 = TOP if top else BOTTOM
    fx = (x - X0) / (X1 - X0)
    u = u0 + (u1 - u0) * (fx if top else 1 - fx)  # seen from below the aircraft's left is on the viewer's right
    return u, v1 - (v1 - v0) * (s - S0) / (S1 - S0)


def uv_swatch(name):
    i = SWATCHES.index(name)
    return (i + 0.5) / 16.0, 0.02


def px(u, v):
    return u * N, (1 - v) * N


def assign_uv(pieces, D):
    for name, ob in pieces.items():
        me = ob.data
        uvl = me.uv_layers.new(name="UV")
        for poly in me.polygons:
            n = poly.normal
            c = poly.center
            # solid-colour parts are decided per face: a triangle spanning two swatches would show the ones between
            swatch = None
            if name == "intake":
                swatch = "interior"
            elif name == "spinner":
                swatch = "spinner"
            elif name == "blades":
                swatch = "yellow" if c.xz.length > 0.122 else "blades"
            elif name.startswith(("nose_gear", "main_gear")):
                # leg axis: the nose leg on the centreline, the mains at +-main_x; the wheels sit far from it
                if name == "nose_gear":
                    ax, a_s = 0.0, D["nose_s"]
                else:
                    ax, a_s = math.copysign(D["main_x"], c.x), D["main_s"]
                swatch = "rubber" if math.hypot(c.x - ax, -c.y - a_s) > 0.007 else "metal"
            for li in poly.loop_indices:
                co = me.vertices[me.loops[li].vertex_index].co
                s = -co.y
                if swatch:
                    uv = uv_swatch(swatch)
                elif abs(n.x) >= abs(n.z) and not name.startswith(("wing", "flap", "aileron", "elevator")):
                    uv = uv_side(s, co.z, n.x > 0 if abs(n.x) > 1e-4 else co.x >= 0)
                else:
                    uv = uv_plan(co.x, s, n.z >= 0)
                uvl.data[li].uv = uv


def font(size):
    from PIL import ImageFont
    for f in ("C:/Windows/Fonts/arialbd.ttf", "C:/Windows/Fonts/arial.ttf"):
        try:
            return ImageFont.truetype(f, size)
        except OSError:
            pass
    return ImageFont.load_default()


def star(dr, cx, cy, r, colour):
    pts = []
    for i in range(10):
        a = -math.pi / 2 + i * math.pi / 5
        rr = r if i % 2 == 0 else r * 0.382
        pts.append((cx + rr * math.cos(a), cy + rr * math.sin(a)))
    dr.polygon(pts, fill=colour)


def roundel(dr, cx, cy, r, bars=True):
    """US star-and-bar, centre (cx, cy) in pixels, roundel radius r."""
    if bars:
        bl, bh = r * 2.0, r * 0.62
        dr.rectangle([cx - bl, cy - bh / 2 - r * 0.08, cx + bl, cy + bh / 2 + r * 0.08], fill=BLUE)
        dr.rectangle([cx - bl + r * 0.08, cy - bh / 2, cx + bl - r * 0.08, cy + bh / 2], fill=WHITE)
        dr.rectangle([cx - bl + r * 0.08, cy - bh * 0.17, cx + bl - r * 0.08, cy + bh * 0.17], fill=RED)
    dr.ellipse([cx - r, cy - r, cx + r, cy + r], fill=BLUE)
    star(dr, cx, cy, r * 0.96, WHITE)


def text(img, t, cx, cy, h, colour, mirror=False, rotate=0):
    from PIL import Image, ImageDraw
    f = font(int(h))
    tmp = Image.new("RGBA", (int(h * len(t) * 0.9) + 20, int(h * 1.4)), (0, 0, 0, 0))
    ImageDraw.Draw(tmp).text((10, 0), t, font=f, fill=colour + (255,))
    tmp = tmp.crop(tmp.getbbox())
    if mirror:
        tmp = tmp.transpose(Image.FLIP_LEFT_RIGHT)
    if rotate:
        tmp = tmp.rotate(rotate, expand=True)
    img.alpha_composite(tmp, (int(cx - tmp.width / 2), int(cy - tmp.height / 2)))


def paint(path, D=None):
    from PIL import Image, ImageDraw
    img = Image.new("RGBA", (N, N), WHITE + (255,))
    dr = ImageDraw.Draw(img)
    L = (S1 - S0)

    def side_px(s, z, right):
        return px(*uv_side(s, z, right))

    def plan_px(x, s, top):
        return px(*uv_plan(x, s, top))

    def rect_side(s0, s1, z0, z1, colour, right):
        a, b = side_px(s0, z0, right), side_px(s1, z1, right)
        dr.rectangle([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])], fill=colour)

    def rect_plan(x0, x1, s0, s1, colour, top):
        a, b = plan_px(x0, s0, top), plan_px(x1, s1, top)
        dr.rectangle([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])], fill=colour)

    px_per_m_side = N * 0.5 / L
    for right in (True, False):
        # red cowl with a white pinstripe, red rear band, red fin top and rudder
        rect_side(0.0, 0.118, Z0, Z1, RED, right)
        rect_side(0.104, 0.109, Z0, Z1, WHITE, right)
        rect_side(0.655, 0.690, Z0, 0.11, RED, right)
        rect_side(0.80, 1.0, 0.150, Z1, RED, right)
        rect_side(0.94, 1.0, 0.04, Z1, RED, right)
        # anti-glare panel ahead of the canopy, canopy glass with white frames
        rect_side(0.118, 0.300, 0.070, Z1, BLACK, right)
        for k in range(120):
            t = k / 119
            s = 0.30 + 0.34 * t
            height = canopy_profile(t)[0]
            rect_side(s, s + 0.34 / 119 + 0.001, 0.084, 0.081 + height + 0.004, GLASS, right)
        for sf in (0.300, 0.405, 0.512, 0.585):
            rect_side(sf, sf + 0.006, 0.08, 0.15, WHITE, right)
        rect_side(0.30, 0.64, 0.079, 0.086, WHITE, right)
        # panel lines
        for sl in (0.118, 0.30, 0.64, 0.80):
            rect_side(sl, sl + 0.0015, Z0, 0.08, LINE, right)
        # star-and-bar, NAVY, side number, rudder hinge line
        cx, cy = side_px(0.585, 0.004, right)
        roundel(dr, cx, cy, 0.029 * px_per_m_side)
        cx, cy = side_px(0.755, 0.030, right)
        text(img, "NAVY", cx, cy, 0.036 * px_per_m_side, BLACK)
        dr = ImageDraw.Draw(img)
        cx, cy = side_px(0.215, 0.02, right)
        text(img, "314", cx, cy, 0.036 * px_per_m_side, BLACK)
        cx, cy = side_px(0.92, 0.205, right)
        text(img, "2S", cx, cy, 0.030 * px_per_m_side, WHITE)
        dr = ImageDraw.Draw(img)
        rect_side(0.935, 0.937, 0.03, Z1, LINE, right)

    px_per_m_plan = N * 0.5 / (X1 - X0)
    for top in (True, False):
        # red wing tips and stab tips, panel lines, control surface outlines
        rect_plan(X0, -0.530, S0, S1, RED, top)
        rect_plan(0.530, X1, S0, S1, RED, top)
        rect_plan(X0, -0.200, 0.80, S1, RED, top)
        rect_plan(0.200, X1, 0.80, S1, RED, top)
        rect_plan(-0.08, 0.08, S0, 0.118, RED, top)
        for sg in (-1, 1):
            # aileron and flap outlines along the trailing edge
            # physics sections: flaps on x 0-0.411, ailerons 0.411-0.560; trailing edge 0.60 -> 0.45 at the tip
            for x_a, x_b in ((0.42, 0.555), (0.07, 0.40)):
                a = plan_px(sg * x_a, 0.0, top)[0]
                b = plan_px(sg * x_b, 0.0, top)[0]
                t_a = 0.60 - 0.15 * x_a / 0.612 - 0.045
                t_b = 0.60 - 0.15 * x_b / 0.612 - 0.045
                ya, yb = plan_px(0, t_a, top)[1], plan_px(0, t_b, top)[1]
                dr.line([(a, ya), (b, yb)], fill=LINE, width=3)
                dr.line([(a, ya), (a, ya + 0.06 * N * 0.46)], fill=LINE, width=3)
            dr.line([plan_px(sg * 0.065, 0.33, top), plan_px(sg * 0.065, 0.62, top)], fill=LINE, width=3)
        if top:
            rect_plan(-0.035, 0.035, 0.118, 0.300, BLACK, True)
            rect_plan(-0.028, 0.028, 0.300, 0.640, GLASS, True)
            for sf in (0.300, 0.405, 0.512, 0.585):
                rect_plan(-0.03, 0.03, sf, sf + 0.006, WHITE, True)
            cx, cy = plan_px(-0.40, 0.415, True)
            roundel(dr, cx, cy, 0.050 * px_per_m_plan, bars=True)
        else:
            cx, cy = plan_px(0.40, 0.415, False)
            roundel(dr, cx, cy, 0.050 * px_per_m_plan, bars=True)
            cx, cy = plan_px(-0.36, 0.42, False)
            text(img, "NAVY", cx, cy, 0.075 * px_per_m_plan, BLACK, rotate=0)
            dr = ImageDraw.Draw(img)

    for name in SWATCHES:
        u, v = uv_swatch(name)
        x, y = px(u, v)
        dr.rectangle([x - N / 32 + 2, y - N * 0.02 + 2, x + N / 32 - 2, y + N * 0.02 - 2], fill=COLOURS[name])
    img.convert("RGB").save(path)


if __name__ == "__main__":
    # Blender's own Python has no PIL: t28_model.py runs this file with the system Python to paint the atlas
    paint(sys.argv[1])
