"""
High-Five Robot - parametric parts generator for Blender (tested on 4.3).

Run headless from the project root:
    /Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \
        --python cad/highfive_robot.py -- --render

Outputs:
    stl/*.stl                 printable parts, already in print orientation
    cad/highfive_robot.blend  posable assembly (rotate the *_pivot empties)
    renders/*.png             preview renders (with --render)

Arm: pan (MG995) -> shoulder (MG995) -> elbow (MG995) -> wrist (MG90S) -> twist (MG90S) -> hand.
Camera: Seeed XIAO ESP32S3 Sense with a wide OV2640, in a box on a wedge on the base's 30 deg seat.

Units: 1 Blender unit = 1 mm. World frame: +X = forward (towards the person),
+Z = up, pan axis = Z axis, pan servo flange seat at Z=0, desk at BASE_BOTTOM.
Each arm link is modelled in its own frame: proximal joint axis on the Y axis at
the origin, link pointing +X. Positive joint angle = link rotates upwards.

!!! Measure your servos + horns with calipers, fix the SERVOS values below,
!!! then print the fit_test_*.stl pieces before printing the big parts.
"""
import bpy
import bmesh
import math
import os
import sys
from types import SimpleNamespace as NS
from mathutils import Matrix, Vector

# ----------------------------------------------------------------------------
# Parameters
# ----------------------------------------------------------------------------
# Servo local frame: origin on the output shaft axis at the flange underside,
# x along the case (body spans -off .. L-off), z out along the shaft.
SERVOS = {
    "MG995": NS(L=40.7, W=19.7, off=10.2, below=26.5, case_top=10.3,
                flange_L=54.5, flange_t=2.5, pitch_L=48.1, pitch_W=10.0, pilot=2.6,
                horn="round", horn_d=20.4, horn_t=2.5, horn_z=19.5, access_d=7.0,
                seat=0.6),   # measured 2026-10-06: 21.0 recess fits best; horn top ~0.7 mm above horn_z
    "MG90S": NS(L=22.8, W=12.2, off=5.9, below=15.9, case_top=6.8,
                flange_L=32.2, flange_t=2.5, pitch_L=27.8, pitch_W=0.0, pilot=1.8,
                horn="bar", horn_len=32.0, horn_w=6.0, hub_d=7.0, horn_t=1.8, seat=0.0,
                horn_z=14.5, access_d=5.0),
}
MG995, MG90S = SERVOS["MG995"], SERVOS["MG90S"]

CLR = 0.4              # print clearance per side for pockets
CRADLE_CLR = 0.6       # servo windows in the arm cradles: the 7 mm wall + ribs make a long channel, so looser
HORN_CLR = 0.55        # round horn recess clearance per side: 21.5 mm for the measured 20.4 mm horn
                       # (2026-10-07: 21.0 barely fit printed flat on the bed, not at all printed sideways)
HORN_CLR_SIDE = 0.65   # sideways (teardrop) recesses in the arm forks: 21.7 mm, the roomier of the two that fit
                       # in the 2026-10-07 sideways test
HORN_CHAMFER = 0.5     # 45-degree flare at a recess mouth that prints on the bed (eats the first-layer lip)
WINDOW_EXTRA = 1.0     # closed (slide-through) windows are this much longer at each end; more would reach the screw pilots
M3_CLEAR = 3.4
M3_NUT_R = 3.35        # hex nut trap circumradius (5.5 mm flats + clearance)
CAM_NUT_R = 3.55       # camera seat nut pockets, printed tilted 30 deg: 6.15 mm across the flats
CAM_SEAT_HOLE = 3.8    # camera seat screw holes, printed tilted 30 deg
WASHER = 5.0           # pivot gap between the fork's idler plate and the cradle's idler wall: room to slide the
                       # horn onto the spline during assembly, then filled by a printed C-spacer (pivot_spacer)
SPACER_CLR = 0.3       # spacer is this much thinner than the gap

# Base / pan
BASE_R = 50.0
BASE_BOTTOM = -34.0    # desk level
FOOT_R = 68.0
FOOT_T = 5.0
COLLAR_IN_R = 42.0
DISC_GAP = 0.5
DISC_T = 6.0
TILT_H = 45.0          # turntable top -> shoulder axis

# Links
L1 = 150.0             # shoulder -> elbow
L2 = 140.0             # elbow -> wrist
LINK_H = 15.0          # links are 2*LINK_H thick (flat top/bottom for printing)

# Hand
HAND_DROP = 17.0       # wrist axis -> back of the hand
HAND_T = 9.0           # back of hand -> palm face
ADAPTER_T = 3.0        # wrist adapter base plate, taken from the top of the hand's plateau

# Camera: Seeed XIAO ESP32S3 Sense (board + camera expansion board stack)
CAM_BOARD = (21.0, 17.8, 15.0)   # width, height, stack thickness incl. camera
CAM_TILT = 30.0                  # degrees, looks up towards raised palms

# Electronics bay behind the base (same print): half-size breadboard (82.5 x 54.5)
# with an ESP32-S3 dev board plugged in, DC jack, capacitor, servo wiring
BAY_X = (-120.0, -30.0)          # outer extent (overlaps the base cylinder)
BAY_IN_X = (-117.0, -52.0)       # cavity, starts outside the turntable radius
BAY_HALF_W = 51.0                # outer half width (cavity is 3 mm less); 46 -> 51 on 2026-10-13, the bay was too tight
BAY_TOP = 14.0
DC_JACK_D = 11.0                 # panel-mount 5.5x2.1 barrel jack hole (MEASURE yours)
LID_POST = 8.0                   # square screw posts in the bay's back corners, flush with the wall tops
LID_SCREW_PILOT = 2.5            # M3 self-tapping screw pilot in the posts
LID_SCREW_CLEAR = 3.4            # matching clearance hole in the lid
LID_SCREW_DEPTH = 12.0

SEG = 96

# ----------------------------------------------------------------------------
# Derived geometry
# ----------------------------------------------------------------------------
for s in SERVOS.values():
    s.cx = s.L / 2 - s.off                                         # case centre
    s.holes = [(s.cx + sx * s.pitch_L / 2, sy * s.pitch_W / 2)
               for sx in (-1, 1) for sy in ((-1, 1) if s.pitch_W else (0,))]
    s.fx = (s.cx - s.flange_L / 2, s.cx + s.flange_L / 2)          # flange x span


def joint_layout(s, wall_a, wall_b, gap, plate_t):
    """Y positions of a joint: cradle walls on the proximal link, clevis plates
    on the distal link. Centred so the clevis is symmetric about Y=0."""
    p2_out = -(s.below + gap + wall_b + WASHER + plate_t)
    p1_out = s.horn_z - s.horn_t + plate_t
    yf = -(p2_out + p1_out) / 2
    b1 = yf - s.below - gap
    return NS(s=s, yf=yf, A=(yf - wall_a, yf), B=(b1 - wall_b, b1),
              P1=(yf + s.horn_z - s.horn_t, yf + s.horn_z - s.horn_t + plate_t),
              P2=(b1 - wall_b - WASHER - plate_t, b1 - wall_b - WASHER),
              wall_a=wall_a, wall_b=wall_b, gap=gap)


J_SH = joint_layout(MG995, 7.0, 7.0, 3.0, 6.0)    # shoulder
J_EL = joint_layout(MG995, 7.0, 7.0, 3.0, 6.0)    # elbow
J_WR = joint_layout(MG90S, 5.0, 5.0, 3.0, 4.0)    # wrist (3 mm under the servo so an M3x16 pivot bolt clears it)
J_TW = joint_layout(MG90S, 5.0, 5.0, 3.0, 4.0)    # wrist twist: same joint as the wrist, axis along the hand

# Wrist twist (5th joint), in the wrist frame (wrist axis = Y through the origin, hand along +X):
# the twist bracket (rigid on the wrist horn) holds an MG90S whose shaft points along +X; the hand mount is a fork
# turning on it, and the hand screws onto the mount's base plate. The bracket prints base-down, so its servo cradle
# starts on the same bed plane as the wrist fins; that fixes the twist axis height.
TW_X = 80.0           # twist joint centre along the hand
TW_T = 4.0            # hand mount plate thickness
TW_HALF_W = 20.0      # hand mount plate half-width (across the hand)
TW_RANGE = 60.0       # +- twist travel to clear
TW_BED = -HAND_DROP - ADAPTER_T                   # bracket print bed (bottom of the wrist base plate)
# the twist servo lies on its side (case across the hand, +Y) in a cradle open on top (+Z), like the arm links, so it
# drops in from above and nothing prints over air; the cradle's bottom rib sits on the bed
TW_CRADLE_HW = MG90S.W / 2 + CRADLE_CLR + 5.0       # cradle half-height around the twist axis
TW_Z = TW_BED + TW_CRADLE_HW                         # twist axis height
TW_CRADLE_R = math.hypot(max(abs(MG90S.cx - MG90S.flange_L / 2 - 4), MG90S.cx + MG90S.flange_L / 2 + 4), TW_CRADLE_HW)
TW_DIN = round(TW_CRADLE_R + 1.5, 1)    # hand mount plate inner face below the twist axis: clears the cradle corners
HAND_SEAT = 3.0       # depth of the seat cut into the back of the hand for the mount plate
HAND_X0 = 56.0        # palm starts here, ahead of the bracket's spine, so it can twist freely
TW_SCREWS = [(TW_X + dx, dy) for dx in (-15.0, 15.0) for dy in (-14.0, 14.0)]   # M3x8 mount plate -> hand

DISC_BOT = MG995.horn_z - MG995.horn_t
DISC_TOP = DISC_BOT + DISC_T
COLLAR_TOP = DISC_BOT - DISC_GAP
Z_SH = DISC_TOP + TILT_H                          # shoulder axis height


def M_(rows):
    return Matrix([list(r) for r in rows] + [[0, 0, 0, 1]])


# servo-local -> part-frame matrices
# shaft up, body towards the camera: the MG995 cable leaves the case at the shaft end,
# so that end faces the bay and the cable goes straight into the tunnel
M_PAN = M_(((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0)))
M_SH = M_(((0, -1, 0, 0), (0, 0, 1, J_SH.yf), (-1, 0, 0, Z_SH)))         # body hangs down


def M_link_end(j, L):
    """Servo sitting at the far end of a link, shaft on +Y, body pointing back."""
    return M_(((-1, 0, 0, L), (0, 0, 1, j.yf), (0, 1, 0, 0)))


M_EL = M_link_end(J_EL, L1)
M_WR = M_link_end(J_WR, L2)
# twist joint frame (axis = Y, distal fork pointing +X) -> wrist frame: +X_j down, Y_j along the hand, Z_j to -Y
M_TWJ = M_(((0, 1, 0, TW_X), (0, 0, -1, 0), (-1, 0, 0, TW_Z)))
# twist servo: shaft along +X, case length across the hand (+Y), width up (+Z), so the cradle opens upwards
M_TWS = M_(((0, 0, 1, TW_X + J_TW.yf), (1, 0, 0, 0), (0, 1, 0, TW_Z)))

# ----------------------------------------------------------------------------
# Mesh helpers
# ----------------------------------------------------------------------------
scene = bpy.context.scene


def _obj(name, bm):
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    scene.collection.objects.link(ob)
    return ob


def box(x0, x1, y0, y1, z0, z1, name="box", M=None):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    m = Matrix.Translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)) @ \
        Matrix.Diagonal((abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), 1))
    bmesh.ops.transform(bm, matrix=(M or Matrix()) @ m, verts=bm.verts)
    return _obj(name, bm)


def cyl(r, a0, a1, axis="Z", c=(0, 0), seg=SEG, name="cyl", M=None):
    """Cylinder along axis from a0 to a1; c = centre in the other two axes
    (Z: (x, y), Y: (x, z), X: (y, z)). Optional M maps the result."""
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=seg,
                          radius1=r, radius2=r, depth=a1 - a0)
    mid = (a0 + a1) / 2
    if axis == "Z":
        m = Matrix.Translation((c[0], c[1], mid))
    elif axis == "Y":
        m = Matrix.Translation((c[0], mid, c[1])) @ Matrix.Rotation(math.radians(-90), 4, "X")
    else:
        m = Matrix.Translation((mid, c[0], c[1])) @ Matrix.Rotation(math.radians(90), 4, "Y")
    bmesh.ops.transform(bm, matrix=(M or Matrix()) @ m, verts=bm.verts)
    return _obj(name, bm)


def cone_z(r0, r1, z0, z1, c=(0, 0), seg=SEG, name="cone"):
    """Truncated cone along Z: radius r0 at z0, r1 at z1."""
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=seg, radius1=r0, radius2=r1, depth=z1 - z0)
    bmesh.ops.transform(bm, matrix=Matrix.Translation((c[0], c[1], (z0 + z1) / 2)), verts=bm.verts)
    return _obj(name, bm)


def hexp(r, a0, a1, axis="Z", c=(0, 0), M=None):
    return cyl(r, a0, a1, axis, c, seg=6, name="hex", M=M)


def prism_xz(pts, y0, y1, name="prism"):
    """Polygon given in (x, z), extruded along Y."""
    bm = bmesh.new()
    lo = [bm.verts.new((x, y0, z)) for x, z in pts]
    hi = [bm.verts.new((x, y1, z)) for x, z in pts]
    bm.faces.new(lo)
    bm.faces.new(hi[::-1])
    for i in range(len(pts)):
        j = (i + 1) % len(pts)
        bm.faces.new((lo[i], lo[j], hi[j], hi[i]))
    return _obj(name, bm)


def hull2d(pts):
    pts = sorted(set(pts))
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def circle_pts(c, r, n=48):
    return [(c[0] + r * math.cos(2 * math.pi * i / n), c[1] + r * math.sin(2 * math.pi * i / n))
            for i in range(n)]


def hull(points, name="hull"):
    bm = bmesh.new()
    for p in points:
        bm.verts.new(p)
    res = bmesh.ops.convex_hull(bm, input=bm.verts, use_existing_faces=False)
    junk = list({g for g in res["geom_interior"] + res["geom_unused"] if isinstance(g, bmesh.types.BMVert)})
    bmesh.ops.delete(bm, geom=junk, context="VERTS")
    return _obj(name, bm)


def sphere_pts(c, r, zscale=1.0, u=40, v=20):
    pts = []
    for i in range(1, v):
        th = math.pi * i / v
        for j in range(u):
            ph = 2 * math.pi * j / u
            pts.append((c[0] + r * math.sin(th) * math.cos(ph),
                        c[1] + r * math.sin(th) * math.sin(ph),
                        c[2] + zscale * r * math.cos(th)))
    pts += [(c[0], c[1], c[2] + zscale * r), (c[0], c[1], c[2] - zscale * r)]
    return pts


def boolean(target, others, op):
    if not isinstance(others, (list, tuple)):
        others = [others]
    for o in others:
        mod = target.modifiers.new("b", "BOOLEAN")
        mod.operation = op
        mod.solver = "EXACT"
        mod.object = o
        dg = bpy.context.evaluated_depsgraph_get()
        me = bpy.data.meshes.new_from_object(target.evaluated_get(dg))
        target.modifiers.clear()
        old = target.data
        target.data = me
        bpy.data.meshes.remove(old)
        bpy.data.objects.remove(o, do_unlink=True)
    return target


def union(objs, name):
    base = objs[0]
    boolean(base, objs[1:], "UNION")
    base.name = name
    base.data.name = name
    return base


def cut(target, cutters):
    return boolean(target, cutters, "DIFFERENCE")


def cleanup(ob):
    """Weld slivers left by coplanar booleans so slicers get a watertight mesh."""
    bm = bmesh.new()
    bm.from_mesh(ob.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.01)
    bmesh.ops.dissolve_degenerate(bm, edges=bm.edges, dist=0.01)
    bmesh.ops.holes_fill(bm, edges=[e for e in bm.edges if e.is_boundary], sides=0)
    bmesh.ops.triangulate(bm, faces=bm.faces)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(ob.data)
    bm.free()
    return ob


# ----------------------------------------------------------------------------
# Reusable joint features
# ----------------------------------------------------------------------------
def cradle(j, M, x_span, half_w, rib_x=None, rib_sides=(-1, 1), open_top=False):
    """Walls A (flange) + B (idler) and side ribs, in servo-local coords mapped by M.
    x_span: local x extent of the walls; half_w: local y half-width of the walls.
    rib_sides: which side ribs to build (local y sign); the arm links skip the +y rib, which prints as a roof bridge.
    open_top: run the servo window out through the +y edge of the flange wall, so the servo drops in from that
    side (no bridge over the window either); the flange ears still sit on the wall's two ends, where the screws are."""
    s = j.s
    solids = [box(*x_span, -half_w, half_w, -j.wall_a, 0, M=M),
              box(*x_span, -half_w, half_w, j.B[0] - j.yf, j.B[1] - j.yf, M=M)]
    rin = s.W / 2 + CRADLE_CLR + 0.01
    rx = rib_x or x_span
    for sg in rib_sides:
        solids.append(box(*rx, sg * rin, sg * half_w, j.B[1] - j.yf - 0.1, -j.wall_a + 0.1, M=M))
    win_top = half_w + 1 if open_top else s.W / 2 + CRADLE_CLR
    ext = 0 if open_top else WINDOW_EXTRA
    win_x0, win_x1 = -s.off - CRADLE_CLR - ext, s.L - s.off + CRADLE_CLR + ext
    cutters = [box(win_x0, win_x1, -s.W / 2 - CRADLE_CLR, win_top, -j.wall_a - 1, 1, M=M)]  # servo window
    if s.horn == "round" and not open_top:
        # cable dent (5 wide x 3 mm) in the window's far edge, the end away from the shaft (at the bottom on the
        # shoulder); stays ~1.2 mm clear of the flange screw pilots
        cutters.append(box(win_x1 - 0.1, win_x1 + 3.0, -2.5, 2.5, -j.wall_a - 1, 1, M=M))
    for hx, hy in s.holes:
        cutters.append(cyl(s.pilot / 2, -j.wall_a - 1, 1, c=(hx, hy), seg=24, M=M))
    zb = j.B[1] - j.yf
    cutters.append(cyl(1.6, zb - j.wall_b - 1, zb + 1, c=(0, 0), seg=24, M=M))  # idler bolt
    cutters.append(hexp(M3_NUT_R, zb - 3.0, zb + 0.5, M=M))                      # nut trap
    return solids, cutters


def clevis_cutters(j, teardrop=False):
    """Horn recess + screw access in plate P1, idler clearance hole in plate P2
    (link frame: joint axis = Y axis through the origin). teardrop: the plates print standing up (link z up), so
    the round recess gets a 45-degree peak on top instead of an arch that would droop."""
    s = j.s
    face = j.yf + s.horn_z + s.seat                  # seat: recess this much deeper (measured horn)
    cs = []
    if s.horn == "round":
        r = s.horn_d / 2 + (HORN_CLR_SIDE if teardrop else HORN_CLR)
        cs.append(cyl(r, j.P1[0] - 1, face, "Y"))
        if teardrop:
            k, top = r * math.sqrt(0.5), min(r * math.sqrt(2), LINK_H - 1.0)
            w = max(0.9, k - (top - k))                    # flat-topped where the peak is clipped below the plate edge
            cs.append(hull([(x, y, z) for y in (j.P1[0] - 1, face) for x, z in ((-k, k), (k, k), (-w, top), (w, top))], "tear"))
    else:
        clr = 0.6 if teardrop else 0.3                  # sideways print: same lesson as the round pockets
        cs.append(cyl(s.hub_d / 2 + clr, j.P1[0] - 1, face, "Y"))
        r = s.horn_w / 2 + clr
        half = s.horn_len / 2 + clr - r
        cs.append(box(-half, half, j.P1[0] - 1, face, -r, r))
        cs += [cyl(r, j.P1[0] - 1, face, "Y", (sx * half, 0), seg=32) for sx in (-1, 1)]
    cs.append(cyl(s.access_d / 2, j.P1[0] - 1, j.P1[1] + 1, "Y", seg=32))
    cs.append(cyl(M3_CLEAR / 2, j.P2[0] - 1, j.P2[1] + 1, "Y", seg=32))
    return cs


def clevis_plates(j, outline):
    return [prism_xz(outline, *j.P1), prism_xz(outline, *j.P2)]


def plate_outline(x1, h=LINK_H, back=LINK_H, ch=6.0):
    return [(-back + ch, -h), (x1, -h), (x1, h), (-back + ch, h), (-back, h - ch), (-back, -h + ch)]


def link_body(j_prox, j_dist, L, name):
    """Clevis at the proximal joint + ladder beam + cradle for the next servo."""
    s = j_dist.s
    M = M_link_end(j_dist, L)
    parts = clevis_plates(j_prox, plate_outline(48))
    parts.append(box(32, 48, j_prox.P2[0], j_prox.P1[1], -LINK_H, LINK_H))       # cross beam
    back = L - (s.fx[1] + 4)
    parts.append(box(40, back + 1, j_dist.B[0], j_dist.A[1], -LINK_H, LINK_H))   # beam
    # only the bottom rib (on the print bed side), and the window open at the top: nothing bridges over the
    # servo pocket, and the servo drops in from above with its cable
    w, c = cradle(j_dist, M, (s.fx[0] - 4, s.fx[1] + 4), LINK_H,
                  rib_x=(s.fx[0] - 4, s.L - s.off + CLR), rib_sides=(-1,), open_top=True)
    body = union(parts + w, name)
    # ladder window between the side rails (also where the servo cable escapes)
    win_end = L - (s.L - s.off) - CLR + 0.01
    c.append(box(52, win_end, j_dist.B[1], j_dist.A[0], -LINK_H - 1, LINK_H + 1))
    # cable channel in the floor under the servo: the cable leaves the servo's shaft-end face, drops into this
    # groove, runs under the servo and comes out in the ladder window (open on top in print, 2 mm floor left)
    yc = j_dist.yf - 0.68 * s.below                      # where the cable leaves the end face (link y)
    z_floor = -(s.W / 2 + CRADLE_CLR)
    c.append(box(win_end - 3, L + s.off + 4, yc - 3, yc + 3, z_floor - 2.5, z_floor + 0.5))
    # zip-tie slot pairs through the A-side rail near each end of the ladder window: the tie goes in one slot,
    # around the cables lying against the rail's inside face, and out the other (3.5 x 1.8 mm, 2.5 mm ties)
    for xs in (62, win_end - 10):
        for zs in (-5, 5):
            c.append(box(xs - 0.9, xs + 0.9, j_dist.A[0] - 1, j_dist.A[1] + 1, zs - 1.75, zs + 1.75))
    return cut(body, c + clevis_cutters(j_prox, teardrop=True))


# ----------------------------------------------------------------------------
# Parts
# ----------------------------------------------------------------------------
def lid_screw_xy():
    """Centres of the two lid screws, in the back corners of the bay."""
    hw = BAY_HALF_W - 3.0
    return [(BAY_IN_X[0] + LID_POST / 2, sg * (hw - LID_POST / 2)) for sg in (-1, 1)]


def foot_screw_xy():
    """4 x 4 mm wood screws through the foot into the board. Front pair at 45 deg on r59. The back pair used to sit
    there too, but the bay block covered them (2026-10-15), so they moved forward and out: 10 mm in front of the bay's
    front corner and 6 mm out from its side wall, clear for the screw head and the driver."""
    a = math.radians(45)
    return [(59 * math.cos(a), sg * 59 * math.sin(a)) for sg in (-1, 1)] + \
           [(-20.0, sg * (BAY_HALF_W + 6.0)) for sg in (-1, 1)]


def build_base():
    wall = 3.0
    body = union([cyl(BASE_R, BASE_BOTTOM, COLLAR_TOP),
                  cyl(FOOT_R, BASE_BOTTOM, BASE_BOTTOM + FOOT_T),
                  box(*BAY_X, -BAY_HALF_W, BAY_HALF_W, BASE_BOTTOM, BAY_TOP)],    # electronics bay
                 "base")
    cutters = [cyl(COLLAR_IN_R, 0, COLLAR_TOP + 1)]
    s = MG995
    floor = BASE_BOTTOM + 3.5
    cutters.append(box(-s.off - CLR - 1.5, s.L - s.off + CLR + 1.5, -s.W / 2 - CLR, s.W / 2 + CLR,
                       floor, 0.1, M=M_PAN))                                     # pan servo pocket
    # servo cable -> bay: roomy tunnel from the shaft end of the pocket, tall enough to catch the cable
    # wherever it leaves the case, wide enough for the plug
    cutters.append(box(BAY_IN_X[1] - 1, -(s.off + CLR + 1.5) + 1, -6, 6, floor, -14))
    for hx, hy in s.holes:
        cutters.append(cyl(s.pilot / 2, -12, 0.1, c=(hx, hy), seg=24, M=M_PAN))
    for c in foot_screw_xy():
        cutters.append(cyl(2.2, BASE_BOTTOM - 1, BASE_BOTTOM + FOOT_T + 1, c=c, seg=24))
    # electronics bay: open top (lid), cable notches in both side walls, DC jack in the back
    hw = BAY_HALF_W - wall
    cutters.append(box(*BAY_IN_X, -hw, hw, BASE_BOTTOM + wall, BAY_TOP + 1))
    for sg in (-1, 1):
        cutters.append(box(-100, -70, sg * (hw - 1), sg * (BAY_HALF_W + 1), BAY_TOP - 12, BAY_TOP + 1))
    cutters.append(cyl(DC_JACK_D / 2, BAY_X[0] - 1, BAY_IN_X[0] + 1, "X", (25, -8), seg=48))
    # camera seat: the printed camera box sits on a 30 deg sloped seat (lip on three sides, open at the USB-C end) and
    # is clamped to it, together with its lid, by 2 x M3 x 25 into nuts pressed into pockets in the seat. The seat faces
    # up, so it prints without support; the box and lid print flat.
    Mc = cam_box_frame()
    cx, cy, x0, x1, oy, top = cb_dims()
    lip, rim = 1.0, 2.3
    corners = [Mc @ Vector((x, y, lip)) for x in (x0 - rim, x1) for y in (-oy - rim, oy + rim)]
    pts = corners + [Vector((c.x, c.y, BASE_BOTTOM)) for c in corners]
    pts += [Vector((44.0, c.y, z)) for c in corners for z in (BASE_BOTTOM, min(c.z, COLLAR_TOP - 2))]
    body = union([body, hull([tuple(v) for v in pts], "cam_seat")], "base")
    cutters.append(box(x0 - 0.3, x1 + 6, -oy - 0.3, oy + 0.3, 0, top + 5, M=Mc))           # the box sits in here
    zt = top + 2.4 - CB_SCREW                                                                  # screw tip (box frame)
    for y in (-CB_SCREW_Y, CB_SCREW_Y):
        # 2026-10-14: the first print's r3.25 pockets (0.07 mm per side on the flats) and 3.4 mm holes were too tight.
        # Both are tilted 30 deg, so the up-facing side of each sags and shrinks. The nut drops in loose
        # (the hex stops it turning, the box covers it) and the screw has 0.4 mm per side.
        cutters += [hexp(CAM_NUT_R, -2.8, 0.1, c=(0, y), M=Mc),
                    cyl(CAM_SEAT_HOLE / 2, zt - 2, 0.1, c=(0, y), seg=24, M=Mc)]
    # power wires (5V/GND, soldered at the XIAO's USB-C end): box floor slot -> seat slot -> riser -> hidden channel -> bay
    cutters.append(box(6.5, 10.5, -8.0, 8.0, -8.0, 0.1, M=Mc))
    r = Mc @ Vector((8.5, 0.0, -6.0))
    z0, z1 = BASE_BOTTOM + 3, BASE_BOTTOM + 9
    cutters.append(box(r.x - 2.5, r.x + 2.5, r.y - 3.0, r.y + 3.0, z0, r.z + 1.0))           # riser
    cutters.append(box(r.x - 2.5, 70.0, r.y - 3.0, 22, z0, z1))
    cutters.append(box(BAY_IN_X[1] - 1, 66, 14, 22, z0, z1))
    base = cut(body, cutters)
    # lid screw posts: solid columns in the back corners from the bay floor up to the wall tops
    posts = [box(x - LID_POST / 2, x + LID_POST / 2, y - LID_POST / 2, y + LID_POST / 2,
                 BASE_BOTTOM + wall - 0.1, BAY_TOP) for x, y in lid_screw_xy()]
    base = union([base] + posts, "base")
    return cut(base, [cyl(LID_SCREW_PILOT / 2, BAY_TOP - LID_SCREW_DEPTH, BAY_TOP + 1, c=c, seg=24)
                      for c in lid_screw_xy()])


def build_bay_lid():
    """Drop-in lid for the electronics bay (prints upside down: flat top on the bed)."""
    hw = BAY_HALF_W - 3.0
    top = box(BAY_IN_X[0] - 2, BAY_IN_X[1] + 2, -hw - 2, hw + 2, BAY_TOP, BAY_TOP + 2.5, "bay_lid")
    plug = box(BAY_IN_X[0] + CLR, BAY_IN_X[1] - CLR, -hw + CLR, hw - CLR, BAY_TOP - 4, BAY_TOP + 0.1)
    # crush ribs: 3 mm bumps standing 0.15 mm proud of the bay walls give a snug push fit;
    # they stop 1 mm short of the plug's lower edge as a lead-in, and avoid the side cable notches
    rib, z0, z1 = CLR + 0.15, BAY_TOP - 3, BAY_TOP
    xa, xb, ya = BAY_IN_X[0] + CLR, BAY_IN_X[1] - CLR, hw - CLR
    ribs = [box(x - 1.5, x + 1.5, sg * ya - (rib if sg < 0 else 0), sg * ya + (rib if sg > 0 else 0), z0, z1)
            for x in (-104, -60) for sg in (-1, 1)]
    ribs += [box(xa - rib, xa, y - 1.5, y + 1.5, z0, z1) for y in (-20, 20)]
    ribs += [box(xb, xb + rib, y - 1.5, y + 1.5, z0, z1) for y in (-20, 20)]
    lid = union([top, plug] + ribs, "bay_lid")
    cutters = [box(BAY_IN_X[0] + CLR + 2, BAY_IN_X[1] - CLR - 2, -hw + CLR + 2, hw - CLR - 2,
                   BAY_TOP - 5, BAY_TOP - 0.5),                                  # hollow the plug
               cyl(8, BAY_TOP - 1, BAY_TOP + 3, c=(BAY_IN_X[0] - 2, 0), seg=48)]  # finger notch
    # notch the plug around the screw posts, and clearance holes for the screws
    for x, y in lid_screw_xy():
        h = LID_POST / 2 + CLR
        cutters.append(box(x - h, x + h, y - h, y + h, BAY_TOP - 5, BAY_TOP))
        cutters.append(cyl(LID_SCREW_CLEAR / 2, BAY_TOP - 5, BAY_TOP + 3, c=(x, y), seg=24))
    for i in range(4):                                                           # vents
        x = BAY_IN_X[0] + 12 + i * 10
        cutters.append(box(x, x + 4, -25, 25, BAY_TOP - 1, BAY_TOP + 3))
    # arm cable slot just behind the turntable: the servo bundle loops from the turntable into the bay
    sx0, sx1, sw = BAY_IN_X[1] - 14, BAY_IN_X[1] - 6, 14
    cutters.append(box(sx0, sx1, -sw + 4, sw - 4, BAY_TOP - 1, BAY_TOP + 3))
    cutters += [cyl(4, BAY_TOP - 1, BAY_TOP + 3, c=((sx0 + sx1) / 2, sg * (sw - 4)), seg=32) for sg in (-1, 1)]
    return cut(lid, cutters)


def build_pan_pointer():
    """Stage-1 stand-in for the shoulder: turntable with a pointer, shows where pan aims."""
    s = MG995
    body = union([cyl(BASE_R, DISC_BOT, DISC_TOP),
                  box(-10, 60, -6, 6, DISC_TOP - 0.1, DISC_TOP + 10),
                  hull([(x, y, z) for x, y in ((60, -14), (60, 14), (85, 0))
                        for z in (DISC_TOP - 0.1, DISC_TOP + 10)], "tip")], "pan_pointer")
    r = s.horn_d / 2 + HORN_CLR
    return cut(body, [cyl(r, DISC_BOT - 1, s.horn_z + s.seat),
                      cone_z(r + HORN_CHAMFER + 1, r, DISC_BOT - 1, DISC_BOT + HORN_CHAMFER),
                      cyl(s.access_d / 2, DISC_BOT - 1, DISC_TOP + 11, seg=32)])


def build_pan_pointer_support():
    """Print-only break-away support under the part of the arrow that overhangs the disc rim.
    Thin rib grid, one layer (0.25 mm) below the arrow and 0.6 mm clear of the rim; snap off after printing."""
    top = DISC_TOP - 0.1 - 0.25
    rib = 0.8
    ribs = [box(50, 86, y - rib / 2, y + rib / 2, DISC_BOT, top) for y in (-11, -5.5, 0, 5.5, 11)]
    ribs += [box(x - rib / 2, x + rib / 2, -15, 15, DISC_BOT, top) for x in (54, 63, 72, 80)]
    grid = union(ribs, "pan_pointer_support")
    # keep the ribs 0.5 mm inside the arrow outline
    outline = union([box(50, 59.6, -5.5, 5.5, DISC_BOT - 1, top + 1),
                     hull([(x, y, z) for x, y in ((59.5, -13.2), (59.5, 13.2), (83.6, 0))
                           for z in (DISC_BOT - 1, top + 1)], "tip_in")], "outline")
    boolean(grid, outline, "INTERSECT")
    return cut(grid, [cyl(BASE_R + 0.6, DISC_BOT - 1, top + 1)])


def build_shoulder():
    """Turntable on the pan horn + cradle holding the shoulder servo sideways."""
    s = MG995
    top_x = Z_SH - DISC_TOP + 0.1                     # servo-local x of the disc top
    solids, cutters = cradle(J_SH, M_SH, (s.fx[0] - 5, top_x), s.W / 2 + CLR + 6,
                             rib_x=(-8, top_x))
    sh = union([cyl(BASE_R, DISC_BOT, DISC_TOP)] + solids, "shoulder")
    r = s.horn_d / 2 + HORN_CLR
    cutters += [cyl(r, DISC_BOT - 1, s.horn_z + s.seat),                         # pan horn recess
                cone_z(r + HORN_CHAMFER + 1, r, DISC_BOT - 1, DISC_BOT + HORN_CHAMFER),  # flared mouth (bed side)
                cyl(s.access_d / 2, DISC_BOT - 1, DISC_TOP + 1, seg=32)]
    hw = s.W / 2 + CLR + 7
    cutters.append(box(-hw, hw, J_SH.B[1] + 3, J_SH.A[0] - 3, DISC_TOP + 3, DISC_TOP + 13))  # cable exit
    # zip-tie mounts for the arm cables (a block with a 3.5 x 2 mm tunnel for a 2.5 mm tie):
    # A on the back rib near the top, where the shoulder servo's cable comes out of its shaft end;
    # B on the turntable's back edge, the anchor before the slack loop into the bay lid
    rib_x = -(s.W / 2 + CLR + 6)
    yc = (J_SH.B[1] + J_SH.A[0]) / 2
    # mount A has a 45-degree underside so it prints without support (the shoulder prints disc-down)
    mount_a = hull([(x, y, z) for y in (yc - 4, yc + 4) for x, z in
                    ((rib_x + 0.1, Z_SH - 7), (rib_x + 0.1, Z_SH + 6), (rib_x - 5, Z_SH - 2), (rib_x - 5, Z_SH + 6))], "mount_a")
    sh = union([sh, mount_a,
                box(-48, -40, -5, 5, DISC_TOP - 0.1, DISC_TOP + 5)], "shoulder")
    cutters += [box(rib_x - 3, rib_x - 1, yc - 5, yc + 5, Z_SH + 0.25, Z_SH + 3.75),
                box(-45.75, -42.25, -6, 6, DISC_TOP + 1, DISC_TOP + 3)]
    return cut(sh, cutters)


def build_pivot_spacer():
    """C-shaped spacer that fills the pivot gap after assembly; slides in around the bolt. Print 3."""
    t = WASHER - SPACER_CLR
    ring = cyl(5.0, 0, t, seg=48, name="pivot_spacer")
    return cut(ring, [cyl(M3_CLEAR / 2, -1, t + 1, seg=32), box(0, 6, -M3_CLEAR / 2, M3_CLEAR / 2, -1, t + 1)])


def build_hand():
    """Hand, palm down, thumb on the -Y side. Link frame: wrist axis on Y through the origin."""
    seat = TW_Z - TW_DIN - TW_T       # seat floor = underside of the hand mount plate
    zb = seat + HAND_SEAT             # back of hand
    zp = zb - HAND_T                  # palm face (print bed)
    x0 = HAND_X0                      # palm starts here, ahead of the twist bracket
    zs = 0.72
    pieces = []
    pts = []
    for c in ((x0 + 12, 28), (x0 + 12, -28), (x0 + 76, 28), (x0 + 76, -28)):
        pts += sphere_pts((c[0], c[1], zp), 12, 0.75)
    pieces.append(hull(pts, "palm"))
    fingers = [((x0 + 78, 25), (x0 + 143, 31), 9.0, 8.0),     # index
               ((x0 + 80, 8), (x0 + 152, 10), 9.0, 8.0),      # middle
               ((x0 + 78, -9), (x0 + 147, -12), 9.0, 8.0),    # ring
               ((x0 + 74, -26), (x0 + 124, -35), 8.0, 7.0),   # pinky
               ((x0 + 30, 30), (x0 + 72, 62), 10.0, 8.5)]     # thumb
    for (b, t, rb, rt) in fingers:                    # mirrored: thumb on the -Y side (user's choice, 2026-10-07)
        b, t = (b[0], -b[1]), (t[0], -t[1])
        pieces.append(hull(sphere_pts((*b, zp), rb, zs) + sphere_pts((*t, zp), rt, zs), "finger"))
    hand = union(pieces, "hand")
    j = J_TW
    cutters = [box(-50, 300, -100, 100, zp - 30, zp),
               box(TW_X + j.P2[0] - CLR, TW_X + j.P1[1] + CLR, -TW_HALF_W - CLR, TW_HALF_W + CLR, seat, zb + 20)]  # seat
    cutters += [cyl(1.25, seat - 5.0, seat + 1, c=c, seg=24) for c in TW_SCREWS]   # M3 pilots, 1 mm above the palm
    return cut(hand, cutters)


def build_twist_bracket():
    """Wrist side of the twist joint: the fins that clamp the wrist MG90S horn, a base plate, a spine up behind the hand
    mount, and a cradle holding the twist MG90S (shaft along +X, lying on its side, open on top so it drops in).
    Prints base-down: the fins, base, spine and cradle all start on the same bed plane."""
    j, jt, s = J_WR, J_TW, MG90S
    zb = -HAND_DROP
    x0 = 24.0
    top = TW_Z + 21.0                                           # top of the spine and the beam over the idler plate
    parts = [box(x0 - 2, x0 + 28, j.P2[0] + 1, j.P1[1] - 1, TW_BED, zb, "twist_bracket")]   # base under the wrist
    fin = hull2d(circle_pts((0, 0), 18.0) + [(-12, TW_BED), (x0, TW_BED), (x0 + 26, TW_BED), (x0 + 26, zb + 6)])
    parts += clevis_plates(j, fin)
    spine_x1 = TW_X + jt.P2[0] - 1.5                             # 1.5 mm clear of the hand mount's idler plate
    parts.append(box(44, spine_x1, -8, 8, TW_BED, top))          # spine up from the base
    parts.append(box(44, TW_X + jt.B[1] - 2.5, -8, 8, TW_Z + 13, top))   # beam over the idler plate's hub ...
    parts.append(box(TW_X + jt.P2[1] + 1.5, TW_X + jt.B[1] - 2.5, -8, 8, TW_Z + 6.5, top))   # ... into wall B, over the spacer
    walls, cutters = cradle(jt, M_TWS, (s.fx[0] - 4, s.fx[1] + 4), TW_CRADLE_HW, rib_sides=(-1,), open_top=True)
    br = union(parts + walls, "twist_bracket")
    cutters += clevis_cutters(j, teardrop=True)
    # tunnel through the spine along the twist axis: the idler bolt and screwdriver go in through it, and the bolt head
    # turns inside it; peaked on top because it prints sideways
    r = 3.5
    cutters.append(cyl(r, 40, spine_x1 + 1, "X", c=(0, TW_Z), seg=40))
    cutters.append(hull([(x, y, TW_Z + z) for x in (40, spine_x1 + 1) for y, z in ((-r * 0.707, r * 0.707), (r * 0.707, r * 0.707),
                                                                                   (-0.5, r * 1.414), (0.5, r * 1.414))], "tear"))
    return cut(br, cutters)


def build_hand_mount():
    """Distal side of the twist joint: a fork (horn plate + idler plate) turning on the twist MG90S, joined by a flat
    plate under the joint that the hand screws onto (4x M3x8). Built in the twist joint frame, then placed.
    Prints plate-down."""
    j = J_TW
    outline = hull2d(circle_pts((0, 0), 12.0) + [(TW_DIN, -TW_HALF_W), (TW_DIN, TW_HALF_W),
                                                 (TW_DIN + TW_T, -TW_HALF_W), (TW_DIN + TW_T, TW_HALF_W)])
    plate = box(TW_DIN, TW_DIN + TW_T, j.P2[0], j.P1[1], -TW_HALF_W, TW_HALF_W, "hand_mount")
    hm = union([plate] + clevis_plates(j, outline), "hand_mount")
    cutters = clevis_cutters(j, teardrop=True)
    cutters += [cyl(M3_CLEAR / 2, TW_DIN - 1, TW_DIN + TW_T + 1, "X", c=(x - TW_X, -y), seg=24) for x, y in TW_SCREWS]
    hm = cut(hm, cutters)
    hm.data.transform(M_TWJ)
    return hm


# ---- XIAO ESP32S3 Sense as measured (user's photos with a mm ruler, 2026-10-13; +-0.5-1 mm).
# Board frame: x along the 21 mm side (USB-C end at +x), y along the 17.8 mm side, z up from the XIAO's flat underside.
# The camera board stacks on the XIAO's shield side, so the shield, USB-C and U.FL socket sit between the two boards.
XS_W, XS_H = 21.0, 17.8
XS_PCB_TOP = 6.4                                    # top face of the camera board
XS_USB = (7.0, 11.5, -4.5, 4.5, 1.0, 4.4)          # USB-C socket (sticks out ~1 mm past the board end)
XS_FPC = (-10.5, -4.6, -8.9, 8.9, XS_PCB_TOP, 10.0)  # white ribbon connector across the -x end
XS_MOD = (4.8, 13.6, -4.85, 4.85, 11.2, 13.8)      # camera module body (hangs ~3 mm past the USB-C end)
XS_LENS = (9.7, 0.0, 3.4, 17.9)                    # lens barrel: centre x, y, radius, top
# (XS_MOD / XS_LENS are the stock OV3660 module; the box size and the base's seat position were built around them, so
#  they stay. The camera in use now is the wide-angle module below.)
# Wide-angle OV2640 module (Treedix, ~120 deg), fitted 2026-10-22, measured from the user's photos with a ruler: 9.5 mm
# square body 2.8 mm thick, resting ~11.5 mm above the XIAO's underside; lens centred on it ~7 mm towards the USB-C end
# (x) and on the centre line (y); threaded barrel ~8 mm across, then a 10.8 mm front ring from z ~21.9 to ~26.1.
XW_MOD = (2.25, 11.75, -4.75, 4.75, 11.5, 14.3)
XW_LENS = (7.0, 0.0, 4.0, 21.9, 5.4, 26.1)          # centre x, y, barrel radius, front ring start, ring radius, top
XW_SLACK_X = 1.2                                     # the lid lets the module sit +-1.2 mm along x (photo estimate)

# ---- camera box test v2 (screw-down lid), built in the board frame with the box floor under z = 0
CB_CLR = 0.35          # board to wall, each side
CB_WALL = 2.4
CB_FLOOR = 2.4
CB_H = XS_MOD[5] + 0.6 # cavity floor -> lid underside: 0.6 mm over the camera module (shims take up any slack)
CB_SCREW_Y = XS_H / 2 + CB_CLR + 4.2
CB_X1 = XS_MOD[1] + 0.4 + 1.2   # +x end, past the camera module and the lid's collar around it
CB_SHIMS = (0.6, 1.2)


def cb_dims():
    cx, cy = XS_W / 2 + CB_CLR, XS_H / 2 + CB_CLR
    return cx, cy, -cx - CB_WALL, CB_X1, CB_SCREW_Y + 4.0, CB_FLOOR + CB_H


CB_SCREW = 25        # M3 x 25: through the lid and the box into nuts in the camera wedge (v5) / the seat (v3)
# v5 (2026-10-22, wide lens): the box sits on a 25 deg wedge (camera_wedge) on the base's 30 deg seat, so it looks up
# 55 deg. The wedge screws to the seat's two nuts at x = 0, so the box and lid screws move out to x = +-8 (4 screws,
# 4 nuts pressed into the wedge's top).
CB_SCREW_XY = [(sx * 8.0, sy * CB_SCREW_Y) for sx in (-1, 1) for sy in (-1, 1)]
WEDGE_TILT = 30.0     # extra camera tilt, degrees
WEDGE_T0 = 6.5        # wedge thickness at its thin (upper, hinge) end
WEDGE_SHELF = 3.0     # wedge-to-seat screws (M3 x 8 or x 10): their heads sit on a shelf this high above the seat


def wedge_tf():
    """Box frame -> seat frame (the seat's box frame from cam_box_frame()): the box is tilted WEDGE_TILT about its x
    axis so the lens looks further UP. The seat's +y runs up the slope, so the wedge is thin (WEDGE_T0) at its upper
    edge (y = +oy, the hinge) and thick at its lower edge. (A first version hinged the other way tilted it down to
    5 deg; the seat check's 'looking N deg above horizontal' line catches that.)"""
    oy = cb_dims()[4]
    return (Matrix.Translation((0, oy, WEDGE_T0)) @ Matrix.Rotation(math.radians(-WEDGE_TILT), 4, "X")
            @ Matrix.Translation((0, -oy, 0)))


def cam_box_frame_fitted():
    """Box frame -> world for the box as mounted (on the wedge)."""
    return cam_box_frame() @ wedge_tf()


def cb_cutters(M=None):
    """Cavity, open USB-C end, antenna notch, through screw holes and the power-wire floor slot of the camera box,
    in the box frame (mapped by M)."""
    cx, cy, x0, x1, oy, top = cb_dims()
    cut_ = [box(-cx, cx, -cy, cy, CB_FLOOR, top + 1, M=M)]                                # cavity
    cut_ += [cyl(M3_CLEAR / 2, -1, top + 1, c=c, seg=24, M=M) for c in CB_SCREW_XY]
    # +x end open: USB-C plug body (12.5 x 7 mm, centred on the socket) and the overhanging camera module
    cut_.append(box(cx - 0.5, x1 + 1, -6.75, 6.75, CB_FLOOR + (XS_USB[4] + XS_USB[5]) / 2 - 4.0, top + 1, M=M))
    # -x end: notch for the antenna wire, which leaves from between the two boards
    cut_.append(box(x0 - 1, -cx + 0.5, -7.0, 7.0, CB_FLOOR + 0.3, CB_FLOOR + XS_PCB_TOP - 0.4, M=M))
    # floor slot under the USB-C end: the 5V/GND wires go down through it into the base
    cut_.append(box(6.5, 10.5, -8.0, 8.0, -1, CB_FLOOR + 0.1, M=M))
    return cut_


def build_camera_box():
    """Box the XIAO + camera stack sits in, flat underside down. Prints bottom-down. It screws onto the base's camera
    seat with the lid: 2 x M3 x 25 through the lid and the box into nuts in the seat."""
    cx, cy, x0, x1, oy, top = cb_dims()
    return cut(box(x0, x1, -oy, oy, 0, top, "camera_box"), cb_cutters())


def build_camera_lid():
    """Lid for the wide-angle module (XW_*). A collar around the module body locates it (on the -x side, where the
    ribbon comes in, the wall starts 1.2 mm above the module's underside so the ribbon passes under it), the lid's underside holds it down, and the lens passes up through a slot (its 10.8 mm front
    ring has to get through when the lid goes on). The screw heads sit in 1 mm counterbores so the M3 screws reach
    1 mm further. Same pads over the ribbon connector and the board corners as the OV3660 lid. Exported flipped."""
    cx, cy, x0, x1, oy, top = cb_dims()
    t = 2.4
    z0 = top
    m, (lx, ly, br, rz, rr, lt) = XW_MOD, XW_LENS
    clr_y, wall = 0.45, 1.2
    ix0, ix1 = m[0] - XW_SLACK_X - clr_y, m[1] + XW_SLACK_X + clr_y          # pocket inside the collar
    iy = m[3] + clr_y
    parts_ = [box(x0, x1, -oy, oy, z0, z0 + t, "camera_lid"),
              box(ix0, min(ix1 + wall, x1), -iy - wall, iy + wall, CB_FLOOR + 12.0, z0 + 0.1),                 # collar
              box(ix0 - wall, ix0 + 0.1, -iy - wall, iy + wall, CB_FLOOR + m[4] + 1.2, z0 + 0.1),            # -x stop, ribbon passes under
              box(-XS_W / 2, XS_FPC[1] - 0.5, -8.0, 8.0, CB_FLOOR + XS_FPC[5] + 0.6, z0 + 0.1)]                    # over the connector
    for sy in (-1, 1):                                                                    # +x corners, beside the collar
        parts_.append(box(5.0, XS_W / 2, min(sy * 6.8, sy * 8.6), max(sy * 6.8, sy * 8.6), CB_FLOOR + XS_PCB_TOP + 0.8, z0 + 0.1))
    lid = union(parts_, "camera_lid")
    cut_ = [box(ix0, ix1, -iy, iy, CB_FLOOR + 11.0, z0 + 0.6),                              # module pocket
            hull([(lx + d + (rr + 0.4) * math.cos(math.radians(a)), ly + (rr + 0.4) * math.sin(math.radians(a)), z)
                  for d in (-XW_SLACK_X, XW_SLACK_X) for a in range(0, 360, 10) for z in (z0 - 1, z0 + t + 1)], "slot")]   # lens slot
    for c in CB_SCREW_XY:
        cut_ += [cyl(M3_CLEAR / 2, z0 - 1, z0 + t + 1, c=c, seg=24),
                 cyl(3.4, z0 + t - 1.0, z0 + t + 1, c=c, seg=32)]                            # counterbore for the head
    return cut(lid, cut_)


def build_camera_lid_ov3660():
    """Lid: a collar locates the camera module and the lens, pads stop the boards lifting. Built in place, exported
    flipped so it prints with its flat top face on the bed."""
    cx, cy, x0, x1, oy, top = cb_dims()
    t = 2.4
    z0 = top                                                                              # lid underside
    m = XS_MOD
    parts_ = [box(x0, x1, -oy, oy, z0, z0 + t, "camera_lid"),
              box(m[0] - 0.4 - 1.2, m[1] + 0.4 + 1.2, m[2] - 0.4 - 1.2, m[3] + 0.4 + 1.2, CB_FLOOR + 12.0, z0 + 0.1),   # collar
              box(-XS_W / 2, XS_FPC[1] - 0.5, -8.0, 8.0, CB_FLOOR + XS_FPC[5] + 0.6, z0 + 0.1)]                        # over the connector
    for sy in (-1, 1):                                                                    # +x corners, beside the collar
        parts_.append(box(5.0, XS_W / 2, sy * 6.8, sy * 8.6, CB_FLOOR + XS_PCB_TOP + 0.8, z0 + 0.1) if sy > 0 else
                      box(5.0, XS_W / 2, -8.6, -6.8, CB_FLOOR + XS_PCB_TOP + 0.8, z0 + 0.1))
    lid = union(parts_, "camera_lid")
    lx, ly, lr, _ = XS_LENS
    cut_ = [box(m[0] - 0.4, m[1] + 0.4, m[2] - 0.4, m[3] + 0.4, CB_FLOOR + 11.0, z0),                                  # module pocket
            cyl(lr + 0.9, z0 - 1, z0 + t + 1, c=(lx, ly), seg=48),                                                     # lens hole
            cone_z(lr + 0.9, lr + 0.9 + 1.8, z0 + 0.6, z0 + t + 0.01, c=(lx, ly), seg=48)]                             # flared out
    cut_ += [cyl(M3_CLEAR / 2, z0 - 1, z0 + t + 1, c=(0, y), seg=24) for y in (-CB_SCREW_Y, CB_SCREW_Y)]
    return cut(lid, cut_)


def build_camera_wedge():
    """25 deg wedge between the base's camera seat and the camera box (v5). Built in the seat's box frame (z = 0 is
    the seat surface). Two M3 x 8/10 screws go down through it into the seat's nuts (heads on a shelf, in pockets the
    box covers); four M3 nuts press into hex pockets in its top for the box/lid screws; the power wires pass through a
    tunnel from the box's floor slot to the seat's slot. Prints seat-face down."""
    cx, cy, x0, x1, oy, top = cb_dims()
    W = wedge_tf()
    lo = [(x, y, 0.0) for x in (x0, x1) for y in (-oy, oy)]
    up = [tuple(W @ Vector((x, y, 0.0))) for x in (x0, x1) for y in (-oy, oy)]
    wedge = hull(lo + up, "camera_wedge")
    cut_ = []
    zt = top + 2.4 - CB_SCREW                                         # box screw tip in the box frame
    for c in CB_SCREW_XY:                                             # box/lid screws: nut pocket open at the top
        cut_ += [hexp(CAM_NUT_R, -2.7, 0.5, c=c, M=W),             # printed tilted 25 deg: the roomier sizes
                 cyl(CAM_SEAT_HOLE / 2, zt - 1.0, 0.5, c=c, seg=24, M=W)]
    for y in (-CB_SCREW_Y, CB_SCREW_Y):                               # wedge -> seat screws, heads on a shelf
        cut_ += [cyl(M3_CLEAR / 2, -1, WEDGE_SHELF + 0.1, c=(0, y), seg=24),
                 cyl(3.3, WEDGE_SHELF, 40, c=(0, y), seg=32)]
    # power wires: tunnel from the box floor slot down to the seat's slot
    slot_b = [tuple(W @ Vector((x, y, z))) for x in (6.5, 10.5) for y in (-8.0, 8.0) for z in (-0.5, 0.5)]
    slot_s = [(x, y, z) for x in (6.5, 10.5) for y in (-8.0, 8.0) for z in (-1.0, 0.5)]
    cut_.append(hull(slot_b + slot_s, "wire_tunnel"))
    return cut(wedge, cut_)


LENS_RING_OD, LENS_RING_ID = 17.0, 8.4   # cover the lid's lens slot (wherever the lens sits in its +-1.2 mm slack)
LENS_RING_HS = (3.5, 4.0, 4.5)           # the exposed neck is ~5.1 mm (photo estimate): print all three, keep the snug one


def build_lens_ring_halves(h):
    """'Choker' ring around the wide lens's threaded neck, between the lid and the lens's front ring, in two halves
    that press together around it: half A has two 2 mm pins on its cut face, half B two 2.3 mm holes. Built around
    the lens axis (z up, on z = 0..h, split on y = 0); both halves print flat as built (like half a washer)."""
    ro, ri = LENS_RING_OD / 2, LENS_RING_ID / 2
    xm = (ro + ri) / 2
    tag = f"{int(round(h * 10)):02d}"
    out = []
    for half, sy in (("a", 1), ("b", -1)):
        r = cut(cyl(ro, 0, h, seg=64, name=f"lens_ring_{tag}_{half}"),
                [cyl(ri, -1, h + 1, seg=48), box(-ro - 1, ro + 1, -sy * (ro + 1), 0, -1, h + 1)])   # keep y*sy >= 0
        if half == "a":                                                  # pins stick out of the cut face into -y
            r = union([r] + [cyl(1.0, -2.2, 0.01, "Y", c=(x, h / 2), seg=24) for x in (-xm, xm)], f"lens_ring_{tag}_a")
        else:                                                            # matching holes go into -y
            r = cut(r, [cyl(1.15, -2.7, 0.01, "Y", c=(x, h / 2), seg=24) for x in (-xm, xm)])
        out.append(r)
    return out


def build_camera_shim(th):
    """Shim under the XIAO's flat underside, if the camera rattles under the lid."""
    cx, cy, x0, x1, oy, top = cb_dims()
    return box(-cx + 0.3, cx - 0.3, -cy + 0.3, cy - 0.3, 0, th, f"camera_shim_{int(th * 10):02d}")


# Camera box built into the base: box frame -> world. Box x (USB-C end) points to the robot's left (+Y), box y runs up
# the 30 deg slope, box z (the lens) looks forward and 30 deg up. The lens sits on the centre line (y = 0).
CAM_BOX_X, CAM_BOX_Z = 62.0, -10.0        # box frame origin (centre of the box's back face)


def cam_box_frame():
    t = math.radians(CAM_TILT)
    return Matrix(((0, -math.sin(t), math.cos(t), CAM_BOX_X),
                   (1, 0, 0, -XS_LENS[0]),
                   (0, math.cos(t), math.sin(t), CAM_BOX_Z),
                   (0, 0, 0, 1)))


def camera_dummy(z0, name="xiao_dummy", dx_mod=0.0):
    """The measured XIAO Sense stack with the wide-angle module (XW_*) as solid blocks, at height z0 (board frame), for
    fit checks. dx_mod shifts just the module along x (its position is a photo estimate)."""
    m, (lx, ly, br, rz, rr, lt) = XW_MOD, XW_LENS
    return union([box(-XS_W / 2, XS_W / 2, -XS_H / 2, XS_H / 2, z0, z0 + XS_PCB_TOP),
                  box(*XS_USB[:4], z0 + XS_USB[4], z0 + XS_USB[5]),
                  box(*XS_FPC[:4], z0 + XS_FPC[4] - 0.1, z0 + XS_FPC[5]),
                  box(m[0] + dx_mod, m[1] + dx_mod, m[2], m[3], z0 + m[4], z0 + m[5]),
                  box(m[0] + dx_mod + 1.0, m[1] + dx_mod - 1.0, -4.0, 4.0, z0 + XS_PCB_TOP - 0.1, z0 + m[4] + 0.01),  # what it rests on
                  cyl(br, z0 + m[5] - 0.01, z0 + rz + 0.01, c=(lx + dx_mod, ly), seg=48),
                  cyl(rr, z0 + rz, z0 + lt, c=(lx + dx_mod, ly), seg=48)], name)


def build_fit_test(s, name):
    """Quick print to check servo pocket, screw pilots and horn recess."""
    L = s.flange_L + 8
    t = box(s.cx - L / 2, s.cx + L / 2, -s.W / 2 - 5, s.W / 2 + 5, 0, 4, name)
    hy = s.W / 2 + 5 + 22
    horn = box(s.cx - 22, s.cx + 22, hy - 16, hy + 16, 0, s.horn_t + 1.5)
    union([t, horn], name)
    cutters = [box(-s.off - CLR, s.L - s.off + CLR, -s.W / 2 - CLR, s.W / 2 + CLR, -1, 5)]
    cutters += [cyl(s.pilot / 2, -1, 5, c=h, seg=24) for h in s.holes]
    # horn recess, same shape as in the arm parts
    j = NS(s=s, yf=0, P1=(s.horn_z - s.horn_t, s.horn_z - s.horn_t + 4), P2=(-100, -99))
    rc = clevis_cutters(j)
    for o in rc[:-1]:
        o.data.transform(Matrix.Translation((s.cx, hy, 1.5)) @ Matrix.Rotation(math.radians(90), 4, "X")
                         @ Matrix.Translation((0, -(s.horn_z - s.horn_t), 0)))
    bpy.data.objects.remove(rc[-1], do_unlink=True)
    return cut(t, cutters + rc[:-1])


def text_solid(txt, x, y, z0, z1, size):
    """Engraving cutter: extruded text centred on (x, y), spanning z0..z1."""
    cu = bpy.data.curves.new("txt", "FONT")
    cu.body = txt; cu.size = size; cu.extrude = (z1 - z0) / 2
    cu.align_x = "CENTER"; cu.align_y = "CENTER"
    ob = bpy.data.objects.new("txt", cu); scene.collection.objects.link(ob)
    ob.location = (x, y, (z0 + z1) / 2)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    me.transform(ob.matrix_world)
    bpy.data.objects.remove(ob, do_unlink=True)
    solid = bpy.data.objects.new("txt", me); scene.collection.objects.link(solid)
    return solid


HORN_TEST_D = (20.6, 20.8, 21.0, 21.2, 21.4, 21.6)     # recess diameters on the horn fit test
FEELER_T = (0.4, 0.6, 0.8, 1.0, 1.5, 2.0)              # feeler tab thicknesses


def build_horn_fit_test():
    """Stage-1 test print. Strip of round-horn recesses (pick the snuggest) + a feeler comb for the gap
    between the turntable and the base rim (nominal DISC_GAP). Lies flat, engraved labels on top."""
    s = MG995
    pitch, t = 28.0, 4.0
    n = len(HORN_TEST_D)
    strip = box(0, n * pitch + 4, -26, 15, 0, t, "horn_fit_test")
    cutters = []
    for i, d in enumerate(HORN_TEST_D):
        x = 2 + pitch / 2 + i * pitch
        cutters += [cyl(d / 2, t - s.horn_t, t + 1, c=(x, 0), seg=96),
                    cyl(3.5, -1, t + 1, c=(x, 0), seg=32),                       # push the horn out from below
                    text_solid(f"{d:.1f}", x, -19, t - 0.6, t + 1, 4.5)]
    strip = cut(strip, cutters)
    # feeler comb: handle with engraved thicknesses, tabs pointing away from the strip
    hy0, hy1, tab_w, tab_len, tp = 24, 36, 10, 22, 17.0
    handle = box(0, n * tp + 4, hy0, hy1, 0, 3)
    tabs = [box(4 + i * tp, 4 + i * tp + tab_w, hy1 - 0.1, hy1 + tab_len, 0, ft) for i, ft in enumerate(FEELER_T)]
    comb = union([handle] + tabs, "feeler")
    comb = cut(comb, [text_solid(f"{ft:g}", 4 + i * tp + tab_w / 2, (hy0 + hy1) / 2, 2.4, 4, 4.5)
                      for i, ft in enumerate(FEELER_T)])
    return union([strip, comb], "horn_fit_test")


def build_horn_fit_test2():
    """Horn fit test in the real parts' print orientations: (1) pockets opening onto the bed with a chamfered mouth,
    like the turntable underside; (2) an upright plate with sideways teardrop pockets, like the arm forks."""
    s = MG995
    depth = s.horn_t + s.seat
    ds_flat, ds_side = (21.3, 21.5, 21.7), (21.5, 21.7)
    # (1) flat plate, 6 mm thick like the turntable disc, pockets on the underside
    pitch = 27.0
    flat = box(0, len(ds_flat) * pitch + 2, 0, 30, 0, 6.0, "horn_fit_test2")
    cut_f = []
    for i, d in enumerate(ds_flat):
        x = 1 + pitch / 2 + i * pitch
        cut_f += [cyl(d / 2, -1, depth, c=(x, 15), seg=96),
                  cone_z(d / 2 + HORN_CHAMFER + 1, d / 2, -1, HORN_CHAMFER, c=(x, 15)),
                  cyl(3.5, -1, 7, c=(x, 15), seg=32),
                  text_solid(f"{d:.1f}", x, 2.6, 5.4, 7, 3.2)]
    flat = cut(flat, cut_f)
    # (2) upright plate, 6 mm thick like a fork plate (y), 30 mm tall (z), on a foot; pockets open towards +y
    y0, H = 40.0, 30.0
    plate = union([box(0, 64, y0, y0 + 6, 0, H), box(0, 64, y0 - 6, y0 + 12, 0, 3)], "side")
    cut_s = []
    for i, d in enumerate(ds_side):
        x, zc = 17 + i * 30, H / 2
        r = d / 2
        cut_s.append(cyl(r, y0 + 6 - depth, y0 + 7, "Y", c=(x, zc), seg=96))
        k, top = r * math.sqrt(0.5), min(r * math.sqrt(2), H / 2 - 1.0)
        w = max(0.9, k - (top - k))
        cut_s.append(hull([(x + dx, y, zc + dz) for y in (y0 + 6 - depth, y0 + 7)
                           for dx, dz in ((-k, k), (k, k), (-w, top), (w, top))], "tear"))
        cut_s.append(cyl(3.5, y0 - 1, y0 + 7, "Y", c=(x, zc), seg=32))
        cut_s.append(text_solid(f"{d:.1f}", x, y0 + 9, 2.4, 3.2, 3.2))
    plate = cut(plate, cut_s)
    return union([flat, plate], "horn_fit_test2")


def dummy_servo(s, M, name):
    """Visual stand-in (not printed)."""
    w = s.W / 2
    ob = union([box(-s.off, s.L - s.off, -w, w, -s.below, s.case_top, M=M),
                box(s.fx[0], s.fx[1], -w, w, 0, s.flange_t, M=M),
                cyl(4.0, s.case_top, s.horn_z - s.horn_t, c=(0, 0), seg=32, M=M)], name)
    if s.horn == "round":
        h = cyl(s.horn_d / 2, s.horn_z - s.horn_t, s.horn_z, c=(0, 0), seg=48, M=M)
    else:
        h = box(-s.horn_len / 2, s.horn_len / 2, -s.horn_w / 2, s.horn_w / 2,
                s.horn_z - s.horn_t, s.horn_z, M=M)
    return union([ob, h], name)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob, do_unlink=True)

    parts = {
        "base": build_base(),
        "shoulder": build_shoulder(),
        "upper_arm": link_body(J_SH, J_EL, L1, "upper_arm"),
        "forearm": link_body(J_EL, J_WR, L2, "forearm"),
        "hand": build_hand(),
        "twist_bracket": build_twist_bracket(),
        "hand_mount": build_hand_mount(),
        "fit_test_mg995": build_fit_test(MG995, "fit_test_mg995"),
        "fit_test_mg90s": build_fit_test(MG90S, "fit_test_mg90s"),
        "bay_lid": build_bay_lid(),
        "pan_pointer": build_pan_pointer(),
        "pan_pointer_support": build_pan_pointer_support(),
        "pivot_spacer": build_pivot_spacer(),
        "horn_fit_test": build_horn_fit_test(),
        "horn_fit_test2": build_horn_fit_test2(),
    }
    parts["camera_box"] = build_camera_box()
    parts["camera_lid"] = build_camera_lid()
    parts["camera_wedge"] = build_camera_wedge()
    for h in LENS_RING_HS:
        for r in build_lens_ring_halves(h):
            parts[r.name] = r
    for th in CB_SHIMS:
        parts[f"camera_shim_{int(th * 10):02d}"] = build_camera_shim(th)
    colors = {"base": (0.18, 0.2, 0.24, 1), "shoulder": (0.9, 0.45, 0.1, 1),
              "upper_arm": (0.95, 0.95, 0.95, 1), "forearm": (0.9, 0.45, 0.1, 1),
              "hand": (0.95, 0.75, 0.2, 1), "twist_bracket": (0.9, 0.45, 0.1, 1), "hand_mount": (0.95, 0.95, 0.95, 1), "fit_test_mg995": (0.3, 0.7, 0.4, 1),
              "fit_test_mg90s": (0.3, 0.7, 0.4, 1), "bay_lid": (0.25, 0.28, 0.33, 1),
              "pan_pointer": (0.9, 0.45, 0.1, 1), "pan_pointer_support": (0.6, 0.6, 0.6, 1),
              "pivot_spacer": (0.95, 0.95, 0.95, 1), "horn_fit_test": (0.3, 0.7, 0.4, 1), "horn_fit_test2": (0.3, 0.7, 0.4, 1), "camera_box": (0.3, 0.7, 0.4, 1),
              "camera_lid": (0.3, 0.7, 0.4, 1), "camera_wedge": (0.3, 0.7, 0.4, 1), "camera_shim_06": (0.3, 0.7, 0.4, 1), "camera_shim_12": (0.3, 0.7, 0.4, 1)}
    os.makedirs(os.path.join(root, "stl"), exist_ok=True)
    for k, ob in parts.items():
        cleanup(ob)
        ob.color = colors.get(k, (0.3, 0.7, 0.4, 1))
        bpy.ops.object.select_all(action="DESELECT")
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        flip = Matrix.Rotation(math.pi, 4, "X") if k in ("bay_lid", "camera_lid") else None   # lids print flat top down
        if flip:
            ob.data.transform(flip)
        bpy.ops.wm.stl_export(filepath=os.path.join(root, "stl", f"{k}.stl"),
                              export_selected_objects=True, global_scale=1.0)
        if flip:
            ob.data.transform(flip.inverted())
        print(f"[part] {k:15s} dims={tuple(round(d, 1) for d in ob.dimensions)}")

    # ---- assembly
    servos = {"servo_pan": dummy_servo(MG995, M_PAN, "servo_pan"),
              "servo_shoulder": dummy_servo(MG995, M_SH, "servo_shoulder"),
              "servo_elbow": dummy_servo(MG995, M_EL, "servo_elbow"),
              "servo_wrist": dummy_servo(MG90S, M_WR, "servo_wrist"),
              "servo_twist": dummy_servo(MG90S, M_TWS, "servo_twist")}
    for s in servos.values():
        s.color = (0.08, 0.08, 0.1, 1)
    xiao = camera_dummy(CB_FLOOR, "xiao")                     # the measured camera, seated in the base's camera box
    xiao.data.transform(cam_box_frame_fitted())
    xiao.color = (0.1, 0.35, 0.8, 1)
    boxf = parts["camera_box"].copy()                         # the camera box on its seat
    boxf.data = parts["camera_box"].data.copy()
    boxf.data.transform(cam_box_frame_fitted())
    boxf.name = "cam_box_fitted"
    scene.collection.objects.link(boxf)
    boxf.hide_render = False
    boxf.hide_set(False)
    boxf.color = (0.9, 0.9, 0.86, 1)
    lidf = parts["camera_lid"].copy()                         # and a lid screwed on (the printed part is camera_lid)
    lidf.data = parts["camera_lid"].data.copy()
    lidf.data.transform(cam_box_frame_fitted())
    lidf.name = "cam_lid_fitted"
    scene.collection.objects.link(lidf)
    lidf.hide_render = False
    lidf.hide_set(False)
    lidf.color = (0.9, 0.45, 0.1, 1)
    wdgf = parts["camera_wedge"].copy()                       # the 25 deg wedge on the seat, under the box
    wdgf.data = parts["camera_wedge"].data.copy()
    wdgf.data.transform(cam_box_frame())
    wdgf.name = "cam_wedge_fitted"
    scene.collection.objects.link(wdgf)
    wdgf.hide_render = False
    wdgf.hide_set(False)
    wdgf.color = (0.3, 0.7, 0.4, 1)
    cxd, cyd, x0d, x1d, oyd, topd = cb_dims()                 # the 4.0 mm lens ring around the lens neck, on the lid
    for half in ("a", "b"):
        src = parts[f"lens_ring_40_{half}"]
        rf = src.copy(); rf.data = src.data.copy()
        rf.data.transform(cam_box_frame_fitted() @ Matrix.Translation((XW_LENS[0], XW_LENS[1], topd + 2.4)))
        rf.name = f"lens_ring_fitted_{half}"
        scene.collection.objects.link(rf)
        rf.hide_render = False
        rf.hide_set(False)
        rf.color = (0.9, 0.45, 0.1, 1)
    for k in ("fit_test_mg995", "fit_test_mg90s", "pan_pointer", "pan_pointer_support", "pivot_spacer", "horn_fit_test", "horn_fit_test2", "camera_box", "camera_lid",
              "camera_wedge", "camera_shim_06", "camera_shim_12") + tuple(k for k in parts if k.startswith("lens_ring_")):
        parts[k].hide_render = True
        parts[k].hide_set(True)

    def pivot(name, parent, loc):
        e = bpy.data.objects.new(name, None)
        scene.collection.objects.link(e)
        e.parent = parent
        e.location = loc
        e.rotation_mode = "XYZ"
        return e

    pan = pivot("pan_pivot", None, (0, 0, 0))
    sho = pivot("shoulder_pivot", pan, (0, 0, Z_SH))
    elb = pivot("elbow_pivot", sho, (L1, 0, 0))
    wri = pivot("wrist_pivot", elb, (L2, 0, 0))
    for ob, par in ((parts["shoulder"], pan), (servos["servo_shoulder"], pan),
                    (parts["upper_arm"], sho), (servos["servo_elbow"], sho),
                    (parts["forearm"], elb), (servos["servo_wrist"], elb),
                    (parts["twist_bracket"], wri), (servos["servo_twist"], wri)):
        ob.parent = par
    twi = pivot("twist_pivot", wri, (TW_X, 0, TW_Z))             # twist axis = X through (TW_X, 0, TW_Z)
    for ob in (parts["hand"], parts["hand_mount"]):
        ob.parent = twi
        ob.matrix_parent_inverse = Matrix.Translation((-TW_X, 0, -TW_Z))

    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(root, "cad", "highfive_robot.blend"))
    if "--render" in sys.argv:
        render(root)


def pose(pan=0, sh=0, el=0, wr=0, tw=0):
    O = bpy.data.objects
    O["pan_pivot"].rotation_euler = (0, 0, math.radians(pan))
    O["shoulder_pivot"].rotation_euler = (0, math.radians(-sh), 0)
    O["elbow_pivot"].rotation_euler = (0, math.radians(-el), 0)
    O["wrist_pivot"].rotation_euler = (0, math.radians(-wr), 0)
    O["twist_pivot"].rotation_euler = (math.radians(tw), 0, 0)
    bpy.context.view_layer.update()


POSES = {   # shoulder, elbow, wrist in degrees (0 = straight out, + = up); from the IK check
    "rest": dict(sh=90, el=90, wr=-5),          # folded up and back, out of the camera's view
    "reach": dict(pan=15, sh=2, el=61, wr=27),  # palm 24 cm out, 30 cm above the desk
    "slap": dict(sh=-9, el=55, wr=43),          # palm 27 cm out, 25 cm above the desk
}


def render(root):
    scene.render.engine = "BLENDER_WORKBENCH"
    scene.display.shading.color_type = "OBJECT"
    scene.display.shading.light = "STUDIO"
    scene.display.shading.show_cavity = True
    scene.display.shading.show_shadows = True
    scene.render.resolution_x, scene.render.resolution_y = 1400, 1000
    cam_data = bpy.data.cameras.new("cam")
    cam = bpy.data.objects.new("cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam

    def shot(name, loc, target, **p):
        pose(**p)
        cam.location = loc
        cam.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = os.path.join(root, "renders", name)
        bpy.ops.render.render(write_still=True)

    shot("rest_iso.png", (650, -560, 420), (30, 0, 110), **POSES["rest"])
    shot("reach_iso.png", (650, -560, 420), (100, 0, 130), **POSES["reach"])
    shot("slap_side.png", (130, -850, 180), (130, 0, 150), **POSES["slap"])
    shot("slap_front.png", (800, 200, 220), (80, 0, 150), **POSES["slap"])
    shot("camera_closeup.png", (240, -170, 70), (60, 0, 0), **POSES["rest"])
    shot("wrist_closeup.png", (380, -260, 330), (220, 0, 200), **POSES["reach"])

    # stage 1: base + pan servo + camera + pointer, lid off to show the bay
    O = bpy.data.objects
    stage1 = {"base", "servo_pan", "xiao", "pan_pointer", "breadboard", "devkit"}
    bb = box(-112, -57.5, -41.25, 41.25, BASE_BOTTOM + 3, BASE_BOTTOM + 11.5, "breadboard")
    bb.color = (0.92, 0.92, 0.88, 1)
    dk = box(-98, -72, -35, 35, BASE_BOTTOM + 20, BASE_BOTTOM + 21.6, "devkit")
    dk.color = (0.1, 0.25, 0.12, 1)
    hidden = [o for o in O if o.type == "MESH" and o.name not in stage1 and not o.hide_render]
    for o in hidden:
        o.hide_render = True
    O["pan_pointer"].hide_render = False
    shot("stage1_iso.png", (420, -380, 300), (-20, 0, -5), pan=25)
    shot("stage1_bay.png", (-330, -220, 260), (-80, 0, -15), pan=25)
    for o in hidden:
        o.hide_render = False
    for o in (bb, dk):
        bpy.data.objects.remove(o, do_unlink=True)
    O["pan_pointer"].hide_render = True
    pose()


if __name__ == "__main__":
    main()
