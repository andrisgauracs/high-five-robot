"""Camera mount v5 on the base's camera seat: seat -> 25 deg wedge -> camera box + lid (wide-angle OV2640).
Fit of the wedge, box, camera, all screws and nuts, the power-wire route, the USB-C plug and the lens view.
Controls must fail.  blender -b cad/highfive_robot.blend --python tools/check_camera_seat.py"""
import bpy, math, sys
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
sys.argv = ["x"]
P = {"__name__": "cad"}
exec(open("cad/highfive_robot.py").read(), P)
O = bpy.data.objects; sc = bpy.context.scene
Ms = P["cam_box_frame"]()            # the seat's frame (z = 0 on the seat surface)
Mb = P["cam_box_frame_fitted"]()     # the box as mounted on the wedge
F = P["CB_FLOOR"]
cx, cy, x0, x1, oy, top = P["cb_dims"]()
def T(ob):
    dg = bpy.context.evaluated_depsgraph_get(); e = ob.evaluated_get(dg); me = e.to_mesh()
    t = BVHTree.FromPolygons([e.matrix_world @ v.co for v in me.vertices], [tuple(p.vertices) for p in me.polygons]); e.to_mesh_clear(); return t
BASE, BOX, LID, WDG = T(O["base"]), T(O["cam_box_fitted"]), T(O["cam_lid_fitted"]), T(O["cam_wedge_fitted"])
def tmp(ob):
    t = T(ob); bpy.data.objects.remove(ob); return t
def rep(label, t, against, expect_clear=True):
    hit = [n for n, A in against if A.overlap(t)]
    ok = (not hit) == expect_clear
    print(f"SEAT {'OK  ' if ok else 'FAIL'} {label}: {'touches ' + '+'.join(hit) if hit else 'clear'}")
ALL = [("base", BASE), ("wedge", WDG), ("box", BOX), ("lid", LID)]
# the wedge in the seat's lip, and the box on the wedge (shrunk 0.05 so flush contacts don't count)
wd = P["build_camera_wedge"](); wd.data.transform(Ms @ Matrix.Translation((0, 0, 0.05)) @ Matrix.Scale(0.995, 4)); rep("wedge on the seat", tmp(wd), [("base", BASE)])
wd = P["build_camera_wedge"](); wd.data.transform(Ms @ Matrix.Translation((0, -1.0, 0.05))); rep("CONTROL wedge 1 mm down the slope (into the lip)", tmp(wd), [("base", BASE)], False)
rep("camera box on the wedge", tmp(P["box"](x0 + 0.05, x1 - 0.05, -oy + 0.05, oy - 0.05, 0.05, top - 0.05, "b", M=Mb)), [("base", BASE), ("wedge", WDG)])
rep("CONTROL box 1 mm into the wedge", tmp(P["box"](x0 + 0.05, x1 - 0.05, -oy + 0.05, oy - 0.05, -1.0, top - 0.05, "b", M=Mb)), [("wedge", WDG)], False)
def dummy(z0, dx=0.0):
    d = P["camera_dummy"](z0, "dum"); d.data.transform(Mb @ Matrix.Translation((dx, 0, 0))); return tmp(d)
rep("camera in the mounted box", dummy(F + 0.05), ALL)
rep("CONTROL camera 1 mm towards the USB-C end", dummy(F + 0.05, 1.0), ALL, False)
# box/lid screws: M3 x 25 from the lid's counterbore into nuts in the wedge's top
zt = top + 2.4 - P["CB_SCREW"]
for c in P["CB_SCREW_XY"]:
    rep(f"M3x25 box screw at {c[0]:+.0f},{c[1]:+.1f} (tip {zt:.1f} mm below the box)", tmp(P["cyl"](1.45, zt, top + 1.4, c=c, seg=24, name="s", M=Mb)), ALL)
    rep(f"M3 nut in the wedge at {c[0]:+.0f},{c[1]:+.1f}", tmp(P["hexp"](3.17, -2.6, -0.1, c=c, M=Mb)), ALL)
rep("CONTROL box screw 3 mm off its hole", tmp(P["cyl"](1.45, zt, top + 1.4, c=(P["CB_SCREW_XY"][0][0] + 3, P["CB_SCREW_XY"][0][1]), seg=24, name="s", M=Mb)), ALL, False)
print(f"SEAT     box screws: tip {zt:.1f} below the box underside, nut -2.6..0: each passes right through its nut")
# wedge -> seat screws (M3 x 8 and x 10) into the seat's nuts, heads (5.5 x 3) on the shelf
sh = P["WEDGE_SHELF"]
for L in (8, 10):
    for y in (-P["CB_SCREW_Y"], P["CB_SCREW_Y"]):
        scr = P["union"]([P["cyl"](1.45, sh - L, sh, c=(0, y), seg=24), P["cyl"](2.75, sh + 0.05, sh + 3.0, c=(0, y), seg=24)], "w")
        scr.data.transform(Ms); rep(f"M3x{L} wedge screw at y {y:+.1f} (tip {sh - L:.1f}, seat nut -2.8..0)", tmp(scr), ALL)
    rep(f"M3 nut in the seat (under the wedge screws)", tmp(P["hexp"](3.17, -2.6, -0.2, c=(0, P["CB_SCREW_Y"]), M=Ms)), ALL)
# power wires: the XIAO's USB-C end -> box floor slot -> wedge tunnel -> seat slot -> riser -> channel -> bay
def tube(pts, r):
    cu = bpy.data.curves.new("w", "CURVE"); cu.dimensions = "3D"; cu.bevel_depth = r; cu.bevel_resolution = 3
    sp = cu.splines.new("POLY"); sp.points.add(len(pts) - 1)
    for p_, c in zip(sp.points, pts): p_.co = (*c, 1)
    ob = bpy.data.objects.new("w", cu); sc.collection.objects.link(ob); bpy.context.view_layer.update(); return tmp(ob)
z0 = P["BASE_BOTTOM"] + 6
for label, ay in (("power wires", 0.0), ("CONTROL power wires 6 mm off the slot", 14.0)):
    a = Mb @ Vector((8.5, ay, F - 0.5)); m_ = Mb @ Vector((8.5, ay, -0.3)); s_ = Ms @ Vector((8.5, ay, -0.5)); b_ = Ms @ Vector((8.5, ay, -6.0))
    pts = [a, m_, s_, b_, Vector((b_.x, b_.y, z0)), Vector((b_.x + 2, 18.0, z0)), Vector((-40.0, 18.0, z0))]
    rep(label, tube(pts, 0.85), [("base", BASE), ("wedge", WDG), ("box", BOX)], expect_clear=(ay == 0.0))
# USB-C plug body from the +x end
zc = F + (P["XS_USB"][4] + P["XS_USB"][5]) / 2
rep("USB-C plug body", tmp(P["box"](P["XS_USB"][1] + 0.1, 40, -6.25, 6.25, zc - 3.5, zc + 3.5, "u", M=Mb)), ALL)
# lens view: rays from the lens top out to 60 deg off its axis (wide lens) against everything, arm at rest
O["shoulder_pivot"].rotation_euler = (0, math.radians(-90), 0); O["elbow_pivot"].rotation_euler = (0, math.radians(-90), 0)
bpy.context.view_layer.update()
arm = [T(O[n]) for n in ("shoulder", "upper_arm", "forearm", "twist_bracket", "hand", "servo_shoulder")]
lx, ly, br, rz, rr, lt = P["XW_LENS"]
o = Mb @ Vector((lx, ly, F + lt + 0.05)); N = (Mb.to_3x3() @ Vector((0, 0, 1))).normalized()
A_ = (Mb.to_3x3() @ Vector((1, 0, 0))).normalized(); B_ = N.cross(A_)
blocked = total = 0
for th in range(0, 61, 4):
    for ph in range(0, 360, 12):
        d = (N * math.cos(math.radians(th)) + (A_ * math.cos(math.radians(ph)) + B_ * math.sin(math.radians(ph))) * math.sin(math.radians(th))).normalized()
        total += 1
        blocked += any(t.ray_cast(o + d * 0.05, d, 3000)[0] is not None for t in [BASE, WDG, BOX, LID] + arm)
up = math.degrees(math.asin(N.z))
print(f"SEAT lens at {tuple(round(c, 1) for c in o)}, looking {up:.0f} deg above horizontal; view to 60 deg off axis: {blocked}/{total} rays blocked (arm at rest)")
