"""Fit check of the camera box + lid against the measured XIAO Sense stack (camera_dummy), with controls.
blender -b cad/highfive_robot.blend --python tools/check_camera_box.py"""
import bpy, math, sys
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
sys.argv = ["x"]
P = {"__name__": "cad"}
exec(open("cad/highfive_robot.py").read(), P)
O = bpy.data.objects
def T(ob):
    dg = bpy.context.evaluated_depsgraph_get(); e = ob.evaluated_get(dg); me = e.to_mesh()
    t = BVHTree.FromPolygons([e.matrix_world @ v.co for v in me.vertices], [tuple(p.vertices) for p in me.polygons]); e.to_mesh_clear(); return t
BOX, LID = T(O["camera_box"]), T(O["camera_lid"])
F = P["CB_FLOOR"]
def dummy(z0, dx=0.0, dy=0.0):
    d = P["camera_dummy"](z0, "dum"); d.location = (dx, dy, 0); bpy.context.view_layer.update(); t = T(d); bpy.data.objects.remove(d); return t
def rep(label, t, expect_clear=True):
    hb, hl = BOX.overlap(t), LID.overlap(t)
    hit = (["box"] if hb else []) + (["lid"] if hl else [])
    ok = (not hit) == expect_clear
    print(f"CAMBOX {'OK  ' if ok else 'FAIL'} {label}: {'touches ' + '+'.join(hit) if hit else 'clear'}")
rep("camera seated on the floor", dummy(F + 0.05))
rep("camera on 0.5 mm of shim (just under the snug point)", dummy(F + 0.5))
rep("camera on 0.7 mm of shim: lid presses it (snug)", dummy(F + 0.7), expect_clear=False)
rep("CONTROL camera 1 mm towards +x (module into the collar)", dummy(F + 0.05, dx=1.0), expect_clear=False)
rep("CONTROL camera 1 mm towards +y", dummy(F + 0.05, dy=1.0), expect_clear=False)
rep("CONTROL camera 1.2 mm higher (too many shims)", dummy(F + 1.25), expect_clear=False)
# USB-C plug body from the +x end, centred on the socket, at both shim heights
zc = F + (P["XS_USB"][4] + P["XS_USB"][5]) / 2
for sh in (0.0, 0.6):
    b = P["box"](P["XS_USB"][1] + 0.1, 35, -6.25, 6.25, zc + sh - 3.5, zc + sh + 3.5, "plug"); t = T(b); bpy.data.objects.remove(b)
    rep(f"USB-C plug body (12.5 x 7 mm) at shim {sh}", t)
# screws M3x16 from the lid top, and the nuts
cx, cy, x0, x1, oy, top = P["cb_dims"]()
zn = top + 2.4 - 16 + 0.3
for y in (-P["CB_SCREW_Y"], P["CB_SCREW_Y"]):
    s_ = P["cyl"](1.45, top + 2.4 - 16, top + 2.4, c=(0, y), seg=24, name="scr"); t = T(s_); bpy.data.objects.remove(s_)
    rep(f"M3x16 screw at y {y:+.1f}", t)
    n_ = P["hexp"](3.1, zn + 0.2, zn + 2.6, c=(0, y)); t = T(n_); bpy.data.objects.remove(n_)
    rep(f"M3 nut in its slot at y {y:+.1f} (screw tip at z {top + 2.4 - 16:.1f}, nut z {zn + 0.2:.1f}..{zn + 2.6:.1f})", t)
# lens view: rays from the lens top centre out to 40 deg off axis must clear the lid
lx, ly, br, rz, rr, lt = P["XW_LENS"]                  # wide-angle module (~120 deg): check out to 60 deg off axis
o = Vector((lx, ly, F + lt + 0.05)); blocked = 0; total = 0
for th in range(0, 61, 5):
    for ph in range(0, 360, 15):
        d = Vector((math.sin(math.radians(th)) * math.cos(math.radians(ph)), math.sin(math.radians(th)) * math.sin(math.radians(ph)), math.cos(math.radians(th))))
        for off in ((0, 0), (1.5, 0), (-1.5, 0), (0, 1.5), (0, -1.5)):
            total += 1; blocked += LID.ray_cast(o + Vector((*off, 0)), d, 200)[0] is not None
print(f"CAMBOX lens view to 60 deg off axis (rays from the lens and +-1.5 mm around it): {blocked}/{total} blocked by the lid")
