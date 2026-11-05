"""Collision sweep for the whole robot model: every motion-demo frame, the funny_moves key poses, the reference poses,
and a wrist x twist grid. Parts that share a joint are skipped (their dummy horns touch by design), except the base
against the turntable/shoulder, which must clear.
blender -b cad/highfive_robot.blend --python tools/check_collisions.py"""
import bpy, math, os, sys
from mathutils.bvhtree import BVHTree

O = bpy.data.objects
GROUPS = {"fix": ["base", "bay_lid", "servo_pan", "xiao", "cam_box_fitted", "cam_lid_fitted", "cam_wedge_fitted"], "pan": ["shoulder", "servo_shoulder"],
          "sho": ["upper_arm", "servo_elbow"], "elb": ["forearm", "servo_wrist"],
          "wri": ["twist_bracket", "servo_twist"], "twi": ["hand", "hand_mount"]}
ORDER = list(GROUPS)
EXTRA = [("base", "shoulder"), ("bay_lid", "shoulder"), ("xiao", "shoulder")]


def tree(n):
    dg = bpy.context.evaluated_depsgraph_get(); e = O[n].evaluated_get(dg); me = e.to_mesh()
    T = BVHTree.FromPolygons([e.matrix_world @ v.co for v in me.vertices], [tuple(p.vertices) for p in me.polygons])
    e.to_mesh_clear(); return T


def pose(pan=0, sh=0, el=0, wr=0, tw=0):
    O["pan_pivot"].rotation_euler = (0, 0, math.radians(pan)); O["shoulder_pivot"].rotation_euler = (0, math.radians(-sh), 0)
    O["elbow_pivot"].rotation_euler = (0, math.radians(-el), 0); O["wrist_pivot"].rotation_euler = (0, math.radians(-wr), 0)
    O["twist_pivot"].rotation_euler = (math.radians(tw), 0, 0); bpy.context.view_layer.update()


poses = []
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "anim", "arm_motion.py")).read()
A = {}; exec(src.split("def pose(")[0].replace("OUT = os.path.join(sys.argv[-1], \"frames\")", "OUT = None").replace("os.makedirs(OUT, exist_ok=True)", ""), A)
poses += [A["at"](f) for f in range(0, A["KEYS"][-1][0] + 1, 3)]
D = 2000 / 180
for sh0 in (0, 35):        # funny_moves key poses (pulse offsets -> angles), shoulder reference 0 or 35 deg up
    for (p, s, e, w, t) in [(0, -150, -400, 0, 0), (40, -150, -400, -200, -250), (-40, -150, -400, 200, 250),
                            (0, -80, -400, 180, 0), (120, -150, -400, 0, 350), (-120, -150, -400, 0, -350)]:
        poses.append(dict(pan=p / D, sh=sh0 - s / D, el=-e / D, wr=-w / D, tw=t / D))
poses += [dict(sh=90, el=90, wr=-5), dict(pan=15, sh=2, el=61, wr=27), dict(sh=-9, el=55, wr=43)]
poses += [dict(sh=35, el=68, wr=w, tw=t) for w in range(-38, 112, 15) for t in (-60, 0, 60)]
bad = {}
for q in poses:
    pose(**q); T = {n: tree(n) for g in GROUPS.values() for n in g}
    pairs = [(x, y) for i, ga in enumerate(ORDER) for gb in ORDER[i + 2:] for x in GROUPS[ga] for y in GROUPS[gb]] + EXTRA
    for x, y in pairs:
        if T[x].overlap(T[y]):
            bad.setdefault((x, y), q)
pose()
print(f"CHECK {len(poses)} poses:", "no collisions" if not bad else {k: {a: round(b) for a, b in v.items()} for k, v in bad.items()})
