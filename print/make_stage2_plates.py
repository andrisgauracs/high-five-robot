"""Rebuild print/stage2 (oriented STLs + 3MF bed plates for a 225 x 225 mm bed) from stl/.
Run from the project root after regenerating the STLs:  python3 print/make_stage2_plates.py"""
import struct, zipfile

BED_C = 112.5   # bed centre, ELEGOO Neptune 3 Pro
OUT = "print/stage2"


def load(f):
    d = open(f, "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    return [struct.unpack("<9f", d[84 + i * 50 + 12:84 + i * 50 + 48]) for i in range(n)]


def bbox(t):
    xs = [a[i] for a in t for i in (0, 3, 6)]
    ys = [a[i] for a in t for i in (1, 4, 7)]
    zs = [a[i] for a in t for i in (2, 5, 8)]
    return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)


def place(tris, cx, cy):
    """Centre the part on (cx, cy) and drop it onto the bed."""
    x0, x1, y0, y1, z0, _ = bbox(tris)
    d = (cx - (x0 + x1) / 2, cy - (y0 + y1) / 2, -z0)
    return [tuple(t[i] + d[i % 3] for i in range(9)) for t in tris]


def rot90(tris):
    return [tuple(v for k in range(3) for v in (-t[3 * k + 1], t[3 * k], t[3 * k + 2])) for t in tris]


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"\0" * 80 + struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<12fH", 0, 0, 0, *t, 0))


def write_3mf(path, objs):
    res, items = [], []
    for oid, (name, tris) in enumerate(objs, 1):
        vmap, vs, fs = {}, [], []
        for t in tris:
            idx = []
            for k in range(3):
                v = tuple(round(c, 4) for c in t[3 * k:3 * k + 3])
                if v not in vmap:
                    vmap[v] = len(vs)
                    vs.append(v)
                idx.append(vmap[v])
            if len(set(idx)) == 3:
                fs.append(idx)
        res.append(f'<object id="{oid}" name="{name}" type="model"><mesh><vertices>'
                   + "".join(f'<vertex x="{a}" y="{b}" z="{c}"/>' for a, b, c in vs)
                   + '</vertices><triangles>'
                   + "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in fs)
                   + '</triangles></mesh></object>')
        items.append(f'<item objectid="{oid}"/>')
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"><resources>'
             + "".join(res) + '</resources><build>' + "".join(items) + '</build></model>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr("3D/3dmodel.model", model)



parts = {k: load(f"stl/{k}.stl") for k in ("shoulder", "upper_arm", "pivot_spacer")}
for k, t in parts.items():
    write_stl(f"{OUT}/{k}.stl", place(t, BED_C, BED_C))
# plate 1: shoulder in the middle, three pivot spacers in a row in front of it (one per joint)
sp = parts["pivot_spacer"]
write_3mf(f"{OUT}/plate1_shoulder_and_spacers.3mf",
          [("shoulder", place(parts["shoulder"], BED_C, BED_C + 15))]
          + [(f"pivot_spacer_{i + 1}", place(sp, BED_C - 20 + 20 * i, BED_C - 60)) for i in range(3)])
# plate 1b: shoulder on its own, for a reprint when the spacers are already printed
write_3mf(f"{OUT}/plate1b_shoulder_only.3mf", [("shoulder", place(parts["shoulder"], BED_C, BED_C))])
# plate 2: upper arm lying on its side, as oriented in the file
write_3mf(f"{OUT}/plate2_upper_arm.3mf", [("upper_arm", place(parts["upper_arm"], BED_C, BED_C))])
# plate 3: shoulder and upper arm together, stacked front to back, 10 mm apart
sh, ua = parts["shoulder"], parts["upper_arm"]
shh = bbox(sh)[3] - bbox(sh)[2]; uah = bbox(ua)[3] - bbox(ua)[2]
y0 = BED_C - (shh + 10 + uah) / 2
write_3mf(f"{OUT}/plate3_shoulder_and_upper_arm.3mf",
          [("upper_arm", place(ua, BED_C, y0 + uah / 2)), ("shoulder", place(sh, BED_C, y0 + uah + 10 + shh / 2))])
# horn fit test in the real parts' print orientations (pockets on the bed side, and sideways teardrop pockets)
write_3mf(f"{OUT}/plate4_horn_test2.3mf", [("horn_fit_test2", place(load("stl/horn_fit_test2.stl"), BED_C, BED_C))])
# just the upright plate of horn_fit_test2 (sideways teardrop pockets, like the arm forks)
ht2 = load("stl/horn_fit_test2.stl")
parent = {}
def _find(a):
    while parent.setdefault(a, a) != a:
        parent[a] = parent[parent[a]]; a = parent[a]
    return a
keys = [[tuple(round(c, 3) for c in t[k:k + 3]) for k in (0, 3, 6)] for t in ht2]
for ks in keys:
    for k in ks[1:]:
        ra, rb = _find(ks[0]), _find(k)
        if ra != rb: parent[ra] = rb
shells = {}
for t, ks in zip(ht2, keys):
    shells.setdefault(_find(ks[0]), []).append(t)
upright = max(shells.values(), key=lambda tris: bbox(tris)[5] - bbox(tris)[4])
write_3mf(f"{OUT}/plate4b_horn_test_sideways.3mf", [("horn_test_sideways", place(upright, BED_C, BED_C))])
print("plates written to", OUT)
