"""Rebuild print/stage1 (oriented STLs + 3MF bed plates for a 225 x 225 mm bed) from stl/.
Run from the project root after regenerating the STLs:  python3 print/make_stage1_plates.py"""
import struct, zipfile

BED_C = 112.5   # bed centre, ELEGOO Neptune 3 Pro
OUT = "print/stage1"

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

parts = {k: load(f"stl/{k}.stl") for k in ("fit_test_mg995", "base", "bay_lid", "pan_pointer")}
# the pointer prints together with its break-away support (same coordinates, one object)
parts["pan_pointer"] = parts["pan_pointer"] + load("stl/pan_pointer_support.stl")
for k, t in parts.items():
    write_stl(f"{OUT}/{k}.stl", place(t, BED_C, BED_C))
write_3mf(f"{OUT}/plate1_fit_test.3mf", [("fit_test_mg995", place(parts["fit_test_mg995"], BED_C, BED_C))])
write_3mf(f"{OUT}/plate2_base.3mf", [("base", place(parts["base"], BED_C, BED_C))])

# plate 3: lid turned 90 degrees, stacked front-to-back with the pointer, 12 mm apart
lid, ptr = rot90(parts["bay_lid"]), parts["pan_pointer"]
lh = bbox(lid)[3] - bbox(lid)[2]
ph = bbox(ptr)[3] - bbox(ptr)[2]
y0 = BED_C - (lh + 12 + ph) / 2
write_3mf(f"{OUT}/plate3_lid_and_pointer.3mf",
          [("bay_lid", place(lid, BED_C, y0 + lh / 2)), ("pan_pointer", place(ptr, BED_C, y0 + lh + 12 + ph / 2))])
# lid on its own, for reprinting just the lid
write_3mf(f"{OUT}/plate4_lid_only.3mf", [("bay_lid", place(parts["bay_lid"], BED_C, BED_C))])
# horn fit test (recess strip + feeler comb, flat) with the bay lid turned 90 degrees in front of it, 10 mm apart
ht = load("stl/horn_fit_test.stl")
hh = bbox(ht)[3] - bbox(ht)[2]
lh = bbox(lid)[3] - bbox(lid)[2]
y0 = BED_C - (hh + 10 + lh) / 2
write_3mf(f"{OUT}/plate5_horn_test.3mf", [("horn_fit_test", place(ht, BED_C, y0 + lh + 10 + hh / 2)),
                                           ("bay_lid", place(lid, BED_C, y0 + lh / 2))])
print("plates written to", OUT)
