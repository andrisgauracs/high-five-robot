"""Rebuild print/stage4 (oriented STLs + 3MF bed plates for a 225 x 225 mm bed) from stl/.
Run from the project root after regenerating the STLs:  python3 print/make_stage4_plates.py"""
import struct, zipfile

BED_C = 112.5   # bed centre, ELEGOO Neptune 3 Pro
OUT = "print/stage4"


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




parts = {k: load(f"stl/{k}.stl") for k in ("hand", "twist_bracket", "hand_mount", "pivot_spacer")}
for k, t in parts.items():
    write_stl(f"{OUT}/{k}.stl", place(t, BED_C, BED_C))
# all in their exported orientation: hand palm down, bracket base down, hand mount plate down, spacer flat
h, br, hm, sp = (parts[k] for k in ("hand", "twist_bracket", "hand_mount", "pivot_spacer"))
H = lambda t: bbox(t)[3] - bbox(t)[2]
W = lambda t: bbox(t)[1] - bbox(t)[0]
y0 = BED_C - (H(h) + 10 + H(br)) / 2                  # hand in front, the twist parts in a row behind it
yb = y0 + H(h) + 10 + H(br) / 2
x0 = BED_C - (W(br) + 8 + W(hm) + 8 + W(sp)) / 2
write_3mf(f"{OUT}/plate1_hand_and_twist.3mf",
          [("hand", place(h, BED_C, y0 + H(h) / 2)),
           ("twist_bracket", place(br, x0 + W(br) / 2, yb)),
           ("hand_mount", place(hm, x0 + W(br) + 8 + W(hm) / 2, yb)),
           ("pivot_spacer", place(sp, x0 + W(br) + 8 + W(hm) + 8 + W(sp) / 2, yb - 7)),
           ("pivot_spacer_2", place(sp, x0 + W(br) + 8 + W(hm) + 8 + W(sp) / 2, yb + 7))])
write_3mf(f"{OUT}/plate1a_hand_only.3mf", [("hand", place(h, BED_C, BED_C))])
write_3mf(f"{OUT}/plate1b_twist_parts_only.3mf",
          [("twist_bracket", place(br, BED_C, BED_C - 30)), ("hand_mount", place(hm, BED_C - 20, BED_C + 30)),
           ("pivot_spacer", place(sp, BED_C + 25, BED_C + 23)), ("pivot_spacer_2", place(sp, BED_C + 25, BED_C + 37))])
print("plates written to", OUT)
