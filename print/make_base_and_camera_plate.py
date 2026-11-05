"""One plate: base + bay lid + camera box, camera lid and shims. Tries base angles/offsets and packs the small parts
into the free space on a 1 mm occupancy grid built from the parts' real footprints.  python3 print/make_base_and_camera_plate.py"""
import math
src = open("print/make_stage4_plates.py").read().split("parts = {")[0]
exec(src)
BED = 225.0
GAP = 4.0          # clear gap between parts
EDGE = 8.0         # clear margin to the bed edge (user: 4 mm looked too close)
OUT = "print/stage1/plate2b_base_and_camera_box.3mf"
N = int(BED)


def rotz(tris, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [tuple(v for k in range(3) for v in (c * t[3 * k] - s * t[3 * k + 1], s * t[3 * k] + c * t[3 * k + 1], t[3 * k + 2]))
            for t in tris]


def footprint(tris):
    """Cells (1 mm) covered by the part's triangles (projected), as a set relative to its bbox corner."""
    x0, x1, y0, y1, _, _ = bbox(tris)
    cells = set()
    for t in tris:
        P = [(t[0] - x0, t[1] - y0), (t[3] - x0, t[4] - y0), (t[6] - x0, t[7] - y0)]
        xs, ys = [p[0] for p in P], [p[1] for p in P]
        (ax, ay), (bx, by), (cx, cy) = P
        d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        for i in range(int(min(xs)), int(max(xs)) + 1):
            for j in range(int(min(ys)), int(max(ys)) + 1):
                if abs(d) < 1e-9:
                    cells.add((i, j)); continue
                hit = False
                for px in (i, i + 1):
                    for py in (j, j + 1):
                        l1 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / d
                        l2 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / d
                        if l1 >= -0.02 and l2 >= -0.02 and 1 - l1 - l2 >= -0.02: hit = True
                if hit or (min(xs) >= i and max(xs) <= i + 1): cells.add((i, j))
    return cells, x1 - x0, y1 - y0


def grow(cells, r):
    r = int(math.ceil(r)); out = set()
    for (i, j) in cells:
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                out.add((i + di, j + dj))
    return out


parts = {k: load(f"stl/{k}.stl") for k in ("base", "bay_lid", "camera_box", "camera_lid", "camera_shim_06", "camera_shim_12")}
order = ["bay_lid", "camera_box", "camera_lid", "camera_shim_06", "camera_shim_12"]
fp = {}
for k in order:
    for a in (0, 90, 45, 135):
        t = rotz(parts[k], a); c, w, h = footprint(t); fp[(k, a)] = (t, c, w, h)


def try_layout(base_deg, bx, by):
    tb = rotz(parts["base"], base_deg); cb, w, h = footprint(tb)
    ox, oy = bx - w / 2, by - h / 2
    if ox < EDGE or oy < EDGE or ox + w > BED - EDGE or oy + h > BED - EDGE: return None
    occ = {(i + int(ox), j + int(oy)) for (i, j) in grow(cb, GAP)}
    placed = [("base", place(tb, bx, by))]
    for k in order:
        done = False
        for a in (0, 90, 45, 135):
            t, c, w2, h2 = fp[(k, a)]
            cg = grow(c, GAP)
            for y in range(int(EDGE), int(BED - EDGE - h2)):
                for x in range(int(EDGE), int(BED - EDGE - w2)):
                    if all((i + x, j + y) not in occ for (i, j) in c):
                        occ |= {(i + x, j + y) for (i, j) in cg}
                        placed.append((k, place(t, x + w2 / 2, y + h2 / 2))); done = True; break
                if done: break
            if done: break
        if not done: return None
    return placed


best = None
for deg in (0, 90):                       # base pushed towards one edge leaves a long free strip on the other
    for bx in range(112, 114):
        for by in range(76, 86, 1):
            r = try_layout(deg, bx, by)
            if r: best = (deg, bx, by, r); break
        if best: break
    if best: break
if not best:
    raise SystemExit("no layout found: print the bay lid on its own plate")
deg, bx, by, objs = best
print(f"base turned {deg} deg, centred at ({bx}, {by})")
for k, t in objs:
    x0, x1, y0, y1, _, _ = bbox(t)
    print(f"  {k:15s} x {x0:5.1f}..{x1:5.1f}  y {y0:5.1f}..{y1:5.1f}")
write_3mf(OUT, objs)
print("written", OUT)
