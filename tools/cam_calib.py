"""Camera calibration with the robot's own hand: step the hand through a grid of poses, find it in each upright camera
frame by differencing against an empty-scene frame (arm at home, out of view), and record pose -> pixel.
Run: .venv/bin/python tools/cam_calib.py --v5 --red --all
Writes calibration/calib_v5redall.json and calibration/calib_sheet*.jpg (every frame, detection marked)."""
import json, os, sys, time, itertools
import numpy as np, cv2
sys.path.insert(0, os.path.dirname(__file__))
from arm import send
from cam_snap import snap, cam_setup, CAM

OUT = "calibration"
HOME = [1500, 1500, 1450, 1580, 1500]
# 2026-10-20 wide grid: base out to about +-45 deg, shoulder 1700..2100 (elbow 1450), plus a 1550-elbow layer
PANS = [1000, 1150, 1300, 1500, 1700, 1850, 2000]
LAYERS = [(1450, [1700, 1800, 1900, 2000, 2100]), (1550, [1800, 1900, 2000])]
# --top: the upper band (the picture's top corners), finer steps
TOP_PANS = [1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900]
TOP_LAYERS = [(1450, [1750, 1800, 1850, 1900])]
# --wide: the wide-angle OV2640 (2026-10-22): it sees much more, so a bigger grid in one run
WIDE_PANS = [1000, 1125, 1250, 1375, 1500, 1625, 1750, 1875, 2000]
WIDE_LAYERS = [(1450, [1650, 1750, 1850, 1950, 2050, 2100]), (1550, [1800, 1950])]
# --v5: the camera on the 30 deg wedge (2026-10-26) looks 60 deg up, so the hand enters the top of the picture from
# shoulder ~1600 and comes down as it rises. The home pose's hand tip already shows at the top edge, so the empty scene
# is taken with the shoulder at 1300.
# The first v5 runs showed the hand only in view for pan ~1250..1750, so the grid is dense over that area only.
V5_PANS = list(range(1225, 1776, 50))
V5_LAYERS = [(1450, list(range(1700, 2101, 50))), (1600, [1800, 1950, 2100])]
TAG = "_top" if "--top" in sys.argv else "_wide" if "--wide" in sys.argv else "_v5" if "--v5" in sys.argv else ""
if TAG == "_v5" and "--red" in sys.argv:      # 2026-10-26: a red cardboard label on the underside of the hand
    TAG = "_v5red"
    if "--ext" in sys.argv:                      # the areas the dense grid missed: the upper side wedges + top band
        TAG = "_v5redext"
    elif "--all" in sys.argv:                    # dense grid + those areas in one run (one background, one lighting)
        TAG = "_v5redall"
BG_POSE = [1500, 1300, 1450, 1580, 1500] if TAG.startswith("_v5") else None
if TAG == "_top":
    PANS, LAYERS = TOP_PANS, TOP_LAYERS
elif TAG == "_wide":
    PANS, LAYERS = WIDE_PANS, WIDE_LAYERS
elif TAG == "_v5redext":
    PANS, LAYERS = None, None
elif TAG == "_v5redall":
    PANS, LAYERS = V5_PANS, V5_LAYERS
elif TAG.startswith("_v5"):
    PANS, LAYERS = V5_PANS, V5_LAYERS
# --v5 --red --ext: explicit pose list (pan, shoulder, elbow)
EXT_POSES = [(p, s, 1450) for s in range(1600, 2051, 50) for p in (1100, 1125, 1150, 1175, 1200)] + \
            [(p, s, 1450) for s in range(1600, 2051, 50) for p in (1800, 1825, 1850, 1875, 1900)] + \
            [(p, s, 1450) for s in (1600, 1650) for p in range(1225, 1776, 50)]


def grab(path):
    return snap(path)


def find_hand(img, bg):
    """The black tape on the hand: new dark pixels (dark now, light in the empty scene) in the upper picture. The servos
    are dark too, but the hand hangs below them, so the tape is the lowest dark patch."""
    g, gb = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY)
    # dark now, and clearly darker than the empty scene (the ceiling near the top edge is only ~130, so no fixed
    # 'light background' threshold)
    # threshold relative to the background: with the room lit dimly the ceiling is only ~60, so ask for 'clearly
    # darker' (25 levels and below 65 % of the empty scene) rather than a fixed 45-level drop
    gbe = cv2.erode(gb, np.ones((7, 7), np.uint8)).astype(int)
    m = ((g < 90) & (gbe - g > 25) & (g < 0.65 * gbe)).astype(np.uint8)
    m[cv2.dilate((gb > 170).astype(np.uint8), np.ones((19, 19), np.uint8)) > 0] = 0      # the fairy lights flicker
    m[int(m.shape[0] * 0.7):] = 0               # (0.6 before the wide lens)
    m[:2] = 0                                   # the top rows of a frame can be garbage; tape cut off at the edge still counts
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    blobs = [k for k in range(1, n) if stats[k, cv2.CC_STAT_AREA] >= 25]
    if not blobs:
        return None
    k = max(blobs, key=lambda k: stats[k, cv2.CC_STAT_TOP] + stats[k, cv2.CC_STAT_HEIGHT])
    return {"x": float(cent[k][0]), "y": float(cent[k][1]), "area": int(stats[k, cv2.CC_STAT_AREA]),
            "box": [int(v) for v in stats[k, :4]]}


def find_hand_bright(img, bg):
    """For a dim room (wide-lens run, 2026-10-22): the white printed hand is far brighter than the dark ceiling, while
    the tape only shows as a thin edge-on strip. Take the biggest region that got clearly brighter (fairy lights masked)
    and use its lowest 15 px: the bottom edge of the hand, where the tape is."""
    g = cv2.GaussianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(int)
    gb = cv2.GaussianBlur(cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY), (5, 5), 0).astype(int)
    m = ((g - gb > 40) & (g > 110)).astype(np.uint8)
    m[cv2.dilate((gb > 150).astype(np.uint8), np.ones((21, 21), np.uint8)) > 0] = 0      # the fairy lights
    m[int(m.shape[0] * 0.75):] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[k, cv2.CC_STAT_AREA] < 300:
        return None
    ys, xs = np.nonzero(lab == k)
    sel = ys > ys.max() - 15
    return {"x": float(xs[sel].mean()), "y": float(ys[sel].mean()), "area": int(stats[k, cv2.CC_STAT_AREA]),
            "box": [int(v) for v in stats[k, :4]]}


def find_hand_diff(img, bg):
    """Lit room (v5): anything that changed against the empty scene. The arm comes in from the top of the picture and
    the hand hangs lowest, so take the lowest sizeable changed blob and use its lowest 15 px. Small blobs are the fairy
    lights flickering; a hand that reaches the masked lamp strip at the bottom is cut off, so it counts as not found."""
    d = cv2.absdiff(cv2.GaussianBlur(img, (5, 5), 0), cv2.GaussianBlur(bg, (5, 5), 0)).max(axis=2)
    m = (d > 12).astype(np.uint8)                # the white hand on the white ceiling only differs by ~15
    def yellow(im):                              # the fairy lights are bright and yellow; the printed hand is white
        b, g_, r = (im[..., i].astype(int) for i in range(3))
        return (r > 150) & (r - b > 45)
    lights = yellow(bg) | yellow(img)
    m[cv2.dilate(lights.astype(np.uint8), np.ones((25, 25), np.uint8)) > 0] = 0
    cut = int(m.shape[0] * 0.92)                 # the ceiling lamp at the bottom edge
    m[cut:] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))        # noise and the sensor's dotted rows
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    blobs = [k for k in range(1, n) if stats[k, cv2.CC_STAT_AREA] >= 1200]
    if not blobs:
        return None
    k = max(blobs, key=lambda k: stats[k, cv2.CC_STAT_TOP] + stats[k, cv2.CC_STAT_HEIGHT])
    if stats[k, cv2.CC_STAT_TOP] + stats[k, cv2.CC_STAT_HEIGHT] >= cut - 2:
        return None
    ys, xs = np.nonzero(lab == k)
    sel = ys > ys.max() - 15
    return {"x": float(xs[sel].mean()), "y": float(ys[sel].mean()), "area": int(stats[k, cv2.CC_STAT_AREA]),
            "box": [int(v) for v in stats[k, :4]]}


def find_hand_red(img, bg=None):
    """The red label on the underside of the hand. With the white balance locked on the greenish ceiling, the label
    reads R ~140, G ~25, B ~60 up close and purplish far away; the servos and wires are dark, the ceiling has G > R.
    So: clearly more red than green, and not dark; the warm lamp is masked. Centre of the biggest such patch."""
    im = cv2.GaussianBlur(img, (5, 5), 0).astype(int)
    b, g, r = im[..., 0], im[..., 1], im[..., 2]
    m = ((r - g > 25) & (r > 70)).astype(np.uint8)
    if bg is not None:                           # the warm ceiling lamp at the bottom edge
        lamp = cv2.cvtColor(bg, cv2.COLOR_BGR2GRAY) > 200
        lamp[:int(lamp.shape[0] * 0.8)] = False      # (the top rows can be bright too, and the hand is often there)
        m[cv2.dilate(lamp.astype(np.uint8), np.ones((41, 41), np.uint8)) > 0] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))     # one label, even when partly purple
    n, lab, stats, cent = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[k, cv2.CC_STAT_AREA] < 40:
        return None
    return {"x": float(cent[k][0]), "y": float(cent[k][1]), "area": int(stats[k, cv2.CC_STAT_AREA]),
            "box": [int(v) for v in stats[k, :4]]}


if TAG == "_wide":
    find_hand = find_hand_bright
elif TAG == "_v5":
    find_hand = find_hand_diff
elif TAG.startswith("_v5red"):
    find_hand = find_hand_red


def cam_set(var, val):
    import urllib.request
    urllib.request.urlopen(f"http://{CAM}/control?var={var}&val={val}", timeout=5).close()


def main():
    os.makedirs(OUT, exist_ok=True)
    cam_setup()
    send("V 25")
    send("M " + " ".join(map(str, BG_POSE or HOME)))
    time.sleep(1.5)
    if TAG.startswith("_v5"):
        for var in ("aec", "agc", "awb"):
            cam_set(var, 0)
        time.sleep(0.5)
    # lock exposure, gain and white balance on the empty scene: otherwise they re-adjust when the white arm enters
    # the picture and the whole frame differs from the background
    bg = grab(f"{OUT}/cal{TAG}_bg.jpg")
    rows = []
    poses = list(EXT_POSES) if TAG in ("_v5redext", "_v5redall") else []
    for e, shoulders in (LAYERS or []):
        for i, s in enumerate(shoulders):
            for p in (PANS if i % 2 == 0 else PANS[::-1]):
                poses.append((p, s, e))
    try:
        run(poses, bg, rows)
    finally:
        send("M " + " ".join(map(str, HOME)))
        for var in ("aec", "agc", "awb", "awb_gain"):
            cam_set(var, 1)
    report(bg, rows)


def run(poses, bg, rows):
    for p, s, e in poses:
        r = send(f"M {p} {s} {e} {HOME[3]} {HOME[4]}")
        if not r.startswith("DONE"):
            print("arm error:", r); break
        time.sleep(0.6)                           # let the arm stop swinging
        path = f"{OUT}/cal{TAG}_p{p}_s{s}_e{e}.jpg"
        img = grab(path)
        h = find_hand(img, bg)
        rows.append({"pan": p, "shoulder": s, "elbow": e, "hand": h, "file": path})
        print(p, s, e, "->", None if h is None else (round(h["x"]), round(h["y"])), flush=True)


def report(bg, rows):
    hgt, wid = bg.shape[:2]
    json.dump({"frame": [wid, hgt], "home": HOME, "poses": rows}, open(f"{OUT}/calib{TAG}.json", "w"), indent=1)
    tiles = []
    for r in rows:
        im = cv2.imread(r["file"])
        if r["hand"]:
            x, y = int(r["hand"]["x"]), int(r["hand"]["y"])
            cv2.circle(im, (x, y), 9, (0, 0, 255), 2)
            bx, by, bw, bh = r["hand"]["box"]
            cv2.rectangle(im, (bx, by), (bx + bw, by + bh), (0, 200, 255), 1)
        cv2.putText(im, f"p{r['pan']} s{r['shoulder']} e{r['elbow']}", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
        cv2.putText(im, f"p{r['pan']} s{r['shoulder']} e{r['elbow']}", (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        tiles.append(cv2.resize(im, (wid // 2, hgt // 2)))
    while len(tiles) % 7:
        tiles.append(np.zeros_like(tiles[0]))
    sheet = np.vstack([np.hstack(tiles[i:i + 7]) for i in range(0, len(tiles), 7)])
    cv2.imwrite(f"{OUT}/calib{TAG}_sheet.jpg", sheet)
    print("done", len(rows), "poses")


def redetect():
    """Re-run detection on the saved frames (no arm motion)."""
    data = json.load(open(f"{OUT}/calib{TAG}.json"))
    bg = cv2.imread(f"{OUT}/cal{TAG}_bg.jpg")
    for r in data["poses"]:
        r["hand"] = find_hand(cv2.imread(r["file"]), bg)
        h = r["hand"]
        print(r["pan"], r["shoulder"], r["elbow"], "->", None if h is None else (round(h["x"]), round(h["y"])))
    report(bg, data["poses"])


if __name__ == "__main__":
    redetect() if "--redetect" in sys.argv else main()
