"""Fit tools/cam_model.py's polynomial to hand detections from cam_calib.py runs, dropping outliers (wrong blobs).
.venv/bin/python tools/cam_fit.py calibration/calib_v5redall.json [more.json ...]
A full degree-3 polynomial in (p, s, e), at most linear in e. Drops the worst point while it is > 5 x the median error,
reports the leave-one-out error (the honest number), prints the coefficients for cam_model.py and writes
calibration/calib_points.json (the points kept)."""
import itertools, json, sys
import numpy as np

DEG = 3
TERMS = [(i, j, k) for i, j, k in itertools.product(range(DEG + 1), repeat=3) if i + j + k <= DEG and k <= 1]


def design(p, s, e):
    return np.array([p ** i * s ** j * e ** k for i, j, k in TERMS]).T


def load(paths):
    rows = []
    for path in paths:
        for r in json.load(open(path))["poses"]:
            if r["hand"]:
                rows.append((r["pan"], r["shoulder"], r["elbow"], r["hand"]["x"], r["hand"]["y"], r["file"]))
    return rows


def solve(X, a, keep):
    return (np.linalg.lstsq(X[keep], a[keep, 3], rcond=None)[0], np.linalg.lstsq(X[keep], a[keep, 4], rcond=None)[0])


if __name__ == "__main__":
    rows = load(sys.argv[1:])
    a = np.array([r[:5] for r in rows], float)
    X = design((a[:, 0] - 1500) / 100, (a[:, 1] - 1900) / 100, (a[:, 2] - 1450) / 100)
    keep = np.ones(len(a), bool)
    while True:
        cx, cy = solve(X, a, keep)
        r = np.hypot(X @ cx - a[:, 3], X @ cy - a[:, 4])
        w = int(np.argmax(np.where(keep, r, -1)))
        if r[w] <= max(5 * np.median(r[keep]), 10):
            break
        keep[w] = False
    loo = []
    for i in np.nonzero(keep)[0]:
        m = keep.copy(); m[i] = False
        lx, ly = solve(X, a, m)
        loo.append((X[i] @ lx - a[i, 3], X[i] @ ly - a[i, 4]))
    loo = np.array(loo)
    print(f"{keep.sum()} of {len(a)} points kept; leave-one-out rms {np.sqrt(np.mean(loo[:, 0] ** 2)):.1f} px across, "
          f"{np.sqrt(np.mean(loo[:, 1] ** 2)):.1f} px down, worst {np.hypot(*loo.T).max():.0f}")
    for i in np.nonzero(~keep)[0]:
        print("dropped", tuple(int(v) for v in a[i, :3]), "at", tuple(round(v) for v in a[i, 3:5]), f"off by {r[i]:.0f} px")
    print("TERMS =", TERMS)
    print("AX =", [round(float(v), 5) for v in cx])
    print("AY =", [round(float(v), 5) for v in cy])
    json.dump([{"pan": r_[0], "shoulder": r_[1], "elbow": r_[2], "x": r_[3], "y": r_[4], "file": r_[5]}
               for r_, k in zip(rows, keep) if k], open("calibration/calib_points.json", "w"), indent=1)
