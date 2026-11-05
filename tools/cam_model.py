"""Camera <-> arm model from the robot-hand calibration (tools/cam_calib.py --v5 --red --all, 2026-10-26): wide-angle
OV2640 on the 30 deg wedge (looks 60 deg up), a red label on the underside of the hand lit from below, fairy lights off.
Upright 480 x 640 frames (VGA), correct (unmirrored) orientation: see cam_snap.cam_setup(). Wrist 1580, twist 1500.
Units: p = (pan - 1500) / 100, s = (shoulder - 1900) / 100, e = (elbow - 1450) / 100.
    x, y = sum of A[n] * p^i s^j e^k over TERMS[n] = (i, j, k): a full cubic, at most linear in e (tools/cam_fit.py)
One run of 268 poses: pan 1100..1900, shoulder 1600..2100 at elbow 1450, plus elbow 1600 at shoulder 1800/1950/2100.
225 of 261 points kept (the rest: label cut off at the picture edge); leave-one-out rms 5.4 px across, 3.1 px down, worst 19
Higher pan -> hand further LEFT; higher shoulder/elbow -> lower.
REACH: the measured pan range per shoulder (elbow 1450): the model is only trusted inside it (the high-shoulder rows
lose the label at the sides). Points: calibration/calib_points.json."""
import sys

TERMS = [(0, 0, 0), (0, 0, 1), (0, 1, 0), (0, 1, 1), (0, 2, 0), (0, 2, 1), (0, 3, 0), (1, 0, 0), (1, 0, 1), (1, 1, 0), (1, 1, 1), (1, 2, 0), (2, 0, 0), (2, 0, 1), (2, 1, 0), (3, 0, 0)]
AX = (247.68425, -9.55795, -1.11598, 9.87552, -1.29268, 18.75323, -0.05591, -67.92320, -6.42090, -10.69660, 1.17514,
      1.06406, -0.37115, -0.19937, -0.01902, 0.49462)
AY = (377.73237, 61.46714, 98.77627, -13.80588, 1.93716, -23.95552, -0.85486, 1.24057, -0.93023, -0.55899, -0.71420,
      -0.34802, -4.36004, -0.74993, -1.01429, -0.06184)
REACH = {1600: (1100, 1900), 1650: (1100, 1900), 1700: (1100, 1900), 1750: (1100, 1900), 1800: (1100, 1900), 1850: (1100, 1900), 1900: (1100, 1900), 1950: (1125, 1875), 2000: (1175, 1775), 2050: (1200, 1725), 2100: (1425, 1625)}
LIMITS = {"pan": (1100, 1900), "shoulder": (1600, 2100), "elbow": (1450, 1600)}


def pixel(pan, shoulder, elbow=1450):
    p, s, e = (pan - 1500) / 100, (shoulder - 1900) / 100, (elbow - 1450) / 100
    t = [p ** i * s ** j * e ** k for i, j, k in TERMS]
    return sum(a * v for a, v in zip(AX, t)), sum(a * v for a, v in zip(AY, t))


def pan_range(shoulder):
    """The measured pan range at this shoulder (interpolated between rows), or None outside the shoulder range."""
    ss = sorted(REACH)
    if not ss[0] <= shoulder <= ss[-1]:
        return None
    for a, b in zip(ss, ss[1:]):
        if a <= shoulder <= b:
            f = (shoulder - a) / (b - a)
            return (REACH[a][0] + f * (REACH[b][0] - REACH[a][0]), REACH[a][1] + f * (REACH[b][1] - REACH[a][1]))
    return REACH[ss[-1]]


def in_reach(pan, shoulder):
    r = pan_range(shoulder)
    return r is not None and r[0] <= pan <= r[1]


def reach_edge(step=25):
    """(pan, shoulder) points around the edge of the measured area, for drawing it."""
    ss = sorted(REACH)
    return [(REACH[ss[0]][0] + k * step, ss[0]) for k in range(int((REACH[ss[0]][1] - REACH[ss[0]][0]) / step) + 1)] + \
        [(REACH[s][1], s) for s in ss] + [(REACH[s][0], s) for s in reversed(ss)]


def pose_for(x, y, elbow=1450):
    """(pan, shoulder, elbow) that puts the hand at picture pixel (x, y), elbow fixed (Newton on the model).
    None when that needs a pose outside the measured area (REACH)."""
    pan, sh = 1500.0, 1950.0
    for _ in range(20):
        x0, y0 = pixel(pan, sh, elbow)
        dx = [(pixel(pan + 1, sh, elbow)[i] - (x0, y0)[i]) for i in (0, 1)]
        ds = [(pixel(pan, sh + 1, elbow)[i] - (x0, y0)[i]) for i in (0, 1)]
        det = dx[0] * ds[1] - ds[0] * dx[1]
        if abs(det) < 1e-9:
            return None
        rx, ry = x - x0, y - y0
        pan += (rx * ds[1] - ry * ds[0]) / det
        sh += (dx[0] * ry - dx[1] * rx) / det
        if abs(rx) < 0.2 and abs(ry) < 0.2:
            break
    pan, sh = round(pan), round(sh)
    if not in_reach(pan, sh):
        return None
    return pan, sh, elbow


if __name__ == "__main__":
    print(pose_for(float(sys.argv[1]), float(sys.argv[2])))
