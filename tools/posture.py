"""High-five posture: how far to push the elbow and fold the wrist (and turn the twist) after aiming at a palm, so the
robot's palm stands up facing the person's, learned from the pairs recorded on the voice page (calibration/
wrist_pairs.jsonl: the arm pose the person settled on + their palm as seen before the robot's hand covered it).
Aiming (pan, shoulder) comes from cam_model.pose_for at elbow 1450 / wrist 1580 / twist 1500; the pairs show the
person only changes the elbow, wrist and twist after that, so those offsets are what is learned.
Few pairs -> mostly their average; more pairs -> a ridge fit on the palm's position, size, tilt and turn.
.venv/bin/python tools/posture.py   prints the pairs, the fit and its leave-one-out error."""
import json, os
import numpy as np
from cam_model import pose_for

PAIRS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "calibration", "wrist_pairs.jsonl")
BASE = {"elbow": 1450, "wrist": 1580, "twist": 1500}
FEATS = ("y", "tilt", "turn")          # height + palm angles; x and size made the leave-one-out error worse (14 pairs)
SCALE = {"x": 100.0, "y": 100.0, "size": 15.0, "tilt": 15.0, "turn": 15.0}   # rough spread of each, for the ridge
RIDGE = 3.0              # pull towards the plain average; with 2-3 pairs the fit is nearly just the average
LIMIT = (700, 2300)
ODD_SHOULDER = 150       # a pair that moved the shoulder this far reached the palm a different way (shoulder up, elbow
                         # back: one far bottom-left corner pair did +300 / +77); mixing it in spoils the elbow fit
JOINTS = ("shoulder", "elbow", "wrist", "twist")   # shoulder: the "up"/"down" voice commands (contact along the palm)


def load():
    """Recorded pairs as (palm, offsets), duplicates (the same palm recorded twice) dropped."""
    out, seen = [], set()
    if not os.path.exists(PAIRS):
        return out
    for line in open(PAIRS):
        r = json.loads(line)
        p = r.get("palm")
        if not p or not r.get("palm_locked"):
            continue
        key = (round(p["t"], 3), tuple(r["arm"]))
        if key in seen:
            continue
        seen.add(key)
        tgt = pose_for(p["x"], p["y"])
        if tgt is None or abs(r["arm"][1] - tgt[1]) > ODD_SHOULDER:
            continue
        out.append((p, {"shoulder": r["arm"][1] - tgt[1], "elbow": r["arm"][2] - tgt[2], "wrist": r["arm"][3] - BASE["wrist"],
                        "twist": r["arm"][4] - BASE["twist"]}))
    return out


def fit(pairs):
    """Per joint: offset = mean + w . (features - their mean) / scale, ridge-regularised."""
    if not pairs:
        return None
    F = np.array([[p[f] / SCALE[f] for f in FEATS] for p, _ in pairs])
    mu = F.mean(axis=0)
    Fc = F - mu
    model = {"mu": mu}
    for j in JOINTS:
        y = np.array([o[j] for _, o in pairs], float)
        w = np.linalg.solve(Fc.T @ Fc + RIDGE * np.eye(len(FEATS)), Fc.T @ (y - y.mean()))
        model[j] = (float(y.mean()), w)
    return model


def offsets(model, palm):
    f = np.array([palm[k] / SCALE[k] for k in FEATS]) - model["mu"]
    return {j: model[j][0] + float(model[j][1] @ f) for j in JOINTS}


def predict(palm, target):
    """Full high-five pose (pan, shoulder, elbow, wrist, twist) for a palm, given the aim target from pose_for.
    Re-reads the pairs every time, so a pair recorded a moment ago already counts. No pairs -> the plain aim pose."""
    model = fit(load())
    pan, sh, el = target
    if model is None:
        return (pan, sh, el, BASE["wrist"], BASE["twist"]), None
    o = offsets(model, palm)
    clamp = lambda v: int(round(min(max(v, LIMIT[0]), LIMIT[1])))
    return (pan, clamp(sh + o["shoulder"]), clamp(el + o["elbow"]), clamp(BASE["wrist"] + o["wrist"]), clamp(BASE["twist"] + o["twist"])), o


if __name__ == "__main__":
    pairs = load()
    for p, o in pairs:
        print(f"palm x {p['x']:.0f} y {p['y']:.0f} size {p['size']:.0f} tilt {p['tilt']:+.0f} turn {p['turn']:+.0f}"
              f"  ->  shoulder {o['shoulder']:+.0f}  elbow {o['elbow']:+.0f}  wrist {o['wrist']:+.0f}  twist {o['twist']:+.0f}")
    print(len(pairs), "pairs")
    if len(pairs) >= 3:
        errs = []
        for i in range(len(pairs)):
            m = fit(pairs[:i] + pairs[i + 1:])
            o = offsets(m, pairs[i][0])
            errs.append([o[j] - pairs[i][1][j] for j in JOINTS])
        e = np.abs(np.array(errs))
        print("leave-one-out mean error: shoulder %.0f  elbow %.0f  wrist %.0f  twist %.0f" % tuple(e.mean(axis=0)))
