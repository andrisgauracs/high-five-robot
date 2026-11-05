"""High-five robot. Watches the robot camera, finds an open raised palm with MediaPipe, aims the arm at it with the
calibrated camera model (tools/cam_model.py) and high-fives it. tools/arm.py finds the arm on Wi-Fi or USB.
    .venv/bin/python tools/highfive.py [camera-ip]      ->  http://localhost:8091
The page shows the upright camera picture with the hand drawn on it, the reachable zone (green outline) and the palm
size (a distance cue). The arm only moves after ARM ON on the page; it starts OFF (watch only).
A high-five needs: an open raised palm, inside the zone, palm size MIN_PALM..MAX_PALM px, held still for STEADY_S."""
import os, socket, sys, time, threading, urllib.request
from collections import deque
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import numpy as np, cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks
from mediapipe.tasks.python import vision

sys.path.insert(0, os.path.dirname(__file__))
from cam_snap import upright, cam_setup
from cam_model import pixel, pose_for, reach_edge
from arm import send
from posture import predict

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
AUTO = "--armed" in sys.argv     # start with the arm ON (the launchd service: power the robot on and it high-fives)
CAMS = ARGS[:1] or ["highfive-cam.local"]   # firmware/camera_highfive announces the name; pass an IP to override
CAM = CAMS[-1]
MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models", "hand_landmarker.task")
STEADY_S = 0.25         # the palm must stay put this long (and be seen in at least 2 frames): near-instant
STEADY_PX = 18          # ... within this many pixels
MIN_PALM, MAX_PALM = 42, 100   # wrist -> middle-finger base in px: your natural high-five distance measured 42..63
HOME = (1500, 1405, 1450, 1580, 1500)
WRIST, TWIST = 1580, 1500      # the camera model was calibrated with these
SLAP_US = 110                  # slap this far past the touching posture (elbow, higher pulse = towards your palm), ~3 cm
FAST = 360                     # deg/s, firmware/arm_wifi's top speed (in practice the servos' own limit)
COOLDOWN_S = 2.5
# live-tunable from the page's sliders, kept in highfive_tune.json across restarts (defaults: the constants above)
TUNE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "highfive_tune.json")
TUNE_RANGE = {"steady": (0.0, 1.5), "speed": (30, 360), "slap": (0, 250)}
TUNE = {"steady": STEADY_S, "speed": FAST, "slap": SLAP_US}
try:
    TUNE.update({k: v for k, v in __import__("json").load(open(TUNE_FILE)).items() if k in TUNE})
except (OSError, ValueError):
    pass

state = {"armed": AUTO, "arm_ok": False, "busy": False, "last": 0.0, "count": 0,
         "hold": False, "holding": False, "held": None}   # hold mode: reach the palm and stay (wrist calibration)


def high_five(pan, shoulder, elbow, palm=None):
    """Reach to the palm, slap, come back home. Runs in its own thread; tracking keeps going.
    In hold mode: reach to the palm and stay there (no slap), remembering the palm as seen before the hand covered it;
    /release (or "home" on the voice page) ends the hold."""
    try:
        if state["hold"]:
            # arrive in the learned high-five posture (tools/posture.py: elbow pushed forward, wrist folded back so
            # the palm stands up), as the starting guess the person then corrects on the voice page
            pose, off = predict(palm, (pan, shoulder, elbow))
            send("V 50")
            send("M " + " ".join(map(str, pose)))
            state["holding"], state["held"] = True, dict(palm, guess=list(pose))
            print(f"HOLDING at {pose} (guess offsets {off}; palm {palm})", flush=True)
            return
        # the learned posture is where the robot's palm touches yours; go straight through it to SLAP_US past it
        # ("closer" = higher elbow pulse) in one move, and straight back home, both at the firmware's top speed
        (pan, shoulder, elbow, wrist, twist), off = predict(palm, (pan, shoulder, elbow))
        print(f"HIGH-FIVE posture {(pan, shoulder, elbow, wrist, twist)} (offsets {off})", flush=True)
        send(f"V {TUNE['speed']:.0f}")
        send(f"M {pan} {shoulder} {min(elbow + int(TUNE['slap']), 2300)} {wrist} {twist}")
        send("M " + " ".join(map(str, HOME)))
        state["count"] += 1
        print(f"HIGH-FIVE #{state['count']} at pan {pan} shoulder {shoulder}", flush=True)
    except Exception as e:
        print("arm error:", e, flush=True)
        state["arm_ok"] = False          # the arm watchdog brings it back once the arm answers again
        if not AUTO:
            state["armed"] = False
    finally:
        state["last"] = time.time()
        state["busy"] = False
KNUCKLES = [(8, 5), (12, 9), (16, 13), (20, 17)]         # (tip, knuckle) per finger, MediaPipe hand landmark ids
REACH_RATIO = 1.2        # fingertip this much further from the wrist than its knuckle = finger not curled in
PALM = [0, 5, 9, 13, 17]                                  # wrist + finger bases: the palm centre
EDGES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
         (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]

latest_jpeg = None
lock = threading.Lock()


STALL = 4                # seconds without a whole frame -> drop the connection and reconnect


def find_camera():
    """The camera's address: its mDNS name looked up once per connection (looking it up per request is slow), else
    the fallback IP. Then put it in the expected picture state, since a camera that rebooted has lost it."""
    global CAM
    for c in CAMS:
        try:
            ip = socket.gethostbyname(c)
            urllib.request.urlopen(f"http://{ip}/status", timeout=3).close()
            if ip != CAM:
                print(f"camera: {c} at {ip}", flush=True)
            CAM = ip
            cam_setup(CAM)
            return True
        except Exception:
            continue
    return False


def arm_watchdog():
    """Ask the arm how it is every few seconds while idle, so a high five is only tried when it answers (it may be
    switched off, or still joining Wi-Fi after power-up)."""
    while True:
        if not state["busy"] and not state["holding"]:
            try:
                ok = send("?").startswith("AT")
            except Exception:
                ok = False
            if ok != state["arm_ok"]:
                print("arm:", "connected" if ok else "not answering", flush=True)
            state["arm_ok"] = ok
        time.sleep(3)


def frames():
    """Raw JPEG frames from the camera's MJPEG stream (port 81), reconnecting if it drops or stalls.
    After a Wi-Fi blip the socket can stay open with nothing (or junk) arriving, so a watchdog thread closes it
    when no whole frame has come for STALL seconds; the blocked read then fails and we reconnect."""
    while True:
        s = None
        try:
            if not find_camera():
                raise IOError("camera not found (" + " / ".join(CAMS) + ")")
            s = urllib.request.urlopen(f"http://{CAM}:81/stream", timeout=10)
            last = [time.time()]
            conn = s

            def watchdog():
                while conn is s and not conn.closed:
                    if time.time() - last[0] > STALL:
                        print(f"camera stream: no frame for {STALL}s - reconnecting", flush=True)
                        try:
                            conn.fp.raw._sock.shutdown(2)
                        except Exception:
                            pass
                        conn.close()
                        return
                    time.sleep(0.5)
            threading.Thread(target=watchdog, daemon=True).start()
            buf = b""
            while True:
                chunk = s.read1(65536)
                if not chunk:
                    raise IOError("stream ended")
                buf += chunk
                a = buf.find(b"\xff\xd8")
                b = buf.find(b"\xff\xd9", a + 2)
                if a >= 0 and b >= 0:
                    last[0] = time.time()
                    yield buf[a:b + 2]
                    buf = buf[b + 2:]
                elif len(buf) > 2_000_000:
                    buf = buf[-200_000:]
        except Exception as e:
            if s is None or not s.closed:          # a stall was already reported by the watchdog
                print("camera stream:", e, "- retrying", flush=True)
            if s is not None:
                try:
                    s.close()
                except Exception:
                    pass
            time.sleep(1)


def reach_outline(w, h):
    """The picture region the arm can reach: the model's pixel() around the edge of the measured area."""
    return np.array([pixel(p, s) for p, s in reach_edge()], np.int32)


def palm_angles(world, handed):
    """Which way the palm faces, from MediaPipe's 3D world landmarks (metres, x right, y down, z away from the camera).
    Returns (tilt, turn) in degrees: tilt > 0 = palm tilted to face upwards, turn > 0 = palm turned to face the picture's
    right; (0, 0) = palm square to the camera. The palm normal comes from the wrist and the index/pinky bases; its sign
    depends on which hand it is (MediaPipe's label assumes a mirrored selfie view, so it is swapped here)."""
    P = lambda i: np.array([world[i].x, world[i].y, world[i].z])
    n = np.cross(P(5) - P(0), P(17) - P(0))
    if handed == "Left":                          # MediaPipe "Left" = the person's right hand in an unmirrored view
        n = -n
    n /= np.linalg.norm(n) + 1e-9
    if n[2] > 0:                                  # we only ever see a palm facing us; flip a back-of-hand estimate
        n = -n
    tilt = float(np.degrees(np.arctan2(-n[1], -n[2])))
    turn = float(np.degrees(np.arctan2(n[0], -n[2])))
    return tilt, turn


def open_palm(lm, world, w, h):
    """An open hand (not a fist), pointing up. The fingers may be bent or cupped, as in a relaxed high five; what counts is
    that at least 3 fingertips reach well past their knuckles: tip-to-wrist > REACH_RATIO x knuckle-to-wrist, in MediaPipe's
    3D world landmarks (a palm tilted far back points its fingers at the camera, so in the picture they look short).
    Straight finger ~1.9, cupped ~1.3-1.6, fist ~1.0 or less. Returns (ok, why-not)."""
    P = lambda i: np.array([world[i].x, world[i].y, world[i].z])
    ratios = [np.linalg.norm(P(t) - P(0)) / (np.linalg.norm(P(m) - P(0)) + 1e-9) for t, m in KNUCKLES]
    out = sum(r > REACH_RATIO for r in ratios)
    up = lm[12].y < lm[0].y or lm[9].y < lm[0].y or P(9)[1] < P(0)[1]     # fingers/knuckles above the wrist (y down)
    if out < 3:
        return False, f"looks like a fist: {out}/4 fingertips out ({' '.join(f'{r:.2f}' for r in ratios)})"
    if not up:
        return False, "fingers pointing down"
    return True, ""

def track():
    global latest_jpeg
    track.last_log = 0.0
    track.angles = None
    track.last_ts = 0
    track.palm = None
    track.last_ang = 0.0
    opts = vision.HandLandmarkerOptions(base_options=mp_tasks.BaseOptions(model_asset_path=MODEL),
                                        running_mode=vision.RunningMode.VIDEO, num_hands=2,
                                        min_hand_detection_confidence=0.5, min_tracking_confidence=0.5)
    det = vision.HandLandmarker.create_from_options(opts)
    hist = deque()
    outline, t_start, fps = None, time.time(), 0.0
    for raw in frames():
        try:
            img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            img = upright(img)
            h, w = img.shape[:2]
            if outline is None or outline.shape[0] == 0:
                outline = reach_outline(w, h)
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(1e-3, now - t_start)
            t_start = now
            ts = max(int(now * 1000), track.last_ts + 1)          # MediaPipe needs strictly increasing timestamps
            track.last_ts = ts
            res = det.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)), ts)
            cv2.polylines(img, [outline], True, (80, 200, 120), 1)
            status, colour, target = "no hand", (160, 160, 160), None
            best = None
            track.why = ""
            for k, lm in enumerate(res.hand_landmarks):
                pts = np.array([[p.x * w, p.y * h] for p in lm])
                palm = pts[PALM].mean(axis=0)
                size = float(np.linalg.norm(pts[0] - pts[9]))           # wrist -> middle-finger base, in pixels
                ok, why = open_palm(lm, res.hand_world_landmarks[k], w, h)
                if not ok:
                    track.why = why
                for a, b in EDGES:
                    cv2.line(img, tuple(pts[a].astype(int)), tuple(pts[b].astype(int)), (255, 200, 80) if ok else (120, 120, 120), 1)
                if ok and (best is None or size > best[1]):
                    ang = palm_angles(res.hand_world_landmarks[k], res.handedness[k][0].category_name)
                    best = (palm, size, ang)
            if best is None:
                hist.clear()
                if res.hand_landmarks:
                    status = f"hand seen, not an open raised palm ({track.why})"
                    if now - track.last_ang > 1.0:
                        print("REJECTED hand:", track.why, flush=True)
                        track.last_ang = now
            else:
                palm, size, ang = best
                hist.append((now, palm))
                while hist and now - hist[0][0] > TUNE["steady"]:
                    hist.popleft()
                spread = max(np.linalg.norm(p - palm) for _, p in hist)
                steady = len(hist) >= 2 and now - hist[0][0] >= TUNE["steady"] * 0.9 and spread < STEADY_PX
                target = pose_for(*palm)
                if now - track.last_ang > 1.0:
                    print(f"PALM {palm[0]:.0f},{palm[1]:.0f} size {size:.0f} tilt {ang[0]:+.0f} turn {ang[1]:+.0f}", flush=True)
                    track.last_ang = now
                if state["holding"]:
                    status, colour = "HOLDING at your palm: match the wrist on the voice page, then say home", (80, 220, 80)
                    hist.clear()
                elif state["busy"]:
                    status, colour = "HIGH-FIVE!", (80, 220, 80)
                    hist.clear()
                elif target is None:
                    status, colour = "open palm, OUT OF REACH (raise it / move it towards the centre)", (60, 60, 230)
                elif size < MIN_PALM:
                    status, colour = "open palm, too far: come closer", (60, 200, 230)
                elif size > MAX_PALM:
                    status, colour = "open palm, too close: step back", (60, 60, 230)
                elif not steady:
                    status, colour = "open palm, hold still...", (60, 200, 230)
                elif now - state["last"] < COOLDOWN_S:
                    status, colour = "cooling down...", (60, 200, 230)
                elif state["armed"] and not state["arm_ok"]:
                    status, colour = "READY, but the arm is not answering", (60, 200, 230)
                elif state["armed"]:
                    status, colour = "HIGH-FIVE!", (80, 220, 80)
                    state["busy"] = True
                    hist.clear()
                    seen = {"t": now, "x": float(palm[0]), "y": float(palm[1]), "size": size, "tilt": ang[0], "turn": ang[1]}
                    threading.Thread(target=high_five, args=(*target, seen), daemon=True).start()
                else:
                    status, colour = "READY: would high-five (arm OFF)", (80, 220, 80)
                    if now - track.last_log > 1.0:
                        print(f"READY palm {palm[0]:.0f},{palm[1]:.0f} size {size:.0f}px tilt {ang[0]:+.0f} turn {ang[1]:+.0f} -> {target}", flush=True)
                        track.last_log = now
                cv2.circle(img, tuple(palm.astype(int)), 8, colour, 2)
                cv2.putText(img, f"palm {size:.0f}px", (int(palm[0]) + 12, int(palm[1]) + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)
                cv2.putText(img, f"tilt {ang[0]:+.0f}  turn {ang[1]:+.0f}", (int(palm[0]) + 12, int(palm[1]) + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)
                track.angles = ang
                track.palm = {"t": now, "x": float(palm[0]), "y": float(palm[1]), "size": size, "tilt": ang[0], "turn": ang[1]}
            lines = [status, f"arm would go: pan {target[0]} shoulder {target[1]} elbow {target[2]}" if target else "",
                     f"{fps:.1f} fps   arm {'ON' if state['armed'] else 'OFF (watch only)'}{' HOLD' if state['hold'] else ''}   high-fives {state['count']}"]
            y = h - 52
            cv2.rectangle(img, (0, y - 16), (w, h), (0, 0, 0), -1)
            for t in lines:
                cv2.putText(img, t, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour if t == status else (230, 230, 230), 1, cv2.LINE_AA)
                y += 18
            ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with lock:
                latest_jpeg = jpg.tobytes()

        except Exception as e:                                 # one bad frame must not stop the tracker
            print("tracker frame error:", repr(e), flush=True)

PAGE = b"""<!doctype html><meta charset=utf-8><title>High-five robot</title>
<style>body{margin:0;background:#111;color:#ddd;font:15px system-ui;display:grid;place-items:center;min-height:100vh}
img{height:82vh;border-radius:8px}.bar{display:flex;gap:10px;align-items:center;margin:10px 0}
button{font:700 16px system-ui;border:0;border-radius:10px;padding:12px 20px;cursor:pointer}
#arm.on{background:#e5484d;color:#fff}#arm.off{background:#4cc38a;color:#000}
#tune{gap:22px;font-size:14px}#tune input{vertical-align:middle;width:150px}#tune b{display:inline-block;min-width:2.6em}</style>
<div><div class=bar><b>High-five robot</b><button id=arm class=off onclick="toggle()">ARM ON</button>
<button id=hold class=off onclick="hold()">HOLD mode</button><button id=rel class=off onclick="release()">RELEASE (home)</button>
<span id=st></span></div>
<div class=bar id=tune>
<label>React after <input type=range id=steady min=0 max=1.5 step=0.05> <b id=steady_v></b> s still</label>
<label>Speed <input type=range id=speed min=30 max=360 step=10> <b id=speed_v></b> &deg;/s</label>
<label>Slap <input type=range id=slap min=0 max=250 step=10> <b id=slap_v></b> &micro;s past</label></div>
<img src="/stream"></div>
<script>
let armed=false,holdOn=false;
function show(s){armed=s.armed;holdOn=s.hold;const b=document.getElementById('arm');b.className=armed?'on':'off';
 b.textContent=armed?'ARM OFF (stop high-fives)':'ARM ON';
 const h=document.getElementById('hold');h.className=holdOn?'on':'off';h.textContent=holdOn?'HOLD mode: ON':'HOLD mode: off';
 document.getElementById('st').textContent=!armed?'Watch only: the arm does not move.':s.holding?'Holding at your palm. Say home (or RELEASE) when done.':
  holdOn?'Hold mode: an open palm held still gets the arm, and it STAYS there.':'Arm is live: an open palm held still in the green zone gets a high-five.'}
async function toggle(){const r=await fetch('/armed?on='+(armed?0:1));show(await r.json())}
async function hold(){const r=await fetch('/hold?on='+(holdOn?0:1));show(await r.json())}
async function release(){const r=await fetch('/release');show(await r.json())}
setInterval(()=>fetch('/state').then(r=>r.json()).then(show),1000);
fetch('/state').then(r=>r.json()).then(show);
let tuneT;
function showTune(t){for(const k in t){document.getElementById(k).value=t[k];document.getElementById(k+'_v').textContent=t[k]}}
for(const k of ['steady','speed','slap']){const el=document.getElementById(k);el.oninput=()=>{
 document.getElementById(k+'_v').textContent=el.value;clearTimeout(tuneT);
 tuneT=setTimeout(()=>fetch('/tune?'+k+'='+el.value).then(r=>r.json()).then(showTune),150)}}
fetch('/tune').then(r=>r.json()).then(showTune);
</script>"""


class H(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200); self.send_header("Content-Type", "text/html"); self.end_headers()
            self.wfile.write(PAGE); return
        if self.path.startswith("/armed"):
            state["armed"] = self.path.endswith("on=1")
            print("ARM", "ON" if state["armed"] else "OFF", flush=True)
        if self.path.startswith("/hold"):
            state["hold"] = self.path.endswith("on=1")
            print("HOLD", "ON" if state["hold"] else "OFF", flush=True)
        if self.path.startswith("/release"):
            if state["holding"] and not self.path.endswith("move=0"):
                try:
                    send("V 30"); send("M " + " ".join(map(str, HOME)))
                except Exception as e:
                    print("arm error:", e, flush=True)
            state["holding"], state["held"], state["last"] = False, None, time.time()
        if self.path.startswith("/tune"):
            import json as _json
            from urllib.parse import urlparse, parse_qs
            changed = False
            for k, v in parse_qs(urlparse(self.path).query).items():
                if k in TUNE:
                    lo, hi = TUNE_RANGE[k]
                    TUNE[k] = min(hi, max(lo, float(v[0])))
                    changed = True
            if changed:
                _json.dump(TUNE, open(TUNE_FILE, "w"))
                print("TUNE", TUNE, flush=True)
            body = _json.dumps(TUNE).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
            self.wfile.write(body); return
        if self.path == "/held":
            import json as _json
            body = _json.dumps(state["held"] if state["holding"] else None).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
            self.wfile.write(body); return
        if self.path.startswith(("/armed", "/hold", "/release")) or self.path == "/state":
            import json as _json
            body = _json.dumps({k: state[k] for k in ("armed", "count", "hold", "holding")}).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
            self.wfile.write(body); return
        if self.path == "/palm":
            import json as _json
            body = _json.dumps(getattr(track, "palm", None)).encode()
            self.send_response(200); self.send_header("Content-Type", "application/json"); self.end_headers()
            self.wfile.write(body); return
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
            last = None
            try:
                while True:
                    with lock:
                        jpg = latest_jpeg
                    if jpg is not None and jpg is not last:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                        last = jpg
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                return
        self.send_error(404)

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    threading.Thread(target=track, daemon=True).start()
    threading.Thread(target=arm_watchdog, daemon=True).start()
    print(f"highfive: http://localhost:8091 (arm starts {'ON' if AUTO else 'OFF'})", flush=True)
    ThreadingHTTPServer(("127.0.0.1", 8091), H).serve_forever()
