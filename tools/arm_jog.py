"""Jog page for the arm: engage, nudge and release each joint, then save the pose as the boot home pose.
.venv/bin/python tools/arm_jog.py   ->  http://localhost:8090 (voice page: /voice)
--demo: everything works and looks the same, but "that's it" saves nothing (for filming a training session)."""
import json, re, sys, os
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
sys.path.insert(0, os.path.dirname(__file__))
from arm import send
from cam_model import pose_for

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEMO = "--demo" in sys.argv
CONFIGS = [os.path.join(ROOT, "firmware", d, "robot_config.h") for d in ("", "arm_wifi", "arm_serial")]
NAMES = ["Base (pan)", "Shoulder", "Elbow", "Wrist", "Twist"]
CENTRE = [1500, 1500, 1450, 1580, 1500]   # 1500 + JOINT_TRIM_US (robot_config.h)
JOINTS = [{"name": n, "pin": p, "min": lo, "max": 2500 if k == 0 else 2300, "centre": c}
          for k, (n, p, lo, c) in enumerate(zip(NAMES, [4, 5, 6, 7, 15], [500, 700, 700, 700, 700], CENTRE))]

PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Arm control</title>
<style>
:root{--bg:#121212;--card:#1d1d1d;--line:#333;--fg:#e8e8e8;--dim:#9a9a9a;--acc:#5aa9ff;--on:#4cc38a;--off:#c9a227;--stop:#e5484d}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:16px system-ui;padding:16px;max-width:900px}
h1{font-size:20px;margin:0}
.top{display:flex;gap:12px;align-items:center;justify-content:space-between;margin-bottom:12px;flex-wrap:wrap}
.stop{background:var(--stop);color:#fff;border:0;border-radius:10px;padding:14px 22px;font:700 18px system-ui;cursor:pointer}
p{color:var(--dim);margin:0 0 14px}
.row{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px;margin-bottom:10px}
.row.active{border-color:var(--acc)}
.head{display:flex;gap:10px;align-items:baseline;flex-wrap:wrap;margin-bottom:10px}
.name{font-weight:700;font-size:17px}.pin{color:var(--dim);font-size:13px}
.st{font-size:13px;padding:2px 8px;border-radius:99px}.st.on{background:#1f3a2c;color:var(--on)}.st.off{background:#3a331a;color:var(--off)}
.us{margin-left:auto;font:600 18px ui-monospace,monospace}
.ctl{display:flex;gap:6px;flex-wrap:wrap;align-items:center}
button{background:#2b2b2b;color:var(--fg);border:1px solid #3a3a3a;border-radius:8px;padding:9px 12px;font:inherit;cursor:pointer;min-width:44px}
button:hover{border-color:var(--acc)}button:disabled{opacity:.4;cursor:default}
input{width:84px;background:#111;color:var(--fg);border:1px solid #3a3a3a;border-radius:8px;padding:9px;font:inherit}
.sep{width:1px;height:28px;background:var(--line);margin:0 4px}
.save{background:var(--acc);color:#000;border:0;font-weight:700;padding:12px 18px}
#msg{margin:12px 0;color:var(--dim);min-height:1.4em}
</style>
<div class=top><h1>Arm control</h1><button class=stop onclick="stopAll()">STOP: all limp</button></div>
<p>Every joint starts <b>limp</b> (no signal, it stays where it is). Only the joint you press acts. <b>Go</b> sends that
joint to the pulse in its box. If the joint is limp, it jumps there, so keep clear. <b>Wiggle</b> moves it a little either
side and back, to check which servo it is. <b>STOP</b> releases every joint.</p>
<div id=rows></div>
<div id=msg></div>
<button class=save onclick="save()">Save holding joints as home</button>
<script>
const J=%JOINTS%;let st=null,busy=false,abort=false;
const $=id=>document.getElementById(id);
async function call(c){const r=await fetch('/cmd?c='+encodeURIComponent(c));const s=await r.json();if(s.us)st=s;render(s.raw);return s}
function render(raw){if(!st)return;$('msg').textContent=raw||'';
 $('rows').innerHTML=J.map((j,i)=>`<div class="row ${st.on[i]?'active':''}"><div class=head>
 <span class=name>${j.name}</span><span class=pin>GPIO${j.pin}</span>
 <span class="st ${st.on[i]?'on':'off'}">${st.on[i]?'holding':'limp'}</span><span class=us>${st.on[i]?st.us[i]+' µs':'no signal'}</span></div>
 <div class=ctl><input id=in${i} type=number min=${j.min} max=${j.max} step=10 value=${st.on[i]?st.us[i]:($('in'+i)?.value||j.centre)}>
 <button onclick="go(${i})">Go</button><span class=sep></span>
 ${[-100,-20,-5,5,20,100].map(d=>`<button ${st.on[i]?'':'disabled'} onclick="nudge(${i},${d})">${d>0?'+':''}${d}</button>`).join('')}
 <span class=sep></span><button onclick="wiggle(${i})">Wiggle</button><button onclick="call('L ${i}')">Limp</button></div></div>`).join('')}
async function run(f){if(busy)return;busy=true;abort=false;try{await f()}finally{busy=false}}
function go(i){const v=+$('in'+i).value;run(()=>call(`J ${i} ${v}`))}
function nudge(i,d){run(()=>call(`J ${i} ${st.us[i]+d}`))}
function wiggle(i){run(async()=>{const c=+$('in'+i).value;
 for(const v of [c,c+80,c-80,c+80,c-80,c]){if(abort)return;await call(`J ${i} ${v}`)}})}
async function stopAll(){abort=true;await call('L')}
async function save(){const r=await fetch('/save');$('msg').textContent=(await r.json()).msg}
call('V 20').then(()=>call('?'));
</script>"""


def parse(reply):
    m = re.match(r"\w+ ([\d ]+) \| on ([01 ]+)", reply)
    if not m:
        return {"raw": reply}
    return {"us": [int(v) for v in m[1].split()], "on": [v == "1" for v in m[2].split()], "raw": reply}


VOICE_PAGE = """<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Robot dashboard</title>
<style>
:root{--bg:#121212;--card:#1d1d1d;--line:#2c2c2c;--fg:#e8e8e8;--dim:#9a9a9a;--acc:#5aa9ff;--ok:#4cc38a;--warn:#e5484d;--amber:#d8a531}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:16px system-ui;padding:16px}
.wrap{display:grid;grid-template-columns:minmax(0,auto) minmax(300px,1fr);gap:16px;align-items:start}
@media (max-width:760px){.wrap{grid-template-columns:1fr}}
.cam img{height:calc(100vh - 32px);max-width:100%;border-radius:12px;display:block}
.side{display:flex;flex-direction:column;gap:12px}
h1{font-size:20px;margin:0}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
button{font:700 16px system-ui;border:0;border-radius:10px;padding:12px 18px;cursor:pointer;background:var(--acc);color:#000}
button.on{background:var(--warn);color:#fff}
.dot{width:12px;height:12px;border-radius:50%;background:#555;display:inline-block}
.dot.live{background:var(--ok);box-shadow:0 0 8px var(--ok)}.dot.err{background:var(--warn)}
#status{color:var(--dim);font-size:14px}
#pose{font:600 22px ui-monospace,monospace}
#palm{font:15px ui-monospace,monospace;color:var(--dim)}
#interim{color:var(--amber);min-height:1.3em;font-style:italic}
#log{max-height:38vh;overflow:auto;font-size:15px}
#log div{padding:3px 0;border-bottom:1px solid var(--line)}.heard{color:var(--dim)}.did{color:var(--ok)}.err{color:var(--warn)}
table{border-collapse:collapse;color:var(--dim);font-size:14px}td{padding:2px 10px 2px 0;vertical-align:top}b{color:var(--fg)}
</style>
<div class=wrap>
<div class=cam><img src="http://localhost:8091/stream" alt="camera (start tools/highfive.py)"></div>
<div class=side>
<div class=row><h1>Robot dashboard</h1></div>
<div class="card row"><button id=mic onclick="toggle()">Start listening</button><span class=dot id=dot></span><span id=status>microphone off</span></div>
<div class=card><div id=pose>-</div><div id=palm>your palm: -</div></div>
<div class=card><div id=interim></div><div id=log></div></div>
<div class=card><table>
<tr><td><b>towards me</b> / <b>away</b></td><td>tilt the robot's palm (wrist)</td></tr>
<tr><td><b>come here</b></td><td>lock your palm's angle, bring the robot's hand to it</td></tr>
<tr><td><b>left</b> / <b>right</b></td><td>turn the robot's palm (twist)</td></tr>
<tr><td><b>up</b> / <b>down</b></td><td>raise / lower the hand (elbow, step 30)</td></tr>
<tr><td><b>shoulder up</b> / <b>shoulder down</b></td><td>raise / lower the whole arm (shoulder, step 30)</td></tr>
<tr><td><b>closer</b> / <b>further</b></td><td>push the hand towards your palm / pull it back (elbow, step 30)</td></tr>
<tr><td>+ <b>bit</b> / <b>lot</b></td><td>step 20 / 120 (default 60, about 5 deg)</td></tr>
<tr><td><b>again</b></td><td>repeat the last move</td></tr>
<tr><td><b>that's it</b></td><td>record robot + your (locked) palm angles</td></tr>
<tr><td><b>home</b></td><td>arm back to the home pose</td></tr></table></div>
</div></div>
<script>
const R=window.SpeechRecognition||window.webkitSpeechRecognition;let rec=null,want=false,running=false;
const $=id=>document.getElementById(id);
function add(cls,t){const d=document.createElement('div');d.className=cls;d.textContent=new Date().toLocaleTimeString()+'  '+t;$('log').prepend(d)}
function stat(s,c){$('status').textContent=s;$('dot').className='dot '+(c||'')}
async function say(t){add('heard','heard: "'+t+'"');try{const r=await fetch('/voicecmd?c='+encodeURIComponent(t));const s=await r.json();
 add(s.did.startsWith('not a command')?'err':'did',s.did);if(s.pose)$('pose').textContent=s.pose}catch(e){add('err','server error: '+e)}}
// one phrase per session: Chrome's continuous mode stalls silently after a few results, so each session ends
// after one phrase (or silence) and onend starts the next one straight away
function make(){rec=new R();rec.lang='en-US';rec.continuous=false;rec.interimResults=true;
 rec.onstart=()=>{running=true;stat('listening','live')};
 rec.onresult=e=>{let interim='';for(let i=e.resultIndex;i<e.results.length;i++){const r=e.results[i];
   if(r.isFinal)say(r[0].transcript.trim());else interim+=r[0].transcript}$('interim').textContent=interim};
 rec.onerror=e=>{if(!['no-speech','aborted'].includes(e.error))add('err','speech: '+e.error);if(e.error==='not-allowed'){want=false;stat('microphone blocked','err')}};
 rec.onend=()=>{running=false;$('interim').textContent='';if(want){stat('listening','live');setTimeout(start,100)}else stat('microphone off')}}
function start(){if(!want||running)return;try{make();rec.start()}catch(e){add('err','restart failed: '+e.message);setTimeout(start,1000)}}
function toggle(){if(!R){add('err','No speech recognition in this browser: use Chrome.');return}
 want=!want;$('mic').textContent=want?'Stop listening':'Start listening';$('mic').className=want?'on':'';
 if(want)start();else if(rec)rec.stop()}
setInterval(()=>{if(want&&!running)start()},3000);            // watchdog: Chrome ends sessions on its own
async function poll(){try{const s=await (await fetch('/voicecmd?c=')).json();if(s.pose)$('pose').textContent=s.pose;
 const p=await (await fetch('/palm')).json();
 $('palm').textContent=p&&p.age<1.5?`your palm: tilt ${p.tilt>0?'+':''}${p.tilt.toFixed(0)}  turn ${p.turn>0?'+':''}${p.turn.toFixed(0)}  size ${p.size.toFixed(0)}px`:'your palm: not seen'}catch(e){}}
setInterval(poll,700);poll();
</script>"""

SHOULDER, ELBOW, WRIST, TWIST = 1, 2, 3, 4
last_move = None
locked = None          # your palm as read by "come here", before the robot's hand covers it in the picture
HOME = (1500, 1405, 1450, 1580, 1500)


def live_palm():
    import urllib.request, time as _t
    try:
        p = json.loads(urllib.request.urlopen("http://127.0.0.1:8091/palm", timeout=2).read())
    except Exception:
        return None
    if not p or _t.time() - p["t"] > 1.5:
        return None
    return p


def held_palm():
    """The palm the high-five page's HOLD mode reached for (seen before the robot's hand covered it), if holding."""
    import urllib.request
    try:
        return json.loads(urllib.request.urlopen("http://127.0.0.1:8091/held", timeout=2).read())
    except Exception:
        return None


def release_hold():
    import urllib.request
    try:
        urllib.request.urlopen("http://127.0.0.1:8091/release?move=0", timeout=2).close()
    except Exception:
        pass


def voice(text):
    """Turn a spoken phrase into a wrist/twist nudge. Returns (what was done, pose line)."""
    global last_move
    t = " " + text.lower().replace("'", "").replace("-", " ") + " "
    st = parse(send("?"))
    if "us" not in st:
        return "arm not answering: " + st["raw"], ""
    us = st["us"]
    pose = f"shoulder {us[SHOULDER]}   elbow {us[ELBOW]}   wrist {us[WRIST]}   twist {us[TWIST]}"
    if not text.strip():
        return "ready", pose
    print("HEARD", repr(text), flush=True)
    global locked
    if any(k in t for k in (" come ", " here ", " aim ", " lock ", " go to ")):
        palm = live_palm()
        if not palm:
            return "can't see an open palm right now: hold it up in the green zone and say it again", pose
        target = pose_for(palm["x"], palm["y"])
        if target is None:
            return f"your palm is outside the reach zone (at {palm['x']:.0f}, {palm['y']:.0f})", pose
        locked = palm
        send("V 30")
        r = parse(send(f"M {target[0]} {target[1]} {target[2]} {us[WRIST]} {us[TWIST]}"))
        us = r.get("us", us)
        return (f"locked your palm (tilt {palm['tilt']:+.0f}, turn {palm['turn']:+.0f}); arm to pan {target[0]}, "
                f"shoulder {target[1]}: now adjust, then say that's it"), f"shoulder {us[SHOULDER]}   elbow {us[ELBOW]}   wrist {us[WRIST]}   twist {us[TWIST]}"
    if any(k in t for k in (" home ", " rest ")):
        locked = None
        release_hold()
        send("V 30")
        send("M " + " ".join(map(str, HOME)))
        return "arm home (wrist and twist back to 1580 / 1500)", "wrist 1580   twist 1500"
    if any(k in t for k in (" thats it ", " that is it ", " record ", " perfect ", " save ")):
        held = None if locked else held_palm()
        palm = locked or held or live_palm()
        import time as _t
        rec = {"t": _t.time(), "arm": us, "palm": palm, "palm_locked": locked is not None or held is not None}
        path = os.path.join(ROOT, "calibration", "wrist_pairs.jsonl")
        if not DEMO:
            with open(path, "a") as f:
                f.write(json.dumps(rec) + "\n")
        print("DEMO, NOT SAVED" if DEMO else "RECORD", json.dumps(rec), flush=True)
        age = "" if not palm else f"; your palm: tilt {palm['tilt']:+.0f}, turn {palm['turn']:+.0f}" + (" (locked)" if locked or held else "")
        locked = None
        return f"recorded wrist {us[WRIST]}, twist {us[TWIST]}{age}", pose
    move = None
    if " again " in t and last_move:
        move = last_move
    else:
        step = 20 if any(k in t for k in (" bit ", " little ", " tiny ", " slightly ")) else 120 if any(k in t for k in (" lot ", " much ", " big ")) else 60
        if " shoulder " in t and any(k in t for k in (" up ", " higher ", " raise ", " down ", " lower ")):
            # shoulder: home is 1405 (arm resting), reaching poses are 1600-2100, so a higher pulse lifts the arm
            ss = 15 if step == 20 else 60 if step == 120 else 30
            move = (SHOULDER, +ss if any(k in t for k in (" up ", " higher ", " raise ")) else -ss)
        elif any(k in t for k in (" up ", " higher ", " raise ", " down ", " lower ")):
            # elbow: a lower pulse lifts the hand (a higher one pushes it forward/down, the slap's direction)
            es = 15 if step == 20 else 60 if step == 120 else 30
            move = (ELBOW, -es if any(k in t for k in (" up ", " higher ", " raise ")) else +es)
        elif any(k in t for k in (" closer ", " reach ", " further ", " farther ")):
            # elbow: higher pulse pushes the hand forward (the slap's direction), towards a palm in front of the robot.
            # Checked before the wrist words, since "move closer to me" also contains "to me".
            es = 15 if step == 20 else 60 if step == 120 else 30
            move = (ELBOW, +es if any(k in t for k in (" closer ", " reach ")) else -es)
        elif any(k in t for k in (" towards ", " toward ", " forward ", " to me ")):
            move = (WRIST, +step)            # higher wrist pulse = hand towards the camera side (2026-10-08)
        elif any(k in t for k in (" away ", " back ", " backward ", " backwards ")):
            move = (WRIST, -step)
        elif " left " in t:
            move = (TWIST, +step)
        elif " right " in t:
            move = (TWIST, -step)
    if not move:
        return f"not a command (say towards / away / left / right / closer / further / up / down / shoulder up / shoulder down, bit / lot, again, that's it)", pose
    j, d = move
    last_move = move
    send("V 20")
    r = parse(send(f"J {j} {us[j] + d}"))
    us = r.get("us", us)
    return f"{ {SHOULDER: 'shoulder', ELBOW: 'elbow', WRIST: 'wrist', TWIST: 'twist'}[j]} {d:+d} -> {us[j]}", f"shoulder {us[SHOULDER]}   elbow {us[ELBOW]}   wrist {us[WRIST]}   twist {us[TWIST]}"


class H(BaseHTTPRequestHandler):
    def reply(self, obj, ctype="application/json"):
        body = obj if isinstance(obj, bytes) else json.dumps(obj).encode()
        self.send_response(200); self.send_header("Content-Type", ctype); self.end_headers(); self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            return self.reply(PAGE.replace("%JOINTS%", json.dumps(JOINTS)).encode(), "text/html")
        if u.path == "/cmd":
            c = parse_qs(u.query)["c"][0]
            r = send(c)
            return self.reply(parse(r if r.startswith(("AT", "DONE")) else send("?")) | {"raw": r})
        if u.path == "/voice":
            return self.reply(VOICE_PAGE.encode(), "text/html")
        if u.path == "/palm":
            try:
                import urllib.request, time as _t
                p = json.loads(urllib.request.urlopen("http://127.0.0.1:8091/palm", timeout=2).read())
                if p:
                    p["age"] = _t.time() - p["t"]
            except Exception:
                p = None
            return self.reply(p)
        if u.path == "/voicecmd":
            did, pose = voice(parse_qs(u.query).get("c", [""])[0])
            return self.reply({"did": did, "pose": pose})
        if u.path == "/save":
            s = parse(send("?"))
            if "us" not in s:
                return self.reply({"msg": "Could not read the arm: " + s["raw"]})
            home = [us if on else 0 for us, on in zip(s["us"], s["on"])]
            for p in CONFIGS:
                txt = open(p).read()
                txt = re.sub(r"const int HOME_US\[JOINT_COUNT\] = \{[^}]*\};",
                             "const int HOME_US[JOINT_COUNT] = {" + ", ".join(map(str, home)) + "};", txt)
                open(p, "w").write(txt)
            print("SAVED HOME", home, flush=True)
            limp = [NAMES[j] for j, on in enumerate(s["on"]) if not on]
            return self.reply({"msg": f"Saved home {home}." + (f" Still limp (no home): {', '.join(limp)}." if limp else "")
                               + " Flash firmware/arm_wifi to apply it."})
        self.send_error(404)

    def log_message(self, *a):
        pass


print("arm_jog: http://localhost:8090" + (" (DEMO: recordings are not saved)" if DEMO else ""), flush=True)
ThreadingHTTPServer(("127.0.0.1", 8090), H).serve_forever()
