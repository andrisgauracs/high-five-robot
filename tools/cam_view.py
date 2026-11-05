"""Local viewer for the XIAO camera: relays its MJPEG stream to http://localhost:8080 so a browser that is
blocked from the home network (macOS Local Network privacy) can still show it.
The sensor sits sideways on the Sense board: raw frames have world-up on the right. The page shows them upright
and unmirrored (checked 2026-10-20 with a raised right hand), which is a transpose of the raw frame:
numpy frame.transpose(1, 0, 2)[::-1, ::-1] (= ffmpeg "transpose=clock,vflip"). Keep the sensor at hmirror 0, vflip 0.
.venv/bin/python tools/cam_view.py [camera-ip]"""
import sys, urllib.request
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

CAM = sys.argv[1] if len(sys.argv) > 1 else "highfive-cam.local"
PAGE = b"""<!doctype html><meta charset=utf-8><title>Robot camera</title>
<style>body{margin:0;background:#111;color:#ddd;font:15px system-ui;display:grid;place-items:center;min-height:100vh}
img{height:min(60vw,58vh);border-radius:8px;transform:scaleY(-1) rotate(90deg)}.v{display:grid;place-items:center;width:min(60vw,58vh);height:min(90vw,87vh)}</style>
<div><p>Robot camera (relayed from the XIAO)</p><div class=v><img src="/stream"></div></div>"""


class H(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200); self.send_header("Content-Type", "text/html"); self.end_headers()
            self.wfile.write(PAGE); return
        if self.path == "/stream":
            up = urllib.request.urlopen(f"http://{CAM}:81/stream", timeout=10)
            self.send_response(200); self.send_header("Content-Type", up.headers["Content-Type"]); self.end_headers()
            try:
                while chunk := up.read(4096):
                    self.wfile.write(chunk)
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                up.close()
            return
        self.send_error(404)

    def log_message(self, *a):
        pass


print(f"cam_view: relaying http://{CAM}:81/stream -> http://localhost:8080", flush=True)
# the camera firmware turns vflip on at every boot; the upright view above assumes the raw sensor orientation
for var, val in (("framesize", 10), ("vflip", 1), ("hmirror", 0)):   # vflip 1 for the OV2640 (see cam_snap.CAM_VFLIP)
    urllib.request.urlopen(f"http://{CAM}/control?var={var}&val={val}", timeout=5).close()
ThreadingHTTPServer(("127.0.0.1", 8080), H).serve_forever()
