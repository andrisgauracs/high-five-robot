"""Write wifi_secrets.h from the project's .env (WIFI_SSID / WIFI_PASSWORD) without printing them.
python3 firmware/camera_test/make_secrets.py"""
import json, os, sys
root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
env = {}
for line in open(os.path.join(root, ".env")):
    line = line.strip()
    if line and not line.startswith("#") and "=" in line:
        k, v = line.split("=", 1)
        env[k.strip()] = v.strip()
missing = [k for k in ("WIFI_SSID", "WIFI_PASSWORD") if not env.get(k) or env[k].startswith("your-")]
if missing:
    sys.exit("fill in " + ", ".join(missing) + " in .env first")
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "wifi_secrets.h"), "w") as f:
    f.write("// generated from .env by make_secrets.py, do not commit\n#pragma once\n")
    for k in ("WIFI_SSID", "WIFI_PASSWORD"):
        f.write(f"#define {k} {json.dumps(env[k])}\n")
print(f"wifi_secrets.h written (SSID {len(env['WIFI_SSID'])} chars, password {len(env['WIFI_PASSWORD'])} chars)")
