"""Send one command to the arm and wait for the move to finish.  python3 tools/arm.py M 1500 1500 1450 1580 1500
The arm is reached over USB through tools/arm_bridge.py (127.0.0.1:8766) when that is running, otherwise straight
over Wi-Fi (firmware/arm_wifi, highfive-arm.local:8766, same one-line protocol). ARM_HOST=<ip or name> forces one."""
import os, socket, sys

PORT = 8766
WIFI_NAME = "highfive-arm.local"
_wifi_ip = None              # the board's address, looked up once (mDNS lookups are slow) and again after a failure


def _hosts():
    if os.environ.get("ARM_HOST"):
        return [os.environ["ARM_HOST"]]
    return ["127.0.0.1", "wifi"]


def send(cmd):
    global _wifi_ip
    err = None
    for host in _hosts():
        try:
            if host == "wifi":
                if _wifi_ip is None:
                    _wifi_ip = socket.gethostbyname(WIFI_NAME)
                host = _wifi_ip
            with socket.create_connection((host, PORT), timeout=40) as s:
                s.sendall((cmd + "\n").encode())
                reply = s.makefile().readline().strip()
            if not reply:
                raise IOError("no reply")
            return reply
        except OSError as e:
            if host == _wifi_ip:
                _wifi_ip = None
            err = e
    raise ConnectionError(f"arm not reachable (USB bridge or {WIFI_NAME}): {err}")


if __name__ == "__main__":
    print(send(" ".join(sys.argv[1:])))
