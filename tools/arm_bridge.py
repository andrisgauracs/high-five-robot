"""Keeps the servo board's serial port open (opening it resets the board) and relays commands from a local TCP
port.  .venv/bin/python tools/arm_bridge.py [port]
Send one line, e.g. "M 1500 1500 1450 1580 1500"; the reply is the board's last line once the move is DONE."""
import socket, sys, time, serial

if len(sys.argv) < 2:
    sys.exit("usage: arm_bridge.py <serial port>, for example /dev/cu.usbmodem1234")
PORT = sys.argv[1]


def open_board():
    """Open the serial port, waiting for it to (re)appear, and let the board finish booting."""
    while True:
        try:
            s = serial.Serial(PORT, 115200, timeout=0.1)
            break
        except (serial.SerialException, OSError) as e:
            print("arm_bridge: board not there yet:", e, flush=True)
            time.sleep(2)
    time.sleep(0.2)
    boot_until = time.time() + 6               # the board resets on open and centres its joints one by one
    while time.time() < boot_until:
        ln = s.readline().decode(errors="replace").strip()
        if ln:
            print("board:", ln, flush=True)
            if ln.startswith("READY"):
                break
    return s


ser = open_board()

srv = socket.create_server(("127.0.0.1", 8766))
print("arm_bridge: listening on 127.0.0.1:8766", flush=True)
while True:
    conn, _ = srv.accept()
    with conn:
        cmd = conn.makefile().readline().strip()
        try:
            ser.reset_input_buffer()
            ser.write((cmd + "\n").encode())
            reply, until = "", time.time() + 30
            while time.time() < until:
                ln = ser.readline().decode(errors="replace").strip()
                if not ln:
                    continue
                reply = ln
                if ln.startswith(("DONE", "ERR", "AT")) or (ln.startswith("OK speed")):
                    break
        except (serial.SerialException, OSError) as e:   # board reset or USB blip: answer, then reopen the port
            reply = f"ERR board disconnected ({e}) - reconnecting"
            print(f"{cmd} -> {reply}", flush=True)
            conn.sendall((reply + "\n").encode())
            try:
                ser.close()
            except Exception:
                pass
            ser = open_board()
            continue
        print(f"{cmd} -> {reply}", flush=True)
        conn.sendall((reply + "\n").encode())
