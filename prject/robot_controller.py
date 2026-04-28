"""
robot_controller.py — Game-like keyboard controller for the balance robot.

Hold keys — no Enter needed:
  ↑ / W   Forward    ↓ / S   Backward    Space  Stop    Q  Quit

Usage:
  python robot_controller.py          # auto-detect
  python robot_controller.py COM4     # explicit port
"""

import sys, os, re, time, threading
import serial, serial.tools.list_ports

# ── Config ─────────────────────────────────────────────────────────────
BAUD_RATE      = 115200
SEND_RATE_HZ   = 20          # Hz — command repeat rate while key held
IDLE_RESET_HZ  = 5           # Hz — how often '0' is sent while idle
# ───────────────────────────────────────────────────────────────────────

if sys.platform == "win32":
    os.system("")   # enable ANSI on Windows

BOLD  = "\033[1m";  DIM   = "\033[2m";  RESET = "\033[0m"
GREEN = "\033[92m"; RED   = "\033[91m"; YELLOW= "\033[93m"
CYAN  = "\033[96m"; WHITE = "\033[97m"; MAG   = "\033[95m"
ERASE = "\r\033[2K"

CMD_LABEL = {
    "FWD":  f"{GREEN}{BOLD}[ ↑  FORWARD  ]{RESET}",
    "BWD":  f"{RED}{BOLD}[  ↓  BACK   ]{RESET}",
    "STOP": f"{YELLOW}{BOLD}[  ■  STOP   ]{RESET}",
    "IDLE": f"{DIM}[    idle     ]{RESET}",
}

# ── Shared UART line ────────────────────────────────────────────────────
_last_line = ""
_line_lock = threading.Lock()


def _colorize(raw: str) -> str:
    """Add ANSI colors to the raw telemetry string from the firmware."""
    s = raw
    # color field labels cyan, numbers white
    s = re.sub(r"(tilt=)([+-]?\d+\.\d+)",
               lambda m: f"{CYAN}tilt={WHITE}{m.group(2)}{RESET}", s)
    s = re.sub(r"(gyro=)([+-]?\d+\.\d+)",
               lambda m: f"{CYAN}gyro={WHITE}{m.group(2)}{RESET}", s)
    s = re.sub(r"(acc=)([+-]?\d+\.\d+)",
               lambda m: f"{CYAN}acc={WHITE}{m.group(2)}{RESET}", s)
    # setpoint in bold green so it stands out
    s = re.sub(r"(setpoint=)([+-]?\d+\.\d+)",
               lambda m: f"{MAG}{BOLD}setpoint={GREEN}{m.group(2)}{RESET}", s)
    return s


def _render(cmd: str):
    with _line_lock:
        raw = _last_line
    colored = _colorize(raw) if raw else f"{DIM}waiting for telemetry…{RESET}"
    sys.stdout.write(ERASE + CMD_LABEL[cmd] + "  " + colored)
    sys.stdout.flush()


def _reader(ser: serial.Serial, stop: threading.Event):
    """Background thread: read UART lines and store the latest one."""
    global _last_line
    buf = b""
    while not stop.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode(errors="replace").strip()
                    if text:
                        with _line_lock:
                            _last_line = text
        except Exception:
            break


def find_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if any(k in (p.description or "").lower()
               for k in ("stm32", "stlink", "usb serial")):
            return p.device
    return ports[0].device if ports else None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    if not port:
        print("ERROR: No COM port found. Usage: python robot_controller.py COM4")
        sys.exit(1)

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0)  # non-blocking read
    except serial.SerialException as e:
        print(f"ERROR opening {port}: {e}")
        sys.exit(1)

    # ── Header ─────────────────────────────────────────────────────────
    print(f"{BOLD}{CYAN}═══════════════════════════════════════════════{RESET}")
    print(f"{BOLD}  Balance Robot  │  {CYAN}{port}{RESET}{BOLD}  │  {BAUD_RATE} baud{RESET}")
    print(f"{BOLD}{CYAN}═══════════════════════════════════════════════{RESET}")
    print(f"  {GREEN}↑/W{RESET} Forward   {RED}↓/S{RESET} Backward   "
          f"{YELLOW}Space{RESET} Stop   {DIM}Q{RESET} Quit")
    print(f"{BOLD}{CYAN}───────────────────────────────────────────────{RESET}\n")

    try:
        import keyboard
    except ImportError:
        print("Run:  pip install keyboard"); sys.exit(1)

    stop = threading.Event()
    threading.Thread(target=_reader, args=(ser, stop), daemon=True).start()

    interval   = 1.0 / SEND_RATE_HZ
    last_sent  = b"0"   # track last transmitted byte

    def send(b: bytes):
        """Send only if needed; swallow write errors gracefully."""
        nonlocal last_sent
        try:
            ser.write(b)
            last_sent = b
        except Exception:
            pass   # skip this tick on write error

    try:
        while True:
            if keyboard.is_pressed("q"):
                sys.stdout.write(ERASE + f"{BOLD}Goodbye!{RESET}\n")
                sys.stdout.flush()
                send(b"0")
                break

            if keyboard.is_pressed("up") or keyboard.is_pressed("w"):
                cmd = "FWD";  send(b"F")
            elif keyboard.is_pressed("down") or keyboard.is_pressed("s"):
                cmd = "BWD";  send(b"B")
            elif keyboard.is_pressed("space"):
                cmd = "STOP"; send(b"0")
            else:
                cmd = "IDLE"
                # Send '0' only once on transition to idle;
                # stay silent after that until a key is pressed.
                if last_sent != b"0":
                    send(b"0")

            _render(cmd)
            time.sleep(interval)

    except KeyboardInterrupt:
        sys.stdout.write(ERASE + f"{BOLD}Interrupted.{RESET}\n")
        sys.stdout.flush()
        ser.write(b"0")
    finally:
        stop.set()
        ser.close()


if __name__ == "__main__":
    main()
