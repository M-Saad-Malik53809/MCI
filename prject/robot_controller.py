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
BRAKE_TICKS    = 6           # ticks of opposite command sent on key release
                             # 6 ticks @ 20 Hz = 300 ms brake burst
# ───────────────────────────────────────────────────────────────────────

if sys.platform == "win32":
    os.system("")   # enable ANSI on Windows

BOLD  = "\033[1m";  DIM   = "\033[2m";  RESET = "\033[0m"
GREEN = "\033[92m"; RED   = "\033[91m"; YELLOW= "\033[93m"
CYAN  = "\033[96m"; WHITE = "\033[97m"; MAG   = "\033[95m"
ERASE = "\r\033[2K"

CMD_LABEL = {
    "FWD":       f"{GREEN}{BOLD}[ ↑  FORWARD  ]{RESET}",
    "BWD":       f"{RED}{BOLD}[  ↓  BACK   ]{RESET}",
    "STOP":      f"{YELLOW}{BOLD}[  ■  STOP   ]{RESET}",
    "IDLE":      f"{DIM}[    idle     ]{RESET}",
    "BRAKE_FWD": f"{YELLOW}{BOLD}[ ⟳  BRAKING ]{RESET}",
    "BRAKE_BWD": f"{YELLOW}{BOLD}[ ⟳  BRAKING ]{RESET}",
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

    interval    = 1.0 / SEND_RATE_HZ
    last_sent   = b"0"   # track last transmitted byte
    brake_left  = 0      # remaining brake ticks
    brake_byte  = b"0"  # byte to send during braking
    state       = "IDLE"

    def send(b: bytes):
        """Transmit one byte; swallow write errors gracefully."""
        nonlocal last_sent
        try:
            ser.write(b)
            last_sent = b
        except Exception:
            pass

    try:
        while True:
            if keyboard.is_pressed("q"):
                sys.stdout.write(ERASE + f"{BOLD}Goodbye!{RESET}\n")
                sys.stdout.flush()
                send(b"0")
                break

            fwd = keyboard.is_pressed("up") or keyboard.is_pressed("w")
            bwd = keyboard.is_pressed("down") or keyboard.is_pressed("s")
            stp = keyboard.is_pressed("space")

            if fwd:
                # Key active — cancel any pending brake, drive forward
                brake_left = 0
                state = "FWD"
                send(b"F")
            elif bwd:
                brake_left = 0
                state = "BWD"
                send(b"B")
            elif stp:
                brake_left = 0
                state = "STOP"
                send(b"0")
            elif brake_left > 0:
                # Mid-brake: send opposite direction burst
                brake_left -= 1
                send(brake_byte)
                if brake_left == 0:
                    # Brake finished — settle to zero
                    send(b"0")
                    state = "IDLE"
            else:
                # Truly idle
                prev_state = state
                state = "IDLE"
                if prev_state == "FWD":
                    # Released forward → kick backward to cancel lean
                    brake_left = BRAKE_TICKS
                    brake_byte = b"B"
                    state = "BRAKE_FWD"
                    send(brake_byte)
                    brake_left -= 1
                elif prev_state == "BWD":
                    # Released backward → kick forward to cancel lean
                    brake_left = BRAKE_TICKS
                    brake_byte = b"F"
                    state = "BRAKE_BWD"
                    send(brake_byte)
                    brake_left -= 1
                else:
                    # Already idle — send '0' once if not already sent
                    if last_sent != b"0":
                        send(b"0")

            _render(state)
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
