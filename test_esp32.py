"""
test_esp32.py — minimal smoke-test for the PC ↔ ESP32 link.

Run this BEFORE trying to plot anything. It proves the serial protocol
works end-to-end:

  1. opens the port
  2. waits for the READY banner (or pings an already-running board)
  3. sends 4 demo commands and verifies every OK/ERR reply
  4. detaches the servos and closes the port cleanly

Usage (edit the PORT constant below first, or pass --port):

    python test_esp32.py --port COM4
    python test_esp32.py --port /dev/ttyUSB0
"""
import argparse
import glob
import json
import os
import queue
import socket
import sys
import threading
from time import sleep, monotonic

import serial


DEFAULT_PORTS = (
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/serial/by-id/*",
)

PORT     = None
BAUDRATE = 115200
TCP_PORT = 8888


def detect_serial_port():
    env_port = os.environ.get("BGRAPH_SERIAL_PORT")
    if env_port:
        return env_port

    for pattern in DEFAULT_PORTS:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]

    return None


class Esp32Link:
    """Standalone copy of the same protocol used in plotter.py — kept here
    so this test works even if the main package is broken."""

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(
            port=port, baudrate=baudrate,
            timeout=0.2, write_timeout=2.0,
            dsrdtr=False, rtscts=False, xonxoff=False,
        )
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass
        sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.q = queue.Queue()
        self.running = True
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    def _reader(self):
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(64)
            except Exception:
                return
            if not chunk:
                continue
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                line = bytes(buf[:nl]).rstrip(b"\r").decode("utf-8", "replace").strip()
                del buf[: nl + 1]
                if not line:
                    continue
                if line.startswith("#"):
                    print(f"  [esp32] {line[1:].strip()}")
                    continue
                self.q.put(line)

    def handshake(self, timeout=8.0):
        deadline = monotonic() + timeout
        last_poke = 0.0
        while monotonic() < deadline:
            try:
                line = self.q.get(timeout=0.3)
            except queue.Empty:
                line = None
            if line in ("READY", "OK"):
                # drain then ping to confirm
                try:
                    while True:
                        self.q.get_nowait()
                except queue.Empty:
                    pass
                self.ser.write(b"{}\n")
                try:
                    pong = self.q.get(timeout=1.0)
                except queue.Empty:
                    return False
                return pong == "OK"
            now = monotonic()
            if now - last_poke > 0.5:
                self.ser.write(b"{}\n")
                last_poke = now
        return False

    def cmd(self, obj, expect_ok=True, label=None):
        payload = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
        self.ser.write(payload)
        try:
            reply = self.q.get(timeout=1.0)
        except queue.Empty:
            print(f"  ✗ {label or obj}: NO REPLY")
            return False
        if expect_ok:
            ok = (reply == "OK")
        else:
            ok = reply.startswith("ERR:")
        tick = "✓" if ok else "✗"
        print(f"  {tick} {label or obj} -> {reply}")
        return ok

    def close(self):
        self.running = False
        sleep(0.25)
        try:
            self.ser.close()
        except Exception:
            pass


class Esp32TcpLink:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=2.0)
        self.sock.settimeout(0.2)
        self.q = queue.Queue()
        self.running = True
        self.t = threading.Thread(target=self._reader, daemon=True)
        self.t.start()

    def _reader(self):
        buf = bytearray()
        while self.running:
            try:
                chunk = self.sock.recv(64)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                line = bytes(buf[:nl]).rstrip(b"\r").decode("utf-8", "replace").strip()
                del buf[: nl + 1]
                if not line:
                    continue
                if line.startswith("#"):
                    print(f"  [esp32] {line[1:].strip()}")
                    continue
                self.q.put(line)

    def handshake(self, timeout=8.0):
        deadline = monotonic() + timeout
        last_poke = 0.0
        while monotonic() < deadline:
            try:
                line = self.q.get(timeout=0.3)
            except queue.Empty:
                line = None
            if line in ("READY", "OK"):
                try:
                    while True:
                        self.q.get_nowait()
                except queue.Empty:
                    pass
                self.sock.sendall(b"{}\n")
                try:
                    pong = self.q.get(timeout=1.0)
                except queue.Empty:
                    return False
                return pong == "OK"
            now = monotonic()
            if now - last_poke > 0.5:
                self.sock.sendall(b"{}\n")
                last_poke = now
        return False

    def cmd(self, obj, expect_ok=True, label=None):
        payload = (json.dumps(obj, separators=(",", ":")) + "\n").encode()
        self.sock.sendall(payload)
        try:
            reply = self.q.get(timeout=1.0)
        except queue.Empty:
            print(f"  ✗ {label or obj}: NO REPLY")
            return False
        if expect_ok:
            ok = (reply == "OK")
        else:
            ok = reply.startswith("ERR:")
        tick = "✓" if ok else "✗"
        print(f"  {tick} {label or obj} -> {reply}")
        return ok

    def close(self):
        self.running = False
        sleep(0.25)
        try:
            self.sock.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--port",
        default=PORT or detect_serial_port(),
        help="serial port (e.g. /dev/ttyUSB0 or /dev/ttyACM0)",
    )
    ap.add_argument("--baud", type=int, default=BAUDRATE)
    ap.add_argument("--host", default=os.environ.get("BGRAPH_ESP32_HOST"))
    ap.add_argument("--tcp-port", type=int, default=int(os.environ.get("BGRAPH_ESP32_PORT", TCP_PORT)))
    args = ap.parse_args()

    if not args.host and not args.port:
        print("FAIL: no serial port detected. Export BGRAPH_SERIAL_PORT or pass --port.")
        return 1

    if args.host:
        print(f"Opening {args.host}:{args.tcp_port} ...")
        try:
            link = Esp32TcpLink(args.host, args.tcp_port)
        except OSError as e:
            print(f"FAIL: cannot open TCP connection: {e}")
            return 1
    else:
        print(f"Opening {args.port} @ {args.baud} baud...")
        try:
            link = Esp32Link(args.port, args.baud)
        except serial.SerialException as e:
            print(f"FAIL: cannot open port: {e}")
            return 1

    print("Waiting for handshake (READY or OK to ping)...")
    if not link.handshake():
        print("FAIL: no response from ESP32. "
              "Flash the matching firmware, check the port/host, check power.")
        link.close()
        return 1
    print("Handshake OK.\n")

    print("Sending test commands:")
    ok = True
    # 1) center both arm servos
    ok &= link.cmd({"s1": 1500, "s2": 1500}, label="center s1+s2")
    sleep(0.7)
    # 2) small nudge
    ok &= link.cmd({"s1": 1400, "s2": 1600}, label="nudge s1-, s2+")
    sleep(0.7)
    # 3) pen demo with easing
    ok &= link.cmd({"pen": 1500},            label="pen snap 1500")
    sleep(0.3)
    ok &= link.cmd({"pen": 1700, "pen_ms": 400}, label="pen ease to 1700 (400 ms)")
    sleep(0.6)
    # 4) deliberately bad pulse-width — we EXPECT an ERR here
    ok &= link.cmd({"s1": 99999}, expect_ok=False, label="bad pw (should be ERR)")
    # 5) detach everything
    ok &= link.cmd({"s1": 0, "s2": 0, "pen": 0}, label="detach all")

    link.close()
    if ok:
        print("\nAll tests passed. The PC ↔ ESP32 link is healthy.")
        return 0
    print("\nSome tests failed. Check the log above.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
