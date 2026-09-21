#!/usr/bin/env python3
"""Read N300Pro / HiPNUC RPY from serial and serve it to the MuJoCo web demo."""

from __future__ import annotations

import argparse
import json
import logging
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

logging.basicConfig(level=logging.WARNING)

EXAMPLE_DIR = Path(__file__).resolve().parents[2] / "WHEELTEC N300Pro惯导产品资料" / "5.模块例程" / "python例程"
sys.path.insert(0, str(EXAMPLE_DIR))

import serial  # noqa: E402
from parsers.hipnuc_serial_parser import hipnuc_parser  # noqa: E402

STATE = {
    "ok": False,
    "connected": False,
    "port": "",
    "error": "",
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "hz": 0,
    "stamp": 0.0,
}
LOCK = threading.Lock()


def serial_loop(port: str, baudrate: int) -> None:
    parser = hipnuc_parser()
    frame_count = 0
    last_hz = time.time()
    while True:
        try:
            with serial.Serial(port, baudrate, timeout=0.2) as ser:
                with LOCK:
                    STATE.update(connected=True, port=port, error="", ok=False)
                print(f"opened {port} @ {baudrate}", flush=True)
                while True:
                    waiting = ser.in_waiting
                    chunk = ser.read(waiting or 1)
                    if not chunk:
                        continue
                    frames = parser.parse(chunk)
                    for frame in frames:
                        if frame.roll is None or frame.pitch is None or frame.yaw is None:
                            continue
                        frame_count += 1
                        now = time.time()
                        with LOCK:
                            STATE.update(
                                ok=True,
                                roll=float(frame.roll),
                                pitch=float(frame.pitch),
                                yaw=float(frame.yaw),
                                stamp=now,
                            )
                            if now - last_hz >= 1.0:
                                STATE["hz"] = frame_count
                                frame_count = 0
                                last_hz = now
        except (serial.SerialException, PermissionError, OSError) as error:
            with LOCK:
                STATE.update(ok=False, connected=False, error=str(error))
            print(f"serial error: {error}", flush=True)
            time.sleep(1.0)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self._cors()
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.end_headers()

    def do_GET(self) -> None:
        if self.path.split("?", 1)[0] not in ("/", "/imu", "/imu.json"):
            self.send_response(404)
            self._cors()
            self.end_headers()
            return
        with LOCK:
            payload = dict(STATE)
        body = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self._cors()
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    parser = argparse.ArgumentParser(description="N300Pro IMU bridge for web_mujoco")
    parser.add_argument("-p", "--port", default="/dev/cu.usbserial-5B0B0306631")
    parser.add_argument("-b", "--baudrate", type=int, default=115200)
    parser.add_argument("--http", type=int, default=8768)
    args = parser.parse_args()

    worker = threading.Thread(target=serial_loop, args=(args.port, args.baudrate), daemon=True)
    worker.start()
    server = ThreadingHTTPServer(("127.0.0.1", args.http), Handler)
    print(f"IMU bridge http://127.0.0.1:{args.http}/imu", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("stopped", flush=True)


if __name__ == "__main__":
    main()
