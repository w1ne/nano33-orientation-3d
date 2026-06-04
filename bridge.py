#!/usr/bin/env python3
"""
Bridge: Arduino serial  ->  WebSocket  ->  browser 3D visualization.

- Reads CSV orientation lines from the Nano 33 BLE over USB serial.
- Serves the static web/ page over HTTP (http://localhost:8000).
- Broadcasts each parsed sample as JSON to all WebSocket clients (ws://localhost:8765).

Usage:
    python3 bridge.py [--port /dev/ttyACM2] [--baud 115200]
Then open http://localhost:8000 in Chrome.
"""

import argparse
import asyncio
import http.server
import json
import socketserver
import threading
from functools import partial
from pathlib import Path

import serial
import serial.tools.list_ports
import websockets

WEB_DIR = Path(__file__).parent / "web"
HTTP_PORT = 8000
WS_PORT = 8765

clients: set = set()
latest = {"q": [1.0, 0.0, 0.0, 0.0], "gyroMag": 0.0, "connected": False}


def find_port(preferred: str | None) -> str | None:
    if preferred:
        return preferred
    for p in serial.tools.list_ports.comports():
        desc = f"{p.description} {p.manufacturer or ''}".lower()
        if "nano 33" in desc or "arduino" in desc or p.vid == 0x2341:
            return p.device
    return None


def serial_reader(port: str, baud: float, loop: asyncio.AbstractEventLoop):
    """Blocking serial read loop, run in a background thread."""
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                print(f"[serial] connected to {port} @ {baud}")
                latest["connected"] = True
                while True:
                    raw = ser.readline().decode("ascii", "ignore").strip()
                    if not raw or raw.startswith("ERR"):
                        if raw:
                            print(f"[serial] {raw}")
                        continue
                    parts = raw.split(",")
                    if len(parts) != 5:
                        continue
                    try:
                        q0, q1, q2, q3, gmag = (float(x) for x in parts)
                    except ValueError:
                        continue
                    latest["q"] = [q0, q1, q2, q3]
                    latest["gyroMag"] = gmag
                    msg = json.dumps({"q": [q0, q1, q2, q3], "gyroMag": gmag})
                    asyncio.run_coroutine_threadsafe(broadcast(msg), loop)
        except serial.SerialException as e:
            latest["connected"] = False
            print(f"[serial] {e} -- retrying in 2s")
            import time
            time.sleep(2)


async def broadcast(msg: str):
    if not clients:
        return
    dead = set()
    for ws in clients:
        try:
            await ws.send(msg)
        except Exception:
            dead.add(ws)
    clients.difference_update(dead)


async def ws_handler(ws):
    clients.add(ws)
    print(f"[ws] client connected ({len(clients)} total)")
    try:
        await ws.wait_closed()
    finally:
        clients.discard(ws)
        print(f"[ws] client disconnected ({len(clients)} total)")


def start_http():
    handler = partial(http.server.SimpleHTTPRequestHandler, directory=str(WEB_DIR))
    socketserver.TCPServer.allow_reuse_address = True
    httpd = socketserver.TCPServer(("", HTTP_PORT), handler)
    print(f"[http] serving {WEB_DIR} at http://localhost:{HTTP_PORT}")
    httpd.serve_forever()


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    port = find_port(args.port)
    if not port:
        print("[serial] no Arduino port found; pass --port /dev/ttyACMx")
    loop = asyncio.get_running_loop()

    threading.Thread(target=start_http, daemon=True).start()
    if port:
        threading.Thread(target=serial_reader, args=(port, args.baud, loop), daemon=True).start()

    async with websockets.serve(ws_handler, "", WS_PORT):
        print(f"[ws] listening on ws://localhost:{WS_PORT}")
        print(f"\n  ==>  Open  http://localhost:{HTTP_PORT}  in Chrome\n")
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nbye")
