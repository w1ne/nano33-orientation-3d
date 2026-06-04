# Board Orientation

Real-time 3D orientation tracking for an **Arduino Nano 33 BLE Rev2** (BMI270 + BMM150),
visualized live in the browser. The board's on-board sensors are fused into an orientation
quaternion and a little 3D model mirrors the board's real attitude — tilt or spin the
board, the model follows.

![demo](docs/demo.png)

The panel shows live **roll / pitch / yaw**, the current angular **rate**, and a
**stability** indicator (STEADY / MOVING / FAST) with a rolling sparkline.

## How it works

```
BMI270 (accel+gyro) + BMM150 (mag)  ──>  9-axis Mahony fusion (on-board)
                                                 │   quaternion + rate
                                          USB serial (~50 Hz)
                                                 │
                          bridge.py (serial → WebSocket + static file server)
                                                 │
                          browser: Three.js 3D model + HUD (web/)
```

1. **`sketch/OrientationStream`** reads the accelerometer, gyroscope **and magnetometer**
   and runs a [Mahony](https://nitinjsanket.github.io/tutorials/attitudeest/mahony) AHRS
   filter *on the board*, streaming a unit quaternion `q0,q1,q2,q3` plus the gyro
   magnitude (a stability metric) over USB at ~50 Hz.
2. **`bridge.py`** reads that serial stream, serves the web page, and forwards each
   sample to the browser over a WebSocket. It auto-detects the board's serial port and
   re-detects it on reconnect, so the board can be replugged/moved without restarting.
3. **`web/`** renders a 3D board model with Three.js and applies the quaternion every
   frame, plus the orientation/stability HUD.

## Sensor fusion

Full **9-axis fusion** (accelerometer + gyroscope + magnetometer), Mahony filter:

- **Accelerometer** anchors **roll & pitch** to gravity — absolute, drift-free.
- **Magnetometer** anchors **yaw** to the Earth's magnetic field — an absolute compass
  heading, so yaw doesn't accumulate drift (a 6-axis gyro-only filter can't fix yaw,
  because it has no absolute yaw reference).
- **Mahony's integral term** continuously estimates and removes the gyroscope bias,
  including the transient bias that appears right after a fast movement.

**Gyro startup calibration:** on reset the sketch averages the gyro for ~2 s to seed the
zero-rate bias — **keep the board still for a couple of seconds after flashing/reset.**

**Magnetometer calibration (board-specific):** the BMM150's axes are *not* aligned with
the BMI270's, and the library passes magnetometer data through raw. This sketch's mapping
(`board_x=-my, board_y=+mx, board_z=+mz`) and hard-iron offsets were recovered empirically
from a tumble capture and validated against gravity (the gravity↔field angle came out 23.4°,
matching the local magnetic dip). If you use a different board or location, re-run the
calibration — flash `sketch/RawStream`, tumble the board through all orientations, and
recompute the offsets/mapping from the captured `mx,my,mz` vs `ax,ay,az`.

> ⚠️ **The magnetometer needs a clean magnetic environment.** Near a laptop, motors,
> speakers, magnets, or metal, the local field is distorted *and time-varying*, so the
> compass heading wanders and yaw follows it — even while the board sits still. If yaw
> drifts at rest, that's almost always local interference, not the filter. Measured on a
> desk next to a laptop the field swung from 24–37 µT (it should be a steady ~50 µT), so
> for trustworthy yaw run the board on battery / a long cable, ~0.5–1 m clear of
> electronics and metal.

## What it can (and can't) do

- ✅ **Orientation** — roll / pitch / yaw (which way the board is tilted and pointing).
- ✅ **Motion / stability** — how fast it's rotating right now.
- ❌ **Absolute XYZ position** — *not possible* from an IMU alone; integrating
  acceleration drifts to nonsense within seconds. Position needs external reference
  hardware (camera, UWB, etc.).

## Quick start

```bash
# 1. Flash the board (needs arduino-cli + the mbed_nano core)
arduino-cli core install arduino:mbed_nano
arduino-cli lib install Arduino_BMI270_BMM150     # Rev2 IMU; see note below for rev1
arduino-cli compile --fqbn arduino:mbed_nano:nano33ble sketch/OrientationStream
arduino-cli upload  --fqbn arduino:mbed_nano:nano33ble -p /dev/ttyACM0 sketch/OrientationStream
# keep the board still for ~2 s after upload (gyro calibration)

# 2. Run the bridge (Python 3 + pyserial + websockets)
python3 bridge.py            # auto-detects the Arduino port

# 3. Open the visualization
#    http://localhost:8000   in Chrome
```

## Running the visualization

The visualization is served by `bridge.py` itself — there's no separate build or web
server to start.

1. **Start the bridge** (with the board plugged in):
   ```bash
   python3 bridge.py
   ```
   You should see `[serial] connected to /dev/ttyACMx`, `[http] serving … :8000`, and
   `[ws] listening … :8765`.
2. **Open** `http://localhost:8000` in a Chromium-based browser (Chrome, Edge, Brave).
3. The status dot turns **green ("board connected")** and the 3D board model starts
   mirroring the real board. The HUD shows roll / pitch / yaw, angular rate, and the
   stability bar + sparkline.

**Controls:** drag to orbit the camera · scroll to zoom · **double-click to re-zero yaw**.

**Notes**
- The page auto-connects to the bridge over WebSocket and **auto-reconnects** if the
  bridge restarts or the board is replugged — no need to refresh.
- If the dot stays red / "waiting for board…", the bridge isn't seeing the board: check
  it's plugged in and that `python3 bridge.py` printed a `[serial] connected` line.
- Only one program can hold the serial port at a time — stop other serial monitors (and
  `arduino-cli` uploads) before/while running the bridge.
- Ports/host are fixed in `bridge.py`: HTTP `8000`, WebSocket `8765`.

### Which IMU library?

The Nano 33 BLE comes in two flavors with **different IMU chips** but the *same* USB name:

| Board | IMU | Library | I2C addresses |
|-------|-----|---------|---------------|
| Nano 33 BLE (rev1) / Sense | LSM9DS1 | `Arduino_LSM9DS1` | `0x6B`, `0x1E` |
| Nano 33 BLE Rev2 / Sense Rev2 | BMI270 + BMM150 | `Arduino_BMI270_BMM150` | `0x68`, `0x10` |

Both expose the identical `IMU.*` API. If `IMU.begin()` fails you likely have the other
variant — flash `sketch/I2CScan` to see which I2C addresses respond and switch the
`#include` accordingly. This repo's sketch targets the **Rev2 (BMI270 + BMM150)**; the
magnetometer axis mapping above is specific to that board.

## Repo layout

```
sketch/OrientationStream/   Arduino sketch: 9-axis Mahony fusion, streams quaternion + rate
sketch/I2CScan/             Utility: identify which IMU chip is fitted (scans I2C)
sketch/RawStream/           Utility: dump raw accel/gyro/mag for calibration & diagnostics
bridge.py                   Serial → WebSocket bridge + static file server (auto-reconnect)
diag_capture.py             Records orientation over the WebSocket to analyze drift/recovery
web/                        Three.js 3D visualization (index.html, app.js, three.module.min.js)
```

## Requirements

- Arduino Nano 33 BLE (Rev2 / BMI270 + BMM150 for the 9-axis magnetometer path)
- [`arduino-cli`](https://arduino.github.io/arduino-cli/) with the `arduino:mbed_nano` core
- Python 3 with `pyserial` and `websockets`
- A Chromium-based browser
