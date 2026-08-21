# Touching Virtual Walls

A handheld kinetic object that traces invisible virtual walls through movement. A performer carries the object; its position and orientation are tracked; a virtual counterpart in TouchDesigner computes how four servo-driven sticks should open to trace the planes of virtual walls, and sends those angles back to the object in real time.

This repository contains the firmware, TouchDesigner project, scripts, 3D models, and setup details needed to reproduce the system.

## System overview

Sensors → ESP32 (fusion) → MQTT → TouchDesigner (compute angles) → MQTT → ESP32 → servos

The object senses its pose, publishes it to an MQTT broker, TouchDesigner mirrors it in a virtual scene and calculates each stick's angle against the virtual walls, then publishes the angles back for the ESP32 to drive the servos — a closed loop between the physical object and its virtual model.

## Hardware

| Component | Role |
|---|---|
| ESP32-S3 Zero | Reads sensors, runs fusion, drives servos, handles WiFi/MQTT |
| DWM1001-DEV (x5) | UWB tracking — 1 tag on the object, 4 anchors in the space |
| BNO055 | 9-axis IMU — orientation |
| PCA9685 | 16-channel PWM driver for the servos |
| SG92R (x4) | Micro servos actuating the four sticks |
| Power | 5V bank for logic; 4xAA (6V) for servos; common ground |

**Wiring:**
- I2C (BNO055 + PCA9685): SDA = GPIO8, SCL = GPIO9
- UART (UWB tag): RX = GPIO44, TX = GPIO43
- Servo channels: N = 0, E = 3, S = 2, W = 1
- Common ground across both power sources is essential

## Firmware (`/firmware`)

`combined_firmware.ino` — runs on the ESP32-S3.

**Before flashing, set your own credentials at the top:**

    const char* ssid = "YOUR_SSID";
    const char* password = "YOUR_PASSWORD";
    const char* mqtt_server = "YOUR_BROKER_IP";

**What it does:**
- Requests position from the DWM1001 tag over UART (wake byte `0xFF`, then request `0x0C 0x00`), parses the returned TLV packet
- Reads the BNO055 over I2C every 20 ms (quaternion + Euler)
- Rejects implausible readings (outside room bounds), then fuses position through per-axis Kalman filters
- Fusion is adaptive: process noise Q drops when still (stillness = accel < 0.08 m/s2 AND gyro < 3.0 deg/s for 5 samples); measurement noise R scales with UWB quality
- Publishes pose JSON every 100 ms
- Subscribes to the sticks topic, maps incoming angles (0–150 deg) to PWM (150–600) and drives the servos

**Per-anchor calibration** (applied on the ESP32 before filtering):

    0x169A: cal = raw * 0.9733 + 0.0901
    0x41A5: cal = raw * 0.9781 + 0.0789
    0x179E: cal = raw * 0.9634 + 0.0984

Derived by placing the tag at known laser-measured distances and fitting a linear correction per anchor.

## TouchDesigner (`/touchdesigner`)

- `VirtualWall.toe` — the full project
- `scriptangles.py` — per-frame stick-angle calculation (readable copy)
- `mqtt_callback.py` — parses incoming pose JSON to storage

**Key structure:**
- MQTT Client DAT subscribes to the pose topic; callback stores position (when quality >= 30) and orientation
- Position smoothed through DAT-to-CHOP → resample → lag
- `scriptangles` sweeps each stick's angle range, finds where the tip is closest to the nearest bounded wall plane, applies hysteresis to prevent oscillation, writes angles to the `sticksval` table and publishes to the sticks topic
- The four stick transforms and the body transform read from `sticksval` (decoupled from the script to avoid a cook-dependency loop)

**Axis note:** UWB is Z-up, TouchDesigner is Y-up; the Y and Z axes are inverted between the DRTLS/Decawave frame and TouchDesigner and must be remapped.

## UWB anchor setup (`/setup`)

1. DWM1001-DEV boards ship blank. Flash each with `DWM1001_PANS_R2.0.hex` using SEGGER J-Flash Lite over USB/SWD (programs the onboard nRF52832): connect → Erase Chip → Program Device.
2. Configure each module as tag or anchor in the Decawave DRTLS Manager app (over Bluetooth LE). **One anchor must be set as the network initiator** or the network will not form.
3. Use 4 anchors (minimum for stable 3D positioning), two mounted high and two low so height (Z) can be resolved. Keep them off metal.
4. Measure each anchor's XYZ with a laser measurer; Anchor 1 = origin (0,0), Anchor 2 along the X-axis. Enter coordinates in the app — these form the reference frame, so measurement error propagates to every reading.

**Anchor coordinates used:** [your values]

## 3D models (`/3d-models`)

- [list your STL / Fusion files]
- Tag/IMU enclosure modified from *DWM1001 Case (Oxidize Conf Edition)*, Cults3D — [link]. Check licence before redistributing modified files.

## MQTT topics

- Pose (object → TD): `[your pose topic]`
- Sticks (TD → object): `[your sticks topic]`

## Notes / known limitations

- TouchDesigner frame rate reached ~10 fps after decoupling the cook loop; a more efficient solution is possible.
- Corners where two walls meet are not fully resolved.
- UWB accuracy degrades near metal, near bodies, and below ~2.5 m tag-to-anchor range.
