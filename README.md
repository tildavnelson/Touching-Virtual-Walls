# Touching Virtual Walls

A handheld kinetic object that traces invisible virtual walls through movement. A performer carries the object; its position and orientation are tracked; a virtual counterpart in TouchDesigner computes how four servo-driven sticks should open to trace the planes of the virtual walls, and sends those angles back to the object in real time.

The object senses its pose, publishes it to an MQTT broker, and TouchDesigner mirrors it in a virtual scene, calculates each stick's angle against the virtual walls, and publishes the angles back for the ESP32 to drive the servos — a closed loop between the physical object and its virtual model.

 **Outdoor UWB demo:** https://youtu.be/p2YoM8ZEzS0

This repository contains the firmware, TouchDesigner project, scripts, 3D models, and setup details needed to reproduce the system. It accompanies the dissertation *Touching Virtual Walls* and documents the first (UWB) prototype in full technical detail.

---

## System overview

Sensors → ESP32 (fusion) → MQTT → TouchDesigner (compute angles) → MQTT → ESP32 → servos

<img width="1101" height="447" alt="Device data flow-flow chart" src="https://github.com/user-attachments/assets/dcc2ae06-0719-4d52-bce8-58254b03f490" />

1. **Recording the physical.** The ESP32-S3 reads UWB position (DWM1001C) and IMU orientation (BNO055), rejects implausible readings, and fuses position through per-axis Kalman filters.
2. **Physical puppeteering the virtual.** The ESP32 publishes the fused pose as JSON over MQTT.
3. **Mirroring.** TouchDesigner subscribes to the pose, drives an identical virtual object in a 3D scene, and computes each stick's angle against the virtual wall planes.
4. **Virtual puppeteering the physical.** TouchDesigner publishes the four stick angles over MQTT; the ESP32 maps them to PWM and actuates the servos.

The canonical implementation of stages 1, 2 and 4 is `firmware/combined_firmware.ino`; stage 3 lives in the TouchDesigner project. The sections below explain what the code does, with the exact bytes, constants and message formats used.

---

## Hardware

| Component | Role |
|---|---|
| ESP32-S3 Zero | Reads sensors, runs fusion, drives servos, handles Wi-Fi/MQTT |
| DWM1001-DEV (×5) | UWB tracking — 1 tag on the object, 4 anchors in the space |
| BNO055 | 9-axis IMU — orientation |
| PCA9685 | 16-channel PWM driver for the servos |
| SG92R (×4) | Micro servos actuating the four sticks |
| Power | 5 V bank for logic; 4×AA (6 V) for servos; common ground |

**Wiring**

<img width="2036" height="1531" alt="skematics uwb drawio" src="https://github.com/user-attachments/assets/a77911ca-e21a-4f88-8f7a-9fc9b1419639" />


- I²C (BNO055 at `0x28` + PCA9685 at `0x40`): SDA = GPIO8, SCL = GPIO9 (`Wire.begin(8, 9)`)
- UART to the UWB tag: RX = GPIO44, TX = GPIO43, 115200 baud, 8N1 (`DWM.begin(115200, SERIAL_8N1, 44, 43)`)
- Servo channels on the PCA9685: N = 0, E = 3, S = 2, W = 1
- A common ground across both power sources is essential.

The BNO055 is initialised with axis remap `P1` and the external crystal enabled; the PCA9685 runs at 50 Hz.

> The Vive-tracked exhibition prototype (Prototype 2) replaces the UWB/IMU pose feed with a SteamVR CHOP in TouchDesigner; the servo-actuation half of the firmware is unchanged. .

<img width="1356" height="709" alt="Screenshot 2026-08-19 202649" src="https://github.com/user-attachments/assets/916d75ba-eb57-4da3-8a1b-7bf5f642b8bb" />


## Firmware (`/firmware`)

`combined_firmware.ino` — runs on the ESP32-S3.

**Before flashing, set credentials and topics at the top** (blank in the repo):

```cpp
const char* ssid         = "YOUR_SSID";
const char* password     = "YOUR_PASSWORD";
const char* mqtt_server  = "YOUR_BROKER_IP";
const char* pose_topic   = "YOUR_POSE_TOPIC";     // object → TouchDesigner
const char* petals_topic = "YOUR_STICKS_TOPIC";   // TouchDesigner → object
```

The main loop runs four things on independent timers: a fusion update every 20 ms, a DWM position request every 100 ms, a pose publish every 100 ms, and continuous MQTT servicing. The sections below correspond one-to-one to the pipeline stages.

### 1. Requesting position from the DWM1001C Dev board

Every 100 ms the ESP32 wakes the DWM1001 and asks for a position report over UART:

write 0xFF // wake byte — brings the DWM1001C out of low-power sleep
delay 10 ms
write 0x0C 0x00 // "get location" request


Incoming bytes are buffered (up to 256 bytes), and the buffer is parsed once it has had time to arrive (80 ms after the request). The DWM1001C replies with a **TLV (Type–Length–Value)** packet.

### 2. Parsing the TLV packet

The parser scans the buffer for the position record, identified by type byte `0x41` with length `0x0D` (13 bytes):

- **Position** — `x, y, z` as signed 32-bit little-endian integers in **millimetres**, followed by a **position quality factor** `qf` (0–100).
- **Per-anchor distances** — introduced by a distance type byte (`0x49` or `0x48`) and a count, then one record per anchor. Each record carries the anchor **address** (2 bytes), the **raw distance** in millimetres (4 bytes) and a **distance quality factor** (1 byte). Up to 15 anchors are read (`MAX_ANCHORS`).

The two distance types differ only in record stride, so both layouts are handled explicitly.

**Sanity gate.** A reading is only accepted if it falls inside the expected room volume — roughly `x ∈ (−5, 15) m`, `y ∈ (−5, 15) m`, `z ∈ (−5, 10) m`. Anything outside this is discarded rather than fused, which rejects gross multilateration errors before they reach the filter.

### 3. Reading orientation (BNO055)

In parallel, every 20 ms the ESP32 reads the BNO055 over I²C, taking the fused **quaternion** (`qw, qx, qy, qz`), the **Euler angles** (heading, pitch, roll in degrees), and the **calibration status** (`sys, gyro, accel, mag`, each 0–3). It also reads the linear-acceleration and gyroscope vectors, which drive the stillness detection used by the filter.

### 4. Stabilising: adaptive Kalman fusion

The IMU is precise but drifts; the UWB is stable but jitters. Position is fused through **three independent 1-D Kalman filters** (one per axis), whose noise terms adapt every 20 ms to how the object is moving and to how good the UWB reading is.

**Stillness detection.** A sample counts as "still" when acceleration magnitude < `0.08 m/s²` **and** gyroscope magnitude < `3.0 °/s`. A vote counter debounces this over `5` samples so brief spikes don't flip the state.

**Process noise Q** (how much the filter trusts new UWB readings):

- **Still** → `Q` is pinned low (`Q_MIN = 0.002`, and `Q_BASE_Z = 0.004` for the noisier vertical axis) to suppress drift and hold the object steady.
- **Moving** → `Q` scales up with motion magnitude toward `Q_MAX = 0.15`, so the estimate can follow fast movement without lag.

**Measurement noise R** (how much a given UWB reading is trusted) is derived inversely from the position quality factor: high quality drives `R` down toward `R_MIN = 0.3` (trust the reading), low quality drives it up toward `R_MAX = 3.0` (discount it).

| Parameter | Value | Meaning |
|---|---|---|
| `Q_MIN` | 0.002 | Process noise when still (X, Y) |
| `Q_BASE_Z` | 0.004 | Process noise when still (Z) |
| `Q_MAX` | 0.15 | Process noise ceiling when moving |
| `R_MIN` | 0.3 | Measurement noise at best UWB quality |
| `R_MAX` | 3.0 | Measurement noise at worst UWB quality |
| stillness | accel < 0.08 m/s², gyro < 3.0 °/s | held for 5 samples |

The filter outputs a fused position (`raw_x, raw_y, raw_z`) that is steady at rest and responsive in motion.

### 5. Per-anchor calibration

Each DWM1001C chip has a device-specific range bias, largest at short range and flattening beyond ~2.5 m. Because the DWM1001C firmware exposes no direct antenna-delay setting, calibration is applied on the ESP32 instead. Each anchor was placed at known laser-measured distances, the reported values compared against truth, and a linear correction fitted per anchor address:

```cpp
float calibrateAnchorDist(uint16_t addr, float dist_m) {
  switch (addr) {
    case 0x169A: return (dist_m * 0.9733f) + 0.0901f;
    case 0x41A5: return (dist_m * 0.9781f) + 0.0789f;
    case 0x179E: return (dist_m * 0.9634f) + 0.0984f;
    default:     return dist_m;   // uncalibrated anchors pass through unchanged
  }
}
```

The calibrated per-anchor distance is stored and published alongside the raw value, so the correction can be inspected downstream. (Coefficients are specific to these anchors; re-fit for a different set.)

### 6. Publishing the pose (MQTT)

Every 100 ms the ESP32 serialises the fused pose to JSON and publishes it to `pose_topic`. Full schema:

| Field | Meaning |
|---|---|
| `x, y, z` | Fused position (metres) |
| `q` | DWM1001C position quality factor (0–100) |
| `qw, qx, qy, qz` | BNO055 orientation quaternion |
| `heading, pitch, roll` | BNO055 Euler angles (degrees) |
| `cal` | BNO055 system calibration status (0–3) |
| `still` | Stillness state (boolean) |
| `q_xy, r` | Current Kalman process/measurement noise (diagnostic) |
| `t` | Device timestamp (`millis()`) |
| `anchors` | Number of anchors in this reading |
| `anch[]` | Per-anchor array: `addr`, `raw` (mm), `cal` (m), `qf` |

**Worked example** (one real packet):

```json
{"x":3.413633,"y":2.822842,"z":1.115937,"q":60,
 "qw":-0.036926,"qx":-0.117981,"qy":0.213318,"qz":-0.969116,
 "heading":184.3125,"pitch":14.125,"roll":24.625,
 "cal":3,"t":118761,"still":false,"q_xy":0.049244,"r":1.353,
 "anchors":4,
 "anch":[{"addr":6046,"raw":2281,"cal":2.295915,"qf":100},
         {"addr":16805,"raw":3091,"cal":3.102207,"qf":100},
         {"addr":54790,"raw":4492,"cal":4.492,"qf":100},
         {"addr":5786,"raw":4590,"cal":4.557547,"qf":100}]}
```

### 7. Receiving stick angles and driving the servos

The ESP32 subscribes to `petals_topic`. Incoming messages are JSON with keys `"n"`, `"e"`, `"s"`, `"w"`, each an open angle in degrees; the callback calls `setPetal()` for each stick present.

The conversion from open angle to servo pulse happens in two steps:

- **Open angle → servo angle.** The requested open angle is clamped to `0–150°`, then mapped so that `0°` open is the closed rest position and larger values open the petal: `servo_angle = SERVO_CLOSED − open` (with `SERVO_CLOSED = 180°`, `SERVO_OPEN = 30°`).
- **Servo angle → PWM.** The servo angle (`0–180°`) is mapped linearly to the PCA9685 pulse-count range `SERVOMIN = 150 … SERVOMAX = 600` and written to the stick's channel.

| Stick | PCA9685 channel |
|---|---|
| North | 0 |
| East | 3 |
| South | 2 |
| West | 1 |

The four SG92R servos actuate together as the performer moves the tracked prop, producing a real-time kinetic response to the invisible virtual boundaries.

---

## TouchDesigner (`/touchdesigner`)

- `VirtualWall.toe` — the full project
- `scriptangles.py` — per-frame stick-angle calculation (readable copy)
- `mqtt_callback.py` — parses incoming pose JSON to storage

### Mirroring the physical

The MQTT Client DAT subscribes to the pose topic. On receiving the fused pose JSON, the callback parses position and orientation into storage, storing position **only when the quality factor is ≥ 30** and always updating orientation (the IMU is stable). A geometry SOP holds an identical 3D model of the tracked prop and its four servo-driven sticks (north, south, east, west); a transform operator applies the received position and orientation to the whole geometry, placing and rotating the model in the virtual scene.

### Feeling the wall: stick-angle calculation

`scriptangles.py` runs every frame. For each of the four sticks independently, it:

1. Defines the stick's pivot point relative to the tracked object's local coordinate frame.
2. Sweeps every angle from −160° to +160° in 1° steps.
3. For each angle, computes the 3D world position of the stick's tip, accounting for the object's current BNO055-derived rotation.
4. Measures the distance from the tip to each of the three virtual planes.
5. Picks the angle that minimises the distance to the nearest plane.
6. Applies hysteresis — once engaged (distance < 0.10 m), the stick needs 0.15 m of clearance before disengaging — to prevent oscillation.
7. Outputs the final angle, clamped to the servo range.

The angles are written to the `sticksval` table and published to the sticks topic.

### Key structure

- Position is smoothed through DAT-to-CHOP → resample → lag before use.
- The four stick transforms and the body transform read from `sticksval`, **decoupled from the script** to avoid a cook-dependency loop (the loop that otherwise capped the frame rate; see Limitations).

> **Axis note.** UWB is Z-up, TouchDesigner is Y-up; the Y and Z axes are inverted between the DRTLS/Decawave frame and TouchDesigner and must be remapped.

---

## UWB anchor setup (`/setup`)

1. DWM1001-DEV boards ship blank. Flash each with `DWM1001_PANS_R2.0.hex` using SEGGER J-Flash Lite over USB/SWD (this programs the onboard nRF52832): connect → Erase Chip → Program Device.
2. Configure each module as a tag or an anchor in the Decawave DRTLS Manager app (over Bluetooth LE). **One anchor must be set as the network initiator**, or the network will not form.
3. Use 4 anchors (the minimum for stable 3D positioning), two mounted high and two low so height (Z) can be resolved. Keep them off metal.
4. Measure each anchor's XYZ with a laser measurer; Anchor 1 = origin (0,0), Anchor 2 along the X-axis, the rest following in order. Enter the coordinates in the app — these form the reference frame, so any measurement error propagates into every reading.

**Anchor coordinates used:** `[your values]`

---

## 3D models (`/3d-models`)

- `[list your STL / Fusion files]`
- Tag/IMU enclosure modified from *DWM1001 Case (Oxidize Conf Edition)*, Cults3D — `[link]`. Check the licence before redistributing modified files.

---

## MQTT topics

- Pose (object → TouchDesigner): `[your pose topic]`
- Sticks (TouchDesigner → object): `[your sticks topic]`

---

## Media

- **Outdoor UWB recording:** https://youtu.be/p2YoM8ZEzS0
- **BodyLab composited performance:** https://youtu.be/O6ULjkn_9qE

---

## Notes / known limitations

- The TouchDesigner frame rate reached ~10 fps after decoupling the cook loop; a more efficient solution is possible.
- Corners where two walls meet are not fully resolved — the interaction jumps when two walls are close and the script cannot reliably pick the nearest.
- UWB accuracy degrades near metal, near bodies, and below ~2.5 m tag-to-anchor range; it was markedly better outdoors than indoors ([outdoor recording](https://youtu.be/p2YoM8ZEzS0)).

### Planned improvements

- Prototype 2: mount the servos directly on the cog to improve the mechanical linkage.
