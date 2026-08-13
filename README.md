# Touching-Virtual-Walls

DWM1001C Position Data 
Every 100 milliseconds, the ESP32-S3 Zero initiates a DWM1001C positioning request via serial communication. The ESP32-S3 Zero sends a wake byte in binary ( this wakes the DWM1001C from low-power sleep mode), delays 10 milliseconds, then sends the positioning request command (0x0C, 0x00). The DWM1001C responds with a TLV (Type-Length-Value) formatted packet.
DWM1001C Serial Parsing and Calibration
The ESP32-S3 Zero parses the incoming TLV packet. Position data arrives as type 0x41 containing x, y, z as signed 32-bit integers (in millimetres) plus a quality factor (qf, 0–100). Distance data arrives as type 0x49 or type 0x48 with up to 15 anchors per re-sponse. Each anchor entry provides its address, the raw distance to it in millimetres, and a quality factor indicating how reliable that measurement is.



3.3 BNO055 Orientation Data
Simultaneously with DWM1001C polling, the ESP32-S3 Zero continuously reads the BNO055 IMU via I2C every 20 milliseconds. The BNO055 outputs euler angles (head-ing, pitch, roll in degrees) and quaternion data (qw, qx, qy, qz).
3.4 Stabalising  and Kalman Filter Fusion
Every 20 milliseconds, the ESP32-S3 Zero reads linear acceleration and gyroscopic vectors from the BNO055, computes their magnitudes, and determines if the object is still. If acceleration magnitude < 0.08 m/s² AND gyroscopic magnitude < 3.0°/s for 5 consecutive samples, the object is considered still. When still, the Kalman filter Q (process noise) is set to Q_MIN (0.002) to maximize trust in DWM1001C position and minimize drift. When moving, Q increases up to Q_MAX (0.15) based on motion magni-tude, allowing faster DWM1001C position corrections. The measurement noise R is inversely derived from DWM1001C quality factor (pos_q, 0–100): higher quality de-creases R (more trust in DWM1001C), lower quality increases R (up to R_MAX = 3.0).
The Kalman filters fuse DWM1001C position with BNO055 motion state, producing fused position (raw_x, raw_y, raw_z) and maintaining internal filter states (cur-rent_q_xy, current_r).

paper 2 (have to add here) fusing IMU and TAG data
I had to increase the accuracy of DWM1001C 
Massive challenge, DW1001C won’t allow you to change the delay in the chip , only can do this in binary, or by making custom code. An advantage I had is that all the data was going through the esp32, which is why I decided to calibrate the DWM1001 there. 
I measured each anchor at set distances and watched in the serial monitor what would show, after getting a few seconds of values, I would compare the set distances using a laser measurer to the values that would show up and compare them. What I found as expected from previous research, was that each DWM1001C chip has it’s own specific delay. And that the closer to the anchor the more distortion, up until 2.5 meters ish, where the distortion flattens out.. I then added this delay into my esp32 calculation to be able to get higher accuracy. 
Raw distances are calibrated per anchor address using empirically derived linear coef-ficients:
Anchor 0x169A: calibrated_distance = (raw_distance × 0.9733) + 0.0901
Anchor 0x41A5: calibrated_distance = (raw_distance × 0.9781) + 0.0789
Anchor 0x179E: calibrated_distance = (raw_distance × 0.9634) + 0.0984
Calibrated position (x, y, z in metres) is extracted and fed into three separate Kalman filters (one for each axis).

-	This will be improved in second prototype,: Servos directly on cog. 

PHYSICAL PUPPETEERING THE VIRTUAL
 Publication to Mosquitto MQTT
Every 100 milliseconds, the ESP32-S3 Zero publishes the fused pose as JSON to Mos-quitto MQTT broker on topic student/dissertation/tilda/pose. The JSON packet 
-Fused position: x, y, z (metres)
-DWM1001C quality factor: q (0–100)
-BNO055 orientation: quaternion (qw, qx, qy, qz) and euler angles (heading, pitch, roll in degrees)
-BNO055 calibration status: sys, gyro, accel, mag (0–3 per axis)
-Motion state: still (boolean)
-Kalman filter state: q_xy, r
-Anchor metadata: count and per-anchor data (address, raw distance in mm, calibrated distance in m, quality factor)

With the vive…..

MIRRORING THE PHYSICAL AND VIRTUAL

TouchDesigner 
TouchDesigner's MQTT Client DAT subscribes to the Mosquitto pose topic. Upon re-ceiving the fused pose JSON, TouchDesigner parses position (x, y, z) and orientation (quaternion qw, qx, qy, qz or euler angles). 
A geometry SOP contains identical 3D model of the tracked prop and four servo-driven sticks (north, south, east, west). A transform operator applies the received position and orientation to the entire geometry, placing the model in 3D virtual space and rotat-ing it according to the live BNO055 data.
Screenshots of touchdesigner



FEELING THE WALL
Stick Angle Calculation
A Python script in TouchDesigner executes every frame. For each of the four sticks in-dividually, the script:
Defines the stick's pivot point relative to the tracked object's local coordinate frame
Iterates through all possible angles (−160° to +160° in 1° increments)
For each angle, calculates the 3D world position of the stick's tip, accounting for the object's current BNO055-derived rotation
Computes the distance from the stick tip to each of three virtual planes
Identifies the angle that minimizes distance to the nearest plane
Applies hysteresis: if the stick was already engaged (distance < 0.10m), requires 0.15m clearance before disengaging to prevent oscillation
Outputs the final angle for each stick (−160° to +160°, clamped to servo range)

THE VIRTUAL PUPPETEERING THE PHYSICAL

Stick Angles Publication to Mosquitto MQTT
TouchDesigner publishes the four calculated stick angles to Mosquitto topic stu-dent/dissertation/tilda/sticks as JSON with keys "n", "e", "s", "w", each containing an angle in degrees.

Sketch of the data flowing up to this point.

ESP32-S3 Zero Stick Angle Reception
The ESP32-S3 Zero subscribes to topic student/dissertation/tilda/sticks. When stick angle JSON arrives, the MQTT callback function is triggered, deserializing the JSON and calling setPetal() for each stick.
Angle-to-PWM Conversion and PCA9685 Control
setPetal() receives an angle (0–150°), constrains it to the valid range, then maps it to a PWM pulse width (SERVOMIN = 150 pulse counts to SERVOMAX = 600 pulse counts, corresponding to 0° to 180° servo rotation). The mapped pulse width is sent to the PCA9685 PWM driver (I2C address 0x40, 50Hz frequency) on the appropriate channel:
•	North stick: PCA9685 Channel 0
•	East stick: PCA9685 Channel 3
•	South stick: PCA9685 Channel 2
•	West stick: PCA9685 Channel 1

SG92R Servo Actuation
The PCA9685 driver sends PWM signals to four SG92R servos. The four servos actuate synchronously as the performer moves the DWM1001C-tracked prop, creating real-time kinetic response to invisible virtual boundaries.

