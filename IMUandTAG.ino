#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

const char* ssid = "";
const char* password = "";
const char* mqtt_server = "";
const int mqtt_port = 1883;
const char* pose_topic   = "";
const char* petals_topic = "";

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
HardwareSerial DWM(1);

WiFiClient espClient;
PubSubClient client(espClient);

#define SERVOMIN 150
#define SERVOMAX 600
#define SERVO_CLOSED 180
#define SERVO_OPEN   30
#define CH_N 0
#define CH_E 3
#define CH_S 2
#define CH_W 1

unsigned long lastReq = 0;
unsigned long lastPublish = 0;
unsigned long lastFusionUpdate = 0;

const float STILL_ACCEL_THRESH   = 0.08f;
const float STILL_GYRO_THRESH    = 3.0f;
const int   STILL_DEBOUNCE_COUNT = 5;

const float Q_MIN = 0.002f;
const float Q_MAX = 0.15f;
const float Q_BASE_Z = Q_MIN * 2.0f;

const float R_MIN = 0.3f;
const float R_MAX = 3.0f;

float raw_x = 0, raw_y = 0, raw_z = 0;
int pos_q = 0;

#define MAX_ANCHORS 15
struct AnchorReading {
  uint16_t addr;
  uint32_t dist_mm;
  float    dist_cal_m;
  uint8_t  dist_qf;
  bool     valid;
};
AnchorReading anchors[MAX_ANCHORS];
uint8_t anchor_count = 0;

float current_q_xy = Q_MIN;
float current_q_z  = Q_BASE_Z;
float current_r    = 1.0f;

struct Kalman {
  float x, p, q, r, k;
};
Kalman kx = {0, 3, Q_MIN, 1.0, 0};
Kalman ky = {0, 3, Q_MIN, 1.0, 0};
Kalman kz = {0, 3, Q_BASE_Z, 1.0, 0};

float kalmanUpdate(Kalman &k, float measurement, float q_override, float r_override) {
  k.q = q_override;
  k.r = r_override;
  k.p = k.p + k.q;
  k.k = k.p / (k.p + k.r);
  k.x = k.x + k.k * (measurement - k.x);
  k.p = (1 - k.k) * k.p;
  return k.x;
}

bool isStill = false;
int stillVoteCounter = 0;

uint8_t buf[256];
int bufLen = 0;

float calibrateAnchorDist(uint16_t addr, float dist_m) {
  switch (addr) {
    case 0x169A: return (dist_m * 0.9733f) + 0.0901f;
    case 0x41A5: return (dist_m * 0.9781f) + 0.0789f;
    case 0x179E: return (dist_m * 0.9634f) + 0.0984f;
    default:     return dist_m;
  }
}

void setServoAngle(uint8_t channel, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void setPetal(uint8_t channel, float openDeg) {
  if (openDeg < 0) openDeg = 0;
  if (openDeg > 150) openDeg = 150;
  int dir = (SERVO_OPEN >= SERVO_CLOSED) ? 1 : -1;
  int angle = SERVO_CLOSED + dir * (int)openDeg;
  setServoAngle(channel, angle);
}

void setup_wifi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500); Serial.print("."); attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    Serial.println("MAC: " + WiFi.macAddress());
  } else {
    Serial.println("\nWiFi FAILED");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT msg on "); Serial.println(topic);
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) {
    Serial.println("JSON parse failed");
    return;
  }
  if (doc.containsKey("n")) setPetal(CH_N, doc["n"].as<float>());
  if (doc.containsKey("e")) setPetal(CH_E, doc["e"].as<float>());
  if (doc.containsKey("s")) setPetal(CH_S, doc["s"].as<float>());
  if (doc.containsKey("w")) setPetal(CH_W, doc["w"].as<float>());
}

void reconnect_mqtt() {
  if (client.connected()) return;
  unsigned long now = millis();
  static unsigned long lastAttempt = 0;
  if (now - lastAttempt < 5000) return;
  lastAttempt = now;

  IPAddress resolvedIP;
  if (WiFi.hostByName(mqtt_server, resolvedIP)) {
    Serial.print("Resolved to: ");
    Serial.println(resolvedIP);
  } else {
    Serial.println("DNS resolution FAILED");
  }

  Serial.println("Attempting MQTT connection...");
  if (client.connect("ESP32S3Zero_tilda")) {
    Serial.println("MQTT connected");
    client.subscribe(petals_topic);
  } else {
    Serial.print("MQTT failed, rc="); Serial.println(client.state());
  }
}

void parseTLV(uint8_t* buf, int len) {
  anchor_count = 0;
  for (int i = 0; i < len - 14; i++) {
    if (buf[i] == 0x41 && buf[i+1] == 0x0D) {
      int32_t x = buf[i+2]  | (buf[i+3]<<8)  | (buf[i+4]<<16)  | (buf[i+5]<<24);
      int32_t y = buf[i+6]  | (buf[i+7]<<8)  | (buf[i+8]<<16)  | (buf[i+9]<<24);
      int32_t z = buf[i+10] | (buf[i+11]<<8) | (buf[i+12]<<16) | (buf[i+13]<<24);
      uint8_t q = buf[i+14];

      pos_q = q;

      int cursor = i + 15;
      if (cursor + 2 < len) {
        uint8_t distType = buf[cursor];
        uint8_t cnt = buf[cursor + 2];
        cursor += 3;

        if (cnt > MAX_ANCHORS) cnt = MAX_ANCHORS;

        if (distType == 0x49) {
          for (uint8_t a = 0; a < cnt; a++) {
            if (cursor + 20 > len) break;
            uint16_t addr = buf[cursor] | (buf[cursor+1] << 8);
            cursor += 2;
            uint32_t dist = buf[cursor] | (buf[cursor+1]<<8) | (buf[cursor+2]<<16) | (buf[cursor+3]<<24);
            cursor += 4;
            uint8_t dqf = buf[cursor];
            cursor += 1;
            anchors[a].addr       = addr;
            anchors[a].dist_mm    = dist;
            anchors[a].dist_qf    = dqf;
            anchors[a].valid      = true;
            anchors[a].dist_cal_m = calibrateAnchorDist(addr, dist / 1000.0f);
            cursor += 13;
          }
          anchor_count = cnt;
        } else if (distType == 0x48) {
          for (uint8_t a = 0; a < cnt; a++) {
            if (cursor + 13 > len) break;
            uint16_t addr = buf[cursor] | (buf[cursor+1] << 8);
            cursor += 8;
            uint32_t dist = buf[cursor] | (buf[cursor+1]<<8) | (buf[cursor+2]<<16) | (buf[cursor+3]<<24);
            cursor += 4;
            uint8_t dqf = buf[cursor];
            cursor += 1;
            anchors[a].addr       = addr;
            anchors[a].dist_mm    = dist;
            anchors[a].dist_qf    = dqf;
            anchors[a].valid      = true;
            anchors[a].dist_cal_m = calibrateAnchorDist(addr, dist / 1000.0f);
          }
          anchor_count = cnt;
        }
      }

      float mx = x / 1000.0f;
      float my = y / 1000.0f;
      float mz = z / 1000.0f;

      if (mx > -5.0f && mx < 15.0f &&
          my > -5.0f && my < 15.0f &&
          mz > -5.0f && mz < 10.0f) {
        raw_x = kalmanUpdate(kx, mx, current_q_xy, current_r);
        raw_y = kalmanUpdate(ky, my, current_q_xy, current_r);
        raw_z = kalmanUpdate(kz, mz, current_q_z,  current_r);
      }
    }
  }
}

float qualityToR(uint8_t qf) {
  float frac = qf / 100.0f;
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  return R_MAX - frac * (R_MAX - R_MIN);
}

float motionToQ(float motionMagnitude, float qMin, float qMax, float motionCeiling) {
  float frac = motionMagnitude / motionCeiling;
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  return qMin + frac * (qMax - qMin);
}

void updateFusion() {
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro     = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float accelMag = sqrt(linAccel.x()*linAccel.x() + linAccel.y()*linAccel.y() + linAccel.z()*linAccel.z());
  float gyroMag  = sqrt(gyro.x()*gyro.x() + gyro.y()*gyro.y() + gyro.z()*gyro.z());

  bool sampleLooksStill = (accelMag < STILL_ACCEL_THRESH) && (gyroMag < STILL_GYRO_THRESH);

  if (sampleLooksStill) {
    stillVoteCounter++;
    if (stillVoteCounter > STILL_DEBOUNCE_COUNT) stillVoteCounter = STILL_DEBOUNCE_COUNT;
  } else {
    stillVoteCounter--;
    if (stillVoteCounter < -STILL_DEBOUNCE_COUNT) stillVoteCounter = -STILL_DEBOUNCE_COUNT;
  }

  if (stillVoteCounter >= STILL_DEBOUNCE_COUNT) {
    isStill = true;
  } else if (stillVoteCounter <= -STILL_DEBOUNCE_COUNT) {
    isStill = false;
  }

  if (isStill) {
    current_q_xy = Q_MIN;
    current_q_z  = Q_BASE_Z;
  } else {
    float motionMag = max(accelMag, gyroMag / 50.0f);
    current_q_xy = motionToQ(motionMag, Q_MIN, Q_MAX, 2.0f);
    current_q_z  = motionToQ(motionMag, Q_BASE_Z, Q_MAX, 2.0f);
  }

  current_r = qualityToR((uint8_t)pos_q);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(8, 9);
  if (!bno.begin()) {
    Serial.println("BNO055 not found");
  } else {
    bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
    bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);
    bno.setExtCrystalUse(true);
    Serial.println("BNO055 ready");
  }
  pwm.begin();
  pwm.setPWMFreq(50);
  setServoAngle(CH_N, SERVO_CLOSED);
  setServoAngle(CH_E, SERVO_CLOSED);
  setServoAngle(CH_S, SERVO_CLOSED);
  setServoAngle(CH_W, SERVO_CLOSED);
  DWM.begin(115200, SERIAL_8N1, 44, 43);
  delay(2000);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(1024);
}

void loop() {
  if (!client.connected()) reconnect_mqtt();
  client.loop();

  if (millis() - lastFusionUpdate > 20) {
    updateFusion();
    lastFusionUpdate = millis();
  }

  if (millis() - lastReq > 100) {
    DWM.write(0xFF); delay(10);
    uint8_t req[2] = {0x0C, 0x00};
    DWM.write(req, 2);
    lastReq = millis(); bufLen = 0;
  }
  while (DWM.available()) {
    byte b = DWM.read();
    if (bufLen < 256) buf[bufLen++] = b;
  }

  if (bufLen > 0 && millis() - lastReq > 80) {
    parseTLV(buf, bufLen);
    Serial.printf("pos: x=%.2f y=%.2f z=%.2f qf=%d | anchors=%d\n",
      raw_x, raw_y, raw_z, pos_q, anchor_count);

    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    uint8_t sys, gyro_cal, accel_cal, mag_cal;
    bno.getCalibration(&sys, &gyro_cal, &accel_cal, &mag_cal);
    Serial.printf("imu: h=%.1f p=%.1f r=%.1f | qw=%.3f qx=%.3f qy=%.3f qz=%.3f | cal sys=%d gyro=%d accel=%d mag=%d\n",
      euler.x(), euler.y(), euler.z(),
      quat.w(), quat.x(), quat.y(), quat.z(),
      sys, gyro_cal, accel_cal, mag_cal);

    for (int a = 0; a < anchor_count; a++) {
      Serial.printf("  anchor %d: addr=0x%04X raw=%dmm cal=%.3fm qf=%d\n",
        a, anchors[a].addr, anchors[a].dist_mm, anchors[a].dist_cal_m, anchors[a].dist_qf);
    }
  }

  if (millis() - lastPublish > 100) {
    lastPublish = millis();
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    StaticJsonDocument<1024> doc;
    doc["x"]=raw_x; doc["y"]=raw_y; doc["z"]=raw_z; doc["q"]=pos_q;
    doc["qw"]=quat.w(); doc["qx"]=quat.x(); doc["qy"]=quat.y(); doc["qz"]=quat.z();
    doc["heading"]=euler.x(); doc["pitch"]=euler.y(); doc["roll"]=euler.z();
    doc["cal"]=sys; doc["t"]=millis();
    doc["still"]=isStill;
    doc["q_xy"]=current_q_xy; doc["r"]=current_r;
    doc["anchors"]=anchor_count;

    JsonArray anch = doc.createNestedArray("anch");
    for (int a = 0; a < anchor_count; a++) {
      JsonObject o = anch.createNestedObject();
      o["addr"] = anchors[a].addr;
      o["raw"]  = anchors[a].dist_mm;
      o["cal"]  = anchors[a].dist_cal_m;
      o["qf"]   = anchors[a].dist_qf;
    }

    char payload[1024];
    serializeJson(doc, payload);
    client.publish(pose_topic, payload);
  }
}