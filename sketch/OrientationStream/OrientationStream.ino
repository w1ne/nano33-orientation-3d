/*
 * OrientationStream - Arduino Nano 33 BLE (LSM9DS1)
 *
 * Reads the on-board accelerometer + gyroscope, fuses them with a Madgwick
 * filter into an absolute orientation quaternion, and streams it over USB
 * serial as a CSV line:
 *
 *     q0,q1,q2,q3,gyroMag\n
 *
 *   q0..q3  : orientation quaternion (w, x, y, z), unit length
 *   gyroMag : magnitude of angular rate in deg/s (stability indicator;
 *             ~0 when the board is held still, large while moving)
 *
 * 6-axis fusion (no magnetometer): roll & pitch are absolute and rock
 * stable; yaw is relative and may drift slowly. Add the magnetometer later
 * (with hard/soft-iron calibration) for an absolute compass heading.
 */

#include <Arduino_BMI270_BMM150.h>   // Nano 33 BLE Rev2 IMU (same IMU.* API as LSM9DS1)

// --- Madgwick filter state ---
static float beta = 0.1f;                  // filter gain
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static unsigned long lastMicros = 0;

static float invSqrt(float x) { return 1.0f / sqrtf(x); }

// 6-axis (gyro + accel) Madgwick update. gyro in rad/s, accel in any unit.
void madgwickUpdateIMU(float gx, float gy, float gz,
                       float ax, float ay, float az, float dt) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
  float q0q0, q1q1, q2q2, q3q3;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    _2q0 = 2 * q0; _2q1 = 2 * q1; _2q2 = 2 * q2; _2q3 = 2 * q3;
    _4q0 = 4 * q0; _4q1 = 4 * q1; _4q2 = 4 * q2;
    _8q1 = 8 * q1; _8q2 = 8 * q2;
    q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4 * q1q1 * q3 - _2q1 * ax + 4 * q2q2 * q3 - _2q2 * ay;

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
  }

  q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) { /* wait briefly for host */ }

  if (!IMU.begin()) {
    while (1) { Serial.println("ERR: LSM9DS1 init failed"); delay(1000); }
  }
  lastMicros = micros();
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  bool haveA = false, haveG = false;

  if (IMU.accelerationAvailable()) { IMU.readAcceleration(ax, ay, az); haveA = true; }
  if (IMU.gyroscopeAvailable())    { IMU.readGyroscope(gx, gy, gz);    haveG = true; }

  if (haveA && haveG) {
    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.2f) dt = 0.01f;   // guard against stalls

    float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);   // deg/s

    const float DEG2RAD = 0.01745329252f;
    madgwickUpdateIMU(gx * DEG2RAD, gy * DEG2RAD, gz * DEG2RAD, ax, ay, az, dt);

    // Throttle serial output to ~50 Hz
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 20000) {
      lastPrint = now;
      Serial.print(q0, 5); Serial.print(',');
      Serial.print(q1, 5); Serial.print(',');
      Serial.print(q2, 5); Serial.print(',');
      Serial.print(q3, 5); Serial.print(',');
      Serial.println(gyroMag, 2);
    }
  }
}
