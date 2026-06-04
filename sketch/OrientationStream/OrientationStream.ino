/*
 * OrientationStream - Arduino Nano 33 BLE Rev2 (BMI270 + BMM150)
 *
 * 9-axis fusion with a MAHONY filter (proportional + integral). The magnetometer
 * gives an absolute compass heading so yaw does not drift; the integral term
 * continuously estimates and removes gyroscope bias (incl. post-fast-move
 * transients).
 *
 * Streams:  q0,q1,q2,q3,gyroMag\n   (quaternion w,x,y,z + angular rate deg/s)
 *
 * Magnetometer calibration (recovered empirically + validated against gravity,
 * std 0.048, gravity-field angle 23.4 deg matches local magnetic dip):
 *   hard-iron offsets in raw BMM150 axes, then remap raw BMM150 -> board frame:
 *   board_x=-my, board_y=+mx, board_z=+mz
 *
 * NOTE: the magnetometer needs a clean magnetic environment. Near a laptop,
 * motors, speakers, or metal the local field is distorted and yaw will follow
 * it. For best heading, run the board away from such sources.
 */

#include <Arduino_BMI270_BMM150.h>

// --- Magnetometer hard-iron offsets (raw BMM150 frame, uT) ---
static const float MX_OFF = -1.0f, MY_OFF = 0.5f, MZ_OFF = -15.5f;

// --- Mahony filter gains ---
static const float twoKp = 2.0f * 1.0f;    // 2 * proportional gain
static const float twoKi = 2.0f * 0.15f;   // 2 * integral gain (gyro-bias estimation)

// --- Filter state ---
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float integralFBx = 0, integralFBy = 0, integralFBz = 0;  // estimated gyro bias (rad/s)
static unsigned long lastMicros = 0;

// Startup gyro bias (deg/s) -- a good initial guess; Mahony's integral refines it live.
static float gxBias = 0, gyBias = 0, gzBias = 0;

static float invSqrt(float x) { return 1.0f / sqrtf(x); }

// 6-axis Mahony update (fallback when magnetometer is unavailable). gyro in rad/s.
void mahonyUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm, halfvx, halfvy, halfvz, halfex, halfey, halfez, qa, qb, qc;
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az); ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt; integralFBy += twoKi * halfey * dt; integralFBz += twoKi * halfez * dt;
      gx += integralFBx; gy += integralFBy; gz += integralFBz;
    }
    gx += twoKp * halfex; gy += twoKp * halfey; gz += twoKp * halfez;
  }
  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
  qa = q0; qb = q1; qc = q2;
  q0 += -qb * gx - qc * gy - q3 * gz;
  q1 +=  qa * gx + qc * gz - q3 * gy;
  q2 +=  qa * gy - qb * gz + q3 * gx;
  q3 +=  qa * gz + qb * gy - qc * gx;
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// 9-axis Mahony AHRS update. gyro in rad/s; accel & mag any unit.
void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az,
                  float mx, float my, float mz, float dt) {
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) { mahonyUpdateIMU(gx, gy, gz, ax, ay, az, dt); return; }

  float recipNorm, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz, halfvx, halfvy, halfvz, halfwx, halfwy, halfwz, halfex, halfey, halfez, qa, qb, qc;

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az); ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
    recipNorm = invSqrt(mx * mx + my * my + mz * mz); mx *= recipNorm; my *= recipNorm; mz *= recipNorm;

    q0q0 = q0 * q0; q0q1 = q0 * q1; q0q2 = q0 * q2; q0q3 = q0 * q3;
    q1q1 = q1 * q1; q1q2 = q1 * q2; q1q3 = q1 * q3; q2q2 = q2 * q2; q2q3 = q2 * q3; q3q3 = q3 * q3;

    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    if (twoKi > 0.0f) {
      integralFBx += twoKi * halfex * dt; integralFBy += twoKi * halfey * dt; integralFBz += twoKi * halfez * dt;
      gx += integralFBx; gy += integralFBy; gz += integralFBz;
    }
    gx += twoKp * halfex; gy += twoKp * halfey; gz += twoKp * halfez;
  }
  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
  qa = q0; qb = q1; qc = q2;
  q0 += -qb * gx - qc * gy - q3 * gz;
  q1 +=  qa * gx + qc * gz - q3 * gy;
  q2 +=  qa * gy - qb * gz + q3 * gx;
  q3 +=  qa * gz + qb * gy - qc * gx;
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) { /* wait briefly for host */ }
  if (!IMU.begin()) { while (1) { Serial.println("ERR: IMU init failed"); delay(1000); } }

  // Startup gyro bias: hold still ~2 s after reset.
  Serial.println("CAL: hold the board still...");
  const int N = 800; int got = 0; float bx = 0, by = 0, bz = 0;
  unsigned long t0 = millis();
  while (got < N && millis() - t0 < 5000) {
    if (IMU.gyroscopeAvailable()) { float gx, gy, gz; IMU.readGyroscope(gx, gy, gz); bx += gx; by += gy; bz += gz; got++; }
  }
  if (got > 0) { gxBias = bx / got; gyBias = by / got; gzBias = bz / got; }
  Serial.print("CAL: done, bias deg/s = ");
  Serial.print(gxBias, 3); Serial.print(','); Serial.print(gyBias, 3); Serial.print(','); Serial.println(gzBias, 3);
  lastMicros = micros();
}

void loop() {
  float ax, ay, az, gx, gy, gz, mxr, myr, mzr;
  static float mbx = 0, mby = 0, mbz = 0;     // magnetometer in board frame (last known)
  bool haveA = false, haveG = false;

  if (IMU.accelerationAvailable()) { IMU.readAcceleration(ax, ay, az); haveA = true; }
  if (IMU.gyroscopeAvailable())    { IMU.readGyroscope(gx, gy, gz);    haveG = true; }
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mxr, myr, mzr);
    float cmx = mxr - MX_OFF, cmy = myr - MY_OFF, cmz = mzr - MZ_OFF;  // hard-iron
    mbx = -cmy; mby = cmx; mbz = cmz;                                  // remap -> board frame
  }

  if (haveA && haveG) {
    float cx = gx - gxBias, cy = gy - gyBias, cz = gz - gzBias;   // remove startup bias

    unsigned long now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;
    if (dt <= 0 || dt > 0.2f) dt = 0.01f;

    float gyroMag = sqrtf(cx * cx + cy * cy + cz * cz);   // deg/s (stability metric)

    const float DEG2RAD = 0.01745329252f;
    mahonyUpdate(cx * DEG2RAD, cy * DEG2RAD, cz * DEG2RAD, ax, ay, az, mbx, mby, mbz, dt);

    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 20000) {
      lastPrint = now;
      Serial.print(q0, 5); Serial.print(','); Serial.print(q1, 5); Serial.print(',');
      Serial.print(q2, 5); Serial.print(','); Serial.print(q3, 5); Serial.print(',');
      Serial.println(gyroMag, 2);
    }
  }
}
