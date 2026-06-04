/* RawStream - dumps raw accel(g), gyro(dps), mag(uT) as CSV for analysis:
 *   ax,ay,az,gx,gy,gz,mx,my,mz
 * Used to determine magnetometer axis alignment + hard-iron calibration before
 * building 9-axis fusion. Nano 33 BLE Rev2 (BMI270 + BMM150). */
#include <Arduino_BMI270_BMM150.h>

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}
  if (!IMU.begin()) { while (1) { Serial.println("ERR: IMU init failed"); delay(1000); } }
  Serial.println("# ax,ay,az,gx,gy,gz,mx,my,mz");
}

void loop() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  static float lax, lay, laz, lgx, lgy, lgz, lmx, lmy, lmz;  // last-known
  if (IMU.accelerationAvailable())  IMU.readAcceleration(lax, lay, laz);
  if (IMU.gyroscopeAvailable())     IMU.readGyroscope(lgx, lgy, lgz);
  bool m = false;
  if (IMU.magneticFieldAvailable()) { IMU.readMagneticField(lmx, lmy, lmz); m = true; }

  // Stream at ~50 Hz, only emit fresh rows when mag updates (slowest sensor).
  static unsigned long last = 0;
  if (m && micros() - last >= 20000) {
    last = micros();
    Serial.print(lax,3); Serial.print(','); Serial.print(lay,3); Serial.print(','); Serial.print(laz,3); Serial.print(',');
    Serial.print(lgx,2); Serial.print(','); Serial.print(lgy,2); Serial.print(','); Serial.print(lgz,2); Serial.print(',');
    Serial.print(lmx,2); Serial.print(','); Serial.print(lmy,2); Serial.print(','); Serial.println(lmz,2);
  }
}
