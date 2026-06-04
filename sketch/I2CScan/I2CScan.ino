/* I2C scanner for Nano 33 BLE on-board sensors.
 * Enables sensor power rail + pull-ups, then scans the internal I2C bus (Wire1)
 * and the external bus (Wire). Identifies which IMU is fitted.
 *   LSM9DS1  -> 0x6B (accel/gyro) + 0x1E (mag)
 *   BMI270   -> 0x68,  BMM150 -> 0x10   (Rev2 boards)
 */
#include <Wire.h>

void scan(TwoWire &bus, const char *name) {
  Serial.print("Scanning "); Serial.print(name); Serial.println(":");
  int found = 0;
  for (byte a = 1; a < 127; a++) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) {
      Serial.print("  found 0x");
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  (none)");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}

#if defined(PIN_ENABLE_SENSORS_3V3)
  pinMode(PIN_ENABLE_SENSORS_3V3, OUTPUT);
  digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
#endif
#if defined(PIN_ENABLE_I2C_PULLUP)
  pinMode(PIN_ENABLE_I2C_PULLUP, OUTPUT);
  digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);
#endif
  delay(500);

  Wire.begin();
  Wire1.begin();
}

void loop() {
  scan(Wire1, "Wire1 (internal)");
  scan(Wire, "Wire (external)");
  Serial.println("----");
  delay(2000);
}
