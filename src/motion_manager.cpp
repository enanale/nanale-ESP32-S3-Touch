#include "motion_manager.h"
#include "user_config.h"
#include <Arduino.h>
#include <Wire.h>

MotionManager::MotionManager()
    : lastX(0), lastY(0), lastZ(0), initialized(false) {}

bool MotionManager::begin() {
  // QMI8658 is on Wire1 (System Bus)
  // SDA is 47, SCL is 48
  if (!qmi.begin(Wire1, IMU_I2C_ADDR, PIN_I2C_SDA, PIN_I2C_SCL)) {
    Serial.println("[IMU] Failed to find QMI8658");
    return false;
  }

  uint8_t id = qmi.getChipID();
  Serial.print("[IMU] Found QMI8658, Chip ID: 0x");
  Serial.println(id, HEX);

  // Configure Accelerometer
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                          SensorQMI8658::ACC_ODR_1000Hz,
                          SensorQMI8658::LPF_MODE_0);
  qmi.enableAccelerometer();

  initialized = true;
  return true;
}

bool MotionManager::hasSignificantMotion() {
  if (!initialized)
    return false;

  float x, y, z;
  if (qmi.getAccelerometer(x, y, z)) {
    // Calculate delta
    float dx = abs(x - lastX);
    float dy = abs(y - lastY);
    float dz = abs(z - lastZ);

    lastX = x;
    lastY = y;
    lastZ = z;

    // Threshold for "significant" motion (e.g., 0.1g)
    const float threshold = 0.15;
    if (dx > threshold || dy > threshold || dz > threshold) {
      return true;
    }
  }
  return false;
}
