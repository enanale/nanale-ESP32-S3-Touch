#ifndef MOTION_MANAGER_H
#define MOTION_MANAGER_H

#include "SensorQMI8658.hpp"
#include <Arduino.h>

class MotionManager {
public:
  MotionManager();
  bool begin();
  bool hasSignificantMotion();

private:
  SensorQMI8658 qmi;
  float lastX, lastY, lastZ;
  bool initialized;
};

#endif
