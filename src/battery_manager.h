#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <Arduino.h>

class BatteryManager {
public:
  BatteryManager();
  void begin();
  void update();
  float getVoltage();
  int getPercentage();

private:
  float _voltage;
  unsigned long _lastUpdate;
  const int _pin = 4; // GPIO 4 confirmed from vendor code
};

#endif
