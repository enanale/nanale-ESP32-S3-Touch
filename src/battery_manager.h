#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include "user_config.h"
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
  bool _isCharging;
  const int _pin = PIN_BAT_ADC;
};

#endif
