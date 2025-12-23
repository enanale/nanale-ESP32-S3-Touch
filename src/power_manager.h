#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

class PowerManager {
public:
  PowerManager();
  void begin();
  void update();
  void resetTimer();
  bool isTimeoutReached();
  void goToDeepSleep();

private:
  unsigned long lastActivityTime;
  uint32_t timeoutMs;
  uint32_t dimTimeoutMs;
  bool isDimmed;
};

#endif
