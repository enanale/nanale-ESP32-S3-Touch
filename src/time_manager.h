#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H

#include <Arduino.h>
#include <time.h>

class TimeManager {
public:
  TimeManager();
  void begin();
  void update();

  String getTimeString(); // HH:MM
  bool isSynced();

  void syncWithNTP();

private:
  void loadFromRTC();
  void saveToRTC();

  bool _isSynced;
  unsigned long _lastSyncAttempt;
  unsigned long _syncInterval;
};

#endif
