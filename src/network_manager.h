#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>

class AppNetworkManager {
public:
  AppNetworkManager();
  void begin();
  void update();
  bool isConnected();
  String getIP();
  String getStatusString();

private:
  bool _connected;
  unsigned long _lastCheck;
};

#endif
