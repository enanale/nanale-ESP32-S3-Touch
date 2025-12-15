#include "network_manager.h"
#include "user_config.h"

AppNetworkManager::AppNetworkManager() : _connected(false), _lastCheck(0) {}

void AppNetworkManager::begin() {
  Serial.println("[NET] Initializing WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[NET] Connecting to ");
  Serial.println(WIFI_SSID);
}

void AppNetworkManager::update() {
  if (millis() - _lastCheck > 1000) {
    _lastCheck = millis();
    bool currentStatus = (WiFi.status() == WL_CONNECTED);

    if (currentStatus != _connected) {
      _connected = currentStatus;
      if (_connected) {
        Serial.print("[NET] Connected! IP: ");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("[NET] Disconnected/Connecting...");
      }
    }
  }
}

bool AppNetworkManager::isConnected() {
  return (WiFi.status() == WL_CONNECTED);
}

String AppNetworkManager::getIP() { return WiFi.localIP().toString(); }

String AppNetworkManager::getStatusString() {
  if (WiFi.status() == WL_CONNECTED)
    return "Connected";
  if (WiFi.status() == WL_NO_SSID_AVAIL)
    return "SSID Not Found";
  if (WiFi.status() == WL_CONNECT_FAILED)
    return "Failed";
  if (WiFi.status() == WL_CONNECTION_LOST)
    return "Lost";
  return "Searching...";
}
