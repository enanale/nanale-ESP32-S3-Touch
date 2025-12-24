#include "battery_manager.h"

#include "user_config.h"
#include <Wire.h>

BatteryManager::BatteryManager()
    : _voltage(0.0), _lastUpdate(0), _isCharging(false) {}

void BatteryManager::begin() {
  // ADC is initialized automatically by analogRead in Arduino
  pinMode(_pin, INPUT);
}

void BatteryManager::update() {
  if (millis() - _lastUpdate > 2000) {
    _lastUpdate = millis();
    int raw = analogRead(_pin);
    _voltage = (raw / 4095.0) * 3.3 * 3.0;

    // Serial.printf("[BATT] Volts: %.2fV\n", _voltage);
  }
}

float BatteryManager::getVoltage() { return _voltage; }

int BatteryManager::getPercentage() {
  // 18650 Lithium Battery Discharge Curve (Typical)
  if (_voltage >= 4.2)
    return 100;
  if (_voltage >= 4.1)
    return 90 + (_voltage - 4.1) * 100;
  if (_voltage >= 4.0)
    return 80 + (_voltage - 4.0) * 100;
  if (_voltage >= 3.9)
    return 60 + (_voltage - 3.9) * 200;
  if (_voltage >= 3.8)
    return 40 + (_voltage - 3.8) * 200;
  if (_voltage >= 3.7)
    return 20 + (_voltage - 3.7) * 200;
  if (_voltage >= 3.6)
    return 10 + (_voltage - 3.6) * 100;
  if (_voltage >= 3.5)
    return 5 + (_voltage - 3.5) * 50;
  if (_voltage >= 3.3)
    return (_voltage - 3.3) * 25;

  return 0;
}
