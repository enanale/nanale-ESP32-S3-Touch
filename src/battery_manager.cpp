#include "battery_manager.h"

BatteryManager::BatteryManager() : _voltage(0.0), _lastUpdate(0) {}

void BatteryManager::begin() {
  // ADC is initialized automatically by analogRead in Arduino
  pinMode(_pin, INPUT);
}

void BatteryManager::update() {
  if (millis() - _lastUpdate > 30000) { // Update every 30 seconds
    _lastUpdate = millis();

    // Read raw value (0-4095)
    int raw = analogRead(_pin);

    // Calculation: (Raw / 4095.0) * 3.3V_Ref * 3.0_Multiplier
    // Filter slightly to reduce noise? For now, raw is fine or simple average.
    _voltage = (raw / 4095.0) * 3.3 * 3.0;

    // Optional: Simple smoothing
    // static float smoothed = 0;
    // if (smoothed == 0) smoothed = _voltage;
    // smoothed = (smoothed * 0.8) + (_voltage * 0.2);
    // _voltage = smoothed;
  }
}

float BatteryManager::getVoltage() { return _voltage; }

int BatteryManager::getPercentage() {
  // 18650 Lithium Battery Discharge Curve (Typical)
  // 4.2V = 100%
  // 4.1V = 90%
  // 4.0V = 80%
  // 3.9V = 60%
  // 3.8V = 40%
  // 3.7V = 20%
  // 3.6V = 10%
  // 3.5V = 5%
  // 3.3V = 0%

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
