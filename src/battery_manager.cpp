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
  // Simple LiPo estimation
  // 4.2V = 100%, 3.3V = 0%
  if (_voltage >= 4.2)
    return 100;
  if (_voltage <= 3.3)
    return 0;

  return static_cast<int>(((_voltage - 3.3) / (4.2 - 3.3)) * 100);
}
