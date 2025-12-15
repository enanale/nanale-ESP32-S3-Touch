#include "weather_manager.h"

WeatherManager::WeatherManager() : _lastFetch(0) {}

void WeatherManager::update() {
  // Refresh mock data every 10 seconds for demo purposes
  if (millis() - _lastFetch > 10000 || _forecasts.empty()) {
    fetchMockData();
    _lastFetch = millis();
  }
}

void WeatherManager::fetchMockData() {
  _forecasts.clear();

  // Mock Data Generator
  // Oakland
  _forecasts.push_back({"Oakland", 18 + (int)random(-2, 3), "Partly Cloudy"});

  // San Francisco
  _forecasts.push_back({"San Francisco", 15 + (int)random(-2, 3), "Foggy"});

  // Mountain View
  _forecasts.push_back({"Mountain View", 22 + (int)random(-2, 3), "Sunny"});

  Serial.println("[WEATHER] Mock Data Updated");
}

std::vector<WeatherData> WeatherManager::getForecasts() { return _forecasts; }

bool WeatherManager::hasData() { return !_forecasts.empty(); }
