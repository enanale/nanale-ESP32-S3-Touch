#ifndef WEATHER_MANAGER_H
#define WEATHER_MANAGER_H

#include <Arduino.h>
#include <vector>

struct WeatherData {
  String city;
  int tempC;
  String condition;
};

class WeatherManager {
public:
  WeatherManager();
  void update(); // Check if time to fetch
  std::vector<WeatherData> getForecasts();
  bool hasData();

private:
  std::vector<WeatherData> _forecasts;
  unsigned long _lastFetch;
  void fetchMockData();
};

#endif
