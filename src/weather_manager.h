#ifndef WEATHER_MANAGER_H
#define WEATHER_MANAGER_H

#include <Arduino.h>
#include <vector>

#include <ArduinoJson.h>
#include <HTTPClient.h>

struct WeatherData {
  String city;
  int tempF;
  String condition;
};

struct CityCoords {
  String name;
  float lat;
  float lon;
};

class WeatherManager {
public:
  WeatherManager();
  void update();
  std::vector<WeatherData> getForecasts();
  bool hasData();

private:
  std::vector<WeatherData> _forecasts;
  unsigned long _lastFetch;
  void fetchRealData();
  String getWeatherCondition(int code);
};

#endif
