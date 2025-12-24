#ifndef WEATHER_MANAGER_H
#define WEATHER_MANAGER_H

#include <Arduino.h>
#include <vector>

#include <ArduinoJson.h>
#include <HTTPClient.h>

struct WeatherData {
  String city;
  float tempF;
  String condition;
  int weather_code;
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
  void fetchRealData();              // Now public
  WeatherData getWeather(int index); // New method

private:
  std::vector<WeatherData> _forecasts;
  unsigned long _lastFetch;
  String getWeatherCondition(int code);
};

#endif
