#include "weather_manager.h"
#include "esp_log.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>

static const char *TAG = "Weather";

// Cities Configuration
const std::vector<CityCoords> CITIES = {{"Oakland", 37.80, -122.27},
                                        {"San Francisco", 37.77, -122.41},
                                        {"Mountain View", 37.38, -122.08}};

WeatherManager::WeatherManager() : _lastFetch(0) {}

void WeatherManager::update() {
  // Fetch every 15 minutes (900,000 ms) or if empty
  if (millis() - _lastFetch > 900000 || _forecasts.empty()) {
    fetchRealData();
    _lastFetch = millis();
  }
}

void WeatherManager::fetchRealData() {
  if (WiFi.status() != WL_CONNECTED) {
    ESP_LOGW(TAG, "WiFi not connected, skipping weather fetch");
    return;
  }

  _forecasts.clear();

  WiFiClientSecure client;
  client.setInsecure(); // Skip SSL verification
  HTTPClient http;

  for (const auto &city : CITIES) {
    String url =
        "https://api.open-meteo.com/v1/forecast?latitude=" + String(city.lat) +
        "&longitude=" + String(city.lon) +
        "&current=temperature_2m,weather_code&temperature_unit=fahrenheit";

    Serial.print("[WEATHER] Fetching for ");
    Serial.println(city.name);

    http.begin(client, url);
    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        float tempF = doc["current"]["temperature_2m"];
        int code = doc["current"]["weather_code"];

        _forecasts.push_back(
            {city.name, (int)tempF, getWeatherCondition(code)});
        Serial.println("[WEATHER] Success!");
      } else {
        Serial.print("[WEATHER] JSON Error: ");
        Serial.println(error.c_str());
      }
    } else {
      Serial.printf("[WEATHER] HTTP Error: %d\n", httpCode);
    }
    http.end();
    delay(500); // Be nice to the API
  }
}

String WeatherManager::getWeatherCondition(int code) {
  // WMO Weather interpretation codes (WW)
  // https://open-meteo.com/en/docs
  switch (code) {
  case 0:
    return "Clear Sky";
  case 1:
    return "Mainly Clear";
  case 2:
    return "Partly Cloudy";
  case 3:
    return "Overcast";
  case 45:
  case 48:
    return "Fog";
  case 51:
  case 53:
  case 55:
    return "Drizzle";
  case 56:
  case 57:
    return "Freezing Drizzle";
  case 61:
  case 63:
  case 65:
    return "Rain";
  case 66:
  case 67:
    return "Freezing Rain";
  case 71:
  case 73:
  case 75:
    return "Snow";
  case 77:
    return "Snow Grains";
  case 80:
  case 81:
  case 82:
    return "Rain Showers";
  case 85:
  case 86:
    return "Snow Showers";
  case 95:
    return "Thunderstorm";
  case 96:
  case 99:
    return "Thunderstorm/Hail";
  default:
    return "Unknown";
  }
}

WeatherData WeatherManager::getWeather(int index) {
  if (_forecasts.empty()) {
    return {"Loading...", 0.0, ""};
  }
  if (index >= 0 && index < _forecasts.size()) {
    return _forecasts[index];
  }
  return _forecasts[0];
}

std::vector<WeatherData> WeatherManager::getForecasts() { return _forecasts; }

bool WeatherManager::hasData() { return !_forecasts.empty(); }
