#include "time_manager.h"
#include "user_config.h"
#include <WiFi.h>
#include <Wire.h>
#include <time.h>

// PCF85063 Registers
#define RTC_SEC_REG 0x04
#define RTC_MIN_REG 0x05
#define RTC_HR_REG 0x06
#define RTC_DAY_REG 0x07
#define RTC_WDAY_REG 0x08
#define RTC_MON_REG 0x09
#define RTC_YR_REG 0x0A

static uint8_t bcdToDec(uint8_t val) {
  return ((val >> 4) * 10) + (val & 0x0F);
}

static uint8_t decToBcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

TimeManager::TimeManager()
    : _isSynced(false), _lastSyncAttempt(0), _syncInterval(86400000) {} // 24h

void TimeManager::begin() {
  Serial.println("[TIME] Initializing...");

  // Configure system timezone
  configTime(0, 0, NTP_SERVER);
  setenv("TZ", TZ_INFO, 1);
  tzset();

  // Boot: Load from RTC to system clock
  loadFromRTC();
}

void TimeManager::update() {
  // Attempt NTP sync if WiFi is connected and we aren't synced or interval
  // passed
  if (WiFi.status() == WL_CONNECTED) {
    if (!_isSynced || (millis() - _lastSyncAttempt > _syncInterval)) {
      syncWithNTP();
    }
  }
}

void TimeManager::syncWithNTP() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println("[TIME] NTP Sync Successful. Updating RTC...");
    saveToRTC();
    _isSynced = true;
    _lastSyncAttempt = millis();
  } else {
    Serial.println("[TIME] NTP Sync Failed.");
    _lastSyncAttempt = millis(); // Retry soon
  }
}

String TimeManager::getTimeString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "--:--";
  }
  char buf[10];
  snprintf(buf, sizeof(buf), "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  return String(buf);
}

bool TimeManager::isSynced() { return _isSynced; }

void TimeManager::loadFromRTC() {
  Wire1.beginTransmission(RTC_I2C_ADDR);
  Wire1.write(RTC_SEC_REG);
  if (Wire1.endTransmission() != 0) {
    Serial.println("[TIME] Error: PCF85063 not found!");
    return;
  }

  Wire1.requestFrom(RTC_I2C_ADDR, 7);
  if (Wire1.available() < 7)
    return;

  uint8_t sec = bcdToDec(Wire1.read() & 0x7F);
  uint8_t min = bcdToDec(Wire1.read() & 0x7F);
  uint8_t hr = bcdToDec(Wire1.read() & 0x3F);
  uint8_t day = bcdToDec(Wire1.read() & 0x3F);
  uint8_t wday = bcdToDec(Wire1.read() & 0x07);
  uint8_t mon = bcdToDec(Wire1.read() & 0x1F);
  uint8_t yr = bcdToDec(Wire1.read());

  struct tm tm;
  tm.tm_sec = sec;
  tm.tm_min = min;
  tm.tm_hour = hr;
  tm.tm_mday = day;
  tm.tm_wday = wday;
  tm.tm_mon = mon - 1;
  tm.tm_year = yr + 100; // Assuming 2000s

  time_t t = mktime(&tm);
  struct timeval now = {.tv_sec = t};
  settimeofday(&now, NULL);

  Serial.printf("[TIME] RTC Sync: %02d:%02d:%02d\n", hr, min, sec);
}

void TimeManager::saveToRTC() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
    return;

  Wire1.beginTransmission(RTC_I2C_ADDR);
  Wire1.write(RTC_SEC_REG);
  Wire1.write(decToBcd(timeinfo.tm_sec));
  Wire1.write(decToBcd(timeinfo.tm_min));
  Wire1.write(decToBcd(timeinfo.tm_hour));
  Wire1.write(decToBcd(timeinfo.tm_mday));
  Wire1.write(decToBcd(timeinfo.tm_wday));
  Wire1.write(decToBcd(timeinfo.tm_mon + 1));
  Wire1.write(decToBcd(timeinfo.tm_year - 100));
  Wire1.endTransmission();

  Serial.println("[TIME] RTC Updated from System Clock.");
}
