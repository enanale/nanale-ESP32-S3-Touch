#include "battery_manager.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "lvgl.h"
#include "lvgl_port.h"
#include "network_manager.h"
#include "user_config.h"
#include "weather_manager.h"
#include <Arduino.h>
#include <Wire.h>

#include "motion_manager.h"
#include "power_manager.h"
#include "time_manager.h"
#include "ui_manager.h" // Added UIManager include
#include "weather_icons.h"

#include "audio_manager.h"

// Globals
AppNetworkManager netMgr;
WeatherManager weatherMgr;
BatteryManager batMgr;
MotionManager motMgr;
PowerManager pwrMgr;
TimeManager timeMgr;
WeatherIcon weatherIcon;
UIManager uiMgr(weatherMgr, batMgr, timeMgr, pwrMgr,
                weatherIcon); // Added UIManager global

AudioManager audioMgr;

// Persist city index across deep sleep reboots
static int RTC_DATA_ATTR rtc_city_index = 0;
static unsigned long last_weather_update = 0;

// Helper: Check Power Button
void check_power_button() {
  if (digitalRead(PIN_USER_KEY) == LOW) {
    delay(50);
    if (digitalRead(PIN_USER_KEY) == LOW) {
      Serial.println("[PWR] Button Pressed -> Going to Sleep...");
      lvgl_port_set_backlight(0);
      delay(100);
      while (digitalRead(PIN_USER_KEY) == LOW) {
        delay(10);
      }
      delay(50);
      pwrMgr.goToDeepSleep();
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Backlight off during boot
  lvgl_port_set_backlight(0);

  lvgl_port_init();

  if (lvgl_lock(1000)) {
    uiMgr.setCurrentCityIndex(rtc_city_index);
    uiMgr.build();
    lvgl_unlock();
  }

  // Backlight On
  lvgl_port_set_backlight(255);

  netMgr.begin();
  batMgr.begin();
  timeMgr.begin();
  audioMgr.begin();
  audioMgr.playJingle();

  pinMode(PIN_USER_KEY, INPUT);
  motMgr.begin();
  pwrMgr.begin();
}

void loop() {
  check_power_button();

  if (motMgr.hasSignificantMotion()) {
    pwrMgr.resetTimer();
  }

  pwrMgr.update();
  batMgr.update();
  timeMgr.update();

  // Sync back local city index to RTC for next boot
  rtc_city_index = uiMgr.getCurrentCityIndex();

  static unsigned long last_ui_update = 0;
  if (millis() - last_ui_update > 1000) {
    if (lvgl_lock(100)) {
      uiMgr.updateStatusBar();
      lvgl_unlock();
    }
    last_ui_update = millis();
  }

  if (millis() - last_weather_update > 600000 || last_weather_update == 0) {
    if (netMgr.isConnected()) {
      weatherMgr.fetchRealData();
      if (lvgl_lock(100)) {
        uiMgr.updateWeather();
        lvgl_unlock();
      }
      last_weather_update = millis();
    }
  }

  audioMgr.update();
  if (audioMgr.isRecording() || audioMgr.isMemoPlaying()) {
    pwrMgr.resetTimer(); // Don't sleep during audio activity
  }

  delay(5);
}
