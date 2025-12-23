#include "power_manager.h"
#include "esp_sleep.h"
#include "lvgl_port.h"
#include "user_config.h"
#include <Arduino.h>

PowerManager::PowerManager() : lastActivityTime(0), timeoutMs(60000) {}

void PowerManager::begin() {
  lastActivityTime = millis();
#ifdef CONFIG_SLEEP_TIMEOUT_SEC
  timeoutMs = CONFIG_SLEEP_TIMEOUT_SEC * 1000;
#endif

  // Configure wakeup source:
  // 1. Power Button (IO 16) - RTC capable, works for Deep Sleep
  // 2. EXIO_INT (IO 42) - NOT RTC, works for Light Sleep (Touch/Motion)

  // For Deep Sleep, we can only use RTC pins
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_USER_KEY, 0);
}

void PowerManager::update() {
  if (isTimeoutReached()) {
    Serial.println("[PWR] Activity timeout reached. Entering Deep Sleep...");
    goToDeepSleep();
  }
}

void PowerManager::resetTimer() { lastActivityTime = millis(); }

bool PowerManager::isTimeoutReached() {
  return (millis() - lastActivityTime > timeoutMs);
}

void PowerManager::goToDeepSleep() {
  // 1. Turn off backlight
  lvgl_port_set_backlight(false);

  // 2. Small delay for logs to flush
  delay(100);

  // 3. Ensure wakeup is set
  // Deep Sleep ONLY supports RTC pins (0-21). IO 16 is the Power Button.
  // IO 42 (EXIO_INT) is not RTC and cannot wake from Deep Sleep.
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_USER_KEY, 0);

  // 4. Start Deep Sleep
  esp_deep_sleep_start();
}
