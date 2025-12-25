#include "power_manager.h"
#include "esp_sleep.h"
#include "lvgl_port.h"
#include "user_config.h"
#include <Arduino.h>

PowerManager::PowerManager()
    : lastActivityTime(0), timeoutMs(60000), dimTimeoutMs(30000),
      isDimmed(false) {}

void PowerManager::begin() {
  lastActivityTime = millis();
#ifdef CONFIG_SLEEP_TIMEOUT_SEC
  timeoutMs = CONFIG_SLEEP_TIMEOUT_SEC * 1000;
#endif
#ifdef CONFIG_DIM_TIMEOUT_SEC
  dimTimeoutMs = CONFIG_DIM_TIMEOUT_SEC * 1000;
#endif

  // Configure wakeup source:
  // 1. Power Button (IO 16) - RTC capable, works for Deep Sleep
  // 2. EXIO_INT (IO 42) - NOT RTC, works for Light Sleep (Touch/Motion)

  // For Deep Sleep, we can only use RTC pins
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_USER_KEY, 0);
}

void PowerManager::update() {
  unsigned long elapsed = millis() - lastActivityTime;

  // Stage 1: Dimming
  if (!isDimmed && elapsed > dimTimeoutMs && elapsed < timeoutMs) {
    uint8_t dimBrightness = CONFIG_DISPLAY_BRIGHTNESS / 2;
    Serial.printf("[PWR] Dimming display to %d (from %d)...\n", dimBrightness,
                  CONFIG_DISPLAY_BRIGHTNESS);
    lvgl_port_set_backlight(dimBrightness);
    isDimmed = true;
  }

  // Stage 2: Sleep
  if (elapsed > timeoutMs) {
    Serial.println("[PWR] Activity timeout reached. Entering Deep Sleep...");
    goToDeepSleep();
  }
}

void PowerManager::resetTimer() {
  lastActivityTime = millis();
  if (isDimmed) {
    Serial.println("[PWR] Activity detected. Restoring brightness...");
    lvgl_port_set_backlight(CONFIG_DISPLAY_BRIGHTNESS);
    isDimmed = false;
  }
}

bool PowerManager::isTimeoutReached() {
  return (millis() - lastActivityTime > timeoutMs);
}

void PowerManager::goToDeepSleep() {
  // 1. Turn off backlight completely
  lvgl_port_set_backlight(0);

  // 2. Small delay for logs to flush
  delay(100);

  // 3. Ensure wakeup is set
  // Deep Sleep ONLY supports RTC pins (0-21). IO 16 is the Power Button.
  // IO 42 (EXIO_INT) is not RTC and cannot wake from Deep Sleep.
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_USER_KEY, 0);

  // 4. Start Deep Sleep
  esp_deep_sleep_start();
}
