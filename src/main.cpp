#include "driver/gpio.h"
#include "esp_sleep.h"
#include "lvgl.h"
#include "lvgl_port.h"
#include "network_manager.h"
#include "user_config.h"
#include "weather_manager.h"
#include <Arduino.h>

// Globals
AppNetworkManager netMgr;
WeatherManager weatherMgr;
lv_obj_t *info_label = NULL;

// Helper: Check Power Button
void check_power_button() {
  if (digitalRead(EXAMPLE_PIN_NUM_PWR_BTN) == LOW) {
    // Debounce
    delay(50);
    if (digitalRead(EXAMPLE_PIN_NUM_PWR_BTN) == LOW) {
      Serial.println("[PWR] Button Pressed -> Going to Sleep...");

      lvgl_port_set_backlight(false);

      while (digitalRead(EXAMPLE_PIN_NUM_PWR_BTN) == LOW) {
        delay(10);
      }
      delay(50);

      esp_sleep_enable_ext0_wakeup((gpio_num_t)EXAMPLE_PIN_NUM_PWR_BTN, 0);
      esp_light_sleep_start();

      Serial.println("[PWR] Woke up!");
      lvgl_port_set_backlight(true);

      while (digitalRead(EXAMPLE_PIN_NUM_PWR_BTN) == LOW) {
        delay(10);
      }
    }
  }
}

void update_ui() {
  if (lvgl_lock(10)) {
    if (info_label) {
      String statusText = "WiFi: " + netMgr.getStatusString() + "\n";

      if (netMgr.isConnected()) {
        statusText += "IP: " + netMgr.getIP() + "\n\n";

        if (weatherMgr.hasData()) {
          std::vector<WeatherData> data = weatherMgr.getForecasts();
          for (const auto &w : data) {
            statusText +=
                w.city + ": " + String(w.tempF) + "F, " + w.condition + "\n";
          }
        } else {
          statusText += "Fetching Weather...";
        }
      }

      lv_label_set_text(info_label, statusText.c_str());
    }
    lvgl_unlock();
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("==========================================");
  Serial.println(" [BOOT] Hello Fresh Start! (Clean)");
  Serial.println("==========================================");

  pinMode(EXAMPLE_PIN_NUM_PWR_BTN, INPUT_PULLUP);

  lvgl_port_init();
  netMgr.begin();

  // Create UI
  if (lvgl_lock(1000)) {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xFFFFFF), 0);

    info_label = lv_label_create(scr);
    if (info_label) {
      lv_label_set_text(info_label, "Initializing...");
      lv_obj_set_style_text_color(info_label, lv_color_hex(0x000000), 0);
      lv_obj_center(info_label);
    }
    lvgl_unlock();
  }
}

void loop() {
  check_power_button();
  netMgr.update();
  weatherMgr.update();
  update_ui();
  delay(10);
}
