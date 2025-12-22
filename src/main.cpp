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

// Globals
AppNetworkManager netMgr;
WeatherManager weatherMgr;
BatteryManager batMgr;
lv_obj_t *info_label = NULL;

// UI Objects
static lv_obj_t *ui_status_bar = NULL;
static lv_obj_t *ui_status_label_wifi = NULL;
static lv_obj_t *ui_status_label_bat = NULL;

static lv_obj_t *ui_content_area = NULL;
static lv_obj_t *ui_city_label = NULL;
static lv_obj_t *ui_temp_label = NULL;
static lv_obj_t *ui_cond_label = NULL;

static int current_city_index = 0;
static unsigned long last_weather_update = 0;

// Helper: Check Power Button
void check_power_button() {
  if (digitalRead(EXAMPLE_PIN_NUM_PWR_STAT) == LOW) {
    // Debounce
    delay(50);
    if (digitalRead(EXAMPLE_PIN_NUM_PWR_STAT) == LOW) {
      Serial.println("[PWR] Button Pressed -> Going to Sleep...");

      lvgl_port_set_backlight(false);

      while (digitalRead(EXAMPLE_PIN_NUM_PWR_STAT) == LOW) {
        delay(10);
      }
      delay(50);

      esp_sleep_enable_ext0_wakeup((gpio_num_t)EXAMPLE_PIN_NUM_PWR_STAT, 0);
      esp_light_sleep_start();

      Serial.println("[PWR] Woke up!");
      lvgl_port_set_backlight(true);

      while (digitalRead(EXAMPLE_PIN_NUM_PWR_STAT) == LOW) {
        delay(10);
      }
    }
  }
}

void update_weather_ui() {
  if (!ui_city_label || !ui_temp_label)
    return;

  WeatherData data = weatherMgr.getWeather(current_city_index);
  lv_label_set_text(ui_city_label, data.city.c_str());
  String tempStr = String(data.tempF, 1) + "°F";
  lv_label_set_text(ui_temp_label, tempStr.c_str());
  lv_label_set_text(ui_cond_label, data.condition.c_str());
}

void update_status_bar() {
  if (!ui_status_label_wifi || !ui_status_label_bat)
    return;

  // WiFi & IP
  String ip = WiFi.localIP().toString();
  String status = WiFi.isConnected() ? ip : "Disconnected";
  lv_label_set_text(ui_status_label_wifi, status.c_str());

  // Battery
  float voltage = batMgr.getVoltage();
  int pct = batMgr.getPercentage();

  // Icon based on percentage
  const char *icon = LV_SYMBOL_BATTERY_FULL;
  if (pct < 10)
    icon = LV_SYMBOL_BATTERY_EMPTY;
  else if (pct < 30)
    icon = LV_SYMBOL_BATTERY_1;
  else if (pct < 60)
    icon = LV_SYMBOL_BATTERY_2;
  else if (pct < 90)
    icon = LV_SYMBOL_BATTERY_3;

  // Combine icon and text to prevent overlap
  String batStr = String(icon);

  // DEBUG INFO for user to see without Serial Monitor
  char debug_buf[64];
  snprintf(debug_buf, sizeof(debug_buf), " %d%% (%.2fV)", pct, voltage);
  batStr += debug_buf;

  lv_label_set_text(ui_status_label_bat, batStr.c_str());
}

// Gesture Callback
static void gesture_event_cb(lv_event_t *e) {
  lv_indev_t *indev = lv_indev_active();
  if (!indev)
    return;

  lv_dir_t dir = lv_indev_get_gesture_dir(indev);
  if (dir == LV_DIR_LEFT) {
    // Next City
    current_city_index++;
    if (current_city_index >= 3)
      current_city_index = 0;
    update_weather_ui();
    Serial.println("Swipe LEFT -> Next City");
  } else if (dir == LV_DIR_RIGHT) {
    // Prev City
    current_city_index--;
    if (current_city_index < 0)
      current_city_index = 2;
    update_weather_ui();
    Serial.println("Swipe RIGHT -> Prev City");
  }
}

void build_ui() {
  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

  // 1. Status Bar (Bottom, Blue)
  ui_status_bar = lv_obj_create(scr);
  lv_obj_set_size(ui_status_bar, LV_PCT(100), 30);
  lv_obj_set_align(ui_status_bar, LV_ALIGN_BOTTOM_MID);
  lv_obj_set_style_bg_color(ui_status_bar, lv_palette_main(LV_PALETTE_BLUE), 0);
  lv_obj_set_style_radius(ui_status_bar, 0, 0);
  lv_obj_set_style_border_width(ui_status_bar, 0, 0);

  // Status Labels
  ui_status_label_wifi = lv_label_create(ui_status_bar);
  lv_label_set_text(ui_status_label_wifi, "Connecting...");
  lv_obj_set_style_text_color(ui_status_label_wifi, lv_color_white(), 0);
  lv_obj_align(ui_status_label_wifi, LV_ALIGN_LEFT_MID, 5, 0);

  // Battery Group (Now consolidated)
  ui_status_label_bat = lv_label_create(ui_status_bar);
  lv_label_set_text(ui_status_label_bat, "Bat: --%");
  lv_obj_set_style_text_color(ui_status_label_bat, lv_color_white(), 0);
  lv_obj_align(ui_status_label_bat, LV_ALIGN_RIGHT_MID, -10, 0);

  // 2. Main Content (Top, Black)
  ui_content_area = lv_obj_create(scr);
  lv_obj_set_size(ui_content_area, LV_PCT(100),
                  142); // 172 total - 30 status = 142
  lv_obj_align(ui_content_area, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(ui_content_area, lv_color_black(), 0);
  lv_obj_set_style_border_width(ui_content_area, 0, 0);
  lv_obj_remove_flag(ui_content_area, LV_OBJ_FLAG_SCROLLABLE);

  // City Label
  ui_city_label = lv_label_create(ui_content_area);
  lv_obj_set_style_text_font(ui_city_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(ui_city_label, lv_color_white(), 0);
  lv_obj_align(ui_city_label, LV_ALIGN_TOP_MID, 0, 10); // Higher offset
  lv_label_set_text(ui_city_label, "Loading...");

  // Temp Label
  ui_temp_label = lv_label_create(ui_content_area);
  lv_obj_set_style_text_font(ui_temp_label, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(ui_temp_label, lv_palette_main(LV_PALETTE_YELLOW),
                              0);
  lv_obj_align(ui_temp_label, LV_ALIGN_CENTER, 0,
               0); // Center instead of off-center
  lv_label_set_text(ui_temp_label, "--°F");

  // Condition Label
  ui_cond_label = lv_label_create(ui_content_area);
  lv_obj_set_style_text_font(ui_cond_label, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(ui_cond_label,
                              lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
  lv_obj_align(ui_cond_label, LV_ALIGN_BOTTOM_MID, 0, -10); // Lower offset
  lv_label_set_text(ui_cond_label, "---");

  // 3. Gestures
  lv_obj_add_event_cb(scr, gesture_event_cb, LV_EVENT_GESTURE, NULL);

  update_weather_ui();
}

void setup() {
  Serial.begin(115200);

  // Backlight off during boot
  lvgl_port_set_backlight(false); // Ensure off during init

  lvgl_port_init();

  // Show Splash (or just build UI directly)
  if (lvgl_lock(1000)) {
    build_ui();
    lvgl_unlock();
  }

  // Backlight On
  lvgl_port_set_backlight(true);

  netMgr.begin();
  batMgr.begin();

  // Initialize power status pin
  pinMode(EXAMPLE_PIN_NUM_PWR_STAT, INPUT);
}

void loop() {
  check_power_button(); // Check power button for sleep

  // Update Battery (Every 30s handled internally, but we need to update UI)
  batMgr.update();

  static unsigned long last_ui_update = 0;
  if (millis() - last_ui_update > 1000) {
    if (lvgl_lock(100)) {
      update_status_bar();
      lvgl_unlock();
    }
    last_ui_update = millis();
  }

  // Update Weather Data (Every 10 min)
  if (millis() - last_weather_update > 600000 || last_weather_update == 0) {
    if (netMgr.isConnected()) {
      weatherMgr.fetchRealData();
      if (lvgl_lock(100)) {
        update_weather_ui();
        lvgl_unlock();
      }
      last_weather_update = millis();
    }
  }

  delay(5);
}
