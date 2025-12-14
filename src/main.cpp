#include "lvgl.h"
#include "lvgl_port.h"
#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for USB
  Serial.println("==========================================");
  Serial.println(" [BOOT] Hello Fresh Start! (Clean)");
  Serial.println("==========================================");

  // Init LVGL Port
  lvgl_port_init();

  // Create UI (Thread Safe)
  if (lvgl_lock(1000)) {
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0xFFFFFF),
                              0); // White Background

    lv_obj_t *label = lv_label_create(scr);
    if (label) {
      lv_label_set_text(label, "Hello Fresh Start!");
      lv_obj_set_style_text_color(label, lv_color_hex(0x000000),
                                  0); // Black Text
      // lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0); //
      // Default Font is fine
      lv_obj_center(label);
    } else {
      Serial.println("[APP] Failed to create label");
    }
    lvgl_unlock();
    Serial.println("[APP] Label Created");
  } else {
    Serial.println("[APP] Failed to lock for setup UI");
  }
}

void loop() { delay(1000); }
