#ifndef LVGL_PORT_H
#define LVGL_PORT_H

#include <stdbool.h>

void lvgl_port_init(void);
void lvgl_port_set_backlight(bool on);
void touch_init(void); // Internal touch init
bool lvgl_lock(int timeout_ms);
void lvgl_unlock(void);

#endif
