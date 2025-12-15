#ifndef LVGL_PORT_H
#define LVGL_PORT_H

#include <stdbool.h>

void lvgl_port_init(void);
void lvgl_port_set_backlight(bool on);
bool lvgl_lock(int timeout_ms);
void lvgl_unlock(void);

#endif
