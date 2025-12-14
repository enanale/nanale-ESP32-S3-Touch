#ifndef LVGL_PORT_H
#define LVGL_PORT_H

#include <stdbool.h>

void lvgl_port_init(void);
bool lvgl_lock(int timeout_ms);
void lvgl_unlock(void);

#endif
