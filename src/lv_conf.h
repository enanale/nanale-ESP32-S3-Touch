#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

#define LV_COLOR_DEPTH 16
#define LV_USE_STDLIB_MALLOC 0
#define LV_MEM_SIZE (128 * 1024U)

#define LV_DEF_INDEV_READ_PERIOD 33

#define LV_USE_DRAW_SW 1
// #define LV_USE_DRAW_SW_ASM LV_DRAW_SW_ASM_NONE // Avoid ASM issues on Xtensa
// if needed
#define LV_USE_LABEL 1
#define LV_USE_BTN 1
#define LV_USE_THEME_DEFAULT 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_18 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_MONTSERRAT_40 1
#define LV_COLOR_16_SWAP 1 // Enabled for Main Weather Display

#endif
