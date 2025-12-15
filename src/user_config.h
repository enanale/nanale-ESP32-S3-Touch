#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#include "driver/gpio.h"

// SPI & I2C Hosts
// SPI & I2C Hosts
#define LCD_HOST SPI3_HOST

// Touch I2C Pins (Port 1)
#define Touch_SCL_NUM (GPIO_NUM_18)
#define Touch_SDA_NUM (GPIO_NUM_17)

// Display QSPI Pins
#define EXAMPLE_PIN_NUM_LCD_CS (GPIO_NUM_9)
#define EXAMPLE_PIN_NUM_LCD_PCLK (GPIO_NUM_10)
#define EXAMPLE_PIN_NUM_LCD_DATA0 (GPIO_NUM_11)
#define EXAMPLE_PIN_NUM_LCD_DATA1 (GPIO_NUM_12)
#define EXAMPLE_PIN_NUM_LCD_DATA2 (GPIO_NUM_13)
#define EXAMPLE_PIN_NUM_LCD_DATA3 (GPIO_NUM_14)
#define EXAMPLE_PIN_NUM_LCD_RST (GPIO_NUM_21)
#define EXAMPLE_PIN_NUM_BK_LIGHT 8
#define EXAMPLE_PIN_NUM_PWR_BTN (GPIO_NUM_16)

// Resolution
#define EXAMPLE_LCD_H_RES 172
#define EXAMPLE_LCD_V_RES 640
#define LVGL_DMA_BUFF_LEN (EXAMPLE_LCD_H_RES * 64 * 2) // Tuned buffer size
#define LVGL_SPIRAM_BUFF_LEN (EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * 2)

// LVGL Task Config
#define LVGL_TICK_PERIOD_MS 5
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 5
#define LVGL_TASK_STACK_SIZE (8 * 1024)
#define LVGL_TASK_PRIORITY 2

// WiFi Credentials
// WiFi Credentials
#if __has_include("secrets.h")
#include "secrets.h"
#else
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASS "YOUR_PASS"
#endif

#endif
