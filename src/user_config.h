#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#include "driver/gpio.h"
#include "driver/spi_master.h"

// -------------------------------------------------------------------------
// GPIO Pin Assignments (Standard Waveshare Naming)
// -------------------------------------------------------------------------

// LCD QSPI Interface
#define PIN_LCD_CS (GPIO_NUM_9)
#define PIN_LCD_CLK (GPIO_NUM_10)
#define PIN_LCD_D0 (GPIO_NUM_11)
#define PIN_LCD_D1 (GPIO_NUM_12)
#define PIN_LCD_D2 (GPIO_NUM_13)
#define PIN_LCD_D3 (GPIO_NUM_14)
#define PIN_LCD_RST (GPIO_NUM_21)
#define PIN_LCD_BL (GPIO_NUM_8)

// Touch Panel I2C
#define PIN_TP_SDA (GPIO_NUM_17)
#define PIN_TP_SCL (GPIO_NUM_18)

// System I2C (IMU, RTC, IO Expander)
#define PIN_I2C_SDA (GPIO_NUM_47)
#define PIN_I2C_SCL (GPIO_NUM_48)

// Battery & Power
#define PIN_BAT_ADC (GPIO_NUM_4)
#define PIN_USER_KEY (GPIO_NUM_16) // Labeled as Touch_INT / Power Button
#define PIN_EXIO_INT (GPIO_NUM_42)

// UART
#define PIN_UART_TX (GPIO_NUM_43)
#define PIN_UART_RX (GPIO_NUM_44)

// I2S Audio
#define PIN_I2S_MCLK (GPIO_NUM_7)
#define PIN_I2S_SCLK (GPIO_NUM_15)
#define PIN_I2S_LRCK (GPIO_NUM_46)
#define PIN_I2S_DOUT (GPIO_NUM_45)
#define PIN_I2S_DIN (GPIO_NUM_6)

// -------------------------------------------------------------------------
// Hardware Constants
// -------------------------------------------------------------------------

#define LCD_HOST SPI3_HOST
#define IMU_I2C_ADDR 0x6B
#define RTC_I2C_ADDR 0x51
#define TOUCH_I2C_ADDR 0x3B
#define TCA9554_ADDR 0x20
#define ES8311_I2C_ADDR 0x18
#define ES7210_I2C_ADDR 0x40

// IO Expander (TCA9554) Output Pins
#define EXIO_BL_EN 1
#define EXIO_SYS_EN 6
#define EXIO_NS_MODE 7

// Resolution
#define LCD_H_RES 172
#define LCD_V_RES 640

// -------------------------------------------------------------------------
// Application Config
// -------------------------------------------------------------------------

#define CONFIG_DIM_TIMEOUT_SEC 30
#define CONFIG_SLEEP_TIMEOUT_SEC 600

// Feature Toggles
#define CONFIG_ENABLE_JINGLE 0

// Display Configuration
#define CONFIG_DISPLAY_BRIGHTNESS 200 // 0-255 (128 = 50%, 255 = 100%)

// Time & NTP
#define NTP_SERVER "pool.ntp.org"
#define TZ_INFO "PST8PDT,M3.2.0,M11.1.0" // Pacific Time

// LVGL Memory & Task Config
#define LVGL_DMA_BUFF_LEN (LCD_H_RES * 64 * 2)
#define LVGL_SPIRAM_BUFF_LEN (LCD_H_RES * LCD_V_RES * 2)
#define LVGL_TICK_PERIOD_MS 5
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_STACK_SIZE (8 * 1024)
#define LVGL_TASK_PRIORITY 2

// WiFi Credentials
#if __has_include("secrets.h")
#include "secrets.h"
#else
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASS "YOUR_PASS"
#endif

#endif
