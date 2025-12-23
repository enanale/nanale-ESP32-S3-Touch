#include "lvgl_port.h"
#include "audio_manager.h" // Added AudioManager
#include "axs15231b/esp_lcd_axs15231b.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "user_config.h"
#include <Arduino.h>
#include <Wire.h>
#include <cstring>
#include <driver/ledc.h>

static const char *TAG = "lvgl_port";

// Globals
static SemaphoreHandle_t lvgl_mux = NULL;
static SemaphoreHandle_t flush_done_semaphore = NULL;
static uint8_t tca9554_output_state = 0xFF;
static uint16_t *trans_buf_1 = NULL;
static uint8_t *lvgl_dest = NULL;
static lv_display_t *display = NULL;
static lv_indev_t *indev_touch = NULL;

// Prototypes
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data);
static void example_increase_lvgl_tick(void *arg);
static bool
example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                esp_lcd_panel_io_event_data_t *edata,
                                void *user_ctx);
static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area,
                                  uint8_t *color_p);
static void lvgl_port_task(void *arg);

// -------------------------------------------------------------------------
// Touch Controller (CST328) Initialization
// -------------------------------------------------------------------------
static const uint8_t CST328_INIT_CMD[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0,
                                            0x0,  0x0e, 0x0,  0x0,  0x0};

void touch_init(void) {
  Wire.begin(PIN_TP_SDA, PIN_TP_SCL);
  Wire.setClock(400000);

  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touch_read_cb);
  indev_touch = indev;
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  extern AudioManager audioMgr; // Declare audioMgr inside the function scope
  uint8_t buff[32] = {0};

  Wire.beginTransmission(TOUCH_I2C_ADDR);
  Wire.write(CST328_INIT_CMD, sizeof(CST328_INIT_CMD));
  if (Wire.endTransmission(false) != 0) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  Wire.requestFrom((int)TOUCH_I2C_ADDR, 32);
  if (Wire.available() < 6) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }
  Wire.readBytes(buff, 32);

  bool touched = (buff[1] > 0 && buff[1] < 5);
  if (touched) {
    data->state = LV_INDEV_STATE_PRESSED;
    uint16_t pointX = (((uint16_t)buff[2] & 0x0f) << 8) | (uint16_t)buff[3];
    uint16_t pointY = (((uint16_t)buff[4] & 0x0f) << 8) | (uint16_t)buff[5];

    // Coordinate Mapping: Landscape 270 deg
    data->point.x = pointY;
    data->point.y = LCD_V_RES - pointX;

    static uint8_t last_state = LV_INDEV_STATE_RELEASED;
    if (last_state == LV_INDEV_STATE_RELEASED) {
      audioMgr.playClick();
    }
    last_state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
    static uint8_t last_state = LV_INDEV_STATE_RELEASED;
    last_state = LV_INDEV_STATE_RELEASED;
  }
}

// -------------------------------------------------------------------------
// IO Expander & Power Management
// -------------------------------------------------------------------------
static void i2c_scan(TwoWire &wire, const char *bus_name) {
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    error = wire.endTransmission();
    if (error == 0) {
      ESP_LOGI(TAG, "[I2C] %s device found at address 0x%02x", bus_name,
               address);
      nDevices++;
    }
  }
  if (nDevices == 0)
    ESP_LOGE(TAG, "[I2C] No devices found on %s", bus_name);
}

static void enable_display_power(void) {
  Wire1.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);
  delay(50);
  i2c_scan(Wire1, "System (Wire1)");

  // TCA9554 Config: Bit 1 (BL_EN), 6 (SYS_EN), 7 (NS_MODE) as Output
  Wire1.beginTransmission(TCA9554_ADDR);
  Wire1.write(0x03); // Config register
  Wire1.write(0x3D); // 0011 1101
  Wire1.endTransmission();

  // Latch Power
  tca9554_output_state =
      (1 << EXIO_SYS_EN) | (1 << EXIO_BL_EN) | (1 << EXIO_NS_MODE);
  Wire1.beginTransmission(TCA9554_ADDR);
  Wire1.write(0x01); // Output register
  Wire1.write(tca9554_output_state);
  Wire1.endTransmission();

  vTaskDelay(pdMS_TO_TICKS(120));
}

void lvgl_port_set_backlight(uint8_t brightness) {
  // 1. PWM Control (Active Low)
  // 0 -> Duty 1023 (Off), 255 -> Duty 0 (Full)
  uint32_t duty = 1023 - (brightness * 1023 / 255);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

  // 2. Hardware Enable Pin (EXIO 1)
  if (brightness > 0)
    tca9554_output_state |= (1 << EXIO_BL_EN);
  else
    tca9554_output_state &= ~(1 << EXIO_BL_EN);

  Wire1.beginTransmission(TCA9554_ADDR);
  Wire1.write(0x01);
  Wire1.write(tca9554_output_state);
  Wire1.endTransmission();
}

// -------------------------------------------------------------------------
// LVGL Porting Callbacks
// -------------------------------------------------------------------------
static void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static bool
example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                esp_lcd_panel_io_event_data_t *edata,
                                void *user_ctx) {
  BaseType_t TaskWoken = pdFALSE;
  if (flush_done_semaphore)
    xSemaphoreGiveFromISR(flush_done_semaphore, &TaskWoken);
  return (TaskWoken == pdTRUE);
}

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area,
                                  uint8_t *color_p) {
  if (!flush_done_semaphore || !trans_buf_1) {
    lv_disp_flush_ready(disp);
    return;
  }

  esp_lcd_panel_handle_t panel_handle =
      (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
  uint16_t *pixel_ptr = (uint16_t *)color_p;
  lv_display_rotation_t rotation = lv_display_get_rotation(disp);

  if (rotation == LV_DISPLAY_ROTATION_90 ||
      rotation == LV_DISPLAY_ROTATION_270) {
    if (!lvgl_dest) {
      lv_disp_flush_ready(disp);
      return;
    }
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t src_stride =
        lv_draw_buf_width_to_stride(lv_area_get_width(area), cf);
    uint32_t dest_stride =
        lv_draw_buf_width_to_stride(lv_area_get_height(area), cf);
    lv_draw_sw_rotate(color_p, lvgl_dest, lv_area_get_width(area),
                      lv_area_get_height(area), src_stride, dest_stride,
                      rotation, cf);
    pixel_ptr = (uint16_t *)lvgl_dest;
  }

  const int flush_count = (LVGL_SPIRAM_BUFF_LEN / LVGL_DMA_BUFF_LEN);
  const int offgap = (LCD_V_RES / flush_count);
  const int dmalen = (LVGL_DMA_BUFF_LEN / 2);

  int offsetx1 = 0;
  int offsety1 = 0;
  int offsetx2 = LCD_H_RES;
  int offsety2 = offgap;

  xSemaphoreTake(flush_done_semaphore, 0);
  for (int i = 0; i < flush_count; i++) {
    memcpy(trans_buf_1, pixel_ptr, LVGL_DMA_BUFF_LEN);
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2,
                              offsety2, trans_buf_1);
    xSemaphoreTake(flush_done_semaphore, portMAX_DELAY);
    offsety1 += offgap;
    offsety2 += offgap;
    pixel_ptr += dmalen;
  }
  lv_disp_flush_ready(disp);
}

static void lvgl_port_task(void *arg) {
  while (1) {
    if (lvgl_lock(-1)) {
      uint32_t next = lv_timer_handler();
      lvgl_unlock();
      if (next > LVGL_TASK_MAX_DELAY_MS)
        next = LVGL_TASK_MAX_DELAY_MS;
      vTaskDelay(pdMS_TO_TICKS(next));
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

// -------------------------------------------------------------------------
// Init
// -------------------------------------------------------------------------
static const axs15231b_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t[]){0x00}, 0, 100},
    {0x29, (uint8_t[]){0x00}, 0, 100},
};

void lvgl_port_init(void) {
  enable_display_power();

  lvgl_mux = xSemaphoreCreateMutex();
  flush_done_semaphore = xSemaphoreCreateBinary();

  // 1. LEDC PWM Init
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 5000,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
      .gpio_num = PIN_LCD_BL,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 1023, // Start OFF (Active Low)
      .hpoint = 0,
  };
  ledc_channel_config(&ledc_channel);

  // 2. GPIO Init (Reset only)
  gpio_config_t lcd_gpio_conf = {};
  lcd_gpio_conf.intr_type = GPIO_INTR_DISABLE;
  lcd_gpio_conf.mode = GPIO_MODE_OUTPUT;
  lcd_gpio_conf.pin_bit_mask = ((uint64_t)1 << PIN_LCD_RST);
  gpio_config(&lcd_gpio_conf);

  lvgl_port_set_backlight(0);

  // SPI Bus
  spi_bus_config_t buscfg = {};
  buscfg.data0_io_num = PIN_LCD_D0;
  buscfg.data1_io_num = PIN_LCD_D1;
  buscfg.sclk_io_num = PIN_LCD_CLK;
  buscfg.data2_io_num = PIN_LCD_D2;
  buscfg.data3_io_num = PIN_LCD_D3;
  buscfg.max_transfer_sz = LVGL_DMA_BUFF_LEN;
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // Panel IO
  esp_lcd_panel_io_handle_t panel_io = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.cs_gpio_num = PIN_LCD_CS;
  io_config.dc_gpio_num = -1;
  io_config.spi_mode = 3;
  io_config.pclk_hz = 40 * 1000 * 1000;
  io_config.trans_queue_depth = 10;
  io_config.on_color_trans_done = example_notify_lvgl_flush_ready;
  io_config.lcd_cmd_bits = 32;
  io_config.lcd_param_bits = 8;
  io_config.flags.quad_mode = true;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &panel_io));

  // Panel handle
  esp_lcd_panel_handle_t panel = NULL;
  axs15231b_vendor_config_t vendor_config = {};
  vendor_config.flags.use_qspi_interface = 1;
  vendor_config.init_cmds = lcd_init_cmds;
  vendor_config.init_cmds_size =
      sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]);

  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = -1;
  panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
  panel_config.bits_per_pixel = 16;
  panel_config.vendor_config = &vendor_config;
  ESP_ERROR_CHECK(esp_lcd_new_panel_axs15231b(panel_io, &panel_config, &panel));

  // Reset
  gpio_set_level(PIN_LCD_RST, 1);
  vTaskDelay(pdMS_TO_TICKS(30));
  gpio_set_level(PIN_LCD_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(250));
  gpio_set_level(PIN_LCD_RST, 1);
  vTaskDelay(pdMS_TO_TICKS(30));

  ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
  lvgl_port_set_backlight(true);

  // LVGL Init
  lv_init();
  display = lv_display_create(LCD_H_RES, LCD_V_RES);
  lv_display_set_rotation(display, LV_DISPLAY_ROTATION_270);

  touch_init();

  // Alloc
  trans_buf_1 = (uint16_t *)heap_caps_malloc(LVGL_DMA_BUFF_LEN, MALLOC_CAP_DMA);
  uint8_t *spiram_buf =
      (uint8_t *)heap_caps_malloc(LVGL_SPIRAM_BUFF_LEN, MALLOC_CAP_SPIRAM);
  lvgl_dest =
      (uint8_t *)heap_caps_malloc(LVGL_SPIRAM_BUFF_LEN, MALLOC_CAP_SPIRAM);

  lv_display_set_buffers(display, spiram_buf, NULL, LVGL_SPIRAM_BUFF_LEN,
                         LV_DISPLAY_RENDER_MODE_FULL);
  lv_display_set_flush_cb(display, example_lvgl_flush_cb);
  lv_display_set_user_data(display, panel);

  // Tick
  const esp_timer_create_args_t tick_timer_args = {
      .callback = &example_increase_lvgl_tick, .name = "lvgl_tick"};
  esp_timer_handle_t tick_timer = NULL;
  esp_timer_create(&tick_timer_args, &tick_timer);
  esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000);

  // Task
  xTaskCreatePinnedToCore(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL,
                          LVGL_TASK_PRIORITY, NULL, 1);
}

bool lvgl_lock(int timeout_ms) {
  if (!lvgl_mux)
    return false;
  return xSemaphoreTake(lvgl_mux, (timeout_ms == -1)
                                      ? portMAX_DELAY
                                      : pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void lvgl_unlock(void) {
  if (lvgl_mux)
    xSemaphoreGive(lvgl_mux);
}
