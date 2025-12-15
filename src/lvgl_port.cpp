#include "lvgl_port.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <Wire.h> // Required for IO Expander
#include <cstring>
#include <driver/spi_master.h>

#include "lvgl.h"
#include "user_config.h"

// LCD Driver Includes
#include "axs15231b/esp_lcd_axs15231b.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

// Globals
static const char *TAG = "lvgl_port";
static SemaphoreHandle_t lvgl_mux = NULL;
static SemaphoreHandle_t flush_done_semaphore = NULL;

// Rotation Buffer
static uint8_t *lvgl_dest = NULL;

// TCA9554PWR Address
#define TCA9554_ADDR 0x20

static void i2c_scan(void) {
  ESP_LOGI(TAG, "Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      ESP_LOGI(TAG, "I2C device found at address 0x%02x !", address);
      nDevices++;
    } else if (error == 4) {
      ESP_LOGW(TAG, "Unknown error at address 0x%02x", address);
    }
  }
  if (nDevices == 0)
    ESP_LOGE(TAG, "No I2C devices found\n");
  else
    ESP_LOGI(TAG, "done\n");
}

static void enable_display_power(void) {
  ESP_LOGI(TAG, "Initializing TCA9554PWR (IO Expander)...");

  // Power Cycle / Stability
  // Force Pins to Input Pullup first to ensure bus is high
  pinMode(Touch_SDA_NUM, INPUT_PULLUP);
  pinMode(Touch_SCL_NUM, INPUT_PULLUP);
  delay(10);

  Wire.begin(Touch_SDA_NUM, Touch_SCL_NUM);
  Wire.setClock(10000); // Slow down to 10kHz for stability
  delay(50);

  i2c_scan(); // Scan first to confirm device

  // Config: Set P1/P2 Output
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(0x03);
  Wire.write(0x00);
  byte err = Wire.endTransmission();
  if (err != 0) {
    ESP_LOGE(TAG, "TCA9554 Config Failed! Error: %d", err);
    // Try Alternative Address 0x38 just in case
    Wire.beginTransmission(0x38);
    Wire.write(0x03);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0) {
      ESP_LOGI(TAG, "FOUND TCA9554 AT 0x38! UPDATING ADDR.");
      // We can't easily update the macro, but we can continue here
      Wire.beginTransmission(0x38);
      Wire.write(0x01);
      Wire.write(0xFF);
      Wire.endTransmission();
      return;
    }
    return;
  }

  // Output: Set High
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(0x01);
  Wire.write(0xFF);
  err = Wire.endTransmission();
  if (err != 0) {
    ESP_LOGE(TAG, "TCA9554 Output Failed! Error: %d", err);
  } else {
    ESP_LOGI(TAG, "TCA9554PWR Power/RST Set High (Success)");
  }

  vTaskDelay(pdMS_TO_TICKS(120));
}

// Buffers
static uint16_t *trans_buf_1 = NULL;
// We only use one full-frame buffer for SPIRAM to save space if needed, or two
// for double buffering. Config uses SPIRAM buffer size defined in macros.

// -------------------------------------------------------------------------
// Helper: Tick Timer
// -------------------------------------------------------------------------
static void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// -------------------------------------------------------------------------
// Helper: Flush Ready Callback (ISR context)
// -------------------------------------------------------------------------
static bool
example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                                esp_lcd_panel_io_event_data_t *edata,
                                void *user_ctx) {
  BaseType_t TaskWoken = pdFALSE;
  if (flush_done_semaphore) {
    xSemaphoreGiveFromISR(flush_done_semaphore, &TaskWoken);
  }
  return (TaskWoken == pdTRUE);
}

// -------------------------------------------------------------------------
// Helper: Flush Callback
// -------------------------------------------------------------------------
static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area,
                                  uint8_t *color_p) {
  if (!flush_done_semaphore || !trans_buf_1) {
    Serial.println("[LVGL] Flush error: invalid semaphore or buffer");
    lv_disp_flush_ready(disp);
    return;
  }

  esp_lcd_panel_handle_t panel_handle =
      (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
  if (!panel_handle) {
    Serial.println("[LVGL] Flush error: invalid panel handle");
    lv_disp_flush_ready(disp);
    return;
  }

  // --- MANUAL SOFTWARE ROTATION (Vendor Style) ---
  // The vendor code manually rotates the buffer to handle the mismatch
  // between the logical Landscape UI and the physical Portrait Driver striping.

  uint16_t *pixel_ptr = (uint16_t *)color_p; // Default to incoming buffer

  lv_display_rotation_t rotation = lv_display_get_rotation(disp);
  lv_area_t rotated_area;

  if (rotation == LV_DISPLAY_ROTATION_90 ||
      rotation == LV_DISPLAY_ROTATION_270) {
    // Must have the destination buffer allocated
    if (!lvgl_dest) {
      ESP_LOGE(TAG, "Rotation buffer missing!");
      lv_disp_flush_ready(disp);
      return;
    }

    lv_color_format_t cf = lv_display_get_color_format(disp);

    // Calculate rotated area position
    rotated_area = *area;
    lv_display_rotate_area(disp, &rotated_area);

    // Calculate strides
    uint32_t src_stride =
        lv_draw_buf_width_to_stride(lv_area_get_width(area), cf);
    uint32_t dest_stride =
        lv_draw_buf_width_to_stride(lv_area_get_width(&rotated_area), cf);

    int32_t src_w = lv_area_get_width(area);
    int32_t src_h = lv_area_get_height(area);

    // Perform Rotation: color_p -> lvgl_dest
    lv_draw_sw_rotate(color_p, lvgl_dest, src_w, src_h, src_stride, dest_stride,
                      rotation, cf);

    // Update pointer to point to the new ROTATED data
    // And update area to match the physical orientation
    pixel_ptr = (uint16_t *)lvgl_dest;
    // Note: We don't change 'area' pointer variable because we don't use it
    // below, but 'pixel_ptr' is what matters.
  }

  // -----------------------------------------------

  // Convert/Byte-Swap for SPI RGB565
  // REMOVED: Managed by LV_COLOR_16_SWAP in platformio.ini
  // lv_draw_sw_rgb565_swap(color_p, lv_area_get_width(area) *
  // lv_area_get_height(area));

  // For Simplicity in FRESH START: Block-based transfer mechanism mimicking
  // original code's logic But slightly cleaner logic.

  // NOTE: The original code had complex logic for chunks.
  // We will trust the original chunk logic was required for this specific
  // panel/QSPI setup. Re-implementing simplified version:

  // const int flush_count = (LVGL_SPIRAM_BUFF_LEN / LVGL_DMA_BUFF_LEN);
  // Re-calculate flush count based on the *Physical* (Rotated) buffer size if
  // needed, but simpler to use the fixed parameters for now as we are
  // essentially sending a full frame or chunks of a full frame.

  // NOTE: Vendor uses EXAMPLE_LCD_V_RES (640) for chunking loop.
  // Should match physical vertical res.

  const int flush_count = (LVGL_SPIRAM_BUFF_LEN / LVGL_DMA_BUFF_LEN);
  const int offgap = (EXAMPLE_LCD_V_RES / flush_count);
  const int dmalen = (LVGL_DMA_BUFF_LEN / 2); // 16-bit pixels

  int offsetx1 = 0;
  int offsety1 = 0;
  int offsetx2 = EXAMPLE_LCD_H_RES; // Horizontal 172
  int offsety2 = offgap;

  // uint16_t *pixel_ptr = (uint16_t *)color_p; // Moved up
  // Rotation check removed (logic consolidated above)

  // Ensure semaphore is clear before starting
  xSemaphoreTake(flush_done_semaphore, 0);

  // We signal "flush ready" only after ALL blocks are sent.
  // Ideally, we should do this async, but the original code blocked.
  // Blocking is safer for stability right now.

  for (int i = 0; i < flush_count; i++) {
    // Wait for previous transfer to finish if we are pipelining
    // But here we might be strictly serial.

    // Copy to DMA buffer
    memcpy(trans_buf_1, pixel_ptr, LVGL_DMA_BUFF_LEN);

    // Send via Driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2,
                              offsety2, trans_buf_1);

    // WAIT for this specific partial transfer to finish
    // The driver calls `on_color_trans_done` which gives the semaphore.
    xSemaphoreTake(flush_done_semaphore, portMAX_DELAY);

    offsety1 += offgap;
    offsety2 += offgap;
    pixel_ptr += dmalen;
  }

  // Notify LVGL we are done
  lv_disp_flush_ready(disp);
}

// -------------------------------------------------------------------------
// Helper: Touch Read
// -------------------------------------------------------------------------
static void TouchInputReadCallback(lv_indev_t *indev, lv_indev_data_t *data) {
  // Placeholder for touch logic.
  // If you have strict I2C drivers, put them here.
  // For now, let's keep it safe and empty or minimal to avoid I2C crashes if
  // not ready.
  data->state = LV_INDEV_STATE_RELEASED;
}

// -------------------------------------------------------------------------
// Lock / Unlock
// -------------------------------------------------------------------------
bool lvgl_lock(int timeout_ms) {
  if (!lvgl_mux)
    return false;
  const TickType_t timeout_ticks =
      (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_unlock(void) {
  if (!lvgl_mux)
    return;
  xSemaphoreGive(lvgl_mux);
}

// -------------------------------------------------------------------------
// Task
// -------------------------------------------------------------------------
static void lvgl_port_task(void *arg) {
  Serial.println("[LVGL] Task Started");
  uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
  uint32_t check_timer = 0;

  while (1) {
    // Heartbeat removed to allow sleep mode control

    if (lvgl_lock(-1)) {
      task_delay_ms = lv_timer_handler();
      lvgl_unlock();
    }
    if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
      task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
      task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
    vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
  }
}

// -------------------------------------------------------------------------
// Init
// -------------------------------------------------------------------------
static const axs15231b_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t[]){0x00}, 0, 100},
    {0x29, (uint8_t[]){0x00}, 0, 100},
};

// -------------------------------------------------------------------------
// Helper: Backlight Control (Active Low)
// -------------------------------------------------------------------------
void lvgl_port_set_backlight(bool on) {
  // Active Low: 0 = ON, 1 = OFF
  gpio_set_level((gpio_num_t)EXAMPLE_PIN_NUM_BK_LIGHT, on ? 0 : 1);
  ESP_LOGI(TAG, "Backlight Set: %s", on ? "ON" : "OFF");
}

void lvgl_port_init(void) {
  Serial.println("[LVGL] Init Starting...");

  // 1. Create Synchronization Primitives
  lvgl_mux = xSemaphoreCreateMutex();
  flush_done_semaphore = xSemaphoreCreateBinary();
  if (!lvgl_mux || !flush_done_semaphore) {
    Serial.println("[LVGL] Critical Error: Failed to create semaphores");
    return;
  }

  // 2. Hardware Init (GPIO/SPI)
  ESP_LOGI(TAG, "Initialize LCD RESET GPIO");
  gpio_config_t gpio_conf = {};
  gpio_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_conf.mode = GPIO_MODE_OUTPUT;
  gpio_conf.pin_bit_mask = ((uint64_t)1 << EXAMPLE_PIN_NUM_LCD_RST);
  gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&gpio_conf);

  // Backlight Init (Robust)
  gpio_config_t bk_gpio_conf = {};
  bk_gpio_conf.intr_type = GPIO_INTR_DISABLE;
  bk_gpio_conf.mode = GPIO_MODE_OUTPUT;
  bk_gpio_conf.pin_bit_mask = ((uint64_t)1 << EXAMPLE_PIN_NUM_BK_LIGHT);
  bk_gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  bk_gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&bk_gpio_conf);

  // Backlight Control (Active Low confirmed: 0=ON)
  // We keep it OFF initially, then turn ON after panel init
  lvgl_port_set_backlight(false); // OFF initially

  // Bus Config
  spi_bus_config_t buscfg = {};
  buscfg.data0_io_num = EXAMPLE_PIN_NUM_LCD_DATA0;
  buscfg.data1_io_num = EXAMPLE_PIN_NUM_LCD_DATA1;
  buscfg.sclk_io_num = EXAMPLE_PIN_NUM_LCD_PCLK;
  buscfg.data2_io_num = EXAMPLE_PIN_NUM_LCD_DATA2;
  buscfg.data3_io_num = EXAMPLE_PIN_NUM_LCD_DATA3;
  buscfg.max_transfer_sz = LVGL_DMA_BUFF_LEN;
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // Panel IO Config
  esp_lcd_panel_io_handle_t panel_io = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS;
  io_config.dc_gpio_num = -1; // QSPI usually doesn't need DC
  io_config.spi_mode = 3;
  io_config.pclk_hz = 40 * 1000 * 1000;
  io_config.trans_queue_depth = 10;
  io_config.on_color_trans_done = example_notify_lvgl_flush_ready;
  io_config.lcd_cmd_bits = 32;
  io_config.lcd_param_bits = 8;
  io_config.flags.quad_mode = true; // Required for QSPI
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &panel_io));

  // Panel Driver Config
  esp_lcd_panel_handle_t panel = NULL;
  axs15231b_vendor_config_t vendor_config = {};
  vendor_config.flags.use_qspi_interface = 1;
  vendor_config.init_cmds = lcd_init_cmds;
  vendor_config.init_cmds_size =
      sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]);

  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = -1; // We handle reset manually
  // panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB; // Deprecated
  panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
  panel_config.bits_per_pixel = 16;
  panel_config.vendor_config = &vendor_config;
  ESP_ERROR_CHECK(esp_lcd_new_panel_axs15231b(panel_io, &panel_config, &panel));
  ESP_LOGI(TAG, "Panel Handle after create: %p", panel);

  // Manual Reset Sequence (Per Demo)
  gpio_set_level(EXAMPLE_PIN_NUM_LCD_RST, 1);
  vTaskDelay(pdMS_TO_TICKS(30));
  gpio_set_level(EXAMPLE_PIN_NUM_LCD_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(250));
  gpio_set_level(EXAMPLE_PIN_NUM_LCD_RST, 1);
  vTaskDelay(pdMS_TO_TICKS(30));

  ESP_ERROR_CHECK(esp_lcd_panel_init(panel));

  // Ensure Backlight is ON again (Active Low)
  lvgl_port_set_backlight(true);

  // --- DRIVER INIT COMPLETE ---
  ESP_LOGI(TAG, "Display Driver Initialized.");
  // --------------------------

  // 3. LVGL Core Init
  lv_init();

  // Display Creation
  lv_display_t *disp = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);

  // Landscape Rotation (Software - managed in flush_cb)
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270); // Or 90

  // Alloc
  uint8_t *spiram_buf =
      (uint8_t *)heap_caps_malloc(LVGL_SPIRAM_BUFF_LEN, MALLOC_CAP_SPIRAM);
  trans_buf_1 = (uint16_t *)heap_caps_malloc(LVGL_DMA_BUFF_LEN, MALLOC_CAP_DMA);

  // Allocate Rotation Buffer (Same size as SPIRAM buffer / Full Frame)
  lvgl_dest =
      (uint8_t *)heap_caps_malloc(LVGL_SPIRAM_BUFF_LEN, MALLOC_CAP_SPIRAM);

  if (!spiram_buf || !trans_buf_1 || !lvgl_dest) {
    Serial.println("[LVGL] Alloc Failed!");
    return;
  }

  // Set Buffers - Mode FULL for 1/2 buffer or PARTIAL
  // Original code used FULL mode with SPIRAM.
  lv_display_set_buffers(disp, spiram_buf, NULL, LVGL_SPIRAM_BUFF_LEN,
                         LV_DISPLAY_RENDER_MODE_FULL);
  lv_display_set_flush_cb(disp, example_lvgl_flush_cb);
  lv_display_set_user_data(disp, panel);

  // 4. Peripherals
  // Touch (Disabled for now to isolate display)
  // lv_indev_t *touch = lv_indev_create();
  // lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
  // lv_indev_set_read_cb(touch, TouchInputReadCallback);

  // 5. Timer
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &example_increase_lvgl_tick, .name = "lvgl_tick_timer"};
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

  // 6. Task
  xTaskCreatePinnedToCore(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL,
                          LVGL_TASK_PRIORITY, NULL, 1); // Pin to Core 1

  Serial.println("[LVGL] Init Complete.");
}
