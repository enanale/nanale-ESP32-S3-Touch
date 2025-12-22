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

// Hardware Config
#define LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL 0
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_BK_LIGHT 8
#define PIN_NUM_LCD_CS 9
#define PIN_NUM_LCD_PCLK 10
#define PIN_NUM_LCD_DATA0 11
#define PIN_NUM_LCD_DATA1 12
#define PIN_NUM_LCD_DATA2 13
#define PIN_NUM_LCD_DATA3 14
#define PIN_NUM_LCD_RST 21

// Touch Config (CST328)
#define TOUCH_I2C_ADDR 0x3B
#define TOUCH_SDA 17
#define TOUCH_SCL 18
#define TOUCH_RES_X 172
#define TOUCH_RES_Y 640

static SemaphoreHandle_t lvgl_mutex = NULL;
static lv_display_t *display = NULL;
static lv_indev_t *indev_touch = NULL;

// Internal prototypes
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data);

// CST328 Touch Logic
// Magic Command Sequence to wake up the controller (from Vendor Demo)
static const uint8_t CST328_INIT_CMD[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0,
                                            0x0,  0x0e, 0x0,  0x0,  0x0};

void touch_init(void) {
  Wire.begin(TOUCH_SDA, TOUCH_SCL);

  // Register Touch Driver to LVGL
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touch_read_cb);
  indev_touch = indev;
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  uint8_t buff[32] = {0};

  // 1. Wake / Init Command
  Wire.beginTransmission(TOUCH_I2C_ADDR);
  Wire.write(CST328_INIT_CMD, sizeof(CST328_INIT_CMD));
  if (Wire.endTransmission(false) != 0) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  // 2. Read Data
  Wire.requestFrom((int)TOUCH_I2C_ADDR, 32);
  if (Wire.available() < 6) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }
  Wire.readBytes(buff, 32);

  // 3. Parse Gesture / Points
  // Byte 1 represents the number of touch points
  bool touched = (buff[1] > 0 && buff[1] < 5);

  if (touched) {
    data->state = LV_INDEV_STATE_PRESSED;

    // Parse raw coordinates I2C data (Big Endian)
    uint16_t pointX = (((uint16_t)buff[2] & 0x0f) << 8) | (uint16_t)buff[3];
    uint16_t pointY = (((uint16_t)buff[4] & 0x0f) << 8) | (uint16_t)buff[5];

    // Coordinate Mapping for Landscape Mode:
    // Physical X becomes Logical Y (inverted)
    // Physical Y becomes Logical X
    data->point.x = pointY;
    data->point.y = TOUCH_RES_Y - pointX;

  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}
static SemaphoreHandle_t flush_done_semaphore = NULL;

// Rotation Buffer
static uint8_t *lvgl_dest = NULL;

// TCA9554PWR on Wire1 (Pins 47/48)
// Address already defined in user_config.h as 0x20

static void i2c_scan(TwoWire &wire, const char *bus_name) {
  ESP_LOGI(TAG, "Scanning %s I2C bus...", bus_name);
  byte error, address;
  int nDevices = 0;
  for (address = 1; address < 127; address++) {
    wire.beginTransmission(address);
    error = wire.endTransmission();
    if (error == 0) {
      ESP_LOGI(TAG, "[I2C] %s device found at address 0x%02x", bus_name,
               address);
      nDevices++;
    } else if (error == 4) {
      ESP_LOGW(TAG, "[I2C] %s unknown error at 0x%02x", bus_name, address);
    }
  }
  if (nDevices == 0)
    ESP_LOGE(TAG, "[I2C] No devices found on %s\n", bus_name);
}

static void enable_display_power(void) {
  ESP_LOGI(TAG, "Initializing TCA9554PWR on System Bus...");

  // System I2C on Wire1
  Wire1.begin(SYS_SDA_NUM, SYS_SCL_NUM, 100000);
  delay(100);

  i2c_scan(Wire1, "System (Wire1)");

  // Config: Set Pin 6 as Output (Power Latch), others as Input
  // Bit 6 = 0 (Output), others = 1 (Input) -> 0xBF
  Wire1.beginTransmission(TCA9554_ADDR);
  Wire1.write(0x03);
  Wire1.write(0xBF);
  byte err = Wire1.endTransmission();

  if (err != 0) {
    ESP_LOGE(TAG, "TCA9554 Config Failed! Error: %d", err);
  } else {
    // Output: Set Bit 6 HIGH to latch power
    Wire1.beginTransmission(TCA9554_ADDR);
    Wire1.write(0x01); // Output port
    Wire1.write(0xFF); // Set all high (Pin 6 will be high)
    err = Wire1.endTransmission();
    if (err == 0) {
      ESP_LOGI(TAG, "TCA9554PWR Power Latch Set (Success)");
    }
  }

  // Also ensure Wire (Touch Bus) is initialized
  Wire.begin(Touch_SDA_NUM, Touch_SCL_NUM);
  Wire.setClock(400000);
  i2c_scan(Wire, "Touch (Wire)");

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

  // Initialize Power / IO Expander
  enable_display_power();
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
  // Verify Rotation
  // lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90); // Handled by
  // software buffer
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270); // Or 90

  touch_init(); // Initialize Touch Driver

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
