# Technical Design Document (TDD): ESP32-S3 Weather Display

## 1.# Test-Driven Development (TDD) Plan

## 1. Hardware Abstraction Layer (HAL) Verify

**Status:** [SUCCESS]

**Learnings from Debugging:**
- **I2C is Split:**
    - **Bus 0 (Sensors/RTC):** SDA=47, SCL=48
    - **Bus 1 (Touch):** SDA=17, SCL=18
- **Driver Strategy:** Drop `Arduino_GFX`. Use ESP-IDF `esp_lcd` panel drivers (AXS15231B) tailored for this board, as provided in vendor examples.
- **Build Configuration:** 
    - Requires `#include <Arduino.h>` before IDF headers in C++ files.
    - `driver/spi_master.h` is available but sensitive to include order.
- **LVGL Integration:**
    - **Assembler Error:** `LV_USE_DRAW_SW_ASM` must be disabled (`LV_DRAW_SW_ASM_NONE`) or patched for ESP32 (Xtensa) as the default helium assembly is for Arm.
    - **Linking Demos:** Library source exclusion rules prevent `lv_demo_widgets` from linking. Required manual compilation adapter (`src/lvgl_demos.c`) to include demo sources and assets.

**New HAL Goals:**
1.  **I2C Manager:** Initialize both I2C buses. Scan both to confirm RTC (`0x51`) and Touch (`0x3B`). [VERIFIED]
2.  **Display Driver:** Port `lvgl_port.c` capabilities to a C++ `DisplayDriver` class. [VERIFIED]
    - Init QSPI bus.
    - Init AXS15231B panel.
    - Init LVGL v9 display & buffers.
    - Init LVGL Input device (Touch).

## 2. UI Development with LVGL v9

**Goal:** Create a responsive, beautiful UI using LVGL widgets.

**Components:**
- **StatusBar:** WiFi icon, Battery %.
- **MainContent:**
    - **Splash:** "Hello World" or Logo.
    - **Weather:** Current temp, Icon, Scroll text.
- **Interaction:** Touch swipes/clicks (managed by LVGL events).

## 3. Manager Refactor

- **NetworkManager:** Keep existing logic, just ensure it doesn't block LVGL.
- **WeatherManager:** Keep logic, feed data into LVGL UI objects (Labels/Images).
- **PowerManager:** (New) Handle Battery reading (ADC) and potential sleep modes.

## 4. Implementation Steps (Revised)

1.  **Project Clean:** Remove `Arduino_GFX` lib. Add `lvgl` lib.
2.  **Drivers:**
    - Create `src/hal/i2c_hal.cpp` (Dual bus setup).
    - Create `src/hal/display_hal.cpp` (QSPI + LVGL Buffer setup).
3.  **Display Verification:**
    - Draw a simple red screen or LVGL label to prove the driver works.
4.  **Touch Verification:**
    - Log touch coordinates to Serial.
5.  **UI Build:**
    - Implement the Hello World Splash using LVGL.
    - Implement the Status Bar.
ring (optional if RAM permits, otherwise distinct UI refresh areas).
  - Methods: `showSplash()`, `updateStatusBar(wifi, battery)`, `updateWeatherTicker(data)`.

### 2.2 Connectivity Manager
- **WiFi:**
  - Async connection attempts to avoid blocking UI during startup.
- **API Client:**
  - Endpoint: OpenWeatherMap (or similar free JSON API).
  - Request Frequency: Every 15-30 minutes to conserve battery/bandwidth.
  - Handling SSL/TLS (required for most modern APIs).

### 2.3 Sensor/Power Manager
- **Battery:**
  - Read ADC pin (Pin TBD from schematics, often GPIO 1-10 range or via PMIC).
  - Convert voltage to percentage (LiPo curve approximation).
- **IO Expander:**
  - Must be initialized *before* display to release resets.

### 2.4 Power Button & Sleep
- **Hardware:** GPIO 16 (labeled Power/Touch-Int).
- **Configuration:** `INPUT_PULLUP` (Active Low).
- **Implementation:**
    - **Trigger:** Polling in `loop()` (or Interrupt). Current impl uses Polling with Debounce.
    - **Backlight Control:** `lvgl_port_set_backlight(bool)` wrapper. Active Low logic (0=ON, 1=OFF).
    - **Sleep API:** `esp_light_sleep_start()`.
    - **Wakeup Source:** `esp_sleep_enable_ext0_wakeup(GPIO_NUM_16, 0)` (Wake on LOW level).
    - **Logic:**
        1. Detect Button Press (Low).
        2. Turn Off Backlight.
        3. Wait for Button Release (to avoid immediate wake loop).
        4. Enter Light Sleep.
        5. Wake on Button Press.
        6. Turn On Backlight.

## 3. Data Flow
1. **Boot:** Initialize IO Expander -> Power up LCD -> Init Graphics -> Show "Hello World".
2. **Background:** Start WiFi Task -> Connect -> Fetch Weather JSON -> Parse to Struct.
3. **Loop:**
   - Update Ticker Position (every ~50ms for smooth scroll).
   - Update Status Bar (every 1s or on event).
   - Check Battery (every 1min).

## 4. Project Structure (Proposed)
```
src/
  main.cpp          # Setup and Loop
  hal_config.h      # Pin definitions and hardware constants
  display_mgr.h/cpp # Wrapper for Arduino_GFX
  network_mgr.h/cpp # WiFi and HTTP handling
  weather.h/cpp     # Data structs and fetching logic
platformio.ini      # Dependencies and board config
```

## 5. Development & Deployment
- **Hardware Connection:** Device connected via USB-C to host computer.
- **Bootloader Mode:** If auto-upload fails, hold `BOOT` button, press `RESET`, then release `BOOT`.
- **Deployment Command:** `pio run -t upload`
- **Monitoring:** `pio device monitor` (Baud rate typically 115200)

## 6. Implementation Roadmap
1. **Project Init:** Setup PlatformIO with `Arduino_GFX` and `ArduinoJson`.
2. **Drivers:** Get screen generic "Red/Green/Blue" test working (verifies IO Expander + QSPI).
3. **UI - Step 1:** "Hello World" Splash.
4. **Network:** Connect to WiFi and print status to Serial.
5. **Integration:** Fetch dummy weather -> Parse -> Scroll on screen.
6. **Refinement:** Battery monitoring and error handling.
