# ESP32-S3-Touch-LCD-3.49 LVGL 9 Demo

A simplified, clean implementation of LVGL 9 on the Waveshare ESP32-S3-Touch-LCD-3.49 development board.

## Features
- **Landscape Orientation**: Correctly configured for 640x172 resolution.
- **Manual Rotation**: Implements vendor-style manual buffer rotation to resolve hardware driver artifacts (stripes) in landscape mode.
- **LVGL 9**: Running latest LVGL 9.x architecture.
- **Connectivity**: WiFi connection management with status display.
- **Weather**: Real-time weather fetching (Open-Meteo) and display in Fahrenheit.
- **Power Management**: Application-level Sleep/Wake using side button (GPIO 16) and Light Sleep.

## Hardware
- **Board**: Waveshare ESP32-S3-Touch-LCD-3.49
- **Display Driver**: QSPI interface (`esp_lcd_axs15231b`).
- **Resolution**: 172(H) x 640(V) Physical, rotated to 640 x 172 Logical.
- **Touch Controller**: CST328 (I2C) - *Skipped per user request*.

## Configuration
- **PlatformIO**: Managed via `platformio.ini`.
- **Credentials**: copy `src/secrets.h.example` (if created) or create `src/secrets.h` with your WiFi credentials.
- **Display Pins**:
  - CS: 9, PCLK: 10, RST: 21, Backlight: 8
  - Data: 11, 12, 13, 14
- **Power Button**: GPIO 16 (Active Low).
- **Color Swap**: `LV_COLOR_16_SWAP=1` enabled in build flags.

## How to Build
1. Install [PlatformIO](https://platformio.org/).
2. Create `src/secrets.h` defining `WIFI_SSID` and `WIFI_PASS`.
3. Clone this repository.
4. Build and Upload:
   ```bash
   # Standard Upload
   pio run -t upload

   # If using Python 3.14+ (or facing version errors), override python path:
   PYTHON_EXE=/usr/local/bin/python3.10 pio run -t upload
   ```

## Status
- [x] Display Initialization (QSPI)
- [x] Backlight Control (Active Low)
- [x] Color Correction
- [x] Landscape Orientation (Manual Software Rotation)
- [x] WiFi Connectivity
- [x] Weather Data (Open-Meteo REST API)
- [x] Power Button Sleep/Wake
- [ ] Touch Input (Skipped)
