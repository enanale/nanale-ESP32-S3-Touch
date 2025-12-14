# ESP32-S3-Touch-LCD-3.49 LVGL 9 Demo

A simplified, clean implementation of LVGL 9 on the Waveshare ESP32-S3-Touch-LCD-3.49 development board.

## Features
- **Landscape Orientation**: Correctly configured for 640x172 resolution.
- **Manual Rotation**: Implements vendor-style manual buffer rotation to resolve hardware driver artifacts (stripes) in landscape mode.
- **LVGL 9**: Running latest LVGL 9.x architecture.
- **Clean Architecture**: Separated hardware/LVGL porting (`lvgl_port.cpp`) from application logic (`main.cpp`).

## Hardware
- **Board**: Waveshare ESP32-S3-Touch-LCD-3.49
- **Display Driver**: QSPI interface (`esp_lcd_axs15231b`).
- **Resolution**: 172(H) x 640(V) Physical, rotated to 640 x 172 Logical.
- **Touch Controller**: CST328 (I2C) - *Implementation In Progress*.

## Configuration
- **PlatformIO**: Managed via `platformio.ini`.
- **Display Pins**:
  - CS: 9, PCLK: 10, RST: 21, Backlight: 8
  - Data: 11, 12, 13, 14
- **Color Swap**: `LV_COLOR_16_SWAP=1` enabled in build flags.

## How to Build
1. Install [PlatformIO](https://platformio.org/).
2. Clone this repository.
3. Build and Upload:
   ```bash
   pio run -t upload
   ```

## Status
- [x] Display Initialization (QSPI)
- [x] Backlight Control (Active Low)
- [x] Color Correction
- [x] Landscape Orientation (Manual Software Rotation)
- [ ] Touch Input (Coming Soon)
