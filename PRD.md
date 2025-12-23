# Product Requirements Document: ESP32-S3 Weather Display

## 1. Product Overview
A portable, battery-powered weather display station running on the Waveshare ESP32-S3-Touch-LCD-3.49. The device shows real-time weather information for selected California cities and provides status updates on connectivity and battery life.

## 2. Hardware Specifications
- **Device:** [Waveshare ESP32-S3-Touch-LCD-3.49](Waveshare%20ESP32-S3.md)
- **Display:** 3.49-inch IPS LCD (172 x 640 resolution)
- **Touch:** Capacitive Touch
- **Connectivity:** WiFi (2.4GHz)
- **Power:** Battery powered with voltage monitoring

## 3. Key Features

### 3.1 Startup Sequence
- **Behavior:** Upon powering on or usage reset.
- **Display:** Show "Hello World!" or LVGL Demo Widgets centered on the screen.
- **Duration:** Remain visible for exactly 2 seconds before transitioning to the main application.
- **Status:** [VERIFIED] Hardware and drivers capable of rendering complex UI (LVGL Widgets Demo running).

### 3.2 Connectivity & Status
- **WiFi Connection:**
    - Connect to home WiFi network (SSID/Password constraints tbd).
    - Display connection status (Searching, Connected, Failed).
- **Battery Monitoring:**
    - Read battery voltage via ADC.
    - Display battery percentage on the main screen.

### 3.3 Weather Information
- **Locations:**
    1. Oakland, CA
    2. San Francisco, CA
    3. Mountain View, CA
- **Data Fetching:**
    - Fetch current weather data from an API (e.g., OpenWeatherMap, stored config or hardcoded).
- **Display Mode:**
    - Scroll the weather information for the specified cities on the display.
    - Information to display: City Name, Temperature, Conditions (e.g., "Clear", "Rain").

### 3.4 Power Management
- **Hardware Trigger:** GPIO 16 (Power Button / Touch Interrupt).
- **Auto-Sleep Logic:**
    - `PowerManager` tracks `lastActivityTime` (Touch or Motion).
    - If `millis() - lastActivityTime > 60000`, enter Deep Sleep.
- **Motion Reset:** `MotionManager` polls QMI8658; resets `lastActivityTime` if delta > 0.15g.
- **Backlight Coordinated Control:**
    - Turn off GPIO 8 (Active Low PWM).
    - Disable EXIO 1 (Active High BL_EN via TCA9554 expander).
- **Deep Sleep Implementation:**
    - `esp_sleep_enable_ext0_wakeup(GPIO_NUM_16, 0)`.
    - `esp_deep_sleep_start()`.
    - Note: Device reboots on wakeup, state is preserved via `RTC_DATA_ATTR`.

### 3.5 UI Enhancements [NEW]
- **Status Bar:**
    - **Location:** Bottom of the screen.
    - **Style:** Blue background, White text.
    - **Content:** WiFi Status, IP Address, Battery Voltage/Percentage.
    - **Visibility:** Always visible.
- **Weather Display:**
    - **Location:** Main central area (above status bar).
    - **Style:** Large Font (approx 2x current size), Centered.
    - **Content:** City Name, Temperature, Condition.
- **Interaction:**
    - **Gesture:** Swipe Left/Right anywhere on the screen.
    - **Action:** Switch between the 3 configured cities (Oakland <-> SF <-> Mountain View). [VERIFIED]

## 4. Success Metrics
- Device boots and shows splash screen.
- Device connects to WiFi.
- Weather data for all 3 cities is visible and updates/scrolls.
- Battery % represents actual charge level.
