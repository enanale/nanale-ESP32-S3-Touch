# Product Requirements Document: ESP32-S3 Weather Display

## 1. Product Overview
A portable, battery-powered weather display station running on the Waveshare ESP32-S3-Touch-LCD-3.49. The device shows real-time weather information for selected California cities and provides status updates on connectivity, battery life, and current time.

## 2. Hardware Specifications
- **Device:** [Waveshare ESP32-S3-Touch-LCD-3.49](Waveshare%20ESP32-S3.md)
- **Display:** 3.49-inch IPS LCD (172 x 640 resolution)
- **Touch:** Capacitive Touch
- **Connectivity:** WiFi (2.4GHz)
- **Power:** Battery powered with voltage monitoring
- **RTC:** PCF85063 hardware RTC for time persistence

## 3. Key Features

### 3.1 Startup Sequence
- **Behavior:** Upon powering on or usage reset.
- **Backlight:** Kept OFF until UI is ready to prevent flickering.
- **Display:** Show "Hello World!" or LVGL Demo Widgets centered on the screen.
- **Duration:** Remain visible for exactly 2 seconds before transitioning to the main application.

### 3.2 Connectivity & Time
- **WiFi Connection:**
    - Connect to home WiFi network (configured in `user_config.h`).
    - Attempt background reconnection if lost.
- **NTP Synchronization:**
    - Sync system time via `pool.ntp.org` upon WiFi connection.
    - Update hardware RTC (PCF85063) with accurate time post-sync.
- **Digital Clock:**
    - Display current time in **HH:MM format** in the status bar (top-left).
    - Persistent time across reboots via RTC.

### 3.3 Weather Information
- **Locations:** Oakland, SF, Mountain View (CA).
- **Data Fetching:**
    - Fetch current weather data via Open-Meteo API every 15 minutes.
- **Display Mode:**
    - Show City Name, Temperature (°F), and Condition (e.g., "Clear Sky").
    - **Interaction:** Swipe Left/Right anywhere to toggle between cities.

### 3.4 Power Management (Staged Auto-Sleep)
- **Activity Tracking:** `PowerManager` resets idle timer on Touch or Motion (> 0.15g).
- **Stage 1 (Dimming):** After **30s** of inactivity, dim backlight to 50% (PWM 128).
- **Stage 2 (Sleep):** After **60s** total inactivity, enter Deep Sleep.
- **Backlight Control:** Uses hardware PWM (LEDC) for fine-grained brightness (0-255).
- **Wakeup:** GPIO 16 (Power Button) triggers full reboot from Deep Sleep.

### 3.5 UI Aesthetic & Layout
- **Status Bar:**
    - **Location:** Top/Bottom of screen (Full coverage).
    - **Style:** **Deep Purple** background, White text.
    - **Top-Left:** Digital Clock (HH:MM).
    - **Bottom-Right:** Battery Status (Icon + Percentage + Voltage).
- **Main Area:**
    - Centered weather information with large, readable fonts.

## 4. Success Metrics
- [x] Device connects to WiFi and syncs time automatically.
- [x] Clock remains accurate after power cycle (RTC verified).
- [x] Screen dims gracefully before entering deep sleep.
- [x] Weather data is searchable via swipe gestures.
- [x] Battery indicators provide accurate voltage and percentage data.

## 5. Future Roadmap

### 5.1 Visual & Aesthetic Upgrades
- **Weather Icons:** Replace text-only conditions with custom icons or animations.
- **Dynamic Backgrounds:** Gradients or effects that change based on weather or time of day.
- **Smooth Transitions:** LVGL animations when switching between cities.

### 5.2 Audio Integration (ES8311 DAC)
- **Haptic/Audio Feedback:** Subtle sounds for touch events and button presses.
- **Startup Jingle:** A premium audio signature on wake/boot.

### 5.3 Advanced Utility
- **Detailed Forecast:** A "drill-down" view (e.g., 5-day forecast) accessible via tap.
- **System Dashboard:** Technical stats (WiFi RSSI, Memory usage, Battery history charts).

### 5.4 Smart & Contextual Features
- **Auto-Brightness:** Time-aware brightness levels (Day vs Night modes).
- **Auto-Orientation:** Use IMU (Accelerometer) to rotate UI for horizontal/vertical use.
