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
    - **Animated Weather Icons:** Dynamic LVGL vector animations representing weather codes (e.g., rotating rays for sun, drifting clouds, falling rain).
- **Interaction:** Swipe Left/Right anywhere to toggle between cities.

### 3.4 Audio Feedback & Control
- **Audio Soundscape:**
    - Startup Jingle: Premium signature sound on boot.
    - Touch Feedback: Subtle "click" sounds for UI interactions.
- **Audio Control:**
    - **Toggle:** Compile-time flag `CONFIG_ENABLE_AUDIO` to completely disable audio components for silent operation.
- **Hardware:** Utilizes ES8311 DAC and Standard Philips I2S protocol.

### 3.4 Power Management (Staged Auto-Sleep)
- **Activity Tracking:** `PowerManager` resets idle timer on Touch or Motion (> 0.15g).
- **Stage 1 (Dimming):** After **30s** of inactivity, dim backlight to 50% (PWM 128).
- **Stage 2 (Sleep):** After **60s** total inactivity, enter Deep Sleep.
- **Backlight Control:** Uses hardware PWM (LEDC) for fine-grained brightness (0-255).
- **Wakeup:** GPIO 16 (Power Button) triggers full reboot from Deep Sleep.

### 3.6 Voice Memo (Voice Notes)
- **Functionality:** Record and playback short voice clips (up to 30 seconds).
- **Controls:**
    - **Record Button:** Toggles recording ON/OFF. Overwrites previous memo. Shows visual "Recording" state.
    - **Playback Button:** Plays the most recent recording.
- **Microphone:** Uses dual onboard microphones via ES7210 ADC (I2S).
- **Audio Output:** Plays back through the ES8311 DAC.
- **Memory Management:** Audio data is stored in the 8MB PSRAM for high fidelity and fast access without wearing out the Flash.

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

### 5.1 Advanced Visuals
- **Dynamic Backgrounds:** Glassmorphism effects and gradients that shift based on current conditions or time (e.g., warm oranges for sunset, deep blues for rainy days).
- **Smooth Transitions:** LVGL-powered sliding animations when switching locations.
- **Particle Systems:** Weather-specific particle effects (e.g., subtle snow particles floating across the background).

### 5.2 Voice & Contextual Intelligence
- **Voice Synthesis (TTS):** Announce the current weather when the device is picked up (using motion trigger).
- **Audio AI:** Basic voice command recognition (e.g., "Refresh", "Next City") using the ES7210.
- **Contextual Notifications:** Audio-visual alerts for extreme weather events (e.g., "Storm Warning" jingle).

### 5.3 Smart Home & Connectivity
- **Web Portal:** A local web dashboard (hosted on the ESP) to manage city lists, WiFi credentials, and custom themes.
- **ESP-NOW / MQTT:** Integration with local sensors to display indoor temperature/humidity alongside outdoor data.
- **Home Assistant:** Integration to use the device as a secondary display for smart home states.

### 5.4 Sleep & Health Utilities
- **Sunrise Alarm:** Gradually fade in the backlight (simulated sunrise) to wake the user gently.
- **Sleep Motion Analysis:** Basic sleep cycle tracking using the onboard accelerometer during bedside charging.

### 5.5 System & Customization
- **Theme Engine:** Pre-built UI profiles (e.g., "Cyberpunk", "Minimalist White", "Retro Terminal").
- **Auto-Orientation:** Automatic UI rotation using the IMU for flexible desktop placement.
