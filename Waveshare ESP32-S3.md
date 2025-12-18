# **Waveshare ESP32-S3-Touch-LCD-3.49 Development Context**

## **1\. Product Overview**

The **ESP32-S3-Touch-LCD-3.49** is a highly integrated development board by Waveshare featuring a rectangular 3.49-inch touch display. It is designed for HMI (Human-Machine Interface) applications and features a unique ultra-wide aspect ratio.

* **Manufacturer:** Waveshare  
* **Model:** ESP32-S3-Touch-LCD-3.49  
* **Core:** ESP32-S3R8 (Dual-core Xtensa LX7, up to 240MHz)  
* **Memory:** 16MB Flash, 8MB PSRAM (Octal)

## **2\. Hardware Specifications**

### **Display System**

* **Panel Type:** IPS LCD  
* **Size:** 3.49 inch  
* **Resolution:** 172 (H) x 640 (V) pixels  
* **Interface:** QSPI (Quad SPI) for Display, I2C for Touch  
* **Display Driver:** **AXS15231B**  
* **Touch Controller:** Capacitive (I2C interface), integrated with AXS15231B  
* **Color Depth:** 16.7M colors (RGB565 or RGB666 supported in software)

### **Onboard Peripherals**

* **IO Expander:** **TCA9554PWR** (I2C Addr: 0x20) \- *CRITICAL: Controls LCD Reset, Backlight, and Power Rails.*  
* **IMU (6-Axis):** **QMI8658** (Accelerometer \+ Gyroscope)  
* **Audio DAC:** **ES8311** (I2S Interface, I2C Addr: 0x18)  
* **Audio ADC:** **ES7210** (I2S Interface, I2C Addr: 0x40) \- Dual Microphone  
* **RTC:** **PCF85063** (I2C Addr: 0x51)  
* **Storage:** MicroSD Card Slot (SPI Interface)  
* **Battery:** Power management via charging circuit; voltage sensing via ADC.
    * **ADC Pin:** GPIO 4 (ADC1 Channel 3)
    * **Voltage Multiplier:** 3.0x
    * **Battery Type:** 3.7V Lithium 18650
    * **Discharge Handling:** Requires a non-linear mapping (discharge curve) for accurate percentage calculation (e.g., 3.6V-3.9V range is non-linear).
    * **Charging Status:** No software-readable flag found on TCA9554 or direct GPIO in standard demo; monitor voltage trends instead.

## **3\. Pin Definitions & GPIO Mapping**

### **System & Communication**

| Function | GPIO Pin | Notes |
| :---- | :---- | :---- |
| **I2C SDA** | GPIO 17 | Touch I2C Data |
| **I2C SCL** | GPIO 18 | Touch I2C Clock |
| **UART TX** | GPIO 43 | USB-UART Bridge (CH343) |
| **UART RX** | GPIO 44 | USB-UART Bridge (CH343) |
| **USB D-** | GPIO 19 | Native ESP32-S3 USB |
| **USB D+** | GPIO 20 | Native ESP32-S3 USB |

### **Display (QSPI) & Touch**

*Note: The Display uses QSPI (4 data lines) for high-speed data transfer.*

| Function | GPIO Pin | Notes |
| :---- | :---- | :---- |
| **LCD\_CLK** | GPIO 10 | SPI Clock |
| **LCD\_D0** | GPIO 11 | Data 0 |
| **LCD\_D1** | GPIO 12 | Data 1 |
| **LCD\_D2** | GPIO 13 | Data 2 |
| **LCD\_D3** | GPIO 14 | Data 3 |
| **LCD\_CS** | GPIO 9  | Chip Select |
| **LCD\_RST** | GPIO 21 | Display Reset (Direct GPIO) |
| **LCD\_BL** | GPIO 8  | Backlight (PWM Capable) |
| **Touch\_INT** | GPIO 16 | Interrupt pin (Active Low) |
| **Touch\_RST** | N/A | Often shared or internal |

### **IO Expander (TCA9554PWR) - Key Functions**

* **Address:** 0x20
* **Pin 6:** Power Rail Control (Set 1 to keep ON, 0 to Power Off)
* **Pin 7:** Often used for peripheral enabling.
* **Note**: Earlier documentation suggested it controlled LCD RST/BL. In the 3.49" Touch revision, these are direct GPIOs (see above).


### **Audio (I2S)**

| Function | GPIO Pin | Notes |
| :---- | :---- | :---- |
| **I2S\_MCLK** | GPIO 2 | Master Clock |
| **I2S\_SCLK** | GPIO 3 | Bit Clock |
| **I2S\_LRCK** | GPIO 5 | Word Select (WS) |
| **I2S\_DOUT** | GPIO 4 | Data Out (To Speaker/DAC) |
| **I2S\_DIN** | GPIO 6 | Data In (From Mic/ADC) |

## **4\. Technical Implementation Details**

### **1\. WiFi Setup**

Standard ESP32-S3 WiFi implementation. Note that using WiFi heavily may introduce noise into the audio lines if not properly filtered, though the board design attempts to mitigate this.

\#include \<WiFi.h\>

void setupWiFi() {  
    WiFi.mode(WIFI\_STA);  
    WiFi.begin("YOUR\_SSID", "YOUR\_PASS");  
    while (WiFi.status() \!= WL\_CONNECTED) {  
        delay(500);  
        Serial.print(".");  
    }  
    Serial.println("\\nWiFi Connected");  
}

### **2\. Display (LVGL V9)**

The transition to LVGL V9 requires specific backend porting for the QSPI AXS15231B driver.

**Key Configurations (`lv_conf.h`):**
- `LV_COLOR_DEPTH 16`
- `LV_COLOR_16_SWAP 1` (Required for correct color rendering on this panel)
- `LV_FONT_MONTSERRAT_18`, `28`, `40` (Enabled for high-resolution weather UI)

**Initialization Notes:**
- Handle display rotation in the flush callback if needed (172x640 is naturally portrait).
- Use `esp_lcd` vendor drivers for the AXS15231B.
- Consolidate UI elements (e.g., battery icon + text) to prevent overlaps on the narrow 172px width.


### **3\. Touch Input**

The touch controller is accessed via I2C (SDA:8, SCL:9). The Touch\_INT pin (GPIO 16\) goes LOW when a touch is detected.

* **Interrupt:** Attach an interrupt to GPIO 16 (FALLING or LOW).  
* **Driver:** The AXS15231B usually embeds the touch logic. Standard generic capacitive touch libraries might need I2C register tweaking.  
* **Reading:** When INT goes low, read I2C to get X/Y coordinates. Ensure you map the 172x640 hardware coordinates to your logical rotation.

### **4\. Audio (Recording & Playback)**

This board uses **ES7210** for recording (ADC) and **ES8311** for playback (DAC). Both must be configured via I2C before I2S data transfer works.

**I2S Configuration (ESP-IDF / Arduino):**

\#include \<driver/i2s.h\>

\#define I2S\_NUM         I2S\_NUM\_0  
\#define I2S\_MCLK\_GPIO   2  
\#define I2S\_BCLK\_GPIO   3  
\#define I2S\_LRCK\_GPIO   5  
\#define I2S\_DOUT\_GPIO   4  
\#define I2S\_DIN\_GPIO    6

void setupAudio() {  
    // 1\. Configure ES8311 (DAC) via I2C (Addr 0x18)  
    // Send I2C commands to unmute, set volume, enable DAC.  
      
    // 2\. Configure ES7210 (ADC) via I2C (Addr 0x40)  
    // Send I2C commands to set gain, mic bias, and enable ADC.

    // 3\. Configure I2S Driver  
    i2s\_config\_t i2s\_config \= {  
        .mode \= (i2s\_mode\_t)(I2S\_MODE\_MASTER | I2S\_MODE\_TX | I2S\_MODE\_RX),  
        .sample\_rate \= 16000, // or 44100  
        .bits\_per\_sample \= I2S\_BITS\_PER\_SAMPLE\_16BIT,  
        .channel\_format \= I2S\_CHANNEL\_FMT\_RIGHT\_LEFT,  
        .communication\_format \= I2S\_COMM\_FORMAT\_STAND\_I2S,  
        .intr\_alloc\_flags \= ESP\_INTR\_FLAG\_LEVEL1,  
        .dma\_buf\_count \= 8,  
        .dma\_buf\_len \= 64,  
        .use\_apll \= true,  
        .tx\_desc\_auto\_clear \= true,  
        .fixed\_mclk \= 0  
    };  
      
    i2s\_pin\_config\_t pin\_config \= {  
        .bck\_io\_num \= I2S\_BCLK\_GPIO,  
        .ws\_io\_num \= I2S\_LRCK\_GPIO,  
        .data\_out\_num \= I2S\_DOUT\_GPIO,  
        .data\_in\_num \= I2S\_DIN\_GPIO  
    };

    // 3\. Configure I2S Driver
    i2s\_config\_t i2s\_config \= {
        .mode \= (i2s\_mode\_t)(I2S\_MODE\_MASTER | I2S\_MODE\_TX | I2S\_MODE\_RX),
        .sample\_rate \= 16000, // or 44100
        .bits\_per\_sample \= I2S\_BITS\_PER\_SAMPLE\_16BIT,
        .channel\_format \= I2S\_CHANNEL\_FMT\_RIGHT\_LEFT,
        .communication\_format \= I2S\_COMM\_FORMAT\_STAND\_I2S,
        .intr\_alloc\_flags \= ESP\_INTR\_FLAG\_LEVEL1,
        .dma\_buf\_count \= 8,
        .dma\_buf\_len \= 64,
        .use\_apll \= true,
        .tx\_desc\_auto\_clear \= true,
        .fixed\_mclk \= 0
    };

    i2s\_pin\_config\_t pin\_config \= {
        .bck\_io\_num \= I2S\_BCLK\_GPIO,
        .ws\_io\_num \= I2S\_LRCK\_GPIO,
        .data\_out\_num \= I2S\_DOUT\_GPIO,
        .data\_in\_num \= I2S\_DIN\_GPIO
    };

    i2s\_driver\_install(I2S\_NUM, \&i2s\_config, 0, NULL);
    i2s\_set\_pin(I2S\_NUM, \&pin\_config);
    i2s\_set\_clk(I2S\_NUM, 16000, I2S\_BITS\_PER\_SAMPLE\_16BIT, I2S\_CHANNEL\_STEREO);

    // Important: MCLK is often required by ES8311/ES7210
    PIN\_FUNC\_SELECT(GPIO\_PIN\_MUX\_REG\[I2S\_MCLK\_GPIO\], PIN\_FUNC\_GPIO);
    // Specific register manipulation might be needed to output MCLK on GPIO 2 if not automatic.
}

## **5\. Critical Development Notes**

1.  **Power Sequencing:** The **TCA9554PWR** is the gatekeeper. If you do not initialize it, the screen will be black, touch won't work, and audio chips might be powered down.
2.  **I2C Bus Stability:** The I2C lines (GPIO 8/9) may be floating on some boards. If you see "ghost devices" (scans showing addresses like 0x08, 0x09, etc.), you **MUST** enable internal pull-ups:
    ```cpp
    pinMode(8, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
    Wire.begin(8, 9);
    ```
3.  **Audio Noise:** Ensure the power supply is stable. The ES8311 is sensitive to noise on the 3.3V line.
4.  **Memory:** Enable PSRAM (OPI) in your build settings. The frame buffer for 172x640x16bpp is \~220KB, which fits in SRAM, but full-screen UI libraries (LVGL) usually perform better with buffers in PSRAM or careful DMA management.