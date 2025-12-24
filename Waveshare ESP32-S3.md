# **Waveshare ESP32-S3-Touch-LCD-3.49 Development Context**

## **1\. Product Overview**

The **ESP32-S3-Touch-LCD-3.49** is a highly integrated development board by Waveshare featuring a rectangular 3.49-inch touch display. It is designed for HMI (Human-Machine Interface) applications and features a unique ultra-wide aspect ratio.

* **Manufacturer:** Waveshare  
* **Model:** ESP32-S3-Touch-LCD-3.49  
* **Core:** ESP32-S3R8 (Dual-core Xtensa LX7, up to 240MHz)  
* **Memory:** 16MB Flash, 8MB PSRAM (Octal)

## **2\. Hardware Specifications**

### GPIO Pinout

| GPIO | LCD | SD Card | IMU | RTC | UART | I2S | KEY_IO | OUTPUT | Other |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| IO0 | | | | | | | BOOT0 | IO0 | |
| IO1 | | | | | | | | IO1 | |
| IO2 | | | | | | | | IO2 | |
| IO3 | | | | | | | | IO3 | |
| IO4 | | | | | | | | IO4 | BAT_ADC |
| IO5 | | | | | | | | IO5 | |
| IO6 | | | | | | I2S_DIN | | IO6 | |
| IO7 | | | | | | I2S_MCLK | | IO7 | |
| IO8 | LCD_BL | | | | | | | | |
| IO9 | LCD_CS | | | | | | | | |
| IO10 | LCD_SCL | | | | | | | | |
| IO11 | LCD_D0 | | | | | | | | |
| IO12 | LCD_D1 | | | | | | | | |
| IO13 | LCD_D2 | | | | | | | | |
| IO14 | LCD_D3 | | | | | | | | |
| IO15 | | | | | | I2S_SCLK | | IO15 | |
| IO16 | | | | | | | | IO16 | SYS_OUT |
| IO17 | TP_SDA | | | | | | | IO17 | |
| IO18 | TP_SCL | | | | | | | IO18 | |
| IO19 | | | | | | | | IO19 | U_N |
| IO20 | | | | | | | | IO20 | U_P |
| IO21 | LCD_RST | | | | | | | IO21 | |
| IO38 | | SD_CS | | | | | | IO38 | |
| IO39 | | SD_MOSI | | | | | | IO39 | |
| IO40 | | SD_MISO | | | | | | IO40 | |
| IO41 | | SD_SCLK | | | | | | IO41 | |
| IO42 | | | | | | | | IO42 | EXIO_INT |
| IO43 | | | | | TXD | | | IO43 | |
| IO44 | | | | | RXD | | | IO44 | |
| IO45 | | | | | | I2S_DOUT | | IO45 | |
| IO46 | | | | | | I2S_LRCK | | IO46 | |
| IO47 | | | IMU_SDA | RTC_SDA | | Audio_SDA | | IO47 | |
| IO48 | | | IMU_SCL | RTC_SCL | | Audio_SCL | | IO48 | |
| RESET | | | | | | | RESET | | |
| EXIO0 | TP_INT | | | | | | | EXIO0 | |
| EXIO1 | BL_EN | | | | | | | EXIO1 | |
| EXIO2 | | | IMU_INT1 | | | | | EXIO2 | |
| EXIO3 | | | IMU_INT2 | | | | | EXIO3 | |
| EXIO4 | | | | RTC_INT | | | | EXIO4 | |
| EXIO5 | LCD_TE | | | | | | | EXIO5 | |
| EXIO6 | | | | | | | | EXIO6 | SYS_EN |
| EXIO7 | | | | | | NS_MODE | | EXIO7 | |

**Key Notes from the Table**

* Shared Bus (I2C): Notice that IO47 and IO48 are heavily shared, carrying signals for the IMU, RTC, and Audio (I2S) simultaneously.

* Extended IO: The pins labeled EXIO likely refer to an IO Expander, meaning these are not direct GPIOs on the main MCU but are accessed via a separate interface (likely I2C).

* USB: IO19 and IO20 are marked U_N and U_P, indicating USB Negative and Positive differential data lines.

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
    * **Charging Status:** **HARDWARE LIMITATION**. The ETA6098 STAT pin (Pin 9) is connected only to an LED and is **not** wired to any ESP32 GPIO or IO Expander pin. Software cannot detect charging state or USB presence on this board revision.
    * **Schematic:** [ESP32-S3-Touch-LCD-3.49 Schematic](https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-3.49/ESP32-S3-Touch-LCD-3.49-Schematic.pdf)

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
| **I2S\_MCLK** | GPIO 7 | Master Clock (Internal derivation recommended) |
| **I2S\_SCLK** | GPIO 15 | Bit Clock (BCLK) |
| **I2S\_LRCK** | GPIO 46 | Word Select (WS/LRCK) |
| **I2S\_DOUT** | GPIO 45 | Data Out (To ES8311 DAC) |
| **I2S\_DIN** | GPIO 6 | Data In (From ES7210 ADC) |

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

* **Reading:** When INT goes low, read I2C to get X/Y coordinates. 
* **Coordinate Mapping:** LVGL V9 expects input device coordinates in the **physical (unrotated) portrait frame** (172 x 640). If the display is rotated 270 degrees in software, the driver must map raw touch points accordingly:
    *   **Physical X** = `pointY` (short axis)
    *   **Physical Y** = `(640 - 1) - pointX` (long axis)
    *   *Note: Native hardware origin is effectively the lower-right corner when held horizontally.*

### **4\. Audio (Recording & Playback)**

This board uses **ES7210** for recording (ADC) and **ES8311** for playback (DAC). Both must be configured via I2C before I2S data transfer works.

**I2S Configuration (ESP-IDF / Arduino):**

\#include \<driver/i2s.h\>

\#define I2S\_NUM         I2S\_NUM\_0  
\#define I2S\_MCLK\_GPIO   7  
\#define I2S\_BCLK\_GPIO   15  
\#define I2S\_LRCK\_GPIO   46  
\#define I2S\_DOUT\_GPIO   45  
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

    i2s\_driver\_install(I2S\_NUM, \&i2s\_config, 0, NULL);
    i2s\_set\_pin(I2S\_NUM, \&pin\_config);

    // CRITICAL: For ES8311 on this board, Internal Clocking is recommended.
    // Configure ES8311 via I2C to derive internal MCLK from BCLK (Reg 0x01 = 0xBF).
}

#### **ES7210 (ADC/Microphone) Critical Configuration**

**⚠️ CRITICAL: I2C Bus Usage**
- ES7210 is on **Wire1** (GPIO 47/48 - System Bus), **NOT Wire** (GPIO 17/18 - Touch Bus)
- Using the wrong I2C bus will cause "device not found" errors
- Always use `Wire1.beginTransmission(0x40)` for ES7210

**Vendor-Verified Configuration (24kHz, 16-bit):**

The vendor's ESP-IDF example uses:
- **Sample Rate**: 24000 Hz (not 44.1kHz)
- **Bit Depth**: 16-bit
- **Mode**: TDM mode in vendor code, but Standard I2S also works
- **I2C Address**: 0x40

**Critical ES7210 Registers (from vendor driver):**

```cpp
// Reset and wake
writeRegES7210(0x00, 0xFF);  // Reset
writeRegES7210(0x00, 0x41);  // Wake

// Clock configuration
writeRegES7210(0x01, 0x3F);  // Clock off register
writeRegES7210(0x02, 0xC1);  // MAINCLK - clear state (CRITICAL)
writeRegES7210(0x06, 0x00);  // Power up
writeRegES7210(0x07, 0x20);  // OSR
writeRegES7210(0x08, 0x00);  // Slave mode

// TDM and HPF (vendor-specific)
writeRegES7210(0x09, 0x30);  // TDM mode / chip state cycle
writeRegES7210(0x0A, 0x30);  // TDM slot / power on cycle
writeRegES7210(0x20, 0x0A);  // ADC34 HPF2
writeRegES7210(0x21, 0x2A);  // ADC34 HPF1
writeRegES7210(0x22, 0x0A);  // ADC12 HPF1
writeRegES7210(0x23, 0x2A);  // ADC12 HPF2

// Format
writeRegES7210(0x11, 0x60);  // 16-bit Philips I2S
writeRegES7210(0x12, 0x00);  // Normal operation (not TDM)

// Microphone configuration
writeRegES7210(0x40, 0x43);  // Analog power
writeRegES7210(0x41, 0x70);  // MIC12 bias 2.87v
writeRegES7210(0x42, 0x70);  // MIC34 bias 2.87v

// CRITICAL MIC POWER REGISTERS (from vendor)
writeRegES7210(0x47, 0x08);  // MIC1 power - CRITICAL!
writeRegES7210(0x48, 0x08);  // MIC2 power - CRITICAL!
writeRegES7210(0x49, 0x08);  // MIC3 power - CRITICAL!
writeRegES7210(0x4A, 0x08);  // MIC4 power - CRITICAL!
writeRegES7210(0x4B, 0x00);  // MIC12 power enable

// Mic gains
writeRegES7210(0x43, 0x1F);  // MIC1 gain max + enable
writeRegES7210(0x44, 0x1F);  // MIC2 gain max + enable

// Final reset (vendor sequence)
writeRegES7210(0x00, 0x71);
writeRegES7210(0x00, 0x41);
```

**Known Issues:**
- Some boards may have defective ES7210 chips or broken I2S traces
- If ES7210 initializes on I2C but produces zero audio data despite correct configuration, suspect hardware issue
- Vendor example uses 24kHz sample rate, not 44.1kHz

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