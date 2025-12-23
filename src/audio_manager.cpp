#include "audio_manager.h"
#include "user_config.h"
#include <Arduino.h>        // For Serial.println, delay, Wire
#include <driver/i2s_std.h> // Keep this for i2s_std types and functions
#include <math.h>

// ES8311 Register Definitions (Simplified)
#define ES8311_RESET_REG 0x00
#define ES8311_CLK_MANAGER_REG 0x01
#define ES8311_SDP_CONFIG_REG 0x02
#define ES8311_SYSTEM_CONFIG_REG 0x03
#define ES8311_DAC_CONFIG1_REG 0x31
#define ES8311_DAC_CONFIG2_REG 0x32
#define ES8311_DAC_VOLUME_REG 0x33

// I2S channel handle for the new I2S driver
static i2s_chan_handle_t tx_chan = NULL;

AudioManager::AudioManager() : _initialized(false) {}

bool AudioManager::begin() {
  Serial.println("[AUDIO] Initializing ES8311...");

  if (!initCodec()) {
    Serial.println("[AUDIO] Failed to init ES8311 Codec");
    return false;
  }

  if (!initI2S()) {
    Serial.println("[AUDIO] Failed to init I2S");
    return false;
  }

  _initialized = true;
  Serial.println("[AUDIO] System Ready");
  return true;
}

bool AudioManager::initCodec() {
  writeReg(ES8311_RESET_REG, 0x1F); // Full Reset
  delay(10);
  writeReg(ES8311_RESET_REG, 0x00); // Normal operation

  // Basic setup for 16kHz, 16-bit, I2S mode
  writeReg(0x01, 0x30); // Clock manager
  writeReg(0x02, 0x10); // SDP Config
  writeReg(0x03, 0x10); // System Config
  writeReg(0x31, 0x00); // DAC Config 1
  writeReg(0x32, 0x00); // DAC Config 2
  writeReg(0x33, 0xBF); // Volume (0-255)

  return true;
}

bool AudioManager::initI2S() {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &tx_chan, NULL);

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                  I2S_SLOT_MODE_MONO),
      .gpio_cfg =
          {
              .mclk = PIN_I2S_MCLK,
              .bclk = PIN_I2S_SCLK,
              .ws = PIN_I2S_LRCK,
              .dout = PIN_I2S_DOUT,
              .din = I2S_GPIO_UNUSED,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };

  i2s_channel_init_std_mode(tx_chan, &std_cfg);
  i2s_channel_enable(tx_chan);

  return true;
}

void AudioManager::writeReg(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(ES8311_I2C_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  Wire1.endTransmission();
}

void AudioManager::playClick() {
  if (!_initialized)
    return;

  // Simple square wave pulse for "click"
  int16_t sample[200];
  for (int i = 0; i < 200; i++) {
    sample[i] = (i < 100) ? 15000 : -15000;
  }

  size_t bytes_written;
  i2s_channel_write(tx_chan, sample, sizeof(sample), &bytes_written, 100);
}

void AudioManager::playJingle() {
  if (!_initialized)
    return;

  // Basic startup melody (3 notes)
  int notes[] = {440, 554, 659, 880};
  int duration = 150; // ms

  for (int n = 0; n < 4; n++) {
    int freq = notes[n];
    int num_samples = (44100 * duration) / 1000;
    int16_t *buf = (int16_t *)malloc(num_samples * sizeof(int16_t));

    for (int i = 0; i < num_samples; i++) {
      buf[i] = 10000 * sin(2 * M_PI * freq * i / 44100);
    }

    size_t bytes_written;
    i2s_channel_write(tx_chan, buf, num_samples * sizeof(int16_t),
                      &bytes_written, 500);
    free(buf);
    delay(30);
  }
}

void AudioManager::update() {
  // Background tasks if needed (e.g. queue management)
}
