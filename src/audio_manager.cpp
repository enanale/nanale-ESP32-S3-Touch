#include "audio_manager.h"
#include "driver/i2s_std.h"
#include "user_config.h"
#include <math.h>

static i2s_chan_handle_t tx_chan = NULL;

AudioManager::AudioManager() : _initialized(false) {}

bool AudioManager::begin() {
#if CONFIG_ENABLE_AUDIO
  Serial.println("[AUDIO] Initializing Standard Philips (Internal Clock)...");
  Serial.flush();

  if (!initCodec()) {
    Serial.println("[AUDIO] Error: ES8311 init failed.");
    Serial.flush();
    return false;
  }

  if (!initI2S()) {
    Serial.println("[AUDIO] Error: Failed to init I2S Philips!");
    Serial.flush();
    return false;
  }

  _initialized = true;
  Serial.println("[AUDIO] System Ready (Standard Philips)");

  return true;
#else
  return false;
#endif
}

bool AudioManager::initCodec() {
  delay(100);

  // --- Clock & Reset (ES8311 Regs 0x00 - 0x08) ---

  // Reg 0x00: Digital Reset & CSM. 0x80 = Reset digital, CSM on.
  writeReg(0x00, 0x80);

  // Reg 0x01: Master Clock Selection. 0xBF = 1011 1111.
  // Bit 7=1: Derive internal MCLK from BCLK/SCLK (Critical workaround for Pin
  // 7/2 issues). Bits 5:0=1: Enable internal logic clocks.
  writeReg(0x01, 0xBF);

  // Reg 0x02: Clock Divider. 0x18 = 0001 1000.
  // Bits 4:3 = 11: Set BCLK as the internal source for the MCLK-derivation
  // logic.
  writeReg(0x02, 0x18);

  // Regs 0x03-0x08: FSM & OSR Dividers. 0x10 and 0x00 values are standard
  // for 44.1kHz with a 64*fs or 32*fs Bit Clock.
  writeReg(0x03, 0x10);
  writeReg(0x04, 0x10);
  writeReg(0x05, 0x00);
  writeReg(0x06, 0x00);
  writeReg(0x07, 0x00);
  writeReg(0x08, 0x00);

  // --- Serial Data Port (Reg 0x09) ---

  // Reg 0x09: 0x0C = 0000 1100.
  // Bits 3:2 = 11: 16-bit word length.
  // Bits 1:0 = 00: Philips Standard I2S format.
  writeReg(0x09, 0x0C);

  // --- System Power & Enable (Regs 0x0B - 0x14) ---

  writeReg(0x0B, 0x00);
  writeReg(0x0C, 0x00);
  writeReg(0x0D, 0x01); // Power Up Analog
  writeReg(0x0E, 0x02); // Power Up DAC
  writeReg(0x10, 0x1F); // VREF Config
  writeReg(0x11, 0x7F); // Bias Config
  writeReg(0x12, 0x00); // Enable DAC Digital
  writeReg(0x14, 0x1A); // Analog Gain (PGA)

  // --- volume & Mute (Regs 0x31 - 0x32) ---

  writeReg(0x31, 0x00); // Analog Mute (0x00 = Unmute)
  writeReg(0x32, 0xBF); // Digital Volume (0xBF is approx 95%)

  return true;
}

bool AudioManager::initI2S() {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;
  if (i2s_new_channel(&chan_cfg, &tx_chan, NULL) != ESP_OK)
    return false;

  // Standard Philips: 44.1kHz, 16-bit, Stereo
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = I2S_GPIO_UNUSED, // Not using physical MCLK pin
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

  // High compatibility slot width
  std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;

  if (i2s_channel_init_std_mode(tx_chan, &std_cfg) != ESP_OK)
    return false;
  if (i2s_channel_enable(tx_chan) != ESP_OK)
    return false;

  return true;
}

bool AudioManager::writeReg(uint8_t r, uint8_t v) {
  Wire1.beginTransmission(ES8311_I2C_ADDR);
  Wire1.write(r);
  Wire1.write(v);
  return (Wire1.endTransmission() == 0);
}

// Final cleanup: removed dumpRegisters implementation

void AudioManager::playClick() {
#if CONFIG_ENABLE_AUDIO
  if (!_initialized)
    return;
  Serial.println("[AUDIO] Playing Click...");

  int num_samples = (44100 * 30) / 1000;
  int16_t *buf = (int16_t *)malloc(num_samples * 2 * sizeof(int16_t));
  if (!buf)
    return;

  for (int i = 0; i < num_samples; i++) {
    int16_t s = (i < num_samples / 2) ? 31000 : -31000;
    buf[i * 2] = s;     // Left
    buf[i * 2 + 1] = s; // Right
  }

  size_t written;
  i2s_channel_write(tx_chan, buf, num_samples * 2 * sizeof(int16_t), &written,
                    100);
  free(buf);
#endif
}

void AudioManager::playJingle() {
#if CONFIG_ENABLE_AUDIO
  if (!_initialized)
    return;
  Serial.println("[AUDIO] Playing Jingle...");

  int notes[] = {440, 554, 659, 880};
  int dur = 150;

  for (int n = 0; n < 4; n++) {
    int num_samples = (44100 * dur) / 1000;
    int16_t *buf = (int16_t *)malloc(num_samples * 2 * sizeof(int16_t));
    if (!buf)
      continue;

    for (int i = 0; i < num_samples; i++) {
      int16_t s = 25000 * sin(2 * 3.14159 * notes[n] * i / 44100);
      buf[i * 2] = s;
      buf[i * 2 + 1] = s;
    }

    size_t written;
    i2s_channel_write(tx_chan, buf, num_samples * 2 * sizeof(int16_t), &written,
                      500);
    free(buf);
    delay(30);
  }
#endif
}

void AudioManager::update() {}
