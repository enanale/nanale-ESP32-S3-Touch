#include "audio_manager.h"
#include "driver/i2s_std.h"
#include "user_config.h"
#include <math.h>

static i2s_chan_handle_t tx_chan = NULL;
static i2s_chan_handle_t rx_chan = NULL; // Add RX channel for recording

AudioManager::AudioManager()
    : _initialized(false), _isRecording(false), _isMemoPlaying(false),
      _memoBuffer(nullptr), _memoSize(0), _maxMemoSamples(0) {}

bool AudioManager::begin() {
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

  if (!initES7210()) {
    Serial.println("[AUDIO] Error: ES7210 init failed.");
    Serial.flush();
    return false;
  }

  _initialized = true;
  Serial.println("[AUDIO] System Ready (DAC + MIC)");

  return true;
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

  // Create both TX and RX channels for playback and recording
  if (i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan) != ESP_OK)
    return false;

  // Standard Philips: 24kHz (vendor config), 16-bit, Stereo
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(24000), // 24kHz like vendor
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                  I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk =
                  PIN_I2S_MCLK, // Enable MCLK for ES7210 (ES8311 uses internal)
              .bclk = PIN_I2S_SCLK,
              .ws = PIN_I2S_LRCK,
              .dout = PIN_I2S_DOUT,
              .din = PIN_I2S_DIN, // Enable DIN for recording!
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

  // Initialize both TX and RX channels
  if (i2s_channel_init_std_mode(tx_chan, &std_cfg) != ESP_OK)
    return false;
  if (i2s_channel_init_std_mode(rx_chan, &std_cfg) != ESP_OK)
    return false;

  // Enable both channels
  if (i2s_channel_enable(tx_chan) != ESP_OK)
    return false;
  if (i2s_channel_enable(rx_chan) != ESP_OK)
    return false;

  return true;
}

bool AudioManager::writeReg(uint8_t r, uint8_t v) {
  Wire1.beginTransmission(ES8311_I2C_ADDR);
  Wire1.write(r);
  Wire1.write(v);
  return (Wire1.endTransmission() == 0);
}

bool AudioManager::initES7210() {
  Serial.println("[AUDIO] Initializing ES7210...");
  delay(10);

  bool ok = true;

  // CRITICAL: Test I2C communication on Wire1 (NOT Wire!)
  Wire1.beginTransmission(ES7210_I2C_ADDR);
  if (Wire1.endTransmission() != 0) {
    Serial.println("[AUDIO] ERROR: ES7210 not responding on I2C!");
    return false;
  }
  Serial.println("[AUDIO] ES7210 I2C communication OK");

  // Reset and wake
  ok &= writeRegES7210(0x00, 0xFF); // Reset
  delay(10);
  ok &= writeRegES7210(0x00, 0x41); // Wake

  // Clock configuration (24kHz from vendor table)
  ok &= writeRegES7210(0x01, 0x01); // Clock ON for 24kHz (was 0x3F = OFF!)
  ok &= writeRegES7210(0x02, 0xC1); // MAINCLK - clear state (CRITICAL)
  ok &= writeRegES7210(0x06, 0x00); // Power up
  ok &= writeRegES7210(0x07, 0x20); // OSR
  ok &= writeRegES7210(0x08, 0x00); // Slave mode

  // TDM and HPF (vendor-specific)
  ok &= writeRegES7210(0x09, 0x30); // TDM mode / chip state cycle
  ok &= writeRegES7210(0x0A, 0x30); // TDM slot / power on cycle
  ok &= writeRegES7210(0x20, 0x0A); // ADC34 HPF2
  ok &= writeRegES7210(0x21, 0x2A); // ADC34 HPF1
  ok &= writeRegES7210(0x22, 0x0A); // ADC12 HPF1
  ok &= writeRegES7210(0x23, 0x2A); // ADC12 HPF2

  // Format
  ok &= writeRegES7210(0x11, 0x60); // 16-bit Philips I2S
  ok &= writeRegES7210(0x12, 0x00); // Normal operation (not TDM)

  // Microphone configuration
  ok &= writeRegES7210(0x40, 0x43); // Analog power
  ok &= writeRegES7210(0x41, 0x70); // MIC12 bias 2.87v
  ok &= writeRegES7210(0x42, 0x70); // MIC34 bias 2.87v

  // CRITICAL MIC POWER REGISTERS (from vendor)
  ok &= writeRegES7210(0x47, 0x08); // MIC1 power - CRITICAL!
  ok &= writeRegES7210(0x48, 0x08); // MIC2 power - CRITICAL!
  ok &= writeRegES7210(0x49, 0x08); // MIC3 power - CRITICAL!
  ok &= writeRegES7210(0x4A, 0x08); // MIC4 power - CRITICAL!
  ok &= writeRegES7210(0x4B, 0x00); // MIC12 power enable

  // Mic gains
  ok &= writeRegES7210(0x43, 0x1F); // MIC1 gain max + enable
  ok &= writeRegES7210(0x44, 0x1F); // MIC2 gain max + enable

  // Final reset (vendor sequence)
  ok &= writeRegES7210(0x00, 0x71);
  ok &= writeRegES7210(0x00, 0x41);

  if (ok)
    Serial.println("[AUDIO] ES7210 Configured OK.");
  else
    Serial.println("[AUDIO] Warning: ES7210 Config failed (I2C error).");

  return ok;
}

bool AudioManager::writeRegES7210(uint8_t r, uint8_t v) {
  Wire1.beginTransmission(ES7210_I2C_ADDR); // CRITICAL: Wire1!
  Wire1.write(r);
  Wire1.write(v);
  return (Wire1.endTransmission() == 0);
}

// Final cleanup: removed dumpRegisters implementation

void AudioManager::playClick() {
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
}

void AudioManager::playJingle() {
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
}

void AudioManager::update() {}

// ===== Voice Memo Recording Functions =====

// Forward declaration for recording task
void audio_recording_task_real(void *arg);

bool AudioManager::startRecording() {
  if (_isRecording) {
    Serial.println("[AUDIO] Already recording!");
    return false;
  }

  // Allocate PSRAM buffer for memo
  _memoBuffer = (int16_t *)ps_malloc(3 * 1024 * 1024); // 3MB in PSRAM
  if (!_memoBuffer) {
    Serial.println("[AUDIO] ERROR: Failed to allocate PSRAM for memo!");
    return false;
  }

  _isRecording = true;
  _memoSize = 0;
  _maxMemoSamples = (3 * 1024 * 1024) / sizeof(int16_t);

  // Start recording task
  xTaskCreatePinnedToCore(audio_recording_task_real, "audio_rec", 4096, this, 3,
                          NULL, 1);

  Serial.println("[AUDIO] Recording Started (PSRAM)...");
  return true;
}

void AudioManager::stopRecording() {
  if (!_isRecording)
    return;

  _isRecording = false;
  Serial.printf("[AUDIO] Recording Stopped. Captured %d bytes.\n", _memoSize);
}

void AudioManager::startMemoPlayback() {
  if (_memoSize == 0) {
    Serial.println("[AUDIO] No recording to play!");
    return;
  }

  if (_isMemoPlaying) {
    Serial.println("[AUDIO] Already playing!");
    return;
  }

  _isMemoPlaying = true;
  Serial.printf("[AUDIO] Playing memo (%d bytes)...\n", _memoSize);

  size_t written = 0;
  size_t totalWritten = 0;

  while (totalWritten < _memoSize && _isMemoPlaying) {
    size_t chunkSize =
        (_memoSize - totalWritten > 4096) ? 4096 : (_memoSize - totalWritten);

    if (i2s_channel_write(tx_chan, (uint8_t *)_memoBuffer + totalWritten,
                          chunkSize, &written, 100) == ESP_OK) {
      totalWritten += written;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  _isMemoPlaying = false;
  Serial.println("[AUDIO] Playback complete.");
}

// Recording task implementation
void audio_recording_task_real(void *arg) {
  AudioManager *mgr = (AudioManager *)arg;
  int16_t *buffer = mgr->getMemoBuffer();
  uint32_t maxSamples = mgr->getMaxMemoSamples();

  Serial.println("[AUDIO] Recording task started");

  while (mgr->isRecording() &&
         (mgr->getMemoSize() / sizeof(int16_t)) < maxSamples) {
    // I2S sends 32-bit slots with 16-bit data left-aligned in upper bits
    int32_t chunk32[256]; // Read as 32-bit
    size_t bytesRead = 0;

    // Read from I2S RX channel (32-bit slots)
    if (i2s_channel_read(rx_chan, chunk32, sizeof(chunk32), &bytesRead, 100) ==
        ESP_OK) {
      if (bytesRead > 0) {
        size_t samples32 = bytesRead / sizeof(int32_t);
        size_t currentSamples = mgr->getMemoSize() / sizeof(int16_t);

        if (currentSamples + samples32 <= maxSamples) {
          // DIAGNOSTIC: Log first raw 32-bit value to see what we're getting
          static bool firstLog = true;
          if (firstLog && samples32 > 0) {
            Serial.printf(
                "[DEBUG] Raw 32-bit I2S values: 0x%08X, 0x%08X, 0x%08X\n",
                chunk32[0], chunk32[1], chunk32[2]);
            Serial.printf("[DEBUG] Upper 16 bits: %d, %d, %d\n",
                          (int16_t)(chunk32[0] >> 16),
                          (int16_t)(chunk32[1] >> 16),
                          (int16_t)(chunk32[2] >> 16));
            Serial.printf("[DEBUG] Lower 16 bits: %d, %d, %d\n",
                          (int16_t)(chunk32[0] & 0xFFFF),
                          (int16_t)(chunk32[1] & 0xFFFF),
                          (int16_t)(chunk32[2] & 0xFFFF));
            firstLog = false;
          }

          // Extract upper 16 bits from each 32-bit sample (left-aligned data)
          for (size_t i = 0; i < samples32; i++) {
            // Shift right by 16 to get the upper 16 bits
            buffer[currentSamples + i] = (int16_t)(chunk32[i] >> 16);
          }
          mgr->addMemoSize(samples32 * sizeof(int16_t));

          // VU meter - calculate peak for this chunk
          int16_t peak = 0;
          for (size_t i = 0; i < samples32; i++) {
            int16_t sample = buffer[currentSamples + i];
            if (abs(sample) > peak)
              peak = abs(sample);
          }

          // Only log if there's any non-zero signal
          if (peak > 0) {
            Serial.printf("[VU] Peak: %d | First samples: %d, %d, %d\n", peak,
                          buffer[currentSamples], buffer[currentSamples + 1],
                          buffer[currentSamples + 2]);
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  Serial.println("[AUDIO] Recording task ended");
  vTaskDelete(NULL);
}
