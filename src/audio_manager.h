#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s_std.h>

class AudioManager {
public:
  AudioManager();
  bool begin();
  void playClick();
  void playJingle();
  void update();

private:
  bool initCodec();
  bool initI2S();
  bool writeReg(uint8_t reg, uint8_t val); // Returns true on ACK

  bool _initialized;
};

#endif
