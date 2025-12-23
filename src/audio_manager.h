#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>

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
  void writeReg(uint8_t reg, uint8_t val);

  bool _initialized;
  i2s_port_t _i2s_port;
};

#endif
