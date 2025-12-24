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

  // Voice Memo Methods
  bool startRecording();
  void stopRecording();
  bool isRecording() const { return _isRecording; }

  void startMemoPlayback();
  bool isMemoPlaying() const { return _isMemoPlaying; }
  bool hasRecording() const { return _memoSize > 0; }

  // Internal helpers for task
  uint32_t getMemoSize() const { return _memoSize; }
  int16_t *getMemoBuffer() { return _memoBuffer; }
  void addMemoSize(size_t bytes) { _memoSize += bytes; }
  uint32_t getMaxMemoSamples() const { return _maxMemoSamples; }

private:
  bool initCodec();
  bool initI2S();
  bool initES7210();
  bool writeReg(uint8_t reg, uint8_t val);       // ES8311
  bool writeRegES7210(uint8_t reg, uint8_t val); // ES7210

  bool _initialized;
  bool _isRecording;
  bool _isMemoPlaying;

  int16_t *_memoBuffer;
  uint32_t _memoSize; // Current size in bytes
  uint32_t _maxMemoSamples;
};

#endif
