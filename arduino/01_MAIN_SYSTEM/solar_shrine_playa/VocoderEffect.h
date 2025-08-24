#ifndef VOCODER_EFFECT_H
#define VOCODER_EFFECT_H

#include <Arduino.h>
#include <stdint.h>

namespace VocoderEffect {

  // === RING BUFFER ===
  const int AUDIO_IN_BUFFER_SIZE = 128; // decoupled from internal frame sizes
  extern volatile int16_t audio_buffer[AUDIO_IN_BUFFER_SIZE];
  extern volatile uint8_t buffer_head;
  extern volatile uint8_t buffer_tail;

  // === PUBLIC FUNCTIONS ===
  void setup();
  void enter();
  void exit();
  void update(bool leftHand, bool rightHand, float d1, float d2);
  void feedAudioSample(int16_t sample);
  int audio();

}

#endif // VOCODER_EFFECT_H
