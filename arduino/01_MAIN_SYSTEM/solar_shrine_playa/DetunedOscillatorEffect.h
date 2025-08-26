#ifndef DETUNED_OSCILLATOR_EFFECT_H
#define DETUNED_OSCILLATOR_EFFECT_H

#include <Arduino.h>

namespace DetunedOscillatorEffect {

  void setup();
  void enter();
  void exit();
  void update(bool leftHand, bool rightHand, float d1, float d2);
  int audio();
  // Accessor for current smoothed level (0..255) used for LED visuals
  int level();

}

#endif // DETUNED_OSCILLATOR_EFFECT_H