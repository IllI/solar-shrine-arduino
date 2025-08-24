#ifndef THEREMIN_EFFECT_H
#define THEREMIN_EFFECT_H

#include <Arduino.h>

namespace ThereminEffect {

  void setup();
  void enter();
  void exit();
  void update(bool leftHand, bool rightHand, float d1_cm, float d2_cm);
  int audio();

}

#endif // THEREMIN_EFFECT_H