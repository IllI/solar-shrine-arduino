#ifndef MOZZI_THEREMIN_ECHO_EFFECT_H
#define MOZZI_THEREMIN_ECHO_EFFECT_H

#include <Arduino.h>

namespace MozziThereminEchoEffect {

  void setup();
  void enter();
  void exit();
  void update(bool leftHand, bool rightHand, float d1_cm, float d2_cm);
  int audio();

}

#endif // MOZZI_THEREMIN_ECHO_EFFECT_H