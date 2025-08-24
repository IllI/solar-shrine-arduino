#line 1 "C:\\Users\\cityz\\IllI\\play\\soulShine\\solar-shrine-arduino\\arduino\\01_MAIN_SYSTEM\\solar_shrine_playa\\AlienEffect.h"
#ifndef ALIEN_EFFECT_H
#define ALIEN_EFFECT_H

#include <Arduino.h>

namespace AlienEffect {

  void setup();
  void enter();
  void exit();
  void update(bool leftHand, bool rightHand, float d1, float d2);
  int audio();

}

#endif // ALIEN_EFFECT_H