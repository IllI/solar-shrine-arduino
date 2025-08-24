#line 1 "C:\\Users\\cityz\\IllI\\play\\soulShine\\solar-shrine-arduino\\arduino\\01_MAIN_SYSTEM\\solar_shrine_playa\\DjScratch.h"
#ifndef DJ_SCRATCH_H
#define DJ_SCRATCH_H

#include <Arduino.h>

namespace DjScratch {

  // Called once during the main setup
  void setup();

  // Called when switching to this effect
  void enter();

  // Called when switching away from this effect
  void exit();

  // Called from the main loop to update controls
  void update(bool leftHand, bool rightHand, float d1, float d2);

  // Called from the Timer1 ISR to generate audio
  void handleISR();

}

#endif // DJ_SCRATCH_H