#include "ThereminEffect.h"

#include <Oscil.h>
#include <tables/saw2048_int8.h>

namespace ThereminEffect {

  namespace {
    Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> thereminOsc(SAW2048_DATA);
    int thereminPitchPattern[] = {220, 330, 440, 330, 220, 165, 220, 330};
    int thereminPatternIndex = 0;
    unsigned long lastThereminChange = 0;
    const unsigned long THEREMIN_NOTE_DURATION = 200;
    int thereminVolume = 255; // Max volume
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    thereminOsc.setFreq(thereminPitchPattern[0]);
    lastThereminChange = millis();
  }

  void exit() {
    // No specific exit actions needed
  }

  void update() {
    if (millis() - lastThereminChange >= THEREMIN_NOTE_DURATION) {
      thereminPatternIndex = (thereminPatternIndex + 1) % 8;
      thereminOsc.setFreq(thereminPitchPattern[thereminPatternIndex]);
      lastThereminChange = millis();
    }
  }

  int audio() {
    return (thereminOsc.next() * thereminVolume);
  }

}
