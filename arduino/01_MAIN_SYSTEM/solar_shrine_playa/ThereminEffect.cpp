#include "ThereminEffect.h"

#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <RollingAverage.h>

namespace ThereminEffect {

  namespace {
    Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> thereminOsc(SIN2048_DATA);
    RollingAverage <int, 4> thereminPitchAvg;
    RollingAverage <int, 8> thereminVolAvg;
    int thereminBaseFreq = 440;
    int thereminSmoothVol = 0;
    int thereminVol = 0;
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    thereminOsc.setFreq(440);
    thereminVol = 0;
    thereminSmoothVol = 0;
    thereminBaseFreq = 440;
  }

  void exit() {
    // No specific exit actions needed
  }

  void update(bool leftHand, bool rightHand, float d1, float d2) {
    int freq;
    if (rightHand) {
      freq = map(d2 * 10, 10, 200, 1046, 131);
      freq = constrain(freq, 131, 1046);
    } else {
      freq = 131;
    }
    thereminBaseFreq = thereminPitchAvg.next(freq);
    thereminOsc.setFreq(thereminBaseFreq);

    if (leftHand) {
      thereminVol = thereminVol + 4;
      if (thereminVol > 255) thereminVol = 255;
    } else {
      thereminVol = thereminVol - 4;
      if (thereminVol < 0) thereminVol = 0;
    }
    thereminSmoothVol = thereminVolAvg.next(thereminVol);
  }

  int audio() {
    return (thereminOsc.next() * thereminSmoothVol) >> 8;
  }

}