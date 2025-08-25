// ControlDelay-based theremin adaptation (Mozzi example) for ScaleEffect
#include "ScaleEffect.h"

#include <Oscil.h>
#include <RollingAverage.h>
#include <ControlDelay.h>
#include <tables/sin2048_int8.h>

namespace ScaleEffect {

  // Keep ring buffer symbols to satisfy header (unused in this adaptation)
  volatile int16_t audio_buffer[AUDIO_IN_BUFFER_SIZE];
  volatile uint8_t buffer_head = 0;
  volatile uint8_t buffer_tail = 0;

  namespace {
    // Four sine voices like the Mozzi example
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin0(SIN2048_DATA);
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin1(SIN2048_DATA);
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin2(SIN2048_DATA);
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin3(SIN2048_DATA);

    // Echo taps and delay buffer
    unsigned int echo_cells_1 = 32;
    unsigned int echo_cells_2 = 60;
    unsigned int echo_cells_3 = 127;
    ControlDelay<128, int> kDelay; // ~2s at CONTROL_RATE=64

    // Control smoothing
    RollingAverage<int, 32> kAverage; // power-of-2 window
    int averaged = 0;
    int lastCtrl = 440; // Hz fallback

    bool active = false; // silence when no valid hands

    inline bool validDistance(float d) { return d > 5 && d < 200; }
  }

  void setup() {
    // no-op
  }

  void enter() {
    // Initialize voices and delay
    aSin0.setFreq(220);
    aSin1.setFreq(220);
    aSin2.setFreq(220);
    aSin3.setFreq(220);
    kDelay.set(echo_cells_1);
    averaged = 0;
    lastCtrl = 440;
    active = false;
  }

  void exit() {
    active = false;
  }

  // Map sensor distances to a 0..1023 "control" (like mozziAnalogRead)
  static int mapDistanceToControl(float d) {
    // Use 10..150 cm window; closer => larger control
    if (d < 10) d = 10; if (d > 150) d = 150;
    float norm = (150.0f - d) / (150.0f - 10.0f); // 0..1
    int ctrl = (int)(norm * 1023.0f);
    if (ctrl < 0) ctrl = 0; if (ctrl > 1023) ctrl = 1023;
    return ctrl;
  }

  void update(bool leftHand, bool rightHand, float d1, float d2) {
    // Prefer right-hand distance for pitch control; fallback to left
    int ctrl = lastCtrl;
    bool have = false;
    if (rightHand && validDistance(d2)) {
      ctrl = mapDistanceToControl(d2);
      have = true;
    } else if (leftHand && validDistance(d1)) {
      ctrl = mapDistanceToControl(d1);
      have = true;
    }

    if (!have) {
      active = false; // no valid hands -> silence in audio()
      return;
    }

    lastCtrl = ctrl;
    active = true;

    // Smooth and drive the echo taps exactly like the example
    averaged = kAverage.next(ctrl);
    aSin0.setFreq(averaged);
    aSin1.setFreq(kDelay.next(averaged));
    aSin2.setFreq(kDelay.read(echo_cells_2));
    aSin3.setFreq(kDelay.read(echo_cells_3));
  }

  void feedAudioSample(int16_t) {
    // unused in this adaptation
  }

  int audio() {
    if (!active) return 0;
    // Mix like the example: 3*(a0 + a1 + a2/2 + a3/4)
    int a0 = aSin0.next();
    int a1 = aSin1.next();
    int a2 = aSin2.next();
    int a3 = aSin3.next();
    int sum = 3 * ( (int)a0 + a1 + (a2 >> 1) + (a3 >> 2) );
    // Soft gain to avoid clipping; clip to int8
    int outI = sum >> 2; // reduce a bit compared to Mozzi's fromAlmostNBit
    if (outI > 127) outI = 127;
    if (outI < -128) outI = -128;
    return outI * 255;
  }
}
