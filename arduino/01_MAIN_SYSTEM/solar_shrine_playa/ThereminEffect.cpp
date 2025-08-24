#include "ThereminEffect.h"

#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <RollingAverage.h>
#include <mozzi_midi.h>  // header-only, safe

// Analog input for theremin pitch control (can be remapped to hand sensor mapping if needed)
#define THEREMIN_PITCH_PIN A0

namespace ThereminEffect {

  namespace {
    // Primary sine voice
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> voice(SIN2048_DATA);
    // Low-frequency vibrato
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> lfo(SIN2048_DATA);

    // Rolling average for pitch smoothing (power-of-2 window)
    RollingAverage<int, 16> pitchAvg;

    // Config
    const uint8_t kMinMidi = 40;     // ~82 Hz
    const uint8_t kMaxMidi = 88;     // ~1319 Hz
    const float   kVibDepthSemis = 0.2f; // subtle vibrato depth

    // State
    float currentFreq = 220.0f;
    float targetFreq  = 220.0f;
    const float kGlideAlpha = 0.2f;  // glide smoothing

    // Shared volume for audio() (0..255)
    int g_volume255 = 0;
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    // Initialize oscillators
    lfo.setFreq(5.0f);     // ~5 Hz vibrato
    currentFreq = 220.0f;
    targetFreq  = 220.0f;
    voice.setFreq(currentFreq);
  }

  void exit() {
    // Nothing to tear down specifically
  }

  void update(bool leftHand, bool rightHand, float d1_cm, float d2_cm) {
    // Volume: left hand controls loudness (closer = louder)
    static int volume255 = 0;
    if (leftHand) {
      // Map 1..20 cm => 255..50 (closer louder)
      int vol = map((int)d1_cm, 1, 20, 255, 50);
      if (vol < 0) vol = 0; if (vol > 255) vol = 255;
      volume255 = vol;
    } else {
      volume255 = 0; // silence when no left hand
    }

    // Pitch: right hand controls pitch (closer = higher)
    int midi = (kMinMidi + kMaxMidi) / 2; // default mid
    if (rightHand) {
      // Use same scale reference as guide (distance *10 like their example), but based on cm input
      int scaled = (int)(d2_cm * 10.0f); // 10..200 typical
      scaled = constrain(scaled, 10, 200);
      // Map 10..200 => high..low midi (closer => higher)
      midi = map(scaled, 10, 200, kMaxMidi, kMinMidi);
    }

    // Gentle vibrato in semitones using LFO (-128..127)
    int v = lfo.next();
    float vibSemis = kVibDepthSemis * ((float)v / 128.0f);

    // Target frequency with vibrato
    targetFreq = mtof((float)midi + vibSemis);

    // Glide toward target for stability
    currentFreq += (targetFreq - currentFreq) * kGlideAlpha;
    voice.setFreq(currentFreq);

    // Stash volume for audio()
    g_volume255 = volume255;
  }

  int audio() {
    // Keep audio path simple and robust per guide: oscillator * volume (0..255)
    return voice.next() * g_volume255;
  }

}
