#include "ThereminEffect.h"

#include <Oscil.h>
#include <Arduino.h>
#include <tables/sin2048_int8.h>
#include <RollingAverage.h>

namespace ThereminEffect {

  namespace {
    // Primary sine voice
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> voice(SIN2048_DATA);

    // Simple smoothing for frequency to avoid zipper noise
    const float kFreqSlew = 0.1f; // Smooth frequency changes

    // Frequency range for theremin (expanded to cover multiple octaves)
    const float kMaxHz = 2000.0f;  // High frequency limit (C7)
    const float kMinHz = 20.0f;    // Low frequency limit (E0 - very low bass)

    // Distance range for hand sensing (expanded to match actual sensor readings)
    const float kNearCm = 3.0f;   // Closest hand distance
    const float kFarCm = 100.0f;  // Farthest hand distance (expanded from 30cm)

    // State
    float currentFreq = 220.0f;
    float targetFreq = 220.0f;
    int g_volume255 = 0;
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    // Initialize oscillator
    currentFreq = 220.0f;
    targetFreq  = 220.0f;
    voice.setFreq(currentFreq);
  }

  void exit() {
    // Nothing to tear down specifically
  }

  void update(bool leftHand, bool rightHand, float d1_cm, float d2_cm) {
    // Only play when at least one hand is detected
    if (!leftHand && !rightHand) {
      g_volume255 = 0;  // Silence when no hands
      return;
    }
    
    // Volume: full volume when hands detected
    g_volume255 = 255;

    // Simple theremin: map distance directly to frequency
    // Use right hand for pitch control (like traditional theremin)
    if (rightHand) {
      // Constrain distance to reasonable range
      float distance = constrain(d2_cm, kNearCm, kFarCm);
      
      // Map distance to frequency (closer = higher pitch)
      // Use logarithmic mapping for more musical feel (like real theremin)
      float normalized = (distance - kNearCm) / (kFarCm - kNearCm); // 0 to 1
      normalized = 1.0f - normalized; // Invert so closer = higher
      
      // Logarithmic frequency mapping (like real theremin)
      // This gives exponential pitch change - more musical
      targetFreq = kMinHz * powf(kMaxHz / kMinHz, normalized);
    } else {
      // Default frequency when no right hand
      targetFreq = 220.0f;
    }

    // Smooth frequency changes to avoid zipper noise
    currentFreq += (targetFreq - currentFreq) * kFreqSlew;
    voice.setFreq(currentFreq);
  }

  int audio() {
    // Keep audio path simple and robust per guide: oscillator * volume (0..255)
    return voice.next() * g_volume255;
  }

}
