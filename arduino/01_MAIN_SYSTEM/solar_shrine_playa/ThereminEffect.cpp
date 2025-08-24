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
    
    // Volume control (like the example)
    byte volume = 0;
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
    // Volume control with left hand (like the example's potentiometer)
    if (leftHand) {
      // Map left hand distance to volume (closer = louder)
      float leftDistance = constrain(d1_cm, kNearCm, kFarCm);
      float normalizedLeft = (leftDistance - kNearCm) / (kFarCm - kNearCm); // 0 to 1
      normalizedLeft = 1.0f - normalizedLeft; // Invert so closer = louder
      volume = (byte)(normalizedLeft * 255); // 0-255 volume range
    } else {
      volume = 0; // Silence when no left hand
    }

    // Frequency control with right hand (like the example's LDR)
    if (rightHand) {
      // Map right hand distance directly to frequency (like the example)
      float rightDistance = constrain(d2_cm, kNearCm, kFarCm);
      
      // Direct frequency mapping (like the example's light_level)
      // Map distance range to frequency range
      float normalizedRight = (rightDistance - kNearCm) / (kFarCm - kNearCm); // 0 to 1
      normalizedRight = 1.0f - normalizedRight; // Invert so closer = higher pitch
      
      // Direct frequency calculation (like the example)
      targetFreq = kMinHz + (kMaxHz - kMinHz) * normalizedRight;
    } else {
      // Default frequency when no right hand
      targetFreq = 220.0f;
    }

    // Smooth frequency changes to avoid zipper noise
    currentFreq += (targetFreq - currentFreq) * kFreqSlew;
    voice.setFreq(currentFreq);
    
    // Update volume for audio output
    g_volume255 = volume;
  }

  int audio() {
    // Keep audio path simple and robust per guide: oscillator * volume (0..255)
    return voice.next() * g_volume255;
  }

}
