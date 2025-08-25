#include "MozziThereminEchoEffect.h"

#include <Oscil.h>
#include <Arduino.h>
#include <tables/sin2048_int8.h>
#include <RollingAverage.h>
#include <ControlDelay.h>

namespace MozziThereminEchoEffect {

  namespace {
    // Echo delay configuration
    unsigned int echo_cells_1 = 32;
    unsigned int echo_cells_2 = 60;
    unsigned int echo_cells_3 = 127;
    
    ControlDelay<128, int> kDelay; // 2 seconds
    
    // Oscillators for echo harmonics
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin0(SIN2048_DATA);
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin1(SIN2048_DATA);
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin2(SIN2048_DATA);
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> aSin3(SIN2048_DATA);
    
    // Smoothing for sensor input
    RollingAverage<int, 32> kAverage; // power of 2
    int averaged;
    
    // Gate audio when no hands detected (like solar_shrine_playa)
    static volatile bool g_handsActive = false;
    
    // Hand detection parameters
    const float kNearCm = 1.0f;   // Closest hand distance
    const float kFarCm = 20.0f;   // Farthest hand distance
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    // Initialize delay
    kDelay.set(echo_cells_1);
    
    // Initialize oscillators with default frequencies
    aSin0.setFreq(200);
    aSin1.setFreq(200);
    aSin2.setFreq(200);
    aSin3.setFreq(200);
    
    g_handsActive = false;
  }

  void exit() {
    // Reset state
    g_handsActive = false;
  }

  void update(bool leftHand, bool rightHand, float d1_cm, float d2_cm) {
    // Check if any hands are present
    g_handsActive = (leftHand || rightHand);
    
    // Use right hand for primary frequency control (like ThereminEffect)
    int sensor_input = 0;
    if (rightHand) {
      // Map distance to frequency range (closer = higher frequency)
      float rightDistance = constrain(d2_cm, kNearCm, kFarCm);
      float normalizedDistance = (rightDistance - kNearCm) / (kFarCm - kNearCm); // 0 to 1
      normalizedDistance = 1.0f - normalizedDistance; // Invert so closer = higher
      sensor_input = (int)(normalizedDistance * 1023); // Convert to 0-1023 range
    } else {
      sensor_input = 200; // Default frequency when no hand
    }
    
    // Apply smoothing and echo effects
    averaged = kAverage.next(sensor_input);
    aSin0.setFreq(averaged);
    aSin1.setFreq(kDelay.next(averaged));
    aSin2.setFreq(kDelay.read(echo_cells_2));
    aSin3.setFreq(kDelay.read(echo_cells_3));
  }

  int audio() {
    // Hard gate: never output audio without hands detected (like solar_shrine_playa)
    if (!g_handsActive) {
      return 0;
    }
    
    // Use the better-sounding fromAlmostNBit implementation with high amplification
    // Convert to simple oscillator * volume pattern as required by guide
    int sample = 24 * ((int)aSin0.next() + aSin1.next() + (aSin2.next() >> 1) + (aSin3.next() >> 2));
    
    // Apply maximum volume (255) as per guide
    return (sample * 255) >> 8;
  }

}