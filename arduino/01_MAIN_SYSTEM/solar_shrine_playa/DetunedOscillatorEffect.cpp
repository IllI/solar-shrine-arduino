#include "DetunedOscillatorEffect.h"

#include <Oscil.h>
#include <Line.h>
#include <Arduino.h>
#include <tables/saw8192_int8.h>
#include <Smooth.h>
#include <mozzi_rand.h>
#include <mozzi_midi.h>
#include <mozzi_fixmath.h>

namespace DetunedOscillatorEffect {

  namespace {
    // Oscillators for detuned effect
    Oscil<SAW8192_NUM_CELLS, AUDIO_RATE> aSaw1(SAW8192_DATA);
    Oscil<SAW8192_NUM_CELLS, AUDIO_RATE> aSaw2(SAW8192_DATA);

    // Lines for smooth frequency transitions
    Line<long> aGliss1;
    Line<long> aGliss2;

    // Smooth sensor readings (less smoothing for more responsive right hand control)
    Smooth<float> leftDistanceSmooth(0.85f);  // Keep smooth for base frequency
    Smooth<float> rightDistanceSmooth(0.6f);  // Less smoothing for more responsive detuning
    Smooth<int> volumeSmooth(0.9f);

    // Musical range (following mozzi_detuned_oscillators)
    const byte lo_note = 24; // C1
    const byte hi_note = 72; // C5

    // Timing for smooth transitions (faster than original to reduce lag)
    const long audio_steps_per_gliss = AUDIO_RATE / 16; // Faster transitions
    const long control_steps_per_gliss = CONTROL_RATE / 16; // Faster control updates

    // State variables
    float leftDistance = 30.0;
    float rightDistance = 30.0;
    bool leftHandPresent = false;
    bool rightHandPresent = false;
    int glissCounter = 0;
    int smoothedLevel = 0; // For LED visuals (0..255)
    
    // Hand detection parameters
    const float kNearCm = 1.0f;
    const float kFarCm = 25.0f; // Hand present within 25cm
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    // Initialize oscillators with default frequencies
    aSaw1.setFreq(220);
    aSaw2.setFreq(220);
    
    // Reset Line objects to prevent accumulation
    aGliss1.set(aSaw1.phaseIncFromFreq(220), aSaw1.phaseIncFromFreq(220), audio_steps_per_gliss);
    aGliss2.set(aSaw2.phaseIncFromFreq(220), aSaw2.phaseIncFromFreq(220), audio_steps_per_gliss);
    
    // Reset Smooth objects to prevent accumulation
    leftDistanceSmooth.next(30.0);
    rightDistanceSmooth.next(30.0);
    volumeSmooth.next(0);
    
    // Reset state
    leftDistance = 30.0;
    rightDistance = 30.0;
    leftHandPresent = false;
    rightHandPresent = false;
    glissCounter = 0;
    smoothedLevel = 0;
  }

  void exit() {
    // Reset state
    leftHandPresent = false;
    rightHandPresent = false;
    smoothedLevel = 0;
  }

  void update(bool leftHand, bool rightHand, float d1_cm, float d2_cm) {
    // Update hand presence
    leftHandPresent = leftHand;
    rightHandPresent = rightHand;
    
    // Smooth the distance readings (like the working prototype)
    leftDistance = leftDistanceSmooth.next(d1_cm);
    rightDistance = rightDistanceSmooth.next(d2_cm);
    
    // Update frequencies based on sensor input (restore original timing logic)
    if (--glissCounter <= 0) {
      int baseNote = lo_note;
      int detuneAmount = 0;
      
      // Left hand controls base frequency (like prototype)
      if (leftHandPresent) {
        // Map distance to musical note with smooth scaling (restore original logic)
        float normalizedDist = constrain((kFarCm - leftDistance) / (kFarCm - kNearCm), 0.0f, 1.0f);
        float exponentialScale = normalizedDist * normalizedDist; // Square for exponential feel
        baseNote = lo_note + (int)(exponentialScale * (hi_note - lo_note));
      }
      
      // Right hand controls detuning amount (like prototype)
      if (rightHandPresent) {
        float normalizedRight = constrain((kFarCm - rightDistance) / (kFarCm - kNearCm), 0.0f, 1.0f);
        detuneAmount = (int)(normalizedRight * 12); // 0-12 semitones detune
      }
      
      // Calculate frequencies (restore original approach)
      int freq1 = mtof(baseNote);
      int freq2 = mtof(baseNote + detuneAmount);
      
      // Add random variation for chorus effect (restore original)
      long variation1 = rand(8191);
      long variation2 = rand(8191);
      
      // Calculate phase increments (restore original method)
      long gliss_start1 = aSaw1.phaseIncFromFreq(freq1);
      long gliss_end1 = aSaw1.phaseIncFromFreq(freq1);
      long gliss_start2 = aSaw2.phaseIncFromFreq(freq2);
      long gliss_end2 = aSaw2.phaseIncFromFreq(freq2);
      
      // Set up smooth transitions (restore original Line usage)
      aGliss1.set(gliss_start1 + variation1, gliss_end1 + variation1, audio_steps_per_gliss);
      aGliss2.set(gliss_start2 + variation2, gliss_end2 + variation2, audio_steps_per_gliss);
      
      glissCounter = control_steps_per_gliss;
    }
    
    // Update smoothed level for LED visuals only (not audio volume)
    // Audio volume is handled by hard gating in audio() function like prototype
    int targetLevel = 0;
    if (leftHandPresent && rightHandPresent) {
      targetLevel = 255; // Full brightness when both hands present
    } else if (leftHandPresent || rightHandPresent) {
      targetLevel = 180; // Moderate brightness if only one hand
    } else {
      targetLevel = 0; // Dark when no hands
    }
    
    smoothedLevel = volumeSmooth.next(targetLevel);
  }

  int audio() {
    // CRITICAL: Use EXACT same pattern as working mozzi_theremin_echo prototype
    // Hard gate at effect level - never output audio when no hands detected
    if (!leftHandPresent && !rightHandPresent) {
      return 0;  // Hard silence like prototype
    }
    
    // Update oscillator phase increments using Line objects (like prototype)
    aSaw1.setPhaseInc(aGliss1.next());
    aSaw2.setPhaseInc(aGliss2.next());
    
    // Mix the two oscillators using EXACT same amplification as prototype
    int sample1 = aSaw1.next();
    int sample2 = aSaw2.next();
    
    // CRITICAL: Use the EXACT same 24x amplification pattern as working prototype
    // This pattern: 24*((int)signal1+signal2+(signal3>>1)+(signal4>>2))
    // For 2-oscillator version: 24*((int)aSaw1+aSaw2)
    int mixed = 24 * ((int)sample1 + sample2);
    
    // Return direct signal without internal volume control
    // Let the main system apply MonoOutput::fromAlmostNBit() like prototype
    return mixed;
  }

  int level() {
    return smoothedLevel;
  }

}