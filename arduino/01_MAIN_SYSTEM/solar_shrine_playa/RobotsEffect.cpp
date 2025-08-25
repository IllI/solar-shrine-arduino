#include "RobotsEffect.h"

#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>
#include <RollingAverage.h>

namespace RobotsEffect {

  namespace {
    // Main oscillator - pure sine wave for musical warmth
    Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> robotOsc(SIN2048_DATA);
    
    // Gentle vibrato for musical expression
    Oscil <COS2048_NUM_CELLS, CONTROL_RATE> robotVibrato(COS2048_DATA);
    
    // Sensor averaging for stability
    RollingAverage <int, 8> robotPitchAvg;
    RollingAverage <int, 12> robotVolAvg;
    
    // Musical frequency range (based on prototype research)
    const int ROBOT_MIN_FREQ = 120;    // Research-proven musical low end
    const int ROBOT_MAX_FREQ = 1500;   // Research-proven musical high end
    const int ROBOT_BASE_FREQ = 440;   // A4 reference note
    
    // Musical scale - C Major Pentatonic (always sounds good, no wrong notes)
    int robotMusicalScale[] = {
      131, 147, 165, 196, 220,           // C3 pentatonic
      262, 294, 330, 392, 440,           // C4 pentatonic (middle range)
      523, 587, 659, 784, 880,           // C5 pentatonic
      1047, 1175, 1319, 1568             // C6 pentatonic (high end)
    };
    const int ROBOT_SCALE_LENGTH = 19;
    
    // Gentle vibrato settings (research shows <2% for musical warmth)
    float robotVibratoDepth = 0.015;     // 1.5% frequency modulation (subtle)
    float robotVibratoRate = 3.2;        // 3.2 Hz (gentle musical vibrato)
    
    // Note persistence (prevents accidental notes)
    const int ROBOT_NOTE_PERSISTENCE_MS = 75;
    unsigned long robotLastNoteChange = 0;
    int robotCurrentNote = 0;
    int robotTargetNote = 0;
    
    // Audio state
    int robotBaseFreq = ROBOT_BASE_FREQ;
    int robotCurrentVol = 0;
    bool robotActive = false;
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    robotOsc.setFreq(ROBOT_BASE_FREQ);
    robotVibrato.setFreq(robotVibratoRate);
    robotCurrentVol = 0;
    robotBaseFreq = ROBOT_BASE_FREQ;
    robotCurrentNote = 0;
    robotTargetNote = 0;
    robotActive = false;
  }

  void exit() {
    // No specific exit actions needed
  }

  // Find closest note in musical scale
  int findClosestNote(int frequency) {
    int bestNote = 0;
    int smallestDiff = abs(robotMusicalScale[0] - frequency);
    
    for (int i = 1; i < ROBOT_SCALE_LENGTH; i++) {
      int diff = abs(robotMusicalScale[i] - frequency);
      if (diff < smallestDiff) {
        smallestDiff = diff;
        bestNote = i;
      }
    }
    return bestNote;
  }

  // Apply musical note persistence (prevents accidental notes)
  int applyNotePersistence(int newNote) {
    if (newNote != robotTargetNote) {
      robotTargetNote = newNote;
      robotLastNoteChange = millis();
    }
    
    if (millis() - robotLastNoteChange > ROBOT_NOTE_PERSISTENCE_MS) {
      robotCurrentNote = robotTargetNote;
    }
    
    return robotCurrentNote;
  }

  void update(bool leftHand, bool rightHand, float d1, float d2) {
    // === PITCH CONTROL (Right hand) ===
    int rawFreq = ROBOT_BASE_FREQ;
    bool pitchActive = false;
    
    if (rightHand) {
      // Map sensor reading to musical frequency range
      int distance = (int)d2;
      distance = constrain(distance, 1, 20);
      
      // Map to musical frequency range (research-based 120-1500Hz)
      rawFreq = map(distance, 1, 20, ROBOT_MAX_FREQ, ROBOT_MIN_FREQ);
      pitchActive = true;
    }
    
    // Apply musical scale quantization with note persistence
    int noteIndex = findClosestNote(rawFreq);
    int persistentNote = applyNotePersistence(noteIndex);
    robotBaseFreq = robotMusicalScale[persistentNote];
    
    // Smooth the frequency changes
    robotBaseFreq = robotPitchAvg.next(robotBaseFreq);
    
    // === VOLUME CONTROL (Left hand) ===
    int rawVolume = 0;
    bool volumeActive = false;
    
    if (leftHand) {
      int distance = (int)d1;
      distance = constrain(distance, 1, 20);
      
      // Map to volume with exponential curve for natural response
      float linearVol = map(distance, 1, 20, 255, 0) / 255.0;
      float expVol = linearVol * linearVol * linearVol;  // Cubic for gentle response
      rawVolume = (int)(expVol * 255);
      volumeActive = true;
    }
    
    // Apply volume smoothing and fade
    if (volumeActive || pitchActive) {
      robotActive = true;
      robotCurrentVol = robotVolAvg.next(rawVolume);
    } else {
      robotActive = false;
      robotCurrentVol = robotVolAvg.next(0);  // Smooth fade out
    }
    
    // === APPLY GENTLE MUSICAL VIBRATO ===
    float vibratoMod = robotVibrato.next() * robotVibratoDepth;
    int modulatedFreq = robotBaseFreq + (int)(robotBaseFreq * vibratoMod);
    
    // Set the main oscillator frequency
    robotOsc.setFreq(modulatedFreq);
  }

  int audio() {
    // Scale oscillator by current smoothed volume (0..255)
    // This restores dynamic loudness based on left-hand control
    return (robotOsc.next() * robotCurrentVol);
  }

  int level() {
    // Smoothed current volume 0..255 used for LED visual height
    return robotCurrentVol;
  }

}
