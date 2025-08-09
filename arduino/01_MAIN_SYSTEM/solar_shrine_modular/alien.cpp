// NewTone-based alien effect for the modular system
// Provides interactive alien sounds using NewTone library on pin 12
#include "alien.h"
#include <Arduino.h>
#include <NewTone.h>

#define AUDIO_PIN 12
#define UPDATE_RATE 50  // Update frequency control parameters 50 times per second

// NewTone-based alien effect state
static float lastLeft = 999.0f, lastRight = 999.0f;
static bool isActive = false;
static unsigned long lastUpdateTime = 0;

// Alien effect parameters
static int baseFreq = 440;          // Base frequency in Hz
static float vibratoDepth = 0.03f;  // Vibrato depth (0.0 - 0.1)
static float vibratoRate = 5.5f;    // Vibrato rate in Hz
static float vibratoPhase = 0.0f;   // Current vibrato phase

// Motion tracking for dynamic effects
static float prevLeftCm = -1.0f;
static float prevRightCm = -1.0f;

// Smoothing for frequency and volume
static float smoothFreq = 440.0f;
static float smoothVol = 0.0f;

static inline bool inRange(float cm) { return cm >= 5.0f && cm <= 50.0f; }

void alien_setup() {
  // Initialize alien effect - NewTone handles all audio setup automatically
  isActive = false;
  lastUpdateTime = millis();
  baseFreq = 440;
  vibratoPhase = 0.0f;
  smoothFreq = 440.0f;
  smoothVol = 0.0f;
}

void alien_disable() {
  // Stop NewTone audio output
  noNewTone(AUDIO_PIN);
  isActive = false;
}

void alien_update(float distanceLeft, float distanceRight) {
  lastLeft = distanceLeft; 
  lastRight = distanceRight;
  
  unsigned long currentTime = millis();
  
  // Frequency ranges
  const int lowestFreq = 131;
  const int highestFreq = 1046;

  bool rightPresent = inRange(distanceRight);
  bool leftPresent  = inRange(distanceLeft);

  // If neither hand present -> complete silence
  if (!rightPresent && !leftPresent) {
    if (isActive) {
      noNewTone(AUDIO_PIN);
      isActive = false;
    }
    smoothVol = 0.0f;
    return;
  }

  // Right hand pitch mapping (closer = higher frequency)
  if (rightPresent) {
    int targetFreq = map((int)constrain(distanceRight, 5, 50), 5, 50, highestFreq, lowestFreq);
    // Smooth frequency changes
    smoothFreq = smoothFreq * 0.9f + targetFreq * 0.1f;
    baseFreq = (int)smoothFreq;
  }

  // Left hand volume mapping (closer = louder), with right-hand solo fallback
  float targetVol = 0.0f;
  if (leftPresent) {
    float t = (distanceLeft - 5.0f) / 45.0f; 
    if (t < 0) t = 0; if (t > 1) t = 1;
    targetVol = (1.0f - t); // 0.0 to 1.0
  } else if (rightPresent) {
    // Right-hand solo mode - quieter
    float t = (distanceRight - 5.0f) / 45.0f; 
    if (t < 0) t = 0; if (t > 1) t = 1;
    targetVol = 0.15f + (1.0f - t) * 0.25f; // 0.15 to 0.4
  }
  
  // Smooth volume changes
  smoothVol = smoothVol * 0.8f + targetVol * 0.2f;

  // Only update vibrato and play tone at the specified rate
  if (currentTime - lastUpdateTime >= (1000 / UPDATE_RATE)) {
    lastUpdateTime = currentTime;
    
    // Calculate motion for dynamic effects
    float rightVel = (prevRightCm >= 0) ? fabsf(distanceRight - prevRightCm) : 0.0f;
    float leftVel  = (prevLeftCm  >= 0) ? fabsf(distanceLeft  - prevLeftCm)  : 0.0f;
    prevRightCm = distanceRight; 
    prevLeftCm = distanceLeft;
    
    float pitchVelNorm = constrain(rightVel / 8.0f, 0.0f, 1.0f);
    float volVelNorm   = constrain(leftVel  / 8.0f, 0.0f, 1.0f);

    // Dynamic vibrato based on proximity and motion
    float baselineDepth = 0.01f + (smoothVol * 0.07f);
    float motionDepth   = pitchVelNorm * 0.03f;
    vibratoDepth = constrain(baselineDepth + motionDepth, 0.0f, 0.15f);
    vibratoRate = 4.0f + (volVelNorm * 6.0f);
    
    // Update vibrato phase
    vibratoPhase += (vibratoRate * 2.0f * PI) / UPDATE_RATE;
    if (vibratoPhase > 2.0f * PI) vibratoPhase -= 2.0f * PI;
    
    // Apply vibrato to frequency
    float vibratoMod = sin(vibratoPhase) * vibratoDepth;
    int modulatedFreq = (int)(baseFreq * (1.0f + vibratoMod));
    
    // Constrain to reasonable range
    modulatedFreq = constrain(modulatedFreq, 80, 2000);
    
    // Play the tone if volume is sufficient
    if (smoothVol > 0.05f) {
      NewTone(AUDIO_PIN, modulatedFreq);
      isActive = true;
    } else if (isActive) {
      noNewTone(AUDIO_PIN);
      isActive = false;
    }
  }
}

void alien_audio_hook() {
  // NewTone doesn't need audio hooks - it handles everything automatically
  // This function is kept for compatibility with the main loop
}

void alien_get_distances(float &outLeftCm, float &outRightCm) {
  outLeftCm = lastLeft; 
  outRightCm = lastRight;
}

void alien_sense(float &outLeftCm, float &outRightCm) {
  // Same as get_distances for this implementation
  outLeftCm = lastLeft; 
  outRightCm = lastRight;
}