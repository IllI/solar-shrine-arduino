// Port of interactive logic from alien_sound_effect.ino adapted for NewTone
#include "alien.h"
#include <Arduino.h>
#include "NewTone.h"
#include <math.h>

static const int AUDIO_PIN = 12;

// Range and pitch mapping
static const float MIN_CM = 5.0f;
static const float MAX_CM = 50.0f;
static const int MIN_FREQ = 200;
static const int MAX_FREQ = 1200;

// Vibrato state
static float vibratoDepth = 0.03f; // dynamic
static float vibratoRate = 5.5f;   // dynamic
static float lfoPhase = 0.0f;      // 0..1
static int baseFrequency = 440;

// Volume gating via duty cycle (0..255)
static const unsigned long VOLUME_GATE_PERIOD_MS = 8; // ~125 Hz gating
static int targetDuty = 0; // 0..255

// Movement tracking
static float prevLeftCm = -1.0f;
static float prevRightCm = -1.0f;
static unsigned long lastMs = 0;

// Right-hand solo fallback
static const bool ALLOW_RIGHT_HAND_SOLO = true;

static inline bool inRange(float cm) {
  return cm >= MIN_CM && cm <= MAX_CM;
}

void alien_setup() {
  pinMode(AUDIO_PIN, OUTPUT);
  noNewTone(AUDIO_PIN);
  baseFrequency = 440;
  lfoPhase = 0.0f;
  lastMs = millis();
  prevLeftCm = -1.0f;
  prevRightCm = -1.0f;
  targetDuty = 0;
}

void alien_disable() {
  noNewTone(AUDIO_PIN);
}

void alien_update(float distanceLeft, float distanceRight) {
  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0f;
  if (dt < 0.0005f) dt = 0.0005f; // guard
  lastMs = now;

  // Presence detection
  bool rightPresent = inRange(distanceRight);
  bool leftPresent  = inRange(distanceLeft);

  // If neither hand present, mute
  if (!rightPresent && !leftPresent) {
    targetDuty = 0;
    noNewTone(AUDIO_PIN);
    return;
  }

  // Map right hand to pitch when present
  if (rightPresent) {
    int freq = map((int)distanceRight, (int)MIN_CM, (int)MAX_CM, MAX_FREQ, MIN_FREQ);
    baseFrequency = constrain(freq, MIN_FREQ, MAX_FREQ);
  }

  // Compute movement speeds (cm per update) and normalize
  float rightVel = 0.0f;
  float leftVel = 0.0f;
  if (prevRightCm >= 0.0f) rightVel = fabsf(distanceRight - prevRightCm);
  if (prevLeftCm  >= 0.0f) leftVel  = fabsf(distanceLeft  - prevLeftCm);
  prevRightCm = distanceRight;
  prevLeftCm  = distanceLeft;

  // Normalize velocities (heuristic scaling)
  float pitchVelNorm = rightVel / 8.0f; if (pitchVelNorm > 1.0f) pitchVelNorm = 1.0f;
  float volVelNorm   = leftVel  / 8.0f; if (volVelNorm   > 1.0f) volVelNorm   = 1.0f;

  // Left distance -> volume baseline [0..1], closer = louder
  float volNormLinear = 0.0f;
  if (leftPresent) {
    float t = (distanceLeft - MIN_CM) / (MAX_CM - MIN_CM); // 0 close .. 1 far
    t = constrain(t, 0.0f, 1.0f);
    volNormLinear = 1.0f - t; // close->1.0, far->0.0
  } else if (ALLOW_RIGHT_HAND_SOLO && rightPresent) {
    // Quiet floor based on right-hand distance
    float t = (distanceRight - MIN_CM) / (MAX_CM - MIN_CM);
    t = constrain(t, 0.0f, 1.0f);
    volNormLinear = 0.15f + (1.0f - t) * 0.25f; // ~0.15..0.40
  }

  // Log volume curve
  float volNorm = volNormLinear * volNormLinear;
  targetDuty = (int)(volNorm * 255.0f);
  if (targetDuty < 0) targetDuty = 0; if (targetDuty > 255) targetDuty = 255;

  // Dynamic vibrato from left distance (baseline) and right-hand movement (motion)
  float baselineDepth = 0.01f + (volNormLinear * 0.07f);
  float motionDepth   = pitchVelNorm * 0.03f;
  vibratoDepth = baselineDepth + motionDepth; if (vibratoDepth > 0.15f) vibratoDepth = 0.15f;
  vibratoRate = 4.0f + (volVelNorm * 6.0f);

  // Update vibrato LFO
  lfoPhase += vibratoRate * dt;
  if (lfoPhase >= 1.0f) lfoPhase -= 1.0f;
  float vibrato = sinf(6.2831853f * lfoPhase) * vibratoDepth;

  // Apply vibrato to frequency
  int playFreq = (int)(baseFrequency * (1.0f + vibrato));
  if (playFreq < MIN_FREQ) playFreq = MIN_FREQ; if (playFreq > MAX_FREQ) playFreq = MAX_FREQ;

  // Duty-cycle based volume gate
  if (targetDuty <= 0) {
    noNewTone(AUDIO_PIN);
    return;
  }

  unsigned long phase = now % VOLUME_GATE_PERIOD_MS;
  unsigned long onWindow = (unsigned long)((targetDuty / 255.0f) * VOLUME_GATE_PERIOD_MS);
  if (phase < onWindow) {
    NewTone(AUDIO_PIN, playFreq);
  } else {
    noNewTone(AUDIO_PIN);
  }
}
