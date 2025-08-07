#include "alien.h"
#include <Arduino.h>
#include "NewTone.h"
#include <math.h>

static const int AUDIO_PIN = 12;

// State
static int baseFrequency = 440; // A4
static float vibratoDepth = 0.03f; // ±3%
static float vibratoRate = 5.5f;   // Hz
static float lfoPhase = 0.0f;      // 0..1
static unsigned long lastMs = 0;

// Map bounds
static const int MIN_CM = 5;
static const int MAX_CM = 50;
static const int MIN_FREQ = 200;
static const int MAX_FREQ = 1200;

void alien_setup() {
  pinMode(AUDIO_PIN, OUTPUT);
  noNewTone(AUDIO_PIN);
  baseFrequency = 440;
  lfoPhase = 0.0f;
  lastMs = millis();
}

void alien_disable() {
  noNewTone(AUDIO_PIN);
}

void alien_update(float distanceLeft, float distanceRight) {
  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  // Update vibrato LFO
  lfoPhase += vibratoRate * dt;
  if (lfoPhase >= 1.0f) lfoPhase -= 1.0f;
  float vibrato = sinf(6.2831853f * lfoPhase) * vibratoDepth; // 2*pi

  bool pitchActive = (distanceRight >= MIN_CM && distanceRight <= MAX_CM);
  bool volumeActive = (distanceLeft >= MIN_CM && distanceLeft <= MAX_CM);

  // If no hands, output absolute silence
  if (!pitchActive && !volumeActive) {
    noNewTone(AUDIO_PIN);
    return;
  }

  // Map right hand to pitch
  if (pitchActive) {
    int freq = map((int)distanceRight, MIN_CM, MAX_CM, MAX_FREQ, MIN_FREQ);
    baseFrequency = constrain(freq, MIN_FREQ, MAX_FREQ);
  }

  // Apply vibrato
  int playFreq = (int)(baseFrequency * (1.0f + vibrato));
  playFreq = constrain(playFreq, MIN_FREQ, MAX_FREQ);

  // "Volume" via on/off gating based on left hand distance (simple tremolo)
  if (volumeActive) {
    int tremoloRate = map((int)distanceLeft, MIN_CM, MAX_CM, 12, 3); // faster when close
    bool gate = ((now / (1000UL / (unsigned long)tremoloRate)) % 2) == 0;
    if (gate) {
      NewTone(AUDIO_PIN, playFreq);
    } else {
      noNewTone(AUDIO_PIN);
    }
  } else {
    NewTone(AUDIO_PIN, playFreq);
  }
}
