#include "robots.h"
#include <Arduino.h>
#include "NewTone.h"
#include <math.h>

static const int AUDIO_PIN = 12;

// Simple "talking robot" via two detuned oscillators with AM for formant-like texture
static int baseFrequency = 300;
static float formantPhase = 0.0f;
static float formantRate = 1.5f;  // Hz
static unsigned long lastMs = 0;

// Map helpers
static const int MIN_CM = 5;
static const int MAX_CM = 50;
static const int MIN_FREQ = 120;
static const int MAX_FREQ = 900;

void robots_setup() {
  pinMode(AUDIO_PIN, OUTPUT);
  noNewTone(AUDIO_PIN);
  baseFrequency = 300;
  formantPhase = 0.0f;
  lastMs = millis();
}

void robots_disable() {
  noNewTone(AUDIO_PIN);
}

void robots_update(float distanceLeft, float distanceRight) {
  unsigned long now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;

  bool pitchActive = (distanceRight >= MIN_CM && distanceRight <= MAX_CM);
  bool voiceActive = (distanceLeft >= MIN_CM && distanceLeft <= MAX_CM);

  // If no hands, output absolute silence
  if (!pitchActive && !voiceActive) {
    noNewTone(AUDIO_PIN);
    return;
  }

  if (pitchActive) {
    int freq = map((int)distanceRight, MIN_CM, MAX_CM, MAX_FREQ, MIN_FREQ);
    baseFrequency = constrain(freq, MIN_FREQ, MAX_FREQ);
  }

  // Formant-ish AM rate from left hand
  float rateHz = map((int)distanceLeft, MIN_CM, MAX_CM, 6, 1);
  formantRate = constrain(rateHz, 0.5f, 10.0f);

  formantPhase += formantRate * dt;
  if (formantPhase >= 1.0f) formantPhase -= 1.0f;
  float am = 0.5f * (1.0f + sinf(6.2831853f * formantPhase)); // 0..1

  // Detune second oscillator by small ratio and modulate via AM
  int detune = (int)(baseFrequency * 0.06f); // 6% detune
  int voicedFreq = baseFrequency + (int)(detune * am);
  NewTone(AUDIO_PIN, voicedFreq);
}
