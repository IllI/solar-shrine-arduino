// Mozzi-based alien effect using PWM mode on pin 12 (via modified local library)
#include "alien.h"
#include <Arduino.h>

// Mozzi configuration is now handled at the sketch level in solar_shrine_modular.ino
// This ensures proper pin 12 configuration before any Mozzi includes
// Only include Mozzi classes, not MozziGuts.h (to avoid multiple definitions)
#include <Oscil.h>
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>

// Audio pin automatically handled by Mozzi PWM mode (configured for pin 12)

// Enable quick audio path diagnostic (set to 1 to force a constant test tone)
#define ALIEN_TEST_TONE 0

// Oscillators - use MOZZI_AUDIO_RATE instead of AUDIO_RATE for modular system
static Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc(SIN2048_DATA);
static Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscHarm(SIN2048_DATA);
static Oscil<COS2048_NUM_CELLS, CONTROL_RATE> vibrato(COS2048_DATA);

// Smoothing
static RollingAverage<int, 4> pAverage;  // pitch
static RollingAverage<int, 8> vAverage;  // volume

// Distances and velocities
static float prevLeftCm = -1.0f;
static float prevRightCm = -1.0f;

// Vibrato and params
static float vibratoDepth = 0.03f;
static float vibratoRate = 5.5f;
static int baseFreq = 440;

// Gesture params
static int harmMix = 0; // 0..255

// Volume
static int smoothVol = 0; // 0..255

// Echo
#define ECHO_BUFFER_SIZE 256
static int echoBuffer[ECHO_BUFFER_SIZE];
static int echoIndex = 0;
static float echoMix = 0.25f;
static int echoDelay = 128;

// Tone control (simple LPF)
static int lpfState = 0;
static int lpfAlpha = 180; // 0..255

// Presence tracking
static float lastLeft = 999.0f, lastRight = 999.0f;

static inline bool inRange(float cm) { return cm >= 5.0f && cm <= 50.0f; }

// Use shield pins (same as working alien_sound_effect.ino)
static const int LEFT_TRIG_PIN = 10;  // left (volume) trigger
static const int LEFT_ECHO_PIN = 11;  // left (volume) echo (must not be driven by Mozzi)
static const int RIGHT_TRIG_PIN = 5;  // right (pitch) trigger
static const int RIGHT_ECHO_PIN = 6;  // right (pitch) echo

void alien_setup() {
  // Mozzi is started at the sketch level, just initialize oscillators
  osc.setFreq(220);
  oscHarm.setFreq(440);
  vibrato.setFreq(vibratoRate);
  for (int i = 0; i < ECHO_BUFFER_SIZE; ++i) echoBuffer[i] = 0;
  
  // Ensure sensor pins are configured (redundant to main setup but harmless)
  pinMode(LEFT_TRIG_PIN, OUTPUT); pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT); pinMode(RIGHT_ECHO_PIN, INPUT);
  
  // No manual timer setup needed - Mozzi's PWM mode handles it automatically
}

void alien_disable() {
  // Don't stop Mozzi - just silence the output by setting volume to 0
  smoothVol = 0;
  // Mozzi remains running to avoid PWM instability
}

void alien_update(float distanceLeft, float distanceRight) {
  lastLeft = distanceLeft; lastRight = distanceRight;
  const int lowestFreq = 131;
  const int highestFreq = 1046;

  // Treat 0 as "very close" (some reads return 0 when hand is very near)
  bool rightPresent = (distanceRight == 0.0f) ? true : inRange(distanceRight);
  bool leftPresent  = (distanceLeft  == 0.0f) ? true : inRange(distanceLeft);

  // True silence: set volume to zero when no hands present
  if (!rightPresent && !leftPresent) {
    smoothVol = 0;
    echoMix = 0.0f;
    // Ensure the physical pin is muted when idle for true silence
    pinMode(12, INPUT);
    return;
  } else {
    // Hands present: ensure PWM output is enabled again
    pinMode(12, OUTPUT);
  }

  // Volume mapping (left hand primary, right-hand fallback)
  int linearVol = 0; // 0..255
  if (leftPresent) {
    int dl = (int)constrain(distanceLeft == 0.0f ? 5.0f : distanceLeft, 5.0f, 50.0f);
    // closer = louder; keep some headroom to reduce clipping
    linearVol = map(dl, 50, 5, 60, 255);
  } else if (rightPresent) {
    int dr = (int)constrain(distanceRight == 0.0f ? 5.0f : distanceRight, 5.0f, 50.0f);
    // modest fallback so right hand alone still produces sound
    linearVol = map(dr, 50, 5, 40, 160);
  }
  // Smooth volume
  int volSmoothed = vAverage.next(linearVol);
  smoothVol = (smoothVol * 3 + volSmoothed) >> 2;
  if (smoothVol < 0) smoothVol = 0; if (smoothVol > 255) smoothVol = 255;
  float normalizedVol = smoothVol / 255.0f;

  // Pitch mapping (right hand)
  if (rightPresent) {
    int freq = map((int)constrain(distanceRight == 0.0f ? 5.0f : distanceRight, 5.0f, 50.0f), 5, 50, highestFreq, lowestFreq);
    baseFreq = pAverage.next(freq);
  }

  // (Removed duplicate volume mapping block to avoid redeclarations)

  // Motion
  float rightVel = (prevRightCm >= 0) ? fabsf(distanceRight - prevRightCm) : 0.0f;
  float leftVel  = (prevLeftCm  >= 0) ? fabsf(distanceLeft  - prevLeftCm)  : 0.0f;
  prevRightCm = distanceRight; prevLeftCm = distanceLeft;
  float pitchVelNorm = rightVel / 8.0f; if (pitchVelNorm > 1.0f) pitchVelNorm = 1.0f;
  float volVelNorm   = leftVel  / 8.0f; if (volVelNorm   > 1.0f) volVelNorm   = 1.0f;

  // Dynamic vibrato
  float baselineDepth = 0.01f + (normalizedVol * 0.07f);
  float motionDepth   = pitchVelNorm * 0.03f;
  vibratoDepth = baselineDepth + motionDepth; if (vibratoDepth > 0.15f) vibratoDepth = 0.15f;
  vibratoRate = 4.0f + (volVelNorm * 6.0f); if (vibratoRate > 10.0f) vibratoRate = 10.0f;
  vibrato.setFreq(vibratoRate);

  // Apply vibrato to baseFreq
  float vibAmount = vibrato.next() * vibratoDepth; // COS table returns -1..1 (int8 scaled)
  int modFreq = baseFreq + (int)(baseFreq * vibAmount);
  if (modFreq < lowestFreq) modFreq = lowestFreq; if (modFreq > highestFreq) modFreq = highestFreq;
  osc.setFreq(modFreq);

  // Harmonics
  harmMix = (int)(pitchVelNorm * 160.0f);
  int harmFreq = modFreq * 2; if (harmFreq > 3000) harmFreq = 3000;
  oscHarm.setFreq(harmFreq);

  // Echo and tone control depend on right-hand distance (clamp raw 0 to 5cm)
  int pd = rightPresent
              ? map((int)constrain(distanceRight == 0.0f ? 5.0f : distanceRight, 5.0f, 50.0f), 5, 50, 30, 800)
              : 800; // pseudo distance scale
  echoMix = 0.10f + (normalizedVol * 0.35f);
  int minDelay = 80, maxDelay = ECHO_BUFFER_SIZE - 1;
  echoDelay = map(pd, 30, 800, minDelay, maxDelay);
  // Tone filter alpha maps with distance (closer=brighter)
  lpfAlpha = map(pd, 30, 800, 230, 40);
  lpfAlpha = constrain(lpfAlpha, 40, 230);
}

void alien_audio_hook() {
  audioHook();
}

void alien_get_distances(float &outLeftCm, float &outRightCm) {
  outLeftCm = lastLeft; outRightCm = lastRight;
}

void alien_sense(float &outLeftCm, float &outRightCm) {
  outLeftCm = lastLeft; outRightCm = lastRight;
}

bool alien_is_test_tone_enabled() {
#if ALIEN_TEST_TONE
  return true;
#else
  return false;
#endif
}

// Internal audio callback for alien effect
int alien_internal_updateAudio() {
#if ALIEN_TEST_TONE
  // Simple test tone matching working alien_sound_effect.ino pattern
  osc.setFreq(440);
  smoothVol = 200;  // Fixed volume for test
  int testSample = (osc.next() * smoothVol) >> 8;
  
  // Apply same clipping as working sketch
  if (testSample > 127) testSample = 127;
  if (testSample < -128) testSample = -128;
  
  return testSample;
#endif
  // If muted, output silence
  if (smoothVol <= 0) {
    return 0;
  }

  // Safety check: ensure smoothVol is in valid range
  if (smoothVol > 255) smoothVol = 255;
  
  // Simple, clean Mozzi synthesis like the working sketches
  
  // Generate pure sine wave sample
  int sample = (osc.next() * smoothVol) >> 8;
  
  // Return clean sample without complex processing
  return sample;
}

// Internal control-rate callback for alien effect
void alien_internal_updateControl() {
  // Use the distances already calculated by the main loop
  float leftCm = lastLeft;
  float rightCm = lastRight;
  
  // Detect presence (5-50cm range)
  bool rightPresent = (rightCm >= 5.0f && rightCm <= 50.0f);
  bool leftPresent = (leftCm >= 5.0f && leftCm <= 50.0f);

  if (!rightPresent && !leftPresent) {
    smoothVol = 0;
    return;
  }

  // PITCH CONTROL (Right hand) - simple mapping like working sketches
  int freq = 440; // default
  if (rightPresent) {
    // Map 5-50cm to frequency range (closer = higher pitch)
    freq = map((int)rightCm, 5, 50, 1046, 131); // C6 to C3 range
  }
  osc.setFreq(freq);

  // VOLUME CONTROL (Left hand) - simple mapping like working sketches
  int vol = 0;
  if (leftPresent) {
    // Map 5-50cm to volume (closer = louder)
    vol = map((int)leftCm, 5, 50, 255, 60);
  } else if (rightPresent) {
    // Right hand solo - quieter fallback
    vol = map((int)rightCm, 5, 50, 160, 40);
  }
  
  smoothVol = vol;
}
