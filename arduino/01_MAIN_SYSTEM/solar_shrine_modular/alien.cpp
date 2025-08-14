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
  
  // Generate basic sine wave like the working alien_sound_effect.ino
  int baseSample = (osc.next() * smoothVol) >> 8;
  
  // Safety clipping to prevent overflow
  if (baseSample > 127) baseSample = 127;
  if (baseSample < -128) baseSample = -128;
  
  // For now, return just the basic sample to avoid complex synthesis issues
  return baseSample;
}

// Internal control-rate callback for alien effect
void alien_internal_updateControl() {
  // Simple test: set a fixed frequency and volume for now
  osc.setFreq(440);  // A4 note
  smoothVol = 200;   // Fixed moderate volume
  
  // For now, skip complex synthesis to get basic sound working
  return;
  
  // Use the distances already calculated by the main loop
  // Convert from cm to microsecond-like values to match original alien_sound_effect.ino
  float leftCm = lastLeft;
  float rightCm = lastRight;
  
  // Detect presence (main loop already handles this, but we need local logic)
  bool rightPresent = (rightCm >= 5.0f && rightCm <= 50.0f);
  bool leftPresent = (leftCm >= 5.0f && leftCm <= 50.0f);

  if (!rightPresent && !leftPresent) {
    smoothVol = 0;
    echoMix = 0.0f;
    return;
  }

  // Convert distances to microsecond-like values for synthesis compatibility
  unsigned long pitchDur = (unsigned long)(rightCm * 6.0f);  // approximate conversion
  unsigned long volDur = (unsigned long)(leftCm * 6.0f);

  // Update synthesis parameters based on sensor readings

  // Map pitch distance to frequency (similar to working sketch):
  const int lowestFreq = 131;
  const int highestFreq = 1046;
  long pitchDistance = rightPresent ? (long)(pitchDur / 6) : 800; // approx cm-like scale
  if (rightPresent) {
    if (pitchDistance < 30) pitchDistance = 30; if (pitchDistance > 800) pitchDistance = 800;
    int freq = map((int)pitchDistance, 30, 800, highestFreq, lowestFreq);
    baseFreq = pAverage.next(freq);
  }

  // Volume mapping (left hand primary, right-hand fallback)
  int linearVol = 0; // 0..255
  if (leftPresent) {
    long volDistance = (long)(volDur / 6);
    if (volDistance < 30) volDistance = 30; if (volDistance > 800) volDistance = 800;
    int v = map((int)volDistance, 30, 800, 255, 0);
    // logarithmic feel
    float n = v / 255.0f; v = (int)(n * n * 255.0f);
    linearVol = v;
  } else if (rightPresent) {
    int pd = (int)constrain(pitchDistance, 30, 800);
    linearVol = map(pd, 30, 800, 160, 40); // modest solo floor
  }
  int volSm = vAverage.next(linearVol);
  smoothVol = (smoothVol * 3 + volSm) >> 2;
  if (smoothVol < 0) smoothVol = 0; if (smoothVol > 255) smoothVol = 255;

  // Motion-derived vibrato and harmonics
  static long prevPd = -1, prevVd = -1;
  long pdNow = pitchDistance;
  long vdNow = leftPresent ? (long)(volDur / 6) : 800;
  float pitchVelNorm = (prevPd >= 0) ? min(1.0f, fabsf(pdNow - prevPd) / 80.0f) : 0.0f;
  float volVelNorm   = (prevVd >= 0) ? min(1.0f, fabsf(vdNow - prevVd) / 80.0f)   : 0.0f;
  prevPd = pdNow; prevVd = vdNow;

  float normalizedVol = smoothVol / 255.0f;
  float baselineDepth = 0.01f + (normalizedVol * 0.07f);
  float motionDepth   = pitchVelNorm * 0.03f;
  vibratoDepth = min(0.15f, baselineDepth + motionDepth);
  vibratoRate  = 4.0f + (volVelNorm * 6.0f); if (vibratoRate > 10.0f) vibratoRate = 10.0f;
  vibrato.setFreq(vibratoRate);

  // Apply vibrato
  float vibAmount = vibrato.next() * vibratoDepth;
  int modFreq = baseFreq + (int)(baseFreq * vibAmount);
  if (modFreq < lowestFreq) modFreq = lowestFreq; if (modFreq > highestFreq) modFreq = highestFreq;
  osc.setFreq(modFreq);

  // Harmonics and tone
  harmMix = (int)(pitchVelNorm * 160.0f);
  int harmFreq = modFreq * 2; if (harmFreq > 3000) harmFreq = 3000; oscHarm.setFreq(harmFreq);

  // Echo/tone params derived from pitchDistance
  int pd = (int)constrain(pitchDistance, 30, 800);
  echoMix = 0.10f + (normalizedVol * 0.35f);
  echoDelay = map(pd, 30, 800, 80, ECHO_BUFFER_SIZE - 1);
  lpfAlpha = map(pd, 30, 800, 230, 40); lpfAlpha = constrain(lpfAlpha, 40, 230);

  // Expose last distances as cm-like values for debug
  lastRight = (float)pdNow; lastLeft = (float)vdNow;

  // Ensure audio pin stays configured for Mozzi PWM while ALIEN is active
}

// audioOutput() function not needed - Mozzi's PWM mode handles output automatically