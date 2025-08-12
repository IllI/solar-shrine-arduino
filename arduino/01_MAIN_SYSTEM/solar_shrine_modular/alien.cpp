// Mozzi-based alien effect using PWM mode on pin 12 (via modified local library)
#include "alien.h"
#include <Arduino.h>

// Configure Mozzi for single-pin PWM on pin 12 (avoiding pin 11 sensor conflict)
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_PWM
#define MOZZI_AUDIO_PIN_1 12
#define MOZZI_AUDIO_PIN_1_REGISTER OCR1B
// Push PWM carrier to 62.5kHz to reduce audible whine
// PWM rate must be equal to or 2x MOZZI_AUDIO_RATE on AVR
#undef MOZZI_PWM_RATE
#define MOZZI_PWM_RATE 32768
#define CONTROL_RATE 128
#warning "[Solar Shrine] Using modified Mozzi library with pin 12 default"
#include <MozziGuts.h>
#include <Oscil.h>
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>

// Audio pin automatically handled by Mozzi PWM mode (configured for pin 12)

// Enable quick audio path diagnostic (set to 1 to force a constant test tone)
#define ALIEN_TEST_TONE 1

// Oscillators
static Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> osc(SIN2048_DATA);
static Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> oscHarm(SIN2048_DATA);
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

void alien_setup() {
  // Initialize Mozzi once and leave it running to avoid PWM instability
  startMozzi(CONTROL_RATE);
  
  // Initialize oscillators
  osc.setFreq(220);
  oscHarm.setFreq(440);
  vibrato.setFreq(vibratoRate);
  for (int i = 0; i < ECHO_BUFFER_SIZE; ++i) echoBuffer[i] = 0;
  
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
    return;
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

// Mozzi audio callback
int updateAudio() {
#if ALIEN_TEST_TONE
  // Force a steady tone to validate PWM pin 12 audio path
  smoothVol = 220;
  osc.setFreq(440);
  int s = (osc.next() * smoothVol) >> 8;
  if (s > 127) s = 127; if (s < -128) s = -128;
  return s;
#endif
  // If muted, output silence
  if (smoothVol <= 0) {
    return 0;
  }

  int baseSample = (osc.next() * smoothVol) >> 8;
  int harmSample = (oscHarm.next() * smoothVol) >> 8;
  int combined = baseSample + ((harmSample * harmMix) >> 8);

  // Low-pass tone (restore smoothing to reduce buzz)
  lpfState = lpfState + (((combined - lpfState) * lpfAlpha) >> 8);
  int filtered = lpfState;

  // Echo - keep disabled for stability
  // int echoPos = (echoIndex - echoDelay + ECHO_BUFFER_SIZE) % ECHO_BUFFER_SIZE;
  // int echoSample = echoBuffer[echoPos];
  // int mixed = filtered + (int)(echoSample * echoMix);
  // echoBuffer[echoIndex] = filtered;
  // echoIndex++; if (echoIndex >= ECHO_BUFFER_SIZE) echoIndex = 0;
  int mixed = filtered; // No echo for now
  if (mixed > 127) mixed = 127; if (mixed < -128) mixed = -128;
  // Return plain int like the working alien_sound_effect.ino
  return mixed;
}

// Mozzi control-rate callback (required). We drive parameters from alien_update().
void updateControl() {
  // no-op; all control updates are done in alien_update via main loop distances
}

// audioOutput() function not needed - Mozzi's PWM mode handles output automatically