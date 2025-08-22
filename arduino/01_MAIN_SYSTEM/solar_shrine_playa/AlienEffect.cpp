#include "AlienEffect.h"

#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>
#include <RollingAverage.h>

namespace AlienEffect {

  namespace {
    // Alien effect variables
    Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> alienOsc(SIN2048_DATA);
    Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> alienOscHarm(SIN2048_DATA);
    Oscil <COS2048_NUM_CELLS, CONTROL_RATE> alienVibrato(COS2048_DATA);
    RollingAverage <int, 4> alienPitchAvg;
    RollingAverage <int, 8> alienVolAvg;
    
    // Motion reactivity (from prototype)
    RollingAverage <int, 4> alienPitchVelAvg;
    RollingAverage <int, 4> alienVolVelAvg;
    int alienPrevPitchDur = 0;
    int alienPrevVolDur = 0;
    int alienBaseFreq = 440;
    int alienSmoothVol = 0;
    int alienVol = 0;
    float alienVibratoDepth = 0.03;
    float alienVibratoRate = 5.5;
    int alienHarmMix = 0;
    bool alienMuteOutput = false;
    #define ALIEN_ECHO_BUFFER_SIZE 256
    int alienEchoBuffer[ALIEN_ECHO_BUFFER_SIZE];
    int alienEchoIndex = 0;
    float alienEchoMix = 0.25;
    int alienEchoDelay = 128;
    int alienLpfState = 0;
    int alienLpfAlpha = 180;
    
    // Frequency range (from prototype)
    const int alienLowestFreq = 131;  // C3
    const int alienHighestFreq = 1046; // C6
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    alienOsc.setFreq(440);
    alienOscHarm.setFreq(880);
    alienVibrato.setFreq(alienVibratoRate);
    alienVol = 0;
    alienSmoothVol = 0;
    alienBaseFreq = 440;
    for (int i = 0; i < ALIEN_ECHO_BUFFER_SIZE; i++) {
      alienEchoBuffer[i] = 0;
    }
  }

  void exit() {
    // No specific exit actions needed for this effect
  }

  void update(bool leftHand, bool rightHand, float d1, float d2) {
    // Convert distance to duration-like values for motion tracking (from prototype)
    int pitchDur = (int)(d2 * 10); // Right hand distance
    int volDur = (int)(d1 * 10);   // Left hand distance
    
    // === PITCH CONTROL (Right hand) ===
    int freq;
    int jitter;

    if (rightHand) {
      freq = map(pitchDur, 10, 200, alienHighestFreq, alienLowestFreq);
      freq = constrain(freq, alienLowestFreq, alienHighestFreq);
    } else {
      freq = alienLowestFreq;
    }

    jitter = random(-5, 5);
    int averaged = alienPitchAvg.next(freq);
    alienBaseFreq = averaged + jitter;

    // === VOLUME CONTROL (Left hand) - KEEP EXISTING LOGIC ===
    if (leftHand) {
      alienVol = 255; // Full volume when hand is present
    } else {
      alienVol = 200; // High volume even without hand (was 0, now 200)
    }

    // Always allow some output, even without hands
    alienMuteOutput = false;

    alienSmoothVol = alienVolAvg.next(alienVol);

    // === MOTION REACTIVITY (from prototype) ===
    int pitchVel = abs(pitchDur - alienPrevPitchDur);
    int volVel = abs(volDur - alienPrevVolDur);
    int pitchVelSm = alienPitchVelAvg.next(pitchVel);
    int volVelSm = alienVolVelAvg.next(volVel);
    alienPrevPitchDur = pitchDur;
    alienPrevVolDur = volDur;

    float pitchVelNorm = min(1.0f, pitchVelSm / 400.0f);
    float volVelNorm = min(1.0f, volVelSm / 400.0f);

    // === DYNAMIC VIBRATO (from prototype) ===
    // Closer left hand -> deeper vibrato; quick movements -> even deeper/faster
    float normalizedVol = alienSmoothVol / 255.0f;
    float baselineDepth = 0.01f + (normalizedVol * 0.07f);
    float motionDepth = pitchVelNorm * 0.03f;
    alienVibratoDepth = baselineDepth + motionDepth;
    if (alienVibratoDepth > 0.15f) alienVibratoDepth = 0.15f;

    alienVibratoRate = 4.0f + (volVelNorm * 6.0f); // 4-10 Hz depending on movement
    alienVibrato.setFreq(alienVibratoRate);

    // Apply vibrato
    float vibratoAmount = alienVibrato.next() * alienVibratoDepth;
    int modulatedFreq = alienBaseFreq + (int)(alienBaseFreq * vibratoAmount);
    if (modulatedFreq < alienLowestFreq) modulatedFreq = alienLowestFreq;
    if (modulatedFreq > alienHighestFreq) modulatedFreq = alienHighestFreq;
    alienOsc.setFreq(modulatedFreq);

    // === REACTIVE HARMONICS (from prototype) ===
    // More harmonic content on faster right-hand movement
    alienHarmMix = (int)(pitchVelNorm * 160.0f); // 0-160
    int harmFreq = modulatedFreq * 2;
    if (harmFreq > 3000) harmFreq = 3000;
    alienOscHarm.setFreq(harmFreq);

    // === GESTURE-CONTROLLED ECHO (from prototype) ===
    // Closer left hand -> more echo; farther right hand -> longer delay
    alienEchoMix = 0.10f + (normalizedVol * 0.35f); // 0.10-0.45
    int minDelay = 80;
    int maxDelay = ALIEN_ECHO_BUFFER_SIZE - 1;
    int pd = (int)constrain(pitchDur, 10, 200);
    alienEchoDelay = map(pd, 10, 200, minDelay, maxDelay);
    
    // === TONE BRIGHTNESS CONTROL (from prototype) ===
    // Right hand distance controls low-pass filter (closer = brighter)
    int minAlpha = 40;   // darker
    int maxAlpha = 230;  // brighter
    alienLpfAlpha = map(pd, 10, 200, maxAlpha, minAlpha);
  }

  int audio() {
    // Use the same approach as the working robots effect
    // Return maximum volume using the oscillator
    return (alienOsc.next() * 255); // Maximum volume, exactly like robots effect
  }
}