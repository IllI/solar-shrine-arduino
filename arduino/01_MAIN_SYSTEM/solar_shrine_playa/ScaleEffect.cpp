#include "ScaleEffect.h"

#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include <RollingAverage.h>
#include <avr/pgmspace.h>
#include <math.h>

namespace ScaleEffect {

  // === RING BUFFER DEFINITION ===
  volatile int16_t audio_buffer[AUDIO_IN_BUFFER_SIZE];
  volatile uint8_t buffer_head = 0;
  volatile uint8_t buffer_tail = 0;

  namespace {
    // === SIMPLIFIED VOCODER CONFIGURATION (now Scale effect) ===
    const int VOCODER_BANDS = 4;  // Reduced for better performance

    // === PHASE ANALYSIS (Goertzel) CONFIG ===
    const int PHASE_FRAME_N = 32;     // frame size
    const int PHASE_HOP = 32;         // hop size (no overlap to keep CPU low)

    struct PhaseBand {
      int16_t coeffQ15;   // 2*cos(w) in Q15
      int16_t cosQ15;     // cos(w) in Q15
      int16_t sinQ15;     // sin(w) in Q15
      int32_t s1;
      int32_t s2;
      int16_t lastPhaseQ15;     // last unwrapped phase
      int16_t displayPhaseQ15;  // phase used for synthesis
      int16_t phaseStepQ15;     // per-sample interpolation step across frame
      int targetFreq;           // center frequency for this band
    };

    PhaseBand phaseBands[VOCODER_BANDS];
    int phaseFrameCount = 0;
    
    // === CARRIER OSCILLATORS (for harmonic content) ===
    Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> robotOsc1(SAW2048_DATA);     // Main
    Oscil<SQUARE_NO_ALIAS_2048_NUM_CELLS, AUDIO_RATE> robotOsc2(SQUARE_NO_ALIAS_2048_DATA); // Harmonic
    Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> robotOsc3(SIN2048_DATA);     // Formant
    Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> robotOsc4(SAW2048_DATA);     // Sub-harmonic
    
    // === AUDIO INPUT PROCESSING ===
    volatile int16_t currentSample = 0;
    volatile int16_t lastSample = 0;
    RollingAverage<int, 8> audioEnvelope;  // Simple envelope follower
    
    // === SIMPLIFIED FREQUENCY ANALYSIS ===
    struct SimpleBand {
      RollingAverage<int, 4> envelope;
      int level;
      int targetFreq;
    };
    
    SimpleBand vocoderBands[VOCODER_BANDS] = {
      {RollingAverage<int, 4>(), 0, 150},   // Bass
      {RollingAverage<int, 4>(), 0, 400},   // Mid-low
      {RollingAverage<int, 4>(), 0, 800},   // Mid-high
      {RollingAverage<int, 4>(), 0, 1600}   // Treble
    };
    
    // === SIMPLE FILTERING (Optimized for Arduino) ===
    int lowpass1 = 0, lowpass2 = 0, lowpass3 = 0;
    int highpass1 = 0, highpass2 = 0;
    int bandpass1 = 0, bandpass2 = 0;
    
    // === CONTROL VARIABLES ===
    int basePitch = 200;        // Base pitch
    int formant = 128;          // Character (0-255)
    int intensity = 180;        // Effect strength
    int outputVolume = 255;     // Output volume
    bool active = false;
    
    // === HARMONIC CHARACTERISTICS ===
    int harmonicMix = 128;       // Harmonic content
    int modulation = 100;        // Modulation depth
    int audioActivity = 0;       // Current audio input level
    
    // === GESTURE CONTROL RANGES ===
    const int MIN_PITCH = 80;    // Low voice
    const int MAX_PITCH = 600;   // High voice
    const int MIN_FORMANT = 60;        // Dark/warm
    const int MAX_FORMANT = 200;       // Bright/metallic
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    // Initialize oscillators
    robotOsc1.setFreq((float)basePitch);
    robotOsc2.setFreq((float)(basePitch * 1.5f));
    robotOsc3.setFreq((float)(basePitch * 0.5f));
    robotOsc4.setFreq((float)(basePitch * 2.0f));
    
    // Initialize band processing
    for (int i = 0; i < VOCODER_BANDS; i++) {
      vocoderBands[i].level = 0;
    }
    
    // Reset processing vars
    currentSample = 0;
    lastSample = 0;
    audioActivity = 0;
    
    // Reset filters
    lowpass1 = lowpass2 = lowpass3 = 0;
    highpass1 = highpass2 = 0;
    bandpass1 = bandpass2 = 0;
    
    // Initialize Goertzel/phase trackers per band
    phaseFrameCount = 0;
    for (int i = 0; i < VOCODER_BANDS; i++) {
      phaseBands[i].s1 = 0;
      phaseBands[i].s2 = 0;
      phaseBands[i].lastPhaseQ15 = 0;
      phaseBands[i].displayPhaseQ15 = 0;
      phaseBands[i].phaseStepQ15 = 0;
      phaseBands[i].targetFreq = vocoderBands[i].targetFreq;
      float w = 2.0f * 3.14159265f * ((float)phaseBands[i].targetFreq / (float)AUDIO_RATE);
      float c = cosf(w);
      float s = sinf(w);
      phaseBands[i].coeffQ15 = (int16_t)lrintf(2.0f * c * 32767.0f);
      phaseBands[i].cosQ15   = (int16_t)lrintf(c * 32767.0f);
      phaseBands[i].sinQ15   = (int16_t)lrintf(s * 32767.0f);
    }
    
    active = true;
    Serial.println(F("Scale Effect: active"));
  }

  void exit() {
    active = false;
  }

  void update(bool leftHand, bool rightHand, float d1, float d2) {
    // === PITCH CONTROL (Right hand) ===
    if (rightHand) {
      int distance = constrain((int)d2, 1, 20);
      basePitch = map(distance, 1, 20, MAX_PITCH, MIN_PITCH);
      intensity = map(distance, 1, 20, 255, 120);
      harmonicMix = map(distance, 1, 20, 200, 80);
      active = true;
    }

    // === CHARACTER / VOLUME (Left hand) ===
    if (leftHand) {
      int distance = constrain((int)d1, 1, 20);
      formant = map(distance, 1, 20, MAX_FORMANT, MIN_FORMANT);
      modulation = map(distance, 1, 20, 200, 50);
      outputVolume = map(distance, 1, 20, 255, 100);
      active = true;
    } else {
      formant = 128;
      modulation = 100;
      outputVolume = 180;
    }
    
    float formantMod = formant / 128.0f; // 0.47 to 1.56 range
    robotOsc1.setFreq((float)(basePitch * formantMod));
    robotOsc2.setFreq((float)(basePitch * 1.5f * formantMod));
    robotOsc3.setFreq((float)(basePitch * 0.5f * formantMod));
    robotOsc4.setFreq((float)(basePitch * 2.0f * formantMod));
  }

  void feedAudioSample(int16_t sample) {
    uint8_t next_head = (buffer_head + 1) % AUDIO_IN_BUFFER_SIZE;
    if (next_head != buffer_tail) {
      audio_buffer[buffer_head] = sample;
      buffer_head = next_head;
    }
  }
 
  int audio() {
    if (!active) {
      return 0; // Silence for Mozzi
    }

    if (buffer_tail == buffer_head) {
      return 0; // Buffer empty
    }

    int16_t currentSample = audio_buffer[buffer_tail];
    buffer_tail = (buffer_tail + 1) % AUDIO_IN_BUFFER_SIZE;

    // === ANALYSIS ===
    lowpass1 += (currentSample - lowpass1) >> 3;  // ~1kHz cutoff
    lowpass2 += (lowpass1 - lowpass2) >> 4;       // ~500Hz cutoff
    lowpass3 += (lowpass2 - lowpass3) >> 5;       // ~250Hz cutoff

    highpass1 = currentSample - lowpass1;
    highpass2 = currentSample - lowpass2;

    bandpass1 = lowpass1 - lowpass2;  // Mid-low band
    bandpass2 = lowpass2 - lowpass3;  // Low band

    vocoderBands[0].level = vocoderBands[0].envelope.next(abs(lowpass3) >> 2);     // Bass
    vocoderBands[1].level = vocoderBands[1].envelope.next(abs(bandpass2) >> 1);    // Mid-low
    vocoderBands[2].level = vocoderBands[2].envelope.next(abs(bandpass1));         // Mid-high
    vocoderBands[3].level = vocoderBands[3].envelope.next(abs(highpass1) >> 1);    // Treble

    // === GOERTZEL ACCUMULATION PER BAND ===
    for (int i = 0; i < VOCODER_BANDS; i++) {
      int32_t s0 = (int32_t)currentSample + ((int32_t)phaseBands[i].coeffQ15 * phaseBands[i].s1 >> 15) - phaseBands[i].s2;
      phaseBands[i].s2 = phaseBands[i].s1;
      phaseBands[i].s1 = s0;
    }
    phaseFrameCount++;

    if (phaseFrameCount >= PHASE_FRAME_N) {
      for (int i = 0; i < VOCODER_BANDS; i++) {
        int32_t re = phaseBands[i].s1 - ((int32_t)phaseBands[i].cosQ15 * phaseBands[i].s2 >> 15);
        int32_t im = ((int32_t)phaseBands[i].sinQ15 * phaseBands[i].s2) >> 15;
        float phase = atan2f((float)im, (float)re);
        int16_t newPhaseQ15 = (int16_t)lrintf(phase * (32768.0f / 3.14159265f));
        int16_t delta = newPhaseQ15 - phaseBands[i].lastPhaseQ15;
        if (delta > 16384) delta -= 32768;
        if (delta < -16384) delta += 32768;
        int16_t unwrapped = phaseBands[i].lastPhaseQ15 + delta;
        phaseBands[i].lastPhaseQ15 = unwrapped;
        int16_t diff = unwrapped - phaseBands[i].displayPhaseQ15;
        phaseBands[i].phaseStepQ15 = diff / PHASE_FRAME_N;
        phaseBands[i].s1 = 0;
        phaseBands[i].s2 = 0;
      }
      phaseFrameCount = 0;
    }

    for (int i = 0; i < VOCODER_BANDS; i++) {
      phaseBands[i].displayPhaseQ15 += phaseBands[i].phaseStepQ15;
    }

    auto phaseToIndex = [](int16_t q15) -> uint16_t {
      int32_t x = (int32_t)q15 + 32768; // [0..65535]
      return (uint16_t)((x * 2048L) >> 16);
    };

    int out = 0;
    {
      uint16_t idx = phaseToIndex(phaseBands[0].displayPhaseQ15);
      int8_t s = pgm_read_byte_near(SIN2048_DATA + idx);
      int bandOut = ((int32_t)s * vocoderBands[0].level) >> 8;
      out += bandOut >> 1;
    }
    {
      uint16_t idx = phaseToIndex(phaseBands[1].displayPhaseQ15);
      int8_t s = pgm_read_byte_near(SIN2048_DATA + idx);
      int bandOut = ((int32_t)s * vocoderBands[1].level) >> 7;
      out += bandOut;
    }
    {
      uint16_t idx = phaseToIndex(phaseBands[2].displayPhaseQ15);
      int8_t s = pgm_read_byte_near(SIN2048_DATA + idx);
      int bandOut = ((int32_t)s * vocoderBands[2].level) >> 7;
      out += bandOut >> 1;
    }
    {
      uint16_t idx = phaseToIndex(phaseBands[3].displayPhaseQ15);
      int8_t s = pgm_read_byte_near(SIN2048_DATA + idx);
      int bandOut = ((int32_t)s * vocoderBands[3].level) >> 8;
      out += bandOut >> 1;
    }

    out = ((int32_t)out * intensity) >> 8;
    int harmonics = ((int32_t)out * harmonicMix) >> 9;
    out += harmonics;
    int modulated = ((int32_t)out * modulation) >> 8;
    out = (out + modulated) >> 1;

    out = constrain(out, -127, 127);
    return (out * 255) >> 7;  // Proper Mozzi volume scaling
  }
}
