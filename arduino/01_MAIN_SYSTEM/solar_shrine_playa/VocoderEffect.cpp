#include "VocoderEffect.h"

#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/saw2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include <RollingAverage.h>
#include <avr/pgmspace.h>
#include <math.h>

namespace VocoderEffect {

  // === RING BUFFER DEFINITION ===
  volatile int16_t audio_buffer[AUDIO_IN_BUFFER_SIZE];
  volatile uint8_t buffer_head = 0;
  volatile uint8_t buffer_tail = 0;

  namespace {
    // === SIMPLIFIED VOCODER CONFIGURATION ===
    const int VOCODER_BANDS = 4;  // Reduced for better performance
    const int BUFFER_SIZE = 16;   // legacy (unused for input ring)

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
    
    // === CARRIER OSCILLATORS (Robot Voice Generators) ===
    Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> robotOsc1(SAW2048_DATA);     // Main robot voice
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
    
    // === VOCODER CONTROL VARIABLES ===
    int robotPitch = 200;        // Base robot voice pitch
    int robotFormant = 128;      // Voice character (0-255)
    int robotIntensity = 180;    // Robot effect strength
    int vocoderVolume = 255;     // Output volume
    bool vocoderActive = false;
    
    // === ROBOT VOICE CHARACTERISTICS ===
    int harmonicMix = 128;       // Harmonic content
    int voiceModulation = 100;   // Voice modulation depth
    int audioActivity = 0;       // Current audio input level
    
    // === GESTURE CONTROL RANGES ===
    const int MIN_ROBOT_PITCH = 80;    // Deep robot voice
    const int MAX_ROBOT_PITCH = 600;   // High robot voice
    const int MIN_FORMANT = 60;        // Dark/warm voice
    const int MAX_FORMANT = 200;       // Bright/metallic voice
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    // Initialize robot voice oscillators with harmonic relationships
    robotOsc1.setFreq((float)robotPitch);                    // Fundamental robot voice
    robotOsc2.setFreq((float)(robotPitch * 1.5f));           // Perfect fifth harmonic
    robotOsc3.setFreq((float)(robotPitch * 0.5f));           // Sub-harmonic for depth
    robotOsc4.setFreq((float)(robotPitch * 2.0f));           // Octave harmonic
    
    // Initialize frequency band processing
    for (int i = 0; i < VOCODER_BANDS; i++) {
      vocoderBands[i].level = 0;
    }
    
    // Reset audio processing variables
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
      // Precompute coefficients at enter() (float ok here)
      float w = 2.0f * 3.14159265f * ((float)phaseBands[i].targetFreq / (float)AUDIO_RATE);
      float c = cosf(w);
      float s = sinf(w);
      phaseBands[i].coeffQ15 = (int16_t)lrintf(2.0f * c * 32767.0f);
      phaseBands[i].cosQ15   = (int16_t)lrintf(c * 32767.0f);
      phaseBands[i].sinQ15   = (int16_t)lrintf(s * 32767.0f);
    }
    
    vocoderActive = true;
  }

  void exit() {
    vocoderActive = false;
  }

  void update(bool leftHand, bool rightHand, float d1, float d2) {
    // === ROBOT PITCH CONTROL (Right hand) ===
    if (rightHand) {
      int distance = constrain((int)d2, 1, 20);
      // Map right hand distance to robot voice pitch
      robotPitch = map(distance, 1, 20, MAX_ROBOT_PITCH, MIN_ROBOT_PITCH);
      // Control robot intensity - closer = more robotic
      robotIntensity = map(distance, 1, 20, 255, 120);
      // Harmonic content control
      harmonicMix = map(distance, 1, 20, 200, 80);
      vocoderActive = true;
    }

    // === VOICE CHARACTER CONTROL (Left hand) ===
    if (leftHand) {
      // Map left hand distance to voice formant/character
      int distance = constrain((int)d1, 1, 20);
      robotFormant = map(distance, 1, 20, MAX_FORMANT, MIN_FORMANT);
      
      // Voice modulation depth - closer = more modulated
      voiceModulation = map(distance, 1, 20, 200, 50);
      
      // Volume control - closer = louder
      vocoderVolume = map(distance, 1, 20, 255, 100);
      
      vocoderActive = true;
    } else {
      robotFormant = 128;      // Neutral voice character
      voiceModulation = 100;   // Moderate modulation
      vocoderVolume = 180;     // Default volume
    }
    
    // === UPDATE ROBOT OSCILLATORS ===
    float formantMod = robotFormant / 128.0f; // 0.47 to 1.56 range
    
    robotOsc1.setFreq((float)(robotPitch * formantMod));           // Main robot voice
    robotOsc2.setFreq((float)(robotPitch * 1.5f * formantMod));     // Harmonic
    robotOsc3.setFreq((float)(robotPitch * 0.5f * formantMod));     // Sub-harmonic
    robotOsc4.setFreq((float)(robotPitch * 2.0f * formantMod));     // Upper harmonic
  }

  void feedAudioSample(int16_t sample) {
    uint8_t next_head = (buffer_head + 1) % AUDIO_IN_BUFFER_SIZE;

    // Check for buffer overflow
    if (next_head != buffer_tail) {
        audio_buffer[buffer_head] = sample;
        buffer_head = next_head;
    }
    // Optional: else, handle buffer overflow by dropping the sample
   }
 
   int audio() {
    if (!vocoderActive) {
        return 0; // Silence for Mozzi
    }

    // Check if there is data in the buffer to process
    if (buffer_tail == buffer_head) {
        return 0; // Buffer is empty, return silence
    }

    // Read the next sample from the buffer
    int16_t currentSample = audio_buffer[buffer_tail];
    buffer_tail = (buffer_tail + 1) % AUDIO_IN_BUFFER_SIZE;

    // === AUDIO ANALYSIS (existing envelopes retained) ===
    lowpass1 += (currentSample - lowpass1) >> 3;  // ~1kHz cutoff
    lowpass2 += (lowpass1 - lowpass2) >> 4; // ~500Hz cutoff
    lowpass3 += (lowpass2 - lowpass3) >> 5; // ~250Hz cutoff

    // High-pass filtering for treble content
    highpass1 = currentSample - lowpass1;
    highpass2 = currentSample - lowpass2;

    // Band-pass approximations
    bandpass1 = lowpass1 - lowpass2;  // Mid-low band
    bandpass2 = lowpass2 - lowpass3;  // Low band

    // Update frequency band levels
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

    // Finalize phase once per frame
    if (phaseFrameCount >= PHASE_FRAME_N) {
      for (int i = 0; i < VOCODER_BANDS; i++) {
        // Finalize Goertzel: re = s1 - s2*cos(w); im = s2*sin(w)
        int32_t re = phaseBands[i].s1 - ((int32_t)phaseBands[i].cosQ15 * phaseBands[i].s2 >> 15);
        int32_t im = ((int32_t)phaseBands[i].sinQ15 * phaseBands[i].s2) >> 15;

        float phase = atan2f((float)im, (float)re); // rarely executed, 4 bands/frame
        int16_t newPhaseQ15 = (int16_t)lrintf(phase * (32768.0f / 3.14159265f)); // map [-pi,pi] -> Q15

        // Unwrap against lastPhaseQ15
        int16_t delta = newPhaseQ15 - phaseBands[i].lastPhaseQ15;
        if (delta > 16384) delta -= 32768;
        if (delta < -16384) delta += 32768;
        int16_t unwrapped = phaseBands[i].lastPhaseQ15 + delta;
        phaseBands[i].lastPhaseQ15 = unwrapped;

        // Interpolation step across next frame
        int16_t diff = unwrapped - phaseBands[i].displayPhaseQ15;
        phaseBands[i].phaseStepQ15 = diff / PHASE_FRAME_N;

        // Reset accumulators
        phaseBands[i].s1 = 0;
        phaseBands[i].s2 = 0;
      }
      phaseFrameCount = 0;
    }

    // Advance phase per sample for synthesis
    for (int i = 0; i < VOCODER_BANDS; i++) {
      phaseBands[i].displayPhaseQ15 += phaseBands[i].phaseStepQ15;
    }
    // === PHASE-DRIVEN CARRIER SYNTHESIS USING SIN LUT ===
    auto phaseToIndex = [](int16_t q15) -> uint16_t {
      int32_t x = (int32_t)q15 + 32768; // [0..65535]
      return (uint16_t)((x * 2048L) >> 16);
    };

    int vocodedOutput = 0;
    // Band 0: Bass with lower weight
    {
      uint16_t idx = phaseToIndex(phaseBands[0].displayPhaseQ15);
      int8_t s = pgm_read_byte_near(SIN2048_DATA + idx);
      int bandOut = ((int32_t)s * vocoderBands[0].level) >> 8;
      vocodedOutput += bandOut >> 1; // softer
    }
    // Band 1: Mid-low (dominant)
    {
      uint16_t idx = phaseToIndex(phaseBands[1].displayPhaseQ15);
      int8_t s = pgm_read_byte_near(SIN2048_DATA + idx);
      int bandOut = ((int32_t)s * vocoderBands[1].level) >> 7;
      vocodedOutput += bandOut;
    }
    // Band 2: Mid-high
    {
      uint16_t idx = phaseToIndex(phaseBands[2].displayPhaseQ15);
      int8_t s = pgm_read_byte_near(SIN2048_DATA + idx);
      int bandOut = ((int32_t)s * vocoderBands[2].level) >> 7;
      vocodedOutput += bandOut >> 1;
    }
    // Band 3: Treble
    {
      uint16_t idx = phaseToIndex(phaseBands[3].displayPhaseQ15);
      int8_t s = pgm_read_byte_near(SIN2048_DATA + idx);
      int bandOut = ((int32_t)s * vocoderBands[3].level) >> 8;
      vocodedOutput += bandOut >> 1;
    }
    
    // === APPLY ROBOT CHARACTERISTICS ===
    vocodedOutput = ((int32_t)vocodedOutput * robotIntensity) >> 8;
    
    int harmonics = ((int32_t)vocodedOutput * harmonicMix) >> 9;
    vocodedOutput += harmonics;
    
    int modulated = ((int32_t)vocodedOutput * voiceModulation) >> 8;
    vocodedOutput = (vocodedOutput + modulated) >> 1;
    
    // === MOZZI VOLUME PATTERN (CRITICAL) ===
    // Must use oscillator * 255 pattern for proper Mozzi volume
    // Scale the vocoded output and apply maximum volume
    vocodedOutput = constrain(vocodedOutput, -127, 127);
    
    // Apply final volume using Mozzi pattern
    return (vocodedOutput * 255) >> 7;  // Proper Mozzi volume scaling
  }
}