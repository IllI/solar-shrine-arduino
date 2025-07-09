/*
 * Test compilation of the fixed Solar Shrine Effect Rotator
 * This is a minimal version to verify all compilation issues are resolved
 */

#include <ArduinoJson.h>
#include <FastLED.h>
#include <NewPing.h>
#include <math.h>

// Mozzi includes
#include <MozziGuts.h>
#include <Oscil.h>
#include <RollingAverage.h>
#include <tables/triangle_valve_2048_int8.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>

// Basic setup
#define CONTROL_RATE_THEREMIN 128
#define CONTROL_RATE_ROBOTS 256

// Test audio variables
Oscil <TRIANGLE_VALVE_2048_NUM_CELLS, AUDIO_RATE> thereminOsc(TRIANGLE_VALVE_2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> robotsOsc(SIN2048_DATA);
Oscil <COS2048_NUM_CELLS, CONTROL_RATE_ROBOTS> robotsModulator(COS2048_DATA);

// Test Timer1 variables
volatile uint16_t alienPhase = 0;
volatile uint16_t alienMod = 0;
volatile bool alienActive = false;
volatile uint8_t alienVolume = 0;

// Test DJ Scratch variables
volatile int32_t djSampleIndex = 0;
volatile uint8_t djSampleCounter = 0;
volatile uint8_t djPlayState = 0;
volatile uint8_t djPlaybackSpeed = 5;
volatile bool djScratchMode = false;
volatile int8_t djScratchSpeed = 1;

const int DJ_BUFFER_SIZE = 512;
volatile uint8_t djAudioBuffer[DJ_BUFFER_SIZE];

// Test state
enum EffectType {
  EFFECT_THEREMIN = 0,
  EFFECT_ALIEN = 1,
  EFFECT_ROBOTS = 2,
  EFFECT_DJ_SCRATCH = 3
};

struct SystemState {
  EffectType currentEffect;
  bool mozziActive;
  bool timer1Active;
  int audioVolume;
  bool handsDetected;
  float distance1, distance2;
  bool inRange1, inRange2;
};

SystemState state;

// Test functions
void updateThereminControl() {
  // Test function body
  state.audioVolume = state.handsDetected ? 200 : 0;
}

int updateThereminAudio() {
  return (thereminOsc.next() * state.audioVolume) >> 8;
}

void updateAlienControl() {
  alienActive = state.handsDetected;
  alienVolume = state.handsDetected ? 200 : 0;
  state.audioVolume = alienVolume;
}

void updateRobotsControl() {
  state.audioVolume = state.handsDetected ? 200 : 0;
}

int updateRobotsAudio() {
  int sample = robotsOsc.next();
  sample = (sample > 0) ? 127 : -127;
  return (sample * state.audioVolume) >> 8;
}

void updateDJScratchControl() {
  djPlayState = state.handsDetected ? 1 : 0;
  djScratchMode = state.inRange1 && state.inRange2;
  state.audioVolume = djPlayState ? 200 : 0;
}

// Combined ISR for Timer1 effects
ISR(TIMER1_COMPA_vect) {
  if (state.currentEffect == EFFECT_ALIEN) {
    if (alienActive && alienVolume > 0) {
      alienPhase += 200;
      alienMod += 50;
      
      uint8_t modulator = (alienMod >> 8) & 0xFF;
      uint8_t carrier = ((alienPhase + (modulator * 4)) >> 8) & 0xFF;
      
      int16_t amp = (carrier > 128) ? (alienVolume / 4) : -(alienVolume / 4);
      OCR1A = ((uint32_t)(amp + 128) * 399) / 255;
    } else {
      OCR1A = 399 / 2;
    }
  } else if (state.currentEffect == EFFECT_DJ_SCRATCH) {
    djSampleCounter++;
    
    if (djSampleCounter >= djPlaybackSpeed) {
      djSampleCounter = 0;
      
      if (djPlayState == 1) {
        if (djSampleIndex < 0) djSampleIndex = 0;
        if (djSampleIndex >= DJ_BUFFER_SIZE) djSampleIndex = 0;
        
        uint8_t sample = djAudioBuffer[djSampleIndex];
        int16_t amp = ((int16_t)sample - 128) * 4;
        amp = constrain(amp, -128, 127);
        
        OCR1A = ((uint32_t)(amp + 128) * 399) / 255;
        
        djSampleIndex++;
        if (djSampleIndex >= DJ_BUFFER_SIZE) djSampleIndex = 0;
      } else {
        OCR1A = 399 / 2;
      }
    }
  } else {
    OCR1A = 399 / 2;
  }
}

// Test audio control function (renamed to avoid conflict)
void updateAudioControl() {
  switch (state.currentEffect) {
    case EFFECT_THEREMIN:
      updateThereminControl();
      break;
    case EFFECT_ALIEN:
      updateAlienControl();
      break;
    case EFFECT_ROBOTS:
      updateRobotsControl();
      break;
    case EFFECT_DJ_SCRATCH:
      updateDJScratchControl();
      break;
  }
}

// Required Mozzi functions
void updateControl() {
  // Called by Mozzi when active
}

AudioOutput_t updateAudio() {
  // Called by Mozzi for audio generation
  switch (state.currentEffect) {
    case EFFECT_THEREMIN:
      return updateThereminAudio();
    case EFFECT_ROBOTS:
      return updateRobotsAudio();
    default:
      return 0;
  }
}

void setup() {
  Serial.begin(9600);
  
  // Initialize state
  state.currentEffect = EFFECT_THEREMIN;
  state.mozziActive = false;
  state.timer1Active = false;
  state.audioVolume = 0;
  state.handsDetected = false;
  state.distance1 = 10.0;
  state.distance2 = 10.0;
  state.inRange1 = false;
  state.inRange2 = false;
  
  // Initialize DJ buffer
  for (int i = 0; i < DJ_BUFFER_SIZE; i++) {
    djAudioBuffer[i] = 128 + (sin(i * 0.1) * 100);
  }
  
  // Start with Theremin
  startMozzi(CONTROL_RATE_THEREMIN);
  thereminOsc.setFreq(220);
  state.mozziActive = true;
  
  Serial.println("Test compilation successful!");
}

void loop() {
  // Test the functions
  updateAudioControl();
  
  // Handle Mozzi audio hook if active
  if (state.mozziActive) {
    audioHook();
  }
  
  delay(10);
} 