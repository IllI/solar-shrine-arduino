/*
 * Modular Audio Effect System - DJ Scratch + Mozzi Alien Effect + Robot Talk
 * Completely separate audio methodologies:
 * - DJ Scratch: Timer1 PWM + ISR for PROGMEM audio playback
 * - Alien Effect: Mozzi library with proper audioHook() integration
 * - Robot Talk: tone() function with a pitch pattern
 * 
 * Toggles between effects every 10 seconds with complete audio system isolation
 */

// =============================================================================
// AUDIO MODE SELECTION - Only one can be active at a time
// =============================================================================
enum AudioMode {
  MODE_DJ_SCRATCH = 0,
  MODE_MOZZI_ALIEN = 1,
  MODE_MOZZI_ROBOT = 2
};

volatile AudioMode currentMode = MODE_DJ_SCRATCH;
unsigned long lastModeChange = 0;
const unsigned long MODE_DURATION = 10000; // 10 seconds per mode

// =============================================================================
// SENSOR SYSTEM (Shared between all modes)
// =============================================================================
#define TRIG1 10  // Left sensor trigger
#define ECHO1 11  // Left sensor echo
#define TRIG2 5   // Right sensor trigger
#define ECHO2 6   // Right sensor echo

// =============================================================================
// DJ SCRATCH SYSTEM (Timer1 + ISR + PROGMEM)
// =============================================================================
#include <avr/pgmspace.h>
#include "whileDancing_medium_data.h"

// DJ Scratch pins and state
#define AUDIO_PIN_DJ 12  // Timer1 OC1B output
volatile int32_t sampleIndex = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t playState = 1;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

// DJ Scratch Timer1 ISR
ISR(TIMER1_COMPB_vect) {
  if (currentMode != MODE_DJ_SCRATCH) {
    OCR1B = 200; // Silence when not in DJ mode
    return;
  }
  
  if (playState == 1 && sampleIndex < WHILEDANCING_MEDIUM_DATA_NUM_CELLS) {
    sampleCounter++;
    if (sampleCounter >= playbackSpeed) {
      sampleCounter = 0;
      int8_t rawSample = pgm_read_byte(&WHILEDANCING_MEDIUM_DATA_DATA[sampleIndex]);
      uint8_t sample = (uint8_t)(rawSample + 128);
      uint16_t amplified = ((uint16_t)sample * 3) / 2;
      if (amplified > 255) amplified = 255;
      OCR1B = amplified;
      
      if (isScratchMode) {
        sampleIndex += scratchSpeed;
        if (sampleIndex < 0) sampleIndex = 0;
        if (sampleIndex >= WHILEDANCING_MEDIUM_DATA_NUM_CELLS) sampleIndex = WHILEDANCING_MEDIUM_DATA_NUM_CELLS - 1;
      } else {
        sampleIndex++;
      }
    }
  } else {
    OCR1B = 128; // Silence
    if (sampleIndex >= WHILEDANCING_MEDIUM_DATA_NUM_CELLS) {
      sampleIndex = 0; // Loop
    }
  }
}

void setupDJScratch() {
  Serial.println(F("Setting up DJ Scratch mode..."));
  pinMode(AUDIO_PIN_DJ, OUTPUT);
  TCCR1A = _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 399;
  OCR1B = 200;
  TIMSK1 = _BV(OCIE1B);
  sampleIndex = 0;
  sampleCounter = 0;
  playState = 1;
  playbackSpeed = 5;
  scratchSpeed = 1;
  isScratchMode = false;
}

void disableDJScratch() {
  Serial.println(F("Disabling DJ Scratch mode..."));
  TIMSK1 &= ~_BV(OCIE1B);
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1B = 0;
  digitalWrite(AUDIO_PIN_DJ, LOW);
}

// =============================================================================
// MOZZI ROBOT SYSTEM (Functions defined after variable declarations)
// =============================================================================

// =============================================================================
// MOZZI SYSTEMS (Alien)
// =============================================================================
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM
#define MOZZI_AUDIO_PIN_1 12
#define MOZZI_AUDIO_PIN_1_REGISTER OCR1B
#define MOZZI_AUDIO_PIN_1_LOW 13
#define MOZZI_AUDIO_PIN_1_LOW_REGISTER OCR1C

#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/saw2048_int8.h>
#include <RollingAverage.h>

#define CONTROL_RATE 64

// --- Alien Effect ---
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> alienOsc(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> vibrato(SIN2048_DATA);
float alienVolume = 0.5;
float alienPitch = 440.0;
float vibratoRate = 6.5;

// --- Robot Effect ---
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> robotOsc1(SIN2048_DATA);
Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> robotOsc2(SAW2048_DATA);
float robotFormantPhase = 0.0f;
float robotFormantRate = 1.5f;
int robotBaseFreq = 300;
uint8_t robotVolume = 0;
unsigned long lastRobotMs = 0;

void setupMozziRobot() {
  Serial.println(F("Setting up Mozzi Robot mode..."));
  startMozzi(CONTROL_RATE);
  robotOsc1.setFreq(300);
  robotOsc2.setFreq(318); // Slightly detuned for beating effect
  robotFormantPhase = 0.0f;
  robotVolume = 0;
  lastRobotMs = millis();
}

void disableMozziRobot() {
  Serial.println(F("Disabling Mozzi Robot mode..."));
  stopMozzi();
}

void updateRobotControl(float distanceLeft, float distanceRight) {
  unsigned long now = millis();
  float dt = (now - lastRobotMs) / 1000.0f;
  lastRobotMs = now;

  bool pitchActive = (distanceRight >= 5 && distanceRight <= 50);
  bool voiceActive = (distanceLeft >= 5 && distanceLeft <= 50);

  // If no hands, silence
  if (!pitchActive && !voiceActive) {
    robotVolume = 0;
    return;
  }

  // Right hand controls base frequency
  if (pitchActive) {
    robotBaseFreq = map((int)distanceRight, 5, 50, 900, 120);
    robotOsc1.setFreq(robotBaseFreq);
    robotOsc2.setFreq(robotBaseFreq * 1.06f); // 6% detune
  }

  // Left hand controls formant rate and volume
  if (voiceActive) {
    robotFormantRate = map((int)distanceLeft, 5, 50, 6, 1);
    robotVolume = map((int)distanceLeft, 5, 50, 50, 255);
  } else {
    robotVolume = 0;
  }

  // Update formant modulation
  robotFormantPhase += robotFormantRate * dt;
  if (robotFormantPhase >= 1.0f) robotFormantPhase -= 1.0f;
}

void setupMozziAlien() {
  Serial.println(F("Setting up Mozzi Alien mode..."));
  startMozzi(CONTROL_RATE);
  alienOsc.setFreq(alienPitch);
  vibrato.setFreq(vibratoRate);
  alienVolume = 0.5;
  alienPitch = 440.0;
}

void disableMozziAlien() {
  Serial.println(F("Disabling Mozzi Alien mode..."));
  stopMozzi();
}

// =============================================================================
// MAIN SYSTEM
// =============================================================================
void setup() {
  Serial.begin(9600);
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  // Start with DJ Scratch mode
  setupDJScratch();
}

void loop() {
  if (currentMode == MODE_MOZZI_ALIEN || currentMode == MODE_MOZZI_ROBOT) {
    audioHook(); // MUST be called in loop for Mozzi to work
  }

  if (millis() - lastModeChange > MODE_DURATION) {
    lastModeChange = millis();

    // Disable current mode
    switch (currentMode) {
      case MODE_DJ_SCRATCH:
        disableDJScratch();
        break;
      case MODE_MOZZI_ALIEN:
        disableMozziAlien();
        break;
      case MODE_MOZZI_ROBOT:
        disableMozziRobot();
        break;
    }

    // Cycle to next mode
    currentMode = (AudioMode)((currentMode + 1) % 3);

    // Enable new mode
    switch (currentMode) {
      case MODE_DJ_SCRATCH:
        setupDJScratch();
        break;
      case MODE_MOZZI_ALIEN:
        setupMozziAlien();
        break;
      case MODE_MOZZI_ROBOT:
        setupMozziRobot();
        break;
    }
  }
}

// =============================================================================
// SHARED FUNCTIONS
// =============================================================================
long readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH, 30000);
}

// =============================================================================
// MOZZI AUDIO PROCESSING
// =============================================================================
void updateControl() {
  if (currentMode == MODE_MOZZI_ALIEN || currentMode == MODE_MOZZI_ROBOT) {
    long d1_us = readSensor(TRIG1, ECHO1);
    long d2_us = readSensor(TRIG2, ECHO2);
    float d1 = d1_us / 58.0;
    float d2 = d2_us / 58.0;

    if (currentMode == MODE_MOZZI_ALIEN) {
      bool leftHand = (d1 >= 1.0 && d1 <= 20.0);
      bool rightHand = (d2 >= 1.0 && d2 <= 20.0);

      if (leftHand) {
        alienVolume = map(d1 * 10, 10, 200, 255, 50);
      } else {
        alienVolume = 0;
      }
      if (rightHand) {
        alienPitch = map(d2 * 10, 10, 200, 800, 200);
      } else {
        alienPitch = 400;
      }
      vibrato.setFreq(vibratoRate);
      alienOsc.setFreq(alienPitch);
    } else if (currentMode == MODE_MOZZI_ROBOT) {
      updateRobotControl(d1, d2);
    }
  }
}

int updateAudio() {
  if (currentMode == MODE_MOZZI_ALIEN) {
    return (int8_t)(alienOsc.next() * (alienVolume/255.0));
  } else if (currentMode == MODE_MOZZI_ROBOT) {
    // Generate robot talking sound with AM modulation
    float am = 0.5f * (1.0f + sin(6.2831853f * robotFormantPhase));
    
    int osc1Sample = robotOsc1.next();
    int osc2Sample = robotOsc2.next();
    
    // Mix oscillators with AM modulation
    int mixedSample = ((osc1Sample + osc2Sample) >> 1);
    mixedSample = (int)(mixedSample * am);
    
    // Apply volume
    mixedSample = (mixedSample * robotVolume) >> 8;
    
    return mixedSample;
  }
  return 0;
}
