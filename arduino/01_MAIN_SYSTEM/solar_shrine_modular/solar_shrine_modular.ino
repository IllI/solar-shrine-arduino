/*
 * Solar Shrine Modular Audio System
 * 
 * A modular audio system with 4 rotating effects:
 * 1) DJ Scratch (Timer1 ISR + PROGMEM samples)
 * 2) Alien (Mozzi oscillators)
 * 3) Robots (Mozzi synthesis)
 * 4) Mini Theremin (Mozzi oscillators)
 * 
 * Hardware:
 * - Arduino Uno/Nano
 * - 2x HC-SR04 ultrasonic sensors
 * - Audio output on pin 12 (OCR1B)
 * - LED strip on pin 3
 * 
 * Key Architecture:
 * - DJ Scratch uses Timer1 ISR (isolated from Mozzi)
 * - Mozzi effects use audioHook() (isolated from Timer1)
 * - Proper mode switching with system shutdown/startup
 * - Executive intelligence prevents timer conflicts
 * 
 * Author: AI Assistant
 * Date: 2024
 */

// =============================================================================
// AUDIO MODE DEFINITIONS
// =============================================================================
enum AudioMode {
  MODE_DJ_SCRATCH,
  MODE_ALIEN,
  MODE_ROBOTS,
  MODE_THEREMIN
};

AudioMode currentMode = MODE_DJ_SCRATCH;
unsigned long lastModeChange = 0;
const unsigned long MODE_DURATION = 5000; // 5 seconds per mode

// =============================================================================
// DJ SCRATCH SYSTEM (Timer1 ISR + PROGMEM)
// =============================================================================
#include "whileDancing_medium_data.h"

#define AUDIO_PIN_DJ 9  // Timer2 OC2B output (moved from pin 12 to avoid Mozzi conflict)

#include "dj_scratch.h"

// DJ Scratch state variables are now defined in dj_scratch.cpp

// Timer2 Compare B interrupt for DJ scratch audio is now defined in dj_scratch.cpp

// =============================================================================
// MOZZI SYSTEM (Alien, Robots, Theremin)
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

#define CONTROL_RATE 128

// Mozzi oscillators
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> alienOsc(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, CONTROL_RATE> vibrato(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> robotOsc(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> thereminOsc(SIN2048_DATA);

// Effect state
float effectVolume = 0.0;
float effectPitch = 440.0;
float vibratoRate = 6.5;

// =============================================================================
// SENSOR SYSTEM
// =============================================================================
const int TRIG1 = 10; // Left sensor
const int ECHO1 = 11; // Left sensor (pin 11 safe when Mozzi uses pin 12/13)
const int TRIG2 = 5;  // Right sensor
const int ECHO2 = 6;  // Right sensor

const float MIN_RANGE = 5.0;
const float MAX_RANGE = 50.0;

// =============================================================================
// LED SYSTEM
// =============================================================================
#include <FastLED.h>
#define LED_PIN 3
#define NUM_LEDS 120
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// =============================================================================
// AUDIO SYSTEM MANAGEMENT FUNCTIONS
// =============================================================================

void setupDJScratch() {
  Serial.println(F("Setting up DJ Scratch mode..."));
  
  // Configure Timer2 PWM for DJ scratch (moved from Timer1 to avoid Mozzi conflict)
  pinMode(AUDIO_PIN_DJ, OUTPUT);
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  // Clear OC2B on compare match, Fast PWM
  TCCR2B = _BV(CS20);                              // Fast PWM, no prescaler
  OCR2A = 199;                                     // 20kHz frequency (TOP value)
  OCR2B = 100;                                     // 50% duty cycle (silence)
  TIMSK2 = _BV(OCIE2B);                            // Enable Timer2 Compare B interrupt
  
  // Reset DJ scratch state
  djSampleIndex = 0;
  sampleCounter = 0;
  playState = 1;
  playbackSpeed = 5;
  scratchSpeed = 1;
  isScratchMode = false;
  
  Serial.println("DJ Scratch Timer2 initialized");
}

void disableDJScratch() {
  Serial.println(F("Disabling DJ Scratch mode..."));
  
  // Disable Timer2 interrupt
  TIMSK2 &= ~_BV(OCIE2B);
  
  // Reset Timer2 to default state
  TCCR2A = 0;
  TCCR2B = 0;
  OCR2B = 0;
  
  // Set audio pin to low
  digitalWrite(AUDIO_PIN_DJ, LOW);
  
  Serial.println("DJ Scratch Timer2 disabled");
}

void setupMozziAlien() {
  Serial.println("Setting up Mozzi Alien mode...");
  startMozzi(CONTROL_RATE);
  alienOsc.setFreq(440);
  vibrato.setFreq(vibratoRate);
  Serial.println("Mozzi Alien initialized");
}

void setupMozziRobots() {
  Serial.println("Setting up Mozzi Robots mode...");
  startMozzi(CONTROL_RATE);
  robotOsc.setFreq(220);
  Serial.println("Mozzi Robots initialized");
}

void setupMozziTheremin() {
  Serial.println("Setting up Mozzi Theremin mode...");
  startMozzi(CONTROL_RATE);
  thereminOsc.setFreq(440);
  Serial.println("Mozzi Theremin initialized");
}

void disableMozziAlien() {
  stopMozzi();
  Serial.println("Mozzi Alien disabled");
}

void disableMozziRobots() {
  stopMozzi();
  Serial.println("Mozzi Robots disabled");
}

void disableMozziTheremin() {
  stopMozzi();
  Serial.println("Mozzi Theremin disabled");
}

// =============================================================================
// SENSOR FUNCTIONS
// =============================================================================

float readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) {
    return -1; // Timeout
  }
  
  float distance = (duration * 0.034) / 2;
  return distance;
}

bool handsDetected() {
  float dist1 = readSensor(TRIG1, ECHO1);
  float dist2 = readSensor(TRIG2, ECHO2);
  
  bool hand1 = (dist1 >= MIN_RANGE && dist1 <= MAX_RANGE);
  bool hand2 = (dist2 >= MIN_RANGE && dist2 <= MAX_RANGE);
  
  return hand1 || hand2;
}

// =============================================================================
// DJ SCRATCH CONTROL FUNCTIONS
// =============================================================================

void handleDJScratchControls() {
  float dist1 = readSensor(TRIG1, ECHO1);
  float dist2 = readSensor(TRIG2, ECHO2);
  
  // Left hand controls playback
  if (dist1 >= MIN_RANGE && dist1 <= MAX_RANGE) {
    playState = 1;
    // Map distance to playback speed (closer = faster)
    playbackSpeed = map(dist1, MIN_RANGE, MAX_RANGE, 1, 10);
  } else {
    playState = 0;
  }
  
  // Right hand controls scratch
  if (dist2 >= MIN_RANGE && dist2 <= MAX_RANGE) {
    isScratchMode = true;
    // Map distance to scratch speed
    scratchSpeed = map(dist2, MIN_RANGE, MAX_RANGE, -5, 5);
  } else {
    isScratchMode = false;
  }
}

// =============================================================================
// MOZZI AUDIO FUNCTIONS
// =============================================================================

void updateControl() {
  if (currentMode == MODE_ALIEN) {
    float dist1 = readSensor(TRIG1, ECHO1);
    float dist2 = readSensor(TRIG2, ECHO2);
    
    if (dist1 >= MIN_RANGE && dist1 <= MAX_RANGE) {
      effectVolume = map(dist1, MIN_RANGE, MAX_RANGE, 0, 255) / 255.0;
    } else {
      effectVolume = 0;
    }
    
    if (dist2 >= MIN_RANGE && dist2 <= MAX_RANGE) {
      effectPitch = map(dist2, MIN_RANGE, MAX_RANGE, 200, 1000);
      alienOsc.setFreq(effectPitch + vibrato.next() * 20);
    }
  }
  else if (currentMode == MODE_ROBOTS) {
    float dist1 = readSensor(TRIG1, ECHO1);
    if (dist1 >= MIN_RANGE && dist1 <= MAX_RANGE) {
      effectVolume = map(dist1, MIN_RANGE, MAX_RANGE, 0, 255) / 255.0;
      effectPitch = map(dist1, MIN_RANGE, MAX_RANGE, 100, 500);
      robotOsc.setFreq(effectPitch);
    } else {
      effectVolume = 0;
    }
  }
  else if (currentMode == MODE_THEREMIN) {
    float dist1 = readSensor(TRIG1, ECHO1);
    float dist2 = readSensor(TRIG2, ECHO2);
    
    if (dist1 >= MIN_RANGE && dist1 <= MAX_RANGE) {
      effectVolume = map(dist1, MIN_RANGE, MAX_RANGE, 0, 255) / 255.0;
    } else {
      effectVolume = 0;
    }
    
    if (dist2 >= MIN_RANGE && dist2 <= MAX_RANGE) {
      effectPitch = map(dist2, MIN_RANGE, MAX_RANGE, 220, 880);
      thereminOsc.setFreq(effectPitch);
    }
  }
}

int updateAudio() {
  int sample = 0;
  
  if (currentMode == MODE_ALIEN) {
    sample = (int)(alienOsc.next() * effectVolume);
  }
  else if (currentMode == MODE_ROBOTS) {
    sample = (int)(robotOsc.next() * effectVolume);
  }
  else if (currentMode == MODE_THEREMIN) {
    sample = (int)(thereminOsc.next() * effectVolume);
  }
  
  return sample;
}

// =============================================================================
// SETUP AND MAIN LOOP
// =============================================================================

void setup() {
  Serial.begin(9600);
  Serial.println("Solar Shrine Modular Audio System Starting...");
  
  // Initialize sensor pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  
  // Initialize LED system
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(50);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Start with DJ Scratch mode
  currentMode = MODE_DJ_SCRATCH;
  setupDJScratch();
  lastModeChange = millis();
  
  Serial.println("System initialized - Starting in DJ Scratch mode");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check for mode rotation every MODE_DURATION
  if (currentTime - lastModeChange >= MODE_DURATION) {
    // Disable current mode
    switch (currentMode) {
      case MODE_DJ_SCRATCH:
        disableDJScratch();
        break;
      case MODE_ALIEN:
        disableMozziAlien();
        break;
      case MODE_ROBOTS:
        disableMozziRobots();
        break;
      case MODE_THEREMIN:
        disableMozziTheremin();
        break;
    }
    
    // Switch to next mode
    switch (currentMode) {
      case MODE_DJ_SCRATCH:
        currentMode = MODE_ALIEN;
        setupMozziAlien();
        Serial.println("Switched to ALIEN mode");
        break;
      case MODE_ALIEN:
        currentMode = MODE_ROBOTS;
        setupMozziRobots();
        Serial.println("Switched to ROBOTS mode");
        break;
      case MODE_ROBOTS:
        currentMode = MODE_THEREMIN;
        setupMozziTheremin();
        Serial.println("Switched to THEREMIN mode");
        break;
      case MODE_THEREMIN:
        currentMode = MODE_DJ_SCRATCH;
        setupDJScratch();
        Serial.println("Switched to DJ_SCRATCH mode");
        break;
    }
    
    lastModeChange = currentTime;
  }
  
  // Handle audio based on current mode
  if (currentMode == MODE_DJ_SCRATCH) {
    // DJ Scratch uses Timer1 ISR - just handle controls
    handleDJScratchControls();
    
    // Update LEDs for DJ mode
    if (playState == 1) {
      fill_solid(leds, NUM_LEDS, CRGB::Red);
    } else {
      fill_solid(leds, NUM_LEDS, CRGB::Blue);
    }
    FastLED.show();
    
  } else {
    // CRITICAL: audioHook() must be called EVERY loop iteration for Mozzi
    // This is the key difference from our previous implementation
    audioHook();
    
    // Update LEDs based on mode
    switch (currentMode) {
      case MODE_ALIEN:
        if (effectVolume > 0) {
          fill_solid(leds, NUM_LEDS, CRGB::Green);
        } else {
          fill_solid(leds, NUM_LEDS, CRGB::DarkGreen);
        }
        break;
      case MODE_ROBOTS:
        if (effectVolume > 0) {
          fill_solid(leds, NUM_LEDS, CRGB::Purple);
        } else {
          fill_solid(leds, NUM_LEDS, CRGB::DarkMagenta);
        }
        break;
      case MODE_THEREMIN:
        if (effectVolume > 0) {
          fill_solid(leds, NUM_LEDS, CRGB::Yellow);
        } else {
          fill_solid(leds, NUM_LEDS, CRGB::Orange);
        }
        break;
    }
    FastLED.show();
  }
  
  // Small delay to prevent overwhelming the system
  delay(10);
}