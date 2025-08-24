/****************************************************************************
 Solar Shrine Playa - Modular Audio Effect System
 Cycles between DJ Scratch, Vocoder Robot, Robots, and Theremin effects every 5 seconds
 
 Effects included:
 - DJ Scratch: PROGMEM audio playback with scratching effects
 - Vocoder Robot: Classic robot voice effect using DJ audio as input
 - Robots: Musical theremin with pentatonic scale
 - Theremin: Automatic sawtooth wave pattern
*********************************************************************************/

#include <avr/pgmspace.h>
#include "audio_data.h"

#include "DjScratch.h"
#include "ScaleEffect.h"
#include "RobotsEffect.h"
#include "ThereminEffect.h"

// =============================================================================
// MOZZI CONFIGURATION
// =============================================================================
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM

// Custom pin configuration to free up pin 11 for sensor
// Default: pin 11 (high) + pin 12 (low)
// Custom:  pin 12 (high) + pin 13 (low)
#define MOZZI_AUDIO_PIN_1 12
#define MOZZI_AUDIO_PIN_1_REGISTER OCR1B
#define MOZZI_AUDIO_PIN_1_LOW 13
#define MOZZI_AUDIO_PIN_1_LOW_REGISTER OCR1C

#include <MozziGuts.h>
#include <Oscil.h>
#include <LowPassFilter.h>
#include <RollingAverage.h>
#include <tables/sin2048_int8.h>
#include <tables/cos2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/triangle_valve_2048_int8.h>
#include <tables/saw2048_int8.h>

#define CONTROL_RATE 128

// =============================================================================
// AUDIO MODE SYSTEM
// =============================================================================
enum AudioMode {
  MODE_DJ_SCRATCH = 0,
  MODE_VOCODER_ROBOT = 1,
  MODE_MOZZI_ROBOTS = 2,
  MODE_MOZZI_THEREMIN = 3
};

AudioMode currentMode = MODE_DJ_SCRATCH;
const unsigned long MODE_DURATION = 5000; // 5 seconds per mode
unsigned long lastModeChange = 0;



// =============================================================================
// SENSOR SYSTEM
// =============================================================================
#define TRIG1 10  // Left sensor trigger
#define ECHO1 11  // Left sensor echo
#define TRIG2 5   // Right sensor trigger
#define ECHO2 6   // Right sensor echo

// Note: DJ Scratch effect variables are now in the modular audio system section above







// =============================================================================
// SENSOR READING FUNCTIONS
// =============================================================================
float readSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999; // No echo received
  
  float distance = duration * 0.034 / 2;
  return distance;
}

bool isHandPresent(float distance) {
  return (distance > 1 && distance < 20);
}

// =============================================================================
// SETUP FUNCTION
// =============================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Solar Shrine Playa - Modular Audio Effect System");
  Serial.println("Cycling: DJ Scratch -> Vocoder Robot -> Robots -> Theremin");
  
  // Initialize sensor pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  
  // Initialize all effect modules
  DjScratch::setup();
  ScaleEffect::setup();
  RobotsEffect::setup();
  ThereminEffect::setup();
  
  // Start in DJ Scratch mode
  DjScratch::enter();
  lastModeChange = millis();
}

// =============================================================================
// DJ SCRATCH TIMER1 ISR
// =============================================================================
ISR(TIMER1_COMPB_vect) {
  if (currentMode == MODE_DJ_SCRATCH) {
    DjScratch::handleISR();
  } else if (currentMode == MODE_VOCODER_ROBOT) {
    // In vocoder mode, we still need to feed DJ audio to the vocoder
    // but we don't output it directly - the vocoder processes it
    DjScratch::handleISR();
    OCR1B = 200; // Silence DJ output, vocoder handles audio output
  } else {
    OCR1B = 200; // Silence when not in DJ mode
  }
}

// =============================================================================
// MODE SWITCHING FUNCTIONS
// =============================================================================

// =============================================================================
// MODE SWITCHING FUNCTIONS
// =============================================================================



// =============================================================================
// MAIN LOOP FUNCTION
// =============================================================================
void loop() {
  // Check if it's time to switch modes
  if (millis() - lastModeChange >= MODE_DURATION) {
    switchToNextMode();
  }
  
  // CRITICAL: audioHook() must be called EVERY loop iteration for Mozzi
  // This is the key difference from our previous implementation
  if (currentMode == MODE_VOCODER_ROBOT || currentMode == MODE_MOZZI_ROBOTS || currentMode == MODE_MOZZI_THEREMIN) {
    audioHook();
  } else if (currentMode == MODE_DJ_SCRATCH) {
    // Handle DJ scratch controls (Mozzi handles its own controls in updateControl)
    static unsigned long lastSensorRead = 0;
    if (millis() - lastSensorRead >= 30) {  // Read sensors every 30ms for DJ scratch
      float d1 = readSensor(TRIG1, ECHO1);
      float d2 = readSensor(TRIG2, ECHO2);
      
      bool leftHand = isHandPresent(d1);
      bool rightHand = isHandPresent(d2);
      
      DjScratch::update(leftHand, rightHand, d1, d2);
      lastSensorRead = millis();
    }
    
    delay(5);  // Small delay for DJ scratch mode
  }
}

void switchToNextMode() {
  AudioMode previousMode = currentMode;

  // Disable current mode
  switch (previousMode) {
    case MODE_DJ_SCRATCH:
      DjScratch::exit();
      break;
    case MODE_VOCODER_ROBOT:
      ScaleEffect::exit();
      break;
    case MODE_MOZZI_ROBOTS:
      RobotsEffect::exit();
      break;
    case MODE_MOZZI_THEREMIN:
      ThereminEffect::exit();
      break;
  }

  // Stop Mozzi if we are leaving a Mozzi mode and entering a non-Mozzi mode
  bool wasMozzi = (previousMode == MODE_VOCODER_ROBOT || previousMode == MODE_MOZZI_ROBOTS || previousMode == MODE_MOZZI_THEREMIN);
  currentMode = (AudioMode)((currentMode + 1) % 4);
  bool isMozzi = (currentMode == MODE_VOCODER_ROBOT || currentMode == MODE_MOZZI_ROBOTS || currentMode == MODE_MOZZI_THEREMIN);

  if (wasMozzi && !isMozzi) {
    stopMozzi();
  }

  lastModeChange = millis();

  // Enable new mode
  switch (currentMode) {
    case MODE_DJ_SCRATCH:
      DjScratch::enter();
      break;
    case MODE_VOCODER_ROBOT:
      ScaleEffect::enter();
      break;
    case MODE_MOZZI_ROBOTS:
      RobotsEffect::enter();
      break;
    case MODE_MOZZI_THEREMIN:
      ThereminEffect::enter();
      break;
  }

  // Start Mozzi if we are entering a Mozzi mode from a non-Mozzi mode
  if (!wasMozzi && isMozzi) {
    startMozzi();
  }
}

// =============================================================================
// MOZZI UPDATE CONTROL FUNCTION (Only called when in Mozzi modes)
// =============================================================================
void updateControl() {
  // Read sensors
  float d1 = readSensor(TRIG1, ECHO1); // Left hand
  float d2 = readSensor(TRIG2, ECHO2); // Right hand
  bool leftHand = isHandPresent(d1);
  bool rightHand = isHandPresent(d2);

  // Add this for debugging
  Serial.print("Mode: ");
  Serial.print(currentMode);
  Serial.print(" | Left Sensor: ");
  Serial.print(d1);
  Serial.print(" cm, Hand: ");
  Serial.print(leftHand);
  Serial.print(" | Right Sensor: ");
  Serial.print(d2);
  Serial.print(" cm, Hand: ");
  Serial.println(rightHand);
  
  // Update current Mozzi effect
  switch (currentMode) {
    case MODE_VOCODER_ROBOT:
      ScaleEffect::update(leftHand, rightHand, d1, d2);
      break;
    case MODE_MOZZI_ROBOTS:
      RobotsEffect::update(leftHand, rightHand, d1, d2);
      break;
    case MODE_MOZZI_THEREMIN:
      ThereminEffect::update(leftHand, rightHand, d1, d2);
      break;
    default:
      // Should not reach here in Mozzi modes
      break;
  }
}

// =============================================================================
// INDIVIDUAL EFFECT UPDATE FUNCTIONS
// =============================================================================


// =============================================================================
// MOZZI AUDIO OUTPUT FUNCTION (Only for Mozzi modes)
// =============================================================================
int updateAudio() {
  return audioOutput();
}

int audioOutput() {
  switch (currentMode) {
    case MODE_VOCODER_ROBOT:
      return ScaleEffect::audio();
    case MODE_MOZZI_ROBOTS:
      return RobotsEffect::audio();
    case MODE_MOZZI_THEREMIN:
      return ThereminEffect::audio();
    default:
      return 0; // DJ Scratch uses Timer1 ISR, not Mozzi
  }
}
