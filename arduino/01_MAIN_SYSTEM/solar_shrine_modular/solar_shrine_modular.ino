/*
 * Solar Shrine - Modular Integration System
 * Combines dual-mode lighting, sensor detection, and DJ scratch audio
 * Designed as the main hub for modular functionality expansion
 * 
 * Hardware (Arduino Mega 2560):
 * - 2x HC-SR04 ultrasonic sensors (pins 5,6,10,11)
 * - WS2812B/WS2815 LED strip (pin 3)
 * - Audio: Pin 12 → 1K resistor → Amplifier right channel (left channel + ground → ground)
 * - WWZMDiB XH-M543 amplifier + Dayton Audio DAEX32QMB-4 exciter
 * 
 * Features:
 * - Attract/Interactive LED modes
 * - Hand detection with averaging
 * - DJ Scratch audio control (Left hand = Play/Pause, Right hand = Scratch)
 * - TouchDesigner JSON integration
 * - Modular architecture for future expansion
 * 
 * Libraries Required:
 * - FastLED
 * - ArduinoJson
 * - NewPing
 */

#include <ArduinoJson.h>
#include <FastLED.h>
#include <NewPing.h>
#include <avr/pgmspace.h>

// Audio data - include the DJ scratch audio
#include "audio_data.h"

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

// Sensor pins - Arduino Mega 2560
const int trigPin1 = 10;  // Left sensor trigger
const int echoPin1 = 11;  // Left sensor echo
const int trigPin2 = 5;   // Right sensor trigger
const int echoPin2 = 6;   // Right sensor echo

// NewPing sensor objects
NewPing sonar1(trigPin1, echoPin1, 200); // Left sensor, max 200cm
NewPing sonar2(trigPin2, echoPin2, 200); // Right sensor, max 200cm

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 120
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// Audio pin - Arduino Mega 2560 Timer1 OC1B
const int AUDIO_PIN = 12;

// =============================================================================
// SYSTEM CONSTANTS
// =============================================================================

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE
};

// Attract mode timing
const float ATTRACT_PERIOD = 5000.0;  // 5 seconds in milliseconds
const unsigned long INTERACTIVE_TIMEOUT = 10000;  // 10 seconds

// Hand detection averaging
const int SAMPLES = 5;

// =============================================================================
// SYSTEM STATE VARIABLES
// =============================================================================

// LED system state
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
float attractPhaseOffset = 0.0;
bool usePhaseOffset = false;
CRGB lastLeftColor = CRGB::Yellow;
CRGB lastRightColor = CRGB::Yellow;

// Sensor averaging
float distance1Samples[SAMPLES];
float distance2Samples[SAMPLES];
int sampleIndex = 0;
bool samplesInitialized = false;

// Audio system state (DJ Scratch)
volatile int32_t audioSampleIndex = 0;
volatile uint8_t audioSampleCounter = 0;
volatile uint8_t playState = 0;
volatile uint8_t playbackSpeed = 5;
volatile int8_t scratchSpeed = 1;
volatile bool isScratchMode = false;

// Audio control state
bool audioActive = false;
unsigned long lastAudioStateChange = 0;

// =============================================================================
// AUDIO SYSTEM (DJ SCRATCH)
// =============================================================================

void setupAudioSystem() {
  pinMode(AUDIO_PIN, OUTPUT);  // Pin 12 (Timer1 OC1B)
  
  // Timer1 PWM - configured for OC1B (pin 12)
  TCCR1A = _BV(COM1B1) | _BV(WGM11);  // Clear OC1B on compare match, Fast PWM
  TCCR1B = _BV(WGM13) | _BV(CS10);    // Fast PWM, no prescaler
  ICR1 = 399;                         // 20kHz frequency
  OCR1B = ICR1 / 2;                   // 50% duty cycle (silence)
  TIMSK1 = _BV(OCIE1B);               // Enable Timer1 Compare B interrupt
  
  Serial.println(F("Audio System Ready - DJ Scratch Mode"));
}

// Timer1 Compare B interrupt - Audio generation
ISR(TIMER1_COMPB_vect) {
  audioSampleCounter++;
  
  uint8_t currentSpeed = isScratchMode ? 2 : playbackSpeed;
  
  if (audioSampleCounter >= currentSpeed) {
    audioSampleCounter = 0;
    
    if (playState == 1) {
      if (audioSampleIndex < 0) audioSampleIndex = 0;
      if (audioSampleIndex >= AUDIO_SAMPLE_COUNT) audioSampleIndex = 0;
      
      uint8_t sample = pgm_read_byte(&audioData[audioSampleIndex]);
      int16_t amp = ((int16_t)sample - 128) * 4;
      amp = constrain(amp, -128, 127);
      
      OCR1B = ((uint32_t)(amp + 128) * ICR1) / 255;
      
      if (isScratchMode) {
        audioSampleIndex += scratchSpeed;
        if (audioSampleIndex < 0) audioSampleIndex = AUDIO_SAMPLE_COUNT - 1;
        if (audioSampleIndex >= AUDIO_SAMPLE_COUNT) audioSampleIndex = 0;
      } else {
        audioSampleIndex++;
        if (audioSampleIndex >= AUDIO_SAMPLE_COUNT) audioSampleIndex = 0;
      }
      
    } else {
      OCR1B = ICR1 / 2;  // Silence
    }
  }
}

// =============================================================================
// SENSOR SYSTEM
// =============================================================================

float readDistanceWithReset(int trigPin, int echoPin) {
  // Check if echo pin is stuck HIGH from previous failed reading
  if (digitalRead(echoPin) == HIGH) {
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(10);
    pinMode(echoPin, INPUT);
    delayMicroseconds(10);
  }
  
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Wait for echo
  unsigned long duration = pulseIn(echoPin, HIGH, 50000UL); // 50ms timeout
  
  if (duration == 0) {
    return 0; // No echo detected
  }
  
  float distance = (duration * 0.0343) / 2.0;
  
  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10; // Return invalid value for out of range
  }
  
  return distance;
}

float readDistanceNewPing(NewPing &sensor) {
  unsigned int distance = sensor.ping_cm();
  if (distance == 0) {
    return MIN_RANGE - 0.5;  // Very close reading
  }
  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10; // Out of range
  }
  return (float)distance;
}

void updateDistanceSamples(float distance1, float distance2) {
  distance1Samples[sampleIndex] = distance1;
  distance2Samples[sampleIndex] = distance2;
  
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  
  if (!samplesInitialized && sampleIndex == 0) {
    samplesInitialized = true;
  }
}

float getAveragedDistance(float samples[]) {
  float sum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < SAMPLES; i++) {
    if (samples[i] >= (MIN_RANGE - 0.5) && samples[i] <= MAX_RANGE) {
      sum += samples[i];
      validSamples++;
    }
  }
  
  if (validSamples < 3) {
    return MAX_RANGE + 10; // Invalid
  }
  
  return sum / validSamples;
}

// =============================================================================
// LED SYSTEM
// =============================================================================

CRGB getInteractiveColor(float distance) {
  if (distance < MIN_RANGE || distance > MAX_RANGE) {
    return CRGB::Black;
  }
  
  // Map distance to color: far = red, close = yellow
  float ratio = (distance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
  ratio = constrain(ratio, 0.0, 1.0);
  
  int red = 255;
  int green = (int)(255 * (1.0 - ratio));
  int blue = 0;
  
  return CRGB(red, green, blue);
}

CRGB getAttractColor(unsigned long currentTime) {
  float phase;
  
  if (usePhaseOffset) {
    phase = attractPhaseOffset + (2.0 * PI * currentTime / ATTRACT_PERIOD);
    usePhaseOffset = false;
  } else {
    phase = 2.0 * PI * currentTime / ATTRACT_PERIOD;
  }
  
  // Sinusoidal fade from yellow to red
  float sineValue = (sin(phase) + 1.0) / 2.0;
  
  int red = 255;
  int green = (int)(255 * sineValue);
  int blue = 0;
  
  return CRGB(red, green, blue);
}

void calculatePhaseOffset(CRGB currentColor) {
  float greenRatio = currentColor.green / 255.0;
  float sineValue = 2.0 * greenRatio - 1.0;
  sineValue = constrain(sineValue, -1.0, 1.0);
  
  attractPhaseOffset = asin(sineValue);
  usePhaseOffset = true;
}

void updateLEDs(float avgDistance1, float avgDistance2, bool handsDetected, unsigned long currentTime) {
  if (currentMode == ATTRACT_MODE) {
    CRGB attractColor = getAttractColor(currentTime);
    fill_solid(leds, NUM_LEDS, attractColor);
    
  } else {  // INTERACTIVE_MODE
    // Left sensor (first half of strip)
    CRGB leftColor = getInteractiveColor(avgDistance1);
    if (leftColor == CRGB::Black) leftColor = lastLeftColor;
    else lastLeftColor = leftColor;
    
    // Right sensor (second half of strip)
    CRGB rightColor = getInteractiveColor(avgDistance2);
    if (rightColor == CRGB::Black) rightColor = lastRightColor;
    else lastRightColor = rightColor;
    
    // Apply colors to respective halves
    int halfPoint = NUM_LEDS / 2;
    for (int i = 0; i < halfPoint; i++) {
      leds[i] = leftColor;
    }
    for (int i = halfPoint; i < NUM_LEDS; i++) {
      leds[i] = rightColor;
    }
  }
  
  FastLED.show();
}

// =============================================================================
// AUDIO CONTROL SYSTEM
// =============================================================================

void updateAudioControl(float avgDistance1, float avgDistance2, bool leftHand, bool rightHand) {
  // LEFT HAND - Play/Stop Control
  static bool prevLeftHand = false;
  
  if (leftHand && !prevLeftHand) {
    if (playState == 0) {
      playState = 1;
      audioSampleIndex = 0;
      audioActive = true;
      lastAudioStateChange = millis();
      Serial.println(F("AUDIO: PLAY"));
    } else {
      playState = 0;
      audioActive = false;
      lastAudioStateChange = millis();
      Serial.println(F("AUDIO: STOP"));
    }
  }
  
  // RIGHT HAND - Scratch Control
  static bool prevRightHand = false;
  static unsigned long lastTransition = 0;
  static uint8_t transitionCount = 0;
  static int8_t scratchDirection = 1;
  static unsigned long lastRightHandDetected = 0;
  
  if (rightHand) {
    lastRightHandDetected = millis();
  }
  
  if (rightHand != prevRightHand) {
    lastTransition = millis();
    transitionCount++;
    scratchDirection = -scratchDirection;
  }
  
  if (millis() - lastTransition > 500) {
    transitionCount = 0;
  }
  
  // Stop scratching if no right hand for 200ms
  if (!rightHand && (millis() - lastRightHandDetected) > 200) {
    if (isScratchMode) {
      isScratchMode = false;
      scratchSpeed = 1;
      playbackSpeed = 5;
      Serial.println(F("AUDIO: NORMAL"));
    }
  }
  // Scratch mode - rapid transitions
  else if (rightHand && transitionCount >= 2 && (millis() - lastTransition) < 300) {
    isScratchMode = true;
    
    if (transitionCount >= 4) {
      scratchSpeed = scratchDirection * 6;
      Serial.println(F("AUDIO: SCRATCH++"));
    } else {
      scratchSpeed = scratchDirection * 3;
      Serial.println(F("AUDIO: SCRATCH"));
    }
  }
  // Distance speed control
  else if (rightHand && playState == 1) {
    isScratchMode = false;
    scratchSpeed = 1;
    
    float ratio = (avgDistance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    playbackSpeed = (uint8_t)(3 + (12 * ratio));  // 3 (fast) to 15 (slow)
  }
  
  prevLeftHand = leftHand;
  prevRightHand = rightHand;
}

// =============================================================================
// JSON OUTPUT SYSTEM
// =============================================================================

void sendJSONUpdate(float avgDistance1, float avgDistance2, bool handsDetected, bool inRange1, bool inRange2) {
  StaticJsonDocument<500> doc;
  
  // Sensor data
  doc["left"] = int(avgDistance1);
  doc["right"] = int(avgDistance2);
  doc["hands_detected"] = handsDetected;
  doc["left_in_range"] = inRange1;
  doc["right_in_range"] = inRange2;
  
  // LED system data
  doc["mode"] = (currentMode == ATTRACT_MODE) ? "attract" : "interactive";
  
  // Audio system data
  doc["audio_active"] = audioActive;
  doc["audio_playing"] = (playState == 1);
  doc["scratch_mode"] = isScratchMode;
  doc["playback_speed"] = playbackSpeed;
  
  // Color correlation values for TouchDesigner
  if (currentMode == INTERACTIVE_MODE) {
    float leftOrange = inRange1 ? (1.0 - (avgDistance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                 (lastLeftColor.green / 255.0);
    float rightOrange = inRange2 ? (1.0 - (avgDistance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                  (lastRightColor.green / 255.0);
    
    doc["left_orange_value"] = leftOrange;
    doc["right_orange_value"] = rightOrange;
  }
  
  // System info
  doc["system"] = "solar_shrine_modular";
  doc["timestamp"] = millis();
  
  serializeJson(doc, Serial);
  Serial.println();
}

// =============================================================================
// MAIN SYSTEM SETUP AND LOOP
// =============================================================================

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  // Initialize sensor pins
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  // Initialize LED system
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Initialize sample arrays
  for (int i = 0; i < SAMPLES; i++) {
    distance1Samples[i] = MAX_RANGE + 10;
    distance2Samples[i] = MAX_RANGE + 10;
  }
  
  // Initialize audio system
  setupAudioSystem();
  
  // Startup sequence - LED flash
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(500);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  Serial.println(F("Solar Shrine Modular System Ready"));
  Serial.println(F("Features: Dual-mode LEDs + DJ Scratch Audio"));
  Serial.println(F("Left Hand: Play/Pause | Right Hand: Scratch Control"));
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensors
  float distance1 = readDistanceWithReset(trigPin1, echoPin1);  // Left sensor with reset fix
  float distance2 = readDistanceNewPing(sonar2);                // Right sensor
  
  // Update sample arrays
  updateDistanceSamples(distance1, distance2);
  
  // Get averaged distances
  float avgDistance1, avgDistance2;
  if (samplesInitialized) {
    avgDistance1 = getAveragedDistance(distance1Samples);
    avgDistance2 = getAveragedDistance(distance2Samples);
  } else {
    avgDistance1 = distance1;
    avgDistance2 = distance2;
  }
  
  // Check if hands are detected
  bool inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  bool inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;
  
  // Mode state machine
  static int falseDetectionCount = 0;
  if (currentMode == ATTRACT_MODE && handsDetected) {
    falseDetectionCount++;
    if (falseDetectionCount < 3) {
      handsDetected = false;
      inRange1 = false;
      inRange2 = false;
    }
  } else {
    falseDetectionCount = 0;
  }
  
  if (handsDetected) {
    if (currentMode == ATTRACT_MODE) {
      currentMode = INTERACTIVE_MODE;
    }
    lastHandDetectedTime = currentTime;
  } else {
    if (currentMode == INTERACTIVE_MODE && 
        (currentTime - lastHandDetectedTime) >= INTERACTIVE_TIMEOUT) {
      
      // Calculate smooth transition phase
      CRGB transitionColor = (lastLeftColor.r + lastLeftColor.g > lastRightColor.r + lastRightColor.g) ? 
                            lastLeftColor : lastRightColor;
      calculatePhaseOffset(transitionColor);
      
      currentMode = ATTRACT_MODE;
    }
  }
  
  // Update systems
  updateLEDs(avgDistance1, avgDistance2, handsDetected, currentTime);
  updateAudioControl(avgDistance1, avgDistance2, inRange1, inRange2);
  
  // Send JSON updates (reduced frequency)
  static unsigned long lastJSONSent = 0;
  if (currentTime - lastJSONSent >= 100) {  // 10Hz
    sendJSONUpdate(avgDistance1, avgDistance2, handsDetected, inRange1, inRange2);
    lastJSONSent = currentTime;
  }
  
  delay(20);  // 50Hz main loop
} 