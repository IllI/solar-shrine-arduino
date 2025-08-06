/*
 * Solar Shrine Modular Theremin
 * 
 * A modular theremin system using ultrasonic sensors for gesture control.
 * This version provides independent pitch and volume control through hand proximity.
 * 
 * Hardware:
 * - Arduino Uno/Nano
 * - 2x HC-SR04 ultrasonic sensors (pitch and volume control)
 * - Audio output via Mozzi library
 * 
 * Features:
 * - Real-time gesture-based audio synthesis
 * - Independent pitch and volume control
 * - Smooth audio transitions with rolling averages
 * - Complete silence when no hands detected
 * 
 * Detection System:
 * - Uses distance-based detection (5-50cm range) like solar_shrine_theremin.ino
 * - Converts sensor duration to actual distance in centimeters
 * - Only detects hands within MIN_RANGE to MAX_RANGE
 * - Handles sensor timeout and very close readings properly
 * 
 * Recent fixes:
 * - Replaced duration-based thresholds with distance-based detection
 * - Fixed false hand detection by using proper range checking
 * - Ensure complete silence when no hands are detected
 * - Hardware-level audio cutoff via pin control
 * 
 * Author: AI Assistant
 * Date: 2024
 */

#include "theremin.h"
#include <NewTone.h>
#include "dj_scratch.h"
#define FASTLED_FORCE_BITBANG
#include <FastLED.h>
#include <ArduinoJson.h>
// NewPing removed to avoid timer conflicts with NewTone

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

// Pin assignments for HC-SR04 sensors
const int trigPin1 = 10; // Left sensor
const int echoPin1 = 11;
const int trigPin2 = 5;  // Right sensor
const int echoPin2 = 6;

// Distance-based detection constants (in cm)
const float MIN_RANGE = 5.0;                  // Minimum detection range in cm
const float MAX_RANGE = 50.0;                 // Maximum detection range in cm

const unsigned long SENSOR_TIMEOUT = 30000;   // Sensor timeout in microseconds

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 120
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

// NewPing setup
// NewPing objects removed - using manual sensor reading

// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE
};

enum EffectType {
  THEREMIN,
  DJ_SCRATCH
};

EffectType currentEffect = THEREMIN;

// State variables
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
const unsigned long INTERACTIVE_TIMEOUT = 10000;  // 10 seconds
const unsigned long EFFECT_ROTATION_TIMEOUT = 5000; // 5 seconds
const unsigned long DJ_LOOP_DURATION = 5000; // 5 seconds


// Attract mode variables
const float ATTRACT_PERIOD = 5000.0;  // 5 seconds in milliseconds
float attractPhaseOffset = 0.0;
bool usePhaseOffset = false;

// Hand detection averaging
const int SAMPLES = 5;
float distance1Samples[SAMPLES];
float distance2Samples[SAMPLES];
int sampleIndex = 0;
bool samplesInitialized = false;

// Last known interactive colors for smooth transition
CRGB lastLeftColor = CRGB::Yellow;
CRGB lastRightColor = CRGB::Yellow;

// =============================================================================
// SETUP AND MAIN LOOP
// =============================================================================



void setup(){
  Serial.begin(9600);
  
  // Sensor pin setup (CRITICAL for manual reading)
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  theremin_setup();

  // FastLED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  
  // Initialize all LEDs to off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  
  // Initialize sample arrays
  for (int i = 0; i < SAMPLES; i++) {
    distance1Samples[i] = MAX_RANGE + 10;
    distance2Samples[i] = MAX_RANGE + 10;
  }
}

CRGB getInteractiveColor(float distance) {
  if (distance < MIN_RANGE || distance > MAX_RANGE) {
    return CRGB::Black;
  }
  
  // Map distance to color: far = red, close = yellow
  float ratio = (distance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
  ratio = constrain(ratio, 0.0, 1.0);
  
  // Red to Yellow interpolation through Orange
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
  float sineValue = (sin(phase) + 1.0) / 2.0;  // Normalize to 0-1
  
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

void updateLEDs(float avgDistance1, float avgDistance2, bool inRange1, bool inRange2, unsigned long currentTime) {
  if (currentMode == ATTRACT_MODE) {
    // Both sides show the same attract color
    CRGB attractColor = getAttractColor(currentTime);
    fill_solid(leds, NUM_LEDS, attractColor);
    
  } else {  // INTERACTIVE_MODE
    CRGB leftColor = getInteractiveColor(avgDistance1);
    CRGB rightColor = getInteractiveColor(avgDistance2);

    // Update last known colors for smooth transition back to attract mode, only if a hand is in range
    if (inRange1) lastLeftColor = leftColor;
    if (inRange2) lastRightColor = rightColor;
    
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

// Direct sensor reading with HC-SR04 reset fix (replaces NewPing)
float readDistanceWithReset(int trigPin, int echoPin) {
  // Check if echo pin is stuck HIGH from previous failed reading
  if (digitalRead(echoPin) == HIGH) {
    // Apply the fix: briefly drive echo pin LOW to reset sensor
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
    return 0; // No echo detected (same as troubleshoot script)
  }
  
  float distance = (duration * 0.0343) / 2.0;
  
  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10; // Return invalid value for out of range
  }
  
  return distance;
}

float readDistanceManual(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long startTime = micros();
  while (digitalRead(echoPin) == LOW) {
    if (micros() - startTime > 50000) {
      return MAX_RANGE + 10;
    }
  }

  startTime = micros();
  while (digitalRead(echoPin) == HIGH) {
    if (micros() - startTime > 50000) {
      return MAX_RANGE + 10;
    }
  }

  unsigned long duration = micros() - startTime;
  float distance = (duration * 0.0343) / 2.0;

  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10;
  }
  return distance;
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
    // Include "very close" readings (hands covering sensors) in the valid range
    if (samples[i] >= (MIN_RANGE - 0.5) && samples[i] <= MAX_RANGE) {
      sum += samples[i];
      validSamples++;
    }
  }
  
  // Require at least 3 out of 5 valid samples to prevent false readings
  if (validSamples < 3) {
    return MAX_RANGE + 10; // Return invalid value
  }
  
  return sum / validSamples;
}

void loop(){
  unsigned long currentTime = millis();
  
  // Read sensors using manual method (same as theremin version)
  float distance1 = readDistanceManual(trigPin1, echoPin1);
  float distance2 = readDistanceManual(trigPin2, echoPin2);

  bool handInRange1 = (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE);
  bool handInRange2 = (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE);

  bool handsDetectedRaw = handInRange1 || handInRange2;

    // State machine for mode and effect switching
  if (handsDetectedRaw) {
    lastHandDetectedTime = currentTime;
    if (currentMode == ATTRACT_MODE) {
      currentMode = INTERACTIVE_MODE;
      if (currentEffect == DJ_SCRATCH) {
        dj_scratch_disable();
        theremin_setup();
        currentEffect = THEREMIN;
      }
    }
  } else { // No hands detected
    if (currentMode == INTERACTIVE_MODE && (currentTime - lastHandDetectedTime) > INTERACTIVE_TIMEOUT) {
      currentMode = ATTRACT_MODE;
      lastHandDetectedTime = currentTime; // Reset timer for effect rotation
    } else if (currentMode == ATTRACT_MODE && currentEffect == THEREMIN && (currentTime - lastHandDetectedTime) > EFFECT_ROTATION_TIMEOUT) {
      theremin_disable();
      dj_scratch_setup();
      dj_scratch_start();
      currentEffect = DJ_SCRATCH;
    } else if (currentMode == ATTRACT_MODE && currentEffect == DJ_SCRATCH && (currentTime - lastHandDetectedTime) > (EFFECT_ROTATION_TIMEOUT + DJ_LOOP_DURATION)) {
        dj_scratch_disable();
        theremin_setup();
        currentEffect = THEREMIN;
        lastHandDetectedTime = currentTime; // Reset timer for next rotation
    }
  }

  // Update the current effect
  if (currentEffect == THEREMIN) {
    theremin_update(distance1, distance2);
  } else if (currentEffect == DJ_SCRATCH) {
    dj_scratch_update(distance1, distance2);
  }

  updateDistanceSamples(distance1, distance2);

  float avgDistance1, avgDistance2;
  if (samplesInitialized) {
    avgDistance1 = getAveragedDistance(distance1Samples);
    avgDistance2 = getAveragedDistance(distance2Samples);
  } else {
    avgDistance1 = distance1;
    avgDistance2 = distance2;
  }

  bool inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  bool inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;

  if (handsDetected) {
    if (currentMode == ATTRACT_MODE) {
      currentMode = INTERACTIVE_MODE;
    }
    // Note: lastHandDetectedTime is already updated in the effect rotation logic above
  } else {
    if (currentMode == INTERACTIVE_MODE && 
        (currentTime - lastHandDetectedTime) >= INTERACTIVE_TIMEOUT) {
      
      CRGB transitionColor = (lastLeftColor.r + lastLeftColor.g > lastRightColor.r + lastRightColor.g) ? 
                            lastLeftColor : lastRightColor;
      calculatePhaseOffset(transitionColor);
      
      currentMode = ATTRACT_MODE;
    }
  }

  // Effect-specific update already called above - no duplicate needed
  updateLEDs(avgDistance1, avgDistance2, inRange1, inRange2, currentTime);

  static LightMode lastReportedMode = ATTRACT_MODE;
  static bool lastHandsDetected = false;
  
  if (handsDetected || lastHandsDetected || (currentMode != lastReportedMode)) {
    StaticJsonDocument<400> doc;
    
    doc["left"] = int(avgDistance1);
    doc["right"] = int(avgDistance2);
    doc["hands_detected"] = handsDetected;
    doc["mode"] = (currentMode == ATTRACT_MODE) ? "attract" : "interactive";
    doc["left_in_range"] = inRange1;
    doc["right_in_range"] = inRange2;
    doc["current_effect"] = (currentEffect == THEREMIN) ? "theremin" : "dj_scratch";
    doc["raw_distance1"] = distance1;
    doc["raw_distance2"] = distance2;
    
    serializeJson(doc, Serial);
    Serial.println();
    
    lastReportedMode = currentMode;
    lastHandsDetected = handsDetected;
  }

  delay(20);
}