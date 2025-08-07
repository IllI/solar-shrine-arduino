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

#include <NewTone.h>
#define FASTLED_FORCE_BITBANG
#include <FastLED.h>
#include <ArduinoJson.h>
#include "dj_scratch.h"
#include "theremin.h"
#include "alien.h"
#include "robots.h"
// NewPing removed to avoid timer conflicts with NewTone

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

// Pin assignments for HC-SR04 sensors
const int trigPin1 = 10; // Left sensor
const int echoPin1 = 11;
const int trigPin2 = 5;  // Right sensor
const int echoPin2 = 6; // Corrected from 12 to match hardware wiring and avoid audio conflict
// Distance-based detection constants (in cm)
const float MIN_RANGE = 5.0;                  // MinimTheremin effect initializedum detection range in cm
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

// Effect types
enum EffectType {
  DJ_SCRATCH,
  ALIEN,
  ROBOTS,
  MINITHEREMIN
};

EffectType currentEffect = DJ_SCRATCH; // Start with DJ Scratch

const unsigned long INTERACTIVE_TIMEOUT = 10000; // 10s no hands -> back to attract
const unsigned long EFFECT_SWITCH_TIMEOUT = 5000; // 5s no hands -> rotate effect


// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE
};

// State variables
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
unsigned long lastEffectRotationTime = 0;

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

  // FastLED setup
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(150);
  
  // Initialize all LEDs to off
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  Serial.begin(9600);
  // Initialize sample arrays
  for (int i = 0; i < SAMPLES; i++) {
    distance1Samples[i] = MAX_RANGE + 10;
    distance2Samples[i] = MAX_RANGE + 10;
  }

  // Initialize timers for idle state and start first effect
  unsigned long now = millis();
  lastHandDetectedTime = now;
  lastEffectRotationTime = now;
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

// =============================================================================
// EFFECT MANAGEMENT
// =============================================================================

void effect_disable(EffectType effect) {
  switch (effect) {
    case DJ_SCRATCH: dj_scratch_disable(); break;
    case ALIEN: alien_disable(); break;
    case ROBOTS: robots_disable(); break;
    case MINITHEREMIN: theremin_disable(); break;
  }
}

void effect_setup(EffectType effect) {
  switch (effect) {
    case DJ_SCRATCH:
      dj_scratch_setup();
      break;
    case ALIEN: alien_setup(); break;
    case ROBOTS: robots_setup(); break;
    case MINITHEREMIN: theremin_setup(); break;
  }
}

void effect_update(EffectType effect, float dLeft, float dRight) {
  switch (effect) {
    case DJ_SCRATCH: dj_scratch_update(dLeft, dRight); break;
    case ALIEN: alien_update(dLeft, dRight); break;
    case ROBOTS: robots_update(dLeft, dRight); break;
    case MINITHEREMIN: theremin_update(dLeft, dRight); break;
  }
}

EffectType nextEffect(EffectType effect) {
  switch (effect) {
    case DJ_SCRATCH: return ALIEN;
    case ALIEN: return ROBOTS;
    case ROBOTS: return MINITHEREMIN;
    case MINITHEREMIN: return DJ_SCRATCH;
  }
  return DJ_SCRATCH;
}

const char* effect_name(EffectType effect) {
  switch (effect) {
    case DJ_SCRATCH: return "dj_scratch";
    case ALIEN: return "alien";
    case ROBOTS: return "robots";
    case MINITHEREMIN: return "mini_theremin";
  }
  return "dj_scratch";
}

// Hard kill audio output on pin 12 regardless of current effect
void audio_all_off() {
  // Stop NewTone
  noNewTone(12);
  // Disable DJ scratch Timer1 if active
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 &= ~_BV(OCIE1B);
  // Tri-state the pin
  pinMode(12, INPUT);
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

void updateDistanceSamples(float distance1, float distance2) {
  distance1Samples[sampleIndex] = distance1;
  distance2Samples[sampleIndex] = distance2;
  
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  if (sampleIndex == 0) {
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




void loop() {
  unsigned long currentTime = millis();
  unsigned long sinceLastHand = currentTime - lastHandDetectedTime;
  
  // Read sensors
  float distance1 = readDistanceWithReset(trigPin1, echoPin1);
  float distance2 = readDistanceWithReset(trigPin2, echoPin2);
  
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

  // Detect rising edge of hand presence to re-initialize the active effect
  static bool prevHandsDetected = false;

  // State machine for mode and effect switching
  if (handsDetected) {
    // Rising edge: set up whichever effect is currently selected by rotation
    if (!prevHandsDetected) {
      effect_setup(currentEffect);
      if (currentEffect == DJ_SCRATCH) {
        dj_scratch_start();
      }
    }
    lastHandDetectedTime = currentTime;
    if (currentMode == ATTRACT_MODE) {
      currentMode = INTERACTIVE_MODE;
      // On entering interactive, leave current effect as-is
    }
  } else { // No hands detected
    if (currentMode == INTERACTIVE_MODE && sinceLastHand > INTERACTIVE_TIMEOUT) {
      currentMode = ATTRACT_MODE; // Back to attract after 10s
      lastEffectRotationTime = currentTime; // reset rotation cadence
    } else if (sinceLastHand > EFFECT_SWITCH_TIMEOUT) {
      // Rotate effects every 5s of no hands
      if (currentTime - lastEffectRotationTime >= EFFECT_SWITCH_TIMEOUT) {
        // Keep hardware silent while rotating through effects with no hands
        effect_disable(currentEffect);
        currentEffect = nextEffect(currentEffect);
        // Defer setup until hands are detected to avoid any audio during idle
        lastEffectRotationTime = currentTime;
        Serial.print(F("Rotated effect to: "));
        Serial.println(effect_name(currentEffect));
      }
    }
  }

  // Update the current effect only if hands detected; otherwise ensure silence
  if (handsDetected) {
    effect_update(currentEffect, distance1, distance2);
  } else {
    audio_all_off();
    // On falling edge, ensure current effect is fully disabled as well
    if (prevHandsDetected && !handsDetected) {
      effect_disable(currentEffect);
    }
  }

  // Update hand edge tracker
  prevHandsDetected = handsDetected;

  // Update LEDs
  updateLEDs(avgDistance1, avgDistance2, inRange1, inRange2, currentTime);

  // Send JSON data
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
    const char* effectName = "dj_scratch";
    if (currentEffect == ALIEN) effectName = "alien";
    else if (currentEffect == ROBOTS) effectName = "robots";
    else if (currentEffect == MINITHEREMIN) effectName = "mini_theremin";
    doc["current_effect"] = effectName;
    doc["raw_distance1"] = distance1;
    doc["raw_distance2"] = distance2;
    
    serializeJson(doc, Serial);
    Serial.println();
    
    lastReportedMode = currentMode;
    lastHandsDetected = handsDetected;
  }

  delay(20);
}