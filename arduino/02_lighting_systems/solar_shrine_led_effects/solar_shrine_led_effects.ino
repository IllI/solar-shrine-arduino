/*
 * Solar Shrine LED Effects
 * Dual-mode lighting system: attract mode and interactive mode
 * Extracted from solar_shrine_theremin.ino for standalone LED testing
 * 
 * Hardware:
 * - 2x HC-SR04 ultrasonic sensors (pins 5,6,10,11)
 * - WS2812B/WS2815 LED strip (pin 3)
 * 
 * Libraries Required:
 * - FastLED
 * - ArduinoJson  
 * - NewPing
 */

#include <ArduinoJson.h>
#include <FastLED.h>
#include <NewPing.h>

// Sensor pins
const int trigPin1 = 10;  // Left sensor
const int echoPin1 = 11;  
const int trigPin2 = 5;   // Right sensor
const int echoPin2 = 6;

// NewPing sensor objects
NewPing sonar1(trigPin1, echoPin1, 200); // Left sensor, max 200cm
NewPing sonar2(trigPin2, echoPin2, 200); // Right sensor, max 200cm

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 120        // Adjust to your strip length
#define LED_TYPE WS2812B   
#define COLOR_ORDER GRB    

CRGB leds[NUM_LEDS];

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Mode constants
enum LightMode {
  ATTRACT_MODE,
  INTERACTIVE_MODE,
  IDLE_MODE
};

// State variables
LightMode currentMode = ATTRACT_MODE;
unsigned long lastHandDetectedTime = 0;
const unsigned long INTERACTIVE_TIMEOUT = 10000;  // 10 seconds

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

// Persistent color tracking for interactive mode
CRGB currentLeftColor = CRGB::Yellow;
CRGB currentRightColor = CRGB::Yellow;
unsigned long lastLeftUpdateTime = 0;
unsigned long lastRightUpdateTime = 0;
const unsigned long COLOR_PERSIST_TIMEOUT = 1000; // 1 second to persist color after hand lost

void setup() {
  Serial.begin(9600);
  delay(2000);
  
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
  
  // Startup LED sequence
  playStartupSequence();
}

void playStartupSequence() {
  // Brief LED flash as startup indicator
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
  delay(200);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
  delay(200);
  
  // Rainbow sweep
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CHSV(i * 255 / NUM_LEDS, 255, 255);
    FastLED.show();
    delay(20);
  }
  
  // Fade to black
  for (int brightness = 255; brightness >= 0; brightness -= 5) {
    FastLED.setBrightness(brightness);
    FastLED.show();
    delay(20);
  }
  
  // Reset brightness
  FastLED.setBrightness(150);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

float readDistanceNewPing(NewPing &sensor) {
  unsigned int distance = sensor.ping_cm();
  if (distance == 0) {
    // No echo detected - hand is covering sensor (very close)
    return MIN_RANGE - 0.5;  // Closer than minimum range
  }
  if (distance > MAX_RANGE) {
    return MAX_RANGE + 10; // Return invalid value for out of range
  }
  return (float)distance;
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

void updateDistanceSamples(float distance1, float distance2) {
  distance1Samples[sampleIndex] = distance1;
  distance2Samples[sampleIndex] = distance2;
  
  sampleIndex = (sampleIndex + 1) % SAMPLES;
  
  if (!samplesInitialized && sampleIndex == 0) {
    samplesInitialized = true;
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

void updateLEDs(float avgDistance1, float avgDistance2, bool handsDetected, unsigned long currentTime) {
  if (currentMode == ATTRACT_MODE) {
    // Both sides show the same attract color
    CRGB attractColor = getAttractColor(currentTime);
    fill_solid(leds, NUM_LEDS, attractColor);
    
  } else if (currentMode == INTERACTIVE_MODE) {
    // Left sensor (first half of strip) - persistent color handling
    CRGB leftColor = getInteractiveColor(avgDistance1);
    if (leftColor != CRGB::Black) {
      // Valid color detected - update current color and timestamp
      currentLeftColor = leftColor;
      lastLeftColor = leftColor;
      lastLeftUpdateTime = currentTime;
    } else {
      // No valid color - check if we should persist the last color
      if (currentTime - lastLeftUpdateTime < COLOR_PERSIST_TIMEOUT) {
        // Within persist timeout - keep showing current color
        leftColor = currentLeftColor;
      } else {
        // Past timeout - fall back to last known color
        leftColor = lastLeftColor;
      }
    }
    
    // Right sensor (second half of strip) - persistent color handling
    CRGB rightColor = getInteractiveColor(avgDistance2);
    if (rightColor != CRGB::Black) {
      // Valid color detected - update current color and timestamp
      currentRightColor = rightColor;
      lastRightColor = rightColor;
      lastRightUpdateTime = currentTime;
    } else {
      // No valid color - check if we should persist the last color
      if (currentTime - lastRightUpdateTime < COLOR_PERSIST_TIMEOUT) {
        // Within persist timeout - keep showing current color
        rightColor = currentRightColor;
      } else {
        // Past timeout - fall back to last known color
        rightColor = lastRightColor;
      }
    }
    
    // Apply colors to respective halves
    int halfPoint = NUM_LEDS / 2;
    for (int i = 0; i < halfPoint; i++) {
      leds[i] = leftColor;
    }
    for (int i = halfPoint; i < NUM_LEDS; i++) {
      leds[i] = rightColor;
    }
    
  } else {  // IDLE_MODE
    // Continue showing the last interactive colors (freeze the effect)
    int halfPoint = NUM_LEDS / 2;
    for (int i = 0; i < halfPoint; i++) {
      leds[i] = currentLeftColor;
    }
    for (int i = halfPoint; i < NUM_LEDS; i++) {
      leds[i] = currentRightColor;
    }
  }
  
  FastLED.show();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensors using NewPing
  float distance1 = readDistanceNewPing(sonar1);
  float distance2 = readDistanceNewPing(sonar2);
  
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
  
  // Check if hands are detected with strict validation
  bool inRange1 = (avgDistance1 >= MIN_RANGE && avgDistance1 <= MAX_RANGE);
  bool inRange2 = (avgDistance2 >= MIN_RANGE && avgDistance2 <= MAX_RANGE);
  bool handsDetected = inRange1 || inRange2;
  
  // Additional validation - if we're in attract mode and getting brief detections, ignore them
  static int falseDetectionCount = 0;
  if (currentMode == ATTRACT_MODE && handsDetected) {
    falseDetectionCount++;
    if (falseDetectionCount < 3) {  // Require 3 consecutive detections to switch modes
      handsDetected = false;
      inRange1 = false;
      inRange2 = false;
    }
  } else {
    falseDetectionCount = 0;  // Reset counter when no hands or in interactive mode
  }
  
  // Mode state machine
  if (handsDetected) {
    // Switch to interactive mode from any mode when hands detected
    if (currentMode != INTERACTIVE_MODE) {
      currentMode = INTERACTIVE_MODE;
    }
    lastHandDetectedTime = currentTime;
  } else {
    // No hands detected
    if (currentMode == INTERACTIVE_MODE) {
      // Just lost hands - switch to idle mode
      currentMode = IDLE_MODE;
      lastHandDetectedTime = currentTime;  // Start idle timer
    } else if (currentMode == IDLE_MODE && 
               (currentTime - lastHandDetectedTime) >= INTERACTIVE_TIMEOUT) {
      // Idle timeout reached - return to attract mode
      
      // Calculate smooth transition phase using current displayed colors
      CRGB transitionColor = (currentLeftColor.r + currentLeftColor.g > currentRightColor.r + currentRightColor.g) ? 
                            currentLeftColor : currentRightColor;
      calculatePhaseOffset(transitionColor);
      
      currentMode = ATTRACT_MODE;
    }
  }
  
  // Update LED effects
  updateLEDs(avgDistance1, avgDistance2, handsDetected, currentTime);
  
  // Create JSON document for debugging/monitoring
  static LightMode lastReportedMode = ATTRACT_MODE;
  static bool lastHandsDetected = false;
  
  if (handsDetected || lastHandsDetected || (currentMode != lastReportedMode)) {
    StaticJsonDocument<400> doc;
    
    doc["left"] = int(avgDistance1);
    doc["right"] = int(avgDistance2);
    doc["hands_detected"] = handsDetected;
    doc["mode"] = (currentMode == ATTRACT_MODE) ? "attract" : 
                  (currentMode == INTERACTIVE_MODE) ? "interactive" : "idle";
    doc["left_in_range"] = inRange1;
    doc["right_in_range"] = inRange2;
    
    // Add color values for monitoring
    if (currentMode == INTERACTIVE_MODE) {
      float leftOrange = inRange1 ? (1.0 - (avgDistance1 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                   (lastLeftColor.green / 255.0);
      float rightOrange = inRange2 ? (1.0 - (avgDistance2 - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)) : 
                                    (lastRightColor.green / 255.0);
      
      doc["left_orange_value"] = leftOrange;
      doc["right_orange_value"] = rightOrange;
    }
    
    // Serialize JSON to string and send
    serializeJson(doc, Serial);
    Serial.println();
    
    lastReportedMode = currentMode;
    lastHandsDetected = handsDetected;
  }
  
  delay(20);  // Smooth LED updates
} 