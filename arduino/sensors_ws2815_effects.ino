/*
 * Advanced WS2815 Lighting Effects with Hand Sensor Integration
 * Multiple lighting modes and effects based on sensor data
 */

#include <ArduinoJson.h>
#include <FastLED.h>

// Sensor pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

// LED strip configuration
#define LED_PIN 3
#define NUM_LEDS 20
#define LED_TYPE WS2815
#define COLOR_ORDER RGB

CRGB leds[NUM_LEDS];

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

// Effect variables
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  // Sensor setup
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
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2;
}

void updateAdvancedLEDs(float distance1, float distance2, bool hands_detected) {
  // Clear all LEDs first
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  
  if (hands_detected) {
    // Calculate the difference between left and right sensor distances
    float difference = distance1 - distance2;
    float maxDifference = MAX_RANGE - MIN_RANGE;
    
    // Mode 1: Basic Red-Blue transition based on which hand is closer
    int colorValue = map(constrain((difference + maxDifference) * 100, 0, 2 * maxDifference * 100), 
                        0, 2 * maxDifference * 100, 0, 255);
    
    // Mode 2: Split strip effects - left sensor controls left half, right sensor controls right half
    int halfPoint = NUM_LEDS / 2;
    
    // Left sensor controls left half
    if (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE) {
      int leftIntensity = map(distance1, MIN_RANGE, MAX_RANGE, 255, 50);
      CRGB leftColor = CRGB(leftIntensity, 0, leftIntensity/2);  // Purple-ish
      
      for (int i = 0; i < halfPoint; i++) {
        leds[i] = leftColor;
      }
    }
    
    // Right sensor controls right half
    if (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE) {
      int rightIntensity = map(distance2, MIN_RANGE, MAX_RANGE, 255, 50);
      CRGB rightColor = CRGB(0, rightIntensity, rightIntensity/2);  // Cyan-ish
      
      for (int i = halfPoint; i < NUM_LEDS; i++) {
        leds[i] = rightColor;
      }
    }
    
    // Mode 3: When both hands are very close (< 5cm), special rainbow effect
    if (distance1 < 5.0 && distance2 < 5.0) {
      fill_rainbow(leds, NUM_LEDS, gHue, 7);
      gHue += 3;  // Animate the rainbow
    }
    
    // Mode 4: Breathing effect based on average distance
    float avgDistance = (distance1 + distance2) / 2.0;
    int breathIntensity = map(avgDistance, MIN_RANGE, MAX_RANGE, 255, 100);
    
    // Apply breathing to existing colors
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].fadeToBlackBy(255 - breathIntensity);
    }
    
  } else {
    // Idle mode - subtle breathing white
    static uint8_t breathCounter = 0;
    breathCounter++;
    
    int brightness = beatsin8(30, 10, 80);  // Slow breathing
    CRGB idleColor = CRGB(brightness/3, brightness/3, brightness/3);
    
    // Only light up a few LEDs in idle mode
    for (int i = 0; i < NUM_LEDS; i += 4) {
      leds[i] = idleColor;
    }
  }
  
  FastLED.show();
}

void rainbowEffect() {
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
  FastLED.show();
  gHue++;
}

void sparkleEffect(CRGB color) {
  // Random sparkles
  if (random8() < 80) {
    leds[random16(NUM_LEDS)] += color;
  }
  
  // Fade all LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].fadeToBlackBy(10);
  }
  
  FastLED.show();
}

void waveEffect(float distance1, float distance2) {
  // Create wave patterns based on sensor distances
  static uint8_t wavePos = 0;
  wavePos++;
  
  for (int i = 0; i < NUM_LEDS; i++) {
    int wave1 = sin8(wavePos + (i * distance1));
    int wave2 = sin8(wavePos + (i * distance2) + 128);
    
    leds[i] = CRGB(wave1, 0, wave2);
  }
  
  FastLED.show();
}

void loop() {
  // Read sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Check if hands are detected (within range)
  bool inRange1 = (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE);
  bool inRange2 = (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE);
  bool hands_detected = inRange1 && inRange2;
  
  // Update LED effects
  updateAdvancedLEDs(distance1, distance2, hands_detected);
  
  // Alternative effects (comment/uncomment to try different modes):
  // rainbowEffect();
  // sparkleEffect(CRGB::White);
  // waveEffect(distance1, distance2);
  
  // Create JSON document for TouchDesigner
  StaticJsonDocument<200> doc;
  doc["left"] = int(distance1);
  doc["right"] = int(distance2);
  doc["hands_detected"] = hands_detected;
  
  // Serialize JSON to string and send
  serializeJson(doc, Serial);
  Serial.println();
  
  delay(50);  // Slightly longer delay for smooth animations
} 