/*
 * Dual Ultrasonic Sensor Reader with Advanced FastLED Effects
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
#define NUM_LEDS 60        // Adjust this to match your LED strip length
#define LED_TYPE WS2812B   // Change if using different LED type
#define COLOR_ORDER GRB    // Most WS2812B strips use GRB order

CRGB leds[NUM_LEDS];

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

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
  FastLED.setBrightness(150);  // Set brightness (0-255)
  
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

void updateLEDs(float distance1, float distance2, bool hands_detected) {
  // Clear all LEDs first
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  
  if (hands_detected) {
    // Both hands detected - special rainbow effect
    static uint8_t hue = 0;
    fill_rainbow(leds, NUM_LEDS, hue, 7);
    hue += 3;  // Animate the rainbow
    
  } else {
    // Individual sensor responses
    
    // Left sensor (first half of strip)
    if (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE) {
      int intensity = map(distance1, MIN_RANGE, MAX_RANGE, 255, 50);
      CRGB leftColor = CRGB(intensity, 0, intensity/2);  // Purple-ish
      
      int leftLEDs = NUM_LEDS / 2;
      for (int i = 0; i < leftLEDs; i++) {
        leds[i] = leftColor;
      }
    }
    
    // Right sensor (second half of strip)
    if (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE) {
      int intensity = map(distance2, MIN_RANGE, MAX_RANGE, 255, 50);
      CRGB rightColor = CRGB(0, intensity, intensity/2);  // Cyan-ish
      
      int rightStart = NUM_LEDS / 2;
      for (int i = rightStart; i < NUM_LEDS; i++) {
        leds[i] = rightColor;
      }
    }
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
  updateLEDs(distance1, distance2, hands_detected);
  
  // Create JSON document
  StaticJsonDocument<200> doc;
  
  // Add values to JSON
  doc["left"] = int(distance1);
  doc["right"] = int(distance2);
  doc["hands_detected"] = hands_detected;
  
  // Serialize JSON to string and send
  serializeJson(doc, Serial);
  Serial.println();  // Add newline for readability
  
  delay(50);  // Slightly longer delay for smooth LED animations
} 